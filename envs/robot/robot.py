# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

import yaml
import numpy as np
import torch
import warp as wp

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.controllers.differential_ik import DifferentialIKController
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.utils import configclass

from ..utils.transforms import *
from ..utils.atom import GRASP_DIRECTION_DIC
from .robot_cfg import RobotCfg
from .curobo_planner import CuroboPlanner, CuroboPlannerCfg
from .._global import *

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from curobo.wrap.reacher.motion_gen import MotionGenResult
    from .._base_task import BaseTask

class RobotManager:
    def __init__(self, robot_cfg:RobotCfg, task:'BaseTask', planner_time_dilation_factor:float=1.0):
        self.cfg = robot_cfg
        self.task = task
        self.device = task.device
        self.sensor_type = task.cfg.tactile_sensor_type
        if self.sensor_type in ['gsmini', 'gf225', 'xensews']: # franka panda
            self.robot_type = 'franka_panda'

        self.robot = Articulation(self.cfg.robot)
        self.task.scene.articulations['robot'] = self.robot
        self.planner_time_dilation_factor = planner_time_dilation_factor

        self.gripper_max_qpos = 0.039
        self.last_arm_velocity = None
        self.last_gripper_velocity = None

        if self.robot_type == 'franka_panda':
            self.hand_name = 'panda_hand'
            self._arm_joint_names = [
                'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                'panda_joint5', 'panda_joint6', 'panda_joint7'
            ]
            self._gripper_joint_names = [
                'panda_finger_joint1', 'panda_finger_joint2'
            ]
            self.gripper_max_qpos = self.cfg.gripper_max_qpos
            self.yaml_path = str(EMBODIMENTS_ROOT / 'franka' / 'curobo.yml')
            offset = self.cfg.gripper_offset
        else:
            raise NotImplementedError(f"Robot type {self.robot_type} not implemented.")

        # offset from end-effector to gripper center frame
        self._offset = Pose(p=[0, 0, -offset], q=[1, 0, 0, 0])
        self._offset_pos = torch.tensor([0.0, 0.0, offset], device=self.device).repeat(self.task.num_envs, 1)
        self._offset_rot = torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).repeat(self.task.num_envs, 1)

    def setup(self):
        """设置机器人属性"""
        body_ids, body_names = self.robot.find_bodies(self.hand_name)
        self._body_idx = body_ids[0]
        self._body_name = body_names[0]
        self._jacobi_body_idx = self._body_idx - 1

        joint_names = self.robot.joint_names
        self.joint_name_to_id = {name: i for i, name in enumerate(joint_names)}

        self._arm_ids = torch.tensor([
            self.joint_name_to_id[n] for n in self._arm_joint_names
        ], dtype=torch.int32, device=self.device)
        self._gripper_ids = torch.tensor([
            self.joint_name_to_id[n] for n in self._gripper_joint_names
        ], dtype=torch.int32, device=self.device)
        self.origin_pose = self.get_gripper_center_pose()
        self._all_ids = torch.cat([self._arm_ids, self._gripper_ids], dim=0)

        self.root_pose = Pose.from_list(wp.to_torch(self.robot.data.root_link_pos_w)[0])
        planner_cfg = CuroboPlannerCfg(
            dt=self.task.cfg.sim.dt,
            all_joints_name=self.robot.joint_names,
            active_joints_name=self._arm_joint_names,
            robot_prime_path=self.cfg.robot.prim_path,
            yaml_path=self.yaml_path
        )
        self.planner = CuroboPlanner(
            task=self.task,
            cfg=planner_cfg,
            robot_origin_pose=self.root_pose,
        )

    def ee_to_gripper_center(self, ee_pose:Pose) -> Pose:
        """将夹爪中心位姿转换为末端执行器目标位姿"""
        return ee_pose.add_offset(self._offset.inv())

    def gripper_center_to_ee(self, gripper_center_pose:Pose) -> Pose:
        """将夹爪中心位姿转换为末端执行器目标位姿"""
        return gripper_center_pose.add_offset(self._offset)

    def get_gripper_center_pose(self, env_ids:slice=None) -> Pose:
        """获取当前夹爪中心位姿"""
        return self.ee_to_gripper_center(self.get_ee_pose())

    def get_inhand_pose(self, actor:'Actor') -> Pose:
        return actor.get_pose().rebase(self.get_gripper_center_pose())

    def get_ee_pose(self, env_ids:slice=None) -> Pose:
        """获取当前末端执行器目标位姿（target_pose）"""
        if env_ids is None:
            env_ids = [0]
        ee_pos_w = wp.to_torch(self.robot.data.body_link_pos_w)[:, self._body_idx]
        ee_quat_w = wp.to_torch(self.robot.data.body_link_quat_w)[:, self._body_idx]
        root_pos_w = wp.to_torch(self.robot.data.root_link_pos_w)
        root_quat_w = wp.to_torch(self.robot.data.root_link_quat_w)
        ee_pos_b, ee_quat_b = math_utils.subtract_frame_transforms(
            root_pos_w, root_quat_w, ee_pos_w, ee_quat_w)
        quat_xyzw = ee_quat_b[0].cpu().numpy()
        return Pose(
            ee_pos_b[0].cpu().numpy(),
            (quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]),
        )

    def get_qpos(self):
        return wp.to_torch(self.robot.data.joint_pos).clone().cpu()

    def get_gripper_qpos(self):
        return self.get_qpos()[0, self._gripper_ids[0]].clone().cpu().item()
    def get_gripper_percentage(self):
        # get_gripper_qpos() already returns a Python float, so a second .item() is invalid.
        return self.get_gripper_qpos() / self.gripper_max_qpos

    def set_arm(self, pos:torch.Tensor, vel:torch.Tensor=None, env_ids:slice=None, force:bool=True):
        '''设置目标位姿'''
        if pos.ndim == 1:
            pos = pos.unsqueeze(0)
        if vel is not None and vel.ndim == 1:
            vel = vel.unsqueeze(0)
        self.robot.set_joint_position_target(pos, joint_ids=self._arm_ids, env_ids=env_ids)
        if vel is not None:
            self.robot.set_joint_velocity_target(vel, joint_ids=self._arm_ids, env_ids=env_ids)
        if force:
            self.robot.write_joint_position_to_sim_index(
                position=pos, joint_ids=self._arm_ids, env_ids=env_ids
            )

    def set_gripper(self, pos:torch.Tensor, vel:torch.Tensor=None, env_ids:slice=None, force:bool=True):
        '''设置目标位姿'''
        def normalize_command(value, name):
            command = torch.as_tensor(value, dtype=torch.float32, device=self.device)
            num_envs = self.task.num_envs
            num_joints = len(self._gripper_ids)
            if command.ndim == 0:
                command = command.repeat(num_envs, num_joints)
            elif command.ndim == 1:
                if num_envs == 1 and command.numel() == num_joints:
                    # A two-element vector is one explicit value per gripper joint.
                    command = command.unsqueeze(0)
                elif command.numel() == num_envs:
                    # A per-environment scalar is mirrored to both finger joints.
                    command = command.unsqueeze(-1).repeat(1, num_joints)
                else:
                    raise ValueError(
                        f"{name} has shape {tuple(command.shape)}; expected "
                        f"({num_joints},) for one environment or ({num_envs},)."
                    )
            elif command.ndim == 2:
                if command.shape == (num_envs, 1):
                    command = command.repeat(1, num_joints)
                elif command.shape != (num_envs, num_joints):
                    raise ValueError(
                        f"{name} has shape {tuple(command.shape)}; expected "
                        f"({num_envs}, {num_joints})."
                    )
            else:
                raise ValueError(f"{name} must be a scalar, vector, or matrix.")
            return command

        pos = normalize_command(pos, 'pos')
        if vel is not None:
            vel = normalize_command(vel, 'vel')
        self.robot.set_joint_position_target(pos, joint_ids=self._gripper_ids, env_ids=env_ids)
        if vel is not None:
            self.robot.set_joint_velocity_target(vel, joint_ids=self._gripper_ids, env_ids=env_ids)
        if force:
            self.robot.write_joint_position_to_sim_index(
                position=pos, joint_ids=self._gripper_ids, env_ids=env_ids
            )

    def plan_arm(self, target_pose:Pose, constraint_pose=None, pre_dis=None, time_dilation_factor=None):
        result:MotionGenResult = self.planner.plan_path(
            curr_joint_pos=wp.to_torch(self.robot.data.joint_pos)[0, :self.robot.num_joints-2],
            curr_joint_vel=wp.to_torch(self.robot.data.joint_vel)[0, :self.robot.num_joints-2],
            target_ee_pose=target_pose,
            real_robot_pose=self.root_pose,
            pre_dis=pre_dis,
            constraint_pose=constraint_pose,
            time_dilation_factor=time_dilation_factor
        )

        if result.success.item():
            return {
                'status': 'Success',
                'num_steps': result.interpolated_plan.position.shape[0],
                'position': result.interpolated_plan.position.detach(),
                'velocity': result.interpolated_plan.velocity.detach()
            }
        else:
            return {'status': 'Fail', 'num_steps': 0, 'position': None, 'velocity': None}

    def gripper_percent2qpos(self, percentage:float):
        gripper_range = [0, self.gripper_max_qpos]
        target_pos = gripper_range[0] + (gripper_range[1] - gripper_range[0]) * percentage
        return target_pos

    def plan_gripper(self, pos:float, type:Literal['percent', 'qpos'] = 'percent'):
        if type == 'percent':
            target_pos = self.gripper_percent2qpos(pos)
        else:
            target_pos = pos
        gripper_pos = wp.to_torch(self.robot.data.joint_pos)[0, self._gripper_ids][0]
        num_steps = np.ceil(abs(target_pos - gripper_pos.cpu().item()) / 0.0005).astype(int)
        position = torch.linspace(gripper_pos, target_pos, num_steps, device=self.device)
        velocity = torch.clip((position - gripper_pos)/self.task.cfg.sim.dt, -0.0001, 0.0001)

        return {
            'status': 'Success',
            'num_steps': num_steps,
            'position': position.detach(),
            'velocity': velocity.detach()
        }

    def _reset_idx(self, env_ids: torch.Tensor | None=None):
        """重置环境"""
        if not hasattr(self, 'origin_pose'):
            self._setup_robot_properties()
        joint_pos = wp.to_torch(self.robot.data.default_joint_pos).clone()
        joint_vel = torch.zeros_like(joint_pos)

        self.planner.reset()
        self.robot.set_joint_position_target(joint_pos)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel)

    def get_observations(self, data_type:list[str]=['joint', 'ee']) -> dict:
        obs = {}
        if 'ee' in data_type:
            obs['ee'] = self.get_ee_pose().totensor(device=self.device)
        if 'joint' in data_type:
            obs['joint'] = wp.to_torch(self.robot.data.joint_pos).squeeze(0)
        return obs

    def get_grasp_perfect_direction(self):
        return 'top_down'
