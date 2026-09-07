# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

from ._base_task import *
import numpy as np
import tempfile
import transforms3d as t3d

@configclass
class TaskCfg(BaseTaskCfg):
    step_lim = 500
    adaptive_grasp_depth_threshold = 27.8

class Task(BaseTask):
    def __init__(self, cfg: BaseTaskCfg, mode:Literal['collect', 'eval'] = 'collect', render_mode: str|None = None, **kwargs):
        # The original 0.1 m/s regularization suppresses most IPC friction at
        # the slow speeds used by this expert trajectory.
        cfg.uipc_sim.contact.eps_velocity = 0.001
        cfg.uipc_sim.contact.default_friction_ratio = 2.0
        # UIPC recovery data is not compatible with a newly constructed scene.
        # Reusing save_dir/scene made cold-start steps take about a minute each.
        # One fresh workspace is shared by all seeds in this collection process.
        if cfg.uipc_workspace is None:
            cfg.uipc_workspace = tempfile.mkdtemp(prefix="univtac_lift_bottle_uipc_")
        # This trajectory is long enough that 60 Hz RGB+tactile capture can
        # exhaust memory before metadata is written.  Keep the 120 Hz physics
        # loop, but record lift_bottle demonstrations at 30 Hz.
        if cfg.save_frequency > 0:
            cfg.save_frequency = max(cfg.save_frequency, 4)
        if cfg.video_frequency > 0:
            cfg.video_frequency = max(cfg.video_frequency, 4)
        if cfg.render_frequency > 0:
            cfg.render_frequency = max(cfg.render_frequency, 4)
        super().__init__(cfg, mode, render_mode, **kwargs)

    def create_actors(self):
        wall_pose = Pose([0.75, 0.0, 0.005], [1, 0, 0, 0])
        bottle_pose = wall_pose.add_bias([-0.08, 0.0, 0.03])

        self.wall = self._actor_manager.add_from_usd_file(
            name='wall',
            asset_path="Wall.usd",
            pose=wall_pose,
            density=1e5
        )
        self.bottle = self._actor_manager.add_from_usd_file(
            name='bottle',
            asset_path="Bottle.usd",
            pose=bottle_pose,
        )

    def _reset_actors(self):
        bottle_offset = self.create_noise([0.01, 0.05, 0.0], [0, 0, np.pi/18])
        bottle_pose = self.wall.get_pose().add_bias([-0.08, 0.0, 0.03]).add_offset(bottle_offset)
        self.bottle.set_pose(bottle_pose)

    def pre_move(self):
        self.delay(10)
        bottle_pose = self.bottle.get_pose()
        # The cap radius is 12.5 mm.  Centering the gripper 15 mm below the
        # cap put the contact patches below the cap and into the support plane.
        target_pose = bottle_pose.add_bias([-0.13, 0, 0.0])
        target_mat = target_pose.to_transformation_matrix()
        self.grasp_noise = self.create_noise(euler=[0, [-np.pi/12, 0.0], 0])
        target_pose = construct_grasp_pose(
            target_pose.p,
            target_mat[:3, 2],
            target_mat[:3, 0]
        ).add_offset(self.grasp_noise)
        grasp_idx = self.bottle.register_point(
            pose=target_pose,
            type='contact'
        )
        self.move(self.atom.grasp_actor(
            self.bottle,
            contact_point_id=grasp_idx,
            is_close=False,
            pre_dis=0.5
        ))
        self.target_pose = self.wall.get_pose().add_bias([-0.08, 0, 0])

    def _play_once(self):
        # The Isaac Sim 6 RTX depth output of the two embedded GelSight
        # cameras is not yet a valid metric contact signal.  Close to the
        # known 25 mm cap diameter instead of letting that invalid depth stop
        # the gripper at its 40 mm initial opening.
        # At this gripper's zero pose the GelPad inner faces retain an
        # approximately 7.1 mm structural gap, so a 25 mm face-to-face opening
        # corresponds to qpos ~= (25 - 7.1) / 2 = 8.95 mm.  A calibrated
        # 7.5 mm command supplies enough compliant-pad compression to retain
        # the cap without involving the wider bottle body.
        cap_grasp_qpos = 0.0075
        self.move(self.atom.close_gripper(
            pos=cap_grasp_qpos / self._robot_manager.gripper_max_qpos,
            depth_threshold=None,
        ))

        self.move(
            self.atom.move_by_displacement(x=-0.18, z=0.20, xyz_coord='world'),
            time_dilation_factor=1.0,
        )
        horizontal_gripper_q = self._robot_manager.get_gripper_center_pose().q.copy()
        target_axis = np.array([0.0, 0.0, -1.0])
        for _ in range(8):
            bottle_axis = self.bottle.get_pose().to_transformation_matrix()[:3, 0]
            bottle_axis /= np.linalg.norm(bottle_axis)
            rotation_axis = np.cross(bottle_axis, target_axis)
            axis_norm = np.linalg.norm(rotation_axis)
            remaining_angle = np.arctan2(
                axis_norm,
                np.clip(np.dot(bottle_axis, target_axis), -1.0, 1.0),
            )
            if remaining_angle < np.deg2rad(1.0):
                break
            rotation_axis /= axis_norm
            delta_q = t3d.quaternions.axangle2quat(
                rotation_axis, min(remaining_angle, np.pi / 12)
            )
            gripper_center = self._robot_manager.get_gripper_center_pose()
            target_center = gripper_center.clone()
            target_center.q = t3d.quaternions.qmult(delta_q, gripper_center.q)
            self.move(
                [Action(
                    action='move',
                    target_pose=self._robot_manager.gripper_center_to_ee(target_center),
                )],
                tag='orient_bottle',
                delay=False,
                time_dilation_factor=1.0,
            )
            if not self.plan_success:
                break

        if self.plan_success and self.check_mid_success():
            bottle_pose = self.bottle.get_pose()
            self.move(
                self.atom.move_by_displacement(
                    z=self.target_pose.p[2] - bottle_pose.p[2], xyz_coord='world'
                ),
                time_dilation_factor=1.0,
            )
            self.move(self.atom.open_gripper(0.5))
            self.delay(10)

        if self.plan_success and self.check_mid_success():
            gripper_center = self._robot_manager.get_gripper_center_pose()
            target_center = gripper_center.clone()
            target_center.q = horizontal_gripper_q
            self.move(
                [Action(
                    action='move',
                    target_pose=self._robot_manager.gripper_center_to_ee(target_center),
                )],
                tag='restore_gripper',
                delay=False,
                time_dilation_factor=1.0,
            )

        if self.plan_success and self.check_mid_success():
            cap_pose = self.bottle.get_pose().add_bias([-0.13, 0, 0])
            gripper_center = self._robot_manager.get_gripper_center_pose()
            target_center = gripper_center.clone()
            target_center.p = cap_pose.p.copy()
            self.move(
                [Action(
                    action='move',
                    target_pose=self._robot_manager.gripper_center_to_ee(target_center),
                )],
                delay=False,
                time_dilation_factor=1.0,
            )
            self.move(self.atom.close_gripper(
                pos=cap_grasp_qpos / self._robot_manager.gripper_max_qpos,
                depth_threshold=None,
            ))
            self.move(
                self.atom.move_by_displacement(z=0.03, xyz_coord='world'),
                time_dilation_factor=1.0,
            )
            bottle_pose = self.bottle.get_pose()
            self.move(
                self.atom.move_by_displacement(
                    x=self.target_pose.p[0] - 0.015 - bottle_pose.p[0],
                    y=self.target_pose.p[1] - bottle_pose.p[1],
                    xyz_coord='world',
                ),
                time_dilation_factor=1.0,
            )
            bottle_pose = self.bottle.get_pose()
            self.move(
                self.atom.move_by_displacement(
                    z=self.target_pose.p[2] - bottle_pose.p[2], xyz_coord='world'
                ),
                time_dilation_factor=1.0,
            )
        self.move(self.atom.open_gripper(0.5))
        self.delay(30, is_save=False)

    def check_mid_success(self):
        bottle_axis = self.bottle.get_pose().to_transformation_matrix()[:3, 0]
        return np.abs(np.dot(bottle_axis, np.array([0, 0, 1]))) > 0.95

    def check_early_stop(self):
        rel_pose = self.bottle.get_pose().rebase(self.target_pose)
        if self.take_action_cnt > 300 and np.abs(np.dot(rel_pose.to_transformation_matrix()[:3, 0], np.array([-1, 0, 0]))) > 0.99:
            return True
        return False

    def check_success(self):
        rel_pose = self.bottle.get_pose().rebase(self.target_pose)
        alignment = np.abs(np.dot(
            rel_pose.to_transformation_matrix()[:3, 0], np.array([0, 0, 1])
        ))
        return rel_pose[0] > -0.02 \
            and np.all(np.abs(rel_pose[1:3]) < np.array([0.1, 0.001])) \
            and alignment > 0.99
