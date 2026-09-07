# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

from ._base_task import *
import numpy as np
import tempfile
import transforms3d as t3d

@configclass
class TaskCfg(BaseTaskCfg):
    adaptive_grasp_depth_threshold = 27.75

class Task(BaseTask):
    def __init__(self, cfg: BaseTaskCfg, mode:Literal['collect', 'eval'] = 'collect', render_mode: str|None = None, **kwargs):
        # The original 0.1 m/s regularization suppresses most IPC friction at
        # the slow speeds used by this expert trajectory.
        cfg.uipc_sim.contact.eps_velocity = 0.001
        cfg.uipc_sim.contact.default_friction_ratio = 2.0
        # Recovery data written by an older Isaac Sim/UIPC scene is unsafe to
        # reuse after reconstructing the scene. Share one fresh workspace
        # across seeds in this collection process.
        if cfg.uipc_workspace is None:
            cfg.uipc_workspace = tempfile.mkdtemp(prefix="univtac_lift_can_uipc_")
        super().__init__(cfg, mode, render_mode, **kwargs)

    def create_actors(self):
        self.cans:dict[int, Actor] = {}
        pose_dict = {
            4: Pose([-1.0, 0.0, 0.022], [1, 0, 0, 0]),
            5: Pose([-1.0, 1.0, 0.027], [1, 0, 0, 0]),
            6: Pose([-1.0, -1.0, 0.032], [1, 0, 0, 0])
        }
        for d in [4, 5, 6]:
            self.cans[d] = self._actor_manager.add_from_usd_file(
                name=f'can_d{d}',
                asset_path=f"Can_d{d}cm.usd",
                pose=pose_dict[d]
            )

    def _reset_actors(self):
        can_offset = self.create_noise([0.02, 0.05, 0.0])
        can_size = self.rng.choice([4, 5, 6])
        can_pose = Pose(
            [0.7, 0.0, 0.005*can_size+0.001], [1, 0, 0, 0]
        ).add_offset(can_offset)

        self.can = self.cans[can_size]
        self.can_size = int(can_size)
        self.metadata['can_size'] = int(can_size)
        self.can.set_pose(can_pose)

    def pre_move(self):
        self.delay(10)

        self.move(self.atom.open_gripper(1.0))
        can_pose = self.can.get_pose()
        # Center the pads on the can axis. The former -8 mm vertical bias put
        # contact on the lower curved surface, close to the support plane, so
        # the gripper produced only point contact and slid off in Isaac Sim 6.
        target_pose = can_pose.add_bias([-0.065, 0, 0.0])
        target_mat = target_pose.to_transformation_matrix()
        x = target_mat[:3, 0].reshape(-1)
        target_mat = np.vstack([
            x, np.cross(x, [0, 0, 1]), [0, 0, 1],
        ])
        self.grasp_noise = self.create_noise(euler=[0, [-np.pi/18, 0.0], 0])
        self.metadata['grasp_noise'] = self.grasp_noise.tolist()
        target_pose = construct_grasp_pose(
            target_pose.p,
            target_mat[:3, 2],
            target_mat[:3, 0],
        ).add_offset(self.grasp_noise)
        grasp_idx = self.can.register_point(
            pose=target_pose,
            type='contact'
        )
        self.move(self.atom.grasp_actor(self.can, contact_point_id=grasp_idx, is_close=False))
        self.origin_inhand_pose = self._robot_manager.get_inhand_pose(self.can)

    def _play_once(self):
        # Isaac Sim 6 RTX depth from the embedded GelSight cameras is not a
        # valid metric contact signal yet, so adaptive closing can stop before
        # touching the can. Convert the known can diameter to the Panda finger
        # joint target. At qpos=0 the compliant pad faces retain an about
        # 7.1 mm structural gap; another 2 mm per side supplies grip preload.
        can_diameter = self.can_size * 0.01
        face_gap_at_zero = 0.0071
        grip_preload = 0.002
        grasp_qpos = max(
            0.0,
            (can_diameter - face_gap_at_zero) / 2.0 - grip_preload,
        )
        self.metadata['grasp_qpos'] = float(grasp_qpos)
        self.move(self.atom.close_gripper(
            pos=grasp_qpos / self._robot_manager.gripper_max_qpos,
            depth_threshold=None,
        ))

        # The original trajectory kept the gripper orientation fixed while
        # orbiting around the can base. It depended on Isaac Sim 4.5 table
        # friction to pivot the can and no longer lifts it reliably in 6.0.1.
        # Lift clear of the table, rotate the grasp itself, then lower the can
        # base back onto the support surface.
        self.move(
            # Pull the grasp back into the Panda's dexterous workspace before
            # pitching the wrist.  Rotating at the original x ~= 0.7 m reach
            # drives the arm into an IK limit after only 15--30 degrees.
            self.atom.move_by_displacement(x=-0.18, z=0.20, xyz_coord='world'),
            time_dilation_factor=1.0,
        )
        target_axis = np.array([0.0, 0.0, -1.0])
        alignment_history = []
        for _ in range(8):
            can_axis = self.can.get_pose().to_transformation_matrix()[:3, 0]
            can_axis /= np.linalg.norm(can_axis)
            alignment_history.append(float(np.dot(can_axis, target_axis)))
            rotation_axis = np.cross(can_axis, target_axis)
            axis_norm = np.linalg.norm(rotation_axis)
            remaining_angle = np.arctan2(
                axis_norm,
                np.clip(np.dot(can_axis, target_axis), -1.0, 1.0),
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
                tag='orient_can',
                delay=False,
                time_dilation_factor=1.0,
            )
            if not self.plan_success:
                break
        can_axis = self.can.get_pose().to_transformation_matrix()[:3, 0]
        alignment_history.append(float(np.dot(can_axis, target_axis)))
        self.metadata['orientation_alignment'] = alignment_history

        if self.plan_success and self.check_mid_success():
            can_pose = self.can.get_pose()
            self.move(
                self.atom.move_by_displacement(
                    z=0.001 - can_pose.p[2], xyz_coord='world'
                ),
                time_dilation_factor=1.0,
            )
        self.move(self.atom.open_gripper())
        self.delay(30, is_save=False)

    def check_mid_success(self):
        can_pose = self.can.get_pose()
        return np.abs(np.dot(can_pose.to_transformation_matrix()[:3, 0], np.array([0, 0, 1]))) > 0.95

    def check_early_stop(self):
        can_pose = self.can.get_pose()
        inhand_pose = self._robot_manager.get_inhand_pose(self.can)
        min_depth = torch.min(self._tactile_manager.get_min_depth()).item()

        # Isaac Sim 6 can return negative RTX-derived tactile depths.  They
        # are invalid measurements, not excessive indentation, and must not
        # reject an otherwise successful episode.
        if 0.0 <= min_depth < 20:
            self.metadata['early_stop'] = True
            self.metadata['min_depth'] = float(min_depth)
            return True
        if np.abs(inhand_pose.p[2] - self.origin_inhand_pose.p[2]) > 0.05 and \
            np.abs(np.dot(can_pose.to_transformation_matrix()[:3, 2], np.array([0, 0, 1]))) > 0.99:
            self.metadata['early_stop'] = True
            self.metadata['inhand_dis'] = float(np.abs(inhand_pose.p[2] - self.origin_inhand_pose.p[2]))
            return True
        return False

    def check_success(self):
        can_pose = self.can.get_pose()
        return can_pose[2] < 0.01 and \
            np.abs(np.dot(can_pose.to_transformation_matrix()[:3, 0], np.array([0, 0, 1]))) > 0.99
