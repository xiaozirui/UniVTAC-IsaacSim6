# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

from ._base_task import *
import numpy as np
import tempfile

@configclass
class TaskCfg(BaseTaskCfg):
    cameras = [
        CameraCfg(
            name="head",
            prim_path="/World/envs/env_.*/Camera",
            # Oblique overview of both the connector and socket. The former
            # camera sat 6.6 cm above the table and looked almost horizontally
            # into the socket housing, leaving the manipulation out of frame.
            offset=CameraCfg.OffsetCfg(
                pos=(0.70, 0.28, 0.14),
                rot=(0.154323, 0.551537, 0.789430, 0.220886),
                convention="opengl",
            ),
            data_types=["rgb", "depth"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=3.2, focus_distance=0.35, horizontal_aperture=3.6, clipping_range=(0.05, 100.0)
            ),
            width=480,
            height=270,
            update_period=1/120
        ),
        CameraCfg(
            name="wrist",
            prim_path="/World/envs/env_.*/Robot/WristCamera/Camera",
            data_types=["rgb", "depth"],
            spawn=None, # use existing camera
            width=480,
            height=270,
            update_period=1/120,
        )
    ]
    step_lim = 600

class Task(BaseTask):
    def __init__(self, cfg: BaseTaskCfg, mode:Literal['collect', 'eval'] = 'collect', render_mode: str|None = None, **kwargs):
        cfg.sim.physics_material.dynamic_friction = 2.5
        cfg.sim.physics_material.static_friction = 2.5
        cfg.uipc_sim.contact.default_friction_ratio = 2.5
        # Slow manipulation needs a much smaller IPC friction regularization
        # than the legacy 0.1 m/s default.
        cfg.uipc_sim.contact.eps_velocity = 0.001
        if cfg.uipc_workspace is None:
            cfg.uipc_workspace = tempfile.mkdtemp(prefix="univtac_insert_hdmi_uipc_")
        super().__init__(cfg, mode, render_mode, **kwargs)

    def create_actors(self):
        base_pose = Pose([0.55, 0.0, 0.002], [1, 0, 0, 0])
        prism_pose = Pose([0.4, 0.0, 0.002], [1, 0, 0, 0])

        self.slot = self._actor_manager.add_from_usd_file(
            name='slot',
            asset_path="HDMISlot.usd",
            pose=base_pose,
            density=1e5
        )

        self.prism = self._actor_manager.add_from_usd_file(
            name='prism',
            asset_path="HDMI.usd",
            pose=prism_pose
        )

    def _reset_actors(self):
        base_offset = self.create_noise([0.005, 0.005, 0.0])
        base_pose = Pose([0.55, 0.0, self.slot.get_pose()[2]], [1, 0, 0, 0]).add_offset(base_offset)
        self.slot.set_pose(base_pose)

    def pre_move(self):
        # Only settle the scene here. All task actions belong in _play_once so
        # the saved demonstration contains the complete pick-and-insert skill.
        self.delay(10)

    def _play_once(self):
        # Record a short initial-state lead-in before the robot starts moving.
        self.delay(5, is_save=True)
        self.move(self.atom.open_gripper(0.5))
        grasp_rotate = self.rng.uniform(-np.pi/18, np.pi/18)
        self.metadata['grasp_rotate'] = float(grasp_rotate)
        target_pose = self.prism.get_pose().add_bias([0, 0, 0.012]).add_rotation([0, grasp_rotate, 0])
        target_mat = target_pose.to_transformation_matrix()
        cpose = construct_grasp_pose(
            target_pose.p,
            target_mat[:3, 2],
            target_mat[:3, 0]
        )
        cid = self.prism.register_point(cpose, type='contact')
        self.move(self.atom.grasp_actor(
            self.prism,
            contact_point_id=cid,
            is_close=False
        ))
        # The connector is 16 mm wide along the finger closing direction.
        # Isaac Sim 6 RTX tactile depth is not a valid metric contact signal,
        # so adaptive closing otherwise stops while the fingers are still
        # open. Account for the 7.1 mm residual pad gap and add 1.5 mm of
        # compliant-pad preload on each side.
        connector_width = 0.016
        face_gap_at_zero = 0.0071
        grip_preload = 0.0015
        grasp_qpos = max(
            0.0,
            (connector_width - face_gap_at_zero) / 2.0 - grip_preload,
        )
        self.metadata['grasp_qpos'] = float(grasp_qpos)
        self.move(self.atom.close_gripper(
            pos=grasp_qpos / self._robot_manager.gripper_max_qpos,
            depth_threshold=None,
        ))
        self.move(self.atom.move_by_displacement(z=0.02))

        self.target_pose = self.slot.get_pose().add_bias([0.0, 0.0, 0.005])
        self.hole_pose = self.slot.get_pose().add_bias([0.0, 0.0, 0.0128])
        noise = self.create_noise([0.005, 0.005, 0.0])
        self.noise_pose = self.hole_pose.add_offset(noise)
        self.move(self.atom.place_actor(
            self.prism,
            target_pose=self.noise_pose,
            pre_dis=0.02,
            dis=0.01,
            is_open=False
        ))
        self.move(self.atom.place_actor(
            self.prism,
            target_pose=self.hole_pose,
            pre_dis=0.01,
            dis=0.002,
            is_open=False
        ), time_dilation_factor=0.5)
        self.move(self.atom.move_by_displacement(
            z=0.005, xyz_coord='local'
        ), time_dilation_factor=0.5, constraint_pose=[1, 1, 1, 1, 1, 0])
        self.move(self.atom.move_by_displacement(
            z=0.002, xyz_coord='local'
        ), time_dilation_factor=0.5, constraint_pose=[1, 1, 1, 1, 1, 0])
        self.delay(20, is_save=True)

    def check_success(self, lateral_threshold=0.003, axial_threshold=0.004):
        prism_pose = self.prism.get_pose().rebase(self.target_pose)
        ee_pose = self._robot_manager.get_ee_pose()
        self.metadata['rel_pose'] = prism_pose.tolist()
        # Require the connector origin to reach the intended fully inserted
        # pose in all three axes. The old p[1:2] slice checked only y, so a
        # connector left at its initial x (about -0.15 m relative to the port)
        # was incorrectly marked successful.
        return np.all(np.abs(prism_pose.p[:2]) < lateral_threshold) \
            and np.abs(prism_pose.p[2]) < axial_threshold \
            and ee_pose[2] > 0.145 and \
            np.dot(
                prism_pose.to_transformation_matrix()[:3, 2],
                np.array([0, 0, 1]),
            ) > 0.999
