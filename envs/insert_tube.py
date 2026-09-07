# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

from ._base_task import *
import numpy as np
import tempfile

@configclass
class TaskCfg(BaseTaskCfg):
    pass

class Task(BaseTask):
    def __init__(self, cfg: TaskCfg, mode:Literal['collect', 'eval'] = 'collect', **kwargs):
        cfg.uipc_sim.contact.eps_velocity = 0.001
        cfg.uipc_sim.contact.default_friction_ratio = 3.0
        if cfg.uipc_workspace is None:
            cfg.uipc_workspace = tempfile.mkdtemp(prefix="univtac_insert_tube_uipc_")
        super().__init__(cfg=cfg, mode=mode, **kwargs)

    def create_actors(self):
        slot_pose = Pose([0.6, 0.0, 0.002], [1, 0, 0, 0])
        base_pose = Pose([0.4, 0.0, 0.002], [1, 0, 0, 0])
        prism_pose = Pose([0.4, 0.0, 0.005], [1, 0, 0, 0])

        self.slot = self._actor_manager.add_from_usd_file(
            name='slot',
            asset_path="TestTubeSlot.usd",
            pose=slot_pose,
            density=1e5
        )
        self.prism_base = self._actor_manager.add_from_usd_file(
            name='prism_base',
            asset_path="TestTubeBase.usd",
            pose=base_pose,
            density=1e5
        )
        self.prism = self._actor_manager.add_from_usd_file(
            name='prism',
            asset_path="TestTube.usd",
            pose=prism_pose,
            density=10,
            # Keep the tube length and grasp locations unchanged.  A 5%
            # radial reduction leaves only 0.5 mm clearance per side, which
            # is entirely consumed by UIPC's 0.5 mm contact barrier and makes
            # entry at the rim nondeterministic.  Leave a small amount of
            # effective clearance after accounting for that barrier.
            scale=(0.90, 0.90, 1.0),
        )

    def _reset_actors(self):
        slot_offset = self.create_noise([0.005, 0.01, 0.0])
        slot_pose = Pose([0.6, 0.0, self.slot.get_pose()[2]], [1, 0, 0, 0]).add_offset(slot_offset)
        self.slot.set_pose(slot_pose)

    def pre_move(self):
        # Only settle the scene here. Record grasping, transport, alignment,
        # and insertion together as one complete policy trajectory.
        self.delay(10)

    def _play_once(self):
        self.delay(5, is_save=True)
        grasp_bias = self.rng.uniform(0.095, 0.10)
        self.metadata['grasp_bias'] = float(grasp_bias)
        target_pose = self.prism.get_pose().add_bias([0, 0, grasp_bias])

        cpose = construct_grasp_pose(
            target_pose.p,
            [0, 0, 1],
            [1, 0, 0]
        )
        self.cid = self.prism.register_point(cpose, type='contact')
        self.move(self.atom.grasp_actor(
            self.prism,
            contact_point_id=self.cid,
            pre_dis=0.0, dis=0.0,
            is_close=False,
        ))
        # TestTube.usd is 20 mm across and is spawned at 95% radial scale for
        # this tight-fit task. Isaac Sim 6 RTX tactile depth is not a valid
        # metric contact signal, so use that known width and a compliant-pad
        # preload instead of adaptive closing.
        tube_diameter = 0.018
        face_gap_at_zero = 0.0071
        grip_preload = 0.004
        grasp_qpos = max(
            0.0,
            (tube_diameter - face_gap_at_zero) / 2.0 - grip_preload,
        )
        self.metadata['grasp_qpos'] = float(grasp_qpos)
        self.move(self.atom.close_gripper(
            pos=grasp_qpos / self._robot_manager.gripper_max_qpos,
            depth_threshold=None,
        ))
        self.origin_inhand_pose = self.prism.get_pose().rebase(
            self._robot_manager.get_gripper_center_pose())

        base_pose = self.slot.get_pose()
        self.random_noise = self.create_noise(
            [[0.001, 0.004], [0.001, 0.004], 0])
        self.random_noise[:2] *= np.sign(self.rng.uniform(-1, 1, size=2))
        self.metadata['random_noise'] = self.random_noise.tolist()
        self.hole_pose = base_pose.add_bias([-0.008, 0, 0.077]).add_rotation(
            [0, -np.pi/6, 0]
        )
        try_pose = self.hole_pose.add_offset(self.random_noise)

        self.move(self.atom.move_by_displacement(z=0.15), constraint_pose=[1, 1, 1, 1, 1, 0])
        self.move(self.atom.place_actor(
            self.prism,
            target_pose=try_pose,
            pre_dis=0.1, dis=0.05,
            is_open=False
        ))
        # Keep the original randomized coarse approach in the demonstration,
        # but correct its 1--4 mm lateral error while still 50 mm clear of the
        # rim. The tube/slot clearance is sub-millimetre, and on Isaac Sim 6 a rim
        # collision lets the tube slip inside the compliant tactile fingers;
        # attempting to correct only after that contact is no longer reliable.
        self.move(self.atom.place_actor(
            self.prism,
            target_pose=self.hole_pose,
            pre_dis=0.05, dis=0.002,
            is_open=False,
        ), tag='fine_align_to_hole', time_dilation_factor=0.5, delay=False)
        self.metadata['rel_after_align'] = self.prism.get_pose().rebase(
            self.hole_pose
        ).tolist()
        # Probe the entrance in small increments.  If the tube catches the
        # rim, retreat before the arm can push it several centimetres through
        # the compliant fingers, then restore the grasp and alignment once.
        # Do not use BaseTask.try_forward here: its generic rigid-object
        # progress check expects the object to track every commanded step
        # exactly, while the compliant GelSight pads intentionally introduce
        # a small lag and cause a premature stop at the first 4 mm step.
        for _ in range(2):
            self.move(self.atom.move_by_displacement(
                z=0.01, xyz_coord='local'
            ), tag='entry_probe', time_dilation_factor=0.5, delay=False)
        entry_depth = -self.prism.get_pose().rebase(self.hole_pose).p[2]
        self.metadata['entry_probe_depth'] = float(entry_depth)
        if entry_depth < 0.008:
            self.move(self.atom.move_by_displacement(
                z=-0.012, xyz_coord='local'
            ), tag='retreat_from_rim', time_dilation_factor=0.3, delay=False)
            self.move(self.atom.close_gripper(
                pos=grasp_qpos / self._robot_manager.gripper_max_qpos,
                depth_threshold=None,
            ), tag='restore_grasp', delay=False)
            self.move(self.atom.place_actor(
                self.prism,
                target_pose=self.hole_pose,
                pre_dis=0.03, dis=0.002,
                is_open=False,
            ), tag='realign_after_rim_contact', time_dilation_factor=0.4,
                delay=False)
            for _ in range(2):
                self.move(self.atom.move_by_displacement(
                    z=0.01, xyz_coord='local'
                ), tag='entry_retry', time_dilation_factor=0.5, delay=False)
            retry_depth = -self.prism.get_pose().rebase(self.hole_pose).p[2]
            self.metadata['entry_retry_depth'] = float(retry_depth)

        # Once the tip is inside, continue in short, slow segments instead of
        # one 40 mm push.  This gives contact dynamics time to settle and
        # prevents a small rim contact from turning into a full-length slip.
        for _ in range(2):
            self.move(self.atom.move_by_displacement(
                z=-0.02, xyz_coord=self.hole_pose
            ), tag='staged_insert', time_dilation_factor=0.4, delay=False)
        self.delay(20, is_save=False)

    def check_mid_success(self):
        prism_pose = self.prism.get_pose().rebase(self.hole_pose)
        return np.all(np.abs(prism_pose.p[:2]) < np.array([0.005, 0.005])) and prism_pose.p[2] < -0.02 and\
            np.dot(prism_pose.to_transformation_matrix()[:3, 2], np.array([0, 0, 1])) > 0.99 # 8°

    def check_early_stop(self):
        prism_inhand_pose = self.prism.get_pose().rebase(
            self._robot_manager.get_gripper_center_pose())
        inhand_bias = np.abs(self.origin_inhand_pose[2] - prism_inhand_pose[2])
        if inhand_bias > 0.03:
            self.metadata['early_stop'] = True
            self.metadata['inhand_bias'] = float(inhand_bias)
            return True

    def check_success(self, z_threshold=0.03):
        prism_pose = self.prism.get_pose().rebase(self.hole_pose)
        prism_inhand_pose = self.prism.get_pose().rebase(
            self._robot_manager.get_gripper_center_pose())
        self.metadata['rel_pose'] = prism_pose.tolist()
        self.metadata['inhand_bias'] = np.abs(self.origin_inhand_pose[2] - prism_inhand_pose[2])
        return np.all(np.abs(prism_pose.p[:2]) < np.array([0.005, 0.005])) and prism_pose.p[2] < -z_threshold \
            and np.dot(prism_pose.to_transformation_matrix()[:3, 2], np.array([0, 0, 1])) > 0.965 \
            and np.abs(self.origin_inhand_pose[2] - prism_inhand_pose[2]) < 0.03
