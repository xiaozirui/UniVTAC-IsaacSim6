# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

from ._base_task import *
import numpy as np
import tempfile

@configclass
class TaskCfg(BaseTaskCfg):
    adaptive_grasp_depth_threshold = 27.5

class Task(BaseTask):
    def __init__(self, cfg: BaseTaskCfg, mode:Literal['collect', 'eval'] = 'collect', render_mode: str|None = None, **kwargs):
        cfg.sim.physics_material.dynamic_friction = 2.5
        cfg.sim.physics_material.static_friction = 2.5
        cfg.sim.render.antialiasing_mode = "Off"
        cfg.sim.render.rendering_mode = "performance"
        cfg.uipc_sim.contact.default_friction_ratio = 2.5
        cfg.uipc_sim.contact.eps_velocity = 0.001
        if cfg.uipc_workspace is None:
            cfg.uipc_workspace = tempfile.mkdtemp(prefix="univtac_put_bottle_shelf_uipc_")
        if cfg.save_frequency > 0:
            cfg.save_frequency = max(cfg.save_frequency, 4)
        if cfg.video_frequency > 0:
            cfg.video_frequency = max(cfg.video_frequency, 4)
        if cfg.render_frequency > 0:
            cfg.render_frequency = max(cfg.render_frequency, 4)
        super().__init__(cfg, mode, render_mode, **kwargs)

    def create_actors(self):
        base_pose = Pose(
            [0.40, 0.40, 0.01],
            [np.sqrt(0.5), 0, 0, np.sqrt(0.5)],
        )
        bottle_pose = Pose([0.50, 0.0, 0.01], [1, 0, 0, 0])

        self.shelf = self._actor_manager.add_from_usd_file(
            name='shelf',
            asset_path="Shelf.usd",
            pose=base_pose,
            constitution_cfg=UipcObjectCfg.AffineBodyConstitutionCfg(
                kinematic=True),
        )
        self.bottle = self._actor_manager.add_from_usd_file(
            name='prism',
            asset_path="BottleLift.usd",
            pose=bottle_pose
        )

    def _reset_actors(self):
        # Shelf.usd's point-cloud proxy closes its actual opening; collision
        # safety is provided by the explicit approach path and kinematic shelf.
        self.planner_ignored_actor_names = set()
        base_offset = self.create_noise([0.01, 0.0, 0.0])
        base_pose = Pose(
            [0.40, 0.40, 0.01],
            [np.sqrt(0.5), 0, 0, np.sqrt(0.5)],
        ).add_offset(base_offset)
        bottle_offset = self.create_noise([0.0, 0.03, 0.0])
        bottle_pose = Pose([0.50, 0.0, 0.01], [1, 0, 0, 0]).add_offset(bottle_offset)

        self.shelf.set_pose(base_pose)
        self.bottle.set_pose(bottle_pose)
        self.shelf_init_pose = base_pose

    def pre_move(self):
        self.delay(10)

        # Open far enough to descend around the bottle without dragging either
        # compliant pad across it.  Do not let invalid Isaac Sim 6 RTX depth
        # terminate this command early.
        self.move(self.atom.open_gripper(0.8, depth_threshold=None))
        bottle_pose = self.bottle.get_pose()
        target_pose = bottle_pose.add_bias([0, 0, 0.110+0.005*self.rng.random()])
        self.metadata['grasp_height'] = float(target_pose.p[2] - bottle_pose.p[2])
        target_pose = construct_grasp_pose(
            target_pose.p,
            [0, 0, 1],
            [1, 0, 0],
        )
        grasp_idx = self.bottle.register_point(
            pose=target_pose,
            type='contact'
        )
        self.move(self.atom.grasp_actor(
            self.bottle, contact_point_id=grasp_idx, pre_dis=0.0, is_close=False
        ))

        # BottleLift.usd is 25 mm across at this 110--115 mm neck grasp height.
        # Each finger joint represents half of the opening.  Account for the
        # gripper's residual face gap, then add a modest compliant-pad preload.
        # Sim 6's non-metric RTX tactile depth cannot be the stopping condition.
        bottle_diameter = 0.025
        face_gap_at_zero = 0.0071
        grip_preload = 0.0003
        self.grasp_qpos = max(
            0.0,
            (bottle_diameter - face_gap_at_zero) / 2.0 - grip_preload,
        )
        self.metadata['grasp_qpos'] = float(self.grasp_qpos)
        self.move(self.atom.close_gripper(
            pos=self.grasp_qpos / self._robot_manager.gripper_max_qpos,
            depth_threshold=None,
        ))
        self.metadata['post_grasp_qpos'] = float(
            self._robot_manager.get_gripper_qpos())
        # Lift before collection begins.  The initial object-to-shelf spacing
        # keeps the wrist clear of the front frame throughout this pre-move.
        self.planner_ignored_actor_names = {'prism'}
        bottle_pose = self.bottle.get_pose()
        self.move(self.atom.move_by_displacement(
            z=0.24 - bottle_pose.p[2],
            xyz_coord='world',
        ), tag='pre_lift', is_save=False, time_dilation_factor=1.0)
        self.metadata['bottle_after_lift'] = self.bottle.get_pose().tolist()

        # Keep the bottle just inside the shelf opening.  A deeper target adds
        # no task value but makes the fingers and wrist collide with the front
        # frame during motion planning.
        self.place_target = self.shelf_init_pose.add_bias([-0.23, 0.0, 0.21])
        # The held bottle is not a static obstacle.  Shelf.usd's point-cloud
        # collision proxy also seals its intended opening, so the following
        # explicit outside-align-then-insert path handles that geometry.
        self.planner_ignored_actor_names = {'prism', 'shelf'}

    def _play_once(self):
        # A top-down grasp is the only pose that produces bilateral contact in
        # Sim 6 for this asset.  Keeping the bottle upright also avoids the
        # Sim-4.5 gravity_rotate routine, whose invalid depth feedback releases
        # the bottle in Sim 6.
        approach_pose = self.place_target.add_bias([-0.06, 0.0, 0.02])
        self.metadata['approach_pose'] = approach_pose.tolist()

        # The approach pose lies outside the rotated shelf.  First align there,
        # then follow the opening axis directly to the placement pose.
        target_center = self._robot_manager.get_gripper_center_pose().clone()
        target_center.p = np.array([
            approach_pose.p[0],
            approach_pose.p[1],
            self.place_target.p[2] + self.metadata['grasp_height'],
        ])
        self.metadata['target_gripper_center'] = target_center.tolist()
        self.move(self.atom.move_to_pose(
            self._robot_manager.gripper_center_to_ee(target_center),
        ), tag='align_in_front_of_shelf', time_dilation_factor=1.0)
        self.metadata['bottle_after_align'] = self.bottle.get_pose().tolist()

        target_center.p[:2] = self.place_target.p[:2]
        self.move(self.atom.move_to_pose(
            self._robot_manager.gripper_center_to_ee(target_center),
        ), tag='insert_into_shelf', time_dilation_factor=0.5)
        self.metadata['bottle_after_insert'] = self.bottle.get_pose().tolist()

        self.move(self.atom.open_gripper(0.8, depth_threshold=None))
        self.delay(20, is_save=False)

    def check_early_stop(self):
        shelf_pose = self.shelf.get_pose().rebase(self.shelf_init_pose)
        shelf_tilt = np.dot(
            shelf_pose.to_transformation_matrix()[:3, 2], np.array([0, 0, 1]))
        if np.linalg.norm(shelf_pose.p) > 0.02 or shelf_tilt < 0.995:
            self.metadata['early_stop'] = True
            self.metadata['shelf_displacement'] = float(np.linalg.norm(shelf_pose.p))
            self.metadata['shelf_tilt'] = float(shelf_tilt)
            return True
        return False

    def check_success(self):
        bottle_pose = self.bottle.get_pose().rebase(self.place_target)
        bottle_upright = np.dot(
            bottle_pose.to_transformation_matrix()[:3, 2], np.array([0, 0, 1]))
        shelf_pose = self.shelf.get_pose().rebase(self.shelf_init_pose)
        shelf_upright = np.dot(
            shelf_pose.to_transformation_matrix()[:3, 2], np.array([0, 0, 1]))
        self.metadata['final_bottle_offset'] = bottle_pose.p.tolist()
        self.metadata['final_bottle_upright'] = float(bottle_upright)
        self.metadata['final_shelf_displacement'] = float(np.linalg.norm(shelf_pose.p))
        self.metadata['final_shelf_upright'] = float(shelf_upright)
        return np.all(np.abs(bottle_pose.p) < np.array([0.02, 0.1, 0.02])) \
            and bottle_upright > 0.965 # 15°
