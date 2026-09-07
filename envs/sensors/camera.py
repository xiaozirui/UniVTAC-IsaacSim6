# Modified from the UniVTAC project.
# Changes in this derivative project include compatibility adaptations
# for Isaac Sim 6.0.1 and Isaac Lab 3.0.0.

from isaaclab.sensors import TiledCameraCfg, TiledCamera
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils
from pxr import Gf

import copy
import torch
import torchvision.transforms.functional as F
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .._base_task import BaseTask
    from tacex_uipc import UipcInteractiveScene

@configclass
class CameraCfg(TiledCameraCfg):
    name: str = 'camera'

class CameraManager:
    def __init__(self, cfg_list: list[CameraCfg], task:'BaseTask'):
        self.scene = task.scene
        self.cfg_list = cfg_list
        self.cameras = {}

    def setup(self):
        self.cameras = {
            cam_cfg.name: self.add_camera(cam_cfg) for cam_cfg in self.cfg_list
        }

    def add_camera(self, cam_cfg: CameraCfg):
        # Camera spawning in Isaac Lab 6 requires a concrete prim path. UIPC
        # collection currently runs a single environment, so resolve the
        # legacy environment regex to that cloned environment.
        cam_cfg = copy.deepcopy(cam_cfg)
        cam_cfg.prim_path = cam_cfg.prim_path.replace("env_.*", "env_0")
        camera = TiledCamera(cam_cfg)
        # Some legacy camera USDs authored a scalar xformOp:scale. The Isaac
        # Lab 6 Fabric frame view requires the declared Vec3d value.
        for prim in sim_utils.find_matching_prims(cam_cfg.prim_path):
            scale_attr = prim.GetAttribute("xformOp:scale")
            if scale_attr and isinstance(scale_attr.Get(), (int, float)):
                scale = float(scale_attr.Get())
                scale_attr.Set(Gf.Vec3d(scale, scale, scale))
        camera._initialize_impl()
        camera._is_initialized = True
        self.scene.sensors[f'camera_{cam_cfg.name}'] = camera
        return camera

    def get_observations(self, data_types: list[str] = None):
        obs = {}
        if data_types is None:
            data_types = ['rgb', 'rgba']
        for name, cam in self.cameras.items():
            obs[name] = {}
            for data_type in data_types:
                if data_type == 'rgb':
                    obs[name]['rgb'] = cam.data.output['rgb'].squeeze(0)
                elif data_type == 'rgba':
                    obs[name]['rgba'] = cam.data.output['rgba'].squeeze(0)
                elif data_type == 'depth':
                    obs[name]['depth'] = cam.data.output['depth'].squeeze(0)
        return obs
