# Modified from the UniVTAC project.
# Changes in this derivative project include external asset-root configuration
# for the source-only Isaac Sim 6.0.1 release.

import os
from pathlib import Path

_default_assets_root = Path(__file__).parent.parent / 'assets'
ASSETS_ROOT = Path(os.environ.get('UNIVTAC_ASSETS_ROOT', _default_assets_root)).expanduser()
EMBODIMENTS_ROOT = ASSETS_ROOT / 'embodiments'
OBJECTS_ROOT = ASSETS_ROOT / 'objects'
SCENE_ASSETS_ROOT = ASSETS_ROOT / 'scene'
STATUS_ROOT = ASSETS_ROOT / 'status_ckpt'
TEXTURES_ROOT = ASSETS_ROOT / 'textures'
