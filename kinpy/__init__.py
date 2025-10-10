from ast import mod
import os

from .chain import Chain
from .mjcf import *
from .sdf import *
from .transform import *
from .urdf import *
try:
    from .visualizer import *
except ImportError:
    pass

def build_chain_from_file(filename: str, model_dir: str = "") -> Chain:
    ext = os.path.splitext(filename)[-1].lower()
    data = open(filename).read()
    if ext == ".urdf" or data.lstrip().startswith("<robot"):
        return build_chain_from_urdf(data)
    elif ext == ".sdf" or data.lstrip().startswith("<sdf"):
        return build_chain_from_sdf(data)
    elif ext in (".mjcf", ".xml") or data.lstrip().startswith("<mujoco"):
        return build_chain_from_mjcf(data, model_dir=model_dir)
    else:
        raise ValueError(f"Invalid file type: '{ext}' file.")
