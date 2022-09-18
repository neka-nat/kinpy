import os

from .chain import Chain
from .mjcf import *
from .sdf import *
from .transform import *
from .urdf import *
from .visualizer import *


def build_chain_from_file(filename: str) -> Chain:
    ext = os.path.splitext(filename)[-1]
    if ext == ".urdf":
        return build_chain_from_urdf(open(filename).read())
    elif ext == ".sdf":
        return build_chain_from_sdf(open(filename).read())
    elif ext == ".mjcf":
        return build_chain_from_mjcf(open(filename).read())
    else:
        raise ValueError(f"Invalid file type: '{ext}' file.")
