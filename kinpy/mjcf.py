import io
from typing import Dict, Optional, TextIO, Union

from . import chain, frame, mjcf_parser, transform

JOINT_TYPE_MAP: Dict[str, str] = {"hinge": "revolute", "slide": "prismatic"}


def geoms_to_visuals(geom, base: Optional[transform.Transform] = None):
    base = base or transform.Transform()
    visuals = []
    for g in geom:
        if g.type == "capsule":
            param = (g.size[0], g.fromto)
        elif g.type == "sphere":
            param = g.size[0]
        elif g.type == "mesh":
            param = g.mesh.file.prefix + g.mesh.file.extension
        else:
            raise ValueError("Invalid geometry type %s." % g.type)
        visuals.append(
            frame.Visual(offset=base * transform.Transform(g.quat, g.pos), geom_type=g.type, geom_param=param)
        )
    return visuals


def body_to_link(body, base: Optional[transform.Transform] = None):
    base = base or transform.Transform()
    return frame.Link(body.name, offset=base * transform.Transform(body.quat, body.pos))


def joint_to_joint(joint, base: Optional[transform.Transform] = None):
    base = base or transform.Transform()
    return frame.Joint(
        joint.name,
        offset=base * transform.Transform(pos=joint.pos),
        joint_type=JOINT_TYPE_MAP[joint.type],
        axis=joint.axis,
    )


def add_composite_joint(root_frame, joints, base: Optional[transform.Transform] = None):
    base = base or transform.Transform()
    if len(joints) > 0:
        root_frame.children = root_frame.children + [
            frame.Frame(link=frame.Link(name=root_frame.link.name + "_child"), joint=joint_to_joint(joints[0], base))
        ]
        ret, offset = add_composite_joint(root_frame.children[-1], joints[1:])
        return ret, root_frame.joint.offset * offset
    else:
        return root_frame, root_frame.joint.offset


def _build_chain_recurse(root_frame, root_body):
    base = root_frame.link.offset
    cur_frame, cur_base = add_composite_joint(root_frame, root_body.joint, base)
    jbase = cur_base.inverse() * base
    if len(root_body.joint) > 0:
        cur_frame.link.visuals = geoms_to_visuals(root_body.geom, jbase)
    else:
        cur_frame.link.visuals = geoms_to_visuals(root_body.geom)
    for b in root_body.body:
        cur_frame.children = cur_frame.children + [frame.Frame()]
        next_frame = cur_frame.children[-1]
        next_frame.name = b.name + "_frame"
        next_frame.link = body_to_link(b, jbase)
        _build_chain_recurse(next_frame, b)


def build_chain_from_mjcf(data: Union[str, TextIO], model_dir: str = "") -> chain.Chain:
    """
    Build a Chain object from MJCF data.

    Parameters
    ----------
    data : str or TextIO
        MJCF string data or file object.

    Returns
    -------
    chain.Chain
        Chain object created from MJCF.
    """
    if isinstance(data, io.TextIOBase):
        data = data.read()
    model = mjcf_parser.from_xml_string(data, model_dir=model_dir)
    root_body = model.worldbody.body[0]
    root_frame = frame.Frame(root_body.name + "_frame", link=body_to_link(root_body), joint=frame.Joint())
    _build_chain_recurse(root_frame, root_body)
    return chain.Chain(root_frame)


def build_serial_chain_from_mjcf(
    data: Union[str, TextIO], end_link_name: str, root_link_name: str = "", model_dir: str = ""
) -> chain.SerialChain:
    """
    Build a SerialChain object from MJCF data.

    Parameters
    ----------
    data : str
        MJCF string data.
    end_link_name : str
        The name of the link that is the end effector.
    root_link_name : str, optional
        The name of the root link.

    Returns
    -------
    chain.SerialChain
        SerialChain object created from MJCF.
    """
    if isinstance(data, io.TextIOBase):
        data = data.read()
    mjcf_chain = build_chain_from_mjcf(data, model_dir)
    return chain.SerialChain(
        mjcf_chain, end_link_name + "_frame", "" if root_link_name == "" else root_link_name + "_frame"
    )
