from typing import Any, Iterator, List, Optional

import numpy as np
import transformations as tf

from . import transform


class Visual:
    TYPES = ["box", "cylinder", "sphere", "capsule", "mesh"]

    def __init__(
        self,
        offset: Optional[transform.Transform] = None,
        geom_type: Optional[str] = None,
        geom_param: Any = None,
    ) -> None:
        self.offset = offset or transform.Transform()
        self.geom_type = geom_type
        self.geom_param = geom_param

    def __repr__(self) -> str:
        return "Visual(offset={0}, geom_type='{1}', geom_param={2})".format(
            self.offset, self.geom_type, self.geom_param
        )


class Link:
    def __init__(
        self, name: Optional[str] = None, offset: Optional[transform.Transform] = None, visuals: Optional[List] = None
    ) -> None:
        self.name = name if name is not None else "none"
        self.offset = offset or transform.Transform()
        self.visuals = visuals or []

    def __repr__(self) -> str:
        return "Link(name='{0}', offset={1}, visuals={2})".format(self.name, self.offset, self.visuals)


class Joint:
    TYPES = ["fixed", "revolute", "prismatic"]

    def __init__(
        self,
        name: Optional[str] = None,
        offset: Optional[transform.Transform] = None,
        joint_type: str = "fixed",
        axis: Optional[List[float]] = None,
    ) -> None:
        self.name = name if name is not None else "none"
        self.offset = offset or transform.Transform()
        self.joint_type = joint_type
        if self.joint_type != "fixed" and axis is None:
            self.axis = np.array([0.0, 0.0, 1.0])
        else:
            self.axis = np.array(axis) if axis is not None else np.array([0.0, 0.0, 1.0])

    def __repr__(self) -> str:
        return "Joint(name='{0}', offset={1}, joint_type='{2}', axis={3})".format(
            self.name, self.offset, self.joint_type, self.axis
        )


class Frame:
    def __init__(
        self,
        name: Optional[str] = None,
        link: Optional[Link] = None,
        joint: Optional[Joint] = None,
        children: Optional[List["Frame"]] = None,
    ) -> None:
        self.name = "None" if name is None else name
        self.link = link or Link()
        self.joint = joint or Joint()
        self.children = children or []

    def _ptree(self, indent_width: int = 4) -> str:
        def _inner_ptree(root: Frame, parent: Frame, grandpa: Optional[Frame] = None, indent: str = ""):
            show_str = ""
            if parent.name != root.name:
                show_str += " " + parent.name + ("" if grandpa is None else "\n")
            if not parent.children:
                return show_str
            for child in parent.children[:-1]:
                show_str += indent + "├" + "─" * indent_width
                show_str += _inner_ptree(root, child, parent, indent + "│" + " " * (indent_width + 1))
            if parent.children:
                child = parent.children[-1]
                show_str += indent + "└" + "─" * indent_width
                show_str += _inner_ptree(root, child, parent, indent + " " * (indent_width + 2))
            return show_str

        show_str = self.name + "\n"
        show_str += _inner_ptree(self, self)
        return show_str

    def __str__(self) -> str:
        return self._ptree()

    def add_child(self, child: "Frame") -> None:
        self.children.append(child)

    def is_end(self) -> bool:
        return len(self.children) == 0

    def get_transform(self, theta: float = 0.0) -> transform.Transform:
        if self.joint.joint_type == "revolute":
            t = transform.Transform(tf.quaternion_about_axis(theta, self.joint.axis))
        elif self.joint.joint_type == "prismatic":
            t = transform.Transform(pos=theta * self.joint.axis)
        elif self.joint.joint_type == "fixed":
            t = transform.Transform()
        else:
            raise ValueError("Unsupported joint type %s." % self.joint.joint_type)
        return self.joint.offset * t

    def walk(self) -> Iterator["Frame"]:
        yield self
        for child in self.children:
            yield from child.walk()
