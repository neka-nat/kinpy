from functools import cached_property
from typing import Dict, Iterator, List, Optional, Union

import numpy as np

from . import frame, ik, jacobian, transform


class Chain:
    """Chain is a class that represents a kinematic chain."""

    def __init__(self, root_frame: frame.Frame) -> None:
        self._root: Optional[frame.Frame] = root_frame

    def __str__(self) -> str:
        return str(self._root)

    def __iter__(self) -> Iterator[frame.Frame]:
        assert self._root is not None, "Root frame is None"
        yield from self._root.walk()

    @cached_property
    def dof(self):
        return len(self.get_joint_parameter_names())

    @staticmethod
    def _find_frame_recursive(name: str, frame: frame.Frame) -> Optional[frame.Frame]:
        for child in frame.children:
            if child.name == name:
                return child
            ret = Chain._find_frame_recursive(name, child)
            if ret is not None:
                return ret
        return None

    def find_frame(self, name: str) -> Optional[frame.Frame]:
        """Find a frame by name.

        Parameters
        ----------
        name : str
            Frame name.

        Returns
        -------
        Optional[frame.Frame]
            Frame if found, None otherwise.
        """
        assert self._root is not None, "Root frame is None"
        if self._root.name == name:
            return self._root
        return self._find_frame_recursive(name, self._root)

    @staticmethod
    def _find_link_recursive(name: str, frame: frame.Frame) -> Optional[frame.Link]:
        for child in frame.children:
            if child.link.name == name:
                return child.link
            ret = Chain._find_link_recursive(name, child)
            if ret is not None:
                return ret
        return None

    def find_link(self, name: str) -> Optional[frame.Link]:
        """Find a link by name.

        Parameters
        ----------
        name : str
            Link name.

        Returns
        -------
        Optional[frame.Link]
            Link if found, None otherwise.
        """
        assert self._root is not None, "Root frame is None"
        if self._root.link.name == name:
            return self._root.link
        return self._find_link_recursive(name, self._root)

    @staticmethod
    def _get_joint_parameter_names(frame: frame.Frame, exclude_fixed: bool = True) -> List[str]:
        joint_names = []
        if not (exclude_fixed and frame.joint.joint_type == "fixed"):
            joint_names.append(frame.joint.name)
        for child in frame.children:
            joint_names.extend(Chain._get_joint_parameter_names(child, exclude_fixed))
        return joint_names

    def get_joint_parameter_names(self, exclude_fixed: bool = True) -> List[str]:
        """Get joint parameter names.

        Parameters
        ----------
        exclude_fixed : bool, optional
            Exclude fixed joints, by default True

        Returns
        -------
        List[str]
            Joint parameter names.
        """
        assert self._root is not None, "Root frame is None"
        names = self._get_joint_parameter_names(self._root, exclude_fixed)
        return list(sorted(set(names), key=names.index))

    def add_frame(self, frame: frame.Frame, parent_name: str) -> None:
        parent_frame = self.find_frame(parent_name)
        if parent_frame is not None:
            parent_frame.add_child(frame)

    @staticmethod
    def _forward_kinematics(
        root: frame.Frame, th_dict: Dict[str, float], world: Optional[transform.Transform] = None
    ) -> Dict[str, transform.Transform]:
        world = world or transform.Transform()
        link_transforms = {}
        trans = world * root.get_transform(th_dict.get(root.joint.name, 0.0))
        link_transforms[root.link.name] = trans * root.link.offset
        for child in root.children:
            link_transforms.update(Chain._forward_kinematics(child, th_dict, trans))
        return link_transforms

    def forward_kinematics(
        self, th: Union[Dict[str, float], List[float]], world: Optional[transform.Transform] = None, **kwargs: Dict
    ) -> Dict[str, transform.Transform]:
        """Forward kinematics.

        Parameters
        ----------
        th : Union[Dict[str, float], List[float]]
            Joint parameters.
        world : Optional[transform.Transform], optional
            World transform, by default None

        Returns
        -------
        Dict[str, transform.Transform]
            Link transforms.
        """
        assert self._root is not None, "Root frame is None"
        world = world or transform.Transform()
        if not isinstance(th, dict):
            jn = self.get_joint_parameter_names()
            assert len(jn) == len(th)
            th_dict = dict((j, th[i]) for i, j in enumerate(jn))
        else:
            th_dict = th
        return self._forward_kinematics(self._root, th_dict, world)

    @staticmethod
    def _visuals_map(root: frame.Frame) -> Dict[str, List[frame.Visual]]:
        vmap = {root.link.name: root.link.visuals}
        for child in root.children:
            vmap.update(Chain._visuals_map(child))
        return vmap

    def visuals_map(self):
        return self._visuals_map(self._root)


class SerialChain(Chain):
    """SerialChain is a class that represents a serial kinematic chain."""

    def __init__(self, chain: Chain, end_frame_name: str, root_frame_name: str = "") -> None:
        assert chain._root is not None, "Chain root frame is None"
        if root_frame_name == "":
            self._root = chain._root
        else:
            self._root = chain.find_frame(root_frame_name)
            if self._root is None:
                raise ValueError("Invalid root frame name %s." % root_frame_name)
        frames = self._generate_serial_chain_recurse(self._root, end_frame_name)
        if frames is None:
            raise ValueError("Invalid end frame name %s." % end_frame_name)
        self._serial_frames = [self._root] + frames

    @staticmethod
    def _generate_serial_chain_recurse(root_frame: frame.Frame, end_frame_name: str) -> Optional[List[frame.Frame]]:
        for child in root_frame.children:
            if child.name == end_frame_name:
                return [child]
            else:
                frames = SerialChain._generate_serial_chain_recurse(child, end_frame_name)
                if frames is not None:
                    return [child] + frames
        return None

    def get_joint_parameter_names(self, exclude_fixed: bool = True) -> List[str]:
        assert self._serial_frames is not None, "Serial chain not initialized."
        names = []
        for f in self._serial_frames:
            if exclude_fixed and f.joint.joint_type == "fixed":
                continue
            names.append(f.joint.name)
        return names

    def forward_kinematics(  # type: ignore[override]
        self,
        th: Union[Dict[str, float], List[float]],
        world: Optional[transform.Transform] = None,
        end_only: bool = True,
    ) -> Union[transform.Transform, Dict[str, transform.Transform]]:
        assert self._serial_frames is not None, "Serial chain not initialized."
        if isinstance(th, dict):
            link_transforms = super().forward_kinematics(th, world)
            if end_only:
                return link_transforms[self._serial_frames[-1].link.name]
            else:
                return link_transforms
        world = world or transform.Transform()
        cnt = 0
        link_transforms = {}
        trans = world
        for f in self._serial_frames:
            if f.joint.joint_type != "fixed":
                trans = trans * f.get_transform(th[cnt])
            else:
                trans = trans * f.get_transform()
            link_transforms[f.link.name] = trans * f.link.offset
            if f.joint.joint_type != "fixed":
                cnt += 1
        return link_transforms[self._serial_frames[-1].link.name] if end_only else link_transforms

    def jacobian(self, th: List[float], end_only: bool = True) -> Union[np.ndarray, Dict[str, np.ndarray]]:
        assert self._serial_frames is not None, "Serial chain not initialized."
        if end_only:
            return jacobian.calc_jacobian(self, th)
        else:
            jacobians = {}
            for serial_frame in self._serial_frames:
                jac = jacobian.calc_jacobian_frames(self, th, link_name=serial_frame.link.name)
                jacobians[serial_frame.link.name] = jac
            return jacobians

    def inverse_kinematics(self, pose: transform.Transform, initial_state: Optional[np.ndarray] = None) -> np.ndarray:
        return ik.inverse_kinematics(self, pose, initial_state)
