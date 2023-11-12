from typing import List, Optional, Union

import numpy as np
import transformations as tf


class Transform:
    """This class calculates the rotation and translation of a 3D rigid body.

    Attributes
    ----------
    rot : np.ndarray
        The rotation parameter. Give in quaternions or roll pitch yaw.
    pos : np.ndarray
        The translation parameter.
    """

    def __init__(self, rot: Union[List, np.ndarray, None] = None, pos: Optional[np.ndarray] = None) -> None:
        if rot is None:
            rot = [1.0, 0.0, 0.0, 0.0]
        if pos is None:
            pos = np.zeros(3)
        if len(rot) == 3:
            self.rot = tf.quaternion_from_euler(*rot)
        elif len(rot) == 4:
            self.rot = np.array(rot)
        else:
            raise ValueError("Size of rot must be 3 or 4.")
        self.pos = np.array(pos)

    def __repr__(self) -> str:
        return "Transform(rot={0}, pos={1})".format(self.rot, self.pos)

    @staticmethod
    def _rotation_vec(rot: np.ndarray, vec: np.ndarray) -> np.ndarray:
        v4 = np.hstack([np.array([0.0]), vec])
        inv_rot = tf.quaternion_inverse(rot)
        ans = tf.quaternion_multiply(tf.quaternion_multiply(rot, v4), inv_rot)
        return ans[1:]

    def __mul__(self, other: "Transform") -> "Transform":
        rot = tf.quaternion_multiply(self.rot, other.rot)
        pos = self._rotation_vec(self.rot, other.pos) + self.pos
        return Transform(rot, pos)

    def inverse(self) -> "Transform":
        rot = tf.quaternion_inverse(self.rot)
        pos = -self._rotation_vec(rot, self.pos)
        return Transform(rot, pos)

    def matrix(self) -> np.ndarray:
        mat = tf.quaternion_matrix(self.rot)
        mat[:3, 3] = self.pos
        return mat

    @property
    def rot_mat(self) -> np.ndarray:
        return tf.quaternion_matrix(self.rot)[:3, :3]

    @property
    def rot_euler(self) -> np.ndarray:
        return tf.euler_from_quaternion(self.rot)
