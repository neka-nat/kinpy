import numpy as np
import transformations as tf

class Transform(object):
    """This class calculates the rotation and translation of a 3D rigid body.

    Attributes
    ----------
    rot : np.ndarray
        The rotation parameter. Give in quaternions or roll pitch yaw.
    pos : np.ndarray
        The translation parameter.
    """
    def __init__(self, rot=[1.0, 0.0, 0.0, 0.0],
                 pos=np.zeros(3)):
        if rot is None:
            self.rot = np.array([1.0, 0.0, 0.0, 0.0])
        elif len(rot) == 3:
            self.rot = tf.quaternion_from_euler(*rot)
        elif len(rot) == 4:
            self.rot = np.array(rot)
        else:
            raise ValueError("Size of rot must be 3 or 4.")
        if pos is None:
            self.pos = np.zeros(3)
        else:
            self.pos = np.array(pos)

    def __repr__(self):
        return "Transform(rot={0}, pos={1})".format(self.rot, self.pos)

    @staticmethod
    def _rotation_vec(rot, vec):
        v4 = np.hstack([np.array([0.0]), vec])
        inv_rot = tf.quaternion_inverse(rot)
        ans = tf.quaternion_multiply(tf.quaternion_multiply(rot, v4), inv_rot)
        return ans[1:]

    def __mul__(self, other):
        rot = tf.quaternion_multiply(self.rot, other.rot)
        pos = self._rotation_vec(self.rot, other.pos) + self.pos
        return Transform(rot, pos)

    def inverse(self):
        rot = tf.quaternion_inverse(self.rot)
        pos = -self._rotation_vec(rot, self.pos)
        return Transform(rot, pos)

    def matrix(self):
        mat = tf.quaternion_matrix(self.rot)
        mat[:3, 3] = self.pos
        return mat
