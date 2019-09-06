import numpy as np
from . import transform


def calc_jacobian(serial_chain, th, tool=transform.Transform()):
    ndof = len(th)
    j_fl = np.zeros((6, ndof))
    cur_transform = tool.matrix()

    cnt = 0
    for f in reversed(serial_chain._serial_frames):
        if f.joint.joint_type == "revolute":
            cnt += 1
            d = np.array([-cur_transform[0, 0] * cur_transform[1, 3]
                          + cur_transform[1, 0] * cur_transform[0, 3],
                          -cur_transform[0, 1] * cur_transform[1, 3]
                          + cur_transform[1, 1] * cur_transform[0, 3],
                          -cur_transform[0, 2] * cur_transform[1, 3]
                          + cur_transform[1, 2] * cur_transform[0, 3]])
            delta = cur_transform[2, 0:3]
            j_fl[:, -cnt] = np.hstack((d, delta))
        cur_frame_transform = f.get_transform(th[-cnt]).matrix()
        cur_transform = np.dot(cur_frame_transform, cur_transform)

    pose = serial_chain.forward_kinematics(th).matrix()
    rotation = pose[:3, :3]
    j_tr = np.zeros((6, 6))
    j_tr[:3, :3] = rotation
    j_tr[3:, 3:] = rotation
    j_w = np.dot(j_tr, j_fl)
    return j_w