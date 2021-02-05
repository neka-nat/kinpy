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
            delta = np.dot(f.joint.axis, cur_transform[:3, :3])
            d = np.dot(np.cross(f.joint.axis, cur_transform[:3, 3]), cur_transform[:3, :3])
            j_fl[:, -cnt] = np.hstack((d, delta))
        elif f.joint.joint_type == "prismatic":
            cnt += 1
            j_fl[:3, -cnt] = np.dot(f.joint.axis, cur_transform[:3, :3])
        cur_frame_transform = f.get_transform(th[-cnt]).matrix()
        cur_transform = np.dot(cur_frame_transform, cur_transform)

    pose = serial_chain.forward_kinematics(th).matrix()
    rotation = pose[:3, :3]
    j_tr = np.zeros((6, 6))
    j_tr[:3, :3] = rotation
    j_tr[3:, 3:] = rotation
    j_w = np.dot(j_tr, j_fl)
    return j_w