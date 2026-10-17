"""Link offsets in FK must also enter world-expressed geometric Jacobians."""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

import kinpy as kp
from kinpy import frame, jacobian
from kinpy.chain import Chain, SerialChain

BASE = ([0.2, -0.3, 0.4], [0.1, -0.2, 0.3])
SPECS = [
    (
        "shoulder",
        "revolute",
        [0, 0, 1],
        ([0.1, 0.2, -0.1], [0.3, 0.1, -0.2]),
        ([-0.2, 0.3, 0.1], [0.4, -0.3, 0.2]),
    ),
    (
        "slide",
        "prismatic",
        [2, 0, 0],
        ([-0.3, 0.1, 0.2], [0.2, -0.1, 0.4]),
        ([0.4, -0.2, 0.3], [-0.1, 0.2, 0.3]),
    ),
    (
        "wrist",
        "revolute",
        [0, 1, 0],
        ([0.2, -0.1, 0.3], [0.1, 0.2, -0.3]),
        ([-0.3, 0.2, 0.4], [0.3, 0.2, 0.1]),
    ),
]


def _matrix(rotation, position):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler("xyz", rotation).as_matrix()
    out[:3, 3] = position
    return out


def _transform(spec):
    return kp.Transform(rot=spec[0], pos=spec[1])


def _chain(identity_links=False):
    root = frame.Frame(
        "base_frame",
        link=frame.Link("base"),
        joint=frame.Joint("base_joint", offset=_transform(BASE)),
    )
    previous = root
    for name, kind, axis, joint_offset, link_offset in SPECS:
        child = frame.Frame(
            name + "_frame",
            link=frame.Link(
                name, offset=None if identity_links else _transform(link_offset)
            ),
            joint=frame.Joint(
                name + "_joint",
                joint_type=kind,
                axis=axis,
                offset=_transform(joint_offset),
            ),
        )
        previous.children.append(child)
        previous = child
    return SerialChain(Chain(root), "wrist_frame")


def _oracle(q, link_name, tool_spec, identity_links=False):
    """Independent world screw columns, using scipy SO(3) rather than kinpy FK."""
    trans = _matrix(*BASE)
    axes, origins, kinds = [], [], []
    reached = 0
    for index, (name, kind, axis, joint_offset, link_offset) in enumerate(SPECS):
        before_motion = trans @ _matrix(*joint_offset)
        axis = np.asarray(axis, dtype=float)
        if kind == "revolute":
            axis = axis / np.linalg.norm(axis)
        axes.append(before_motion[:3, :3] @ axis)
        origins.append(before_motion[:3, 3].copy())
        kinds.append(kind)
        motion = np.eye(4)
        if kind == "revolute":
            motion[:3, :3] = Rotation.from_rotvec(q[index] * axis).as_matrix()
        else:
            motion[:3, 3] = q[index] * axis
        trans = before_motion @ motion
        if name == link_name:
            end = trans @ (np.eye(4) if identity_links else _matrix(*link_offset))
            end = end @ _matrix(*tool_spec)
            reached = index + 1
            break
    assert reached > 0
    expected = np.zeros((6, len(q)))
    for index in range(reached):
        if kinds[index] == "revolute":
            expected[:3, index] = np.cross(axes[index], end[:3, 3] - origins[index])
            expected[3:, index] = axes[index]
        else:
            expected[:3, index] = axes[index]
    return end, expected


class TestLinkOffsetJacobian(unittest.TestCase):
    def test_mjcf_fixed_tip_has_nonzero_position_derivative(self):
        xml = """<mujoco model="offset_tip"><worldbody>
        <body name="base"><body name="arm">
        <joint name="hinge" type="hinge" axis="0 0 1"/>
        <body name="tip" pos="1 0 0"/></body></body>
        </worldbody></mujoco>"""
        chain = kp.build_serial_chain_from_mjcf(xml, "tip")
        self.assertEqual(chain.get_joint_parameter_names(), ["hinge"])
        for angle in [0.0, np.pi / 2, -np.pi / 2, 0.3, -0.7, 1.2]:
            with self.subTest(angle=angle):
                c, s = np.cos(angle), np.sin(angle)
                np.testing.assert_allclose(
                    chain.forward_kinematics([angle]).pos,
                    [c, s, 0],
                    atol=2e-14,
                    rtol=2e-14,
                )
                expected = np.array([[-s], [c], [0], [0], [0], [1]])
                np.testing.assert_allclose(
                    chain.jacobian([angle]), expected, atol=2e-14, rtol=2e-14
                )
                np.testing.assert_allclose(
                    chain.jacobian([angle], end_only=False)["tip"],
                    expected,
                    atol=2e-14,
                    rtol=2e-14,
                )

    def test_mixed_joint_end_link_matches_independent_world_screws(self):
        chain = _chain()
        for q in [[0.2, -0.1, 0.4], [-0.6, 0.3, -0.2]]:
            for tool_spec in [
                ([0, 0, 0], [0, 0, 0]),
                ([0.3, -0.4, 0.2], [0.2, -0.1, 0.3]),
            ]:
                with self.subTest(q=q, tool=tool_spec):
                    tool = _transform(tool_spec)
                    pose, expected = _oracle(q, "wrist", tool_spec)
                    np.testing.assert_allclose(
                        (chain.forward_kinematics(q) * tool).matrix(),
                        pose,
                        atol=2e-14,
                        rtol=2e-14,
                    )
                    np.testing.assert_allclose(
                        jacobian.calc_jacobian(chain, q, tool),
                        expected,
                        atol=2e-14,
                        rtol=2e-14,
                    )

    def test_each_selected_link_includes_only_its_own_link_offset(self):
        chain = _chain()
        q = [0.2, -0.1, 0.4]
        for link in ["shoulder", "slide", "wrist"]:
            for tool_spec in [
                ([0, 0, 0], [0, 0, 0]),
                ([0.3, -0.4, 0.2], [0.2, -0.1, 0.3]),
            ]:
                with self.subTest(link=link, tool=tool_spec):
                    tool = _transform(tool_spec)
                    pose, expected = _oracle(q, link, tool_spec)
                    actual_pose = (
                        chain.forward_kinematics(q, end_only=False)[link] * tool
                    )
                    np.testing.assert_allclose(
                        actual_pose.matrix(), pose, atol=2e-14, rtol=2e-14
                    )
                    actual = jacobian.calc_jacobian_frames(chain, q, link, tool)
                    np.testing.assert_allclose(actual, expected, atol=2e-14, rtol=2e-14)
                    count = ["shoulder", "slide", "wrist"].index(link) + 1
                    np.testing.assert_array_equal(
                        actual[:, count:], np.zeros((6, 3 - count))
                    )

    def test_link_rotation_does_not_rotate_world_angular_velocity(self):
        q, tool_spec = [0.2, -0.1, 0.4], ([0, 0, 0], [0, 0, 0])
        offset_chain, identity_chain = _chain(), _chain(identity_links=True)
        actual = offset_chain.jacobian(q)
        _, expected = _oracle(q, "wrist", tool_spec)
        np.testing.assert_allclose(actual[3:], expected[3:], atol=2e-14, rtol=2e-14)
        np.testing.assert_allclose(
            actual[3:], identity_chain.jacobian(q)[3:], atol=2e-14, rtol=2e-14
        )

    def test_link_offset_and_explicit_tool_composition_are_equivalent(self):
        q = [0.2, -0.1, 0.4]
        offset_chain, identity_chain = _chain(), _chain(identity_links=True)
        tool_spec = ([0.3, -0.4, 0.2], [0.2, -0.1, 0.3])
        tool = _transform(tool_spec)
        folded = _transform(SPECS[-1][-1]) * tool
        actual = jacobian.calc_jacobian(offset_chain, q, tool)
        reference = jacobian.calc_jacobian(identity_chain, q, folded)
        _, expected = _oracle(q, "wrist", tool_spec)
        np.testing.assert_allclose(reference, expected, atol=2e-14, rtol=2e-14)
        np.testing.assert_allclose(actual, expected, atol=2e-14, rtol=2e-14)

    def test_identity_link_offsets_preserve_existing_geometric_columns(self):
        chain = _chain(identity_links=True)
        q, tool_spec = [0.2, -0.1, 0.4], ([0.3, -0.4, 0.2], [0.2, -0.1, 0.3])
        tool = _transform(tool_spec)
        _, expected = _oracle(q, "wrist", tool_spec, identity_links=True)
        np.testing.assert_allclose(
            jacobian.calc_jacobian(chain, q, tool), expected, atol=2e-14, rtol=2e-14
        )
        for link in ["shoulder", "slide", "wrist"]:
            _, expected = _oracle(q, link, tool_spec, identity_links=True)
            np.testing.assert_allclose(
                jacobian.calc_jacobian_frames(chain, q, link, tool),
                expected,
                atol=2e-14,
                rtol=2e-14,
            )


if __name__ == "__main__":
    unittest.main()
