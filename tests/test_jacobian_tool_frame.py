"""Tool orientation must not change a world-expressed geometric Jacobian."""

import unittest

import numpy as np

import kinpy as kp
from kinpy import jacobian


ROBOT = """
<robot name="mixed_chain">
  <link name="base"/><link name="shoulder"/><link name="slide"/>
  <link name="wrist"/><link name="tip"/>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base"/><child link="shoulder"/>
    <origin xyz="0.1 -0.2 0.3" rpy="0.2 -0.1 0.4"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="slide_joint" type="prismatic">
    <parent link="shoulder"/><child link="slide"/>
    <origin xyz="0.4 0.1 -0.2" rpy="-0.3 0.2 0.1"/>
    <axis xyz="1 0 0"/>
  </joint>
  <joint name="wrist_joint" type="revolute">
    <parent link="slide"/><child link="wrist"/>
    <origin xyz="0.2 -0.3 0.1" rpy="0.1 0.3 -0.2"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="tip_joint" type="fixed">
    <parent link="wrist"/><child link="tip"/>
    <origin xyz="0.2 0.1 0.3" rpy="-0.2 0.3 0.4"/>
  </joint>
</robot>
"""


class TestToolFrameJacobian(unittest.TestCase):
    """Check both linear and angular blocks against independent FK derivatives."""

    def setUp(self):
        self.chain = kp.build_serial_chain_from_urdf(ROBOT, "tip")
        self.configurations = ([0.2, 0.3, -0.4], [-0.6, -0.2, 0.7])
        self.rotations = ([0.3, -0.4, 0.7], [-0.5, 0.2, -0.3])
        self.positions = (np.zeros(3), np.array([0.2, -0.1, 0.4]))

    def _pose(self, q, tool, link_name):
        links = self.chain.forward_kinematics(q, end_only=False)
        return (links[link_name] * tool).matrix()

    def _finite_difference(self, q, tool, link_name):
        q = np.asarray(q, dtype=float)
        nominal = self._pose(q, tool, link_name)
        result = np.zeros((6, len(q)))
        step = 1e-6
        for index in range(len(q)):
            positive = q.copy()
            negative = q.copy()
            positive[index] += step
            negative[index] -= step
            derivative = (
                self._pose(positive, tool, link_name)
                - self._pose(negative, tool, link_name)
            ) / (2 * step)
            result[:3, index] = derivative[:3, 3]
            omega = derivative[:3, :3] @ nominal[:3, :3].T
            result[3:, index] = [omega[2, 1], omega[0, 2], omega[1, 0]]
        return result

    def test_end_link_matches_full_pose_finite_difference(self):
        for q in self.configurations:
            for rotation in self.rotations:
                for position in self.positions:
                    with self.subTest(q=q, rotation=rotation, position=position):
                        tool = kp.Transform(rot=rotation, pos=position)
                        original_rotation = tool.rot.copy()
                        original_position = tool.pos.copy()
                        actual = jacobian.calc_jacobian(self.chain, q, tool)
                        # Some transformations versions normalize at roundoff precision.
                        np.testing.assert_allclose(
                            tool.rot, original_rotation, atol=1e-15, rtol=1e-15
                        )
                        np.testing.assert_array_equal(tool.pos, original_position)
                        expected = self._finite_difference(q, tool, "tip")
                        np.testing.assert_allclose(actual, expected, atol=2e-8, rtol=2e-8)

    def test_intermediate_link_matches_full_pose_finite_difference(self):
        for link_name in ("shoulder", "slide", "wrist", "tip"):
            for q in self.configurations:
                for rotation in self.rotations:
                    with self.subTest(link=link_name, q=q, rotation=rotation):
                        tool = kp.Transform(rot=rotation, pos=self.positions[1])
                        actual = jacobian.calc_jacobian_frames(
                            self.chain, q, link_name, tool
                        )
                        expected = self._finite_difference(q, tool, link_name)
                        np.testing.assert_allclose(actual, expected, atol=2e-8, rtol=2e-8)

    def test_world_geometric_jacobian_is_independent_of_tool_axes(self):
        for q in self.configurations:
            for position in self.positions:
                reference = kp.Transform(pos=position)
                for rotation in self.rotations:
                    tool = kp.Transform(rot=rotation, pos=position)
                    with self.subTest(q=q, position=position, rotation=rotation):
                        np.testing.assert_allclose(
                            jacobian.calc_jacobian(self.chain, q, tool),
                            jacobian.calc_jacobian(self.chain, q, reference),
                            atol=1e-14, rtol=1e-14,
                        )
                        for link_name in ("shoulder", "slide", "wrist", "tip"):
                            np.testing.assert_allclose(
                                jacobian.calc_jacobian_frames(
                                    self.chain, q, link_name, tool
                                ),
                                jacobian.calc_jacobian_frames(
                                    self.chain, q, link_name, reference
                                ),
                                atol=1e-14, rtol=1e-14,
                            )


if __name__ == "__main__":
    unittest.main()
