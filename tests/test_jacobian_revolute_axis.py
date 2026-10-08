"""Revolute axis magnitudes cannot rescale physical FK derivatives."""

import unittest

import numpy as np

import kinpy as kp


class TestRevoluteAxisJacobian(unittest.TestCase):
    """Compare both Jacobian APIs to physical point/orientation derivatives."""

    def _chain(self, first_axis, second_axis):
        xml = f"""
        <robot name="scaled_axes">
          <link name="base"/><link name="first"/><link name="second"/>
          <link name="slide"/><link name="tip"/>
          <joint name="first_joint" type="revolute">
            <parent link="base"/><child link="first"/>
            <origin xyz="0.1 -0.2 0.3" rpy="0.2 -0.1 0.4"/>
            <axis xyz="{first_axis}"/>
          </joint>
          <joint name="second_joint" type="revolute">
            <parent link="first"/><child link="second"/>
            <origin xyz="0.4 0.1 -0.2" rpy="-0.3 0.2 0.1"/>
            <axis xyz="{second_axis}"/>
          </joint>
          <joint name="slide_joint" type="prismatic">
            <parent link="second"/><child link="slide"/>
            <origin xyz="0.2 -0.3 0.1"/>
            <axis xyz="2 0 0"/>
          </joint>
          <joint name="tip_joint" type="fixed">
            <parent link="slide"/><child link="tip"/>
            <origin xyz="0.2 0.1 0.3" rpy="-0.2 0.3 0.4"/>
          </joint>
        </robot>
        """
        return kp.build_serial_chain_from_urdf(xml, "tip")

    def _finite_difference(self, chain, q, link_name):
        q = np.array(q, dtype=float)
        nominal = chain.forward_kinematics(q, end_only=False)[link_name].matrix()
        result = np.zeros((6, len(q)))
        step = 1e-6
        for index in range(len(q)):
            positive = q.copy()
            negative = q.copy()
            positive[index] += step
            negative[index] -= step
            plus = chain.forward_kinematics(positive, end_only=False)[link_name].matrix()
            minus = chain.forward_kinematics(negative, end_only=False)[link_name].matrix()
            derivative = (plus - minus) / (2 * step)
            result[:3, index] = derivative[:3, 3]
            omega = derivative[:3, :3] @ nominal[:3, :3].T
            result[3:, index] = [omega[2, 1], omega[0, 2], omega[1, 0]]
        return result

    def test_nonunit_revolute_axes_match_full_fk_derivative(self):
        for axes in (("0 0 3", "0 -5 0"), ("2 3 4", "-1 4 2")):
            for q in ((0.2, -0.4, 0.3), (-0.6, 0.7, -0.2)):
                with self.subTest(axes=axes, q=q):
                    chain = self._chain(*axes)
                    expected = self._finite_difference(chain, q, "tip")
                    np.testing.assert_allclose(chain.jacobian(q), expected, atol=2e-8, rtol=2e-8)

    def test_every_link_jacobian_matches_full_fk_derivative(self):
        for axes in (("0 0 3", "0 -5 0"), ("2 3 4", "-1 4 2")):
            chain = self._chain(*axes)
            q = [0.2, -0.4, 0.3]
            jacobians = chain.jacobian(q, end_only=False)
            for link in ("first", "second", "slide", "tip"):
                with self.subTest(axes=axes, link=link):
                    expected = self._finite_difference(chain, q, link)
                    np.testing.assert_allclose(jacobians[link], expected, atol=2e-8, rtol=2e-8)

    def test_positive_axis_scaling_does_not_change_revolute_fk_or_jacobian(self):
        reference = self._chain("0 0 1", "0 -1 0")
        for scale in (0.1, 3.0, 7.0):
            with self.subTest(scale=scale):
                scaled = self._chain(f"0 0 {scale}", f"0 {-scale} 0")
                q = [0.2, -0.4, 0.3]
                np.testing.assert_allclose(
                    scaled.forward_kinematics(q).matrix(),
                    reference.forward_kinematics(q).matrix(),
                    atol=1e-14, rtol=1e-14,
                )
                np.testing.assert_allclose(
                    scaled.jacobian(q), reference.jacobian(q), atol=1e-14, rtol=1e-14,
                )

    def test_zero_revolute_axes_match_full_fk_derivative(self):
        for axes in (("0 0 0", "0 -5 0"), ("0 0 3", "0 0 0"), ("0 0 0", "0 0 0")):
            chain = self._chain(*axes)
            for q in ((0.2, -0.4, 0.3), (-0.6, 0.7, -0.2)):
                with self.subTest(axes=axes, q=q):
                    expected = self._finite_difference(chain, q, "tip")
                    np.testing.assert_allclose(chain.jacobian(q), expected, atol=2e-8, rtol=2e-8)
                    jacobians = chain.jacobian(q, end_only=False)
                    for link in ("first", "second", "slide", "tip"):
                        with self.subTest(link=link):
                            expected = self._finite_difference(chain, q, link)
                            np.testing.assert_allclose(jacobians[link], expected, atol=2e-8, rtol=2e-8)


if __name__ == "__main__":
    unittest.main()
