"""An omitted URDF <axis> must default to (1, 0, 0), as in urdfdom."""
import unittest

import numpy as np

import kinpy as kp

URDF_TEMPLATE = """<robot name="default_axis">
<link name="base_link"/><link name="tip"/>
<joint name="slide" type="{joint_type}">
  <parent link="base_link"/><child link="tip"/>
  {axis}
  <limit lower="-2" upper="2" effort="1" velocity="1"/>
</joint></robot>"""


def _tip_pose(joint_type, axis, q):
    chain = kp.build_chain_from_urdf(
        URDF_TEMPLATE.format(joint_type=joint_type, axis=axis)
    )
    return chain.forward_kinematics({"slide": q})["tip"]


class URDFDefaultAxisTest(unittest.TestCase):
    def test_omitted_axis_defaults_to_x(self):
        chain = kp.build_chain_from_urdf(
            URDF_TEMPLATE.format(joint_type="revolute", axis="")
        )
        np.testing.assert_allclose(
            chain.find_frame("tip_frame").joint.axis, [1.0, 0.0, 0.0]
        )

    def test_prismatic_omitted_axis_matches_explicit_x(self):
        omitted = _tip_pose("prismatic", "", 0.7)
        explicit_x = _tip_pose("prismatic", '<axis xyz="1 0 0"/>', 0.7)
        np.testing.assert_allclose(omitted.pos, [0.7, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(omitted.pos, explicit_x.pos, atol=1e-12)

    def test_revolute_omitted_axis_matches_explicit_x(self):
        omitted = _tip_pose("revolute", "", 0.5)
        explicit_x = _tip_pose("revolute", '<axis xyz="1 0 0"/>', 0.5)
        np.testing.assert_allclose(omitted.rot, explicit_x.rot, atol=1e-12)

    def test_explicit_z_axis_still_honoured(self):
        explicit_z = _tip_pose("prismatic", '<axis xyz="0 0 1"/>', 0.7)
        np.testing.assert_allclose(explicit_z.pos, [0.0, 0.0, 0.7], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
