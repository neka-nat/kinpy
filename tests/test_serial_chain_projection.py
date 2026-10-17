"""Serial chains expose one path without changing the source robot tree."""

import unittest

import numpy as np

import kinpy as kp
from kinpy.chain import SerialChain


URDF = """<robot name="branched">
<link name="base"/><link name="arm"/><link name="tip"/>
<link name="sibling"/><link name="finger"/>
<joint name="shoulder" type="revolute"><parent link="base"/>
<child link="arm"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint>
<joint name="slide" type="prismatic"><parent link="arm"/>
<child link="tip"/><origin xyz="1 0 0"/><axis xyz="1 0 0"/></joint>
<joint name="other" type="revolute"><parent link="base"/>
<child link="sibling"/><origin xyz="0 2 0"/><axis xyz="0 1 0"/></joint>
<joint name="finger_fixed" type="fixed"><parent link="tip"/>
<child link="finger"/><origin xyz="0 0 1"/></joint>
</robot>"""


class TestSerialChainProjection(unittest.TestCase):
    def test_explicit_root_fk_contains_only_selected_path(self):
        serial = kp.build_serial_chain_from_urdf(URDF, "tip", "base")
        vector = serial.forward_kinematics([0.4, 0.2], end_only=False)
        named = serial.forward_kinematics(
            {"shoulder": 0.4, "slide": 0.2, "other": 0.9}, end_only=False
        )
        self.assertEqual(list(vector), ["base", "arm", "tip"])
        self.assertEqual(list(named), list(vector))

    def test_default_root_fk_contains_only_selected_path(self):
        serial = kp.build_serial_chain_from_urdf(URDF, "tip")
        self.assertEqual(
            list(
                serial.forward_kinematics(
                    {"shoulder": 0.4, "slide": 0.2}, end_only=False
                )
            ),
            ["base", "arm", "tip"],
        )

    def test_tree_interfaces_contain_only_selected_path(self):
        serial = kp.build_serial_chain_from_urdf(URDF, "tip", "base")
        self.assertEqual([f.link.name for f in serial], ["base", "arm", "tip"])
        self.assertEqual(list(serial.visuals_map()), ["base", "arm", "tip"])
        self.assertIsNone(serial.find_frame("sibling_frame"))
        self.assertIsNone(serial.find_link("finger"))
        self.assertIsNotNone(serial.find_frame("arm_frame"))
        self.assertIsNotNone(serial.find_link("tip"))
        self.assertNotIn("sibling_frame", str(serial))
        self.assertNotIn("finger_frame", str(serial))
        self.assertEqual(serial.get_joint_parameter_names(), ["shoulder", "slide"])

    def test_selected_end_prunes_its_descendants(self):
        serial = kp.build_serial_chain_from_urdf(URDF, "arm", "base")
        self.assertEqual(
            list(serial.forward_kinematics([0.4], end_only=False)), ["base", "arm"]
        )
        self.assertEqual(
            list(serial.forward_kinematics({"shoulder": 0.4}, end_only=False)),
            ["base", "arm"],
        )
        self.assertEqual([f.link.name for f in serial], ["base", "arm"])
        self.assertIsNone(serial.find_link("tip"))

    def test_constructing_paths_preserves_original_tree(self):
        full = kp.build_chain_from_urdf(URDF)
        before = [(f, tuple(f.children)) for f in full]
        arm = SerialChain(full, "tip_frame")
        other = SerialChain(full, "sibling_frame")
        self.assertEqual([(f, tuple(f.children)) for f in full], before)
        self.assertEqual(arm.get_joint_parameter_names(), ["shoulder", "slide"])
        self.assertEqual(other.get_joint_parameter_names(), ["other"])
        self.assertEqual(
            set(full.forward_kinematics({"shoulder": 0.4, "slide": 0.2, "other": 0.9})),
            {"base", "arm", "tip", "finger", "sibling"},
        )

    def test_selected_poses_match_closed_form(self):
        phi = 0.3
        world = kp.Transform(rot=[0.0, 0.0, phi], pos=[0.2, -0.3, 0.7])
        for root in ("", "base", "arm"):
            serial = kp.build_serial_chain_from_urdf(URDF, "tip", root)
            for angle, slide in ((0.0, 0.0), (0.4, 0.2), (-0.8, 0.3)):
                c, s = np.cos(phi + angle), np.sin(phi + angle)
                position = [
                    0.2 + np.cos(phi) + (1 + slide) * c,
                    -0.3 + np.sin(phi) + (1 + slide) * s,
                    0.7,
                ]
                expected = np.array(
                    [
                        [c, -s, 0, position[0]],
                        [s, c, 0, position[1]],
                        [0, 0, 1, position[2]],
                        [0, 0, 0, 1],
                    ]
                )
                for values in ([angle, slide], {"shoulder": angle, "slide": slide}):
                    end = serial.forward_kinematics(values, world=world)
                    all_links = serial.forward_kinematics(
                        values, world=world, end_only=False
                    )
                    np.testing.assert_allclose(
                        end.matrix(), expected, rtol=0, atol=1e-12
                    )
                    np.testing.assert_allclose(
                        all_links["tip"].matrix(), expected, rtol=0, atol=1e-12
                    )
