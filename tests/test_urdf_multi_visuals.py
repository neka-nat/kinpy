"""URDF loading must preserve every <visual> element on a link."""
import unittest

import numpy as np

import kinpy as kp


MULTI_VISUAL_URDF = """<robot name="multi_visual_test">
  <link name="base_link">
    <visual>
      <geometry><box size="1 1 1"/></geometry>
    </visual>
  </link>
  <link name="palm">
    <visual>
      <origin xyz="0.1 0 0"/>
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </visual>
    <visual>
      <origin xyz="-0.1 0 0"/>
      <geometry><sphere radius="0.1"/></geometry>
    </visual>
    <visual>
      <origin xyz="0 0.2 0"/>
      <geometry><cylinder radius="0.05" length="0.1"/></geometry>
    </visual>
  </link>
  <link name="tip">
    <visual>
      <geometry><sphere radius="0.02"/></geometry>
    </visual>
  </link>
  <joint name="base_to_palm" type="fixed">
    <parent link="base_link"/><child link="palm"/>
  </joint>
  <joint name="palm_to_tip" type="revolute">
    <parent link="palm"/><child link="tip"/>
    <origin xyz="0 0.1 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""

SINGLE_VISUAL_URDF = """<robot name="single_visual_test">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry><box size="1 1 1"/></geometry>
    </visual>
  </link>
  <link name="tool">
    <visual>
      <origin xyz="0 0 0.1"/>
      <geometry><cylinder radius="0.05" length="0.2"/></geometry>
    </visual>
  </link>
  <joint name="base_to_tool" type="fixed">
    <parent link="base_link"/><child link="tool"/>
  </joint>
</robot>
"""

NO_VISUAL_URDF = """<robot name="no_visual_test">
  <link name="base_link"/>
  <link name="tool"/>
  <joint name="base_to_tool" type="fixed">
    <parent link="base_link"/><child link="tool"/>
  </joint>
</robot>
"""


class MultiVisualURDFTest(unittest.TestCase):
    def test_three_visuals_on_one_link_all_retained(self):
        chain = kp.build_chain_from_urdf(MULTI_VISUAL_URDF)
        palm = chain.find_link("palm")
        self.assertIsNotNone(palm)
        self.assertEqual(len(palm.visuals), 3)

    def test_three_visuals_distinct_offsets_and_geometries(self):
        chain = kp.build_chain_from_urdf(MULTI_VISUAL_URDF)
        palm = chain.find_link("palm")
        geom_types = sorted(v.geom_type for v in palm.visuals)
        self.assertEqual(geom_types, ["box", "cylinder", "sphere"])
        positions = sorted(tuple(np.round(v.offset.pos, 6)) for v in palm.visuals)
        self.assertEqual(
            positions,
            sorted([(-0.1, 0.0, 0.0), (0.0, 0.2, 0.0), (0.1, 0.0, 0.0)]),
        )

    def test_visuals_map_reports_full_list_per_link(self):
        chain = kp.build_chain_from_urdf(MULTI_VISUAL_URDF)
        vmap = chain.visuals_map()
        self.assertEqual(len(vmap["palm"]), 3)
        self.assertEqual(len(vmap["base_link"]), 1)
        self.assertEqual(len(vmap["tip"]), 1)

    def test_single_visual_still_returns_one_entry(self):
        chain = kp.build_chain_from_urdf(SINGLE_VISUAL_URDF)
        base = chain.find_link("base_link")
        tool = chain.find_link("tool")
        self.assertEqual(len(base.visuals), 1)
        self.assertEqual(len(tool.visuals), 1)
        self.assertEqual(base.visuals[0].geom_type, "box")
        self.assertEqual(tool.visuals[0].geom_type, "cylinder")

    def test_links_without_any_visual_preserve_prior_placeholder(self):
        # Keep the pre-existing behavior of exposing a single empty Visual placeholder
        # when the URDF declares no <visual> for a link.
        chain = kp.build_chain_from_urdf(NO_VISUAL_URDF)
        base = chain.find_link("base_link")
        tool = chain.find_link("tool")
        self.assertEqual(len(base.visuals), 1)
        self.assertEqual(len(tool.visuals), 1)
        self.assertIsNone(base.visuals[0].geom_type)
        self.assertIsNone(tool.visuals[0].geom_type)


if __name__ == "__main__":
    unittest.main()
