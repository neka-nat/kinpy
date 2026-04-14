import unittest

import kinpy as kp


SIMPLE_MJCF = b"""
<mujoco model="test">
  <worldbody>
    <body name="base" pos="0 0 0" quat="1 0 0 0">
      <body name="link1" pos="0 0 0" quat="1 0 0 0">
        <geom name="link1_box" type="box" size="0.1 0.2 0.3"/>
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <body name="link2" pos="0 0 0" quat="1 0 0 0">
          <geom name="link2_cylinder" type="cylinder" size="0.05 0.4"/>
          <joint name="j2" type="hinge" axis="0 0 1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class TestMjcf(unittest.TestCase):
    def test_primitive_geoms_are_converted_to_visuals(self):
        mjcf_chain = kp.build_chain_from_mjcf(SIMPLE_MJCF)
        visuals_map = mjcf_chain.visuals_map()

        self.assertEqual("box", visuals_map["link1_child"][0].geom_type)
        self.assertEqual([0.2, 0.4, 0.6], list(visuals_map["link1_child"][0].geom_param))

        self.assertEqual("cylinder", visuals_map["link2_child"][0].geom_type)
        self.assertEqual((0.05, 0.8), tuple(visuals_map["link2_child"][0].geom_param))

    def test_serial_chain_reaches_target_body_joint(self):
        link1_chain = kp.build_serial_chain_from_mjcf(SIMPLE_MJCF, "link1")
        self.assertEqual(["j1"], link1_chain.get_joint_parameter_names())

        link2_chain = kp.build_serial_chain_from_mjcf(SIMPLE_MJCF, "link2")
        self.assertEqual(["j1", "j2"], link2_chain.get_joint_parameter_names())

    def test_so101_serial_chain_joint_counts(self):
        with open("examples/SO101/so101_new_calib.xml", "rb") as f:
            data = f.read()

        gripper_chain = kp.build_serial_chain_from_mjcf(data, "gripper", model_dir="examples/SO101/")
        self.assertEqual(
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            gripper_chain.get_joint_parameter_names(),
        )

        wrist_camera_chain = kp.build_serial_chain_from_mjcf(data, "wrist_camera_mount", model_dir="examples/SO101/")
        self.assertEqual(
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            wrist_camera_chain.get_joint_parameter_names(),
        )

        wrist_camera_adapter_chain = kp.build_serial_chain_from_mjcf(
            data, "wrist_camera_adapter", model_dir="examples/SO101/"
        )
        self.assertEqual(
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            wrist_camera_adapter_chain.get_joint_parameter_names(),
        )

        jaw_chain = kp.build_serial_chain_from_mjcf(data, "moving_jaw_so101_v1", model_dir="examples/SO101/")
        self.assertEqual(
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
            jaw_chain.get_joint_parameter_names(),
        )


if __name__ == "__main__":
    unittest.main()
