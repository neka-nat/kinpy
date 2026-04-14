# SO101 Robot - URDF and MuJoCo Description

This repository contains the URDF and MuJoCo (MJCF) files for the SO101 robot.

## Overview

- The robot model files were generated using the [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) plugin from a CAD model designed in Onshape.
- The generated URDFs were modified to allow meshes with relative paths instead of `package://...`.
- Base collision meshes were removed due to problematic collision behavior during simulation and planning.

## Calibration Methods

The MuJoCo file `scene.xml` supports two differenly calibrated SO101 robot files:

- **New Calibration (Default)**: Each joint's virtual zero is set to the **middle** of its joint range. Use -> `so101_new_calib.xml`. 
- **Old Calibration**: Each joint's virtual zero is set to the configuration where the robot is **fully extended horizontally**. Use -> `so101_old_calib.xml`.

To switch between calibration methods, modify the included robot file in `scene.xml`.

## Motor Parameters

Motor properties for the STS3215 motors used in the robot are adapted from the [Open Duck Mini project](https://github.com/apirrone/Open_Duck_Mini).

## Gripper Note

In LeRobot, the gripper is represented as a **linear joint**, where:

* `0` = fully closed
* `100` = fully open

This mapping is **not yet reflected** in the current URDF and MuJoCo files. 

## Wrist Camera

The upstream SO-100/SO-101 hardware repository includes several official wrist-camera options:

- SO-101-specific 32x32 UVC hex-nut adapter
- SO-100/SO-101 integrated 32x32 UVC wrist-roll replacement
- Intel RealSense D405 and D435/D435i mounts
- Vinmooog webcam mount

The MJCF files in this example now include a fixed wrist camera under the `gripper` body and use the official separated SO-101 hex-nut wrist-camera adapter for visualization while keeping the stock wrist-roll follower:

- camera name: `wrist_camera`
- frame site: `wrist_camera_frame`
- adapter body: `wrist_camera_adapter`

The adapter mesh comes from the official upstream file:

- `Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module/stl/SO-ARM101_camera_wrist_mount.stl`

The local copy of that STL is normalized from millimeters to meters so it matches the unit convention used by the rest of the SO101 assets in this repository.

The adapter pose is derived by aligning that STL to the stock `wrist_roll_follower_so101_v1` frame and checking it against the upstream installation photos. The fixed `wrist_camera_mount` body is still the FK target for the camera lens and keeps the joint tree unchanged.

If you want to represent one of the other upstream mounts more faithfully, the cleanest MJCF strategy is still to keep the camera fixed under `gripper`, then swap or extend the visual mesh around `wrist_roll_follower_so101_v1`.

---

Feel free to open an issue or contribute improvements!
