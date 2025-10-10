import numpy as np
import kinpy as kp


chain = kp.build_serial_chain_from_mjcf(
    open("SO101/so101_new_calib.xml", "rb").read(),
    "gripper",
    model_dir="SO101/",
)
print(chain)

print(chain.get_joint_parameter_names())
th = np.deg2rad([0, 0, 0, 0, 0])
viz = kp.JointAngleEditor(chain, "SO101/assets", initial_state=th, axes=True)
viz.spin()
