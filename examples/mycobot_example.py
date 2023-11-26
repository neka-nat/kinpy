import numpy as np
import kinpy as kp

chain = kp.build_serial_chain_from_urdf(open("mycobot/mycobot.urdf").read(), "pump_head")
print(chain)

print(chain.get_joint_parameter_names())
th = np.deg2rad([0, 20, -130, 20, 0, 0])
viz = kp.JointAngleEditor(chain, "mycobot/", initial_state=th, axes=True)
viz.spin()
