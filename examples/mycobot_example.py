import numpy as np
import kinpy as kp

chain = kp.build_serial_chain_from_urdf(open("mycobot/mycobot.urdf").read(), "joint6_flange")
print(chain)

print(chain.get_joint_parameter_names())
th = np.deg2rad([0, 20, -130, 20, 0, 0])
viz = kp.JointAngleEditor(chain, "mycobot/", initial_state=th)
viz.spin()
