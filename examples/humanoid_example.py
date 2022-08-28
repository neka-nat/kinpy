import numpy as np
import kinpy as kp

chain = kp.build_chain_from_mjcf(open("humanoid/humanoid.xml").read())
print(chain)
print(chain.get_joint_parameter_names())
th = {"left_knee": 0.0, "right_knee": 0.0}
ret = chain.forward_kinematics(th)
print(ret)
viz = kp.Visualizer()
viz.add_robot(ret, chain.visuals_map(), axes=True)
viz.spin()
