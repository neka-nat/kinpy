import numpy as np
import kinpy as kp

chain = kp.build_chain_from_sdf(open("simple_arm/model.sdf").read())
print(chain)
print(chain.get_joint_parameter_names())
ret = chain.forward_kinematics({})
print(ret)
viz = kp.Visualizer()
viz.add_robot(ret, chain.visuals_map(), axes=True)
viz.spin()