import numpy as np
import kinpy as kp

chain = kp.build_chain_from_mjcf(open("ant/ant.xml").read())
print(chain)
print(chain.get_joint_parameter_names())
th = {'hip_1': 0.0, 'ankle_1': np.pi / 4.0,
      'hip_2': 0.0, 'ankle_2': -np.pi / 4.0,
      'hip_3': 0.0, 'ankle_3': -np.pi / 4.0,
      'hip_4': 0.0, 'ankle_4': np.pi / 4.0}
ret = chain.forward_kinematics(th)
print(ret)
viz = kp.Visualizer()
viz.add_robot(ret, chain.visuals_map())
viz.spin()