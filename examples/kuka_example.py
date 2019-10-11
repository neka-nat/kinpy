import numpy as np
import kinpy as kp

chain = kp.build_serial_chain_from_urdf(open("kuka_iiwa/model.urdf").read(), "lbr_iiwa_link_7")
print(chain)
print(chain.get_joint_parameter_names())
th = [0.0, -np.pi / 4.0, 0.0, np.pi / 2.0, 0.0, np.pi / 4.0, 0.0]
ret = chain.forward_kinematics(th, end_only=False)
print(ret)
viz = kp.Visualizer()
viz.add_robot(ret, chain.visuals_map(), mesh_file_path='kuka_iiwa/', axes=True)
viz.spin()