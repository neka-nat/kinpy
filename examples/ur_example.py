import numpy as np
import kinpy as kp


arm = kp.build_serial_chain_from_urdf(
    open("ur/ur.urdf").read(),
    root_link_name="base_link",
    end_link_name="ee_link",
)
fk_solution = arm.forward_kinematics(np.zeros(len(arm.get_joint_parameter_names())))
print(fk_solution)
