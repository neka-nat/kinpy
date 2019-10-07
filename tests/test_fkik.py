import unittest
import numpy as np
import kinpy as kp


class TestFkIk(unittest.TestCase):
    def test_fkik(self):
        data = '<robot name="test_robot">'\
        '<link name="link1" />'\
        '<link name="link2" />'\
        '<link name="link3" />'\
        '<joint name="joint1" type="revolute">'\
        '<origin xyz="1.0 0.0 0.0"/>'\
        '<parent link="link1"/>'\
        '<child link="link2"/>'\
        '</joint>'\
        '<joint name="joint2" type="revolute">'\
        '<origin xyz="1.0 0.0 0.0"/>'\
        '<parent link="link2"/>'\
        '<child link="link3"/>'\
        '</joint>'\
        '</robot>'
        chain = kp.build_serial_chain_from_urdf(data, 'link3')
        th1 = np.random.rand(2)
        tg = chain.forward_kinematics(th1)
        th2 = chain.inverse_kinematics(tg)
        self.assertTrue(np.allclose(th1, th2, atol=1.0e-6))
        

if __name__ == "__main__":
    unittest.main()