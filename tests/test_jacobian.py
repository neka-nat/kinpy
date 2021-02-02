import unittest
import numpy as np
import kinpy as kp


class TestJacobian(unittest.TestCase):
    def test_jacobian1(self):
        data = '<robot name="test_robot">'\
        '<link name="link1" />'\
        '<link name="link2" />'\
        '<link name="link3" />'\
        '<joint name="joint1" type="revolute">'\
        '<origin xyz="0.0 0.0 0.0"/>'\
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
        jc = chain.jacobian([0.0, 0.0])
        np.testing.assert_equal(np.array([[0.0, 0.0],
                                          [1.0, 0.0],
                                          [0.0, 0.0],
                                          [0.0, 0.0],
                                          [0.0, 0.0],
                                          [1.0, 1.0]]), jc)

    def test_jacobian2(self):
        data = '<robot name="test_robot">'\
        '<link name="link1" />'\
        '<link name="link2" />'\
        '<link name="link3" />'\
        '<joint name="joint1" type="revolute">'\
        '<origin xyz="0.0 0.0 0.0"/>'\
        '<parent link="link1"/>'\
        '<child link="link2"/>'\
        '</joint>'\
        '<joint name="joint2" type="prismatic">'\
        '<origin xyz="1.0 0.0 0.0"/>'\
        '<parent link="link2"/>'\
        '<child link="link3"/>'\
        '</joint>'\
        '</robot>'
        chain = kp.build_serial_chain_from_urdf(data, 'link3')
        jc = chain.jacobian([0.0, 0.0])
        np.testing.assert_equal(np.array([[0.0, 0.0],
                                          [1.0, 0.0],
                                          [0.0, 1.0],
                                          [0.0, 0.0],
                                          [0.0, 0.0],
                                          [1.0, 0.0]]), jc)
        


if __name__ == "__main__":
    unittest.main()