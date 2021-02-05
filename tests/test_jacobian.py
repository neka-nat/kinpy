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

    def test_jacobian3(self):
        chain = kp.build_serial_chain_from_urdf(open("examples/kuka_iiwa/model.urdf").read(), "lbr_iiwa_link_7")
        th = [0.0, -np.pi / 4.0, 0.0, np.pi / 2.0, 0.0, np.pi / 4.0, 0.0]
        jc = chain.jacobian(th)
        np.testing.assert_almost_equal(np.array([[0, 1.41421356e-02, 0, 2.82842712e-01, 0, 0, 0],
                                                 [-6.60827561e-01, 0, -4.57275649e-01, 0, 5.72756493e-02, 0, 0],
                                                 [0, 6.60827561e-01, 0, -3.63842712e-01, 0, 8.10000000e-02, 0],
                                                 [0, 0, -7.07106781e-01, 0, -7.07106781e-01, 0, -1],
                                                 [0, 1, 0, -1, 0, 1, 0],
                                                 [1, 0, 7.07106781e-01, 0, -7.07106781e-01, 0, 0]]), jc)

if __name__ == "__main__":
    unittest.main()