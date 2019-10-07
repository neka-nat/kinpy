import unittest
import numpy as np
import kinpy as kp


def random_transform():
    euler = np.random.rand(3) * 2.0 * np.pi - np.pi
    return kp.Transform(euler, np.random.rand(3))


class TestTransform(unittest.TestCase):
    def test_multiply(self):
        t = random_transform()
        res = t * t.inverse()
        self.assertTrue(np.allclose(res.rot, np.array([1.0, 0.0, 0.0, 0.0]), atol=1.0e-6))
        self.assertTrue(np.allclose(res.pos, np.array([0.0, 0.0, 0.0]), atol=1.0e-6))


if __name__ == "__main__":
    unittest.main()