import unittest
import numpy as np

from Controller import CircularTraceController


class MyTestCase(unittest.TestCase):
    def test_speed_round(self):
        controller = CircularTraceController()
        rounded_speed = controller.speed_round(np.array([12, 16]))
        self.assertTrue(np.allclose(rounded_speed, np.array([6, 8])))
        self.assertTrue(np.allclose(controller.speed_round(np.array([1, -1])), np.array([1, -1])))


if __name__ == '__main__':
    unittest.main()
