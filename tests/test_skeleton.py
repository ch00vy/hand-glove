import unittest

import numpy as np

from motion_processing.csv_parser import Measurement, Recording
from motion_processing.skeleton import build_hierarchy


class SkeletonTests(unittest.TestCase):
    def test_chain_and_parent_conflict(self):
        recording = Recording()
        q = np.array([0, 0, 0, 1.0])
        recording.add(1, Measurement("joints_local", "root", "LTP_NED", "ORIENTATION", "FLOAT[4]", q))
        recording.add(1, Measurement("joints_local", "child", "root", "ORIENTATION", "FLOAT[4]", q))
        hierarchy = build_hierarchy(recording)
        self.assertEqual(hierarchy["child"]["parent"], "root")
        self.assertTrue(hierarchy["child"]["consistent"])
        recording.add(2, Measurement("joints_local", "child", "other", "ORIENTATION", "FLOAT[4]", q))
        self.assertFalse(build_hierarchy(recording)["child"]["consistent"])

    def test_cycle_flagged(self):
        recording = Recording()
        q = np.array([0, 0, 0, 1.0])
        recording.add(1, Measurement("joints_local", "a", "b", "ORIENTATION", "FLOAT[4]", q))
        recording.add(1, Measurement("joints_local", "b", "a", "ORIENTATION", "FLOAT[4]", q))
        hierarchy = build_hierarchy(recording)
        self.assertFalse(hierarchy["a"]["consistent"])
        self.assertFalse(hierarchy["b"]["consistent"])


if __name__ == "__main__":
    unittest.main()
