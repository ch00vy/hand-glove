import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from motion_processing.csv_parser import Measurement, Recording
from motion_processing.joint_angles import angle_rows, hinge_angle


class JointAngleTests(unittest.TestCase):
    def test_known_hinge(self):
        rotation = Rotation.from_rotvec([0, 0.4, 0])
        self.assertAlmostEqual(hinge_angle(rotation, [0, 1, 0]), 0.4, places=7)
        self.assertAlmostEqual(hinge_angle(rotation, [0, -1, 0]), -0.4, places=7)

    def test_unmapped_axis_yields_blank(self):
        recording = Recording()
        q = Rotation.from_rotvec([.3, 0, 0]).as_quat()
        recording.add(100, Measurement("joints_local", "finger", "hand", "ORIENTATION", "FLOAT[4]", q))
        calibration = {"joints": {"finger": {"status": "stable", "quaternion_xyzw": [0, 0, 0, 1]}}}
        row = list(angle_rows(recording, calibration, {}, {"finger": {"status": "unmapped"}}))[0]
        self.assertIsNone(row["candidate_flexion_rad"])
        self.assertIn("axis_unmapped", row["calibration_status"])
        row = list(angle_rows(recording, calibration, {"finger": {"flexion_axis": [1, 0, 0]}}, {}))[0]
        self.assertAlmostEqual(row["candidate_flexion_rad"], 0.3, places=7)

    def test_multiaxis_order(self):
        rotation = Rotation.from_rotvec([0.2, 0, 0]) * Rotation.from_rotvec([0, 0.1, 0])
        recording = Recording()
        recording.add(100, Measurement("joints_local", "finger", "hand", "ORIENTATION", "FLOAT[4]",
                                       rotation.as_quat()))
        calibration = {"joints": {"finger": {"status": "stable", "quaternion_xyzw": [0, 0, 0, 1]}}}
        row = list(angle_rows(recording, calibration,
                              {"finger": {"flexion_axis": [1, 0, 0], "abduction_axis": [0, 1, 0]}}, {}))[0]
        self.assertAlmostEqual(row["candidate_flexion_rad"], .2, places=7)
        self.assertAlmostEqual(row["candidate_abduction_rad"], .1, places=7)


if __name__ == "__main__":
    unittest.main()
