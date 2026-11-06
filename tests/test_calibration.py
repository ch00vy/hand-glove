import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from motion_processing.calibration import calibrate, normalize_quat, relative_rotation
from motion_processing.csv_parser import Measurement, Recording


def recording_with_angles(angles):
    result = Recording()
    for idx, angle in enumerate(angles):
        quat = Rotation.from_rotvec([angle, 0, 0]).as_quat()
        result.add(idx * 100_000, Measurement("joints_local", "finger", "hand",
                                             "ORIENTATION", "FLOAT[4]", quat))
    return result


class CalibrationTests(unittest.TestCase):
    def test_normalization_antipodal_and_identity(self):
        q = normalize_quat([0, 0, 0, 2])
        self.assertEqual(q.tolist(), [0, 0, 0, 1])
        samples = recording_with_angles([0] * 12)
        for i, frame in enumerate(samples.sorted_frames()):
            if i % 2:
                frame.groups["joints_local"]["finger"].quaternion *= -1
        result = calibrate(samples, 0, 1.1)
        self.assertEqual(result["joints"]["finger"]["status"], "stable")
        ref = result["joints"]["finger"]["quaternion_xyzw"]
        self.assertAlmostEqual(relative_rotation(ref, q).magnitude(), 0, places=7)
        self.assertAlmostEqual(relative_rotation(ref, -q).magnitude(), 0, places=7)

    def test_rotation_composition_and_inverse(self):
        neutral = Rotation.from_euler("xyz", [20, 10, 5], degrees=True)
        bend = Rotation.from_euler("z", 35, degrees=True)
        current = neutral * bend
        delta = relative_rotation(neutral.as_quat(), current.as_quat())
        np.testing.assert_allclose(delta.as_matrix(), bend.as_matrix(), atol=1e-12)

    def test_unstable_interval(self):
        angles = np.linspace(0, np.deg2rad(60), 16)
        result = calibrate(recording_with_angles(angles), 0, 1.5)
        self.assertEqual(result["overall_status"], "warning")
        self.assertEqual(result["joints"]["finger"]["status"], "unstable")


if __name__ == "__main__":
    unittest.main()
