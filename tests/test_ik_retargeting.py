import json
import unittest
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from myohand.ik_retargeting import (KinematicFitter, load_landmarks, model_points,
                                    register_neutral)
from myohand.model_inspector import default_model_path


class RegistrationTests(unittest.TestCase):
    def test_proper_rotation_scale_translation(self):
        source = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 1]], float)
        rotation = Rotation.from_euler("xyz", [20, -30, 70], degrees=True).as_matrix()
        expected = 0.8 * (rotation @ source.T).T + [0.02, -0.01, 0.03]
        result = register_neutral(source, expected)
        np.testing.assert_allclose(result["rotation"], rotation, atol=1e-12)
        self.assertAlmostEqual(result["scale"], 0.8)
        self.assertLess(result["neutral_shape_rmse_m"], 1e-12)

    @unittest.skipIf(default_model_path() is None, "MyoSuite model unavailable")
    def test_analytic_jacobian_and_synthetic_pose(self):
        model = mujoco.MjModel.from_xml_path(str(default_model_path()))
        config = Path(__file__).resolve().parents[1] / "config/myohand_landmarks.json"
        reference, landmarks = load_landmarks(model, config)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        neutral = model_points(model, data, reference, landmarks)
        registration = {"scale": 1.0, "rotation": np.eye(3)}
        fitter = KinematicFitter(model, reference, landmarks, neutral, registration)
        q = np.clip(fitter.default_qpos[fitter.qadr] + 0.1, fitter.lower + 0.01, fitter.upper - 0.01)
        fitter.evaluate(q)
        jac = fitter._jacobian()
        eps = 1e-6
        numeric = (fitter.evaluate(q + np.eye(len(q))[0] * eps) -
                   fitter.evaluate(q - np.eye(len(q))[0] * eps)).ravel() / (2 * eps)
        np.testing.assert_allclose(jac[:, 0], numeric, atol=1e-6)
        desired = q.copy()
        desired[:] = fitter.default_qpos[fitter.qadr]
        target_index = [model.joint(i).name for i in fitter.joint_ids].index("mcp2_flexion")
        desired[target_index] = 0.5
        target = fitter.evaluate(desired)
        solution, error, status = fitter.fit(target, fitter.default_qpos[fitter.qadr], 40)
        self.assertLess(float(np.mean(error)), 0.001)
        self.assertLess(abs(solution[target_index] - 0.5), 0.02)
        self.assertTrue(status["success"])
        self.assertGreater(status["status"], 0)
        self.assertIsInstance(status["message"], str)
        self.assertGreater(status["nfev"], 0)
        self.assertGreater(status["njev"], 0)
        self.assertGreaterEqual(status["cost"], 0)
        self.assertGreaterEqual(status["optimality"], 0)
        self.assertGreater(status["ik_compute_ms"], 0)
        _, _, limited = fitter.fit(target, fitter.default_qpos[fitter.qadr], 1)
        self.assertFalse(limited["success"])
        self.assertEqual(limited["status"], 0)
        self.assertEqual(limited["nfev"], 1)
        self.assertIn("maximum number", limited["message"].lower())


if __name__ == "__main__":
    unittest.main()
