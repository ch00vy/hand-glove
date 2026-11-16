"""Synthetic orientation and actual-model integration tests; no glove required."""
import time
import unittest
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from motion_processing.csv_parser import HandFrame, Measurement
from myohand.ik_retargeting import KinematicFitter, load_landmarks, model_points
from myohand.model_inspector import default_model_path
from myohand.wrist_retargeting import (Orientations, OrientationCalibration,
                                       WristRetargeter, extract_orientations)
from real_time.pipeline import LiveRetargeter


class OrientationTests(unittest.TestCase):
    def test_world_and_local_metadata(self):
        frame = HandFrame(100)
        f = Rotation.from_euler('xyz', [30, -10, 40], degrees=True)
        rel = Rotation.from_euler('xy', [20, 15], degrees=True)
        def m(group, name, parent, r):
            return Measurement(group, name, parent, 'ORIENTATION', 'FLOAT[4]', r.as_quat())
        for group in ('joints_local', 'joints_world'):
            frame.groups[group]['right_fore_arm'] = m(group, 'right_fore_arm', 'LTP_NED', f)
            frame.groups[group]['right_hand'] = m(group, 'right_hand',
                'right_fore_arm' if group == 'joints_local' else 'LTP_NED',
                rel if group == 'joints_local' else f * rel)
        a, b = extract_orientations(frame), extract_orientations(frame, 'joints_local')
        self.assertLess((a.hand.inv() * b.hand).magnitude(), 1e-12)
        frame.groups['joints_world']['right_hand'].parent = 'other'
        with self.assertRaises(ValueError):
            extract_orientations(frame)
        del frame.groups['joints_world']['right_fore_arm']
        with self.assertRaises(ValueError):
            extract_orientations(frame)

    def test_calibration_antipodal_noise_and_motion(self):
        rng = np.random.default_rng(42)
        base = Rotation.from_euler('xyz', [20, 60, 179], degrees=True)
        cal = OrientationCalibration(.5, 15, 2.)
        for i in range(45):
            r = base * Rotation.from_rotvec(rng.normal(0, .001, 3))
            q = r.as_quat() * (-1 if i % 2 else 1)
            cal.add(i * 12000, Orientations(Rotation.from_quat(q), r, 'LTP_NED'))
        self.assertIsNotNone(cal.neutral)
        self.assertLess(np.rad2deg((cal.neutral.hand.inv() * base).magnitude()), .1)
        spike = base * Rotation.from_rotvec([.5, 0, 0])
        self.assertIsNone(cal.add(45 * 12000, Orientations(spike, spike, 'LTP_NED')))
        self.assertIsNone(cal.neutral)
        moving = OrientationCalibration(.5, 15, 2.)
        for i in range(45):
            r = Rotation.from_rotvec([i * .05, 0, 0])
            self.assertIsNone(moving.add(i * 12000, Orientations(r, r, 'LTP_NED')))
        # A gap/reference change cannot complete a calibration using old frames.
        self.assertIsNone(cal.add(5_000_000, Orientations(base, base, 'other')))
        self.assertIsNone(cal.neutral)
        self.assertEqual(len(cal.samples), 1)


@unittest.skipIf(default_model_path() is None, 'MyoSuite unavailable')
class WristModelTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(default_model_path()))
        self.data = mujoco.MjData(self.model)
        self.hand, self.landmarks = load_landmarks(self.model,
            Path(__file__).resolve().parents[1] / 'config/myohand_landmarks.json')
        self.forearm = self.model.body('radius').id
        self.adr = [self.model.joint(n).qposadr[0] for n in ('pro_sup', 'deviation', 'flexion')]
        # Different noncommuting source world, hand and forearm coordinate frames.
        self.a = Rotation.from_euler('xyz', [55, -28, 70], degrees=True)
        self.c = Rotation.from_euler('xyz', [-10, 25, 42], degrees=True)
        self.b = Rotation.from_euler('zyx', [12, 40, -18], degrees=True)
        self.neutral = self.source([0, 0, 0])

    def source(self, q):
        self.data.qpos[self.adr] = q
        mujoco.mj_forward(self.model, self.data)
        f = Rotation.from_matrix(self.data.xmat[self.forearm].reshape(3, 3).copy())
        h = Rotation.from_matrix(self.data.xmat[self.hand].reshape(3, 3).copy())
        return Orientations(self.a.inv() * f * self.b, self.a.inv() * h * self.c, 'LTP_NED')

    def fitter(self, **kw):
        return WristRetargeter(self.model, self.hand, self.c.as_matrix(), self.neutral, **kw)

    def test_seven_motion_scenarios_and_bounds(self):
        for name, q in [('neutral', [0, 0, 0]), ('palm_down', [.8, 0, 0]),
                        ('palm_up', [-.8, 0, 0]), ('flexion', [0, 0, .5]),
                        ('extension', [0, 0, -.5]), ('radial', [0, -.13, 0]),
                        ('ulnar', [0, .35, 0]), ('combined', [.7, .2, -.4])]:
            with self.subTest(name=name):
                fit = self.fitter(mode='forearm-twist')
                result, metrics = fit.fit(self.source(q))
                np.testing.assert_allclose(result, q, atol=.002)
                self.assertLess(metrics['wrist_orientation_error_deg'], .1)
                self.assertTrue(metrics['wrist_optimizer_success'])
                self.assertGreater(metrics['wrist_compute_ms'], 0)
                self.assertTrue(np.all(result >= fit.lower))
                self.assertTrue(np.all(result <= fit.upper))
        fit = self.fitter(mode='forearm-twist')
        result, metrics = fit.fit(self.source([2., .9, 1.2]))
        self.assertTrue(np.all(result >= fit.lower))
        self.assertTrue(np.all(result <= fit.upper))
        self.assertGreater(metrics['wrist_orientation_error_deg'], 5)

    def test_relative_mode_rejects_global_rotation(self):
        fit = self.fitter()
        q, info = fit.fit(self.source([.9, .2, -.3]))
        np.testing.assert_allclose(q, [0, .2, -.3], atol=.002)
        self.assertEqual(info['wrist_observability'], 'relative_only')
        self.assertLess(info['wrist_orientation_error_deg'], .1)

    def test_deviation_reverses_after_saturation_with_forearm_rotation(self):
        fit = self.fitter(mode='forearm-twist')
        for deviation in (.8, -.6, .3, -.1, 0.):
            q, metrics = fit.fit(self.source([.7, deviation, -.2]))
            expected = np.clip(deviation, fit.lower[1], fit.upper[1])
            self.assertAlmostEqual(q[1], expected, delta=.003)
            self.assertAlmostEqual(q[0], .7, delta=.003)
            self.assertEqual(metrics['deviation_at_limit'], deviation in (.8, -.6))
            if abs(deviation) > .05:
                self.assertEqual(np.sign(metrics['wrist_input_deviation_component_deg']),
                                 np.sign(deviation))
            self.assertGreater(metrics['wrist_input_forearm_delta_deg'], 35.)

    def test_antipodal_wrap_and_continuity(self):
        fit = self.fitter(mode='forearm-twist')
        last = None
        for degrees in range(160, 202, 2):
            current = self.source([np.deg2rad(degrees), .1, -.1])
            if degrees % 4:
                current = Orientations(Rotation.from_quat(-current.forearm.as_quat()),
                                       Rotation.from_quat(-current.hand.as_quat()), 'LTP_NED')
            q, _ = fit.fit(current)
            if last is not None:
                self.assertLess(np.max(np.abs(q - last)), .01)
            last = q
        self.assertGreater(fit.last_twist, np.pi)
        self.assertAlmostEqual(q[0], fit.upper[0], places=4)

    def test_combined_finger_geometry_and_merge(self):
        self.data.qpos[:] = self.model.qpos0
        mujoco.mj_forward(self.model, self.data)
        neutral = model_points(self.model, self.data, self.hand, self.landmarks)
        fingers = KinematicFitter(self.model, self.hand, self.landmarks, neutral,
                                  {'scale': 1., 'rotation': np.eye(3)})
        q = fingers.default_qpos[fingers.qadr].copy()
        q[list(fingers.qadr).index(self.model.joint('mcp2_flexion').qposadr[0])] = .5
        expected = fingers.evaluate(q).copy()
        fit = self.fitter(mode='forearm-twist')
        fit.fit(self.source([.6, .2, -.3]))
        merged = fit.merge_qpos(fingers.full_qpos(q))
        np.testing.assert_array_equal(merged[fingers.qadr], q)
        self.data.qpos[:] = merged
        mujoco.mj_forward(self.model, self.data)
        np.testing.assert_allclose(model_points(self.model, self.data, self.hand, self.landmarks),
                                   expected, atol=1e-12)

    def test_live_atomic_world_missing_and_recalibration(self):
        pipeline = LiveRetargeter(self.model, self.hand, self.landmarks, device_id=7,
                                 wrist_mode='forearm-twist', neutral_window_s=.1,
                                 min_neutral_frames=5)
        def message(i, q, missing=False):
            self.data.qpos[:] = self.model.qpos0
            self.data.qpos[self.adr] = q
            if i > 22:
                self.data.qpos[self.model.joint('mcp2_flexion').qposadr[0]] = .5
            mujoco.mj_forward(self.model, self.data)
            rows = [('right_hand', self.hand)]
            if not missing:
                rows.append(('right_fore_arm', self.forearm))
            streams = []
            for name, body in rows:
                quat = Rotation.from_matrix(self.data.xmat[body].reshape(3, 3)).as_quat()
                streams.append({'metadata': {'target_frame': name, 'reference_frame': 'LTP_NED',
                    'measure_type': 'TRANSFORM', 'data_type': 'FLOAT[7]'},
                    'values': (*self.data.xpos[body], *quat)})
            for landmark in self.landmarks:
                pos = (self.data.site_xpos[landmark.object_id] if landmark.object_type == 'site'
                       else self.data.xpos[landmark.object_id])
                streams.append({'metadata': {'target_frame': landmark.rokoko,
                    'reference_frame': 'LTP_NED', 'measure_type': 'TRANSFORM', 'data_type': 'FLOAT[7]'},
                    'values': (*pos, 0, 0, 0, 1)})
            return {'timestamp_us': 1_000_000 + 12000*i, 'device_id': 7,
                    'group_name': 'joints_world', 'streams': streams}
        def wait_pose(ts):
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                pose = pipeline.poses.get(timeout=3)
                if pose['timestamp_us'] == ts:
                    return pose
            self.fail('Missing current output')
        pipeline.start()
        try:
            for i in range(50):
                pipeline.on_data(message(i, [0, 0, 0] if i < 23 else [.6, .2, -.3]))
                time.sleep(.012)
            pose = wait_pose(1_000_000 + 49*12000)
            self.assertTrue(pipeline.wrist_calibrated)
            self.assertEqual(pose['wrist_sync_status'], 'same_world_message')
            np.testing.assert_allclose(pose['qpos'][self.adr], [.6, .2, -.3], atol=.01)
            self.assertGreater(pose['qpos'][self.model.joint('mcp2_flexion').qposadr[0]], .3)
            pipeline.on_data(message(50, [.6, .2, -.3], missing=True))
            missing = wait_pose(1_000_000 + 50*12000)
            self.assertIsNone(missing['wrist_orientation_error_deg'])
            self.assertFalse(missing['wrist_optimizer_success'])
            self.assertEqual(missing['wrist_sync_status'], 'missing_or_invalid')
            np.testing.assert_array_equal(missing['qpos'][self.adr], pose['qpos'][self.adr])
            invalid = message(51, [.6, .2, -.3])
            arm_stream = next(x for x in invalid['streams']
                              if x['metadata']['target_frame'] == 'right_fore_arm')
            arm_stream['values'] = (0, 0, 0, 0, 0, 0, 0)
            pipeline.on_data(invalid)
            invalid_pose = wait_pose(1_000_000 + 51*12000)
            self.assertIsNone(invalid_pose['wrist_orientation_error_deg'])
            np.testing.assert_array_equal(invalid_pose['qpos'][self.adr], pose['qpos'][self.adr])
            # Foreign devices, local groups and out-of-order frames cannot inject wrist data.
            foreign = message(52, [1., 0, 0])
            foreign['device_id'] = 8
            pipeline.on_data(foreign)
            pipeline.on_data({**foreign, 'device_id': 7, 'group_name': 'joints_local'})
            pipeline.on_data(message(49, [1., 0, 0]))
            time.sleep(.05)
            self.assertTrue(pipeline.poses.empty())
            self.assertGreater(pipeline.invalid_world, 0)
            pipeline.recalibrate_wrist()
            for i in range(52, 75):
                pipeline.on_data(message(i, [0, 0, 0]))
                time.sleep(.012)
            pose = wait_pose(1_000_000 + 74*12000)
            self.assertTrue(pipeline.calibrated)
            self.assertTrue(pipeline.wrist_calibrated)
            np.testing.assert_allclose(pose['qpos'][self.adr], 0, atol=.002)
        finally:
            pipeline.stop()


if __name__ == '__main__':
    unittest.main()
