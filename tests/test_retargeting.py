import unittest

import mujoco

from myohand.retargeting import retarget_frame


class RetargetingTests(unittest.TestCase):
    def test_explicit_hinge_mapping_and_fk(self):
        xml = """<mujoco><compiler angle="radian"/><worldbody><body name="finger">
        <joint name="bend" type="hinge" axis="1 0 0" range="-1 1" limited="true"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.01"/>
        </body></worldbody></mujoco>"""
        model = mujoco.MjModel.from_xml_string(xml)
        mapping = {"mappings": [{"mapping_status": "validated", "rokoko_joint": "finger",
                                  "myohand_joint": "bend", "axis": [1, 0, 0],
                                  "sign": 1, "scale": 1, "offset": 0}]}
        row = {"finger": {"candidate_flexion_rad": 0.4, "axis_status": "configured"}}
        qpos, applied = retarget_frame(model, mapping, row)
        self.assertEqual(applied, ["bend"])
        self.assertAlmostEqual(qpos[0], 0.4)
        data = mujoco.MjData(model)
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        self.assertTrue(all(abs(x) < 10 for x in data.xpos.flatten()))
        row["finger"]["candidate_flexion_rad"] = 2.0
        self.assertAlmostEqual(retarget_frame(model, mapping, row)[0][0], 1.0)

    def test_unmapped_or_unconfigured_does_not_apply(self):
        model = mujoco.MjModel.from_xml_string("<mujoco><worldbody><body><joint name='bend' type='hinge'/><geom size='0.01'/></body></worldbody></mujoco>")
        mapping = {"mappings": [{"mapping_status": "unmapped", "rokoko_joint": "finger",
                                  "myohand_joint": "bend", "axis": None}]}
        qpos, applied = retarget_frame(model, mapping, {"finger": {"candidate_flexion_rad": .3,
                                                                   "axis_status": "candidate_pca"}})
        self.assertEqual(applied, [])
        self.assertAlmostEqual(qpos[0], 0)


if __name__ == "__main__":
    unittest.main()
