import tempfile
import unittest
from pathlib import Path

from motion_processing.csv_parser import Recording, parse_csv, parse_values


class CsvParserTests(unittest.TestCase):
    def test_float4_and_float7(self):
        pos, q = parse_values("FLOAT[4]", "ORIENTATION", "0;0;0;1")
        self.assertIsNone(pos)
        self.assertEqual(q.tolist(), [0, 0, 0, 1])
        pos, q = parse_values("FLOAT[7]", "TRANSFORM", "1;2;3;0;0;0;1")
        self.assertEqual(pos.tolist(), [1, 2, 3])
        self.assertEqual(q.tolist(), [0, 0, 0, 1])

    def test_utf16_header_invalid_incomplete_and_disorder(self):
        text = ("#timestamp_us,group,target_frame,reference_frame,measure_type,custom_label,data_type,values\n"
                "200,joints_local,root,LTP_NED,TRANSFORM,,FLOAT[7],0;0;0;0;0;0;1\n"
                "100,joints_local,root,LTP_NED,TRANSFORM,,FLOAT[7],0;0;0;0;0;0;1\n"
                "100,joints_local,child,root,ORIENTATION,,FLOAT[4],0;0;0;1\n"
                "300,joints_local,child,root,ORIENTATION,,FLOAT[4],broken\n")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.csv"
            path.write_text(text, encoding="utf-16")
            result = parse_csv(path)
        summary = result.summary()
        self.assertEqual(summary["frame_count"], 2)
        self.assertEqual(summary["invalid_rows"], 1)
        self.assertEqual(summary["out_of_order_rows"], 1)
        self.assertEqual(summary["groups"]["joints_local"]["missing_measurements"], 1)
        self.assertEqual(summary["groups"]["joints_local"]["incomplete_frames"], 1)

    def test_no_header_seven_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.csv"
            path.write_text("100,joints_local,root,LTP_NED,ORIENTATION,FLOAT[4],0;0;0;1\n",
                            encoding="utf-8")
            result = parse_csv(path)
        self.assertEqual(result.summary()["joint_count"], 1)

    def test_decoded_stream_ingestion(self):
        result = Recording()
        result.add_rgmp_message({"timestamp_us": 100, "group_name": "joints_local",
                                 "streams": [{"metadata": {"target_frame": "finger", "reference_frame": "hand",
                                                           "measure_type": "ORIENTATION", "data_type": "FLOAT[4]"},
                                              "values": (0, 0, 0, 1)}]})
        self.assertEqual(result.frames[100].groups["joints_local"]["finger"].parent, "hand")


if __name__ == "__main__":
    unittest.main()
