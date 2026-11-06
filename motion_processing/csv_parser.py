"""Parse tabular RGMP solver recordings into timestamped hand frames."""

from __future__ import annotations

import csv
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

FIELDS = ("timestamp_us", "group", "target_frame", "reference_frame", "measure_type",
          "custom_label", "data_type", "values")


@dataclass(slots=True)
class Measurement:
    group: str
    joint: str
    parent: str
    measure_type: str
    data_type: str
    quaternion: np.ndarray  # scalar-last xyzw
    position: np.ndarray | None = None


@dataclass(slots=True)
class HandFrame:
    timestamp_us: int
    groups: dict[str, dict[str, Measurement]] = field(default_factory=lambda: defaultdict(dict))


def parse_values(data_type: str, measure_type: str, raw: str) -> tuple[np.ndarray | None, np.ndarray]:
    if measure_type not in ("ORIENTATION", "TRANSFORM"):
        raise ValueError(f"Measure type tidak didukung: {measure_type}")
    if data_type not in ("FLOAT[4]", "FLOAT[7]"):
        raise ValueError(f"Data type tidak didukung: {data_type}")
    count = 4 if data_type == "FLOAT[4]" else 7
    # rgmp_stream.py uses semicolons for values when the row separator is comma.
    parts = raw.replace(";", " ").replace(",", " ").split()
    if len(parts) != count:
        raise ValueError(f"{data_type} memerlukan {count} komponen; ada {len(parts)}")
    values = np.array([float(x) for x in parts], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("Komponen NaN/Inf")
    if (measure_type == "ORIENTATION" and count != 4 or
            measure_type == "TRANSFORM" and count != 7):
        raise ValueError("measure_type dan data_type tidak sesuai")
    pos = values[:3] if count == 7 else None
    quat = values[3:] if count == 7 else values
    norm = float(np.linalg.norm(quat))
    if not 0.5 <= norm <= 1.5:
        raise ValueError(f"Norm quaternion tidak masuk akal: {norm}")
    return pos, quat / norm


def measurement_from_stream(group: str, entry: dict, values: tuple) -> Measurement:
    """Same internal type can consume decoded RGMP callback stream entries."""
    position, quat = parse_values(entry["data_type"], entry["measure_type"],
                                  " ".join(map(str, values)))
    return Measurement(group, entry["target_frame"], entry.get("reference_frame", ""),
                       entry["measure_type"], entry["data_type"], quat, position)


@dataclass
class Recording:
    frames: dict[int, HandFrame] = field(default_factory=dict)
    invalid_rows: int = 0
    invalid_examples: list[str] = field(default_factory=list)
    out_of_order_rows: int = 0
    duplicate_rows: int = 0
    total_rows: int = 0
    group_joints: dict[str, set[str]] = field(default_factory=lambda: defaultdict(set))
    parents: dict[str, Counter] = field(default_factory=lambda: defaultdict(Counter))
    measure_types: dict[tuple[str, str], Counter] = field(default_factory=lambda: defaultdict(Counter))
    counts: Counter = field(default_factory=Counter)

    def add(self, timestamp_us: int, item: Measurement) -> None:
        frame = self.frames.setdefault(timestamp_us, HandFrame(timestamp_us))
        if item.joint in frame.groups[item.group]:
            self.duplicate_rows += 1
        frame.groups[item.group][item.joint] = item
        self.group_joints[item.group].add(item.joint)
        self.counts[(item.group, item.joint)] += 1
        self.measure_types[(item.group, item.joint)][item.measure_type] += 1
        if item.group == "joints_local":
            self.parents[item.joint][item.parent] += 1

    def add_rgmp_message(self, message: dict) -> None:
        """Ingest one decoded RGMPClient callback message for later streaming use."""
        for stream in message["streams"]:
            item = measurement_from_stream(message["group_name"], stream["metadata"], stream["values"])
            self.add(int(message["timestamp_us"]), item)

    def sorted_frames(self):
        return [self.frames[ts] for ts in sorted(self.frames)]

    def summary(self) -> dict:
        timestamps = np.array(sorted(self.frames), dtype=np.int64)
        intervals = np.diff(timestamps) if len(timestamps) > 1 else np.array([])
        median_interval = float(np.median(intervals)) if len(intervals) else 0
        large_gaps = intervals[intervals > 1.5 * median_interval] if median_interval else np.array([])
        estimated_missing_timestamps = int(np.sum(np.maximum(np.rint(large_gaps / median_interval).astype(int) - 1, 0))) if len(large_gaps) else 0
        missing = {group: sum(len(joints - set(frame.groups.get(group, {})))
                              for frame in self.frames.values())
                   for group, joints in self.group_joints.items()}
        incomplete = {group: sum(bool(joints - set(frame.groups.get(group, {})))
                                 for frame in self.frames.values())
                      for group, joints in self.group_joints.items()}
        return {
            "frame_count": len(timestamps),
            "duration_s": float((timestamps[-1] - timestamps[0]) / 1e6) if len(timestamps) else 0,
            "timestamp_unit": "microseconds",
            "interval_us": {"min": int(np.min(intervals)), "median": float(np.median(intervals)),
                            "p95": float(np.percentile(intervals, 95)), "max": int(np.max(intervals)),
                            "mean": float(np.mean(intervals))} if len(intervals) else None,
            "nominal_hz_from_median": float(1e6 / np.median(intervals)) if len(intervals) and np.median(intervals) else None,
            "large_timestamp_gap_count": len(large_gaps),
            "estimated_missing_timestamps": estimated_missing_timestamps,
            "joint_count": len(set().union(*self.group_joints.values())) if self.group_joints else 0,
            "groups": {g: {"joint_count": len(j), "missing_measurements": missing[g],
                            "incomplete_frames": incomplete[g]} for g, j in self.group_joints.items()},
            "joints": sorted(set().union(*self.group_joints.values())) if self.group_joints else [],
            "invalid_rows": self.invalid_rows, "invalid_examples": self.invalid_examples,
            "out_of_order_rows": self.out_of_order_rows, "duplicate_rows": self.duplicate_rows,
            "total_rows": self.total_rows,
        }


def parse_csv(path: str | Path) -> Recording:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    recording = Recording()
    last_ts = None
    with path.open("rb") as probe:
        bom = probe.read(4)
    encoding = "utf-16" if bom.startswith((b"\xff\xfe", b"\xfe\xff")) else "utf-8-sig"
    with path.open("r", encoding=encoding, newline="") as handle:
        reader = csv.reader(handle)
        try:
            first = next(reader)
        except StopIteration:
            raise ValueError("CSV kosong") from None
        header = [x.strip().lstrip("#").lower() for x in first]
        has_header = "group" in header and ("timestamp_us" in header or "timestamp" in header)
        if has_header:
            columns = {name: i for i, name in enumerate(header)}
            timestamp_name = "timestamp_us" if "timestamp_us" in columns else "timestamp"
            required = ("group", "target_frame", "reference_frame", "measure_type", "data_type", "values")
            if any(name not in columns for name in required):
                raise ValueError("Header CSV tidak memiliki kolom RGMP yang diperlukan")
            start_line = 2
        else:
            # Official rgmp_stream.py has 8 columns including custom_label;
            # older exports omit that optional column.
            columns = {name: i for i, name in enumerate(FIELDS if len(first) >= 8
                                                          else (x for x in FIELDS if x != "custom_label"))}
            timestamp_name = "timestamp_us"
            start_line = 1
        import itertools
        for line_no, row in enumerate(reader if has_header else itertools.chain([first], reader), start_line):
            if not row or all(not cell.strip() for cell in row):
                continue
            recording.total_rows += 1
            try:
                def value(name):
                    return row[columns[name]].strip()
                ts = int(value(timestamp_name))
                if ts < 0:
                    raise ValueError("Timestamp negatif")
                if last_ts is not None and ts < last_ts:
                    recording.out_of_order_rows += 1
                last_ts = ts
                group, joint = value("group"), value("target_frame")
                if not group or not joint:
                    raise ValueError("Group/joint kosong")
                position, quat = parse_values(value("data_type"), value("measure_type"), value("values"))
                item = Measurement(group, joint, value("reference_frame"), value("measure_type"),
                                   value("data_type"), quat, position)
                recording.add(ts, item)
            except (ValueError, IndexError, KeyError) as exc:
                recording.invalid_rows += 1
                if len(recording.invalid_examples) < 10:
                    recording.invalid_examples.append(f"baris {line_no}: {exc}")
    if not recording.frames:
        raise ValueError(f"Tidak ada pengukuran RGMP valid di {path}")
    return recording
