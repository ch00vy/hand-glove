"""Observed parent hierarchy from joints_local, without assumed anatomy."""

from __future__ import annotations

import csv
from pathlib import Path


def build_hierarchy(recording) -> dict[str, dict]:
    joints = recording.group_joints.get("joints_local", set())
    result = {}
    for joint in sorted(joints):
        references = recording.parents[joint]
        parent, count = references.most_common(1)[0]
        result[joint] = {"parent": parent, "observations": count,
                         "consistent": len(references) == 1,
                         "alternatives": dict(references)}
    for joint, info in result.items():
        parent = info["parent"]
        if parent not in result and parent != "LTP_NED":
            info["consistent"] = False
        visited = {joint}
        while parent in result:
            if parent in visited:
                info["consistent"] = False
                break
            visited.add(parent)
            parent = result[parent]["parent"]
    return result


def write_inventory(recording, hierarchy: dict, path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("joint_name", "parent_frame", "local_measurement", "local_frame_count",
                         "world_measurement", "world_frame_count", "parent_consistent", "parent_alternatives"))
        for joint in sorted(set().union(*recording.group_joints.values())):
            info = hierarchy.get(joint, {})
            writer.writerow((joint, info.get("parent", ""),
                             recording.measure_types[("joints_local", joint)].most_common(1)[0][0]
                             if recording.measure_types[("joints_local", joint)] else "",
                             recording.counts[("joints_local", joint)],
                             recording.measure_types[("joints_world", joint)].most_common(1)[0][0]
                             if recording.measure_types[("joints_world", joint)] else "",
                             recording.counts[("joints_world", joint)], info.get("consistent", ""),
                             str(info.get("alternatives", {}))))


def write_skeleton_markdown(hierarchy: dict, path: Path) -> None:
    children = {}
    for joint, info in hierarchy.items():
        children.setdefault(info["parent"], []).append(joint)
    lines = ["# Skeleton teramati dari `joints_local`", "",
             "Parent berasal dari `reference_frame` CSV. `LTP_NED` adalah frame dunia, bukan joint.", "",
             "```text"]
    seen = set()

    def walk(parent: str, depth: int):
        for child in sorted(children.get(parent, [])):
            if child in seen:
                continue
            seen.add(child)
            lines.append("  " * depth + child)
            walk(child, depth + 1)

    walk("LTP_NED", 0)
    for joint in sorted(set(hierarchy) - seen):
        lines.append(f"UNRESOLVED: {joint} parent={hierarchy[joint]['parent']}")
    lines += ["```", "", "## Validasi", "",
              f"- Joint lokal: {len(hierarchy)}",
              f"- Parent konsisten: {sum(x['consistent'] for x in hierarchy.values())}/{len(hierarchy)}",
              "- `ORIENTATION FLOAT[4]` hanya memiliki rotasi; offset lokal tidak dapat disimpulkan dari CSV ini.",
              "- Nama jari dan sendi di atas adalah nama solver, belum label anatomi tervalidasi.", ""]
    path.write_text("\n".join(lines), encoding="utf-8")
