"""Play the offline qpos trajectory in a MuJoCo viewer (pose playback only)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from myohand.model_inspector import default_model_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Play fitted myoHand qpos in MuJoCo")
    parser.add_argument("--trajectory", default=str(ROOT / "outputs/myohand_ik_trajectory.npz"))
    parser.add_argument("--model", help="myohand.xml; defaults to installed MyoSuite model")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed harus positif")
    path = Path(args.trajectory)
    if not path.is_file():
        parser.error(f"Trajectory tidak ditemukan: {path}")
    model_path = Path(args.model) if args.model else default_model_path()
    if model_path is None or not model_path.is_file():
        parser.error("Model myohand.xml tidak ditemukan; gunakan --model")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    with np.load(path) as recording:
        qpos = recording["qpos"]
        times = recording["time_s"]
        joint_names = recording["joint_names"] if "joint_names" in recording else None
    if qpos.ndim != 2 or qpos.shape[1] != model.nq or len(times) != len(qpos):
        parser.error("Trajectory tidak cocok dengan jumlah qpos model")
    if joint_names is not None and list(joint_names) != [model.joint(i).name for i in range(model.njnt)]:
        parser.error("Nama/urutan joint trajectory tidak cocok dengan model")
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                started = time.monotonic()
                for pose, t in zip(qpos, times):
                    if not viewer.is_running():
                        return 0
                    data.qpos[:] = pose
                    data.qvel[:] = 0
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    remaining = started + float(t) / args.speed - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
                if not args.loop:
                    while viewer.is_running():
                        time.sleep(.1)
                    break
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
