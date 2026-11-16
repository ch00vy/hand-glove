"""Inspect the compiled wrist joints, including their actual FK world axes."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from myohand.model_inspector import default_model_path, inspect_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=default_model_path())
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    inventory, model = inspect_model(args.model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rows = []
    for row in inventory['joints']:
        if row['name'] not in ('pro_sup', 'deviation', 'flexion'):
            continue
        jid = model.joint(row['name']).id
        chain = []
        body = int(model.jnt_bodyid[jid])
        while body:
            chain.append(model.body(body).name)
            body = int(model.body_parentid[body])
        rows.append({**row, 'range_deg': np.rad2deg(row['range']).tolist(),
                     'world_axis_at_default': data.xaxis[jid].tolist(),
                     'body_chain': list(reversed(chain))})
    report = {'model_path': inventory['model_path'], 'rotation_order':
              'ancestor bodies first; joints on the same body follow compiled joint order',
              'joints': rows}
    content = json.dumps(report, indent=2)
    print(content)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
