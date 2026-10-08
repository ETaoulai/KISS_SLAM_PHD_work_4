#!/usr/bin/env python3
"""Hilti 2021 + 2022: every sequence x arm, official protocol of each year, mean +- sigma over seeds (#239).

    python scripts/hilti_table.py [--arms=kiss239,ral239] [--out=<prefix>]

Runs: /home/photogrammetry/kiss_runs_ssd/hilti_2021/<seq>/<arm>_s<k>, .../hilti_2022/<exp>/<arm>_s<k> (a run counts when it has a trajectory).
Ground truth: 2021 the challenge file in <seq>/ground_truth/ (new ones on the external SSD, the first six in kiss_data/hilti_2021) scored by
evaluate_hilti.py; 2022 the sparse control points (<exp>.txt, else <exp>_imu_3dof.txt) by evaluate_hilti2022.py, which also gives the
challenge score (0-100 per sequence) - and for exp14 / 16 / 18 also the dense IMU trajectory (rmse_dense).
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from evaluate_hilti import evaluate_hilti  # noqa: E402
from evaluate_hilti2022 import evaluate_hilti2022  # noqa: E402

RUNS = Path("/home/photogrammetry/kiss_runs_ssd")
SSD = Path("/media/photogrammetry/Extreme SSD")
KD = Path("/home/photogrammetry/kiss_data/hilti_2021")


def gt_2021(seq):
    for base in (SSD / "hilti_2021" / seq, KD / seq):
        g = sorted((base / "ground_truth").glob(f"{seq}_*.txt"))
        if g:
            return g[0]
    return None


def gt_2022(exp):
    d = SSD / "hilti_2022" / exp / "ground_truth"
    sparse = d / f"{exp}.txt"
    if not sparse.exists():
        sparse = d / f"{exp}_imu_3dof.txt"
    dense = d / f"{exp}_imu.txt"
    return sparse, (dense if dense.exists() else None)


def runs_of(seq_dir, arm):
    return [r for r in sorted(seq_dir.glob(f"{arm}_s*")) if r.is_dir() and any(r.glob("*/*_poses_tum.txt"))]


def main():
    o = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    arms = o.get("arms", "kiss239,ral239").split(",")
    rows = []
    for year in ("2021", "2022"):
        base = RUNS / f"hilti_{year}"
        for seq_dir in sorted(base.iterdir()) if base.exists() else []:
            seq = seq_dir.name
            for arm in arms:
                rs = runs_of(seq_dir, arm)
                if not rs:
                    continue
                vals = []
                for r in rs:
                    if year == "2021":
                        g = gt_2021(seq)
                        v = evaluate_hilti(g, r)
                        vals.append(dict(rmse=v["rmse"], score=np.nan, compl=v["n_matched"] / v["n_ref"], dense=np.nan))
                    else:
                        g, gd = gt_2022(seq)
                        v = evaluate_hilti2022(g, r)
                        vd = evaluate_hilti2022(gd, r)["rmse"] if gd is not None else np.nan
                        vals.append(dict(rmse=v["rmse"], score=v.get("score", np.nan), compl=v["completeness"], dense=vd))
                m = {k: float(np.mean([x[k] for x in vals])) for k in vals[0]}
                s = {k: float(np.std([x[k] for x in vals], ddof=1)) if len(vals) > 1 else 0.0 for k in vals[0]}
                rows.append((year, seq, arm, len(vals), m, s))
                print(f"{year} {seq:<34} {arm:<9} n={len(vals)}  APE rmse {m['rmse']:.3f} ± {s['rmse']:.3f} m  score {m['score']:.1f}  "
                      f"compl {m['compl']:.2f}  dense {m['dense']:.3f}", flush=True)
    if "out" in o:
        with open(o["out"] + ".md", "w") as f:
            f.write("| year | sequence | arm | runs | APE rmse m | score /100 | completeness | dense IMU rmse m |\n|---|---|---|---|---|---|---|---|\n")
            for year, seq, arm, n, m, s in rows:
                f.write(f"| {year} | {seq} | {arm} | {n} | {m['rmse']:.3f} ± {s['rmse']:.3f} | {m['score']:.1f} | {m['compl']:.2f} | {m['dense']:.3f} |\n")


if __name__ == "__main__":
    main()
