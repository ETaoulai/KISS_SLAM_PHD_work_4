#!/usr/bin/env python3
"""Which motion model does the ground truth follow at the scale of one LiDAR sweep? (#233, offline, no runs)

    python scripts/analyse_gt_motion_models.py [--out=<txt>] [--step=0.5]

For every ground truth with orientation and >= 10 Hz (list GTS below), on a grid of centre times t_c (every --step s):
the GT samples within +-H of t_c (H = 0.1 s at >= 20 Hz, 0.2 s at 10-20 Hz, so that a window has ~5 samples), expressed
relative to the GT pose at t_c (interpolated), with tau = (t - t_c) / 0.1 s (one sweep = 1, as the image motion fit).
The models are those of intensity_deskew.pose_at, as polynomials in tau with no constant (pose(0) = identity):
    cv   rotation vector and translation linear                          (6 parameters)
    car  rotation quadratic (angular acceleration), translation linear   (9, the method's model)
    ca   rotation and translation quadratic                              (12)
    cub  rotation and translation cubic                                  (18, to see where the GT noise starts)
Each model is scored by leave-one-out prediction: every sample of the window predicted by the model fitted to the others
(so extra parameters only win when the motion really has that shape, not by fitting the GT noise).  Error of a sample:
|translation error| + 10 m x rotation error (the displacement of a point 10 m away), cm.  Windows with too few samples for
the model are skipped for all models.
Caveat: a GT built per scan (NCD 10 Hz, Boreas 10 Hz) or smoothed / interpolated (Spires ~25 Hz from VILENS + registration, MulRan
100 Hz INS) carries its own motion model; the result is "which model the GT supports", an upper bound on what the sweep can show.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).parent))
from evaluate_ncd import load_gt  # noqa: E402

X = Path("/home/photogrammetry/kiss_runs")
SS = Path("/media/photogrammetry/Extreme SSD")
MODELS = {"cv": (1, 1), "car": (2, 1), "ca": (2, 2), "cub": (3, 3)}


def gts():
    out = []
    for d in sorted((X / "official_eval").iterdir()):
        f = d / "gt_lidar.txt"
        if f.exists():
            out.append((d.name, "tum", f))
    have = {n for n, _, _ in out}
    for folder, name in [("2024-03-12-keble-college-05", "keble_05"), ("2024-03-13-observatory-quarter-02", "observatory_02"),
                         ("2024-03-14-blenheim-palace-01", "blenheim_01"), ("2024-03-14-blenheim-palace-05", "blenheim_05"),
                         ("2024-03-12-keble-college-02", "keble_02"), ("2024-03-12-keble-college-04", "keble_04"),
                         ("2024-03-20-christ-church-05", "church_05")]:
        if name not in have:
            out.append((name, "spires", SS / "oxford_spires" / folder / "ground_truth" / "gt-tum.txt"))
    for s in ("KAIST01", "DCC01", "Riverside01", "Sejong01"):
        out.append((s, "tum", SS / "mulran" / s / "gt_lidar_tum.txt"))
    for f in sorted((SS / "boreas").glob("gt_boreas-*_lidar_tum.txt")):
        out.append((f.name[3:-14], "tum", f))
    return out


def load(kind, f):
    if kind == "spires":
        t, T = load_gt(str(f), "spires")[:2]
        return np.asarray(t), np.asarray(T)
    g = np.loadtxt(f)
    T = np.tile(np.eye(4), (len(g), 1, 1))
    T[:, :3, :3] = Rotation.from_quat(g[:, 4:8]).as_matrix()
    T[:, :3, 3] = g[:, 1:4]
    return g[:, 0], T


def design(tau, deg):
    return np.stack([tau ** k for k in range(1, deg + 1)], 1)


def loo(tau, y, deg):
    """Leave-one-out prediction errors of a polynomial (no constant) of degree deg fitted per column of y."""
    err = np.zeros((len(tau), y.shape[1]))
    for i in range(len(tau)):
        m = np.arange(len(tau)) != i
        A = design(tau[m], deg)
        c, *_ = np.linalg.lstsq(A, y[m], rcond=None)
        err[i] = design(tau[i:i + 1], deg) @ c - y[i]
    return err


def analyse(t, T, step):
    rate = (len(t) - 1) / (t[-1] - t[0])
    H = 0.1 if rate >= 20 else (0.2 if rate >= 14 else 0.26)   # >= 5 samples per window (10 Hz: +-2.6 sweeps)
    sl = Slerp(t, Rotation.from_matrix(T[:, :3, :3]))
    res = {m: [] for m in MODELS}
    for tc in np.arange(t[0] + H, t[-1] - H, step):
        i0, i1 = np.searchsorted(t, tc - H), np.searchsorted(t, tc + H, side="right")
        tt = t[i0:i1]
        tt_mask = np.abs(tt - tc) > 1e-4
        if tt_mask.sum() < 5:                      # need >= 5 samples: the cubic LOO fits 3 per axis on the other 4
            continue
        idx = np.arange(i0, i1)[tt_mask]
        Rc = sl([tc])[0]
        pc = np.array([np.interp(tc, t, T[:, k, 3]) for k in range(3)])
        R_rel = Rc.inv() * Rotation.from_matrix(T[idx, :3, :3])
        rv = R_rel.as_rotvec()
        tr = (T[idx, :3, 3] - pc) @ Rc.as_matrix()           # in the sensor frame at t_c
        tau = (t[idx] - tc) / 0.1
        for m, (dr, dt) in MODELS.items():
            er, et = loo(tau, rv, dr), loo(tau, tr, dt)
            res[m].append(np.median(np.linalg.norm(et, axis=1) + 10.0 * np.linalg.norm(er, axis=1)) * 100)   # cm
    return rate, H, {m: np.array(v) for m, v in res.items()}


def main():
    o = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--"))
    step = float(o.get("step", 0.5))
    lines = ["sequence | GT Hz | window +-s | windows | LOO error at 10 m, median cm: cv / car / ca / cub | best | car vs cv | ca vs cv"]
    wins = {m: 0 for m in MODELS}
    for name, kind, f in gts():
        if not Path(f).exists():
            continue
        t, T = load(kind, f)
        rate, H, r = analyse(t, T, step)
        if not len(r["cv"]):
            lines.append(f"{name} | {rate:.1f} | {H} | 0 | too sparse")
            continue
        med = {m: float(np.median(v)) for m, v in r.items()}
        best = min(med, key=med.get)
        wins[best] += 1
        lines.append(f"{name} | {rate:.1f} | {H} | {len(r['cv'])} | " + " / ".join(f"{med[m]:.2f}" for m in MODELS) +
                     f" | {best} | {med['car'] / med['cv']:.2f} | {med['ca'] / med['cv']:.2f}")
        print(lines[-1], flush=True)
    lines.append("best model counts: " + ", ".join(f"{m} {n}" for m, n in wins.items()))
    print(lines[-1])
    if "out" in o:
        Path(o["out"]).write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
