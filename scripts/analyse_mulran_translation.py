#!/usr/bin/env python3
"""Is the image translation short on MulRan, and do the near matches cause it? (#212, offline, as #039 / #130 on MulRan)

    python scripts/analyse_mulran_translation.py <mulran seq dir> [--n=300] [--start=auto|<scan>] [--out=<txt>]

Runs the locked method's image estimator (upright SURF, guided, car model, subpixel, floor filter; no range fallback) on N consecutive scans
(start "auto" = the N-scan window with the highest mean ground-truth speed) and compares with the ground truth in the LiDAR frame
(scripts/mulran_gt.py):
1. per scan: scale = (t_image . u) / |t_gt|, t_gt the true motion over one sweep, u its direction;
2. per inlier pair (match_motion.last_pairs: p ~ R q + t, p in the previous scan, q in the current): the displacement it implies along u,
   (p - R q) . u, over the true displacement between the two points' capture times (|v| x (tq - tp) x 0.1 s), by the range of q;
   and the share of inliers per range band.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import kiss_slam.intensity_deskew as d  # noqa: E402
from kiss_slam.tools.mulran import MulRan  # noqa: E402

BINS = [(0, 2), (2, 4), (4, 8), (8, 16), (16, 1e9)]


def main():
    seq = Path(sys.argv[1])
    o = dict(a[2:].split("=") for a in sys.argv[2:] if a.startswith("--"))
    N = int(o.get("n", 300))
    g = np.loadtxt(seq / "gt_lidar_tum.txt")
    gt_t, gt_p, gt_R = g[:, 0], g[:, 1:4], Rotation.from_quat(g[:, 4:8])
    slerp = Slerp(gt_t, gt_R)
    pos = lambda t: np.stack([np.interp(t, gt_t, gt_p[:, k]) for k in range(3)], -1)
    ds = MulRan(seq)
    st = ds.stamps
    if o.get("start", "auto") == "auto":
        v = np.linalg.norm(pos(st + 0.1) - pos(st), axis=1) / 0.1
        v[(st < gt_t[0]) | (st + 0.1 > gt_t[-1])] = 0
        first = int(np.argmax(np.convolve(v, np.ones(N) / N, mode="valid")))
    else:
        first = int(o["start"])
    est = d.ScanMotionEstimator(seed=0, model="car", subpixel=True, detector="surf", surf_upright=True, guided_window=40,
                                guided_prediction="hybrid")
    scales, pair_rows, speeds = [], [], []
    for k in range(first, first + N):
        xyz, t, it, ring = ds[k]
        M, n = est.motion(xyz, t, it, ring)
        t0 = st[k]
        if M is None or not (gt_t[0] <= t0 - 0.1 and t0 + 0.1 <= gt_t[-1]):
            continue
        R0 = slerp([t0])[0]
        tg = R0.inv().apply(pos(t0 + 0.1) - pos(t0))                    # true motion over one sweep, in the LiDAR frame at t0
        L = np.linalg.norm(tg)
        if L < 0.3:                                                              # not moving: no scale
            continue
        u = tg / L
        speeds.append(L / 0.1)
        scales.append(float(M[:3, 3] @ u) / L)
        pr = d.match_motion.last_pairs
        if pr is not None:
            p, tp, q, tq = pr
            disp = (p - q @ M[:3, :3].T) @ u                                     # displacement each pair implies along the motion
            true = (L / 0.1) * (tq - tp) * 0.1
            ok = true > 1e-3
            pair_rows.append(np.column_stack([np.linalg.norm(q, axis=1)[ok], (disp[ok] / true[ok])]))
    P = np.vstack(pair_rows) if pair_rows else np.zeros((0, 2))
    out = [f"{seq.name}: scans {first}..{first + N - 1}, {len(scales)} moving scans with an image motion, speed median {np.median(speeds):.1f} m/s "
           f"({np.median(speeds) * 3.6:.0f} km/h)",
           f"per-scan image translation scale vs GT: median {np.median(scales):.3f}, mean {np.mean(scales):.3f}, 10-90 % "
           f"{np.percentile(scales, 10):.3f}-{np.percentile(scales, 90):.3f}",
           f"inlier pairs {len(P)}, range median {np.median(P[:, 0]):.1f} m",
           "range band | share of inliers | pair scale (median)"]
    for a, b in BINS:
        m = (P[:, 0] >= a) & (P[:, 0] < b)
        out.append(f"{a:>3.0f}-{b if b < 1e8 else 99:>3.0f} m | {100 * m.mean():5.1f} % | {np.median(P[m, 1]) if m.any() else float('nan'):.3f}")
    txt = "\n".join(out)
    print(txt)
    if "out" in o:
        Path(o["out"]).write_text(txt + "\n")


if __name__ == "__main__":
    main()
