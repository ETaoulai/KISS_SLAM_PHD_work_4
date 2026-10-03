#!/usr/bin/env python3
"""Why the image translation is short (#130, follows #039 on the Ouster sensors) and which correction removes it - offline,
against the ground-truth sweep motion.

    python scripts/analyse_translation_scale.py <sequence key> <oracle_motion.npz> [--frames=400] [--guided=40|none] [--first=0]

Re-runs the image motion (the method's settings: car model, subpixel, stuck filter on the floor, SURF upright) on the first scans and
keeps every scan's inlier pairs (match_motion.last_pairs: p in the previous scan, q in the current one, p ~ R q + t).
1. Per pair the translation it implies with the scan's own rotation, t_pair = p - R q, projected on the true direction of motion u
   and divided by the true length: scale_pair.  By range of q (and floor / not floor): where the shortfall comes from.
2. Translation per scan from the same pairs (rotation fixed to the image's) under candidate corrections, against the truth:
   all pairs (median, the reference), far pairs only (> 8 m, #039 "vector"), range-weighted mean (w ~ r, r^2), the direction from
   all pairs with the length from the far ones (#039 "magnitude"), a robust per-sequence factor (causal running median of the far /
   all ratio over 100 scans, #039 "auto").  Scale, error per scan and over 10 scans (mm).
"""
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import expm, logm

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import kiss_slam.intensity_deskew as d  # noqa: E402
from map_pano_check import dataset_for  # noqa: E402

BINS = [(0, 2), (2, 4), (4, 8), (8, 16), (16, 1e9)]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    o = dict(a[2:].split("=") for a in sys.argv[1:] if a.startswith("--"))
    key, orc = args[0], args[1]
    N, first = int(o.get("frames", 400)), int(o.get("first", 0))
    guided = None if o.get("guided", "40") == "none" else float(o.get("guided", "40"))
    D = np.load(orc)["motion"]
    ds, iscale = dataset_for(key)
    est = d.ScanMotionEstimator(seed=0, model="car", subpixel=True, stuck_min=0.05, floor_only=True, detector="surf",
                                surf_upright=True, intensity_scale=iscale, guided_window=guided)
    rec = []
    for i in range(first + N):
        xyz, ts, inten, ring = ds[i][:4]
        if i < first - 1:
            continue
        M, n = est.motion(np.asarray(xyz, float), np.asarray(ts, float), np.asarray(inten, float), np.asarray(ring))
        pr = getattr(d.match_motion, "last_pairs", None)
        rec.append((i, M, pr if M is not None else None))
    pairs_all, per_scan = [], []
    for i, M, pr in rec:
        if M is None or pr is None or i >= len(D) or not np.isfinite(D[i]).all():
            continue
        tD = D[i][:3, 3]; L = np.linalg.norm(tD)
        if L < 0.03:
            continue
        u = tD / L
        p, tp, q, tq = pr
        Rm = M[:3, :3]
        tpair = p - q @ Rm.T
        s = tpair @ u / L
        r = np.linalg.norm(q, axis=1)
        el = np.degrees(np.arctan2(q[:, 2], np.linalg.norm(q[:, :2], axis=1)))
        # 3. the pairs against the ground-truth motion at each point's own time (constant velocity within each sweep):
        # pose at t (periods from the start of scan k) = exp(t log D_k) for t >= 0, exp(t log D_{k-1}) for t < 0.
        lam = np.full(len(p), np.nan)
        if i >= 1 and np.isfinite(D[i - 1]).all():
            Lk, Lp = np.real(logm(D[i])), np.real(logm(D[i - 1]))
            def pose(t):
                return expm(t * (Lk if t >= 0 else Lp))
            for j in range(len(p)):
                T = np.linalg.inv(pose(tp[j])) @ pose(tq[j])
                tr = T[:3, 3]
                if tr @ tr > 1e-6:
                    lam[j] = (p[j] - T[:3, :3] @ q[j]) @ tr / (tr @ tr)
        pairs_all.append(np.c_[s, r, el, lam])
        per_scan.append((i, M, tD, tpair, r))
    P = np.vstack(pairs_all)
    print(f"{key} (guided {guided}): {len(per_scan)} scans with motion > 3 cm, {len(P)} pairs; image scale (per scan, its own fit) "
          f"p50 {np.median([m[:3, 3] @ t / (t @ t) for _, m, t, _, _ in per_scan]):.3f}")
    print("  per-pair scale by range of the match (median, share of pairs):  " + "  ".join(
        f"{a}-{b if b < 1e8 else ''} m: {np.median(P[(P[:, 1] >= a) & (P[:, 1] < b), 0]):.2f} ({100 * np.mean((P[:, 1] >= a) & (P[:, 1] < b)):.0f} %)"
        for a, b in BINS if ((P[:, 1] >= a) & (P[:, 1] < b)).sum() > 20))
    fl = P[:, 2] < -10
    print(f"  floor (elev < -10 deg): {np.median(P[fl, 0]):.2f} ({100 * fl.mean():.0f} %) · not floor: {np.median(P[~fl, 0]):.2f} · "
          f"pairs with scale < 0.2: {100 * np.mean(P[:, 0] < 0.2):.0f} % (near < 4 m: {100 * np.mean(P[P[:, 1] < 4, 0] < 0.2):.0f} %)")

    ok3 = np.isfinite(P[:, 3])
    print("  pairs vs GT motion at their own times, lambda = share of the true translation they show (median):  all "
          f"{np.median(P[ok3, 3]):.3f} · " + "  ".join(f"{a}-{b if b < 1e8 else ''} m: {np.median(P[ok3 & (P[:, 1] >= a) & (P[:, 1] < b), 3]):.2f}"
          for a, b in BINS if (ok3 & (P[:, 1] >= a) & (P[:, 1] < b)).sum() > 20))
    h, e = np.histogram(np.clip(P[:, 0], -0.5, 2.0), bins=np.arange(-0.5, 2.01, 0.1))
    print("  histogram of per-pair scale (bins of 0.1 from -0.5): " + " ".join(f"{100 * x / len(P):.0f}" for x in h))
    good = (P[:, 0] > 0.5) & (P[:, 0] < 1.5)
    print(f"  pairs between 0.5 and 1.5: {100 * good.mean():.0f} %, their median {np.median(P[good, 0]):.3f}")

    def est_t(tpair, r, mode, f=1.0):
        far = r > 8.0
        if mode == "all":
            return np.median(tpair, axis=0)
        if mode == "far":
            return np.median(tpair[far], axis=0) if far.sum() >= 8 else np.median(tpair, axis=0)
        if mode in ("w_r", "w_r2"):
            w = r if mode == "w_r" else r ** 2
            return (w[:, None] * tpair).sum(0) / w.sum()
        if mode == "magnitude":
            t = np.median(tpair, axis=0)
            if far.sum() < 8:
                return t
            tf = np.median(tpair[far], axis=0); return np.clip(tf @ t / max(t @ t, 1e-12), 0.7, 1.6) * t
        if mode == "auto":
            return f * np.median(tpair, axis=0)

    ratios = []
    out = {m: [] for m in ("all", "far", "w_r", "w_r2", "magnitude", "auto", "image fit (method)")}
    for i, M, tD, tpair, r in per_scan:
        f = np.clip(np.median(ratios[-100:]), 0.7, 1.6) if len(ratios) >= 20 else 1.0
        t_all = np.median(tpair, axis=0)
        far = r > 8.0
        if far.sum() >= 8 and t_all @ t_all > 1e-12:
            ratios.append(np.median(tpair[far], axis=0) @ t_all / (t_all @ t_all))
        for m in out:
            t = M[:3, 3] if m.startswith("image") else est_t(tpair, r, m, f)
            out[m].append((i, t, tD))
    print(f"  {'translation from':22s} {'scale p50':>9s} {'err/scan p50':>12s} {'err/10 p50':>10s}  [mm]   (rotation: the image's own)")
    for m, rows in out.items():
        idx = np.array([a for a, _, _ in rows]); T = np.array([t for _, t, _ in rows]); G = np.array([g for _, _, g in rows])
        sc = np.einsum("ij,ij->i", T, G) / np.einsum("ij,ij->i", G, G)
        e1 = np.linalg.norm(T - G, axis=1) * 1000
        e10 = []                                               # translation only, consecutive scans, summed (rotation ignored)
        pos = {a: k for k, a in enumerate(idx)}
        for a in idx:
            ks = [pos.get(a + j) for j in range(10)]
            if all(k is not None for k in ks):
                e10.append(np.linalg.norm(T[ks].sum(0) - G[ks].sum(0)) * 1000)
        print(f"  {m:22s} {np.median(sc):9.3f} {np.median(e1):12.1f} {np.median(e10):10.1f}")


if __name__ == "__main__":
    main()
