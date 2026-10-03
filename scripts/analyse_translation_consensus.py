#!/usr/bin/env python3
"""Fixes for the short image translation (#130), each a refit of the method's own pairs - offline, against the ground-truth sweep motion.

    python scripts/analyse_translation_consensus.py <sequence key> <oracle_motion.npz> [--frames=300] [--multi]

Per scan: the method's motion (car model, its inlier pairs p ~ T(tp)^-1 T(tq) q, match_motion.last_pairs, last_params x).  Variants,
all with the method's time-aware fit (fit_time) unless said otherwise:
  method          as it is (reference)
  no-stationary   drop the pairs that zero motion explains at least as well as the fitted motion (|p - q| <= |residual|), refit
  consensus       trimmed refit: keep the pairs within 2.5 x the MAD of the residuals (their own spread, no fixed distance), refit,
                  three rounds - the motion most pairs agree on, bad pairs stop pulling
  far>8m          refit on the pairs farther than 8 m (>= 8 pairs, else the method)
  far30%          refit on the farthest 30 % of the scan's pairs (relative: works indoors)
  mode            rotation of the method, translation = the densest point of the per-pair translations t_pair = p - R q
                  (mean-shift, bandwidth = their MAD) - where most pairs agree, not their average
--multi: the method with multi_baseline (scans k-2 -> k too, #090) - run separately, it changes the matching.
Scale (median of t . t_GT / |t_GT|^2, scans moving > 3 cm), translation error per scan and summed over 10 scans (mm).
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import kiss_slam.intensity_deskew as d  # noqa: E402
from map_pano_check import dataset_for  # noqa: E402


def to_M(x):
    rot, tr = d.pose_at(x, np.array([1.0]))
    M = np.eye(4); M[:3, :3] = Rotation.from_rotvec(rot[0]).as_matrix(); M[:3, 3] = tr[0]
    return M


def refit(p, tp, q, tq, M, sel):
    if sel.sum() < 8:
        return None
    Mf, _ = d.fit_time(p[sel], tp[sel], q[sel], tq[sel], M, "car")
    return Mf


def res_norm(x, p, tp, q, tq):
    return np.linalg.norm(d.residual(x, p, tp, q, tq).reshape(-1, 3), axis=1)


def mean_shift(t, bw, iters=30):
    c = np.median(t, axis=0)
    for _ in range(iters):
        w = np.exp(-0.5 * np.sum((t - c) ** 2, axis=1) / bw ** 2)
        c_new = (w[:, None] * t).sum(0) / max(w.sum(), 1e-12)
        if np.linalg.norm(c_new - c) < 1e-5:
            break
        c = c_new
    return c


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    o = dict((a[2:].split("=") + [""])[:2] for a in sys.argv[1:] if a.startswith("--"))
    key, orc, N, multi = args[0], args[1], int(o.get("frames", 300)), "multi" in o
    D = np.load(orc)["motion"]
    ds, iscale = dataset_for(key)
    est = d.ScanMotionEstimator(seed=0, model="car", subpixel=True, stuck_min=0.05, floor_only=True, detector="surf",
                                surf_upright=True, intensity_scale=iscale, guided_window=40.0, multi_baseline=multi)
    if multi:                                   # keep the two-scan motion the joint fit starts from
        orig = est._joint
        def joint(M1, *a):
            joint.M1 = M1.copy(); return orig(M1, *a)
        joint.M1 = None; est._joint = joint
    rots = {}
    names = ["two-scan", "k-2 joint", "rot two-scan + tr k-2"] if multi else ["method", "no-stationary", "consensus", "far>8m", "far30%", "mode"]
    out = {m: {} for m in names}
    for i in range(N):
        xyz, ts, inten, ring = ds[i][:4]
        M, n = est.motion(np.asarray(xyz, float), np.asarray(ts, float), np.asarray(inten, float), np.asarray(ring))
        if M is None or i >= len(D) or not np.isfinite(D[i]).all():
            continue
        if multi:
            M1 = est._joint.M1 if est._joint.M1 is not None else M
            est._joint.M1 = None
            C = M.copy(); C[:3, :3] = M1[:3, :3]
            for lab, X in (("two-scan", M1), ("k-2 joint", M), ("rot two-scan + tr k-2", C)):
                out[lab][i] = X[:3, 3].copy(); rots.setdefault(lab, {})[i] = X
            continue
        out["method"][i] = M[:3, 3].copy()
        pr, x = getattr(d.match_motion, "last_pairs", None), getattr(d.match_motion, "last_params", None)
        if pr is None or x is None:
            continue
        p, tp, q, tq = pr
        r_fit = res_norm(x, p, tp, q, tq)
        r_id = res_norm(np.zeros_like(x), p, tp, q, tq)
        Mv = refit(p, tp, q, tq, M, r_id > r_fit)
        if Mv is not None:
            out["no-stationary"][i] = Mv[:3, 3]
        sel, xc, Mc = np.ones(len(p), bool), x, M
        for _ in range(3):
            r = res_norm(xc, p, tp, q, tq)
            mad = 1.4826 * np.median(np.abs(r[sel] - np.median(r[sel])))
            sel = r <= np.median(r[sel]) + 2.5 * max(mad, 1e-3)
            Mn = refit(p, tp, q, tq, Mc, sel)
            if Mn is None:
                break
            Mc, xc = Mn, d.fit_time.last_params
        out["consensus"][i] = Mc[:3, 3]
        rq = np.linalg.norm(q, axis=1)
        for lab, sel in (("far>8m", rq > 8.0), ("far30%", rq >= np.quantile(rq, 0.7))):
            Mf = refit(p, tp, q, tq, M, sel)
            out[lab][i] = (Mf if Mf is not None else M)[:3, 3]
        tpair = p - q @ M[:3, :3].T
        mad = 1.4826 * np.median(np.abs(tpair - np.median(tpair, axis=0)), axis=0)
        out["mode"][i] = mean_shift(tpair, max(float(np.linalg.norm(mad)) / np.sqrt(3), 0.005))
    print(f"{key}{' (multi_baseline)' if multi else ''}: {len(out[names[0]])} scans with a motion")
    print(f"  {'variant':16s} {'scale p50':>9s} {'err/scan p50':>12s} {'err/10 p50':>10s}  [mm]")
    for m, rows in out.items():
        idx = np.array(sorted(rows)); T = np.array([rows[a] for a in idx]); G = D[idx, :3, 3]
        big = np.linalg.norm(G, axis=1) > 0.03
        sc = np.einsum("ij,ij->i", T[big], G[big]) / np.einsum("ij,ij->i", G[big], G[big])
        e1 = np.linalg.norm(T - G, axis=1) * 1000
        pos = {a: k for k, a in enumerate(idx)}; e10 = []
        for a in idx:
            ks = [pos.get(a + j) for j in range(10)]
            if all(k is not None for k in ks):
                e10.append(np.linalg.norm(T[ks].sum(0) - G[ks].sum(0)) * 1000)
        extra = ""
        if m in rots:
            Rr = rots[m]; er = [np.degrees(np.linalg.norm(Rotation.from_matrix(D[a][:3, :3].T @ Rr[a][:3, :3]).as_rotvec())) for a in idx]
            e10r = []
            for a in idx:
                if all(a + j in Rr for j in range(10)):
                    C1, C2 = np.eye(3), np.eye(3)
                    for j in range(10):
                        C1, C2 = C1 @ Rr[a + j][:3, :3], C2 @ D[a + j][:3, :3]
                    e10r.append(np.degrees(np.linalg.norm(Rotation.from_matrix(C2.T @ C1).as_rotvec())))
            extra = f"   rotation {np.median(er):.3f} / 10 scans {np.median(e10r):.3f} deg"
        print(f"  {m:22s} {np.median(sc):9.3f} {np.median(e1):12.1f} {np.median(e10):10.1f}{extra}")


if __name__ == "__main__":
    main()
