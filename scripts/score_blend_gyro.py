#!/usr/bin/env python3
"""Image vs constant velocity vs the adaptive blend (#131) per scan, against the LiDAR's own gyroscope - for sequences without a dense
ground truth (NTU VIRAL drone, Hilti), rotation only.

    python scripts/score_blend_gyro.py <bag or bag dir> <imu topic> <run dir> [<run dir> ...] [--w=20] [--min-rot=0.2]

Gyro extrinsic / clock offset / bias as score_motion_gyro.py (fitted to the first run's pose increments).  Per scan k: image M_k, constant
velocity C_k = P_{k-2}^-1 P_{k-1}, blend = slerp(C_k, M_k, v_C / (v_I + v_C)) with v the mean squared error of each predictor against the
ICP step P_{j-1}^-1 P_j over the last w scans (causal).  Scored on scans whose gyro rotation exceeds --min-rot deg (the drone's static
parts would only measure the gyro's own noise).  Medians, degrees.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from score_motion_gyro import read_imu, gyro_rotvecs, kabsch, times  # noqa: E402


def rot_deg(A):
    return np.degrees(np.linalg.norm(R.from_matrix(A[..., :3, :3]).as_rotvec(), axis=-1))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    o = dict(a[2:].split("=") for a in sys.argv[1:] if a.startswith("--"))
    w, min_rot = int(o.get("w", 20)), float(o.get("min-rot", 0.2))
    bag, topic, runs = args[0], args[1], args[2:]
    imu = read_imu(bag, topic)
    rd, _, _, pt = times(runs[0])
    P = np.load(next(rd.glob("*_poses.npy")))[: len(pt)]
    dP = np.array([R.from_matrix(P[j - 1][:3, :3].T @ P[j][:3, :3]).as_rotvec() for j in range(1, len(P))])
    a, b = pt[:-1], pt[1:]
    big = np.linalg.norm(dP, axis=1) > np.radians(0.3)
    best = None
    for off in np.arange(-0.3, 0.3001, 0.005):
        g = gyro_rotvecs(imu, a[big] + off, b[big] + off, 0.0); k = np.isfinite(g[:, 0])
        Q = kabsch(g[k], dP[big][k]); res = np.median(np.linalg.norm(g[k] @ Q.T - dP[big][k], axis=1))
        if best is None or res < best[0]:
            best = (res, off, Q)
    _, off, Q = best
    rate_p = dP / (b - a)[:, None]
    rate_g = np.stack([np.interp(0.5 * (a + b) + off, imu[:, 0], imu[:, j]) for j in range(1, 4)], axis=1)
    bias = np.median(rate_g[big] - rate_p[big] @ Q, axis=0)
    print(f"IMU {topic}: clock offset {off * 1e3:+.0f} ms · bias {np.degrees(np.linalg.norm(bias)):.3f} deg/s · extrinsic {np.degrees(np.linalg.norm(R.from_matrix(Q).as_rotvec())):.1f} deg")
    print(f"{'run':40s} {'scored':>6s} {'image':>7s} {'CV':>7s} {'blend':>7s} {'img wt':>6s}   [deg per scan, median]")
    for run in runs:
        rd, st, sp, _ = times(run)
        d = np.load(sorted(Path(run).glob("*/image_motions.npz"))[-1]); P = np.load(next(rd.glob("*_poses.npy")))
        n = min(len(P), len(st))
        M = np.full((n, 4, 4), np.nan); s = d["scan"] < n; M[d["scan"][s]] = d["motion"][s]
        I = np.full((n, 4, 4), np.nan); I[1:] = np.linalg.inv(P[:n - 1]) @ P[1:n]
        C = np.full((n, 4, 4), np.nan); C[2:] = I[1:n - 1]
        ok = np.isfinite(M).all(axis=(1, 2)) & np.isfinite(C).all(axis=(1, 2)) & np.isfinite(I).all(axis=(1, 2))
        ri, rc = np.full(n, np.nan), np.full(n, np.nan)
        ri[ok] = rot_deg(np.linalg.inv(I[ok]) @ M[ok]); rc[ok] = rot_deg(np.linalg.inv(I[ok]) @ C[ok])
        Bm = np.full((n, 3, 3), np.nan); wts = []
        for k in range(n):
            if not ok[k]:
                continue
            x, y = ri[max(0, k - w):k], rc[max(0, k - w):k]; m = np.isfinite(x) & np.isfinite(y)
            wi = 1.0 if m.sum() < 3 else (np.mean(y[m] ** 2) + 1e-9) / (np.mean(x[m] ** 2) + np.mean(y[m] ** 2) + 2e-9)
            Bm[k] = Slerp([0, 1], R.from_matrix(np.stack([C[k][:3, :3], M[k][:3, :3]])))(wi).as_matrix(); wts.append(wi)
        G = gyro_rotvecs(imu, st[:n] + off, st[:n] + sp[:n] + off, bias) @ Q.T
        sel = ok & np.isfinite(G[:, 0]) & (np.degrees(np.linalg.norm(G, axis=1)) > min_rot)
        Rg = R.from_rotvec(G[sel])
        e = {lab: np.degrees(np.linalg.norm((Rg.inv() * R.from_matrix(X[sel][:, :3, :3] if X.ndim == 3 and X.shape[1] == 4 else X[sel])).as_rotvec(), axis=1))
             for lab, X in (("image", M), ("CV", C), ("blend", Bm))}
        name = "/".join(Path(run).parts[-2:])
        print(f"{name:40s} {int(sel.sum()):6d} {np.median(e['image']):7.3f} {np.median(e['CV']):7.3f} {np.median(e['blend']):7.3f} {np.mean(wts):6.2f}")


if __name__ == "__main__":
    main()
