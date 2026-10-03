#!/usr/bin/env python3
"""Which reference should judge the blend of image and constant velocity (#136)? Offline, per scan rotation, against the truth.

    python scripts/analyse_blend_reference.py <run dir> [<run dir> ...] (--oracle=<oracle_motion.npz> | --gyro=<bag>,<imu topic>) [--w=20]

#133 / #134: the blend (#131) helps the car, hurts the drone (+33..+82 %) - its weights take the ICP step as the truth, and the 16-beam
ICP is a weak one.  Per scan k: image M_k, constant velocity C_k = P_{k-2}^-1 P_{k-1}, ICP step I_k = P_{k-1}^-1 P_k.  Truth: the GT sweep
motion (--oracle) or the LiDAR's own gyro (--gyro, rotation only, scans rotating > 0.2 deg).  Printed:
  1. each estimator's own error against the truth (image, CV, ICP) - is the ICP a usable reference here?
  2. blends, weight of the image w = v_C / (v_I + v_C) over the last w scans (causal), v from
     icp  : squared error against the ICP step (#131, the current rule)
     hat  : three-cornered hat - v_M = (d_MC^2 + d_MI^2 - d_CI^2) / 2, v_C = (d_MC^2 + d_CI^2 - d_MI^2) / 2 from the pairwise disagreements,
            no estimator taken as the truth (clipped at 1e-4 deg^2)
     self : no ICP - v_C = mean angle(M_{j-1}^-1 M_j)^2 (how much the image says the motion changes), v_M = mean d_MC^2 - v_C (what is left
            of the disagreement), clipped
     oracle-best : the per-scan choice from the truth (ceiling, not a method)
Median rotation error (deg), and the mean image weight.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))


def ang(A, B):
    return np.degrees(np.linalg.norm(R.from_matrix(np.swapaxes(A[..., :3, :3], -1, -2) @ B[..., :3, :3]).as_rotvec(), axis=-1))


def blend(C, M, w):
    return Slerp([0, 1], R.from_matrix(np.stack([C[:3, :3], M[:3, :3]])))(w).as_matrix()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    o = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--"))
    W = int(o.get("w", 20))
    truth_rot = None
    if "gyro" in o:
        from score_motion_gyro import read_imu, gyro_rotvecs, kabsch, times
        bag, topic = o["gyro"].split(",")
        imu = read_imu(bag, topic)
    else:
        D_all = np.load(o["oracle"])["motion"]
    print(f"{'run':36s} {'image':>6s} {'CV':>6s} {'ICP':>6s} | {'icp':>6s} {'hat':>6s} {'self':>6s} {'best':>6s} | image weight icp / hat / self")
    for run in args:
        f = sorted(Path(run).glob("*/image_motions.npz"))[-1]
        d = np.load(f); P = np.load(next(f.parent.glob("*_poses.npy")))
        n = len(P)
        M = np.full((n, 4, 4), np.nan); s = d["scan"] < n; M[d["scan"][s]] = d["motion"][s]
        I = np.full((n, 4, 4), np.nan); I[1:] = np.linalg.inv(P[:-1]) @ P[1:]
        C = np.full((n, 4, 4), np.nan); C[2:] = I[1:-1]
        if "gyro" in o:
            rd, st, sp, pt = times(run)
            dP = np.array([R.from_matrix(P[j - 1][:3, :3].T @ P[j][:3, :3]).as_rotvec() for j in range(1, n)])
            a, b = pt[:n - 1], pt[1:n]; big = np.linalg.norm(dP, axis=1) > np.radians(0.3)
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
            G = gyro_rotvecs(imu, st[:n] + off, st[:n] + sp[:n] + off, bias) @ Q.T
            T = np.full((n, 4, 4), np.nan); okg = np.isfinite(G[:, 0]); T[okg] = np.eye(4); T[okg, :3, :3] = R.from_rotvec(G[okg]).as_matrix()
            score = okg & (np.degrees(np.linalg.norm(np.nan_to_num(G), axis=1)) > 0.2)
        else:
            T = np.full((n, 4, 4), np.nan); m = min(n, len(D_all)); T[:m] = D_all[:m]
            score = np.isfinite(T).all(axis=(1, 2))
        ok = np.isfinite(M).all(axis=(1, 2)) & np.isfinite(C).all(axis=(1, 2)) & np.isfinite(I).all(axis=(1, 2))
        dMC, dMI, dCI = np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.nan)
        dMC[ok], dMI[ok], dCI[ok] = ang(M[ok], C[ok]), ang(M[ok], I[ok]), ang(C[ok], I[ok])
        okM = np.isfinite(M).all(axis=(1, 2)); dMM = np.full(n, np.nan)
        k2 = np.flatnonzero(okM[1:] & okM[:-1]) + 1; dMM[k2] = ang(M[k2 - 1], M[k2])
        rules = {"icp": [], "hat": [], "self": []}
        Rb = {r: np.full((n, 3, 3), np.nan) for r in rules}
        for k in range(n):
            if not ok[k]:
                continue
            sl = slice(max(0, k - W), k)
            a, b_, c = dMC[sl], dMI[sl], dCI[sl]; mm = np.isfinite(a) & np.isfinite(b_) & np.isfinite(c)
            if mm.sum() < 3:
                for r in rules:
                    Rb[r][k] = M[k][:3, :3]; rules[r].append(1.0)
                continue
            A2, B2, C2 = np.mean(a[mm] ** 2), np.mean(b_[mm] ** 2), np.mean(c[mm] ** 2)
            vMi, vCi = np.mean(dMI[sl][mm] ** 2), np.mean(dCI[sl][mm] ** 2)            # "icp": errors against the ICP step
            vMh, vCh = max((A2 + B2 - C2) / 2, 1e-4), max((A2 + C2 - B2) / 2, 1e-4)  # "hat"
            x = dMM[sl]; xm = np.isfinite(x)
            vCs = np.mean(x[xm] ** 2) if xm.sum() >= 3 else A2 / 2
            vMs = max(A2 - vCs, 1e-4); vCs = max(vCs, 1e-4)                           # "self"
            for r, (vm, vc) in (("icp", (vMi, vCi)), ("hat", (vMh, vCh)), ("self", (vMs, vCs))):
                w = vc / (vm + vc); rules[r].append(w); Rb[r][k] = blend(C[k], M[k], w)
        sc = ok & score
        eM, eC, eI = ang(T[sc], M[sc]), ang(T[sc], C[sc]), ang(T[sc], I[sc])
        Tr = T[sc][:, :3, :3]
        eb = {r: np.degrees(np.linalg.norm(R.from_matrix(np.swapaxes(Tr, -1, -2) @ Rb[r][sc]).as_rotvec(), axis=-1)) for r in rules}
        best = np.minimum(eM, eC)
        name = "/".join(Path(run).parts[-2:])
        print(f"{name:36s} {np.median(eM):6.3f} {np.median(eC):6.3f} {np.median(eI):6.3f} | {np.median(eb['icp']):6.3f} {np.median(eb['hat']):6.3f} "
              f"{np.median(eb['self']):6.3f} {np.median(best):6.3f} | {np.mean(rules['icp']):.2f} / {np.mean(rules['hat']):.2f} / {np.mean(rules['self']):.2f}")


if __name__ == "__main__":
    main()
