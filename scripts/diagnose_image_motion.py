#!/usr/bin/env python3
"""Two diagnostics of the image motion on existing runs (#094; ideas of the external reviews, 25-28/9): lag / jerk bias, and aliases.

    python scripts/diagnose_image_motion.py <run dir> [<oracle_motion.npz>]

Lag / jerk bias (needs the ground-truth sweep motion, make_oracle_motion.py): per scan the image rotation error e_k = rotvec(D_k^T M_k)
and the ground-truth rotation r_k = rotvec(D_k) (deg per sweep, sensor frame).  The reviews predict, for the two-scan fit:
  - a constant-velocity fit lags by alpha / 2: e regressed on the change of rotation dr_k = r_k - r_{k-1} gives slope ~ -0.5 when the
    angular acceleration is not estimated (the "car" model should give ~0);
  - the "car" fit has a jerk bias -j / 12: e on the second difference d2r_k = r_{k+1} - 2 r_k + r_{k-1} gives slope ~ -1/12.
Pooled over the three axes; slope, R^2 (share of the error explained) and the joint fit.  If neither explains a useful share, a
one-sweep-lag (k-1, k, k+1) estimator has no bias to remove.

Aliases: speed of the image motion per scan (|t| / span) against a reference - the ground truth when given, else the run's own ICP
motion (P_{k-1}^-1 P_k).  A translational alias (one stair step, a facade period) shows as image speeds far above the reference and
as a second peak in |t_image - t_ref| at the structure period.
"""
import csv
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


def rotvec_deg(A):
    return np.degrees(R.from_matrix(A[..., :3, :3]).as_rotvec())


def fit(y, X):
    X1 = np.c_[X, np.ones(len(X))]
    coef, *_ = np.linalg.lstsq(X1, y, rcond=None)
    res = y - X1 @ coef
    return coef[:-1], 1 - res.var() / y.var()


def main():
    run = Path(sys.argv[1]); orc = sys.argv[2] if len(sys.argv) > 2 else None
    rd = sorted(run.glob("*/image_motions.npz"))[-1].parent
    im = np.load(rd / "image_motions.npz")
    rows = list(csv.DictReader(open(rd / "pose_times.csv")))
    n = len(rows); span = np.array([float(r["span_s"]) for r in rows])
    M = np.full((n, 4, 4), np.nan); M[im["scan"]] = im["motion"]
    okM = np.isfinite(M).all(axis=(1, 2))
    name = f"{run.parent.name}/{run.name}"
    if orc is not None:
        D = np.load(orc)["motion"][:n]
        okD = np.isfinite(D).all(axis=(1, 2))
        k = np.arange(1, n - 1)
        k = k[okM[k] & okD[k] & okD[k - 1] & okD[k + 1]]
        rv = np.full((n, 3), np.nan); rv[okD] = rotvec_deg(D[okD])
        e = rotvec_deg(np.linalg.inv(D[k]) @ M[k])
        dr = rv[k] - rv[k - 1]; d2r = rv[k + 1] - 2 * rv[k] + rv[k - 1]
        y = e.ravel(); a = dr.ravel(); j = d2r.ravel()
        good = np.isfinite(y) & np.isfinite(a) & np.isfinite(j) & (np.abs(y) < 5)       # drop gross failures (> 5 deg)
        (sa,), r2a = fit(y[good], a[good][:, None])
        (sj,), r2j = fit(y[good], j[good][:, None])
        (sa2, sj2), r2b = fit(y[good], np.c_[a[good], j[good]])
        (sr,), r2r = fit(y[good], rv[k].ravel()[good][:, None])
        print(f"{name}: {len(k)} scans; image rotation error per axis rms {np.sqrt((y[good] ** 2).mean()):.3f} deg, "
              f"|GT change| median {np.median(np.linalg.norm(dr, axis=1)):.2f}, |GT 2nd diff| median {np.median(np.linalg.norm(d2r, axis=1)):.2f} deg")
        print(f"  lag  (e ~ dr):   slope {sa:+.3f} (cv no-alpha would be -0.5)  R2 {r2a:.3f}")
        print(f"  jerk (e ~ d2r):  slope {sj:+.3f} (-1/12 = -0.083 predicted)   R2 {r2j:.3f}")
        print(f"  both:            dr {sa2:+.3f}  d2r {sj2:+.3f}  R2 {r2b:.3f}   |  scale (e ~ r): slope {sr:+.3f} R2 {r2r:.3f}")
        tref = D[:, :3, 3]; ref_ok = okD; ref = "GT"
    else:
        P = np.load(next(rd.glob("*_poses.npy")))[:n]
        Mi = np.full((n, 4, 4), np.nan); Mi[1:] = np.linalg.inv(P[:-1]) @ P[1:]
        tref = Mi[:, :3, 3]; ref_ok = np.isfinite(Mi).all(axis=(1, 2)); ref = "ICP"
    k = np.flatnonzero(okM & ref_ok & (span > 0.05))
    vi = np.linalg.norm(M[k, :3, 3], axis=1) / span[k]
    vr = np.linalg.norm(tref[k], axis=1) / span[k]
    dt = np.linalg.norm(M[k, :3, 3] - tref[k], axis=1)
    print(f"  speed [m/s] image p50/p99/max {np.median(vi):.2f}/{np.percentile(vi, 99):.2f}/{vi.max():.2f}   {ref} {np.median(vr):.2f}/"
          f"{np.percentile(vr, 99):.2f}/{vr.max():.2f}")
    print(f"  aliases: image > 2.5 m/s {int((vi > 2.5).sum())}, image > 2x {ref} + 0.5 m/s {int(((vi > 2 * vr) & (vi > vr + 0.5)).sum())} of {len(k)} scans")
    h, edges = np.histogram(dt, bins=[0, 0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 1.0, 100])
    print("  |t_image - t_" + ref + "| [m]: " + "  ".join(f"{edges[i]:g}-{edges[i + 1]:g}: {h[i]}" for i in range(len(h))))


if __name__ == "__main__":
    main()
