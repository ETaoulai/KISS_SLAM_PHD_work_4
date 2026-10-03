#!/usr/bin/env python3
"""Where the geometry does not constrain the ICP, is the image motion better than the ICP result? Offline, against the ground-truth
sweep motion (#128, idea M.T. 3/10: "a good image start gets absorbed into a bad ICP"; #023 re-checked with today's image motion).

    python scripts/analyse_degenerate_image.py <sequence key> <oracle_motion.npz> <run dir> [<run dir> ...] [--every=1] [--win=10]

Per scan k: point-to-plane information of the translation H = mean(n n^T) over the scan's normals (voxel 0.25 m, 20 neighbours,
as #023), smallest eigenvalue lam and its direction v.  The run's ICP motion I_k = P_{k-1}^-1 P_k (pose k at the end of sweep k,
after the pose graph), its image motion M_k (the ICP start), the truth D_k.  Translation error along v (the weak direction) and
across it, rotation error - ICP vs image, by lam tercile; then the fusion "ICP, but along v the image" for lam < tau (tau = the
sequence's own lam quantiles 10 / 25 %, so no threshold in absolute units), per scan and composed over `win` scans.
H is computed once per sequence and cached next to the oracle file (degeneracy_H.npz).
"""
import csv
import sys
import warnings
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_pano_check import dataset_for  # noqa: E402

warnings.simplefilter("ignore")


def rot_deg(A):
    return np.degrees(np.linalg.norm(R.from_matrix(A[..., :3, :3]).as_rotvec(), axis=-1))


def info_H(ds, n, every):
    lam, vec = np.full(n, np.nan), np.full((n, 3), np.nan)
    for i in range(n):                                   # the rosbag reader is serial: read every scan, process a subset
        xyz = np.asarray(ds[i][0], dtype=np.float64)
        if i % every:
            continue
        r = np.linalg.norm(xyz, axis=1); xyz = xyz[(r > 1.0) & (r < 60.0)]
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz)).voxel_down_sample(0.25)
        if len(pc.points) < 100:
            continue
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(20))
        N = np.asarray(pc.normals); H = N.T @ N / len(N)
        w, V = np.linalg.eigh(H)
        lam[i], vec[i] = w[0], V[:, 0]
    return lam, vec


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    o = dict(a[2:].split("=") for a in sys.argv[1:] if a.startswith("--"))
    every, win = int(o.get("every", 1)), int(o.get("win", 10))
    key, orc, runs = args[0], Path(args[1]), args[2:]
    D_all = np.load(orc)["motion"]
    cache = orc.parent / "degeneracy_H.npz"
    if cache.exists():
        c = np.load(cache); lam, vec = c["lam"], c["vec"]
    else:
        ds, _ = dataset_for(key)
        lam, vec = info_H(ds, min(len(D_all), len(ds)), every)
        np.savez(cache, lam=lam, vec=vec)
    print(f"{key}: lam p10 / p50 / p90 {np.nanpercentile(lam, 10):.3f} / {np.nanmedian(lam):.3f} / {np.nanpercentile(lam, 90):.3f}")
    for run in runs:
        f = sorted(Path(run).glob("*/image_motions.npz"))[-1]
        d = np.load(f); P = np.load(next(f.parent.glob("*_poses.npy")))
        n = min(len(P), len(D_all), len(lam))
        M = np.full((n, 4, 4), np.nan); s = d["scan"] < n; M[d["scan"][s]] = d["motion"][s]
        I = np.full((n, 4, 4), np.nan); I[1:] = np.linalg.inv(P[:n - 1]) @ P[1:n]
        D = D_all[:n]
        ok = np.isfinite(D).all(axis=(1, 2)) & np.isfinite(I).all(axis=(1, 2)) & np.isfinite(M).all(axis=(1, 2)) & np.isfinite(lam[:n])
        ok[0] = False
        v = vec[:n]
        def along(X):
            e = X[:, :3, 3] - D[:, :3, 3]; a = np.abs(np.einsum("ij,ij->i", e, v)) * 1000
            return a, np.linalg.norm(e - np.einsum("ij,ij->i", e, v)[:, None] * v, axis=1) * 1000
        ai, ci = along(I); am, cm = along(M)
        ri = np.full(n, np.nan); rm = np.full(n, np.nan)
        ri[ok] = rot_deg(np.linalg.inv(D[ok]) @ I[ok]); rm[ok] = rot_deg(np.linalg.inv(D[ok]) @ M[ok])
        q = np.nanquantile(lam[:n][ok], [1 / 3, 2 / 3])
        name = "/".join(Path(run).parts[-2:])
        out = []
        for lo, hi, lab in [(-1, q[0], "weak"), (q[0], q[1], "mid"), (q[1], 9, "strong")]:
            b = ok & (lam[:n] > lo) & (lam[:n] <= hi)
            out.append(f"{lab}: along v ICP {np.median(ai[b]):5.1f} / img {np.median(am[b]):5.1f} mm, across {np.median(ci[b]):5.1f} / {np.median(cm[b]):5.1f}, "
                       f"rot {np.median(ri[b]):.2f} / {np.median(rm[b]):.2f}")
        print(f"  {name}  " + " | ".join(out))
        for pq in (10, 25):
            tau = np.nanpercentile(lam[:n][ok], pq)
            F = I.copy(); sel = ok & (lam[:n] <= tau)
            dt = np.einsum("ij,ij->i", M[sel, :3, 3] - I[sel, :3, 3], v[sel])
            F[sel, :3, 3] = I[sel, :3, 3] + dt[:, None] * v[sel]
            res = []
            for lab, X in (("ICP", I), ("fused", F), ("image", M)):
                wt = []
                for k in range(1, n - win + 1):
                    if ok[k:k + win].all() and sel[k:k + win].any():
                        Xc, Dc = np.eye(4), np.eye(4)
                        for j in range(k, k + win):
                            Xc, Dc = Xc @ X[j], Dc @ D[j]
                        wt.append(np.linalg.norm((np.linalg.inv(Dc) @ Xc)[:3, 3]) * 1000)
                e = X[sel, :3, 3] - D[sel, :3, 3]
                res.append(f"{lab} scan {np.median(np.linalg.norm(e, axis=1) * 1000):5.1f} / {win} scans {np.median(wt):6.1f} mm")
            print(f"    fusion on the weakest {pq:2d} % ({int(sel.sum())} scans): " + " · ".join(res))


if __name__ == "__main__":
    main()
