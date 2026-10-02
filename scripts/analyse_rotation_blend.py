#!/usr/bin/env python3
"""Would a weighted image rotation deskew better? Offline check against the ground-truth sweep motion (#092).

    python scripts/analyse_rotation_blend.py <run dir> <oracle_motion.npz> [<run dir> <oracle_motion.npz> ...]

Per scan k deskewed with the image motion (pose_times.csv fraction = 1), the rotation during the sweep from
  image  M_k (image_motions.npz, as the estimator gave it)
  icp    P_{k-1}^-1 P_k (the run's poses: the ICP after the image deskew; pose k at the end of sweep k)
  cv     P_{k-2}^-1 P_{k-1} (the constant-velocity prediction of KISS)
and blends R(w) = R_img exp(w log(R_img^T R_other)), w = 0 (image) ... 1 (other), each against the ground truth
D_k (make_oracle_motion.py).  The poses are after the pose graph: pairs across a node boundary are rare and
the medians robust to them.  Also a per-scan weight from the image inliers, w = n0 / (n0 + inliers).
"""
import csv
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

W = tuple(float(w) for w in __import__("os").environ.get("BLEND_W", "0,0.1,0.2,0.3,0.5,0.7,1").split(","))


def rot_err(A, B):
    return np.degrees(np.linalg.norm(R.from_matrix(np.transpose(A, (0, 2, 1)) @ B).as_rotvec(), axis=1))


def blend(Ra, Rb, w):
    w = np.broadcast_to(np.asarray(w, float), (len(Ra),))
    rv = R.from_matrix(np.transpose(Ra, (0, 2, 1)) @ Rb).as_rotvec() * w[:, None]
    return Ra @ R.from_rotvec(rv).as_matrix()


def one(run, orc):
    run = Path(run)
    d = sorted(run.glob("*/image_motions.npz"))[-1]
    rd = d.parent
    im = np.load(d)
    rows = list(csv.DictReader(open(rd / "pose_times.csv")))
    frac = np.array([float(r["fraction"]) for r in rows])
    P = np.load(next(rd.glob("*_poses.npy")))
    D = np.load(orc)["motion"]
    M = np.full((len(rows), 4, 4), np.nan); M[im["scan"]] = im["motion"]
    n = np.full(len(rows), np.nan); n[im["scan"]] = im["inliers"]
    k = np.arange(2, min(len(P), len(D), len(rows)))
    ok = (frac[k] == 1.0) & np.isfinite(M[k]).all(axis=(1, 2)) & np.isfinite(D[k]).all(axis=(1, 2))
    k = k[ok]
    Ri, Rd = M[k, :3, :3], D[k, :3, :3]
    Ricp = (np.linalg.inv(P[k - 1]) @ P[k])[:, :3, :3]
    Rcv = (np.linalg.inv(P[k - 2]) @ P[k - 1])[:, :3, :3]
    out = {"name": f"{run.parent.name}/{run.name}", "n": len(k), "truth": np.median(np.linalg.norm(R.from_matrix(Rd).as_rotvec(), axis=1)) * 180 / np.pi}
    out["img"], out["icp"], out["cv"] = (np.median(rot_err(Rd, X)) for X in (Ri, Ricp, Rcv))
    # are the image and icp errors independent? correlation of the error vectors
    ei = R.from_matrix(np.transpose(Rd, (0, 2, 1)) @ Ri).as_rotvec()
    ec = R.from_matrix(np.transpose(Rd, (0, 2, 1)) @ Ricp).as_rotvec()
    out["corr_img_icp"] = np.corrcoef(ei.ravel(), ec.ravel())[0, 1]
    for name, Ro in (("icp", Ricp), ("cv", Rcv)):
        out[f"blend_{name}"] = [np.median(rot_err(Rd, blend(Ri, Ro, w))) for w in W]
    nn = n[k]
    out["inl"] = {n0: np.median(rot_err(Rd, blend(Ri, Rcv, n0 / (n0 + nn)))) for n0 in (10, 30, 100)}
    return out


def main():
    a = sys.argv[1:]
    res = [one(a[i], a[i + 1]) for i in range(0, len(a), 2)]
    print("median rotation error per sweep [deg]; blend weight w on the other estimate (0 = image)")
    print(f"{'run':38s} {'n':>5s} {'truth':>6s} {'img':>6s} {'icp':>6s} {'cv':>6s} {'r':>5s} | " +
          " ".join(f"icp{w:.1f}" for w in W) + " | " + " ".join(f"cv{w:.1f}" for w in W) + " | inl-cv n0=10/30/100")
    for r in res:
        print(f"{r['name']:38s} {r['n']:5d} {r['truth']:6.2f} {r['img']:6.3f} {r['icp']:6.3f} {r['cv']:6.3f} {r['corr_img_icp']:5.2f} | " +
              " ".join(f"{v:6.3f}" for v in r["blend_icp"]) + " | " + " ".join(f"{v:5.3f}" for v in r["blend_cv"]) + " | " +
              " ".join(f"{v:.3f}" for v in r["inl"].values()))
    if len(res) > 1:
        rel = lambda key: np.median([np.array(r[key]) / r["img"] for r in res], axis=0)
        print("median over runs, relative to image: icp blend " + " ".join(f"{v:.3f}" for v in rel("blend_icp")) +
              " | cv blend " + " ".join(f"{v:.3f}" for v in rel("blend_cv")))


if __name__ == "__main__":
    main()
