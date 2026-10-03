#!/usr/bin/env python3
"""Image motion only where the motion is erratic, constant velocity elsewhere - offline, against the ground-truth sweep motion (#127,
idea M.T. 3/10).

    python scripts/analyse_gated_deskew.py <oracle_motion.npz> <run dir> [<run dir> ...] [--win=10] [--c=1.5,2,3] [--w=50]

Per scan k of a run: the image motion M_k (image_motions.npz) and the constant-velocity prediction C_k = P_{k-2}^-1 P_{k-1} (the run's
own previous pose step, as KISS predicts it).  Their disagreement d_k = angle(C_k^-1 M_k) is compared with the sequence's own usual
disagreement s_k = median of d over the previous w scans (causal, no threshold in degrees or m/s - nothing tied to the platform):
d_k > c s_k -> the motion changed more than usual -> image; otherwise constant velocity.  An image failure -> constant velocity.
Reported (medians, deg / mm, per scan and over `win` consecutive scans, composed): image always, constant velocity always, the gate
for every c, and the per-scan oracle choice (whichever is closer to the GT - a ceiling, not a method); plus the share of scans on
the image, and image vs constant-velocity error binned by the true change of motion (angle(D_{k-1}^-1 D_k), terciles).
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


def rot_deg(A):
    return np.degrees(np.linalg.norm(R.from_matrix(A[..., :3, :3]).as_rotvec(), axis=-1))


def errors(M, D, ok, win):
    E = np.linalg.inv(D[ok]) @ M[ok]
    er, et = rot_deg(E), np.linalg.norm(E[:, :3, 3], axis=1) * 1000
    wr, wt = [], []
    for k in range(2, len(M) - win + 1):
        if ok[k:k + win].all():
            Mc, Dc = np.eye(4), np.eye(4)
            for j in range(k, k + win):
                Mc, Dc = Mc @ M[j], Dc @ D[j]
            Ew = np.linalg.inv(Dc) @ Mc
            wr.append(rot_deg(Ew)); wt.append(np.linalg.norm(Ew[:3, 3]) * 1000)
    return np.median(er), np.median(et), np.median(wr), np.median(wt)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    o = dict(a[2:].split("=") for a in sys.argv[1:] if a.startswith("--"))
    win, W = int(o.get("win", 10)), int(o.get("w", 50))
    cs = [float(c) for c in o.get("c", "1.5,2,3").split(",")]
    D_all = np.load(args[0])["motion"]
    print(f"{'run / variant':40s} {'image %':>7s} {'rot/scan':>8s} {'tr/scan':>7s} {'rot/' + str(win):>8s} {'tr/' + str(win):>7s}   [deg, mm]")
    for run in args[1:]:
        f = sorted(Path(run).glob("*/image_motions.npz"))[-1]
        d = np.load(f)
        P = np.load(next(f.parent.glob("*_poses.npy")))
        n = min(len(P), int(d["scan"].max()) + 1, len(D_all))
        M = np.full((n, 4, 4), np.nan); s = d["scan"] < n; M[d["scan"][s]] = d["motion"][s]
        D = D_all[:n]
        C = np.full((n, 4, 4), np.nan)
        C[2:] = np.linalg.inv(P[:n - 2]) @ P[1:n - 1]
        good = np.isfinite(D).all(axis=(1, 2)) & np.isfinite(C).all(axis=(1, 2))
        img_ok = np.isfinite(M).all(axis=(1, 2))
        dis = np.full(n, np.nan)
        dis[img_ok & good] = rot_deg(np.linalg.inv(C[img_ok & good]) @ M[img_ok & good])
        name = "/".join(Path(run).parts[-2:])
        rows = [("image (fail -> CV)", np.where(img_ok[:, None, None], M, C), img_ok), ("constant velocity", C, np.zeros(n, bool))]
        for c in cs:
            use = np.zeros(n, bool)
            for k in range(n):
                if not np.isfinite(dis[k]):
                    continue
                past = dis[max(0, k - W):k]; past = past[np.isfinite(past)]
                use[k] = len(past) < 10 or dis[k] > c * np.median(past)
            rows.append((f"gate c={c:g}", np.where(use[:, None, None], M, C), use))
        # #131 adaptive, no threshold: each predictor's recent error against the ICP result of the previous scans (causal: the ICP of
        # scan j is known before scan k > j is deskewed).  "track-best": the predictor with the lower mean error over the last w scans;
        # "blend": rotation / translation interpolated with the inverse-variance weight of the two recent errors.
        Iq = np.full((n, 4, 4), np.nan); Iq[1:] = np.linalg.inv(P[:n - 1]) @ P[1:n]
        gi = img_ok & np.isfinite(Iq).all(axis=(1, 2)) & np.isfinite(C).all(axis=(1, 2))
        ri, rc = np.full(n, np.nan), np.full(n, np.nan)
        ri[gi] = rot_deg(np.linalg.inv(Iq[gi]) @ M[gi]); rc[gi] = rot_deg(np.linalg.inv(Iq[gi]) @ C[gi])
        from scipy.spatial.transform import Slerp
        for ww in (5, 20):
            pick = np.zeros(n, bool); B = C.copy(); wimg = np.zeros(n)
            for k in range(n):
                if not img_ok[k] or not np.isfinite(C[k]).all():
                    continue
                a, b = ri[max(0, k - ww):k], rc[max(0, k - ww):k]; m = np.isfinite(a) & np.isfinite(b)
                if m.sum() < 3:
                    pick[k] = True; wimg[k] = 1.0; B[k] = M[k]; continue
                vi, vc = np.mean(a[m] ** 2) + 1e-9, np.mean(b[m] ** 2) + 1e-9
                pick[k] = vi <= vc
                wimg[k] = vc / (vi + vc)
                sl = Slerp([0, 1], R.from_matrix(np.stack([C[k][:3, :3], M[k][:3, :3]])))
                B[k] = np.eye(4); B[k][:3, :3] = sl(wimg[k]).as_matrix(); B[k][:3, 3] = (1 - wimg[k]) * C[k][:3, 3] + wimg[k] * M[k][:3, 3]
            rows.insert(-1, (f"track-best w={ww}", np.where(pick[:, None, None], M, C), pick))
            rows.insert(-1, (f"blend w={ww}", B, wimg > 0.5))
        ei = np.full(n, np.inf); ec = np.full(n, np.inf)
        ei[img_ok & good] = rot_deg(np.linalg.inv(D[img_ok & good]) @ M[img_ok & good])
        ec[good] = rot_deg(np.linalg.inv(D[good]) @ C[good])
        orc = ei < ec
        rows.append(("oracle choice (ceiling)", np.where(orc[:, None, None], M, C), orc))
        for lab, X, use in rows:
            e = errors(X, D, good, win)
            print(f"{name + ' ' + lab:40s} {100 * use[good].mean():7.0f} {e[0]:8.3f} {e[1]:7.1f} {e[2]:8.3f} {e[3]:7.1f}")
        ch = np.full(n, np.nan); fd = np.isfinite(D).all(axis=(1, 2)); b2 = np.r_[False, fd[:-1] & fd[1:]]
        ch[b2] = rot_deg(np.linalg.inv(D[np.r_[b2[1:], False]]) @ D[b2])
        k = good & img_ok & np.isfinite(ch)
        q = np.quantile(ch[k], [1 / 3, 2 / 3])
        parts = []
        for lo, hi, lab in [(-1, q[0], "calm"), (q[0], q[1], "mid"), (q[1], 1e9, "erratic")]:
            b = k & (ch > lo) & (ch <= hi)
            parts.append(f"{lab} (change <= {min(hi, ch[k].max()):.2f} deg): image {np.median(ei[b]):.3f} / CV {np.median(ec[b]):.3f}")
        print(f"{'':40s} rotation per scan by true change of motion - " + " · ".join(parts))


if __name__ == "__main__":
    main()
