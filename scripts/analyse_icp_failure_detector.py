#!/usr/bin/env python3
"""Detect where the ICP is failing from its CONSISTENT disagreement with the image motion, and use the image translation there - offline,
against the ground-truth sweep motion (#129; follows #128: the image beats the ICP where the ICP fails, but the geometry's weak
direction does not find those places).

    python scripts/analyse_icp_failure_detector.py <oracle_motion.npz> <run dir> [<run dir> ...] [--w=10] [--win=10]

Per scan k: d_k = R_{k-1} (t_icp,k - t_img,k), the translation disagreement in the world frame (R = the ICP pose rotation).  Over the
last w scans (causal): S_k = |sum d_j|, consistency rho_k = S_k / sum |d_j| (random disagreement ~ 1/sqrt(w), steady drift of one of the
two -> 1).  Flag: rho_k > r and S_k > c x the running median of S (the sequence's own scale, no absolute threshold).  Flagged scans take
the image translation (rotation stays the ICP's).  Reported: share flagged, how often the flag is right (the image closer to the truth
over that window), translation error over `win` scans for ICP / flagged-fusion / image-only, over ALL windows; and the ceiling: flag
where the image is better over the window (from the GT).
"""
import sys
from pathlib import Path

import numpy as np


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    o = dict(a[2:].split("=") for a in sys.argv[1:] if a.startswith("--"))
    w, win = int(o.get("w", 10)), int(o.get("win", 10))
    D_all = np.load(args[0])["motion"]
    print(f"{'run / variant':44s} {'flag %':>6s} {'right %':>7s} {'tr/' + str(win) + ' p50':>9s} {'p90':>7s}  [mm]")
    for run in args[1:]:
        f = sorted(Path(run).glob("*/image_motions.npz"))[-1]
        d = np.load(f); P = np.load(next(f.parent.glob("*_poses.npy")))
        n = min(len(P), len(D_all))
        M = np.full((n, 4, 4), np.nan); s = d["scan"] < n; M[d["scan"][s]] = d["motion"][s]
        I = np.full((n, 4, 4), np.nan); I[1:] = np.linalg.inv(P[:n - 1]) @ P[1:n]
        D = D_all[:n]
        ok = np.isfinite(D).all(axis=(1, 2)) & np.isfinite(I).all(axis=(1, 2)) & np.isfinite(M).all(axis=(1, 2)); ok[0] = False
        dd = np.zeros((n, 3))
        dd[ok] = np.einsum("kij,kj->ki", P[np.flatnonzero(ok) - 1][:, :3, :3], I[ok, :3, 3] - M[ok, :3, 3])
        cs = np.cumsum(dd, axis=0); cn = np.cumsum(np.linalg.norm(dd, axis=1))
        S = np.zeros(n); rho = np.zeros(n)
        for k in range(w, n):
            S[k] = np.linalg.norm(cs[k] - cs[k - w]); rho[k] = S[k] / max(cn[k] - cn[k - w], 1e-9)
        tw = np.zeros((n, 3)); tw[ok] = np.einsum("kij,kj->ki", P[np.flatnonzero(ok) - 1][:, :3, :3], I[ok, :3, 3]); ct = np.cumsum(tw, axis=0)
        rel = np.zeros(n)
        for k in range(w, n):
            rel[k] = S[k] / max(np.linalg.norm(ct[k] - ct[k - w]), 0.05)   # disagreement / distance travelled over the window
        medS = np.array([np.median(S[w:k]) if k > w + 20 else np.inf for k in range(n)])

        def window_err(X):
            e = np.full(n, np.nan)
            for k in range(1, n - win + 1):
                if ok[k:k + win].all():
                    Xc, Dc = np.eye(4), np.eye(4)
                    for j in range(k, k + win):
                        Xc, Dc = Xc @ X[j], Dc @ D[j]
                    e[k] = np.linalg.norm((np.linalg.inv(Dc) @ Xc)[:3, 3]) * 1000
            return e

        def fuse(flag):
            F = I.copy(); F[flag & ok, :3, 3] = M[flag & ok, :3, 3]; return F

        eI, eM = window_err(I), window_err(M)
        better = np.zeros(n, bool)                                  # the image is better over the window that ENDS at k
        better[win:] = eM[1:n - win + 1] < eI[1:n - win + 1]
        name = "/".join(Path(run).parts[-2:])
        rows = [("ICP", np.zeros(n, bool)), ("image only", ok.copy())]
        for r in (0.6, 0.8):
            for c in (2.0, 3.0):
                rows.append((f"flag rho>{r:g} S>{c:g}xmed", ok & (rho > r) & (S > c * medS)))
        for r in (0.4, 0.6):
            rows.append((f"flag rho>{r:g} alone", ok & (rho > r)))
        for q in (0.05, 0.10, 0.20):
            rows.append((f"flag drift>{100 * q:g}% of path", ok & (rel > q)))
            rows.append((f"flag drift>{100 * q:g}% & rho>0.4", ok & (rel > q) & (rho > 0.4)))
        rows.append(("ceiling (GT: image better)", ok & better))
        for lab, flag in rows:
            e = window_err(fuse(flag))
            fl = flag & ok
            right = 100 * better[fl].mean() if fl.any() and lab.startswith("flag") else np.nan
            print(f"{name + ' ' + lab:44s} {100 * fl[ok].mean():6.1f} {right:7.0f} {np.nanmedian(e):9.1f} {np.nanpercentile(e, 90):7.1f}")


if __name__ == "__main__":
    main()
