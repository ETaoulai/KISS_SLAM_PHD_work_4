#!/usr/bin/env python3
"""Image-motion accuracy per scan AND over windows of consecutive scans, against the ground-truth sweep motion (#093).

    python scripts/analyse_motion_windows.py <oracle_motion.npz> <run dir> [<run dir> ...] [--win=10]

For each run (image_motions.npz, any run since #086): the rotation / translation error of every image motion M_k against the
ground truth D_k (make_oracle_motion.py), and of the composed motion over `win` consecutive scans (M_k ... M_{k+win-1} against
D_k ... D_{k+win-1}; windows with a failed scan skipped).  #092: a lower error per scan does not make a better trajectory - the
error that matters is the part correlated over many scans, which the window error weighs (random noise grows ~sqrt(win), a
correlated error ~win).  Medians; failures counted.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


def rot_deg(A):
    return np.degrees(np.linalg.norm(R.from_matrix(A[..., :3, :3]).as_rotvec(), axis=-1))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    win = int(dict(a[2:].split("=") for a in sys.argv[1:] if a.startswith("--win=")).get("win", 10))
    D_all = np.load(args[0])["motion"]
    print(f"{'run':44s} {'scans':>5s} {'fail':>4s} {'rot/scan':>8s} {'tr/scan':>7s} {'rot/' + str(win):>8s} {'tr/' + str(win):>7s}   [deg, mm]")
    for run in args[1:]:
        f = sorted(Path(run).glob("*/image_motions.npz"))[-1]
        d = np.load(f)
        n = int(d["scan"].max()) + 1
        M = np.full((n, 4, 4), np.nan); M[d["scan"]] = d["motion"]
        D = D_all[:n]
        ok = np.isfinite(M).all(axis=(1, 2)) & np.isfinite(D).all(axis=(1, 2))
        ok[0] = False                                             # no motion for the first scan
        E = np.linalg.inv(D[ok]) @ M[ok]
        er, et = rot_deg(E), np.linalg.norm(E[:, :3, 3], axis=1) * 1000
        wr, wt = [], []
        for k in range(1, n - win + 1):
            if ok[k:k + win].all():
                Mc, Dc = np.eye(4), np.eye(4)
                for j in range(k, k + win):
                    Mc, Dc = Mc @ M[j], Dc @ D[j]
                Ew = np.linalg.inv(Dc) @ Mc
                wr.append(rot_deg(Ew)); wt.append(np.linalg.norm(Ew[:3, 3]) * 1000)
        fails = int((~np.isfinite(M[1:]).all(axis=(1, 2))).sum())
        name = f"{Path(run).parent.name}/{Path(run).name}"
        print(f"{name:44s} {n:5d} {fails:4d} {np.median(er):8.3f} {np.median(et):7.1f} {np.median(wr):8.3f} {np.median(wt):7.1f}")


if __name__ == "__main__":
    main()
