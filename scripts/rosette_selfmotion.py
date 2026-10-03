#!/usr/bin/env python3
"""Motion within the sweep from the scan ITSELF on a non-repetitive (rosette) LiDAR (#116, open_tasks B.10).  Offline check, no SLAM.

    python scripts/rosette_selfmotion.py <TIERS bag> <optitrack.csv> [--kinds=avia,horizon] [--gap=0.25] [--frames=423] [--alpha] [--out=<prefix>]

A rosette scanner revisits the same directions several times within one 0.1 s sweep (every petal crosses the centre); a spinning LiDAR never
does.  So the sweep's own points can measure its motion: with a constant-velocity motion (w, v) over the sweep, every point goes to the
frame at the START of the sweep, x_i = Exp(s_i w) p_i + s_i v (s_i = time in sweeps).  The right (w, v) makes the sweep self-consistent:
points measured at different times (|s_i - s_k| >= gap) land on the same surfaces.  Cost: point-to-plane distance from x_i to its nearest
neighbour x_k among the points at least `gap` apart in time, normal from the 8 nearest points around x_k, Geman-McClure weights, Gauss-
Newton from zero motion (10 iterations).  First-order Jacobians: d x_i / d w = -s_i [p_i]x, d x_i / d v = s_i I - only the time difference
s_i - s_k enters, which is why the revisits make the motion observable.  --alpha: also a constant angular acceleration (s^2 / 2 alpha).
Score per scan against the sensor's own gyro (solid_state_check.score_gyro): rotation over the sweep.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parent))
from solid_state_check import read_bag, score_gyro, AVIA_TOPIC, HORIZON_TOPIC  # noqa: E402


def skew(v):
    S = np.zeros(v.shape[:-1] + (3, 3))
    S[..., 0, 1], S[..., 0, 2], S[..., 1, 2] = -v[..., 2], v[..., 1], -v[..., 0]
    return S - np.swapaxes(S, -1, -2)


def self_motion(xyz, s, gap=0.25, iters=10, alpha=False, kernel=0.05, max_pts=8000, rng=np.random.default_rng(0)):
    """(4x4 motion over the sweep: pose of the end in the start frame, number of constraints)."""
    r = np.linalg.norm(xyz, axis=1)
    ok = (r > 1.0) & (r < 60.0)
    xyz, s = xyz[ok], s[ok]
    if len(xyz) > max_pts:
        k = rng.choice(len(xyz), max_pts, replace=False); xyz, s = xyz[k], s[k]
    npar = 9 if alpha else 6
    x = np.zeros(npar)
    n_used = 0
    for _ in range(iters):
        w, v = x[:3], x[3:6]
        phi = s[:, None] * w + (0.5 * s[:, None] ** 2 * x[6:9] if alpha else 0.0)
        X = R.from_rotvec(phi).apply(xyz) + s[:, None] * v
        tree = cKDTree(X)
        dist, idx = tree.query(X, k=12)
        ds = np.abs(s[idx] - s[:, None])
        cand = (ds >= gap) & np.isfinite(dist) & (dist < 0.5)
        first = np.where(cand.any(1), cand.argmax(1), -1)
        use = first >= 0
        i = np.flatnonzero(use); kk = idx[i, first[use]]
        if len(i) < 50:
            return None, len(i)
        _, nb = tree.query(X[kk], k=8)                                 # normals around the partner
        C = X[nb] - X[nb].mean(1, keepdims=True)
        cov = np.einsum("nki,nkj->nij", C, C)
        eva, eve = np.linalg.eigh(cov)
        nrm = eve[:, :, 0]
        planar = eva[:, 0] < 0.05 * eva[:, 1]
        i, kk, nrm = i[planar], kk[planar], nrm[planar]
        if len(i) < 50:
            return None, len(i)
        res = np.einsum("ni,ni->n", nrm, X[i] - X[kk])
        wgt = kernel ** 2 / (kernel + res ** 2) ** 2
        def J_of(j):                                                  # d X_j / d params, (n, 3, npar), first order
            Jj = np.zeros((len(j), 3, npar))
            Rp = X[j] - s[j, None] * v
            Jj[:, :, 0:3] = -s[j, None, None] * skew(Rp)
            Jj[:, :, 3:6] = s[j, None, None] * np.eye(3)
            if alpha:
                Jj[:, :, 6:9] = -0.5 * (s[j] ** 2)[:, None, None] * skew(Rp)
            return Jj
        J = np.einsum("ni,nij->nj", nrm, J_of(i) - J_of(kk))
        H = (J * wgt[:, None]).T @ J
        g = (J * wgt[:, None]).T @ res
        dx = -np.linalg.solve(H + 1e-6 * np.eye(npar), g)
        x = x + dx
        n_used = len(i)
        rms = float(np.sqrt(np.average(res ** 2, weights=wgt)))
        if np.linalg.norm(dx) < 1e-6:
            break
    phi1 = x[:3] + (0.5 * x[6:9] if alpha else 0.0)
    M = np.eye(4); M[:3, :3] = R.from_rotvec(phi1).as_matrix(); M[:3, 3] = x[3:6]
    self_motion.last_rms = rms
    return M, n_used


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    frames = read_bag(args[0], int(opts.get("frames", 423)))
    gaps = [float(g) for g in opts.get("gap", "0.25,0.5").split(",")]
    for kind in opts.get("kinds", "avia,horizon").split(","):
        off = frames["clock"][AVIA_TOPIC if kind == "avia" else HORIZON_TOPIC]
        for gap in gaps:
            for alpha in ([False, True] if "--alpha" in sys.argv else [False]):
                times, motions, used, rmss = [], [], [], []
                for stamp, xyz, ts, inten, line in frames[kind][1:]:
                    t0 = float(ts.min()); s = np.clip((ts - t0) / 0.1, 0, 1)
                    self_motion.last_rms = np.nan
                    M, n = self_motion(xyz, s, gap=gap, alpha=alpha)
                    times.append((t0, t0 + 0.1)); motions.append(M); used.append(n); rmss.append(self_motion.last_rms)
                if "out" in opts:                            # motion file for run_ncd --motion-file (scan 0 has none), + constraints / residual
                    mot = np.full((len(frames[kind]), 4, 4), np.nan)
                    for j, M in enumerate(motions):
                        if M is not None:
                            mot[j + 1] = M
                    np.savez(f"{opts['out']}_{kind}_gap{gap:g}{'_alpha' if alpha else ''}.npz", motion=mot, n=np.r_[0, used],
                             rms=np.r_[np.nan, rmss], times=np.array(times), off=frames["clock"][AVIA_TOPIC if kind == "avia" else HORIZON_TOPIC])
                score_gyro(f"{kind} self, gap {gap:g}{' +alpha' if alpha else ''}", [(a - off, b - off) for a, b in times], motions, used,
                           frames["imu"][kind])
                print(f"{'':28s} constraints p50 {np.median(used):.0f}", flush=True)


if __name__ == "__main__":
    main()
