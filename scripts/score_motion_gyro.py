#!/usr/bin/env python3
"""Image rotation per scan and over windows of consecutive scans against the LiDAR's own gyroscope - for sequences
without a dense ground truth (Hilti 2021, NTU VIRAL), #126.

    python scripts/score_motion_gyro.py <bag or bag dir> <imu topic> <reference run dir> <run dir> [<run dir> ...] [--win=10]

Each run's image_motions.npz holds the motion during every sweep (pose_times.csv: stamp = first point, span_s = sweep length),
in the LiDAR frame.  The gyro rotation over the same interval is mapped into the LiDAR frame by a fixed rotation fitted
(Kabsch on rotation vectors) to the reference run's pose increments - so the extrinsic and the clock offset (searched over
+-60 ms) come from the ICP trajectory, never from the variants being compared.  Gyro bias: median of (gyro - pose rate) over
the sequence.  Only rotation: a gyro gives no translation.  Medians, degrees.
"""
import csv
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation as R


def read_imu(bag, topic):
    p = Path(bag)
    paths = [p] if p.is_file() else (sorted(p.glob("*.bag")) or [p])
    out = []
    with AnyReader(paths) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == topic]):
            m = r.deserialize(raw, c.msgtype)
            out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    imu = np.array(out)
    return imu[np.argsort(imu[:, 0])]


def gyro_rotvecs(imu, t0, t1, bias):
    """Rotation vector of the gyro rotation over each [t0, t1]; trapezoid on a fine grid of the interpolated rate."""
    t, w = imu[:, 0], imu[:, 1:4] - bias
    out = np.full((len(t0), 3), np.nan)
    for i, (a, b) in enumerate(zip(t0, t1)):
        if a < t[0] or b > t[-1] or not b > a:
            continue
        g = np.r_[a, t[(t > a) & (t < b)], b]
        ww = np.stack([np.interp(g, t, w[:, j]) for j in range(3)], axis=1)
        Rm = R.identity()
        for k in range(len(g) - 1):
            Rm = Rm * R.from_rotvec(0.5 * (ww[k] + ww[k + 1]) * (g[k + 1] - g[k]))
        out[i] = Rm.as_rotvec()
    return out


def kabsch(a, b):
    """Rotation Q with b ~ Q a (rows are vectors)."""
    U, _, Vt = np.linalg.svd(b.T @ a)
    S = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return U @ S @ Vt


def times(run):
    rd = sorted(Path(run).glob("*/pose_times.csv"))[-1].parent
    rows = list(csv.DictReader(open(rd / "pose_times.csv")))
    st = np.array([float(r["stamp"]) for r in rows]); sp = np.array([float(r["span_s"]) for r in rows])
    pt = np.array([float(r["pose_time"]) for r in rows])
    return rd, st, sp, pt


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    win = int(dict(a[2:].split("=") for a in sys.argv[1:] if a.startswith("--")).get("win", 10))
    bag, topic, ref, runs = args[0], args[1], args[2], args[3:]
    imu = read_imu(bag, topic)

    # Extrinsic, clock offset and bias from the reference run's pose increments.
    rd, _, _, pt = times(ref)
    P = np.load(next(rd.glob("*_poses.npy")))[: len(pt)]
    dP = np.array([R.from_matrix(P[j - 1][:3, :3].T @ P[j][:3, :3]).as_rotvec() for j in range(1, len(P))])
    a, b = pt[:-1], pt[1:]
    big = np.linalg.norm(dP, axis=1) > np.radians(0.3)
    best = None
    for off in np.arange(-0.06, 0.0601, 0.005):
        g = gyro_rotvecs(imu, a[big] + off, b[big] + off, 0.0)
        k = np.isfinite(g[:, 0])
        Q = kabsch(g[k], dP[big][k])
        res = np.median(np.linalg.norm(g[k] @ Q.T - dP[big][k], axis=1))
        if best is None or res < best[0]:
            best = (res, off, Q)
    _, off, Q = best
    rate_p = dP / (b - a)[:, None]
    rate_g = np.stack([np.interp(0.5 * (a + b) + off, imu[:, 0], imu[:, j]) for j in range(1, 4)], axis=1)
    bias = np.median(rate_g - rate_p @ Q, axis=0)  # in the IMU frame (Q maps IMU -> LiDAR)
    print(f"IMU {topic}: {len(imu)} samples, {1 / np.median(np.diff(imu[:, 0])):.0f} Hz · clock offset {off * 1e3:+.0f} ms · "
          f"bias {np.degrees(np.linalg.norm(bias)):.3f} deg/s · extrinsic angle {np.degrees(np.linalg.norm(R.from_matrix(Q).as_rotvec())):.1f} deg")

    print(f"{'run':44s} {'scans':>5s} {'fail':>4s} {'rot/scan':>8s} {'p90':>6s} {'rot/' + str(win):>8s} {'p90':>6s} {'inl p50':>7s}   [deg]")
    for run in runs:
        rd, st, sp, _ = times(run)
        d = np.load(sorted(Path(run).glob("*/image_motions.npz"))[-1])
        M, inl, sc = d["motion"], d["inliers"], d["scan"]
        ok = np.isfinite(M[:, 0, 0]) & (inl > 0)
        t0, t1 = st[sc] + off, st[sc] + sp[sc] + off
        G = gyro_rotvecs(imu, t0, t1, bias) @ Q.T
        e1 = [np.degrees(np.linalg.norm((R.from_rotvec(G[i]).inv() * R.from_matrix(M[i][:3, :3])).as_rotvec()))
              for i in range(len(M)) if ok[i] and np.isfinite(G[i, 0])]
        ew = []
        Gw = gyro_rotvecs(imu, t0[:-win + 1], t1[win - 1:], bias) @ Q.T
        for i in range(len(M) - win + 1):
            if not ok[i:i + win].all() or not np.isfinite(Gw[i, 0]) or sc[i + win - 1] - sc[i] != win - 1:
                continue
            C = np.eye(3)
            for k in range(i, i + win):
                C = C @ M[k][:3, :3]
            ew.append(np.degrees(np.linalg.norm((R.from_rotvec(Gw[i]).inv() * R.from_matrix(C)).as_rotvec())))
        name = "/".join(Path(run).parts[-2:])
        print(f"{name:44s} {len(M):5d} {int((~ok).sum()):4d} {np.median(e1):8.3f} {np.percentile(e1, 90):6.3f} "
              f"{np.median(ew):8.3f} {np.percentile(ew, 90):6.3f} {np.median(inl[inl > 0]):7.0f}")


if __name__ == "__main__":
    main()
