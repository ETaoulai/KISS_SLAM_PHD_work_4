#!/usr/bin/env python3
"""TIERS (Livox) runs against the motion capture and the sensor's own gyro (#101).  No official protocol exists for this pairing.

    python scripts/evaluate_tiers.py <bag> <optitrack.csv> <avia|horizon> <reference run dir> <run dir> [<run dir> ...]

Per run, from its poses (*_poses.npy) at their own instants (pose_times.csv, ROS time via the bag clock, livox.py):
  - APE: position RMSE against the mocap after an SE(3) Umeyama alignment of the positions (no scale).  The mocap measures the body,
    not the LiDAR: the unknown lever arm adds the same error to every arm.  Clock: one residual offset per sensor, the one that
    minimises the APE of the REFERENCE run (search +-0.3 s, 5 ms), then applied to every run (no per-arm advantage).
  - RPE 1 s rotation: angle between the run's relative rotation over ~1 s (pose pairs 10 scans apart) and the sensor's gyro integrated
    over the same interval (same frame and clock as the LiDAR: no calibration; bias from the first 1 s at rest).
  - RPE 1 s distance: | |dp_run| - |dp_mocap| | over the same pairs (frame-free), and the path length against the mocap.
One seed per arm (the image motions are precomputed): a single-run APE has sd ~0.02-0.04 m (#037) - rely on the RPE.
"""
import csv
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parent))
from solid_state_check import gyro_rotation, IMU_TOPICS, AVIA_TOPIC, HORIZON_TOPIC  # noqa: E402


def load_run(run):
    rd = sorted(Path(run).glob("*/pose_times.csv"))[-1].parent
    P = np.load(next(rd.glob("*_poses.npy")))
    rows = list(csv.DictReader(open(rd / "pose_times.csv")))
    return np.array([float(r["pose_time"]) for r in rows]), P


def umeyama(A, B):
    """R, t with B ~ R A + t (rows are points)."""
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    Rm = U @ D @ Vt
    return Rm, mb - Rm @ ma


def ape(t, P, gt_t, gt_p, off):
    ok = (t + off > gt_t[0]) & (t + off < gt_t[-1])
    g = np.column_stack([np.interp(t[ok] + off, gt_t, gt_p[:, i]) for i in range(3)])
    e = P[ok, :3, 3]
    Rm, tt = umeyama(e, g)
    return float(np.sqrt(((e @ Rm.T + tt - g) ** 2).sum(1).mean())), ok.sum()


def main():
    bag, gtf, kind, ref, runs = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5:]
    g = np.loadtxt(gtf, delimiter=",", usecols=range(11))
    gt_t, gt_p = g[:, 0] * 1e-9, g[:, 1:4]
    topic = AVIA_TOPIC if kind == "avia" else HORIZON_TOPIC
    imu_topic = next(t for t, k in IMU_TOPICS.items() if k == kind)
    imu, clk = [], None
    with AnyReader([Path(bag)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in (topic, imu_topic)]):
            if c.topic == topic and clk is not None:
                continue
            m = r.deserialize(raw, c.msgtype); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if c.topic == topic:
                clk = t * 1e-9 - st
            else:
                imu.append((st, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    imu = np.array(imu); rest = imu[:, 0] < imu[0, 0] + 1.0; imu[:, 1:4] -= imu[rest, 1:4].mean(axis=0)
    t_ref, P_ref = load_run(ref)
    offs = np.arange(-0.3, 0.3001, 0.005)
    off = offs[int(np.argmin([ape(t_ref, P_ref, gt_t, gt_p, o)[0] for o in offs]))]
    path_gt = None
    print(f"{kind}: clock offset (from {Path(ref).name}) {off * 1000:+.0f} ms")
    print(f"{'run':34s} {'APE m':>7s} {'RPE1s rot p50':>14s} {'rmse':>6s} {'RPE1s dist p50 cm':>18s} {'rmse':>6s} {'path vs GT':>11s}")
    for run in [ref] + runs:
        t, P = load_run(run)
        a, n = ape(t, P, gt_t, gt_p, off)
        er, ed = [], []
        for j in range(len(t) - 10):
            k = j + 10
            Rg = gyro_rotation(imu, t[j] - clk, t[k] - clk)
            if Rg is None:
                continue
            Rr = R.from_matrix((np.linalg.inv(P[j]) @ P[k])[:3, :3])
            er.append(np.degrees(np.linalg.norm((Rg.inv() * Rr).as_rotvec())))
            if gt_t[0] < t[j] + off and t[k] + off < gt_t[-1]:
                pj = np.array([np.interp(t[j] + off, gt_t, gt_p[:, i]) for i in range(3)])
                pk = np.array([np.interp(t[k] + off, gt_t, gt_p[:, i]) for i in range(3)])
                ed.append(abs(np.linalg.norm(P[k, :3, 3] - P[j, :3, 3]) - np.linalg.norm(pk - pj)) * 100)
        ok = (t + off > gt_t[0]) & (t + off < gt_t[-1])
        L = np.linalg.norm(np.diff(P[ok, :3, 3], axis=0), axis=1).sum()
        gp = np.column_stack([np.interp(t[ok] + off, gt_t, gt_p[:, i]) for i in range(3)])
        Lg = np.linalg.norm(np.diff(gp, axis=0), axis=1).sum()
        er, ed = np.array(er), np.array(ed)
        print(f"{Path(run).name:34s} {a:7.3f} {np.median(er):14.3f} {np.sqrt((er ** 2).mean()):6.3f} {np.median(ed):18.2f} "
              f"{np.sqrt((ed ** 2).mean()):6.2f} {100 * (L - Lg) / Lg:+10.1f}%")


if __name__ == "__main__":
    main()
