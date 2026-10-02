#!/usr/bin/env python3
"""Per-scan rotation of a run (P_{j-1}^-1 P_j, poses at their pose_times) against the sensor's own gyro - TIERS Livox (#100).

    python scripts/score_run_gyro.py <bag> <kind: avia|horizon> <run dir> [<run dir> ...]

The Livox IMU shares the LiDAR's device clock and axes (solid_state_check.py); gyro bias from the first 1 s at rest.  Pose j stands at
stamp_j + fraction_j x 0.1 s (pose_times.csv), so the motion between poses j-1 and j is compared with the gyro over that interval.
"""
import csv
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from solid_state_check import score_gyro, IMU_TOPICS, AVIA_TOPIC, HORIZON_TOPIC  # noqa: E402


def main():
    bag, kind, runs = sys.argv[1], sys.argv[2], sys.argv[3:]
    topic = AVIA_TOPIC if kind == "avia" else HORIZON_TOPIC
    imu_topic = next(t for t, k in IMU_TOPICS.items() if k == kind)
    imu, off = [], None
    with AnyReader([Path(bag)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in (topic, imu_topic)]):
            m = r.deserialize(raw, c.msgtype) if (c.topic == imu_topic or off is None) else None
            if m is None:
                continue
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if c.topic == topic:
                off = t * 1e-9 - st
            else:
                imu.append((st, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    imu = np.array(imu); rest = imu[:, 0] < imu[0, 0] + 1.0; imu[:, 1:4] -= imu[rest, 1:4].mean(axis=0)
    for run in runs:
        rd = sorted(Path(run).glob("*/pose_times.csv"))[-1].parent
        P = np.load(next(rd.glob("*_poses.npy")))
        rows = list(csv.DictReader(open(rd / "pose_times.csv")))
        tp = np.array([float(r["stamp"]) + float(r["fraction"]) * 0.1 for r in rows]) - off
        M = [np.linalg.inv(P[j - 1]) @ P[j] for j in range(1, len(P))]
        score_gyro(f"{Path(run).name}", [(tp[j - 1], tp[j]) for j in range(1, len(P))], M, [1] * len(M), imu)


if __name__ == "__main__":
    main()
