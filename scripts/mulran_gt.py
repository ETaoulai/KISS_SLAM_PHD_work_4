#!/usr/bin/env python3
"""MulRan ground truth in the LiDAR frame, TUM format (#208).

    python scripts/mulran_gt.py <seq dir> <out_tum.txt>

global_pose.csv gives the vehicle base in UTM at 100 Hz (stamp ns, 3 x 4 row-major).  The Ouster's pose = base pose x T_base_ouster, with
T_base_ouster the published calibration (calib_base2ouster: translation 1.7042, -0.021, 1.8047 m; rotation xyz 0.0001, 0.0003, 179.6654 deg,
the values KISS-ICP's MulRan loader uses).  Positions relative to the first pose (UTM numbers are too large for float precision in evo).
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def main():
    seq, out = sys.argv[1], sys.argv[2]
    g = np.loadtxt(f"{seq}/global_pose.csv", delimiter=",")
    t = g[:, 0] * 1e-9
    T = np.tile(np.eye(4), (len(g), 1, 1))
    T[:, :3, :4] = g[:, 1:].reshape(-1, 3, 4)
    X = np.eye(4)
    X[:3, :3] = Rotation.from_euler("xyz", [0.0001, 0.0003, 179.6654], degrees=True).as_matrix()
    X[:3, 3] = [1.7042, -0.021, 1.8047]
    L = T @ X
    L[:, :3, 3] -= L[0, :3, 3]
    q = Rotation.from_matrix(L[:, :3, :3]).as_quat()                  # x y z w
    np.savetxt(out, np.column_stack([t, L[:, :3, 3], q]), fmt="%.9f")
    print(f"{len(t)} poses, {t[-1] - t[0]:.1f} s, path {np.linalg.norm(np.diff(L[:, :3, 3], axis=0), axis=1).sum():.1f} m -> {out}")


if __name__ == "__main__":
    main()
