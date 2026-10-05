"""Boreas (Velodyne Alpha Prime, 128 beams, on a car) scans with intensity, ring and per-point time (#085).

    <seq>/lidar/<microseconds>.bin          float32 x, y, z, intensity (0-~210), ring (0-127), t (s, about -0.052..+0.052)
    <seq>/applanix/lidar_poses.csv          ground truth at every scan time (scripts/boreas_gt.py)

The file name is the scan time in microseconds, the middle of the sweep (t runs from about -0.05 to +0.05 s around it).
As the other readers: absolute point times in s (file time + t), and the scan stamp = the sweep's first point, so the
pose-time conventions of evaluate_official hold.  Intensity on the sensor's own 0-~210 scale: the panorama clips at 255,
run_ncd.py --intensity-scale=1.0.  Points at the origin (no return) are dropped.
"""
from pathlib import Path

import numpy as np


class Boreas:
    def __init__(self, data_dir, first=0, last=None):
        self.data_dir = Path(data_dir)
        self.sequence_id = self.data_dir.name
        files = sorted((self.data_dir / "lidar").glob("*.bin"), key=lambda f: int(f.stem))[slice(first, last)]
        self.scan_files = files
        self.file_times = np.array([int(f.stem) * 1e-6 for f in files])
        self.stamps = self.file_times - 0.0518            # nominal sweep start, replaced by the first point when read
        self.use_global_visualizer = True

    def __len__(self):
        return len(self.scan_files)

    def get_frames_timestamps(self):
        return self.stamps

    def __getitem__(self, idx):
        p = np.fromfile(self.scan_files[idx], np.float32).reshape(-1, 6).astype(np.float64)
        p = p[np.linalg.norm(p[:, :3], axis=1) > 0.0]
        t = self.file_times[idx] + p[:, 5]
        self.stamps[idx] = t.min()                         # the scan stamp = its first point (as the Ouster readers)
        return np.ascontiguousarray(p[:, :3]), t, p[:, 3], p[:, 4].astype(np.int64)   # contiguous: pybind copies a strided slice point by point (#170: 165 s of 600)
