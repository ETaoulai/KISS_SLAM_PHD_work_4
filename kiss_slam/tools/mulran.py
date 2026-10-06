"""MulRan (Ouster OS1-64 on a car, KAIST / DCC / Riverside / Sejong; Kim et al., ICRA 2020) scans with intensity, ring and per-point time (#208).

    <seq>/Ouster/<nanoseconds>.bin     float32 x, y, z, intensity; always 64 x 1024 = 65536 points, column by column (index i: ring i % 64,
                                       column i // 64 - checked: constant elevation per i % 64, azimuth monotonic in the column)
    <seq>/global_pose.csv              ground truth of the vehicle base at 100 Hz (stamp, 3 x 4 row-major in UTM) -> scripts/mulran_gt.py

The file name is taken as the sweep start; a point's time is that plus column / 1024 x 0.1 s (as KISS-ICP's MulRan loader, which uses
column / 1024 as the relative time).  Points without a return (at the origin) are dropped after ring and time are assigned.
Intensity on the sensor's own scale (median ~140), as the NCD 2020 Ouster.
"""
from pathlib import Path

import numpy as np

H, W, PERIOD = 64, 1024, 0.1


class MulRan:
    def __init__(self, data_dir, first=0, last=None):
        self.data_dir = Path(data_dir)
        self.sequence_id = self.data_dir.name
        files = sorted((self.data_dir / "Ouster").glob("*.bin"), key=lambda f: int(f.stem))[slice(first, last)]
        self.scan_files = files
        self.stamps = np.array([int(f.stem) * 1e-9 for f in files])
        self.use_global_visualizer = True
        idx = np.arange(H * W)
        self._ring, self._dt = (idx % H).astype(np.int64), (idx // H) / W * PERIOD

    def __len__(self):
        return len(self.scan_files)

    def get_frames_timestamps(self):
        return self.stamps

    def __getitem__(self, idx):
        p = np.fromfile(self.scan_files[idx], np.float32).reshape(-1, 4).astype(np.float64)
        if len(p) == H * W:
            ring, t = self._ring, self.stamps[idx] + self._dt
        else:                                              # incomplete scan (KISS-ICP: "broken point clouds"): no column layout
            ring = np.zeros(len(p), np.int64)
            t = np.full(len(p), self.stamps[idx] + PERIOD / 2)
        keep = (p[:, :3] != 0.0).any(axis=1)
        return np.ascontiguousarray(p[keep, :3]), t[keep], p[keep, 3], ring[keep]
