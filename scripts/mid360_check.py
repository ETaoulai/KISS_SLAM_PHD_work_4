#!/usr/bin/env python3
"""Image motion on a Livox Mid-360 (360 deg, non-repetitive) under fast hand-held motion (#105): Hard Point Cloud Localization outdoor_hard_01.

    python scripts/mid360_check.py <bag dir> <traj_lidar_*.txt> [--frames=1500] [--K=1,2,3] [--comp=estimate,gt] [--ds=1,2] [--icp=<KISS run>] [--out=<prefix>]

No SLAM.  The image of scan k as in solid_state_accum.py (#097): azimuth x elevation inside the field of view (here 360 x ~59 deg), pixel
= sqrt(FoV area / accumulated points), the K-1 previous scans carried into the frame at the start of scan k by a compensation, keypoints
lifted to the newest scan's raw points (so the time-aware fit is as today).  Compensation: "estimate" (the image's own motions, as the
method would run), "gt" (oracle: the ground-truth motion), "icp" (a KISS run's poses).  K = 1: the single scan, no compensation.
Score per scan against the ground truth (LiDAR frame, one pose per scan; motion over [t0, t0 + 0.1 s] by SLERP / linear interpolation):
rotation error p50 / p90, failures, inliers.  Reference rows: constant velocity from the ground truth itself (the previous sweep's TRUE
motion as the prediction) - the error of a perfect constant-velocity deskew under this motion.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import kiss_slam.intensity_deskew as d  # noqa: E402
from solid_state_accum import Grid, deskew  # noqa: E402


def read_frames(bag_dir, n):
    from kiss_slam.tools.ros2bags import Ros2Bags
    from kiss_slam.tools.point_cloud2 import read_point_cloud_raw
    ds = Ros2Bags(Path(bag_dir), "/livox/points")
    ds.read_point_cloud = read_point_cloud_raw
    out = []
    for i in range(min(n, len(ds))):
        xyz, ts, inten, ring = ds[i][:4]
        out.append((xyz, np.asarray(ts, float), np.asarray(inten, float)))
    return out


class GT:
    def __init__(self, path):
        g = np.loadtxt(path)
        self.t, self.p = g[:, 0], g[:, 1:4]
        self.sl = Slerp(self.t, R.from_quat(g[:, 4:8]))

    def motion(self, t0, t1):
        if t0 < self.t[0] or t1 > self.t[-1]:
            return None
        R0, R1 = self.sl([t0, t1])
        p0 = np.array([np.interp(t0, self.t, self.p[:, i]) for i in range(3)]); p1 = np.array([np.interp(t1, self.t, self.p[:, i]) for i in range(3)])
        M = np.eye(4); M[:3, :3] = (R0.inv() * R1).as_matrix(); M[:3, 3] = R0.inv().apply(p1 - p0)
        return M


def rot_err(A, B):
    return np.degrees(np.linalg.norm(R.from_matrix(A[:3, :3].T @ B[:3, :3]).as_rotvec()))


def run(fr, gt, K, ds, comp, icp=None):
    n_pts = np.median([len(f[0]) for f in fr])
    az, el = Grid.angles(fr[0][0])
    area = (np.percentile(az, 99.5) - np.percentile(az, 0.5)) * (np.percentile(el, 99.5) - np.percentile(el, 0.5))
    step = np.sqrt(area / (K * n_pts))
    g = Grid(fr[0][0], step)
    d.W, d.UP, d.DETECT_SCALE, d.GUIDED_WINDOW = g.W, 1, ds, None
    det = d.make_detector("surf", 100.0, True); bf = cv2.BFMatcher(cv2.NORM_L2); rng = np.random.default_rng(0)
    hist, prev_feat, last_M = [], None, np.eye(4)
    errs, fails, inl = [], 0, []
    import time
    t_img = 0.0
    for i, (xyz, ts, inten) in enumerate(fr):
        t0 = float(ts.min()); s = np.clip((ts - t0) / 0.1, 0, 1)
        Mgt = gt.motion(t0, t0 + 0.1)
        if comp == "gt":
            pred = gt.motion(t0 - 0.1, t0) if gt.motion(t0 - 0.1, t0) is not None else np.eye(4)
        elif comp == "icp":
            pred = icp[i - 1] if i >= 1 else np.eye(4)
        else:
            pred = last_M
        cur = deskew(xyz, s, pred, to_end=False) if K > 1 else xyz
        pos, val = [cur], [inten]
        carry = np.eye(4)
        for hx, hs, hin, hM in reversed(hist):
            pe = deskew(hx, hs, hM, to_end=True); Ci = np.linalg.inv(carry)
            pos.append(pe @ Ci[:3, :3].T + Ci[:3, 3]); val.append(hin); carry = hM @ carry
        tic = time.perf_counter()
        img = g.image(np.concatenate(pos), np.concatenate(val))
        P, T, valid = g.lift(cur, xyz, ts)
        kps, desc = d._detect(det, img)
        feat = (P, T, valid, kps, desc, t0, img)
        t_img += time.perf_counter() - tic
        M = None
        if prev_feat is not None:
            _, M1, n = d.match_motion(prev_feat, feat, 0.1, rng, bf, "car", True, 0.05, True, -10.0, 5.0)
            M = None if M1 is None else np.asarray(M1)
            if M is None:
                fails += 1
            elif Mgt is not None:
                errs.append(rot_err(Mgt, M)); inl.append(n)
        prev_feat = feat
        if M is not None:
            last_M = M
        used = (Mgt if Mgt is not None else np.eye(4)) if comp == "gt" else icp[i] if comp == "icp" else (M if M is not None else last_M)
        if K > 1:
            hist = (hist + [(xyz, s, inten, used)])[-(K - 1):]
    e = np.array(errs)
    if not len(e):
        return f"failures {fails:4d}/{len(fr) - 1}  - no motion to score   (pixel {step:.2f} deg, {g.W} x {g.H})"
    tm = f"  image + detect {1000 * t_img / len(fr):.0f} ms / scan"
    return f"failures {fails:4d}/{len(fr) - 1}  inliers p50 {np.median(inl) if inl else 0:5.0f}  rotation error p50 {np.median(e):.3f} p90 {np.percentile(e, 90):.3f} deg   (pixel {step:.2f} deg, {g.W} x {g.H}){tm}"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    fr = read_frames(args[0], int(opts.get("frames", 1500)))
    gt = GT(args[1])
    icp = None
    if "icp" in opts:
        P = np.load(sorted(Path(opts["icp"]).glob("*/*_poses.npy"))[-1])
        icp = [np.eye(4)] + [np.linalg.inv(P[j - 1]) @ P[j] for j in range(1, len(P))]
    # references: ground-truth rotation per scan, and a perfect constant-velocity prediction (previous sweep's true motion)
    gr, cv = [], []
    for xyz, ts, _ in fr[1:]:
        t0 = float(ts.min()); A = gt.motion(t0, t0 + 0.1); B = gt.motion(t0 - 0.1, t0)
        if A is not None and B is not None:
            gr.append(np.degrees(np.linalg.norm(R.from_matrix(A[:3, :3]).as_rotvec()))); cv.append(rot_err(A, B))
    print(f"{len(fr)} scans, {np.median([len(f[0]) for f in fr]):.0f} points per scan; GT rotation per scan p50 {np.median(gr):.2f} p90 {np.percentile(gr, 90):.2f} p99 "
          f"{np.percentile(gr, 99):.2f} deg")
    print(f"{'constant velocity from the GT (perfect CV)':44s} rotation error p50 {np.median(cv):.3f} p90 {np.percentile(cv, 90):.3f} deg")
    for K in [int(k) for k in opts.get("K", "1,2,3").split(",")]:
        for comp in (["none"] if K == 1 else opts.get("comp", "estimate,gt").split(",")):
            for ds in [float(v) for v in opts.get("ds", "1,2").split(",")]:
                print(f"{f'K{K} {comp} detect x{ds:g}':44s} {run(fr, gt, K, ds, comp, icp)}", flush=True)


if __name__ == "__main__":
    main()
