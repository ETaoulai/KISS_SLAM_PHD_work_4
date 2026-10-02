#!/usr/bin/env python3
"""Scan-to-MAP image motion (#112, open_tasks B.12a; LLOL / UPSLAM panorama idea): match the raw current scan against an intensity image of the
local map instead of against the previous scan.  Offline check, no SLAM.

    python scripts/map_pano_check.py <sequence key> <run dir with poses> <oracle_motion.npz> [--frames=600] [--win=5,10,20] [--prev=<run dir>] [--gt-poses] [--fix-start]

Sequence key: a row of kiss_runs/all_seqs.tsv (its bag / folder and options).  Map: the W scans before scan k, each deskewed with its own ICP
motion M_j = P_{j-1}^-1 P_j (KISS convention: pose j at the end of sweep j) and placed with P_j, then carried into the frame at the START of
scan k (= P_{k-1}).  Image grid: azimuth x elevation, columns = the sensor's own (native_width), rows = its median ring spacing, upscaled for the
detector as the panorama (8 rows per ring).  Per pixel the front surface: points within 5 % of the nearest range, intensity and 3D position
averaged over them (an offline form of the panorama fusion of LLOL / UPSLAM).  The current scan: raw points on the same grid, gaps filled as
in the panorama; keypoints lifted to its raw point and time.  Matching: SURF upright, guided (window 40 columns at 1024, +-4 rings) at zero
shift - the map image is already predicted at the start of scan k.  Fit: RANSAC, then p_map = T0 . curve(t_q) . q with the scan
start T0 estimated too (fixing it to the ICP pose puts the pose error into the motion - first version) and curve(t) the method's "car" motion
within the sweep; the motion over the sweep = curve(1).
Score: rotation error of the motion over scan k against the ground truth (oracle_motion.npz), per scan and composed over 10 scans, next to
--prev (a run's own image_motions.npz on the same scans: the method's scan-to-scan motion).  The ICP poses come from a finished run (after its
pose graph; causal in the pipeline, where the poses of scans < k exist when scan k arrives).
"""
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import kiss_slam.intensity_deskew as d  # noqa: E402

X = Path("/home/photogrammetry/kiss_runs")


def dataset_for(key):
    row = next(r for r in csv.reader(open(X / "all_seqs.tsv"), delimiter="\t") if r[0] == key)
    src, extra = Path(row[1]), row[3]
    opts = dict(a[2:].split("=", 1) for a in extra.split() if a.startswith("--") and "=" in a)
    from kiss_slam.tools.point_cloud2 import read_point_cloud_raw
    if src.suffix == ".bag" or (src.is_dir() and any(src.glob("*.bag"))):
        from kiss_icp.datasets.rosbag import RosbagDataset
        ds = RosbagDataset(src, opts.get("topic", "/os_cloud_node/points"))
    else:
        from kiss_slam.tools.ncd_pcd import NewerCollege2020Pcd
        ds = NewerCollege2020Pcd(src)
    ds.read_point_cloud = read_point_cloud_raw
    return ds, float(opts.get("intensity-scale", 255.0 / 1024.0))


def deskew_to_end(xyz, s, M):
    L = np.real(logm(M)); out = np.empty_like(xyz); bins = np.minimum((s * 50).astype(int), 49)
    for b in np.unique(bins):
        m = bins == b; T = expm(((b + 0.5) / 50 - 1.0) * L); out[m] = xyz[m] @ T[:3, :3].T + T[:3, 3]
    return out


class AngGrid:
    def __init__(self, W, el_step, el_max, H):
        self.W, self.el_step, self.el_max, self.H = W, el_step, el_max, H

    def pix(self, xyz):
        az = np.arctan2(xyz[:, 1], xyz[:, 0]); el = np.degrees(np.arctan2(xyz[:, 2], np.linalg.norm(xyz[:, :2], axis=1)))
        c = (((az + np.pi) / (2 * np.pi)) * self.W).astype(int) % self.W
        r = np.clip(((self.el_max - el) / self.el_step).astype(int), 0, self.H - 1)
        return r, c

    def fuse(self, pts, inten):
        """Front-surface fusion: per pixel the points within 5 % of the nearest range, mean intensity and position."""
        r, c = self.pix(pts); rng = np.linalg.norm(pts, axis=1); idx = r * self.W + c
        near = np.full(self.H * self.W, np.inf); np.minimum.at(near, idx, rng)
        keep = rng <= near[idx] * 1.05
        idx, pts, inten = idx[keep], pts[keep], inten[keep]
        cnt = np.bincount(idx, minlength=self.H * self.W).astype(float)
        I = np.bincount(idx, inten, self.H * self.W); P = np.stack([np.bincount(idx, pts[:, i], self.H * self.W) for i in range(3)], 1)
        valid = cnt > 0
        I[valid] /= cnt[valid]; P[valid] /= cnt[valid][:, None]
        return I.reshape(self.H, self.W), P.reshape(self.H, self.W, 3), valid.reshape(self.H, self.W)

    def image(self, I, valid):
        img = I.copy()
        for r in range(self.H):                      # fill gaps along each row, as the panorama does
            v = valid[r]
            if v.sum() > 1:
                img[r, ~v] = np.interp(np.where(~v)[0], np.where(v)[0], img[r, v])
        return cv2.resize(np.clip(img, 0, 255).astype(np.uint8), (self.W, self.H * d.UP), interpolation=cv2.INTER_LINEAR)


def match_map(map_feat, cur_feat, rng, period=0.1):
    """Guided matches map -> current, lift (map points static at t = 0; current raw point + time), RANSAC + time fit."""
    (Pm, Vm, kpm, dm), (Pc, Tc, Vc, kpc, dc, t0) = map_feat, cur_feat
    if dm is None or dc is None or len(kpm) < 10 or len(kpc) < 10:
        return None, 0
    good = d.guided_matches(kpm, dm, kpc, dc, 0.0, d._px(40.0))
    if len(good) < d.MIN_INL:
        return None, 0
    xym = np.array([kpm[m.queryIdx].pt for m in good]); xyc = np.array([kpc[m.trainIdx].pt for m in good])
    p, _, ok1 = d.lookup_batch(Pm, np.zeros(Pm.shape[:2]), Vm, xym, False)
    q, tq, ok2 = d.lookup_batch(Pc, Tc, Vc, xyc, True)
    both = ok1 & ok2
    if both.sum() < d.MIN_INL:
        return None, 0
    p, q = p[both], q[both]; tq = (tq[both] - t0) / period; tp = np.zeros(len(p))
    M0, inl = d.ransac(p, q, rng)
    if M0 is None:
        return None, 0
    if "--fix-start" in sys.argv:                                   # T0 = I (exact only with --gt-poses): the time fit of the method, tp = 0
        M1, keep = d.fit_time(p[inl], tp[inl], q[inl], tq[inl], M0, "car")
        return (None, 0) if M1 is None else (M1, int(keep.sum()))
    return fit_offset(p[inl], q[inl], tq[inl], M0)


def fit_offset(p, q, tq, M0):
    """p = T0 . curve(tq) . q with T0 the scan start in the map frame (6 parameters, NOT fixed to the ICP pose - its error would otherwise
    go into the motion) and curve(t) = [R(t w + t^2/2 alpha), t v] the motion within the sweep ("car").  Returns curve(1), the motion over
    the sweep.  soft_l1 as the time fit, 3 rounds with the same inlier threshold."""
    from scipy.optimize import least_squares
    x = np.concatenate([R.from_matrix(M0[:3, :3]).as_rotvec(), M0[:3, 3], np.zeros(9)])

    def res(x, p, q, t):
        rc, tr = d.pose_at(x[6:], t)
        y = d._rotate(rc, q) + tr
        return (p - (R.from_rotvec(x[:3]).apply(y) + x[3:6])).ravel()

    keep = np.ones(len(p), bool)
    for _ in range(3):
        x = least_squares(res, x, args=(p[keep], q[keep], tq[keep]), loss="soft_l1", f_scale=0.05).x
        r = np.linalg.norm(res(x, p, q, tq).reshape(-1, 3), axis=1)
        keep = r < d.FIT_THR
        if keep.sum() < d.MIN_INL:
            return None, 0
    rc, tr = d.pose_at(x[6:], np.array([1.0]))
    M = np.eye(4); M[:3, :3] = R.from_rotvec(rc[0]).as_matrix(); M[:3, 3] = tr[0]
    return M, int(keep.sum())


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    key, run, orc = args[0], Path(args[1]), args[2]
    n = int(opts.get("frames", 600))
    rd = sorted(run.glob("*/pose_times.csv"))[-1].parent
    P = np.load(next(rd.glob("*_poses.npy")))
    D = np.load(orc)["motion"]
    ds, iscale = dataset_for(key)
    scans = []
    for i in range(min(n, len(P), len(ds))):
        xyz, ts, inten, ring = ds[i][:4]
        ok = np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > d.MIN_RANGE)
        scans.append((xyz[ok], np.asarray(ts, float)[ok], np.asarray(inten, float)[ok] * iscale, ring[ok]))
    xyz0, ts0, _, ring0 = scans[0]
    d.W = d.native_width(xyz0, ts0, ring0)
    el = np.degrees(np.arctan2(xyz0[:, 2], np.linalg.norm(xyz0[:, :2], axis=1)))
    e = np.sort([np.median(el[ring0 == k]) for k in np.unique(ring0)])
    el_step = float(np.median(np.diff(e)))
    g = AngGrid(d.W, el_step, e[-1] + el_step / 2, int(np.ceil((e[-1] - e[0]) / el_step)) + 1)
    d.UP, d.GUIDED_WINDOW, d.GUIDED_ROWS = 8, 40.0, 4
    det = d.make_detector("surf", 100.0, True); rng = np.random.default_rng(0)
    if "--gt-poses" in sys.argv:                                     # diagnostic: the map from the ground-truth sweep motions, composed
        P = np.zeros((len(scans), 4, 4)); P[0] = np.eye(4)
        for j in range(1, len(scans)):
            P[j] = P[j - 1] @ (D[j] if np.isfinite(D[j]).all() else np.eye(4))
    M_icp = [np.eye(4)] + [np.linalg.inv(P[j - 1]) @ P[j] for j in range(1, len(P))]
    world = {}                                                       # scan j -> (points in world frame, intensity)
    rot = lambda A: np.degrees(np.linalg.norm(R.from_matrix(A[:3, :3]).as_rotvec()))
    wins = [int(w) for w in opts.get("win", "5,10,20").split(",")]
    res = {w: np.full((len(scans), 4, 4), np.nan) for w in wins}
    for k, (xyz, ts, inten, ring) in enumerate(scans):
        t0 = float(ts.min()); s = np.clip((ts - t0) / 0.1, 0, 1)
        if k >= 1:
            Pstart = P[k - 1]                                          # start of scan k = end of scan k-1
            I_c, P_c_unused, V_c = g.fuse(xyz, inten)                 # current scan: raw points, raw times
            r_c, c_c = g.pix(xyz)
            Pc = np.zeros((g.H, g.W, 3)); Tc = np.zeros((g.H, g.W)); Vc = np.zeros((g.H, g.W), bool)
            Pc[r_c, c_c] = xyz; Tc[r_c, c_c] = ts; Vc[r_c, c_c] = True
            kpc, dc = d._detect(det, g.image(I_c, V_c))
            cur = (Pc, Tc, Vc, kpc, dc, t0)
            for w in wins:
                js = [j for j in range(max(0, k - w), k) if j in world]
                if not js:
                    continue
                Tinv = np.linalg.inv(Pstart)
                pts = np.concatenate([world[j][0] for j in js]) @ Tinv[:3, :3].T + Tinv[:3, 3]
                I_m, P_m, V_m = g.fuse(pts, np.concatenate([world[j][1] for j in js]))
                kpm, dm = d._detect(det, g.image(I_m, V_m))
                M, _ = match_map((P_m, V_m, kpm, dm), cur, rng)
                if M is not None:
                    res[w][k] = M
        pe = deskew_to_end(xyz, s, M_icp[k]) if k >= 1 else xyz      # into the frame at the end of sweep k, then world
        world[k] = (pe @ P[k][:3, :3].T + P[k][:3, 3], inten)
        world.pop(k - max(wins) - 1, None)
    # score
    prev = None
    if "prev" in opts:
        f = sorted(Path(opts["prev"]).glob("*/image_motions.npz"))[-1]; z = np.load(f)
        prev = np.full((len(scans), 4, 4), np.nan); m = z["scan"] < len(scans); prev[z["scan"][m]] = z["motion"][m]
    print(f"{key}: {len(scans)} scans, grid {g.W} x {g.H} (x{d.UP}), poses from {run.name}")
    rows = ([("scan-to-scan (the method, --prev)", prev)] if prev is not None else []) + [(f"scan-to-map, last {w} scans", res[w]) for w in wins]
    for name, Mm in rows:
        er, ew = [], []
        for k in range(1, len(scans)):
            if np.isfinite(Mm[k]).all() and np.isfinite(D[k]).all():
                er.append(rot(np.linalg.inv(D[k]) @ Mm[k]))
        for k in range(1, len(scans) - 10):
            if np.isfinite(Mm[k:k + 10]).all() and np.isfinite(D[k:k + 10]).all():
                A, B = np.eye(4), np.eye(4)
                for j in range(k, k + 10):
                    A, B = A @ Mm[j], B @ D[j]
                ew.append(rot(np.linalg.inv(B) @ A))
        fails = int((~np.isfinite(Mm[1:]).all(axis=(1, 2))).sum())
        print(f"  {name:36s} failures {fails:4d}  rotation per scan p50 {np.median(er):.3f} p90 {np.percentile(er, 90):.3f}  over 10 scans p50 "
              f"{np.median(ew) if ew else float('nan'):.3f} deg")


if __name__ == "__main__":
    main()
