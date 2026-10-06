#!/usr/bin/env python3
"""Why does the panorama height matter? (#195, mechanism only - no verdict from scan metrics)

For one sequence and one vertical upscale, runs the image-motion estimator of the locked method (upright SURF, guided matching, car model; range
fallback OFF so the intensity image alone is measured) on N consecutive scans and records per scan pair: keypoints, ratio-test matches, RANSAC /
fit inliers, inlier ratio, azimuth sectors covered by the inliers (of 8), failure, and - where a ground truth in the LiDAR frame exists - the
error of the rotation magnitude against the ground truth over the same sweep (|angle(M) - angle(GT)|, frame independent).

    python scripts/analyse_panorama_rows.py <sequence name in all_seqs.tsv | Boreas> <upscale> <out.json> [--start=200] [--n=200]
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import kiss_slam.intensity_deskew as d

X = Path("/home/photogrammetry/kiss_runs")
GT = {"quad_hard": X / "official_eval/quad_hard/gt_lidar.txt", "church_03": X / "official_eval/church_03/gt_lidar.txt",
      "Boreas": Path("/media/photogrammetry/Extreme SSD/boreas/gt_boreas-2021-01-26-11-22_lidar_tum.txt")}


def scans(seq, start, n):
    if seq == "Boreas":
        fs = sorted(Path("/media/photogrammetry/Extreme SSD/boreas/boreas-2021-01-26-11-22/lidar").glob("*.bin"))[start:start + n]
        for f in fs:
            p = np.fromfile(f, np.float32).reshape(-1, 6).astype(np.float64)
            p = p[(p[:, :3] != 0).any(1)]
            t0 = float(f.stem) * 1e-6
            yield np.ascontiguousarray(p[:, :3]), t0 + p[:, 5], p[:, 3].copy(), p[:, 4].astype(np.int64), 1.0
        return
    from kiss_icp.datasets.rosbag import RosbagDataset
    from kiss_slam.tools.point_cloud2 import read_point_cloud_raw
    row = next(l.rstrip("\n").split("\t") for l in open(X / "all_seqs.tsv") if l.split("\t")[0] == seq)
    opts = dict(a[2:].split("=", 1) for a in row[3].split() if a.startswith("--"))
    src = Path(row[1]) if seq != "church_03" else Path("/home/photogrammetry/kiss_runs_ssd/_lio_bags/church_03.bag")
    ds = RosbagDataset(src, opts.get("topic", "/os_cloud_node/points"))
    ds.read_point_cloud = read_point_cloud_raw
    scale = float(opts.get("intensity-scale", 1.0))
    for k in range(start + n):                       # the reader is serial: read from 0
        x = ds[k]
        if k >= start:
            yield np.asarray(x[0], float), np.asarray(x[1], float).ravel(), np.asarray(x[2], float), np.asarray(x[3]).astype(np.int64), scale


def main():
    seq, up, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[4:] if a.startswith("--"))
    start, n = int(opts.get("start", 200)), int(opts.get("n", 200))
    gt = None
    if seq in GT:
        g = np.loadtxt(GT[seq])
        gt = Slerp(g[:, 0], Rotation.from_quat(g[:, 4:8]))
        gt_t = (g[0, 0], g[-1, 0])
    est = None
    rows = []
    for xyz, ts, it, ring, scale in scans(seq, start, n):
        if est is None:
            est = d.ScanMotionEstimator(model="car", subpixel=True, detector="surf", surf_upright=True, intensity_scale=scale,
                                        guided_window=40, guided_prediction="hybrid", panorama_up=up, seed=0)
        M, nin = est.motion(xyz, ts, it, ring)
        cur = est.prev
        good = getattr(d.match_motion, "last_good", None) or []
        r = dict(kp=len(cur[3]), matches=len(good), inliers=int(nin), failed=M is None)
        pairs = getattr(d.match_motion, "last_pairs", None)
        if pairs is not None and M is not None:
            q = pairs[2]
            az = np.arctan2(q[:, 1], q[:, 0])
            r["sectors"] = int(len(np.unique(np.floor((az + np.pi) / (2 * np.pi) * 8).astype(int) % 8)))
            rng = np.linalg.norm(q, axis=1)                                  # #196: where the inliers are
            el = np.degrees(np.arctan2(q[:, 2], np.linalg.norm(q[:, :2], axis=1)))
            r.update(inl_range_med=float(np.median(rng)), inl_near5=float(np.mean(rng < 5)), inl_far20=float(np.mean(rng > 20)),
                     inl_below10=float(np.mean(el < -10)), inl_el_iqr=float(np.subtract(*np.percentile(el, [75, 25]))))
            if gt is not None:                                               # each inlier pair's residual under the GT motion
                t0 = est.last_t_start
                if gt_t[0] <= t0 and t0 + est.period <= gt_t[1]:
                    p_, tp_, q_, tq_ = pairs
                    Rp = gt(t0 + tp_ * est.period); Rq = gt(t0 + tq_ * est.period); R0 = gt([t0])[0]
                    # rotation-only check (no GT translation in the lidar frame here): angle between the bearings after the GT rotation
                    bp = (R0.inv() * Rp).apply(p_); bq = (R0.inv() * Rq).apply(q_)
                    cosang = np.sum(bp * bq, 1) / (np.linalg.norm(bp, axis=1) * np.linalg.norm(bq, axis=1))
                    r["inl_bearing_err_deg"] = float(np.median(np.degrees(np.arccos(np.clip(cosang, -1, 1)))))
        if gt is not None and M is not None:
            t0 = est.last_t_start
            t1 = t0 + est.period
            if gt_t[0] <= t0 and t1 <= gt_t[1]:
                a_gt = (gt([t0])[0].inv() * gt([t1])[0]).magnitude()
                r["rot_err_deg"] = float(np.degrees(abs(Rotation.from_matrix(M[:3, :3]).magnitude() - a_gt)))
                r["rot_gt_deg"] = float(np.degrees(a_gt))
        rows.append(r)
    rows = rows[1:]                                   # the first scan has no previous one
    ok = [r for r in rows if not r["failed"]]
    summ = dict(seq=seq, up=up, rows=None, scans=len(rows), failures=sum(r["failed"] for r in rows),
                kp=float(np.median([r["kp"] for r in rows])), matches=float(np.median([r["matches"] for r in rows])),
                inliers=float(np.median([r["inliers"] for r in ok])) if ok else 0.0,
                inlier_ratio=float(np.median([r["inliers"] / max(r["matches"], 1) for r in ok])) if ok else 0.0,
                sectors=float(np.median([r.get("sectors", 0) for r in ok])) if ok else 0.0)
    for k in ("inl_range_med", "inl_near5", "inl_far20", "inl_below10", "inl_el_iqr", "inl_bearing_err_deg"):
        v = [r[k] for r in ok if k in r]
        if v:
            summ[k] = float(np.median(v))
    errs = [r["rot_err_deg"] for r in ok if "rot_err_deg" in r]
    if errs:
        summ.update(rot_err_median=float(np.median(errs)), rot_err_p90=float(np.percentile(errs, 90)))
    summ["rows"] = int(round(up * len(np.unique(ring))))
    json.dump(dict(summary=summ, per_scan=rows), open(out, "w"))
    print(json.dumps(summ))


if __name__ == "__main__":
    main()
