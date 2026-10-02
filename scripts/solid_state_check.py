#!/usr/bin/env python3
"""Feasibility of the image motion on solid-state LiDARs (#095, open_tasks B.10): TIERS Indoor02, Livox Avia / Horizon vs Ouster OS0.

    python scripts/solid_state_check.py <bag> <optitrack.csv> [--frames=N] [--kinds=ouster,avia,horizon] [--windows=1,2,3] [--ref=mocap|gyro] [--out=<prefix>]

No SLAM.  For every frame of each LiDAR the image motion (ScanMotionEstimator of the method, model "car", SURF upright) between it and
the previous frame, against the motion capture over the same interval.

Livox (CustomMsg: x, y, z, reflectivity, line, offset_time; non-repetitive pattern, no rings): the image is an azimuth x elevation
projection inside the field of view with the pixel size from the frame's own point density, sqrt(FoV area / points) - about one point
per pixel, the #093 rule without rings.  Intensity splatted and gaps filled by normalised convolution (Gaussian); the 3D point and time
of a pixel from the nearest point within 1.5 px (else invalid).  Variants: pixel 1x / 2x that size, detector on the image x1 / x2.
Brute-force matching (the guided window assumes a 360 deg panorama).  --windows: K > 1 accumulates NON-overlapping windows of K frames into
one scan of K x 0.1 s (sliding windows would share frames and pull the motion toward zero); points keep their own times, period K x 0.1.  Ouster OS0: the method as it is (auto columns + square pixels).

--ref=gyro (Livox only): the image rotation of each scan against the sensor's own IMU integrated over the same interval (/avia/livox/imu,
/livox/imu: same device clock as the LiDAR, axes of the LiDAR) - no clock offset and no extrinsic to estimate (an axis fit between the two is ill-conditioned here:
the rotation is almost all about the vertical).  --out: the motions of every variant as <prefix>_<kind>_w<K>_px<p>_ds<s>.npz.
Ground truth (default --ref=mocap): optitrack body poses.  The LiDAR <-> body rotation and the clock offset are not given for this pair, so both are
estimated from the data: offset by the best correlation of the rotation rates (|rotation| per frame, +-0.3 s), rotation by Kabsch on
the rotation vectors of the frames with > 1 deg.  This calibrates on the same motions it scores (3 rotation + 1 time parameters over
hundreds of frames - mildly optimistic, the same for every variant).
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation as R, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import kiss_slam.intensity_deskew as d  # noqa: E402

OUSTER_TOPIC, AVIA_TOPIC, HORIZON_TOPIC = "/os_cloud_node/points", "/avia/livox/lidar", "/livox/lidar"
IMU_TOPICS = {"/avia/livox/imu": "avia", "/livox/imu": "horizon"}      # each Livox's own IMU: same device clock, axes of the LiDAR


def read_bag(bag, n_frames):
    from rosbags.highlevel import AnyReader
    from kiss_slam.tools.point_cloud2 import read_point_cloud_raw
    frames = {"avia": [], "horizon": [], "ouster": []}
    imu = {"avia": [], "horizon": []}
    with AnyReader([Path(bag)]) as r:
        conns = [c for c in r.connections if c.topic in (OUSTER_TOPIC, AVIA_TOPIC, HORIZON_TOPIC, *IMU_TOPICS)]
        clock = {}                       # TIERS: LiDAR headers carry device time (s since power-on); ROS time = header + (bag time - header)
        for conn, t, raw in r.messages(connections=conns):
            m = r.deserialize(raw, conn.msgtype)
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if conn.topic in IMU_TOPICS:                          # device time, rad/s
                w = m.angular_velocity
                imu[IMU_TOPICS[conn.topic]].append((stamp, w.x, w.y, w.z))
                continue
            off = clock.setdefault(conn.topic, t * 1e-9 - stamp)   # constant, from the first message (the residual delay: score's search)
            stamp += off
            if conn.topic == OUSTER_TOPIC:
                if len(frames["ouster"]) < n_frames:
                    xyz, ts, inten, ring = read_point_cloud_raw(m)[:4]
                    frames["ouster"].append((stamp, xyz, np.asarray(ts, float) + off, inten, ring))
            else:
                key = "avia" if conn.topic == AVIA_TOPIC else "horizon"
                if len(frames[key]) < n_frames:
                    p = m.points
                    xyz = np.array([(q.x, q.y, q.z) for q in p], float)
                    ts = stamp + np.array([q.offset_time for q in p], float) * 1e-9
                    inten = np.array([q.reflectivity for q in p], float)
                    line = np.array([q.line for q in p], int)
                    frames[key].append((stamp, xyz, ts, inten, line))
            if all(len(v) >= n_frames for v in frames.values()):
                break
    frames["clock"] = clock
    frames["imu"] = {}
    for k, v in imu.items():
        a = np.array(v)
        if len(a):
            rest = a[:, 0] < a[0, 0] + 1.0                      # gyro bias from the first 1 s when the rig stands still (TIERS: ~1.5 s)
            if np.median(np.linalg.norm(a[rest, 1:4], axis=1)) < 0.02:
                a[:, 1:4] -= a[rest, 1:4].mean(axis=0)
        frames["imu"][k] = a
    return frames


def gyro_rotation(imu, t0, t1):
    """Rotation over [t0, t1] (device time) from the gyroscope: product of Exp(w dt), w interpolated at the interval ends."""
    t, w = imu[:, 0], imu[:, 1:4]
    if t0 < t[0] or t1 > t[-1]:
        return None
    k = np.flatnonzero((t > t0) & (t < t1))
    tt = np.r_[t0, t[k], t1]
    ww = np.vstack([[np.interp(t0, t, w[:, i]) for i in range(3)], w[k], [np.interp(t1, t, w[:, i]) for i in range(3)]])
    Rm = R.identity()
    for a in range(len(tt) - 1):
        Rm = Rm * R.from_rotvec(0.5 * (ww[a] + ww[a + 1]) * (tt[a + 1] - tt[a]))
    return Rm


def score_gyro(name, times_dev, motions, inliers, imu):
    """Image rotation per scan against the sensor's own gyroscope - no clock offset, no extrinsic (the IMU axes are the LiDAR's)."""
    ok = np.array([m is not None for m in motions])
    G = [gyro_rotation(imu, a, b) for a, b in times_dev]
    k = ok & np.array([g is not None for g in G])
    Rimg = [R.from_matrix(m[:3, :3]) for m, kk in zip(motions, k) if kk]
    Rg = [g for g, kk in zip(G, k) if kk]
    e = np.degrees([np.linalg.norm((g.inv() * ri).as_rotvec()) for g, ri in zip(Rg, Rimg)])
    gm = np.degrees([np.linalg.norm(g.as_rotvec()) for g in Rg]); im = np.degrees([np.linalg.norm(ri.as_rotvec()) for ri in Rimg])
    print(f"{name:28s} scans {len(motions):4d}  failures {int((~ok).sum()):4d}  inliers p50 {np.median([i for i in inliers if i > 0] or [0]):5.0f}  "
          f"vs gyro: rotation error p50 {np.median(e):.3f} p90 {np.percentile(e, 90):.3f} deg, |rot| corr {np.corrcoef(gm, im)[0, 1]:.2f}  "
          f"(gyro rot/scan p50 {np.median(gm):.2f})")


def livox_grid(step_deg):
    """A d.panorama replacement for one Livox: azimuth x elevation inside the FoV at step_deg per pixel (bounds set on first call)."""
    state = {}

    def panorama(xyz, ts, inten, ring):
        az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])); el = np.degrees(np.arctan2(xyz[:, 2], np.linalg.norm(xyz[:, :2], axis=1)))
        if not state:
            state.update(az0=np.percentile(az, 0.5) - 1, az1=np.percentile(az, 99.5) + 1,
                         el0=np.percentile(el, 0.5) - 1, el1=np.percentile(el, 99.5) + 1)
            state["W"] = int(np.ceil((state["az1"] - state["az0"]) / step_deg)); state["H"] = int(np.ceil((state["el1"] - state["el0"]) / step_deg))
            d.W, d.UP = state["W"], 1
        H, W = state["H"], state["W"]
        col = np.clip(((state["az1"] - az) / step_deg).astype(int), 0, W - 1)        # left = +azimuth, as the panoramas
        row = np.clip(((state["el1"] - el) / step_deg).astype(int), 0, H - 1)
        acc = np.zeros((H, W)); cnt = np.zeros((H, W))
        np.add.at(acc, (row, col), inten); np.add.at(cnt, (row, col), 1)
        P = np.zeros((H, W, 3)); T = np.zeros((H, W)); have = cnt > 0
        P[row, col] = xyz; T[row, col] = ts                                          # last point per pixel
        num = cv2.GaussianBlur(acc, (0, 0), 0.8); den = cv2.GaussianBlur(cnt, (0, 0), 0.8)
        img = np.where(den > 1e-3, num / np.maximum(den, 1e-9), 0.0)
        dist, (ri, ci) = ndimage.distance_transform_edt(~have, return_indices=True)
        valid = dist <= 1.5
        P = P[ri, ci]; T = T[ri, ci]
        return np.clip(img, 0, 255).astype(np.uint8), P, T, valid
    return panorama


def estimator(kind):
    if kind == "ouster":
        return d.ScanMotionEstimator(seed=0, model="car", subpixel=True, stuck_min=0.05, floor_only=True, detector="surf",
                                     surf_upright=True, intensity_scale=255.0 / 1024.0, guided_window=40.0,
                                     panorama_width="auto", panorama_up="auto")
    return d.ScanMotionEstimator(seed=0, model="car", subpixel=True, stuck_min=0.05, floor_only=True, detector="surf",
                                 surf_upright=True, intensity_scale=1.0, guided_window=None)


def gt_motion(gt_t, gt_q, gt_p, t0, t1):
    """Body motion G(t0)^-1 G(t1) by interpolation; None outside the ground truth."""
    if t0 < gt_t[0] or t1 > gt_t[-1]:
        return None
    sl = Slerp(gt_t, R.from_quat(gt_q))
    R0, R1 = sl([t0, t1])
    p0 = np.array([np.interp(t0, gt_t, gt_p[:, i]) for i in range(3)]); p1 = np.array([np.interp(t1, gt_t, gt_p[:, i]) for i in range(3)])
    M = np.eye(4); M[:3, :3] = (R0.inv() * R1).as_matrix(); M[:3, 3] = R0.inv().apply(p1 - p0)
    return M


def score(name, times, motions, inliers, gt):
    gt_t, gt_q, gt_p = gt
    ok = np.array([m is not None for m in motions])
    rv = np.array([np.degrees(R.from_matrix(m[:3, :3]).as_rotvec()) if m is not None else [np.nan] * 3 for m in motions])
    # clock offset: rotation rate of the image vs of the ground truth
    best = None
    for off in np.arange(-0.6, 0.6001, 0.005):
        g = [gt_motion(gt_t, gt_q, gt_p, a + off, b + off) for a, b in times]
        gm = np.array([np.degrees(np.linalg.norm(R.from_matrix(x[:3, :3]).as_rotvec())) if x is not None else np.nan for x in g])
        k = ok & np.isfinite(gm)
        if k.sum() > 20:
            c = np.corrcoef(np.linalg.norm(rv[k], axis=1), gm[k])[0, 1]
            if best is None or c > best[0]:
                best = (c, off, g)
    if best is None:
        print(f"{name:28s} frames {len(motions):4d}  failures {int((~ok).sum()):4d}  - too few motions to score")
        return
    c, off, g = best
    gv = np.array([np.degrees(R.from_matrix(x[:3, :3]).as_rotvec()) if x is not None else [np.nan] * 3 for x in g])
    k = ok & np.isfinite(gv).all(1)
    big = k & (np.linalg.norm(gv, axis=1) > 1.0)
    # rotation body -> lidar: a_L = Rx a_B (Kabsch on rotation vectors)
    A, B = rv[big], gv[big]
    U, S, Vt = np.linalg.svd(B.T @ A)
    Rx = (U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt).T
    e = np.array([np.degrees(np.linalg.norm((R.from_rotvec(np.radians(Rx @ b)).inv() * R.from_rotvec(np.radians(a))).as_rotvec()))
                  for a, b in zip(rv[k], gv[k])])
    mag = np.abs(np.linalg.norm(rv[k], axis=1) - np.linalg.norm(gv[k], axis=1))
    print(f"{name:28s} frames {len(motions):4d}  failures {int((~ok).sum()):4d}  inliers p50 {np.median([i for i in inliers if i > 0] or [0]):5.0f}  "
          f"rotation error p50 {np.median(e):.3f} p90 {np.percentile(e, 90):.3f} deg  | |rot| error p50 {np.median(mag):.3f}  "
          f"(GT rot/frame p50 {np.median(np.linalg.norm(gv[k], axis=1)):.2f}; offset {off * 1000:+.0f} ms, corr {c:.2f})")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    n = int(opts.get("frames", 423))
    g = np.loadtxt(args[1], delimiter=",", usecols=range(11))      # t ns, x y z, 3 angles, qx qy qz qw (|q| = 1 checked)
    gt = (g[:, 0] * 1e-9, g[:, 7:11], g[:, 1:4])
    assert np.allclose(np.linalg.norm(g[:, 7:11], axis=1), 1, atol=1e-3)
    print(f"ground truth {len(g)} poses, {gt[0][0]:.3f} .. {gt[0][-1]:.3f}")
    frames = read_bag(args[0], n)
    print({k: len(v) for k, v in frames.items()})
    kinds = opts.get("kinds", "ouster,avia,horizon").split(",")
    ref = opts.get("ref", "mocap")                    # "gyro": the Livox scored against their own IMU
    out = opts.get("out")
    windows = [int(w) for w in opts.get("windows", "1").split(",")]
    for kind in kinds:
        fr0 = frames[kind]
        if not fr0:
            continue
        for K in (windows if kind != "ouster" else [1]):
            # K > 1: non-overlapping windows of K frames as one scan of K x 0.1 s (shared frames would pull the motion toward zero)
            fr = [(fr0[i][0], *(np.concatenate([f[j] for f in fr0[i:i + K]]) for j in (1, 2, 3, 4)))
                  for i in range(0, len(fr0) - K + 1, K)]
            period = 0.1 * K
            variants = [("method (auto panorama)", None, 1.0)] if kind == "ouster" else \
                [(f"window {K} pixel x{px} detect x{ds}", px, ds) for px in (1, 2) for ds in (1, 2)]
            for label, px, ds in variants:
                saved = (d.panorama, d.W, d.UP, d.DETECT_SCALE)
                if kind != "ouster":
                    n_pts = np.median([len(f[1]) for f in fr])
                    az = np.degrees(np.arctan2(fr[0][1][:, 1], fr[0][1][:, 0])); el = np.degrees(np.arctan2(fr[0][1][:, 2], np.linalg.norm(fr[0][1][:, :2], axis=1)))
                    area = (np.percentile(az, 99.5) - np.percentile(az, 0.5)) * (np.percentile(el, 99.5) - np.percentile(el, 0.5))
                    step = np.sqrt(area / n_pts) * px
                    d.panorama = livox_grid(step)
                est = estimator(kind)
                est.period = period
                d.DETECT_SCALE = ds
                times, motions, inl = [], [], []
                for i, (stamp, xyz, ts, inten, ring) in enumerate(fr):
                    M, ni = est.motion(xyz, ts, inten, ring)
                    if i == 0:
                        continue
                    times.append((float(ts.min()), float(ts.min()) + period)); motions.append(None if M is None else np.asarray(M)); inl.append(ni)
                if ref == "gyro" and kind != "ouster":
                    off = frames["clock"][AVIA_TOPIC if kind == "avia" else HORIZON_TOPIC]
                    score_gyro(f"{kind} {label}", [(a - off, b - off) for a, b in times], motions, inl, frames["imu"][kind])
                else:
                    score(f"{kind} {label}", times, motions, inl, gt)
                if out is not None:
                    np.savez(f"{out}_{kind}_w{K}_px{px}_ds{ds}.npz", times=np.array(times),
                             motion=np.array([m if m is not None else np.full((4, 4), np.nan) for m in motions]), inliers=np.array(inl))
                if kind != "ouster":
                    print(f"{'':28s} pixel {step:.3f} deg, image {d.W} columns, {n_pts:.0f} points per scan, rotation per scan of {period:.1f} s")
                d.panorama, d.W, d.UP, d.DETECT_SCALE = saved

if __name__ == "__main__":
    main()
