"""#232 offline: do 180-deg sub-panoramas (stride 90) measure intra-sweep motion changes?  analyse_subpanorama_motion.py <seq> <detector> <up>
at the window's mean time; compare with GT over the same interval and with the full-scan fit (car model) over that interval."""
import sys, numpy as np
sys.path.insert(0, "/home/photogrammetry/Kiss_SLAM-main/scripts"); sys.path.insert(0, "/home/photogrammetry/Kiss_SLAM-main")
import analyse_panorama_rows as a
import kiss_slam.intensity_deskew as d
from scipy.spatial.transform import Rotation, Slerp
seq, det, up = sys.argv[1], sys.argv[2], float(sys.argv[3])
g = np.loadtxt(a.GT[seq]); gt = Slerp(g[:, 0], Rotation.from_quat(g[:, 4:8]))
ang = lambda A, B: np.degrees((A.inv() * B).magnitude())
est = None; E = {"window": [], "full_car": [], "full_cv_total": []}; spread = []; nwin = []
for xyz, ts, it, ring, scale in a.scans(seq, 200, 200):
    if est is None:
        est = d.ScanMotionEstimator(model="car", subpixel=True, detector=det, surf_upright=True, intensity_scale=scale,
                                    guided_window=40, guided_prediction="hybrid", panorama_up=up, seed=0)
    M, k = est.motion(xyz, ts, it, ring)
    pr = getattr(d.match_motion, "last_pairs", None)
    if M is None or pr is None: continue
    x_full = d.fit_time.last_params if hasattr(d.fit_time, "last_params") else None
    x_full = est.last_params
    t0 = est.last_t_start; p, tp, q, tq = pr
    az = np.degrees(np.arctan2(q[:, 1], q[:, 0]))
    rots = []
    for c in (0, 90, 180, 270):
        m = np.abs((az - c + 180) % 360 - 180) <= 90
        if m.sum() < 20: continue
        Mk, keep = d.fit_time(p[m], tp[m], q[m], tq[m], M, "cv")
        if Mk is None: continue
        xk = d.fit_time.last_params; tau = float(np.mean(tq[m]))
        if not (g[0, 0] <= t0 + (tau - 1) * 0.1 and t0 + tau * 0.1 <= g[-1, 0]): continue
        R_gt = ang(gt([t0 + (tau - 1) * 0.1])[0], gt([t0 + tau * 0.1])[0])
        R_win = np.degrees(np.linalg.norm(xk[:3]))
        r1, _ = d.pose_at(x_full, np.array([tau - 1, tau]))
        R_full = ang(Rotation.from_rotvec(r1[0]), Rotation.from_rotvec(r1[1]))
        E["window"].append(abs(R_win - R_gt)); E["full_car"].append(abs(R_full - R_gt)); rots.append(R_gt); nwin.append(m.sum())
    if len(rots) > 1: spread.append(np.ptp(rots))
for k, v in E.items():
    if v: print(f"{seq} {det}: {k:14s} rotation-over-period error vs GT median {np.median(v):.3f} p90 {np.percentile(v, 90):.3f} deg (n {len(v)})")
print(f"GT: spread of the rotation over the period between the window times, per scan: median {np.median(spread):.3f} p90 {np.percentile(spread, 90):.3f} deg; pairs per window median {np.median(nwin):.0f}")
