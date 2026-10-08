"""#230 offline check: image rotation magnitude vs GT for a detector / upscaling on 150 scans (200-349): klt1d_offline_check.py <seq> surf|klt1d <up>"""
import sys, time, numpy as np
sys.path.insert(0, "/home/photogrammetry/Kiss_SLAM-main/scripts"); sys.path.insert(0, "/home/photogrammetry/Kiss_SLAM-main")
import analyse_panorama_rows as a
import kiss_slam.intensity_deskew as d
from scipy.spatial.transform import Rotation, Slerp
seq, det, up = sys.argv[1], sys.argv[2], float(sys.argv[3])
g = np.loadtxt(a.GT[seq]); gt = Slerp(g[:, 0], Rotation.from_quat(g[:, 4:8]))
import os; d.ROBUST = os.environ.get("ROBUST", "ransac")
est = None; errs = []; fails = 0; ns = []; t0 = time.time(); n = 0
for xyz, ts, it, ring, scale in a.scans(seq, 200, 150):
    if est is None:
        est = d.ScanMotionEstimator(model=sys.argv[4] if len(sys.argv) > 4 else "car", subpixel=True, detector=det, surf_upright=True, intensity_scale=scale,
                                    guided_window=40, guided_prediction="hybrid", panorama_up=up, seed=0)
    M, k = est.motion(xyz, ts, it, ring); n += 1
    if n == 1: continue
    if M is None: fails += 1; continue
    ns.append(k); t = est.last_t_start
    e = abs(Rotation.from_matrix(M[:3, :3]).magnitude() - (gt([t])[0].inv() * gt([t + 0.1])[0]).magnitude())
    errs.append(np.degrees(e))
print(f"{seq} {det} up={up}: fails {fails}/{n-1}, inliers median {np.median(ns):.0f}, rot err median {np.median(errs):.3f} deg p90 {np.percentile(errs,90):.3f}, {1000*(time.time()-t0)/n:.0f} ms/scan",
      getattr(d.klt1d_matches, "last", ""))
