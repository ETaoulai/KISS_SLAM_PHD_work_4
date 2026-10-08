#!/usr/bin/env python3
"""Runs on Hilti SLAM Challenge 2022 sequences, scored with the official protocol of the challenge (#238).

    python scripts/evaluate_hilti2022.py <reference> <run dir> [<run dir> ...] [--calib=<lidar_calibration.yaml>] [--out=<dir>]

reference: a ground-truth file of the challenge, name unchanged (Hugging Face Hilti-Research/hilti-slam-challenge-2022, ground_truth/):
           <exp>.txt                sparse: positions of the measurement tip on the control points (total station), "3DoF"
           <exp>_imu.txt            dense IMU trajectory (exp14, exp16, exp18), "6DoF"
           <exp>_imu_3dof.txt       sparse, IMU frame (exp10, exp14, exp16, exp18, exp23)
--calib:   calibration/calibration_files/lidar_calibration.yaml of the dataset ("Calibration Phasma25 22/03/22"; default
           /media/photogrammetry/Extreme SSD/hilti_2022/calibration/lidar_calibration.yaml).

The official protocol (github Hilti-Research/hilti-slam-challenge-2022, evaluation-2022/evaluation.py + batch_evaluation.py, copies in
/home/photogrammetry/kiss_data/hilti_2022_eval/), step by step:
  1. the estimate is the trajectory of the IMU (frame "imu", the frame of the ground truth), TUM;
  2. each estimated pose x T_imu_ref: identity when the reference name ends in "_imu.txt", else the measurement tip
     t = (0.059, -0.00855, 0.1964) m (= measurement_tip of the calibration);
     the official test is ref_file.split('_')[-1] == 'imu.txt', so an "*_imu_3dof.txt" reference ALSO gets the tip offset - and that is
     right: checked on exp14 (#238), the dense *_imu.txt trajectory with the tip offset lands on the *_imu_3dof.txt points (2.6 cm, the noise
     added), without it 7 cm off.  So those points are tip positions despite the name.  Default "official"; --imu3dof=imu for the identity;
  3. association by stamp with max_diff = 2 s;
  4. SE(3) Umeyama alignment, no scale;
  5. APE of the translation (rmse, mean, median, std, min, max);
  6. for sparse references the challenge score: per control point 10 / 6 / 3 / 1 / 0 points for an error < 1 / 3 / 6 / 10 cm / more,
     normalised by (number of reference points / 10) -> 0..100 per sequence; exp04-06 are left out of the official total (published
     before the challenge).  Completeness = matched estimate points / reference points (dense: duration x 10).
Our poses are of the LiDAR (PandarXT-32, frame of the point cloud), at the instant each stands for (evaluate_official.pose_times, #061):
T_world_imu = T_world_lidar . (T_imu_lidar)^-1, T_imu_lidar from the calibration (quaternion x, y, z, w; the imu's own is [0, 0, 0, 1]).
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from evaluate_official import pose_times, write_tum  # noqa: E402

CALIB = Path("/media/photogrammetry/Extreme SSD/hilti_2022/calibration/lidar_calibration.yaml")
MAX_DIFF = 2.0
T_TIP = np.eye(4)
T_TIP[:3, 3] = [0.059, -0.00855, 0.1964]
EXCLUDED = {"exp04_construction_upper_level", "exp05_construction_upper_level_2", "exp06_construction_upper_level_3"}
BINS, POINTS = [0.01, 0.03, 0.06, 0.10], [10, 6, 3, 1, 0]


def T_imu_lidar(calib=CALIB):
    s = yaml.safe_load(open(calib))["sensors"]["PandarXT-32"]
    assert s["parent"] == "imu"
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(s["extrinsics"]["quaternion"]).as_matrix()     # x, y, z, w
    T[:3, 3] = s["extrinsics"]["translation"]
    return T


def ref_kind(name, imu3dof="official"):
    n = name.lower()
    if n.endswith("_imu.txt"):
        return "imu", np.eye(4)
    if n.endswith("_imu_3dof.txt"):
        return ("imu_3dof (official: tip)", T_TIP) if imu3dof == "official" else ("imu_3dof", np.eye(4))
    return "tip", T_TIP


def score(reference, t, T_imu, imu3dof="official"):
    reference = Path(reference)
    kind, T_ref = ref_kind(reference.name, imu3dof)
    est = PoseTrajectory3D(poses_se3=list(np.asarray(T_imu) @ T_ref), timestamps=np.asarray(t, dtype=np.float64))
    ref = file_interface.read_tum_trajectory_file(str(reference))
    dense = ref.num_poses > 100
    ref_s, est_s = sync.associate_trajectories(ref, est, MAX_DIFF)
    est_s.align(ref_s, correct_scale=False, correct_only_scale=False)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((ref_s, est_s))
    out = dict(kind=kind, dense=dense, n_ref=ref.num_poses, n_matched=ref_s.num_poses,
               max_dt=float(np.max(np.abs(ref_s.timestamps - est_s.timestamps))),
               **{k: float(v) for k, v in ape.get_all_statistics().items()})
    if dense:
        out["completeness"] = min(est_s.num_poses / int((ref.timestamps[-1] - ref.timestamps[0]) * 10), 1.0)
    else:
        err = np.linalg.norm(ref_s.positions_xyz - est_s.positions_xyz, axis=1)
        pts = np.array(POINTS)[np.searchsorted(BINS, err, side="right")]
        out.update(score=float(pts.sum() / (ref.num_poses / 10)), completeness=min(est_s.num_poses / ref.num_poses, 1.0),
                   bins=[int(((err >= lo) & (err < hi)).sum()) for lo, hi in zip([0] + BINS, BINS + [np.inf])],
                   official_total=reference.stem not in EXCLUDED)
    return out


def evaluate_hilti2022(reference, run, calib=CALIB, out_dir=None, imu3dof="official"):
    run = Path(run)
    t, T_lidar, source = pose_times(run, "none")
    T_imu = T_lidar @ np.linalg.inv(T_imu_lidar(calib))
    if out_dir is not None:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        write_tum(Path(out_dir) / f"{run.name}_imu_tum.txt", t, T_imu)     # for the official evaluation.py
    return dict(exact_times=source == "exact", **score(reference, t, T_imu, imu3dof))


def main():
    o = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    calib, out, imu3dof = Path(o.get("calib", CALIB)), o.get("out"), o.get("imu3dof", "official")
    print(f"reference {args[0]}  (Hilti 2022 protocol: IMU trajectory, T_imu_ref, max_diff {MAX_DIFF} s, SE(3), APE; score / 100)\n")
    print(f"{'run':<20}{'type':>12}{'matched':>10}{'rmse m':>10}{'max m':>10}{'score':>8}{'compl.':>8}  bins <1/3/6/10/>10 cm")
    for r in args[1:]:
        v = evaluate_hilti2022(args[0], r, calib, out, imu3dof)
        print(f"{Path(r).name:<20}{v['kind']:>12}{v['n_matched']:>5}/{v['n_ref']:<4}{v['rmse']:>10.4f}{v['max']:>10.4f}"
              f"{v.get('score', float('nan')):>8.1f}{v['completeness']:>8.2f}  {v.get('bins', '')}")


if __name__ == "__main__":
    main()
