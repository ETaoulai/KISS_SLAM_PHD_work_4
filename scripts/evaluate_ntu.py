#!/usr/bin/env python3
"""Runs on NTU VIRAL sequences, scored with the official protocol of the dataset (evaluation tutorial / notebook).

    python scripts/evaluate_ntu.py <sequence> <run dir> [<run dir> ...] [--root=<ntu_viral dir>] [--out=<dir>]

sequence: eee_01 ... tnp_03.  Looked up under --root (default /home/photogrammetry/kiss_data/ntu_viral, docs/datasets.md):
          ntuviral_gt/<seq>/ground_truth.csv (the official GT, github ntu-aris/ntuviral_gt) and <seq>/lidar_horz.yaml,
          <seq>/leica_prism.yaml (the calibration shipped in each sequence's zip).

The official protocol (ntu-aris.github.io/ntu_viral_dataset/evaluation_tutorial.html, ntuviral_evaluate.ipynb), step by step:
  1. the estimate is the pose of the BODY (= the IMU, T_Body_Imu = I), only the samples inside the time span of the GT;
  2. each estimated position gets the offset from the body to the prism, where the GT is measured:
     P + R · t_B_prism, t_B_prism = (-0.293656, -0.012288, -0.273095) m (0.40 m; "many users forget" it, per the dataset);
  3. estimate samples kept if a GT sample lies within 0.05 s; GT associated to them with evo, max_diff 0.05 s;
  4. SE(3) Umeyama alignment of the estimate to the GT (no scale);
  5. ATE = RMSE of the translation APE (evo);
  6. completeness = (last estimate stamp - GT start) / GT span, %; an ATE below 20 m with completeness below 90 % counts
     as inf (a run that stopped early).
Our poses are of the horizontal Ouster (/os1_cloud_node1/points, frame sensor1/os_sensor) at the instant each stands
for (evaluate_official.pose_times, #061): T_world_body = T_world_lidar · (T_Body_Lidar)^-1, T_Body_Lidar from
lidar_horz.yaml.  The body trajectory is written to <out>/<run>_body_tum.txt.
"""
import re
import sys
from pathlib import Path

import numpy as np
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from evaluate_official import pose_times, write_tum  # noqa: E402

ROOT = Path("/home/photogrammetry/kiss_data/ntu_viral")
MAX_DIFF = 0.05          # notebook: min |t_gt - t| < 0.05 and associate_trajectories(..., max_diff=0.05)
MIN_COMPLETENESS = 90.0  # notebook: min_completeness
T_B_PRISM = np.array([-0.293656, -0.012288, -0.273095])      # notebook (= leica_prism.yaml of every sequence)


def opencv_matrix(path, name):
    """A 4x4 !!opencv-matrix (T_Body_Lidar, T_Body_Prism) from the dataset's OpenCV YAML."""
    text = Path(path).read_text()
    # rtp / tnp / spms ship the older key names (T_Body2Lidar, the prism as T_Body2Imu), same values (checked 4/10)
    aliases = {"T_Body_Lidar": ["T_Body_Lidar", "T_Body2Lidar"], "T_Body_Prism": ["T_Body_Prism", "T_Body2Prism", "T_Body2Imu"]}
    name = next((n for n in aliases.get(name, [name]) if n in text), name)
    block = text[text.index(name):]
    data = re.search(r"data:\s*\[([^\]]*)\]", block)[1]
    return np.array([float(x) for x in data.replace("\n", " ").split(",")]).reshape(4, 4)


def load_gt(seq, root=ROOT):
    """(t, P) of ground_truth.csv as the notebook reads it: stamp = column 2 (ns), position = columns 3-5."""
    d = np.loadtxt(root / "ntuviral_gt" / seq / "ground_truth.csv", delimiter=",", skiprows=1)
    return d[:, 2] / 1e9, d[:, 3:6], d[:, 6:10]


def score(t_gt, P_gt, Q_gt, t, P_body, R_body):
    """Steps 1-6 on a body trajectory (times t, positions P_body, rotation matrices R_body)."""
    t_min, t_max = t_gt[0], t_gt[-1]
    inside = (t >= t_min) & (t <= t_max)
    t, P, R = t[inside], P_body[inside], R_body[inside]
    P = P + np.einsum("nij,j->ni", R, T_B_PRISM)                     # step 2: the prism
    near = np.array([np.min(np.abs(t_gt - s)) < MAX_DIFF for s in t])  # step 3
    q_wxyz = Rotation.from_matrix(R[near]).as_quat()[:, [3, 0, 1, 2]]
    est = PoseTrajectory3D(P[near], q_wxyz, t[near])
    # As the notebook: the GT quaternions are passed in file order (x, y, z, w); only positions are scored.
    gt = PoseTrajectory3D(P_gt, Q_gt, t_gt)
    est_a, gt_a = sync.associate_trajectories(est, gt, max_diff=MAX_DIFF)
    if est_a.num_poses != est.num_poses:
        # Only a denser estimate than one pose per scan (Traj-LO: every 40 ms, #083) gets here: evo's association drops
        # poses and the notebook would pair est and GT out of step.  Then, and only then, keep per GT sample the closest
        # pose and associate again.  Every one-pose-per-scan run passes the check above unchanged: the official numbers
        # (a first version applied this always and moved NTU APE by up to 0.4 %, corrected 1/10).
        nearest = np.array([np.argmin(np.abs(t_gt - s)) for s in t])
        for g in np.unique(nearest[near]):
            same = np.flatnonzero(near & (nearest == g))
            if len(same) > 1:
                near[same] = False
                near[same[np.argmin(np.abs(t[same] - t_gt[g]))]] = True
        q_wxyz = Rotation.from_matrix(R[near]).as_quat()[:, [3, 0, 1, 2]]
        est = PoseTrajectory3D(P[near], q_wxyz, t[near])
        est_a, gt_a = sync.associate_trajectories(est, gt, max_diff=MAX_DIFF)
    if est_a.num_poses != est.num_poses:
        raise RuntimeError("association dropped estimate samples: the notebook would pair est and GT out of step")
    try:
        est.align(gt_a)                                               # step 4 (evo: SE(3), no scale)
    except Exception as e:                                            # evo GeometryException: an estimate that never moved
        print(f"alignment impossible ({e}): scored as a failure")     # (CT-ICP driving profile on eee_01, #083)
        return dict(ate=float("inf"), ate_raw=float("nan"), completeness=0.0, n=est.num_poses)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((gt_a, est))
    ate = float(ape.get_result(ref_name="reference", est_name="estimate").stats["rmse"])
    completeness = round((t[-1] - t_min) / (t_max - t_min) * 100, 0)
    reported = float("inf") if (ate < 20 and completeness < MIN_COMPLETENESS) else ate
    return dict(ate=reported, ate_raw=ate, completeness=completeness, n=est.num_poses)


def evaluate_ntu(seq, run, root=ROOT, out_dir=None):
    run = Path(run)
    t, T_lidar, source = pose_times(run, "none")
    T_body_lidar = opencv_matrix(root / seq / "lidar_horz.yaml", "T_Body_Lidar")
    prism = opencv_matrix(root / seq / "leica_prism.yaml", "T_Body_Prism")[:3, 3]
    if not np.allclose(prism, T_B_PRISM, atol=1e-6):
        raise SystemExit(f"{seq}: leica_prism.yaml {prism} differs from the notebook's t_B_prism {T_B_PRISM}")
    T_body = T_lidar @ np.linalg.inv(T_body_lidar)
    if out_dir is not None:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        write_tum(Path(out_dir) / f"{run.name}_body_tum.txt", t, T_body)
    t_gt, P_gt, Q_gt = load_gt(seq, root)
    return dict(exact_times=source == "exact", **score(t_gt, P_gt, Q_gt, t, T_body[:, :3, 3], T_body[:, :3, :3]))


def main():
    root = Path(next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--root=")), ROOT))
    out = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--out=")), None)
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    print(f"NTU VIRAL {args[0]}  (official protocol: body + prism offset 0.40 m, 0.05 s, SE(3), ATE rmse, completeness)\n")
    print(f"{'run':<18}{'ATE m':>10}{'complete %':>12}{'poses':>8}")
    for r in args[1:]:
        v = evaluate_ntu(args[0], r, root, out)
        print(f"{Path(r).name:<18}{v['ate']:>10.3f}{v['completeness']:>12.0f}{v['n']:>8}")


if __name__ == "__main__":
    main()
