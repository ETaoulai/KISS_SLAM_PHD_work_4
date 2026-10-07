#!/usr/bin/env python3
"""Another LiDAR-only odometry on one of our sequences, for the comparison of the paper (#083).

    <baselines venv>/bin/python scripts/run_baseline.py <method> <sequence> <out dir> [n_scans] [--topic=...] [--config=<yaml|cfg>]
                                                        [--deskew=true]

method:  genz  GenZ-ICP (Lee et al., RA-L 2025; pip genz-icp 0.3.2)
         mad   MAD-ICP (Ferrari et al., RA-L 2024; pip mad-icp 0.0.10, built with scikit-build-core < 0.10)
         trajlo  Traj-LO (Zheng & Zhu, RA-L 2024; continuous-time), /home/photogrammetry/baselines/Traj-LO, branch kiss_compare
                 (ba273d3 + a headless runner, a folder of bags read as one sequence, Ouster point time = header + t like our reader
                 instead of header - 0.1 s + t).  Reads the bags itself (not our readers); config = its own data/config_{ouster,ntu,hesai}.yaml
                 (--config=ouster|ntu|hesai) with our path / topic and identity T_body_lidar / T_body_gt (LiDAR frame).  A pose every
                 40 ms at the time it stands for.  No .pcd input (NCD 2020 01_short).
         cticp  CT-ICP (Dellenbach et al., ICRA 2022; continuous-time), /home/photogrammetry/baselines/ct_icp, branch kiss_compare
                 (d467813 + SIGSTKSZ fix for glibc >= 2.34 + command/cmd_stream_odometry.cpp; built with CXXFLAGS=-include cstdint).
                 Fed scan by scan from OUR readers (absolute per-point times, points closer than 0.5 m dropped) through a pipe;
                 profile DefaultRobustOutdoorLowInertia (the authors' NCLT profile).  One pose per scan: the END pose at the last
                 point's time.
All four are odometry only (no loop closures).  They run in their own virtual environment,
/home/photogrammetry/baselines/venv (--system-site-packages on the kiss-slam-main python, so kiss_icp's readers are shared).

Same input and output as scripts/run_ncd.py, so results_table.py reads them like our arms (folder <arm>_s0):
- the scans come from OUR readers (kiss_icp RosbagDataset / kiss_slam NewerCollege2020Pcd), not the methods' own;
- the written stamps are each scan's header stamp (the rosbag reader's own are the bag record times);
- the poses are those of the LiDAR frame (MAD-ICP's app writes a "base" frame; here lidar_to_base is not applied).
Settings: each method's own published configuration, as its authors recommend (both turn deskew off):
GenZ-ICP pretuned newer_college.yaml (Newer College), indoor.yaml (Hilti), outdoor.yaml (Oxford Spires, NTU VIRAL);
MAD-ICP default.cfg with its dataset ranges (Newer College / Hilti cfg; 0.7-100 m elsewhere), 4 key frames, 4 threads.
Pose times for the official evaluation (#061): GenZ-ICP / MAD-ICP register raw scans -> a config.yml with data.deskew false, so
evaluate_official.pose_times puts each pose at the mean sweep time like "KISS no deskew"; Traj-LO's poses are at their own
instants -> written also as *_poses_posetime_tum.txt (used as is).
Deterministic apart from multithreaded reductions: one run per sequence (like KISS).
"""
import datetime
import os
import sys
from pathlib import Path

import numpy as np


def car_dataset(seq: Path):
    """#220: our car readers - Boreas (<seq>/lidar + applanix) and MulRan (<seq>/Ouster + global_pose.csv); None otherwise."""
    if (seq / "lidar").is_dir() and (seq / "applanix").is_dir():
        from kiss_slam.tools.boreas import Boreas
        return Boreas(seq)
    if (seq / "Ouster").is_dir() and (seq / "global_pose.csv").exists():
        from kiss_slam.tools.mulran import MulRan
        return MulRan(seq)
    return None


def results_dir(out: Path) -> Path:
    """A timestamped folder, no "latest" link (exFAT has no symbolic links, #081)."""
    d = out.resolve() / datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    d.mkdir(parents=True, exist_ok=True)
    return d


def write_tum(path: Path, poses, stamps):
    from scipy.spatial.transform import Rotation

    rows = [[t, *T[:3, 3], *Rotation.from_matrix(T[:3, :3]).as_quat()] for t, T in zip(stamps, poses)]
    np.savetxt(path, np.array(rows), fmt="%.6f")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    method, seq, out = args[0], Path(args[1]), Path(args[2])
    n_scans = int(args[3]) if len(args) > 3 else -1
    if method not in ("genz", "mad", "trajlo", "cticp"):
        sys.exit(f"method must be genz, mad, trajlo or cticp, not {method!r}")
    if method == "trajlo":
        return run_trajlo(seq, out, opts)
    if method == "cticp":
        return run_cticp(seq, out, n_scans, opts)
    topic = opts.get("topic", "/os_cloud_node/points")
    deskew = opts.get("deskew", "false").lower() in ("true", "1", "yes", "on")

    is_bag = seq.suffix == ".bag" or (seq.is_dir() and any(seq.glob("*.bag")))
    if is_bag:
        from kiss_icp.datasets.rosbag import RosbagDataset
        dataset = RosbagDataset(seq, topic)
        stamps, read = [], dataset.read_point_cloud

        def read_and_stamp(msg):                  # the scan's header stamp, as in run_ncd.py
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            return read(msg)

        dataset.read_point_cloud = read_and_stamp
    elif car_dataset(seq) is not None:                     # #220: Boreas / MulRan (our readers, absolute point times)
        dataset = car_dataset(seq)
        stamps = None
    else:
        from kiss_slam.tools.ncd_pcd import NewerCollege2020Pcd
        dataset = NewerCollege2020Pcd(seq)
        stamps = None
    n = len(dataset) if n_scans < 0 else min(n_scans, len(dataset))
    name = getattr(dataset, "sequence_id", seq.stem)

    poses, t0 = [], datetime.datetime.now()
    if method == "genz":
        from genz_icp.config import load_config
        from genz_icp.genz_icp import GenZICP

        config = load_config(Path(opts["config"]) if "config" in opts else None)
        config.data.deskew = deskew
        odometry = GenZICP(config=config)
        print(f"GenZ-ICP| config {opts.get('config', 'default')}, deskew {config.data.deskew}")
        for i in range(n):
            frame, timestamps = dataset[i][:2]                # the .pcd reader also returns intensity, ring
            odometry.register_frame(frame, timestamps)
            poses.append(np.array(odometry.last_pose))
            if i % 500 == 0:
                print(f"GenZ-ICP| scan {i} / {n}", flush=True)
    else:
        import yaml
        from mad_icp.configurations.mad_params import MADConfiguration_lut
        from mad_icp.src.pybind.pypeline import Pipeline, VectorEigen3d

        cfg = yaml.safe_load(open(opts["config"])) if "config" in opts else {}
        min_range, max_range = cfg.get("min_range", 0.7), cfg.get("max_range", 100.0)
        p = MADConfiguration_lut["default"]
        pipeline = Pipeline(cfg.get("sensor_hz", 10), deskew, p["b_max"], p["rho_ker"], p["p_th"], p["b_min"], p["b_ratio"],
                            4, 4, False)
        print(f"MAD-ICP| ranges {min_range}-{max_range} m, deskew {deskew}, params default, 4 key frames, 4 threads")
        skipped = 0
        for i in range(n):
            frame = dataset[i][0]
            frame = np.asarray(frame, dtype=np.float64)
            r = np.linalg.norm(frame, axis=1)
            frame = frame[(r > min_range) & (r < max_range)]
            ts = stamps[-1] if stamps else float(dataset.get_frames_timestamps()[i]) if hasattr(dataset, "get_frames_timestamps") else i * 0.1
            if len(frame) < 100 and poses:          # MAD-ICP segfaulted near the end of the NCD long experiment (#083):
                poses.append(poses[-1])             # a near-empty scan keeps the last pose, counted in run_info.txt
                skipped += 1
                continue
            pipeline.compute(ts, VectorEigen3d(frame))
            poses.append(np.array(pipeline.currentPose()))
            if i % 500 == 0:
                print(f"MAD-ICP| scan {i} / {n}", flush=True)
    seconds = (datetime.datetime.now() - t0).total_seconds()
    if method == "mad":
        print(f"MAD-ICP| {skipped} scans with < 100 points kept the last pose")

    if stamps is None:
        stamps = list(dataset.get_frames_timestamps())[:n]
    d = results_dir(out)
    write_tum(d / f"{name}_poses_tum.txt", poses, stamps[:len(poses)])
    np.save(d / f"{name}_poses.npy", np.array(poses))
    # The pose-time convention of evaluate_official.pose_times (#061): a scan registered raw stands near its MEAN point time,
    # as the "KISS no deskew" arm.  Recorded the way our arms record it (config.yml data.deskew), so no offset is tuned here.
    (d / "config.yml").write_text(f"# written by run_baseline.py for evaluate_official.pose_times\ndata:\n  deskew: {str(deskew).lower()}\n")
    (d / "run_info.txt").write_text(f"method {method}\nsequence {seq}\nscans {len(poses)}\nconfig {opts.get('config', 'default')}\n"
                                    f"deskew {deskew}\nseconds {seconds:.1f}\nHz {len(poses) / seconds:.2f}\n")
    print(f"{method}| {len(poses)} scans in {seconds:.0f} s ({len(poses) / seconds:.1f} Hz) -> {d}")


TRAJLO = Path("/home/photogrammetry/baselines/Traj-LO")


def run_trajlo(seq: Path, out: Path, opts):
    """Traj-LO headless on its own reader; its pose file rewritten as <name>_poses_tum.txt without the comment line."""
    import subprocess
    import yaml

    kind = opts.get("config", "ouster")
    cfg = yaml.safe_load(open(TRAJLO / "data" / f"config_{kind}.yaml"))
    d = results_dir(out)
    eye = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    cfg["dataset"].update(path=str(seq), topic=opts.get("topic", cfg["dataset"]["topic"]), save_pose=True,
                          pose_file_path=str(d / "estimated_pose.txt"))
    cfg["calibration"].update(time_offset=0.0, T_body_lidar=eye, T_body_gt=eye)
    yaml.safe_dump(cfg, open(d / "trajlo_config.yaml", "w"), sort_keys=False)
    t0 = datetime.datetime.now()
    rc = subprocess.run([str(TRAJLO / "build" / "trajlo_headless"), str(d / "trajlo_config.yaml")]).returncode
    seconds = (datetime.datetime.now() - t0).total_seconds()
    est = np.loadtxt(d / "estimated_pose.txt", comments="#")
    name = seq.stem if seq.suffix == ".bag" else seq.name
    np.savetxt(d / f"{name}_poses_tum.txt", est, fmt="%.9f")
    # Each Traj-LO pose is already at the instant it stands for (segment boundaries): the exact file of pose_times (#061).
    np.savetxt(d / f"{name}_poses_posetime_tum.txt", est, fmt="%.9f")
    (d / "run_info.txt").write_text(f"method trajlo\nsequence {seq}\nconfig config_{kind}.yaml\nposes {len(est)}\n"
                                    f"exit {rc}\nseconds {seconds:.1f}\n")
    print(f"trajlo| exit {rc}, {len(est)} poses in {seconds:.0f} s -> {d}")


CTICP = Path("/home/photogrammetry/baselines/ct_icp/install/CT_ICP/bin/stream_odometry")


def run_cticp(seq: Path, out: Path, n_scans, opts):
    """CT-ICP on our raw readers (absolute point times), streamed to cmd_stream_odometry.cpp."""
    import subprocess

    is_bag = seq.suffix == ".bag" or (seq.is_dir() and any(seq.glob("*.bag")))
    if is_bag:
        from kiss_icp.datasets.rosbag import RosbagDataset
        from kiss_slam.tools.point_cloud2 import read_point_cloud_raw
        dataset = RosbagDataset(seq, opts.get("topic", "/os_cloud_node/points"))
        dataset.read_point_cloud = read_point_cloud_raw          # absolute per-point times in s (header stamp + t)
    elif car_dataset(seq) is not None:                           # #220: Boreas / MulRan, absolute point times
        dataset = car_dataset(seq)
    else:
        from kiss_slam.tools.ncd_pcd import NewerCollege2020Pcd
        dataset = NewerCollege2020Pcd(seq)                         # returns absolute point times already
    n = len(dataset) if n_scans < 0 else min(n_scans, len(dataset))
    name = getattr(dataset, "sequence_id", seq.stem)
    d = results_dir(out)
    t0 = datetime.datetime.now()
    profile = opts.get("profile", "robust_low_inertia")          # the authors' profiles: robust_low_inertia | driving | robust_driving
    proc = subprocess.Popen([str(CTICP), str(d / "estimated_pose.txt"), opts.get("threads", "4"), profile], stdin=subprocess.PIPE)
    for i in range(n):
        xyz, t = dataset[i][:2]
        xyz = np.asarray(xyz, dtype=np.float64)
        t = np.asarray(t, dtype=np.float64)
        keep = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.5)
        block = np.column_stack([xyz[keep], t[keep]])
        proc.stdin.write(np.int64(len(block)).tobytes())
        proc.stdin.write(np.ascontiguousarray(block).tobytes())
    proc.stdin.write(np.int64(-1).tobytes())
    proc.stdin.close()
    rc = proc.wait()
    seconds = (datetime.datetime.now() - t0).total_seconds()
    est = np.loadtxt(d / "estimated_pose.txt")
    np.savetxt(d / f"{name}_poses_tum.txt", est, fmt="%.9f")
    # The end pose of each scan at its last point's time: the exact file of pose_times (#061).
    np.savetxt(d / f"{name}_poses_posetime_tum.txt", est, fmt="%.9f")
    (d / "run_info.txt").write_text(f"method cticp\nsequence {seq}\nprofile {profile}\nscans {len(est)}\n"
                                    f"exit {rc}\nseconds {seconds:.1f}\nHz {len(est) / seconds:.2f}\n")
    print(f"cticp| exit {rc}, {len(est)} scans in {seconds:.0f} s ({len(est) / seconds:.1f} Hz) -> {d}")


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
