#!/usr/bin/env python3
"""One arm of KISS-SLAM on a Newer College sequence (Ouster).

    python scripts/run_ncd.py <arm> <sequence> <out dir> [n_scans] [--config=<yaml>] [--seed=N] [--parallel]
                              [--topic=/os_cloud_node/points] [--intensity-scale=0.249] [--diag]
                              [--parts=full|translation|rotation] [--rot-smooth=k] [--rot-cv=w] [--save-frames=<voxel m>] [--save-fraction=f]
                              [--gate [--gate-min=0] [--gate-rot=10] [--gate-drot=8]] [--fallback=identity|cv|kiss] [--two-start=<deg>|none] [--two-start-margin=0.02] [--range=fallback|candidate|validate] [--validate-k=3] [--two-start-always] [--voxel=auto]
                              [--rotation-weight=100] [--save-failed]
                              [--normalise=gain|gain_clahe] [--panorama-width=2048|auto] [--panorama-up=4|auto] [--image-start=false]
                              [--stuck=none|<m>] [--sigma=adaptive|<m>] [--deskew=false]
                              [--model=cv|car|ca] [--deskew-rotation=cv] [--redeskew] [--surf-upright | --no-upright] [--surf-hessian=<threshold>]
                              [--bearings=<min range m>] [--guided=<window px>|none] [--guided-predict=shift|motion]
                              [--sectors=8] [--whiten=<sr>,<saz>,<sel>] [--cross-check] [--detect-scale=0.5] [--motion-file=<npz>] [--deskew-from=image|cv] [--two-start-kiss]

sequence: a 2020 sequence dir with raw_format/ouster_scan/*.pcd (kiss_slam/tools/ncd_pcd.py), or a
          .bag file, or a folder whose *.bag are ONE split sequence (read in time order; 2021 bags), or a KITTI raw
          drive dir with velodyne_points/ (kiss_slam/tools/kitti_raw.py; --first / --last: scan range, --intensity-scale=255).
arm:  kiss  upstream KISS-SLAM (image_deskew off)
      sift  image-motion deskew (i3), SIFT features on the intensity panorama
      surf  image-motion deskew (i3), SURF features (OpenCV with OPENCV_ENABLE_NONFREE)
All arms read the same scans with the same config (default: the KISS-SLAM defaults, the setting of
the KISS-SLAM paper); only image_deskew.enabled / detector differ.  --seed: RANSAC seed of the
image-motion estimator, for measuring run-to-run spread (#037).  --parallel: the image motion in a
worker process, overlapping the ICP (image_deskew.parallel); same trajectory, less time per scan.
--intensity-scale: image_deskew.intensity_scale, default 255/1024 for the Ouster signal (#041).
--parts / --rot-smooth: ablation of the image motion (image_deskew.use_parts / rotation_smoothing, #047).
--fuse-range: range-panorama matches in the motion fit too (#090).  --drop-stationary: drop the pairs zero motion explains (#132).
--cv-blend=adaptive [--cv-blend-window=20]: blend of the image motion and the constant velocity, weights from their recent errors
against the ICP (image_deskew.cv_blend, #131).  --multi-baseline: the image-motion fit also with the matches of scan k-2 (#130); --multi-baseline=translation: only its translation (#131).
--rot-cv: weight w of the constant-velocity rotation in the image rotation (image_deskew.rotation_cv_weight, #092; 0 = off).
--save-frames: keep every deskewed scan (voxel-downsampled) in deskewed_frames.npz, for the map-sharpness test (#048);
--save-fraction: only this random fraction of each scan's points (#049).
--gate: plausibility gate on each image motion (>= gate-min matches, 0 = off by default; rotation <= gate-rot deg, change from the last
accepted <= gate-drot deg), constant-velocity fallback, and the rejected / failed pairs saved in <out>/rejected_pairs (#054).
--save-failed: for every scan whose intensity motion fails, both panoramas, their matches and a row in rejected.csv, in
<out>/failed_matches (the images of scripts/dump_failed_matches.py, during the run).
--stuck / --sigma: ablations of the paper (open_tasks D) - the near-floor stuck-match filter off (image_deskew.stuck_min = None,
#025-#027) and the KISS adaptive threshold instead of the fixed sigma 2.0 (image_deskew.fixed_sigma = None, #031).
--deskew=false: the image motion only as the ICP start, the scan not deskewed (image_deskew.use_for_deskew, #082).
--diag: per-scan ICP diagnostics (a KD-tree over the local map per scan; off by default here, it
does not change the trajectory).  For bags the written timestamps are the scans' header stamps
(the loader's own are the bag record times), so the evaluation matches them to the ground truth.
"""
import os
import sys
from pathlib import Path


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    arm, seq, out = args[0], Path(args[1]), Path(args[2])
    n_scans = int(args[3]) if len(args) > 3 else -1
    if arm not in ("kiss", "sift", "surf"):
        sys.exit(f"arm must be kiss, sift or surf, not {arm!r}")
    os.environ["KISS_SLAM_OUT_DIR"] = str(out)          # read when the config is built

    import kiss_slam.pipeline as pipeline

    from kiss_slam.tools.livox import LivoxRosbag
    from kiss_slam.tools.ros2bags import Ros2Bags, ros2_bag_dirs
    is_bag = seq.suffix == ".bag" or (seq.is_dir() and any(seq.glob("*.bag"))) or bool(ros2_bag_dirs(seq))
    if is_bag and ros2_bag_dirs(seq):                       # ROS2 bags without type definitions, possibly split (#104)
        dataset = Ros2Bags(seq, opts.get("topic", "/livox/points"))
    elif is_bag:
        from kiss_icp.datasets.rosbag import RosbagDataset
        topic = opts.get("topic", "/os_cloud_node/points")
        if LivoxRosbag.is_livox(seq, topic):                # livox_ros_driver/CustomMsg (#098)
            if arm != "kiss" and "motion-file" not in opts:
                sys.exit("Livox (non-repetitive scan): no online image panorama for this sensor yet (open_tasks B.10) - arm kiss, or --motion-file")
            dataset = LivoxRosbag(seq, topic)
        else:
            dataset = RosbagDataset(seq, topic)
    elif (seq / "lidar").is_dir() and (seq / "applanix").is_dir():   # Boreas sequence (#085)
        from kiss_slam.tools.boreas import Boreas
        dataset = Boreas(seq, int(opts.get("first", 0)), int(opts["last"]) if "last" in opts else None)
    elif (seq / "velodyne_points").is_dir():                # KITTI raw drive, sync or extract (#084)
        from kiss_slam.tools.kitti_raw import KittiRaw
        dataset = KittiRaw(seq, int(opts.get("first", 0)), int(opts["last"]) if "last" in opts else None,
                           correct="--kitti-correction" in sys.argv)   # vertical-angle correction (#085)
    else:
        from kiss_slam.tools.ncd_pcd import NewerCollege2020Pcd
        dataset = NewerCollege2020Pcd(seq)

    auto_voxel = None
    if opts.get("voxel") == "auto":                          # #110: voxel from the sensor and the scene, v = sqrt(20) x median range x point spacing
        import numpy as _np
        xyz0 = _np.asarray(dataset[0][0], dtype=float)
        dataset.reset()
        r = _np.linalg.norm(xyz0, axis=1); xyz0 = xyz0[r > 1.0]; r = r[r > 1.0]
        az = _np.degrees(_np.arctan2(xyz0[:, 1], xyz0[:, 0])); el = _np.degrees(_np.arctan2(xyz0[:, 2], _np.linalg.norm(xyz0[:, :2], axis=1)))
        area = (_np.percentile(az, 99.5) - _np.percentile(az, 0.5)) * (_np.percentile(el, 99.5) - _np.percentile(el, 0.5))
        spacing = _np.radians(_np.sqrt(area / len(xyz0)))
        auto_voxel = float(_np.clip(_np.sqrt(20.0) * _np.median(r) * spacing, 0.05, 2.0))
        print(f"run_ncd| voxel auto: {auto_voxel:.3f} m (median range {_np.median(r):.1f} m, point spacing {_np.degrees(spacing):.3f} deg, "
              f"{len(xyz0)} points, FoV {area:.0f} deg2)", flush=True)

    # Options as config fields, so they also reach the image-motion worker process.
    load_config = pipeline.load_config

    def load_with_overrides(path):
        config = load_config(path)
        if auto_voxel is not None:                           # #110
            config.odometry.mapping.voxel_size = auto_voxel
            config.local_mapper.voxel_size = auto_voxel
        config.image_deskew.seed = int(opts.get("seed", 0))
        config.image_deskew.parallel = "--parallel" in sys.argv
        config.image_deskew.intensity_scale = float(opts.get("intensity-scale", 255.0 / 1024.0))
        config.diagnostics.icp_metrics = "--diag" in sys.argv
        config.image_deskew.use_parts = opts.get("parts", "full")
        config.image_deskew.rotation_smoothing = int(opts.get("rot-smooth", 1))
        config.image_deskew.rotation_cv_weight = float(opts.get("rot-cv", 0.0))
        if "cv-blend" in opts:                                   # #131: adaptive blend of the image motion and the constant velocity
            config.image_deskew.cv_blend = opts["cv-blend"]
            config.image_deskew.cv_blend_window = int(opts.get("cv-blend-window", 20))
            config.image_deskew.cv_blend_part = opts.get("cv-blend-part", "full")     # #137: full | rotation
            config.image_deskew.cv_blend_use = opts.get("cv-blend-use", "both")       # #137: both | deskew
        if "--multi-baseline" in sys.argv:                       # #130: the motion fit with the matches of k-2 <-> k too
            config.image_deskew.multi_baseline = True
        if opts.get("multi-baseline") == "translation":          # #131: only its translation, the rotation of the two-scan fit
            config.image_deskew.multi_baseline = "translation"
        if "--fuse-range" in sys.argv:                           # #090: range-panorama matches in the same motion fit
            config.image_deskew.fuse_range = True
        if "--drop-stationary" in sys.argv:                      # #132: drop pairs zero motion explains as well, refit
            config.image_deskew.drop_stationary = True
        if "sectors" in opts:                                    # #093: equal weight per azimuth sector in the time fit
            config.image_deskew.fit_sectors = int(opts["sectors"])
            config.image_deskew.fit_sectors_part = opts.get("sectors-part", "both")   # #135
        if "whiten" in opts:                                     # #093: range / azimuth / elevation whitening, one loss per point
            config.image_deskew.whiten = [float(v) for v in opts["whiten"].split(",")]
        if "detect-scale" in opts:                               # #093: panorama scale for the detector only (speed)
            config.image_deskew.detect_scale = float(opts["detect-scale"])
        if "--cross-check" in sys.argv:                          # #093: mutual best matches only
            config.image_deskew.cross_check = True
        if "--gate" in sys.argv or "gate-min" in opts:           # plausibility gate + constant-velocity fallback (#054)
            config.image_deskew.gate_min_matches = int(opts.get("gate-min", 0))    # 0 = off (#054: Blenheim has few matches everywhere)
            config.image_deskew.gate_max_rotation_deg = float(opts.get("gate-rot", 10.0))
            config.image_deskew.gate_max_rotation_change_deg = float(opts.get("gate-drot", 8.0))
            config.image_deskew.fallback = "constant_velocity"
            config.image_deskew.save_rejected_dir = str(out / "rejected_pairs")
        if "--save-failed" in sys.argv:                          # panoramas + matches of every scan whose intensity motion failed
            config.image_deskew.save_rejected_dir = str(out / "failed_matches")
        if "fallback" in opts:                                   # identity | constant_velocity (#057) | kiss (#146: CV deskew + start on failed scans)
            config.image_deskew.fallback = {"cv": "constant_velocity"}.get(opts["fallback"], opts["fallback"])
        if "two-start" in opts:                                  # register twice when image and CV disagree (#057)
            v = opts["two-start"]                            # "none" / "off": single start (every result before #059)
            config.image_deskew.two_start_deg = None if v.lower() in ("none", "off") else float(v)
        if "two-start-margin" in opts:                           # switch only when the fit is better by this fraction
            config.image_deskew.two_start_margin = float(opts["two-start-margin"])
        if "image-start" in opts:                                # false: ICP starts from constant velocity, image only deskews (#030)
            config.image_deskew.use_as_initial_guess = opts["image-start"].lower() not in ("false", "0", "no", "off")
        if "normalise" in opts:                                  # per-scan intensity normalisation: gain | gain_clahe (#075)
            config.image_deskew.intensity_normalisation = opts["normalise"]
        if "panorama-up" in opts:                                # vertical upscaling of the panorama, e.g. 4 for 128 beams (#089)
            v = opts["panorama-up"]                              # "auto" (#093): square pixels from the first scan's ring spacing
            config.image_deskew.panorama_up = v if v == "auto" else int(v)
        if "panorama-width" in opts:                             # panorama columns, e.g. 2048 for the Hilti Ouster (#075)
            v = opts["panorama-width"]                           # "auto" (#093): the sensor's own columns, from the first scan
            config.image_deskew.panorama_width = v if v == "auto" else int(v)
        if "rotation-weight" in opts:                            # rotation information of the node graph (#067)
            config.pose_graph_optimizer.rotation_weight = float(opts["rotation-weight"])
        if "range" in opts:                                      # range-image motion: fallback | candidate (#058)
            config.image_deskew.range_motion = opts["range"]
        if "stuck" in opts:                                      # near-floor stuck-match filter: none = off (ablation)
            v = opts["stuck"]
            config.image_deskew.stuck_min = None if v.lower() in ("none", "off") else float(v)
        if "sigma" in opts:                                      # adaptive = KISS adaptive threshold instead of fixed (ablation)
            v = opts["sigma"]
            config.image_deskew.fixed_sigma = None if v.lower() == "adaptive" else float(v)
        if "deskew" in opts:                                     # false: image motion only as ICP start, no deskew (#082)
            config.image_deskew.use_for_deskew = opts["deskew"].lower() not in ("false", "0", "no", "off")
        if "deskew-rotation" in opts:                            # image | cv: hybrid deskew (#086)
            config.image_deskew.deskew_rotation = opts["deskew-rotation"]
        if "--redeskew" in sys.argv:                             # second deskew pass with the ICP's motion (#086)
            config.image_deskew.redeskew = True
        if "--surf-upright" in sys.argv:                         # upright SURF: no keypoint orientation (#086; default since #088)
            config.image_deskew.surf_upright = True
        if "surf-hessian" in opts:                               # SURF Hessian threshold (default 100; #089: 200 = -21 % time on Ouster 128)
            config.image_deskew.surf_hessian_threshold = float(opts["surf-hessian"])
        if "--no-upright" in sys.argv:                           # the SURF of every result before #088
            config.image_deskew.surf_upright = False
        if "bearings" in opts:                                   # rotation from bearings of matches beyond <m> (#087)
            config.image_deskew.rotation_from_bearings = float(opts["bearings"])
        if "guided" in opts:                                     # guided matching within <px> columns (#087; default 40 since #088)
            v = opts["guided"]                                   # "none": brute force, every result before #088
            config.image_deskew.guided_matching_window = None if v.lower() in ("none", "off") else float(v)
        if "guided-predict" in opts:                             # shift | motion: centre of the guided window (#089)
            config.image_deskew.guided_prediction = opts["guided-predict"]
        if "validate-k" in opts:                                 # #109: range validation threshold (x running median)
            config.image_deskew.validate_k = float(opts["validate-k"])
        if "--two-start-always" in sys.argv:                     # #109: every scan from all starts (unconditional floor)
            config.image_deskew.two_start_always = True
        if "--two-start-kiss" in sys.argv:                        # second start / image-failure fallback = KISS (CV deskew + start) (#108)
            config.image_deskew.two_start_kiss = True
        if "deskew-from" in opts:                                # image | cv: deskew from constant velocity, image only as ICP start (#103)
            config.image_deskew.deskew_from = opts["deskew-from"]
        if "motion-file" in opts:                                # precomputed image motions (N, 4, 4), NaN = failed (#101: Livox)
            config.image_deskew.motion_file = opts["motion-file"]
        if "oracle-deskew" in opts:                              # diagnostic: deskew from ground-truth motion (#086)
            config.image_deskew.deskew_motion_file = opts["oracle-deskew"]
        if "model" in opts:                                      # image motion model: cv | car | ca (#021, #086)
            config.image_deskew.model = opts["model"]
        if "save-frames" in opts:
            config.diagnostics.save_deskewed_voxel = float(opts["save-frames"])
            config.diagnostics.save_deskewed_fraction = float(opts.get("save-fraction", 1.0))
        return config

    pipeline.load_config = load_with_overrides
    slam_pipeline = pipeline.SlamPipeline(
        dataset=dataset,
        config_file=Path(opts["config"]) if "config" in opts else None,
        n_scans=n_scans,
        image_deskew=arm != "kiss",
        image_detector=None if arm == "kiss" else arm,
    )
    if is_bag and not isinstance(dataset, LivoxRosbag):     # LivoxRosbag records its own header stamps (ROS time, #098)
        # After SlamPipeline installed its reader: record each scan's header stamp on the way through.
        stamps, read = [], dataset.read_point_cloud

        def read_and_stamp(msg):
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            return read(msg)

        dataset.read_point_cloud = read_and_stamp
        dataset.get_frames_timestamps = lambda: stamps
    slam_pipeline.run().print()


if __name__ == "__main__":      # required: image_deskew.parallel starts its worker with "spawn"
    main()
    # Everything is written: leave without the interpreter's shutdown.  (Added for a run of #044 that looked like a
    # hang at exit — main thread gone, pool threads waiting on a futex; the real cause was a kernel BUG in the ntfs3
    # driver while writing to the NTFS data disk, #046.  Kept: it does no harm.)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
