# MIT License
#
# Copyright (c) 2025 Tiziano Guadagnino, Benedikt Mersch, Saurabh Gupta, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import csv
import os
import time
from pathlib import Path
from typing import Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from kiss_icp.pipeline import OdometryPipeline
from scipy.spatial.transform import Rotation
from tqdm import tqdm, trange

from kiss_slam.config import load_config
from kiss_slam.config.config import write_config as write_slam_config
from kiss_slam.occupancy_mapper import OccupancyGridMapper
from kiss_slam.slam import KissSLAM
from kiss_slam.tools.visualizer import RegistrationVisualizer, StubVisualizer

# ── CSV column order ──────────────────────────────────────────────────────────
_ICP_CSV_FIELDS = [
    "frame_idx",
    # ICP residuals
    "icp_rms_error", "icp_mean_error", "icp_max_error",
    "icp_inlier_count", "icp_inlier_ratio",
    # point counts
    "n_source_pts", "n_filtered_pts", "n_map_pts",
    # adaptive threshold
    "adaptive_sigma",
    # intensity flag
    "has_intensity",
    # motion
    "motion_trans_m", "motion_rot_deg",
    # model deviation
    "model_dev_trans_m", "model_dev_rot_deg",
    # geometric degeneracy
    "geo_linearity", "geo_planarity", "geo_sphericity",
    "geo_condition", "geo_omnivariance",
]

_C = {
    "rms": "#e74c3c", "mean": "#e67e22", "max_err": "#c0392b",
    "inlier_cnt": "#2980b9", "inlier_rat": "#8e44ad",
    "sigma": "#27ae60", "trans": "#16a085", "rot": "#d35400",
    "dev_trans": "#2c3e50", "dev_rot": "#7f8c8d",
    "planarity": "#e74c3c", "linearity": "#3498db",
    "sphericity": "#2ecc71", "condition": "#e67e22",
}


class SlamPipeline(OdometryPipeline):
    def __init__(
        self,
        dataset,
        config_file: Optional[Path] = None,
        visualize: bool = False,
        n_scans: int = -1,
        jump: int = 0,
        refuse_scans: bool = False,
        use_intensity: Optional[bool] = None,
        intensity_mode: Optional[str] = None,
        image_deskew: Optional[bool] = None,
        motion_file: Optional[str] = None,
        image_detector: Optional[str] = None,
    ):
        super().__init__(dataset=dataset, config=None, n_scans=n_scans, jump=jump)
        self.slam_config = load_config(config_file)
        # CLI overrides win over the YAML, so single flags pick the experiment arm.
        if use_intensity is not None:
            self.slam_config.intensity.enabled = use_intensity
        if intensity_mode is not None:
            self.slam_config.intensity.mode = intensity_mode
        if image_deskew is not None:
            self.slam_config.image_deskew.enabled = image_deskew
        if motion_file is not None:
            self.slam_config.image_deskew.motion_file = str(motion_file)
        if image_detector is not None:
            # Plain attribute assignment skips the config's Literal check, so check here.
            if image_detector not in ("sift", "surf", "orb", "akaze"):
                raise ValueError(f"--image-detector must be sift, surf, orb or akaze, not {image_detector!r}")
            self.slam_config.image_deskew.detector = image_detector
        self.use_intensity = self.slam_config.intensity.enabled
        self.use_image_deskew = self.slam_config.image_deskew.enabled
        # Online image deskew needs the raw scan (intensity on the sensor scale + ring).
        self.use_raw_reader = self.use_image_deskew and self.slam_config.image_deskew.motion_file is None
        img = self.slam_config.image_deskew
        print(
            "KissSLAM| Experiment arm: "
            + (
                f"INTENSITY-AIDED ({self.slam_config.intensity.mode})"
                if self.use_intensity
                else f"IMAGE-MOTION DESKEW ({img.model}{', subpixel' if img.subpixel else ''}"
                     f"{f', stuck {img.stuck_min:g}' if img.stuck_min is not None else ''}"
                     f"{' floor-only' if img.stuck_min is not None and img.stuck_floor_only else ''}"
                     f"{', NO deskew' if not img.use_for_deskew else ''}"
                     f"{', deskew rotation CV' if img.deskew_rotation == 'cv' else ''}{', 2nd deskew pass' if img.redeskew else ''}"
                     f"{', + ICP initial guess' if img.use_as_initial_guess else ''}"
                     f", sigma {'fixed ' + format(img.fixed_sigma, 'g') if img.fixed_sigma is not None else 'adaptive'}"
                     f", motion {'online, ' + img.detector.upper() + (' upright' if img.detector == 'surf' and img.surf_upright else '') + (f', guided {img.guided_matching_window:g} px' if img.guided_matching_window else '') + (', parallel' if img.parallel else '') if img.motion_file is None else img.motion_file})"
                if self.use_image_deskew
                else "BASELINE (vanilla KISS-SLAM)"
            )
        )
        self.config = self.slam_config.kiss_icp_config()
        self.visualize = visualize
        self.kiss_slam = KissSLAM(self.slam_config)
        self.kiss_slam.first_scan_index = self._first      # motion_file rows are sequence scans
        self.visualizer = RegistrationVisualizer() if self.visualize else StubVisualizer()
        self.refuse_scans = refuse_scans

        # Patch the dataset's read_point_cloud with our intensity-aware version
        # if it is a RosbagDataset (or any dataset that uses kiss_icp's reader).
        self._patch_dataset_reader()

    def _patch_dataset_reader(self):
        """Replace the dataset's point-cloud reader with our intensity-aware version.

        kiss_icp's RosbagDataset stores `self.read_point_cloud` as an instance
        attribute pointing to `kiss_icp.tools.point_cloud2.read_point_cloud`.
        We swap it for our extended version that also returns intensity.

        For dataset types that don't use this pattern the patch is a no-op.

        Online image deskew (image_deskew.enabled, no motion_file) installs the RAW
        reader instead: intensity on the sensor's own scale plus the ring of every point,
        which the intensity panorama needs (same input as precompute_i3_motion.py).
        """
        if not (self.use_intensity or self.use_raw_reader):
            print("KissSLAM| Keeping the upstream point cloud reader.")
            return
        if hasattr(self._dataset, "read_point_cloud"):
            if self.use_raw_reader:
                from kiss_slam.tools.point_cloud2 import read_point_cloud_raw as _rpc

                self._dataset.read_point_cloud = _rpc
                print("KissSLAM| Raw point cloud reader (intensity + ring) installed.")
            else:
                from kiss_slam.tools.point_cloud2 import read_point_cloud as _rpc

                self._dataset.read_point_cloud = _rpc
                print("KissSLAM| Intensity-aware point cloud reader installed.")
        else:
            print("KissSLAM| Dataset does not use patching; intensity/ring unavailable from reader.")

    @staticmethod
    def _get_results_dir(out_dir: str):
        """kiss_icp's results folder, but the "latest" link is optional: exFAT has no symbolic links, and the upstream
        os.symlink ran only after the whole run and killed it before any result was written (all 324 runs of #081)."""
        import datetime

        results_dir = os.path.join(os.path.realpath(out_dir), datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        latest_dir = os.path.join(os.path.realpath(out_dir), "latest")
        os.makedirs(results_dir, exist_ok=True)
        try:
            if os.path.lexists(latest_dir):
                os.unlink(latest_dir)
            os.symlink(results_dir, latest_dir)
        except OSError as e:
            print(f"KissSLAM| no 'latest' link in {out_dir} ({e.strerror}); results in {results_dir}")
        return results_dir

    def run(self):
        self._run_pipeline()
        self._run_evaluation()
        self._evaluate_closures()
        self._create_output_dir()
        self._write_result_poses()
        self._write_pose_times()
        self._write_gt_poses()
        self._write_cfg()
        self._write_slam_cfg()
        self._write_log()
        self._write_graph()
        self._write_closures()
        self._write_local_maps()
        self._write_deskewed_frames()
        self._write_image_motions()
        self._write_two_start_log()
        self._write_icp_metrics()
        self._plot_icp_residuals()
        self._plot_motion_and_deviation()
        self._plot_geometric_degeneracy()
        self._global_mapping()
        return self.results

    # ─────────────────────────────────────────────────────────────────────────
    # Dataset access — now returns (frame, timestamps, intensity, ring)
    # ─────────────────────────────────────────────────────────────────────────

    def _next(self, idx) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (frame, timestamps, intensity, ring) for scan at index `idx`.

        The dataset's __getitem__ returns a 2-tuple (upstream reader), a 3-tuple
        (intensity reader) or a 4-tuple (raw reader, intensity + ring) depending on
        which reader was installed; the missing items are None.
        """
        dataframe = self._dataset[idx]
        frame, timestamps = dataframe[0], dataframe[1]
        intensity = dataframe[2] if len(dataframe) > 2 else None
        ring = dataframe[3] if len(dataframe) > 3 else None
        return frame, timestamps, intensity, ring

    # ─────────────────────────────────────────────────────────────────────────
    # Pipeline runner
    # ─────────────────────────────────────────────────────────────────────────

    def _run_pipeline(self):
        # image_deskew.parallel: scan idx+1 is read and queued to the image-motion worker before the
        # ICP of scan idx, so the two overlap.  Reading stays in order (serial rosbag reader).
        prefetch = self.kiss_slam.image_motion_parallel
        pending = None
        # #176: scans are read (in order, the reader is serial) by a background thread a few scans ahead, so reading overlaps
        # the registration; the scans and their order are unchanged (ported from b9_joint_new, #159).
        import queue, threading
        q = queue.Queue(maxsize=4)

        def _read_all():
            try:
                for i in range(self._first, self._last):
                    q.put((self._next(i), None))
            except BaseException as e:                    # re-raised in the main thread
                q.put((None, e))
        threading.Thread(target=_read_all, daemon=True).start()

        def read_next():
            item, err = q.get()
            if err is not None:
                raise err
            return item
        for idx in trange(self._first, self._last, unit=" frames", dynamic_ncols=True):
            if prefetch:
                if pending is None:
                    pending = read_next()
                    self.kiss_slam.submit_image_motion(*pending)
                frame, timestamps, intensity, ring = pending
                pending = None
                if idx + 1 < self._last:
                    pending = read_next()
                    self.kiss_slam.submit_image_motion(*pending)
            else:
                frame, timestamps, intensity, ring = read_next()
            start_time = time.perf_counter_ns()
            self.kiss_slam.process_scan(frame, timestamps, intensity, ring)
            self.times[idx - self._first] = time.perf_counter_ns() - start_time
            self.visualizer.update(self.kiss_slam)
        self.kiss_slam.close_image_motion()
        self.kiss_slam.generate_new_node()
        self.kiss_slam.local_map_graph.erase_last_local_map()
        self.poses, self.pose_graph = self.kiss_slam.fine_grained_optimization()
        self.poses = np.array(self.poses)
        if self.use_image_deskew:
            n = self._last - self._first
            print(f"KissSLAM| image motion: {n - self.kiss_slam.n_image_motion_failures}/{n} scans "
                  f"({self.kiss_slam.n_image_motion_failures} fell back to identity)")
            if self.kiss_slam.n_two_start:
                total = self.times.sum() * 1e-9
                print(f"KissSLAM| two starting points: {self.kiss_slam.n_two_start} scans registered twice, "
                      f"constant-velocity start kept in {self.kiss_slam.n_two_start_cv_won}; extra time "
                      f"{self.kiss_slam.two_start_seconds:.1f} s of {total:.1f} s in process_scan "
                      f"({100 * self.kiss_slam.two_start_seconds / max(total, 1e-9):.1f} %)")
            if self.kiss_slam.n_validate_forced:
                print(f"KissSLAM| range validation (#109): {self.kiss_slam.n_validate_forced} scans sent to the two-start choice "
                      f"(image vs range rotation > {self.kiss_slam.image_cfg.validate_k} x running median)")
            est = self.kiss_slam._image_motion_est
            if est is not None and getattr(est, "range_motion", None) == "fallback":
                print(f"KissSLAM| range image used for {est.n_range_used} scans where the intensity motion failed")

    # ─────────────────────────────────────────────────────────────────────────
    # CSV
    # ─────────────────────────────────────────────────────────────────────────

    def _write_slam_cfg(self):
        """Dump the *full* KissSLAMConfig next to the results.

        `OdometryPipeline._write_cfg` only stores the KISS-ICP subset, which
        does not include `intensity.*` or `image_deskew.*` — without this the
        output directory does not record which arm produced it.
        """
        path = os.path.join(self.results_dir, "slam_config.yaml")
        write_slam_config(self.slam_config, path)
        print(f"KissSLAM| SLAM config       → {path}")

    def _write_icp_metrics(self):
        metrics = self.kiss_slam.icp_metrics_log
        if not metrics:
            print("KissSLAM| No ICP metrics to write.")
            return
        csv_path = os.path.join(self.results_dir, "icp_metrics.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=_ICP_CSV_FIELDS, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(metrics)
        print(f"KissSLAM| ICP metrics CSV  → {csv_path}  ({len(metrics)} frames)")

    # ─────────────────────────────────────────────────────────────────────────
    # Plots
    # ─────────────────────────────────────────────────────────────────────────

    def _metrics(self):
        return self.kiss_slam.icp_metrics_log

    @staticmethod
    def _col(metrics, key):
        return [m.get(key, float("nan")) for m in metrics]

    @staticmethod
    def _save(fig, path):
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"KissSLAM| plot saved → {path}")

    def _plot_icp_residuals(self):
        m = self._metrics()
        if not m:
            return
        frames        = self._col(m, "frame_idx")
        rms           = self._col(m, "icp_rms_error")
        mean_err      = self._col(m, "icp_mean_error")
        max_err       = self._col(m, "icp_max_error")
        inlier_count  = self._col(m, "icp_inlier_count")
        n_src         = self._col(m, "n_source_pts")
        n_filt        = self._col(m, "n_filtered_pts")
        inlier_ratio  = self._col(m, "icp_inlier_ratio")
        sigma         = self._col(m, "adaptive_sigma")

        fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True)
        fig.suptitle("ICP Residual Diagnostics – KISS-SLAM (intensity-aware)",
                     fontsize=13, fontweight="bold")

        ax = axes[0]
        ax.plot(frames, rms,      color=_C["rms"],     lw=0.9, label="RMS error")
        ax.plot(frames, mean_err, color=_C["mean"],    lw=0.9, label="Mean error", linestyle="--")
        ax.plot(frames, max_err,  color=_C["max_err"], lw=0.6, label="Max error", alpha=0.5)
        ax.set_ylabel("Distance (m)")
        ax.set_title("Point-to-map error (inliers only, intensity-filtered source)")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.35)

        ax = axes[1]
        ax.fill_between(frames, n_src,  alpha=0.10, color="grey",             label="Source pts (raw)")
        ax.fill_between(frames, n_filt, alpha=0.20, color=_C["inlier_cnt"],   label="Source pts (intensity-filtered)")
        ax.plot(frames, inlier_count,   color=_C["inlier_cnt"], lw=0.9,       label="Inlier pts")
        ax.set_ylabel("Points")
        ax.set_title("Point counts: raw source / intensity-filtered / inliers")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.35)

        ax = axes[2]
        ax.plot(frames, inlier_ratio, color=_C["inlier_rat"], lw=0.9)
        ax.axhline(0.30, color=_C["rms"], linestyle="--", lw=1.0,
                   label="Drift-risk threshold (0.30)")
        ax.set_ylim(0, 1.05)
        ax.set_ylabel("Ratio [0–1]")
        ax.set_title("Inlier ratio  (below 0.30 → likely drift)")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.35)

        ax = axes[3]
        ax.plot(frames, sigma, color=_C["sigma"], lw=0.9, label="Adaptive σ (m)")
        ax.set_xlabel("Frame index"); ax.set_ylabel("σ (m)")
        ax.set_title("KISS-ICP adaptive threshold σ")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.35)

        plt.tight_layout()
        self._save(fig, os.path.join(self.results_dir, "icp_metrics.png"))

    def _plot_motion_and_deviation(self):
        m = self._metrics()
        if not m:
            return
        frames    = self._col(m, "frame_idx")
        trans     = self._col(m, "motion_trans_m")
        rot       = self._col(m, "motion_rot_deg")
        dev_trans = self._col(m, "model_dev_trans_m")
        dev_rot   = self._col(m, "model_dev_rot_deg")

        fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True)
        fig.suptitle("Motion & Model-Deviation Diagnostics – KISS-SLAM",
                     fontsize=13, fontweight="bold")

        for ax, data, ylabel, title, color in zip(
            axes,
            [trans, rot, dev_trans, dev_rot],
            ["m", "°", "m", "°"],
            ["Frame-to-frame translation (m)", "Frame-to-frame rotation (°)",
             "Model-deviation: translational correction by ICP (m)",
             "Model-deviation: rotational correction by ICP (°)"],
            [_C["trans"], _C["rot"], _C["dev_trans"], _C["dev_rot"]],
        ):
            ax.plot(frames, data, color=color, lw=0.9)
            ax.set_ylabel(ylabel); ax.set_title(title); ax.grid(True, alpha=0.35)

        axes[-1].set_xlabel("Frame index")
        plt.tight_layout()
        self._save(fig, os.path.join(self.results_dir, "motion_metrics.png"))

    def _plot_geometric_degeneracy(self):
        m = self._metrics()
        if not m:
            return
        frames     = self._col(m, "frame_idx")
        linearity  = self._col(m, "geo_linearity")
        planarity  = self._col(m, "geo_planarity")
        sphericity = self._col(m, "geo_sphericity")
        condition  = self._col(m, "geo_condition")
        omnivar    = self._col(m, "geo_omnivariance")

        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle("Geometric Degeneracy – KISS-SLAM  (staircase detector)",
                     fontsize=13, fontweight="bold")

        ax = axes[0]
        ax.plot(frames, planarity,  color=_C["planarity"],  lw=0.9, label="Planarity  (↑ = staircase)")
        ax.plot(frames, linearity,  color=_C["linearity"],  lw=0.9, label="Linearity  (↑ = corridor)")
        ax.plot(frames, sphericity, color=_C["sphericity"], lw=0.9, label="Sphericity (↑ = ideal ICP)")
        ax.axhline(0.5, color="grey", linestyle=":", lw=0.8)
        ax.set_ylim(-0.05, 1.05); ax.set_ylabel("Value [0–1]")
        ax.set_title("Point-cloud shape descriptors")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.35)

        ax = axes[1]
        cond_arr = np.array(condition, dtype=float)
        clip_val = np.nanpercentile(cond_arr, 98)
        ax.plot(frames, np.clip(cond_arr, 0, clip_val), color=_C["condition"], lw=0.9,
                label=f"Condition (clipped at 98th pct = {clip_val:.0f})")
        ax.set_ylabel("λ_max/λ_min"); ax.set_title("Covariance condition number")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.35)

        ax = axes[2]
        ax.plot(frames, omnivar, color="#1abc9c", lw=0.9)
        ax.set_xlabel("Frame index"); ax.set_ylabel("m²/³")
        ax.set_title("Omnivariance (overall 3-D spread)")
        ax.grid(True, alpha=0.35)

        plt.tight_layout()
        self._save(fig, os.path.join(self.results_dir, "geometry_metrics.png"))

    # ─────────────────────────────────────────────────────────────────────────
    # Existing pipeline methods — unchanged
    # ─────────────────────────────────────────────────────────────────────────

    def _global_mapping(self):
        if self.refuse_scans:
            from kiss_icp.preprocess import get_preprocessor
            if hasattr(self._dataset, "reset"):
                self._dataset.reset()
            ref_ground_alignment = self.kiss_slam.closer.detector.get_ground_alignment_from_id(0)
            deskewing_deltas = np.vstack((
                np.eye(4)[None],
                np.eye(4)[None],
                np.linalg.inv(self.poses[:-2]) @ self.poses[1:-1],
            ))
            preprocessor = get_preprocessor(self.config)
            occupancy_grid_mapper = OccupancyGridMapper(self.slam_config.occupancy_mapper)
            print("KissSLAM| Computing Occupancy Grid")
            for idx in trange(self._first, self._last, unit=" frames", dynamic_ncols=True):
                frame, timestamps, _, _ = self._next(idx)
                deskewed_scan = preprocessor.preprocess(frame, timestamps, deskewing_deltas[idx])
                occupancy_grid_mapper.integrate_frame(
                    deskewed_scan, ref_ground_alignment @ self.poses[idx - self._first]
                )
            occupancy_grid_mapper.compute_3d_occupancy_information()
            occupancy_grid_mapper.compute_2d_occupancy_information()
            occupancy_dir = os.path.join(self.results_dir, "occupancy_grid")
            os.makedirs(occupancy_dir, exist_ok=True)
            occupancy_grid_mapper.write_3d_occupancy_grid(occupancy_dir)
            occupancy_2d_map_dir = os.path.join(occupancy_dir, "map2d")
            os.makedirs(occupancy_2d_map_dir, exist_ok=True)
            occupancy_grid_mapper.write_2d_occupancy_grid(occupancy_2d_map_dir)

    def _write_pose_times(self):
        """pose_times.csv and *_poses_posetime_tum.txt: every pose at the instant of the sweep it stands for.

        The *_poses_tum.txt of upstream stamps each pose with its scan's stamp, which on the Hesai / Ouster bags and
        the Newer College 2020 pcds is the FIRST point of the sweep (measured, 25/9), while a deskewed scan is
        expressed at its LAST point (KissSLAM.pose_time_fractions).  Here pose time = stamp + fraction x the scan's
        own point-time span, the convention of the ground truth of Oxford Spires and Newer College (a pose at the
        instant of its stamp), so the official evaluation (evo_ape, no time offset) compares like with like.  The
        span comes in the reader's unit (Hesai s, Ouster bag ns, 2020 pcd s): converted by the power of ten that
        brings it closest to the gap between stamps.
        """
        fractions = np.asarray(self.kiss_slam.pose_time_fractions, dtype=np.float64)
        spans = np.asarray(self.kiss_slam.sweep_spans, dtype=np.float64)
        stamps = np.asarray(self._get_frames_timestamps(), dtype=np.float64)[: len(self.poses)]
        n = len(self.poses)
        if len(fractions) != n or len(spans) != n or len(stamps) != n or n < 2:
            print("KissSLAM| pose times not written: one fraction, span and stamp per pose needed")
            return
        unit = 10.0 ** np.round(np.log10(np.median(np.diff(stamps)) / np.nanmedian(spans)))
        spans = np.where(np.isfinite(spans), spans * unit, np.nanmedian(spans) * unit)
        pose_time = stamps + fractions * spans
        with open(os.path.join(self.results_dir, "pose_times.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["scan", "stamp", "fraction", "span_s", "pose_time"])
            for k, (t, a, d, p) in enumerate(zip(stamps, fractions, spans, pose_time)):
                w.writerow([k, f"{t:.6f}", f"{a:.4f}", f"{d:.6f}", f"{p:.6f}"])
        poses = self._calibrate_poses(self.poses)
        rows = [np.r_[t, T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_quat()] for t, T in zip(pose_time, poses)]
        np.savetxt(f"{self.results_dir}/{self.dataset_sequence}_poses_posetime_tum.txt", np.array(rows), fmt="%.6f")

    def _write_two_start_log(self):
        """two_start.csv: every scan registered from several starting points, their fits and the one kept (#058)."""
        log = self.kiss_slam.two_start_log
        if not log:
            return
        keys = sorted({k for row in log for k in row}, key=lambda k: (k != "scan", k))
        with open(os.path.join(self.results_dir, "two_start.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(log)

    def _write_image_motions(self):
        """The image motion of every scan as the estimator gave it, with its inliers (#086): image_motions.npz,
        motion (N,4,4) NaN where it failed, inliers (N,), scan (N,).  Recording only."""
        log = getattr(self.kiss_slam, "image_motion_log", [])
        if not log:
            return
        motion = np.stack([np.full((4, 4), np.nan) if np.isscalar(m) else m for _, m, _ in log])
        np.savez(os.path.join(self.results_dir, "image_motions.npz"), motion=motion,
                 inliers=np.array([n for *_, n in log]), scan=np.array([k for k, *_ in log]))

    def _write_deskewed_frames(self):
        """diagnostics.save_deskewed_voxel: every scan as deskewed (sensor frame, voxel-downsampled), with
        offsets, for the map-sharpness test (#048).  Scan k is frames[offsets[k]:offsets[k+1]], at poses[k]."""
        frames = self.kiss_slam.deskewed_frames
        if not frames:
            return
        np.savez(
            os.path.join(self.results_dir, "deskewed_frames.npz"),
            points=np.concatenate(frames),
            offsets=np.concatenate([[0], np.cumsum([len(f) for f in frames])]),
            voxel=self.slam_config.diagnostics.save_deskewed_voxel,
        )

    def _write_local_maps(self):
        local_maps_dir = os.path.join(self.results_dir, "local_maps")
        os.makedirs(local_maps_dir, exist_ok=True)
        self.kiss_slam.optimizer.write_graph(os.path.join(local_maps_dir, "local_map_graph.g2o"))
        plys_dir = os.path.join(local_maps_dir, "plys")
        os.makedirs(plys_dir, exist_ok=True)
        print("KissSLAM| Writing Local Maps on Disk")
        for local_map in tqdm(self.kiss_slam.local_map_graph.local_maps()):
            filename = os.path.join(plys_dir, "{:06d}.ply".format(local_map.id))
            local_map.write(filename)

    def _evaluate_closures(self):
        self.results.append(
            desc="Number of closures found", units="closures",
            value=len(self.kiss_slam.closures)
        )

    def _write_closures(self):
        locations = [pose[:3, -1] for pose in self.poses]
        loc_x = [loc[0] for loc in locations]
        loc_y = [loc[1] for loc in locations]
        plt.scatter(loc_x, loc_y, s=0.1, color="black")
        key_poses = self.kiss_slam.get_keyposes()
        for closure in self.kiss_slam.closures:
            i, j = closure
            plt.plot(
                [key_poses[i][0, -1], key_poses[j][0, -1]],
                [key_poses[i][1, -1], key_poses[j][1, -1]],
                color="red", linewidth=1, markersize=1,
            )
        plt.savefig(os.path.join(self.results_dir, "trajectory.png"), dpi=2000)
        plt.close()

    def _write_graph(self):
        self.pose_graph.write_graph(os.path.join(self.results_dir, "trajectory.g2o"))
