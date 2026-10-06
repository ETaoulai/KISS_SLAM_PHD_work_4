# MIT License

# Copyright (c) 2025 Tiziano Guadagnino, Benedikt Mersch, Saurabh Gupta, Cyrill
# Stachniss.

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from collections import deque

import os

import numpy as np
import open3d as o3d
from kiss_icp.kiss_icp import KissICP
from kiss_icp.preprocess import Preprocessor
from kiss_icp.voxelization import voxel_down_sample
from scipy.spatial import KDTree

from kiss_slam.config import KissSLAMConfig
from kiss_slam.local_map_graph import LocalMapGraph
from kiss_slam.loop_closer import LoopCloser
from kiss_slam.pose_graph_optimizer import PoseGraphOptimizer
from kiss_slam.voxel_map import VoxelMap


# ─────────────────────────────────────────────────────────────────────────────
# Image-motion worker process (image_deskew.parallel)
# ─────────────────────────────────────────────────────────────────────────────

_WORKER_ESTIMATOR = None


def _motion_worker_init(estimator_kwargs, module_knobs, role="motion"):
    """Runs once in the worker: the module-level knobs of the parent, then the estimator."""
    global _WORKER_ESTIMATOR
    import kiss_slam.intensity_deskew as idsk

    for name, value in module_knobs.items():
        setattr(idsk, name, value)
    _WORKER_ESTIMATOR = idsk.ScanMotionEstimator(**estimator_kwargs)
    import os
    out = os.environ.get("KISS_PROFILE_WORKER")              # #179, diagnostics only: cProfile of the image worker, dumped at its exit
    if out and role != "motion":                              # #180: the features stage writes its own file
        out = out.replace(".prof", "") + f"_{role}.prof"
    if out:
        import cProfile
        import time
        from multiprocessing import util
        global _WORKER_PROFILE
        _WORKER_PROFILE = (cProfile.Profile(), time.perf_counter(), [0.0])

        def _dump():
            prof, t0, busy = _WORKER_PROFILE
            prof.dump_stats(out)
            with open(out + ".txt", "w") as f:
                f.write(f"worker alive {time.perf_counter() - t0:.1f} s, busy in motion() {busy[0]:.1f} s\n")
        util.Finalize(None, _dump, exitpriority=100)


_WORKER_PROFILE = None


def _profiled(fn, *args):
    if _WORKER_PROFILE is None:
        return fn(*args)
    import time
    prof, _, busy = _WORKER_PROFILE
    t = time.perf_counter()
    prof.enable()
    try:
        return fn(*args)
    finally:
        prof.disable()
        busy[0] += time.perf_counter() - t


def _motion_worker(frame, timestamps, intensity, ring, packed=None):
    """Matching + estimation (+ range fallback) of one scan; `packed` = its features from the features stage (#180), else computed here."""
    import kiss_slam.intensity_deskew as idsk

    feats = None if packed is None else idsk.unpack_features(packed)
    return _profiled(lambda: _WORKER_ESTIMATOR.motion(frame, timestamps, intensity, ring, feats=feats)[0])


def _feature_worker(frame, timestamps, intensity, ring):
    """#180: panorama + keypoints / descriptors of one scan - depends only on the scan, so it runs ahead of the matching."""
    import kiss_slam.intensity_deskew as idsk

    return _profiled(lambda: idsk.pack_features(_WORKER_ESTIMATOR.scan_features(frame, timestamps, intensity, ring)))


def transform_points(pcd, T):
    R = T[:3, :3]
    t = T[:3, -1]
    return pcd @ R.T + t


# ─────────────────────────────────────────────────────────────────────────────
# Intensity helpers
# ─────────────────────────────────────────────────────────────────────────────

def _interpolate_intensity(raw_frame: np.ndarray,
                           raw_intensity: np.ndarray,
                           downsampled: np.ndarray) -> np.ndarray:
    """Assign each downsampled point the intensity of its nearest raw neighbour.

    Parameters
    ----------
    raw_frame   : (N, 3) original (deskewed) XYZ frame
    raw_intensity : (N,) normalised [0,1] intensity for every raw point
    downsampled : (M, 3) voxelised subset of the frame

    Returns
    -------
    (M,) float32 intensity values for the downsampled points
    """
    if raw_intensity is None or len(raw_intensity) == 0:
        return None
    tree = KDTree(raw_frame)
    _, idx = tree.query(downsampled, k=1, workers=-1)
    return raw_intensity[idx].astype(np.float32)


def _align_intensity_to_preprocessed(deskewed_all: np.ndarray,
                                     raw_intensity: np.ndarray,
                                     preprocessed: np.ndarray,
                                     max_range: float,
                                     min_range: float):
    """Drop the intensities of the points the KISS preprocessor dropped.

    `raw_intensity[i]` belongs to raw point i.  The KISS preprocessor first
    deskews every point, then removes those whose *deskewed* range falls outside
    [min_range, max_range], keeping the rest **in order**.  Indexing
    `raw_intensity` with positions in `preprocessed` (what the code did before)
    shifts every intensity after the first dropped point onto a different point.
    Measured on church_02: only 7.8-14.1 % of source points had their own
    intensity in outdoor scans with any point beyond 50 m.

    `deskewed_all` must be the raw frame put through the *same* deskew but with
    no range limit, so that deskewed_all[i] is raw point i.  Masking on the raw
    range instead is not exact: deskewing moves points across the limit
    (6 points in scan 450).

    Returns (aligned_intensity, used_fallback).  The fallback (nearest point of
    deskewed_all, which contains every surviving point exactly) only exists as a
    safety net in case the preprocessor's range test ever differs from ours.
    """
    if raw_intensity is None or len(raw_intensity) == 0:
        return raw_intensity, False
    rng = np.linalg.norm(deskewed_all, axis=1)
    # Ίδιες αυστηρές ανισότητες με το KISS (Preprocessing.cpp: range < max && range > min).
    keep = (rng < max_range) & (rng > min_range)
    if int(keep.sum()) == len(preprocessed):
        return raw_intensity[keep], False
    _, idx = KDTree(deskewed_all).query(preprocessed, k=1, workers=-1)
    return raw_intensity[idx], True


def _intensity_filter(points: np.ndarray,
                      intensity: np.ndarray,
                      keep_ratio: float = 0.70,
                      min_intensity: float = 0.05,
                      rng: np.random.Generator = None) -> np.ndarray:
    """Return a boolean mask that keeps the most-informative points.

    Strategy
    --------
    On geometrically degenerate surfaces (staircase treads, flat corridors)
    most LiDAR returns have *similar, low* intensity – they all came from the
    same homogeneous material and provide near-duplicate constraints for ICP.

    We keep:
      1. All points above `min_intensity` (bright returns = edges, paint,
         retro-reflectors, railings) – these are almost always geometrically
         distinctive.
      2. Enough of the remaining points to reach `keep_ratio` of the total,
         sampled randomly so we don't bias toward any region.

    The net effect is that flat-plane dominated frames (high geo_planarity)
    see fewer redundant co-planar points and more edge/corner returns, which
    breaks the degeneracy that causes Z-axis and roll/pitch drift.

    Parameters
    ----------
    points      : (N, 3) source cloud
    intensity   : (N,) normalised [0,1] intensities
    keep_ratio  : overall fraction of points to retain  (default 0.70)
    min_intensity : points above this value are always kept (default 0.05)
    rng         : seeded Generator for the random fill-in, so that repeated
                  runs of the same arm produce identical trajectories

    Returns
    -------
    Boolean mask of shape (N,) selecting the retained points
    """
    if intensity is None or len(intensity) == 0:
        return np.ones(len(points), dtype=bool)

    n = len(points)
    n_keep = max(int(n * keep_ratio), min(50, n))

    bright_mask = intensity >= min_intensity
    n_bright = int(bright_mask.sum())

    if n_bright >= n_keep:
        # More bright points than budget: keep the top-n_keep by intensity
        idx_sorted = np.argsort(intensity)[::-1][:n_keep]
        mask = np.zeros(n, dtype=bool)
        mask[idx_sorted] = True
        return mask

    # Keep all bright points, fill the rest randomly from dim ones
    mask = bright_mask.copy()
    still_needed = n_keep - n_bright
    dim_indices = np.where(~bright_mask)[0]
    if len(dim_indices) > 0:
        rng = rng if rng is not None else np.random.default_rng(0)
        chosen = rng.choice(
            dim_indices,
            size=min(still_needed, len(dim_indices)),
            replace=False,
        )
        mask[chosen] = True
    return mask


def _make_intensity_pcd(points: np.ndarray,
                        intensity: np.ndarray) -> o3d.t.geometry.PointCloud:
    """Build an Open3D tensor PointCloud with intensity stored as colours.

    Intensity is replicated to R=G=B so that Open3D's ColoredICP can use it.
    Normals are estimated from the geometry so that PointToPlane also works.

    Parameters
    ----------
    points    : (N, 3) float32/64 XYZ
    intensity : (N,)  float32 normalised [0,1], or None

    Returns
    -------
    o3d.t.geometry.PointCloud  (positions + normals + colors if intensity given)
    """
    pts32 = points.astype(np.float32)
    pcd = o3d.t.geometry.PointCloud(o3d.core.Tensor(pts32))

    # Estimate normals (required for ColoredICP and PointToPlane)
    pcd_legacy = pcd.to_legacy()
    pcd_legacy.estimate_normals(
        o3d.geometry.KDTreeSearchParamKNN(knn=20)
    )
    pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcd_legacy)

    if intensity is not None and len(intensity) == len(points):
        intens_f32 = intensity.astype(np.float32).reshape(-1, 1)
        intens_rgb = np.repeat(intens_f32, 3, axis=1)   # grayscale trick
        pcd_t.point.colors = o3d.core.Tensor(
            intens_rgb, dtype=o3d.core.Dtype.Float32
        )

    return pcd_t


# ─────────────────────────────────────────────────────────────────────────────
# Diagnostic metric helpers  (unchanged from previous version)
# ─────────────────────────────────────────────────────────────────────────────

def _compute_icp_metrics(source_points, map_tree, pose, max_dist):
    """Point-to-map residuals of the registered source.

    `map_tree` is a prebuilt KDTree over the local map (see
    KissSLAM._refresh_diag_tree), or None to skip — building it here every frame
    was the dominant per-frame cost.
    """
    nan = float("nan")
    empty = {
        "icp_rms_error": nan, "icp_mean_error": nan, "icp_max_error": nan,
        "icp_inlier_count": 0, "icp_inlier_ratio": nan,
    }
    if map_tree is None or len(source_points) == 0:
        return empty
    src_in_map = transform_points(source_points, pose)
    dists, _ = map_tree.query(src_in_map, k=1, workers=-1)
    inlier_mask = dists < max_dist
    inlier_count = int(inlier_mask.sum())
    n_source = len(source_points)
    inlier_ratio = inlier_count / n_source if n_source > 0 else nan
    if inlier_count > 0:
        d = dists[inlier_mask]
        rms  = float(np.sqrt(np.mean(d ** 2)))
        mean = float(np.mean(d))
        mx   = float(np.max(d))
    else:
        rms = mean = mx = nan
    return {
        "icp_rms_error":    rms,
        "icp_mean_error":   mean,
        "icp_max_error":    mx,
        "icp_inlier_count": inlier_count,
        "icp_inlier_ratio": inlier_ratio,
    }


def _compute_motion_metrics(prev_pose, curr_pose):
    delta = np.linalg.inv(prev_pose) @ curr_pose
    trans = float(np.linalg.norm(delta[:3, 3]))
    cos_angle = np.clip((np.trace(delta[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    rot_deg = float(np.degrees(np.arccos(cos_angle)))
    return {"motion_trans_m": trans, "motion_rot_deg": rot_deg}


def _compute_model_deviation(initial_guess, converged_pose):
    correction = np.linalg.inv(initial_guess) @ converged_pose
    trans = float(np.linalg.norm(correction[:3, 3]))
    cos_angle = np.clip((np.trace(correction[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    rot_deg = float(np.degrees(np.arccos(cos_angle)))
    return {"model_dev_trans_m": trans, "model_dev_rot_deg": rot_deg}


def _compute_geometric_degeneracy(source_points):
    nan = float("nan")
    bad = {k: nan for k in ("geo_linearity", "geo_planarity", "geo_sphericity",
                             "geo_condition", "geo_omnivariance")}
    if len(source_points) < 4:
        return bad
    try:
        cov = np.cov(source_points.T)
        ev  = np.sort(np.linalg.eigvalsh(cov))
        l1, l2, l3 = ev
        eps = 1e-9
        return {
            "geo_linearity":    float((l3 - l2) / (l3 + eps)),
            "geo_planarity":    float((l2 - l1) / (l3 + eps)),
            "geo_sphericity":   float(l1        / (l3 + eps)),
            "geo_condition":    float(l3        / (l1 + eps)),
            "geo_omnivariance": float((max(l1, 0) * max(l2, 0) * max(l3, 0)) ** (1.0 / 3.0)),
        }
    except np.linalg.LinAlgError:
        return bad


# ─────────────────────────────────────────────────────────────────────────────
# Main SLAM class
# ─────────────────────────────────────────────────────────────────────────────

class KissSLAM:
    def __init__(self, config: KissSLAMConfig):
        self.config = config
        self.odometry = KissICP(config.kiss_icp_config())

        # ── A/B experiment switch ────────────────────────────────────────────
        # config.intensity.enabled == False  →  vanilla upstream KISS-SLAM
        #                                       (the *baseline* arm)
        # config.intensity.enabled == True   →  intensity-aided arm
        self.deskew_passes = config.deskew_refine.passes if config.odometry.preprocessing.deskew else 1
        self.intensity_cfg = config.intensity
        self.use_intensity = config.intensity.enabled
        # Same range limits the KISS preprocessor applies, needed to keep each
        # intensity attached to its own point (see _align_intensity_to_preprocessed).
        self._max_range = config.odometry.preprocessing.max_range
        self._min_range = config.odometry.preprocessing.min_range
        self.n_intensity_fallbacks = 0
        # Same deskew as KISS but no range cut: row i stays raw point i.
        self._unbounded_preprocessor = Preprocessor(
            max_range=1e9,
            min_range=0.0,
            deskew=config.odometry.preprocessing.deskew,
            max_num_threads=config.odometry.registration.max_num_threads,
        )
        # Fixed seed: the intensity arm must be reproducible run-to-run,
        # otherwise an A/B difference cannot be attributed to the method.
        self._rng = np.random.default_rng(0)

        # ── Image-motion deskew (#018-#032) ─────────────────────────────────
        # The motion measured in the intensity image replaces last_delta for the
        # deskew and (use_as_initial_guess) for the ICP start; sigma stays fixed.
        self.image_cfg = config.image_deskew
        self.use_image_deskew = config.image_deskew.enabled
        if self.use_image_deskew and self.use_intensity:
            raise ValueError(
                "image_deskew.enabled and intensity.enabled are mutually exclusive: "
                "pick one arm (--image-deskew or --use-intensity)."
            )
        self._image_motion_est = None      # online estimator
        self._motion_pool = None           # image_deskew.parallel: worker process running the estimator
        self._feature_pool = None          # #180: second worker, the features of each scan ahead of the matching
        self._motion_futures = deque()     # motions submitted to it, oldest first
        self._rotvec_history = []          # image_deskew.rotation_smoothing (#047)
        self._validate_hist = []           # #109: image vs range rotation differences (running median)
        self.n_validate_forced = 0
        self.n_two_start = 0                # image_deskew.two_start_deg: scans registered twice (#057)
        self.n_two_start_cv_won = 0         # ... of which the constant-velocity start fitted better
        self.two_start_log = []             # one dict per such scan (#058): fits of every start, which was kept
        self.two_start_seconds = 0.0        # time spent on the extra registrations and the fit test
        # Instant of the sweep each pose stands for, as a fraction of the sweep (0 = first point, 1 = last).
        # A scan deskewed with a motion is expressed at its last point (kiss_icp 1.3.0: exp((s - 1) log delta));
        # a scan registered raw (no deskew, identity motion) is a blur, fitted near its mean point time.  The
        # pipeline turns this into the pose's time for the evaluation (pose_times.csv, *_posetime_tum.txt).
        self.pose_time_fractions = []
        self.sweep_spans = []               # last - first point time of each scan, in the reader's own unit
        self._kept_deskew_delta = None      # deskew motion of the result kept by _register_frame_image_motion
        self._image_motions = None         # precomputed (N,4,4), NaN where failed
        self.image_motion_log = []         # (scan, motion or NaN, inliers) per scan, written by the pipeline (#086)
        # Index in the sequence of the first scan fed to process_scan: the precomputed
        # file has one row per scan of the whole sequence.  SlamPipeline sets it to its
        # first scan (--jump); without it a run with jump > 0 read the wrong rows.
        self.first_scan_index = 0
        self.n_image_motion_failures = 0   # scans that fell back to identity
        if self.use_image_deskew:
            if not config.odometry.preprocessing.deskew:
                raise ValueError("image_deskew.enabled needs odometry.preprocessing.deskew: true")
            if self.image_cfg.motion_file is not None:
                self._image_motions = np.load(self.image_cfg.motion_file)["motion"]
                print(f"KissSLAM| image motion from {self.image_cfg.motion_file} "
                      f"({len(self._image_motions)} scans)")
            else:
                import kiss_slam.intensity_deskew as _idsk
                from kiss_slam.intensity_deskew import ScanMotionEstimator

                if self.image_cfg.trans_min_range is not None:      # near-field bias correction (#039)
                    _idsk.TRANS_MIN_RANGE = self.image_cfg.trans_min_range
                    _idsk.TRANS_MODE = self.image_cfg.trans_mode
                estimator_kwargs = dict(
                    seed=self.image_cfg.seed,
                    intensity_scale=self.image_cfg.intensity_scale,
                    gate_min_matches=self.image_cfg.gate_min_matches,
                    gate_max_rotation_deg=self.image_cfg.gate_max_rotation_deg,
                    gate_max_rotation_change_deg=self.image_cfg.gate_max_rotation_change_deg,
                    save_rejected_dir=self.image_cfg.save_rejected_dir,
                    range_motion=self.image_cfg.range_motion,
                    range_hessian=self.image_cfg.range_hessian,
                    intensity_normalisation=self.image_cfg.intensity_normalisation,
                    panorama_width=self.image_cfg.panorama_width,
                    panorama_up=self.image_cfg.panorama_up,                      # #089
                    fit_sectors=self.image_cfg.fit_sectors,                      # #093
                    sectors_part=self.image_cfg.fit_sectors_part,                # #135
                    whiten=self.image_cfg.whiten,
                    cross_check=self.image_cfg.cross_check,
                    detect_scale=self.image_cfg.detect_scale,
                    multi_baseline=self.image_cfg.multi_baseline,                # #130
                    fuse_range=self.image_cfg.fuse_range,                        # #090
                    drop_stationary=self.image_cfg.drop_stationary,              # #132
                    bearing_min_range=self.image_cfg.rotation_from_bearings,      # #087
                    guided_window=self.image_cfg.guided_matching_window,         # #087
                    guided_prediction=self.image_cfg.guided_prediction,          # #089
                    model=self.image_cfg.model,
                    subpixel=self.image_cfg.subpixel,
                    stuck_min=self.image_cfg.stuck_min,
                    floor_only=self.image_cfg.stuck_floor_only,
                    elev=self.image_cfg.stuck_elev_deg,
                    range_=self.image_cfg.stuck_range_m,
                    detector=self.image_cfg.detector,
                    surf_hessian=self.image_cfg.surf_hessian_threshold,
                    surf_upright=self.image_cfg.surf_upright,
                )
                if self.image_cfg.parallel and self.image_cfg.range_motion in ("candidate", "validate"):
                    raise ValueError("image_deskew.range_motion = 'candidate' needs parallel = false (#058)")
                if self.image_cfg.parallel:
                    # "spawn": a fork would copy the parent's TBB / OpenCV thread state.  The worker
                    # gets the module knobs as they are NOW (scripts set them before building KissSLAM).
                    import multiprocessing
                    from concurrent.futures import ProcessPoolExecutor

                    knobs = {k: v for k, v in vars(_idsk).items() if k.isupper()}
                    self._motion_pool = ProcessPoolExecutor(
                        max_workers=1,
                        mp_context=multiprocessing.get_context("spawn"),
                        initializer=_motion_worker_init,
                        initargs=(estimator_kwargs, knobs),
                    )
                    # #180: two stages - the features of each scan (panorama + detector, which depend only on the scan) in a second
                    # worker, ahead of the matching; the same computations in the same order, so the same motions.  Not with the
                    # "auto" panorama (its size is measured from the first scan inside the matching worker) or KISS_IMAGE_STAGES=1.
                    if (self.image_cfg.panorama_width != "auto" and self.image_cfg.panorama_up != "auto"
                            and os.environ.get("KISS_IMAGE_STAGES", "2") != "1"):
                        import queue
                        import threading

                        self._feature_pool = ProcessPoolExecutor(
                            max_workers=1,
                            mp_context=multiprocessing.get_context("spawn"),
                            initializer=_motion_worker_init,
                            initargs=(estimator_kwargs, knobs, "features"),
                        )
                        self._forward_q = queue.Queue()
                        self._forwarder = threading.Thread(target=self._forward_features, daemon=True)
                        self._forwarder.start()
                else:
                    self._image_motion_est = ScanMotionEstimator(**estimator_kwargs)

        # Diagnostics-only KDTree cache (never affects the trajectory).
        self.diag_cfg = config.diagnostics
        self.deskewed_frames = []           # diagnostics.save_deskewed_voxel (#048)
        self._save_rng = np.random.default_rng(0)   # diagnostics.save_deskewed_fraction (#049)
        self._diag_tree = None
        self._diag_map_n = 0
        self._diag_age = 0

        self.closer = LoopCloser(
            config.loop_closer,
            lambda_geometric=config.intensity.lambda_geometric,
        )
        local_map_config = self.config.local_mapper
        self.local_map_voxel_size = local_map_config.voxel_size
        self.voxel_grid = VoxelMap(self.local_map_voxel_size)
        self.local_map_graph = LocalMapGraph()
        self.local_map_splitting_distance = local_map_config.splitting_distance
        self.local_map_splitting_height = local_map_config.splitting_height
        # Vertical direction in the current node frame, for splitting_height.  The sensor
        # is held tilted (5-19 deg on church_02), so the node-frame z is not the height:
        # over 15 m of level floor it changes 1-3 m (#014).  Taken from the ground plane
        # MapClosures estimates for the previous local map; unknown for the first node.
        self._node_up = None
        self.node_up_log = []   # (node id, up vector in its frame)
        self.optimizer = PoseGraphOptimizer(config.pose_graph_optimizer)
        self.optimizer.add_variable(self.local_map_graph.last_id, self.local_map_graph.last_keypose)
        self.optimizer.fix_variable(self.local_map_graph.last_id)
        self.closures = []

        # Intensity storage for the current local map's voxel grid.
        # Maps voxel-grid point index → mean intensity (accumulated online).
        # When finalize_local_map() is called we write intensities into the
        # Open3D pcd as colours so that loop-closure ColoredICP can use them.
        self._intensity_accumulator = {}  # voxel_key → (sum, count)
        # True only when the experiment is enabled AND the data actually carries
        # intensity.  Everything downstream keys off this single flag.
        self._has_intensity = False

        # Diagnostics log
        self.icp_metrics_log = []
        self._frame_counter = 0
        self._blend_pending = None          # #131: (scan, image motion as measured, constant-velocity prediction) of the last blended scan
        self._blend_errors = []             # #131: (image, constant-velocity) rotation errors against the ICP, deg
        self.blend_weights = []
        self._prev_pose = np.eye(4)

    def get_closures(self):
        return self.closures

    def get_keyposes(self):
        return list(self.local_map_graph.keyposes())

    def process_scan(self, frame: np.ndarray, timestamps: np.ndarray,
                     intensity: np.ndarray = None, ring: np.ndarray = None):
        """Process one LiDAR scan.

        Parameters
        ----------
        frame      : (N, 3) XYZ points (float64)
        timestamps : (N,)   per-point timestamps
        intensity  : (N,)   intensity, or None.  Intensity arm: normalised [0,1].
                            Image deskew (online): the sensor's raw scale (0-255).
        ring       : (N,)   laser/ring index per point, or None.  Needed only by the
                            online image deskew (motion_file = None).
        """
        use_int = self.use_intensity and intensity is not None
        mode = self.intensity_cfg.mode

        # ── 1. Capture pre-registration state ────────────────────────────────
        # Refreshed *before* registration so the residual never includes the
        # current frame's own points.
        if self.diag_cfg.icp_metrics:
            self._refresh_diag_tree()
        sigma = self.odometry.adaptive_threshold.get_threshold()
        # Used by the "refine" branch below exactly as in the original method.
        initial_guess = np.copy(self.odometry.last_pose)
        # KISS's real initial guess (constant velocity); diagnostics measure the
        # ICP correction against this.
        prediction = self.odometry.last_pose @ self.odometry.last_delta
        # register_frame deskews with last_delta and then overwrites it.
        deskew_delta = np.copy(self.odometry.last_delta)

        # ── 2. Registration ──────────────────────────────────────────────────
        if use_int and mode == "replace":
            (deskewed_frame, source, keep_mask,
             intensity, source_intensity) = self._register_frame_selected(
                frame, timestamps, intensity, deskew_delta
            )
            current_pose = self.odometry.last_pose
            source_filtered = source[keep_mask]
            self._has_intensity = True
            self._record_pose_time(frame, timestamps, deskew_delta if self.odometry.config.data.deskew else None)
        else:
            if self.use_image_deskew:
                deskewed_frame, source = self._register_frame_image_motion(
                    frame, timestamps, intensity, ring
                )
            elif self.deskew_passes > 1 and not use_int:
                deskewed_frame, source = self._register_frame_deskew_refined(frame, timestamps)
            else:
                deskewed_frame, source = self.odometry.register_frame(frame, timestamps)
            current_pose = self.odometry.last_pose
            source_filtered = source
            if self.use_image_deskew:
                used_delta = self._kept_deskew_delta
            else:
                used_delta = deskew_delta if self.odometry.config.data.deskew else None
            self._record_pose_time(frame, timestamps, used_delta)
            if use_int:
                intensity, source_intensity, keep_mask = self._select_by_intensity(
                    frame, timestamps, intensity, deskew_delta, deskewed_frame, source
                )
                self._has_intensity = True
                source_filtered = source[keep_mask]

                # ── "refine": second ICP on the selected points ──────────────
                # Warm-started from the first result, against a map that already
                # contains this scan (kept as in the original method).
                if len(source_filtered) < len(source):
                    refined_pose = self.odometry.registration.align_points_to_map(
                        points=source_filtered,
                        voxel_map=self.odometry.local_map,
                        initial_guess=current_pose,
                        max_correspondance_distance=3.0 * sigma,
                        kernel=sigma,
                    )
                    correction_trans = float(np.linalg.norm((refined_pose - current_pose)[:3, 3]))
                    if correction_trans < sigma:
                        current_pose = refined_pose
                        self.odometry.last_pose = refined_pose
                        model_deviation = np.linalg.inv(initial_guess) @ refined_pose
                        self.odometry.adaptive_threshold.update_model_deviation(model_deviation)

        # ── 3. Diagnostics ───────────────────────────────────────────────────
        m = {"frame_idx": self._frame_counter}
        m.update(_compute_icp_metrics(
            source_points=source_filtered,
            map_tree=self._diag_tree if self.diag_cfg.icp_metrics else None,
            pose=current_pose,
            max_dist=3.0 * sigma,
        ))
        m["n_source_pts"]    = len(source)
        m["n_filtered_pts"]  = len(source_filtered)
        # Size of the map the residual was measured against (may be cached).
        m["n_map_pts"]       = self._diag_map_n if self.diag_cfg.icp_metrics else 0
        m["adaptive_sigma"]  = float(sigma)
        m["has_intensity"]   = self._has_intensity
        m.update(_compute_motion_metrics(self._prev_pose, current_pose))
        m.update(_compute_model_deviation(prediction, current_pose))
        m.update(_compute_geometric_degeneracy(source_filtered))
        self.icp_metrics_log.append(m)
        self._frame_counter += 1
        self._prev_pose = np.copy(current_pose)

        # ── 4. Map integration ───────────────────────────────────────────────
        if self.diag_cfg.save_deskewed_voxel is not None:     # map-sharpness test (#048), sensor frame
            frame_v = voxel_down_sample(deskewed_frame, self.diag_cfg.save_deskewed_voxel).astype(np.float32)
            if self.diag_cfg.save_deskewed_fraction < 1.0:             # own RNG: the trajectory is unaffected
                frame_v = frame_v[self._save_rng.random(len(frame_v)) < self.diag_cfg.save_deskewed_fraction]
            self.deskewed_frames.append(frame_v)
        # #176: the same scan down-sampled at the same voxel size by the ICP (car: ICP voxel 1.0 m -> 0.5 = local map voxel) is reused
        # instead of computed again - the same points; otherwise (handheld: different sizes) computed as before.
        reuse, self._reuse_downsample = getattr(self, "_reuse_downsample", None), None
        if reuse is not None and reuse[0] is deskewed_frame and reuse[2] == self.local_map_voxel_size:
            mapping_frame = reuse[1]
        else:
            mapping_frame = voxel_down_sample(deskewed_frame, self.local_map_voxel_size)
        self.voxel_grid.integrate_frame(mapping_frame, current_pose)

        # Accumulate intensity for the mapping voxels (used later for ColoredICP)
        if use_int:
            self._accumulate_intensity(mapping_frame, current_pose, intensity, deskewed_frame)

        self.local_map_graph.last_local_map.local_trajectory.append(current_pose)
        traveled_distance = np.linalg.norm(current_pose[:3, -1])
        if traveled_distance > self.local_map_splitting_distance or (
            self.local_map_splitting_height is not None
            and self._node_up is not None
            and abs(self._node_up @ current_pose[:3, -1]) > self.local_map_splitting_height
        ):
            self.generate_new_node()

    def _select_by_intensity(self, frame, timestamps, intensity, deskew_delta,
                             deskewed_frame, source):
        """Intensity for every ICP source point, and which of them to keep.

        Returns (intensity aligned to deskewed_frame, per-source intensity,
        boolean keep mask over `source`).
        """
        # The preprocessor dropped out-of-range points; drop their intensities
        # too so that intensity[i] still belongs to deskewed_frame[i].
        deskewed_all = self._unbounded_preprocessor.preprocess(frame, timestamps, deskew_delta)
        aligned, fell_back = _align_intensity_to_preprocessed(
            deskewed_all, intensity, deskewed_frame, self._max_range, self._min_range
        )
        self.n_intensity_fallbacks += int(fell_back)

        source_intensity = _interpolate_intensity(deskewed_frame, aligned, source)
        keep_mask = _intensity_filter(
            source,
            source_intensity,
            keep_ratio=self.intensity_cfg.keep_ratio,
            min_intensity=self.intensity_cfg.min_intensity,
            rng=self._rng,
        )
        if keep_mask.sum() < 20:          # safety guard: never starve the ICP
            keep_mask = np.ones(len(source), dtype=bool)
        return aligned, source_intensity, keep_mask

    def _register_frame_selected(self, frame, timestamps, intensity, deskew_delta):
        """KissICP.register_frame with ONE change: the ICP sees only the
        intensity-selected source points.

        Line-for-line copy of kiss_icp 1.3.0 `KissICP.register_frame` otherwise:
        same deskew, voxelisation, adaptive threshold, constant-velocity guess,
        and the map is updated with the FULL downsampled frame.  With a selection
        that keeps every point it reproduces the baseline trajectory (verified in
        tests/test_replace_mode.py).
        """
        odo = self.odometry
        # Apply motion compensation
        deskewed = odo.preprocessor.preprocess(frame, timestamps, odo.last_delta)
        # Voxelize
        source, frame_downsample = odo.voxelize(deskewed)
        # ---- the one change: select the points the ICP will use ----
        aligned, source_intensity, keep_mask = self._select_by_intensity(
            frame, timestamps, intensity, deskew_delta, deskewed, source
        )
        # Get adaptive_threshold
        sigma = odo.adaptive_threshold.get_threshold()
        # Compute initial_guess for ICP
        initial_guess = odo.last_pose @ odo.last_delta
        # Run ICP
        new_pose = odo.registration.align_points_to_map(
            points=source[keep_mask],
            voxel_map=odo.local_map,
            initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma,
            kernel=sigma,
        )
        # Compute the difference between the prediction and the actual estimate
        model_deviation = np.linalg.inv(initial_guess) @ new_pose
        # Update step: threshold, local map, delta, and the last pose
        odo.adaptive_threshold.update_model_deviation(model_deviation)
        odo.local_map.update(frame_downsample, new_pose)
        odo.last_delta = np.linalg.inv(odo.last_pose) @ new_pose
        odo.last_pose = new_pose
        return deskewed, source, keep_mask, aligned, source_intensity

    def _register_frame_deskew_refined(self, frame, timestamps):
        """KissICP.register_frame, with the scan deskewed again by its own motion.

        Pass 1 is kiss_icp 1.3.0 `register_frame` line for line (deskew with last_delta,
        the previous scan's motion).  Each further pass deskews the RAW scan with the
        motion this scan has just been estimated to make, inv(last_pose) @ new_pose (the
        quantity the GT-deskew run of #012 took from the ground truth), and runs ICP
        again from new_pose.  The adaptive threshold, the map and last_delta are updated
        once, with the final result, exactly as upstream does with its single result.
        """
        odo = self.odometry
        sigma = odo.adaptive_threshold.get_threshold()
        initial_guess = odo.last_pose @ odo.last_delta
        new_pose, delta = initial_guess, odo.last_delta
        for _ in range(self.deskew_passes):
            deskewed = odo.preprocessor.preprocess(frame, timestamps, delta)
            source, frame_downsample = odo.voxelize(deskewed)
            new_pose = odo.registration.align_points_to_map(
                points=source,
                voxel_map=odo.local_map,
                initial_guess=new_pose,
                max_correspondance_distance=3 * sigma,
                kernel=sigma,
            )
            delta = np.linalg.inv(odo.last_pose) @ new_pose
        odo.adaptive_threshold.update_model_deviation(np.linalg.inv(initial_guess) @ new_pose)
        odo.local_map.update(frame_downsample, new_pose)
        odo.last_delta = delta
        odo.last_pose = new_pose
        return deskewed, source

    def _image_motion(self, frame, timestamps, intensity, ring):
        """Motion of this scan over its sweep, from the intensity image (#018-#032).

        4x4 pose of the sensor at the end of the scan in its frame at the start — the
        quantity KISS calls `delta`.  Identity when the estimate failed (#012: no deskew
        beats the KISS guess).  Precomputed file: indexed by the scan's position in the
        sequence, first_scan_index + scan counter.  Online: the estimator needs the RAW scan with the
        sensor's intensity scale and the ring of every point.
        """
        M = None
        if self._image_motions is not None:
            i = self.first_scan_index + self._frame_counter
            if i >= len(self._image_motions):
                raise ValueError(
                    f"image_deskew.motion_file has {len(self._image_motions)} scans, but scan {i} "
                    "of the sequence was requested: the file does not cover this run."
                )
            if not np.isnan(self._image_motions[i, 0, 0]):
                M = self._image_motions[i]
        else:
            if intensity is None or ring is None:
                raise ValueError(
                    "image_deskew online mode needs per-point intensity and ring: install "
                    "kiss_slam.tools.point_cloud2.read_point_cloud_raw as the dataset reader "
                    "(SlamPipeline does this) or set image_deskew.motion_file."
                )
            if self._motion_pool is not None:
                if not self._motion_futures:        # the caller did not prefetch: submit now and wait
                    self.submit_image_motion(frame, timestamps, intensity, ring)
                M = self._motion_futures.popleft().result()
            else:
                M, n_inl = self._image_motion_est.motion(frame, timestamps, intensity, ring)
                # Diagnostic record (#086): the image motion of every scan as estimated, and its RANSAC inliers.
                self.image_motion_log.append((self._frame_counter, np.nan if M is None else np.asarray(M, float), n_inl))
        if M is None:                         # failed, or rejected by the plausibility gate (#054)
            self.n_image_motion_failures += 1
            return None
        return self._image_motion_parts(np.asarray(M, dtype=np.float64))

    def _image_motion_parts(self, M):
        """Ablation (#047): which part of the image motion is used, and rotation smoothing; weighted rotation (#092).

        rotation_smoothing = k > 1: the rotation is the mean rotation vector of this and the previous
        k-1 successful image motions (causal).  use_parts: "full" as measured; "translation" keeps the
        translation with no rotation; "rotation" keeps the rotation with no translation.  The result
        is used for BOTH the deskew and the ICP initial guess, which must agree (#030).
        """
        from scipy.spatial.transform import Rotation

        k = self.image_cfg.rotation_smoothing
        if k > 1:
            self._rotvec_history.append(Rotation.from_matrix(M[:3, :3]).as_rotvec())
            del self._rotvec_history[:-k]
            M = M.copy()
            M[:3, :3] = Rotation.from_rotvec(np.mean(self._rotvec_history, axis=0)).as_matrix()
        w = self.image_cfg.rotation_cv_weight
        if w > 0 and self._frame_counter >= 2:      # weighted image rotation (#092): a little of the constant velocity
            Rm = M[:3, :3]
            rv = Rotation.from_matrix(Rm.T @ self.odometry.last_delta[:3, :3]).as_rotvec()
            M = M.copy()
            M[:3, :3] = Rm @ Rotation.from_rotvec(w * rv).as_matrix()
        if self.image_cfg.cv_blend == "adaptive" and self._frame_counter >= 2:
            M = self._cv_blend(M)
        parts = self.image_cfg.use_parts
        if parts == "translation":
            M = M.copy()
            M[:3, :3] = np.eye(3)
        elif parts == "rotation":
            M = M.copy()
            M[:3, 3] = 0.0
        return M

    def _cv_blend(self, M):
        """#131: blend the image motion with the constant velocity, weights from their recent errors against the ICP (no threshold)."""
        from scipy.spatial.transform import Rotation, Slerp

        C = np.asarray(self.odometry.last_delta, dtype=np.float64)    # = the ICP motion of the previous scan
        if self._blend_pending is not None and self._blend_pending[0] == self._frame_counter - 1:
            _, Mp, Cp = self._blend_pending
            ang = lambda A: np.degrees(np.linalg.norm(Rotation.from_matrix(C[:3, :3].T @ A[:3, :3]).as_rotvec()))
            self._blend_errors.append((ang(Mp), ang(Cp)))
            del self._blend_errors[:-self.image_cfg.cv_blend_window]
        self._blend_pending = (self._frame_counter, M.copy(), C.copy())
        self._blend_raw = (self._frame_counter, M.copy())      # #137: the image motion as measured (cv_blend_use "deskew")
        if len(self._blend_errors) < 3:
            self.blend_weights.append(1.0)
            return M
        e = np.asarray(self._blend_errors)
        vi, vc = np.mean(e[:, 0] ** 2) + 1e-9, np.mean(e[:, 1] ** 2) + 1e-9
        w = vc / (vi + vc)
        self.blend_weights.append(w)
        B = np.eye(4)
        B[:3, :3] = Slerp([0, 1], Rotation.from_matrix(np.stack([C[:3, :3], M[:3, :3]])))(w).as_matrix()
        B[:3, 3] = (1 - w) * C[:3, 3] + w * M[:3, 3]
        if self.image_cfg.cv_blend_part == "rotation":            # #137
            B[:3, 3] = M[:3, 3]
        return B

    @property
    def image_motion_parallel(self):
        """True when the image motion runs in a worker process (image_deskew.parallel, online)."""
        return self._motion_pool is not None

    def submit_image_motion(self, frame, timestamps, intensity, ring):
        """Queue the image motion of a scan in the worker; process_scan of that scan collects it.

        Scans must be submitted in the order they are processed (one worker: they run in that
        order, so the estimator sees the same sequence as the serial one).
        """
        args = (frame, timestamps, intensity, ring)
        if self._feature_pool is None:
            self._motion_futures.append(self._motion_pool.submit(_motion_worker, *args))
            return
        from concurrent.futures import Future                    # #180: features now, matching when they are ready (in order)

        proxy = Future()
        self._forward_q.put((self._feature_pool.submit(_feature_worker, *args), args, proxy))
        self._motion_futures.append(proxy)

    def _forward_features(self):
        """#180: passes each scan's features to the matching worker in submission order (one thread, so the order is kept)."""
        while True:
            item = self._forward_q.get()
            if item is None:
                return
            ff, args, proxy = item
            try:
                packed = ff.result()
                mf = self._motion_pool.submit(_motion_worker, *args, packed)
            except BaseException as e:
                proxy.set_exception(e)
                continue

            def done(m, proxy=proxy):
                e = m.exception()
                proxy.set_exception(e) if e is not None else proxy.set_result(m.result())
            mf.add_done_callback(done)

    def close_image_motion(self):
        if self._feature_pool is not None:
            self._forward_q.put(None)
            self._forwarder.join()
            self._feature_pool.shutdown(cancel_futures=True)
            self._feature_pool = None
        if self._motion_pool is not None:
            self._motion_pool.shutdown(cancel_futures=True)
            self._motion_pool = None
            self._motion_futures.clear()

    def _register_frame_image_motion(self, frame, timestamps, intensity, ring):
        """KissICP.register_frame (kiss_icp 1.3.0) with the image motion in place of last_delta.

        Line for line scripts/run_i3_deskew.py::register_frame (INIT, sigma "fixed"): the
        scan is deskewed with the motion M measured in the intensity image, the ICP starts
        from last_pose @ M (use_as_initial_guess; #030: deskew and initial guess must
        agree), and sigma is `fixed_sigma` for the whole run, never adapted (#031).  With
        fixed_sigma = None the KISS adaptive threshold is used and updated from the ICP
        correction as upstream does.  Map, last_delta and last_pose are updated as upstream.
        """
        odo = self.odometry
        M = self._image_motion(frame, timestamps, intensity, ring)
        cv_deskew = self.image_cfg.deskew_from == "cv"   # #103: deskew with constant velocity (as KISS), the image only as the ICP start
        kiss_floor = self.image_cfg.two_start_kiss       # #108: the second start (and the fallback of a failed image motion) is KISS itself
        if M is None:                         # no image motion: no deskew (#012); ICP start per image_deskew.fallback
            self._fail_streak = getattr(self, "_fail_streak", 0) + 1
            kiss_fb = (self.image_cfg.fallback == "kiss"                    # #146: the failed scan is registered exactly as KISS would
                       and self._fail_streak >= self.image_cfg.fallback_kiss_after)   # #150: only in a run of failures
            delta = odo.last_delta if (cv_deskew or kiss_floor or kiss_fb) else np.eye(4)
            start = odo.last_delta if (self.image_cfg.fallback == "constant_velocity" or kiss_fb or cv_deskew or kiss_floor) else delta
        else:
            self._fail_streak = 0
            delta = M if self.image_cfg.use_for_deskew else np.eye(4)      # False: ICP start only, no deskew (#082)
            if cv_deskew:
                delta = odo.last_delta
            if self.image_cfg.use_for_deskew and self.image_cfg.deskew_rotation == "cv":   # hybrid deskew (#086)
                delta = M.copy()
                delta[:3, :3] = odo.last_delta[:3, :3]
            start = M if self.image_cfg.use_as_initial_guess else odo.last_delta
            raw = getattr(self, "_blend_raw", None)
            if (self.image_cfg.cv_blend == "adaptive" and self.image_cfg.cv_blend_use == "deskew" and raw is not None
                    and raw[0] == self._frame_counter and self.image_cfg.use_as_initial_guess):
                start = raw[1]                                    # #137: the blend deskews, the ICP starts from the image
        if self.image_cfg.deskew_motion_file is not None:   # oracle deskew (#086): the ground-truth motion, deskew only
            if not hasattr(self, "_oracle_deskew"):
                self._oracle_deskew = np.load(self.image_cfg.deskew_motion_file)["motion"]
            D = self._oracle_deskew[self.first_scan_index + self._frame_counter]
            if np.isfinite(D).all():
                delta = D
        fixed_sigma = self.image_cfg.fixed_sigma
        sigma = odo.adaptive_threshold.get_threshold() if fixed_sigma is None else float(fixed_sigma)

        def register(deskew_delta, start_delta):
            deskewed_ = odo.preprocessor.preprocess(frame, timestamps, deskew_delta)
            source_, frame_downsample_ = odo.voxelize(deskewed_)
            guess_ = odo.last_pose @ start_delta
            pose_ = odo.registration.align_points_to_map(
                points=source_, voxel_map=odo.local_map, initial_guess=guess_,
                max_correspondance_distance=3 * sigma, kernel=sigma,
            )
            return deskewed_, source_, frame_downsample_, guess_, pose_

        deskewed, source, frame_downsample, initial_guess, new_pose = register(delta, start)   # deskew from the image
        kept_delta = delta
        two = self.image_cfg.two_start_deg
        if two is not None and M is not None:
            import time
            t0 = time.perf_counter()
            # Starting points (#057, #058): the image motion (already registered), the range motion if asked for, and
            # constant velocity (no deskew).  Registered again only when some pair disagrees by more than two_start_deg.
            cands = {"image": (delta if self.image_cfg.cv_blend_use == "deskew" else M, start if self.image_cfg.cv_blend_use == "deskew" else M)}
            Mr = self._image_motion_est.last_range_motion if (self._image_motion_est is not None and
                                                              self.image_cfg.range_motion in ("candidate", "validate")) else None
            if Mr is not None:
                cands["range"] = (Mr, Mr)
            cands["cv"] = (odo.last_delta if kiss_floor else np.eye(4), odo.last_delta)   # #108: KISS (CV deskew + CV start)
            rot = lambda A, B: np.degrees(np.arccos(np.clip((np.trace((np.linalg.inv(A) @ B)[:3, :3]) - 1) / 2, -1, 1)))
            starts = [c[1] for c in cands.values()]
            disagree = max(rot(a, b) for i, a in enumerate(starts) for b in starts[i + 1:])
            force = self.image_cfg.two_start_always
            if self.image_cfg.range_motion == "validate" and Mr is not None:     # #109: is the image motion consistent with the range one?
                d_ir = rot(M, Mr)
                hist = self._validate_hist
                if len(hist) >= 20 and d_ir > self.image_cfg.validate_k * float(np.median(hist)):
                    force = True
                    self.n_validate_forced += 1
                hist.append(d_ir)
                del hist[:-200]
            if disagree > two or force:
                results = {"image": (deskewed, source, frame_downsample, initial_guess, new_pose)}
                for name, (dsk, st) in cands.items():
                    if name != "image":
                        results[name] = register(dsk, st)
                map_pts = odo.local_map.point_cloud()
                if len(map_pts):
                    tree = KDTree(map_pts)
                    fit = lambda src, pose: float(np.minimum(tree.query(src @ pose[:3, :3].T + pose[:3, 3], workers=-1)[0],
                                                             3 * sigma).mean())
                    fits = {name: fit(r[1], r[4]) for name, r in results.items()}
                    best = min(fits, key=fits.get)
                    if best != "image" and fits[best] >= (1.0 - self.image_cfg.two_start_margin) * fits["image"]:
                        best = "image"                      # not better by the margin: keep the image start
                    deskewed, source, frame_downsample, initial_guess, new_pose = results[best]
                    kept_delta = delta if best == "image" else cands[best][0]
                    if best == "cv" and self.image_cfg.two_start_cv_deskew and self.pose_time_fractions:
                        deskewed, frame_downsample = self._deskew_with_icp_motion(frame, timestamps, new_pose)   # #163
                    self.n_two_start += 1
                    self.n_two_start_cv_won += best == "cv"
                    from scipy.spatial.transform import Rotation as _R
                    motion_of = lambda pose: np.linalg.inv(odo.last_pose) @ pose      # this scan's motion per start
                    row = dict(scan=self._frame_counter, disagree_deg=disagree, kept=best,
                               **{f"fit_{k}": v for k, v in fits.items()},
                               **{f"rot_{k}_deg": rot(np.eye(4), c[1]) for k, c in cands.items()})
                    for k, r in results.items():       # the motion each start converged to: rotation vector (deg), translation (m)
                        mk = motion_of(r[4])
                        row.update({f"{k}_r{a}": v for a, v in zip("xyz", np.degrees(_R.from_matrix(mk[:3, :3]).as_rotvec()))})
                        row.update({f"{k}_t{a}": v for a, v in zip("xyz", mk[:3, 3])})
                    self.two_start_log.append(row)
            self.two_start_seconds += time.perf_counter() - t0
        if self.image_cfg.redeskew and M is not None:      # second pass (#086): deskew with the ICP's motion, register again
            motion = np.linalg.inv(odo.last_pose) @ new_pose
            deskewed, source, frame_downsample, _, new_pose = register(motion, motion)
            kept_delta = motion
        if fixed_sigma is None:
            odo.adaptive_threshold.update_model_deviation(np.linalg.inv(initial_guess) @ new_pose)
        # else: sigma stays at fixed_sigma; the adaptive threshold is never updated
        odo.local_map.update(frame_downsample, new_pose)
        odo.last_delta = np.linalg.inv(odo.last_pose) @ new_pose
        odo.last_pose = new_pose
        self._kept_deskew_delta = kept_delta
        self._reuse_downsample = (deskewed, frame_downsample, odo.config.mapping.voxel_size * 0.5)   # #176
        return deskewed, source

    def _deskew_with_icp_motion(self, frame, timestamps, new_pose):
        """#163: the scan registered without deskew (constant-velocity start won), deskewed with the ICP's own motion for the map.

        The ICP step P_{k-1}^-1 P_k spans from the previous pose's time (fraction f_prev of its sweep) to this pose's (f, the mean point
        time of an undeskewed scan): (1 - f_prev) + f sweeps, contiguous sweeps assumed.  Per-sweep generator L = log(step) / that span;
        each point at its time s is moved by exp((s - f) L), i.e. expressed at the pose's own instant - the pose stays as registered."""
        from scipy.linalg import expm, logm

        odo = self.odometry
        t = np.asarray(timestamps, dtype=np.float64).ravel()
        pts = np.asarray(frame, dtype=np.float64)
        if len(t) != len(pts) or t.max() <= t.min():
            deskewed = pts
        else:
            s = (t - t.min()) / (t.max() - t.min())
            f = self._pose_time_fraction(frame, t, None)
            span = (1.0 - self.pose_time_fractions[-1]) + f
            step = np.linalg.inv(odo.last_pose) @ new_pose
            L = np.real(logm(step)) / max(span, 1e-3)
            bins = np.minimum((s * 100).astype(int), 99)
            deskewed = np.empty_like(pts)
            for b in np.unique(bins):
                m = bins == b
                T = expm(((b + 0.5) / 100.0 - f) * L)
                deskewed[m] = pts[m] @ T[:3, :3].T + T[:3, 3]
        deskewed = odo.preprocessor.preprocess(deskewed, timestamps, np.eye(4))   # the same range crop as the registered scans
        _, frame_downsample = odo.voxelize(deskewed)
        self.n_two_start_cv_deskewed = getattr(self, "n_two_start_cv_deskewed", 0) + 1
        return deskewed, frame_downsample

    def _record_pose_time(self, frame, timestamps, deskew_delta):
        t = np.asarray(timestamps, dtype=np.float64).ravel()
        self.pose_time_fractions.append(self._pose_time_fraction(frame, t, deskew_delta))
        self.sweep_spans.append(float(t.max() - t.min()) if len(t) else np.nan)

    def _pose_time_fraction(self, frame, timestamps, deskew_delta):
        """Instant of the sweep the registered pose stands for (0 = first point, 1 = last point).

        deskew_delta: the motion the scan was deskewed with, None when deskew is off.  kiss_icp 1.3.0 expresses a
        deskewed scan at its last point (Preprocessing.cpp: exp((s - 1) log delta), s the point time normalised to
        [0, 1]), so its pose is the sensor at s = 1.  With no motion (deskew off, identity fallback, the constant-
        velocity start of two_start, the first scans) the points stay where they were measured and the rigid fit of
        the blur lands near their mean time: the mean normalised time of the points inside the range crop.
        """
        if deskew_delta is not None and not np.allclose(deskew_delta, np.eye(4), atol=1e-12):
            return 1.0
        t = np.asarray(timestamps, dtype=np.float64).ravel()
        if len(t) != len(frame) or t.max() <= t.min():
            return 0.5
        s = (t - t.min()) / (t.max() - t.min())
        r = np.linalg.norm(frame, axis=1)
        data = self.odometry.config.data
        keep = (r > data.min_range) & (r < data.max_range)
        return float(s[keep].mean()) if keep.any() else 0.5

    def _refresh_diag_tree(self):
        """Rebuild the diagnostics KDTree at most every `rebuild_every` frames."""
        if self._diag_tree is not None and self._diag_age < self.diag_cfg.rebuild_every:
            self._diag_age += 1
            return
        points = self.odometry.local_map.point_cloud()
        self._diag_tree = KDTree(points) if len(points) else None
        self._diag_map_n = len(points)
        self._diag_age = 1

    def _accumulate_intensity(self,
                               mapping_frame: np.ndarray,
                               pose: np.ndarray,
                               raw_intensity: np.ndarray,
                               raw_frame: np.ndarray):
        """Build a mapping_frame → mean_intensity lookup, per voxel.

        We store intensity per-voxel (keyed by the voxel map's grid cell) so
        that when we later build the Open3D pcd for loop closure we can colour
        each point by its mean observed intensity.

        Keyed in the current local map's frame (`pose` is relative to the node,
        as VoxelMap.integrate_frame uses it), not the global frame.
        """
        if raw_intensity is None or len(raw_intensity) == 0:
            return

        # Interpolate raw intensity → mapping_frame resolution
        mf_intensity = _interpolate_intensity(raw_frame, raw_intensity, mapping_frame)
        if mf_intensity is None:
            return

        # Into the local map's frame (same as VoxelMap.IntegrateFrame does)
        pts_world = transform_points(mapping_frame, pose)

        # Accumulate per voxel
        vx = self.local_map_voxel_size
        for pt, iv in zip(pts_world, mf_intensity):
            key = (int(np.floor(pt[0] / vx)),
                   int(np.floor(pt[1] / vx)),
                   int(np.floor(pt[2] / vx)))
            if key in self._intensity_accumulator:
                s, c = self._intensity_accumulator[key]
                self._intensity_accumulator[key] = (s + float(iv), c + 1)
            else:
                self._intensity_accumulator[key] = (float(iv), 1)

    def _get_voxel_intensities(self, points_world: np.ndarray) -> np.ndarray:
        """Look up accumulated intensity for a set of world-frame points."""
        vx = self.local_map_voxel_size
        out = np.zeros(len(points_world), dtype=np.float32)
        for i, pt in enumerate(points_world):
            key = (int(np.floor(pt[0] / vx)),
                   int(np.floor(pt[1] / vx)),
                   int(np.floor(pt[2] / vx)))
            entry = self._intensity_accumulator.get(key)
            if entry is not None:
                out[i] = entry[0] / entry[1]
        return out

    def _node_edge_information(self):
        """Information of the node-graph edges: diag(1, 1, 1, w, w, w), w = pose_graph_optimizer.rotation_weight (#067)."""
        w = float(self.config.pose_graph_optimizer.rotation_weight)
        return np.diag([1.0, 1.0, 1.0, w, w, w])

    def compute_closures(self, query_id, query):
        accepted = self.closer.compute(query_id, query, self.local_map_graph)
        for source_id, target_id, pose_constraint in accepted:
            self.closures.append((source_id, target_id))
            self.optimizer.add_factor(source_id, target_id, pose_constraint, self._node_edge_information())
        if accepted:
            self.optimize_pose_graph()

    def optimize_pose_graph(self):
        self.optimizer.optimize()
        estimates = self.optimizer.estimates()
        for id_, pose in estimates.items():
            self.local_map_graph[id_].keypose = np.copy(pose)

    def generate_new_node(self):
        points = self.odometry.local_map.point_cloud()
        last_local_map = self.local_map_graph.last_local_map
        relative_motion = last_local_map.local_trajectory[-1]
        inverse_relative_motion = np.linalg.inv(relative_motion)
        transformed_local_map = transform_points(points, inverse_relative_motion)

        self.odometry.local_map.clear()
        self.odometry.local_map.add_points(transformed_local_map)
        self.odometry.last_pose = np.eye(4)
        self._prev_pose = np.eye(4)
        # The map was just re-expressed in the new node's frame: a cached tree
        # would point at the old coordinates.
        self._diag_tree = None

        query_id = last_local_map.id
        query_points = self.voxel_grid.point_cloud()
        self.local_map_graph.finalize_local_map(
            self.voxel_grid,
            intensity_lookup_fn=self._get_voxel_intensities if self._has_intensity else None,
        )
        # Reset accumulator for the new local map
        self._intensity_accumulator = {}

        self.voxel_grid.clear()
        self.voxel_grid.add_points(transformed_local_map)
        self.optimizer.add_variable(self.local_map_graph.last_id, self.local_map_graph.last_keypose)
        self.optimizer.add_factor(
            self.local_map_graph.last_id, query_id, relative_motion, self._node_edge_information()
        )
        self.compute_closures(query_id, query_points)
        if self.local_map_splitting_height is not None:
            # ground_alignment maps the node frame to a frame whose z is the ground normal,
            # so its third row is "up" in the previous node frame; rotate into the new one.
            ground_alignment = self.closer.detector.get_ground_alignment_from_id(query_id)
            self._node_up = relative_motion[:3, :3].T @ ground_alignment[2, :3]
            self._node_up /= np.linalg.norm(self._node_up)
            self.node_up_log.append((self.local_map_graph.last_id, self._node_up.copy()))
            print(f"KissSLAM| node {self.local_map_graph.last_id} up = {np.round(self._node_up, 4).tolist()}")

    @property
    def poses(self):
        poses = [np.eye(4)]
        for node in self.local_map_graph.local_maps():
            for rel_pose in node.local_trajectory[1:]:
                poses.append(node.keypose @ rel_pose)
        return poses

    def fine_grained_optimization(self):
        pgo = PoseGraphOptimizer(self.config.pose_graph_optimizer)
        id_ = 0
        pgo.add_variable(id_, self.local_map_graph[id_].keypose)
        pgo.fix_variable(id_)
        for node in self.local_map_graph.local_maps():
            odometry_factors = [
                np.linalg.inv(T0) @ T1
                for T0, T1 in zip(node.local_trajectory[:-1], node.local_trajectory[1:])
            ]
            for i, factor in enumerate(odometry_factors):
                pgo.add_variable(id_ + 1, node.keypose @ node.local_trajectory[i + 1])
                pgo.add_factor(id_ + 1, id_, factor, np.eye(6))
                id_ += 1
            pgo.fix_variable(id_ - 1)

        pgo.optimize()
        poses = [x for x in pgo.estimates().values()]
        return poses, pgo
