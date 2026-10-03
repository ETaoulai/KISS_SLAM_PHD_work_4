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
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Union

import yaml
from kiss_icp.config.config import (
    AdaptiveThresholdConfig,
    DataConfig,
    MappingConfig,
    RegistrationConfig,
)
from kiss_icp.config.parser import KISSConfig
from map_closures.config.config import MapClosuresConfig
from pydantic import BaseModel
from pydantic_settings import BaseSettings, SettingsConfigDict


class KissOdometryConfig(BaseModel):
    preprocessing: DataConfig = DataConfig()
    registration: RegistrationConfig = RegistrationConfig()
    mapping: MappingConfig = MappingConfig()
    adaptive_threshold: AdaptiveThresholdConfig = AdaptiveThresholdConfig()


class LoopCloserConfig(BaseModel):
    detector: MapClosuresConfig = MapClosuresConfig()
    overlap_threshold: float = 0.4
    # How many detector candidates to verify with ICP, best (most inliers) first,
    # stopping at the first accepted one.  1 = upstream (only the best candidate).
    top_k: int = 1
    # Reject a closure whose height difference between the two maps disagrees with the
    # odometry's by more than this (m), measured along the vertical of the query map
    # (MapClosures ground plane).  The density maps carry no height, so a floor can be
    # matched to a stair map above or below it (#015: two such closures, 2.3 m off,
    # passed ICP and the overlap check).  None = upstream (no check).
    max_height_disagreement: Optional[float] = None


class DeskewRefineConfig(BaseModel):
    """Deskew each scan again with its OWN estimated motion, then register again.

    KISS deskews a scan with the motion of the PREVIOUS scan (constant velocity).  For a
    handheld unit that sways with every step this is often worse than no deskew at all
    (church_02: ATE 0.339 vs 0.283 m), while deskewing with the true motion of the scan
    gives 0.101 m (#012).  Each extra pass: deskew the raw scan with
    inv(last_pose) @ current estimate, then ICP again, warm-started from that estimate.

    passes = 1 is upstream KISS.  Only used when `odometry.preprocessing.deskew` is true
    and the intensity arm is off.
    """

    passes: int = 1


class LocalMapperConfig(BaseModel):
    voxel_size: float = 0.5
    splitting_distance: float = 100.0
    # Also start a new local map once the height changes by more than this (m) since
    # the map began, so a map does not span two floors.  None = upstream (distance only).
    # Height is the z of the pose in the node frame, i.e. the sensor frame at the node
    # start; for a roughly level handheld unit that is close to the vertical.
    splitting_height: Optional[float] = None


class IntensityConfig(BaseModel):
    """Controls the intensity-aided ICP experiment.

    `enabled = False` reproduces vanilla upstream KISS-SLAM exactly and is the
    A/B *baseline* arm.  Set it to True for the intensity arm.  Keeping this in
    the config (rather than a hardcoded constant) means the value is written to
    `slam_config.yaml` in the results dir, so every run records which arm it was.
    """

    enabled: bool = False
    # How the intensity-selected points reach the ICP:
    #   "refine"  – the original method: standard KISS ICP on all points, then a
    #               second ICP on the selected points, warm-started from the first
    #               result, against a map that ALREADY contains this scan.
    #   "replace" – a single ICP on the selected points only, before the scan is
    #               added to the map (the map still receives every point).  Tests
    #               the selection on its own, without the first ICP's answer.
    mode: Literal["refine", "replace"] = "refine"
    # Source filtering (odometry)
    keep_ratio: float = 0.70
    min_intensity: float = 0.05
    # Loop-closure ColoredICP: 1.0 = pure geometry, 0.0 = pure photometry
    lambda_geometric: float = 0.90


class ImageDeskewConfig(BaseModel):
    """Deskew and ICP initial guess from the motion measured in the intensity image (#018-#032).

    For each scan the motion during the sweep is estimated from SIFT (or SURF, `detector`)
    matches between the intensity panoramas of the previous and the current raw scan, each
    pixel carrying its own 3D point and time (`kiss_slam.intensity_deskew.ScanMotionEstimator`).  This motion
    replaces KISS's constant-velocity guess `last_delta` BOTH for deskewing the scan AND as
    the ICP initial guess (`last_pose @ M`): the two must agree, otherwise the start drifts
    (#030).  The adaptive threshold sigma is kept fixed (#031: with a good initial guess the
    KISS sigma collapses 2.4 -> 0.6 and the ICP corrects too little).  When the estimate
    fails the scan is not deskewed (identity), which beats the KISS guess (#012).

    church_02 0.130 m (KISS 0.319), christ-church-03 0.038 (0.122), keble-college-02 0.094
    (2.133), one setting for all (#031-#032).  This is the config-driven form of
    `scripts/precompute_i3_motion.py --model=car --subpixel --stuck=0.05 --floor-only` +
    `scripts/run_i3_deskew.py ... 0 init fixed`.  Mutually exclusive with `intensity.enabled`.
    """

    enabled: bool = False
    # Motion model over the two scans: constant velocity, acceleration in rotation only
    # (#021, default), constant acceleration in rotation and translation (#020).
    model: Literal["cv", "car", "ca"] = "car"
    # Bilinear interpolation of point and time inside the pixel (#021).
    subpixel: bool = True
    # Features matched between the two panoramas: "sift" (every result so far) or "surf".
    # SURF needs OpenCV built with contrib + OPENCV_ENABLE_NONFREE=ON (not in the pip wheels).
    # surf_hessian_threshold: higher = fewer, stronger keypoints (OpenCV default 100).
    # surf_upright: no keypoint orientation (U-SURF); the panorama is never rotated in-plane.
    detector: Literal["sift", "surf", "orb"] = "sift"   # "orb" since #088
    # Multiplies the raw intensity before the panorama, which clips at 255 (built for the Hesai
    # 0-255 scale).  1.0 = Hesai.  Ouster (0 to ~1100, median 150-450): 255/1024 = 0.249 (#041).
    intensity_scale: float = 1.0
    surf_hessian_threshold: float = 100.0
    # Default True since #088 (ΑΠΟΦΑΣΗ Μ.Τ. 2/10, branch rotation_bearing): -30 % time, +22-31 % inliers (#086).  False = every result before #088.
    surf_upright: bool = True
    # Drop matches that are the same point in the sensor frame, |p - q| < stuck_min (m):
    # intensity patterns travelling with the sensor (#025-#026).  None = keep all.
    stuck_min: Optional[float] = 0.05
    # Apply stuck_min only to the near floor (#027): below stuck_elev_deg in the sensor
    # frame and closer than stuck_range_m.
    stuck_floor_only: bool = True
    stuck_elev_deg: float = -10.0
    stuck_range_m: float = 5.0
    # Also use the image motion as the ICP initial guess (#030).  False = deskew only.
    use_as_initial_guess: bool = True
    # Use the image motion to deskew the scan (#082).  False = the scan is not deskewed and the image motion is only the ICP
    # initial guess (with use_as_initial_guess); the counterpart of use_as_initial_guess = False (deskew only, #081).
    use_for_deskew: bool = True
    # Rotation gap (#086, the image rotation's per-scan noise passes through the deskew, #068 / #082): deskew_rotation "cv" =
    # deskew with the image TRANSLATION and KISS's constant-velocity rotation (the ICP start stays the image motion);
    # redeskew = after the ICP, deskew the scan again with the ICP's own motion and register once more (second pass).
    deskew_rotation: Literal["image", "cv"] = "image"
    redeskew: bool = False
    # Diagnostic only (#086, uses the ground truth): an .npz with "motion" (N,4,4) per scan (NaN = keep the method's) that
    # REPLACES the deskew motion only - ICP start, two starts, map unchanged.  scripts/make_oracle_motion.py writes it.
    deskew_motion_file: Optional[str] = None
    # #087 (branch rotation_bearing): rotation_from_bearings = min range (m) of the matches whose directions re-estimate the
    # rotation after the 3D fit (translation fixed); guided_matching_window = match only within this many panorama columns
    # (and 4 rings) of the same pixel instead of brute force over the whole panorama.  None = off (every result before #087).
    rotation_from_bearings: Optional[float] = None
    # Default 40 since #088 (ΑΠΟΦΑΣΗ Μ.Τ. 2/10): with upright SURF, rotation -3.5 %, RTE -2 %, real time (#087).  None = every result before #088.
    guided_matching_window: Optional[float] = 40.0
    # #089: centre of the guided window - "shift" (median column shift of the previous scan; guided only when turning slowly, #088) or
    # "motion" (each keypoint's 3D point moved by the previous scan's motion and projected into the new panorama; guided also when fast).
    # "hybrid": the shift while turning slowly (|shift| <= 20 columns), the motion prediction when fast (instead of brute force).
    guided_prediction: Literal["shift", "motion", "hybrid"] = "shift"
    # Adaptive threshold: fixed at this value (m) for the whole run (#031).
    # None = KISS adaptive (updated from the ICP correction of the initial guess).
    fixed_sigma: Optional[float] = 2.0
    # Ablation (#047), applied to the image motion before BOTH its uses (deskew, initial guess):
    # use_parts "full" | "translation" (no rotation) | "rotation" (no translation);
    # rotation_smoothing k > 1: mean rotation vector of the last k successful image motions (causal).
    use_parts: Literal["full", "translation", "rotation"] = "full"
    rotation_smoothing: int = 1
    # Weighted image rotation (#092): R = R_img exp(w log(R_img^T R_cv)), R_cv the rotation of the last ICP motion
    # (constant velocity), for BOTH uses of the image motion.  0 = the image rotation as measured (every result before #092).
    rotation_cv_weight: float = 0.0
    # Deskew source (#103): "image" = the image motion (every result before); "cv" = KISS's constant velocity (last_delta), the image motion
    # then only the ICP start (and two starts).  Where the image fails, "cv" is exactly KISS.
    deskew_from: Literal["image", "cv"] = "image"
    # #108: the "cv" start of the two starts, and the fallback where the image motion fails, become KISS itself (constant-velocity deskew
    # AND start) instead of "no deskew" - per scan the method then always has the KISS registration as an option.  False = every result before.
    two_start_kiss: bool = False
    # #109: per-scan validation of the image motion by the range-image motion (range_motion "validate": computed on every scan, also the
    # fallback): when their rotations differ by more than validate_k x the running median of that difference (last 200 scans, at least
    # 20 seen), the scan goes through the two-start choice (image / range / cv) even below two_start_deg.  Scale-free: no angle per sensor.
    validate_k: float = 3.0
    # #109: register EVERY scan from all starts and keep the best map fit (the unconditional floor; ~2x ICP).  False = every result before.
    two_start_always: bool = False
    # #115 (B.9): deskew INSIDE the registration (kiss_slam/ct_registration.py): start and end pose of the sweep, points at their own time.
    # "image" = the end pose initialised from the image motion; "cv" = from constant velocity (LiDAR-only CT baseline); None = off.
    ct_registration: Optional[Literal["image", "cv", "joint"]] = None   # "joint": image start + the image matches as residuals in the cost
    ct_image_weight: float = 1.0       # #115 joint: weight of one image match relative to one geometric point
    ct_max_iter: int = 30              # #115: Gauss-Newton iterations of the CT registration
    ct_lambda: float = 0.1             # weight of the location / velocity constraints, per point (ECTLO: 0.1)
    # Plausibility gate (#054), each None = off (every result before #054): an image motion is rejected when fewer
    # than gate_min_matches matches support it, when it rotates more than gate_max_rotation_deg in one scan, or when its
    # rotation differs from the last accepted one by more than gate_max_rotation_change_deg.  blenheim-palace-02 (#053):
    # ~26 matches and rotations of 30-50 deg where the truth was ~3 deg.
    gate_min_matches: Optional[int] = None
    gate_max_rotation_deg: Optional[float] = None
    gate_max_rotation_change_deg: Optional[float] = None
    # What a scan without an image motion (failed or rejected) gets: "identity" = no deskew and ICP started from the
    # last pose (every result before #054); "constant_velocity" = no deskew, ICP started from KISS's constant-velocity
    # guess last_pose @ last_delta.
    fallback: Literal["identity", "constant_velocity"] = "identity"
    # Two starting points (#057): when the image motion and KISS's constant-velocity guess differ by more than this
    # (deg of rotation), register the scan twice — (A) deskewed with the image motion and started from it, (B) not
    # deskewed and started from constant velocity — and keep the result that fits the local map better (truncated mean
    # distance of the source to its nearest map point).  None = off (every result before #059).  Default 5 since #059
    # (decision M.T. 25/9): fixes blenheim-palace-02 (5.3 -> 0.3 m) with no significant change on 15 other sequences (#058).
    two_start_deg: Optional[float] = 5.0
    # Margin of the two-start choice (open_tasks B.6, #060): another start replaces the image start only when its fit is
    # better by more than this fraction, fit_other < (1 - margin) * fit_image.  0 = the plain best fit (#057-#060).
    two_start_margin: float = 0.0
    # Motion from the RANGE panorama too (#058; log range + CLAHE, SURF at range_hessian): "fallback" = used when the
    # intensity motion fails or is rejected; "candidate" = an extra starting point for the two-start registration.
    range_motion: Optional[Literal["fallback", "candidate", "validate"]] = None
    # Intensity of the panorama (#075): "none" = intensity x intensity_scale (every result before #075); "gain" = per scan,
    # scaled so its 99th percentile is 255 (no per-sensor scale; Hilti failures 447 -> 60 on UZH); "gain_clahe" = gain +
    # local contrast equalisation.  Panorama columns: None = 1024 (every result before #075); 2048 = the Hilti Ouster's own.
    intensity_normalisation: Literal["none", "gain", "gain_clahe", "range2", "range_smooth", "range1", "log", "logr2", "rangefit", "logrf"] = "none"   # #118, #123-#125
    panorama_width: Optional[Union[int, Literal["auto"]]] = None   # "auto" (#093): the sensor's own columns per revolution
    # Vertical upscaling of the panorama for the detector (#089).  None = 8 (every result before).  4 suits 128-beam sensors (0.7 deg per
    # ring: 8 over-samples 4x) - on underground_hard -26 % image time and -10 % rotation error.
    panorama_up: Optional[Union[int, Literal["auto"]]] = None     # "auto" (#093): square pixels from the measured ring spacing
    # Image-motion fit (#093), all off by default: fit_sectors = equal total weight per azimuth sector in the time fit; whiten =
    # (sigma_r m, sigma_az rad, sigma_el rad), residuals in range / azimuth / elevation over their uncertainty, one robust loss per point;
    # cross_check = keep only mutual best matches.
    fit_sectors: Optional[int] = None
    whiten: Optional[List[float]] = None
    cross_check: bool = False
    # Scale of the panorama for the feature detector only (#093): < 1 shrinks it before SURF (speed), keypoints mapped back.  1 = unchanged.
    detect_scale: float = 1.0
    range_hessian: float = 10.0
    # Folder for the panoramas + matches of every failed / rejected scan (rejected.csv lists them).  None = off.
    save_rejected_dir: Optional[str] = None
    # Near-field bias of intensity matching (#039): matches closer than ~5 m report only ~74 %
    # of the true translation, those beyond ~12 m report 100 %, so every translation comes out
    # 1.5-5 % short.  With trans_min_range set (m), the correction is measured online and
    # causally: per scan the ratio of the translation its matches beyond that range imply to the
    # one all its matches imply, then the running median of the PREVIOUS scans (window in
    # intensity_deskew.TRANS_AUTO_WINDOW) multiplies the translation.  No ground truth, and it
    # follows the scene.  None = off.  trans_mode: "auto" (running median), "magnitude" or
    # "vector" (per-scan, noisier; see #039).
    trans_min_range: Optional[float] = None
    trans_mode: str = "auto"
    # Online estimate in a separate worker process: SlamPipeline hands it scan k+1 before the ICP of
    # scan k, so image motion and ICP overlap and a scan costs max(image, ICP) instead of their sum.
    # One worker, scans in order: the same motions as the serial estimate.
    parallel: bool = False
    # Seed of the estimator's RANSAC: another seed is an independent run, for the spread (#037).
    seed: int = 0
    # Precomputed motion (.npz with "motion" (N,4,4), NaN where failed) from
    # scripts/precompute_i3_motion.py, indexed by scan counter.  None = estimate online
    # from the raw scan (needs intensity and ring per point).
    motion_file: Optional[str] = None


class DiagnosticsConfig(BaseModel):
    """Per-frame ICP residual diagnostics (`icp_*` columns of `icp_metrics.csv`).

    Read-only: nothing here can change the estimated trajectory.  Verified on 400
    frames of church_02: poses differ from the diagnostics-on run by at most 1.4e-14 m,
    which is KISS-ICP's own run-to-run noise (two identical runs differ by 1.1e-14 m,
    from multithreaded reductions in the C++ registration).

    The residual needs a nearest-neighbour index over the whole local map; the KISS
    voxel map exposes no NN query to Python, so a scipy KDTree is built.

    Measured on `indoor_fast`, 400 frames (ms/frame, and bias of the mean RMS):

        icp_metrics=false            25.7 ms   2.37x faster   residual columns = NaN
        rebuild_every=1  (default)   60.8 ms   1.00x          exact
        rebuild_every=2              44.3 ms   1.37x          RMS  +5 %
        rebuild_every=3              38.7 ms   1.57x          RMS  +9 %
        rebuild_every=5              34.0 ms   1.79x          RMS +18 %
        rebuild_every=10             30.8 ms   1.98x          RMS +47 %

    Caching the tree is a poor trade: it buys less speed than switching the residual
    off, and inflates it because the reused map lacks the most recent frames.  To go
    fast, set `icp_metrics: false`; the other diagnostics (motion, model deviation,
    adaptive sigma, geometry) stay on because they are cheap.  Keep `rebuild_every=1`
    whenever the absolute RMS value matters, and never compare RMS across runs made
    with different settings.

    The cache is always invalidated when a new local-map node is created, because the
    map is re-expressed in the new node's frame at that point.
    """

    icp_metrics: bool = True
    rebuild_every: int = 1
    # Keep every scan as deskewed and registered (voxel-downsampled to this size, m), and write them to
    # deskewed_frames.npz in the results dir: the map an arm builds, for the map-sharpness test (#048).
    # None = off.  Read-only: does not change the trajectory.
    save_deskewed_voxel: Optional[float] = None
    # Keep only this random fraction of each saved scan's points (own RNG, seed 0): the map-sharpness score
    # samples 2 M points anyway, and a whole cloister run at 5 cm is 2.1 GB (#049).
    save_deskewed_fraction: float = 1.0


class OccupancyMapperConfig(BaseModel):
    free_threshold: float = 0.2
    occupied_threshold: float = 0.65
    resolution: float = 0.5
    max_range: Optional[float] = None
    z_min: float = 0.1
    z_max: float = 0.5


class PoseGraphOptimizerConfig(BaseModel):
    max_iterations: int = 10
    # Information of the node-graph edges (odometry between local maps and loop closures): diag(1, 1, 1, w, w, w).
    # 1 = upstream (identity).  g2o measures the rotation error as the quaternion vector (~theta/2), so with identity
    # rotation is nearly free next to translation and, after a loop closure, the graph tilts its 100 m nodes to make the
    # horizontal correction: NCD 2020 long experiment height error 0.19 m without closures, 2.1 m with them.  w = 100:
    # two starts APE 2.11 -> 0.39 m in an offline replay of the back end, 01_short unchanged (#067).  The per-scan
    # smoothing (fine_grained_optimization) keeps identity.
    rotation_weight: float = 1.0


class KissSLAMConfig(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="kiss_slam_")
    out_dir: str = "slam_output"
    odometry: KissOdometryConfig = KissOdometryConfig()
    deskew_refine: DeskewRefineConfig = DeskewRefineConfig()
    local_mapper: LocalMapperConfig = LocalMapperConfig()
    intensity: IntensityConfig = IntensityConfig()
    image_deskew: ImageDeskewConfig = ImageDeskewConfig()
    diagnostics: DiagnosticsConfig = DiagnosticsConfig()
    occupancy_mapper: OccupancyMapperConfig = OccupancyMapperConfig()
    loop_closer: LoopCloserConfig = LoopCloserConfig()
    pose_graph_optimizer: PoseGraphOptimizerConfig = PoseGraphOptimizerConfig()

    def kiss_icp_config(self) -> KISSConfig:
        return KISSConfig(
            out_dir=self.out_dir,
            data=self.odometry.preprocessing,
            registration=self.odometry.registration,
            mapping=self.odometry.mapping,
            adaptive_threshold=self.odometry.adaptive_threshold,
        )


class KissDumper(yaml.Dumper):
    # HACK: insert blank lines between top-level objects
    # inspired by https://stackoverflow.com/a/44284819/3786245
    def write_line_break(self, data=None):
        super().write_line_break(data)

        if len(self.indents) == 1:
            super().write_line_break()


def _yaml_source(config_file: Optional[Path]) -> Dict[str, Any]:
    data = None
    if config_file is not None:
        with open(config_file) as cfg_file:
            data = yaml.safe_load(cfg_file)
    return data or {}


def load_config(config_file: Optional[Path]) -> KissSLAMConfig:
    """Load configuration from an Optional yaml file. Additionally, deskew and max_range can be
    also specified from the CLI interface"""

    config = KissSLAMConfig(**_yaml_source(config_file))

    # Use specified voxel size or compute one using the max range
    if config.odometry.mapping.voxel_size is None:
        config.odometry.mapping.voxel_size = float(config.odometry.preprocessing.max_range / 100.0)

    if config.occupancy_mapper.max_range is None:
        config.occupancy_mapper.max_range = config.odometry.preprocessing.max_range

    return config


def write_config(config: KissSLAMConfig = KissSLAMConfig(), filename: str = "kiss_slam.yaml"):
    with open(filename, "w") as outfile:
        yaml.dump(
            config.model_dump(),
            outfile,
            Dumper=KissDumper,
            default_flow_style=False,
            sort_keys=False,
            indent=4,
        )
