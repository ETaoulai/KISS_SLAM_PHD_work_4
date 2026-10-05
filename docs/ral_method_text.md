# The locked RA-L method (branch `ral_method`, 5/10/2026) — draft text for the paper

Draft in English for the paper; every claim points to the experiment log entry that supports it. Numbers: official protocols of each dataset,
mean of 4 seeds (KISS-SLAM and the other methods: 1 run); ablation in `docs/ablation_165.md`, all methods in `docs/results_summary_164.md`.

## Overview

The method is KISS-SLAM with one change of principle: the motion of the sensor **during** each sweep is measured in the LiDAR's own intensity
image instead of being extrapolated from the previous scan (constant velocity). That motion is used twice — to deskew the scan and as the
initial guess of the ICP — and three safeguards keep the method at least as robust as KISS where the image is unreliable: a second start from
constant velocity, a fallback to a range image, and a fallback to KISS's own constant-velocity prediction where both images are blind. One
adaptive rule, without thresholds, decides per scan how much the deskew trusts the image and how much constant velocity. Everything else —
voxel map, point-to-point ICP with KISS's kernel, local maps, loop closures, pose graph — is KISS-SLAM unchanged.

## 1. Motion of the sweep from the intensity image (core)

Each raw scan is projected to an intensity panorama (azimuth × ring, upsampled vertically; every pixel keeps the 3D point and the timestamp
of its return). Upright SURF features are matched to the previous scan's panorama — within ±40 columns of the previous column shift when the
sensor turns slowly (guided matching), by brute force otherwise — and lifted to 3D points with their own times. RANSAC rejects outliers; a
continuous-time fit then estimates the motion over the sweep with constant angular acceleration and constant velocity ("car" model), each
match contributing at the times of its two points. Matches that do not move on the near floor (patterns attached to the sensor rig) are
removed (#025–#027). The result is the sweep motion M_k.

*Why:* KISS's constant-velocity deskew assumes the motion of the previous scan; on a handheld or flying sensor that assumption is often
wrong within 0.1 s, and the deskew then adds error instead of removing it — KISS without deskew is better than KISS on 17 of 26 sequences
(ablation, row 2; RPE 1 m rotation 2.81° → 0.98°). *Effect* (ablation, row 3): APE 0.42 / 0.27 / 0.32 / 0.65 of KISS on NCD / Spires / Hilti / NTU,
no failure (KISS: 3), better than the previous row on 18 of 25 sequences.

## 2. Deskew and initial guess from the same motion (#030)

M_k replaces KISS's constant-velocity guess in both of its uses: the scan is deskewed with M_k and the ICP starts from the previous pose
composed with M_k. They must agree — deskewing with one motion and starting from another puts the error of the difference into the
trajectory (#030). The ICP keeps a fixed gate σ = 2.0 instead of KISS's adaptive one: with a good initial guess KISS's σ shrinks about four times
and the ICP can no longer correct small systematic errors of M_k (#031; adaptive σ in the ablation: equal on average, NTU 0.49 → 0.69).

*Leave-one-out* (#081 / #082): the image only for the deskew, the ICP from constant velocity → APE 0.76 / 0.55 / 0.70 of KISS and 2 failures;
the image only as the ICP start, no deskew → 0.51 / 0.32 / 0.28, RPE translation 7.25 → 9.65 cm. Both uses are needed: the start for robustness,
the deskew for accuracy.

## 3. Two starts (#057–#059)

When M_k and the constant-velocity guess differ by more than 5° of rotation, the scan is registered a second time from constant velocity
(without deskew), and the registration whose points fit the local map better is kept (the image wins ties within a 2 % margin). This happens
on under 5 % of scans and protects against a wrong image motion (few or repetitive matches, fast rotation). *Effect* (row 4): Spires 0.27 → 0.18
of KISS (the facade aliasing of Blenheim, #053–#057); elsewhere neutral.

## 4. Range-image fallback (#069, #078)

When the intensity image gives no motion, the same estimation runs on a range panorama of the same scan (computed only then). *Effect*
(row 5): Hilti 0.34 → 0.25 and NTU 0.66 → 0.49 of KISS, where the intensity image of the 16- / 64-beam Ousters fails on 20–31 % of scans.

## 5. Adaptive blend for the deskew (B, #131, #137–#140)

The deskew motion becomes a blend of M_k and the constant-velocity prediction C_k: rotation by slerp, translation linearly, with the image
weight w_k = v_C / (v_M + v_C), where v_M and v_C are the mean squared rotation errors of the image and of constant velocity against the ICP
result over the previous 20 scans (causal). No threshold: on a handheld sensor the image keeps about 95 % of the weight (its motion is far
better than constant velocity), on the car about 30 %, on the drone 40–80 %. The ICP still starts from M_k — blending the start too hurt the
sparse 16-beam drone by 33–82 % (#137). *Effect* (row 7): car 0.88 → 0.73 of KISS (RPE −25 %, now better than KISS on every metric), NTU eee
0.51 → 0.37, new NTU 0.52 → 0.44; handheld unchanged.

## 6. KISS fallback in runs of image failures (C, #143, #146, #150–#151, #164)

When both images fail, a scan is registered without deskew and from the previous pose (zero motion) — unless the image has failed for the
4th scan in a row, in which case it is deskewed and started from constant velocity, exactly as KISS would. The two cases differ in kind:
isolated failures happen in abrupt motion (hand-spinning; longest run 3 scans), where constant velocity is the worst guess; long runs happen
where the image is blind (a drone at 15–35 m altitude sees few, distant points; runs of up to 799 scans), where zero motion is wrong for a
moving sensor. *Effect* (row 8, #164): identical to row 7 on 30 of 43 sequences; spms_01 / 02 / 03 8.0 / 48.1 / 17.6 → 6.7 / 8.3 / 0.8 m (KISS
8.7 / 15.4 / 9.5); new NTU 0.44 → 0.32 of KISS; no divergence where constant velocity on every failure did (dynamic_spinning 0.11 vs 1.42 m, #149).

## Runtime

Measured alone on one machine (48 cores; #161, #165, #166) **with unrestricted thread pools**: a serial run uses about 10–13 cores (OpenBLAS / numpy,
OpenCV — capped at 8 —, the KD-tree of the two-start check; KISS's ICP 4 threads) and 1–3 GB of memory. quad_easy (Ouster 128): 10.3 Hz with
`--parallel`, 8–9 Hz serial; the blend and the fallback add no measurable cost (240.5 vs 244.5 s serial, #165). Hesai and car: #166. For the paper
the runtime must be stated at a fixed thread budget (`KISS_THREADS=4` + OMP / OPENBLAS / MKL_NUM_THREADS=4, #168, in progress).

## Limits (to state in the paper)

- Against the strongest LiDAR-only method, Traj-LO (continuous time), the method is less accurate on most sequences (#162); it is on par with
  GenZ-ICP and CT-ICP and better than DLO and MAD-ICP on most datasets. The claim is a robust, large improvement of KISS-SLAM, not a new state of the art.
- Height drift on long trajectories (#060, #077, #153), the stair case (#063), and sensors whose intensity image is sparse (Livox, #095–#116; the
  16-beam NTU Ouster where the image fails at altitude) remain limits.
