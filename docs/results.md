# Results — all datasets, all methods (consolidated 9/10/2026)

One place for every result of the project. Each section: **date · log entries** (`docs/experiment_log.md`) · protocol · table · summary.
Conclusions are ⏳ until validated by Λ.Γ. Replaces the separate files `results_summary_162/164.md`, `results_official*.md`, `results_all*.{md,csv}`,
`results_comparison_085.md`, `results_panorama_rule_096.md`, `comparisons_best_offset.txt`, `map_sharpness.md` (all in git history; the per-run tables
they were made from are under `/home/photogrammetry/kiss_runs/`). Ablations and option tests: `docs/ablations.md`.

**Conventions.** Evaluation = each dataset's **official protocol only** (decision M.T. 25/9, #061): Newer College / Oxford Spires evo APE `--align`
(t_max_diff 0.01, RPE 1 m / 1 s, RTE = KITTI 100–800 m), Hilti 2021 `evaluate_hilti.py`, Hilti 2022 `evaluate_hilti2022.py`, NTU VIRAL `evaluate_ntu.py`
(prism ATE), MulRan / Boreas evo APE + RTE in the LiDAR frame. Every pose at the instant it stands for, no time offset. Ours = **method C** (locked RA-L
method, decision M.T. 5/10: SURF, two starts, range-image fallback, blend for the deskew, KISS fallback after 4 failures, σ = 2.0), mean of **4 seeds**
(± = σ); KISS-SLAM and the other methods: one run (deterministic). Other methods with their authors' published configuration (#083). † = failure (> 5 m).
Single-run ATE differences < 0.02–0.04 m are not measurable (#037).

## Contents

| § | what | date | log |
|---|---|---|---|
| 1 | Main table: 42 sequences (NCD, Oxford Spires 6, Hilti 2021 6, NTU 18, Boreas 3000 scans), all arms and other methods | 1/10–5/10 | #081–#083, #141, #150–#164 |
| 2 | Oxford Spires — the 7 further sequences with ground truth | 8/10 | #228, #229 |
| 3 | Hilti 2021 (12) and 2022 (16), official protocols, all methods | 9/10 | #238–#241 |
| 4 | Car: Boreas 8 full drives, MulRan 4, KITTI 07 | 2/10–8/10 | #084, #085, #207–#227 |
| 5 | Car configuration on hand-held / drone | 7/10–8/10 | #222, #226 |
| 6 | Runtime | 5/10–6/10 | #165, #166, #170–#202 |
| 7 | Solid-state LiDARs (Livox) | 2/10–3/10 | #095–#116 |
| 8 | Map sharpness against survey maps | 19/9 | #048, #049, #051 |

---

## 1. Main table — 42 sequences (1/10–5/10, #081–#083, #141, #150–#164)

APE / ATE m, official protocols; ours 4 seeds. Columns "Ours #081 … + blend + fallback after 4 (C)" = the method's development (C = locked method).
IMU methods (FAST-LIO2, COIN-LIO) are reference only. **Bold** = best LiDAR-only. Note: for the 6 Hilti 2021 sequences here, §3 has the same arms re-run (#241).

| sequence | KISS-SLAM | KISS no deskew | GenZ-ICP | MAD-ICP | DLO | CT-ICP | Traj-LO | FAST-LIO2 (IMU) | COIN-LIO (IMU) | Ours #081 (paper so far) | Ours default | + blend (B) | + blend + fallback (A) | + blend + sectors | + blend + fallback after 4 (C) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 01_short | 0.419 | 0.350 | 0.512 | 1.741 | – | 0.302 | – | – | – | 0.307 | 0.301 | 0.304 | 0.304 | **0.299** | 0.304 |
| 02_long_experiment | 1.275 | 3.498 | 1.893 | – | **0.339** | 0.483 | – | 0.342 | – | 1.619 | 2.190 | 1.901 | 1.901 | 2.048 | 1.901 |
| quad_easy | 0.104 | 0.083 | 0.077 | 0.090 | 0.082 | 0.073 | **0.070** | 0.067 | 0.068 | 0.079 | 0.077 | 0.077 | 0.077 | 0.075 | 0.077 |
| quad_hard | 0.329 | 0.209 | 0.120 | 5.407 † | 0.134 | 0.054 | **0.053** | 0.066 | 0.050 | 0.218 | 0.233 | 0.234 | 0.234 | 0.200 | 0.234 |
| cloister | 0.396 | 0.480 | 0.152 | 0.936 | 0.186 | 0.393 | **0.061** | 0.105 | 0.053 | 0.188 | 0.180 | 0.180 | 0.180 | 0.182 | 0.180 |
| math_easy | 0.160 | 0.104 | 0.098 | 0.087 | 0.166 | 0.097 | **0.083** | 0.094 | 0.093 | 0.108 | 0.103 | 0.103 | 0.103 | 0.105 | 0.103 |
| math_medium | 0.253 | 0.164 | 0.146 | 0.178 | 0.816 | 0.139 | **0.116** | 0.104 | 0.115 | 0.152 | 0.153 | 0.157 | 0.157 | 0.157 | 0.157 |
| underground_easy | 0.117 | 0.092 | 0.056 | 0.077 | 0.243 | 0.046 | **0.026** | 0.036 | 0.038 | 0.067 | 0.066 | 0.065 | 0.065 | 0.065 | 0.065 |
| underground_medium | 0.162 | 0.097 | 0.078 | 0.113 | 0.058 | 0.044 | **0.028** | 0.036 | 0.039 | 0.061 | 0.057 | 0.059 | 0.059 | 0.056 | 0.059 |
| underground_hard | 12.8 † | 12.3 † | 0.105 | 8.910 † | 0.568 | 9.594 † | **0.050** | 0.053 | 0.054 | 0.086 | 0.091 | 0.091 | 0.091 | 0.089 | 0.091 |
| christ-church-02 | 0.777 | 0.546 | 0.178 | 0.830 | 0.467 | 21.8 † | 0.290 | 0.342 | – | 0.207 | **0.160** | 0.227 | 0.227 | 0.200 | 0.227 |
| christ-church-03 | 0.143 | 0.089 | 0.064 | 0.124 | 0.054 | 0.056 | **0.017** | 0.018 | – | 0.044 | 0.039 | 0.039 | 0.039 | 0.040 | 0.039 |
| keble-college-03 | 9.757 † | 11.4 † | 0.333 | 0.358 | 0.338 | 0.090 | **0.053** | 0.065 | – | 0.094 | 0.089 | 0.090 | 0.090 | 0.090 | 0.090 |
| observatory-quarter-01 | 0.497 | 0.436 | 0.104 | 0.545 | 0.214 | 0.105 | **0.053** | 0.058 | – | 0.080 | 0.072 | 0.067 | 0.067 | 0.076 | 0.067 |
| blenheim-palace-02 | 0.293 | 0.205 | 0.317 | 0.539 | 0.485 | 0.303 | 0.228 | 0.151 | – | 0.268 | 0.214 | 0.197 | 0.198 | **0.196** | 0.197 |
| bodleian-library-02 | 1.911 | 1.363 | 0.657 | 2.077 | 1.694 | **0.520** | 0.877 | 0.247 | – | 0.545 | 0.649 | 0.564 | 0.564 | 0.642 | 0.564 |
| Construction_Site_1 | 0.063 | 0.062 | 0.032 | 0.168 | 0.120 | 0.034 | **0.027** | 0.023 | – | 0.048 | 0.042 | 0.037 | 0.042 | 0.043 | 0.037 |
| Office_Mitte_1 | 4.286 | 0.575 | **0.117** | 0.176 | 0.120 | 0.126 | 1631.7 † | 0.121 | – | 0.241 | 0.185 | 0.248 | 0.248 | 0.206 | 0.248 |
| IC_Office_1 | 6.344 † | 1.655 | 0.069 | 0.941 | 0.208 | **0.061** | 0.062 | 0.074 | – | 0.071 | 0.073 | 0.074 | 0.074 | 0.078 | 0.074 |
| LAB_Survey_2 | 0.062 | 0.050 | 0.036 | 0.035 | 0.076 | 0.038 | **0.026** | 0.026 | – | 0.036 | 0.035 | 0.035 | 0.035 | 0.036 | 0.035 |
| Basement_1 | 0.055 | 0.078 | 0.069 | 0.106 | 0.083 | 0.066 | 0.040 | 0.030 | – | 0.050 | 0.053 | 0.046 | 0.046 | **0.036** | 0.046 |
| UZH_Tracking_Area_Run_2 | 0.585 | 0.204 | 0.198 | **0.188** | 0.196 | 0.498 | 0.270 | 0.188 | – | 0.551 | 0.576 | 0.576 | 0.576 | 0.571 | 0.576 |
| eee_01 | 2.678 | 2.363 | 1.597 | 1.503 | 0.220 | 0.234 | **0.082** | 0.087 | – | 1.737 | 1.483 | 1.553 | 1.454 | 1.607 | 1.601 |
| eee_02 | 1.486 | 1.490 | 0.222 | 1.271 | 0.149 | 0.096 | **0.075** | 0.072 | – | 0.679 | 0.836 | 0.790 | 0.797 | 0.811 | 0.812 |
| eee_03 | 0.864 | 0.841 | 0.739 | 2.477 | 0.226 | 0.287 | **0.111** | 0.111 | – | 0.344 | 0.360 | 0.142 | 0.140 | 0.272 | 0.145 |
| nya_01 | 0.736 | – | – | – | – | – | – | – | – | – | **0.355** | 0.359 | 0.360 | – | 0.359 |
| nya_02 | 1.508 | – | – | – | – | – | – | – | – | – | **0.191** | 0.200 | 0.200 | – | 0.200 |
| nya_03 | 1.052 | – | – | – | – | – | – | – | – | – | **0.519** | 0.536 | 0.536 | – | 0.536 |
| sbs_01 | 0.976 | – | – | – | – | – | – | – | – | – | **0.354** | 0.367 | 0.390 | – | 0.370 |
| sbs_02 | 1.143 | – | – | – | – | – | – | – | – | – | 0.759 | 0.767 | **0.603** | – | 0.723 |
| sbs_03 | 1.214 | – | – | – | – | – | – | – | – | – | **0.793** | 0.815 | 0.850 | – | 0.810 |
| rtp_01 | 3.985 | – | – | – | – | – | – | – | – | – | 0.264 | 0.240 | **0.220** | – | 0.249 |
| rtp_02 | 3.575 | – | – | – | – | – | – | – | – | – | 2.604 | 0.335 | **0.327** | – | 0.337 |
| rtp_03 | 3.408 | – | – | – | – | – | – | – | – | – | 0.232 | **0.215** | 0.227 | – | 0.226 |
| tnp_01 | **2.132** | – | – | – | – | – | – | – | – | – | 2.151 | 2.168 | 2.153 | – | 2.168 |
| tnp_02 | 2.786 | – | – | – | – | – | – | – | – | – | 2.820 | **2.762** | 2.791 | – | **2.762** |
| tnp_03 | 2.613 | – | – | – | – | – | – | – | – | – | **1.086** | 1.118 | 1.228 | – | 1.118 |
| spms_01 | 8.743 † | – | – | – | – | – | – | – | – | – | 8.646 † | 8.047 † | 10.7 † | – | **6.668 †** |
| spms_02 | 15.4 † | – | – | – | – | – | – | – | – | – | 48.3 † | 48.1 † | **7.298 †** | – | 8.329 † |
| spms_03 | 9.518 † | – | – | – | – | – | – | – | – | – | 26.3 † | 17.6 † | **0.410** | – | 0.818 |
| Boreas | 0.266 | 7.126 † | – | – | – | – | – | – | – | 0.273 | 0.233 | **0.195** | 0.196 | 0.205 | **0.195** |
| stairs (special) | 3.586 | 2.705 | 2.003 | **0.136** | 0.175 | 4.129 | 0.191 | 732.5 † | 0.218 | 2.074 | 1.923 | 1.643 | 1.643 | 2.257 | 1.643 |
| dynamic_spinning (special) | 0.159 | 20.8 † | 15.6 † | 26.2 † | 4.450 | 10.8 † | **0.080** | 0.085 | – | 0.504 | 0.171 | 0.111 | 1.415 | 0.146 | 0.114 |

### Per-dataset summaries (special sequences excluded)

#### NCD (10 sequences)

| arm | sequences | median APE / KISS | geo-mean APE / KISS | LiDAR-only wins | failures (> 5 m) | mean rank* |
|---|---|---|---|---|---|---|
| KISS-SLAM | 10 | 1.00 | 1.00 | 0 | 1 | 9.0 |
| KISS no deskew | 10 | 0.79 | 0.88 | 0 | 1 | 8.4 |
| GenZ-ICP | 10 | 0.53 | 0.41 | 0 | 0 | 4.2 |
| MAD-ICP | 9 | 0.70 | 1.37 | 0 | 2 | – |
| DLO | 9 | 0.47 | 0.55 | 1 | 0 | – |
| CT-ICP | 10 | 0.58 | 0.49 | 0 | 1 | 2.6 |
| Traj-LO | 8 | 0.20 | 0.17 | 8 | 0 | – |
| FAST-LIO2 (IMU) | 9 | 0.27 | 0.21 | – | 0 | – |
| COIN-LIO (IMU) | 8 | 0.28 | 0.18 | – | 0 | – |
| Ours #081 (paper so far) | 10 | 0.63 | 0.41 | 0 | 0 | 5.8 |
| Ours default | 10 | 0.62 | 0.42 | 0 | 0 | 4.5 |
| + blend (B) | 10 | 0.63 | 0.41 | 0 | 0 | 4.5 |
| + blend + fallback (A) | 10 | 0.63 | 0.41 | 0 | 0 | 5.3 |
| + blend + sectors | 10 | 0.61 | 0.41 | 1 | 0 | 4.5 |
| + blend + fallback after 4 (C) | 10 | 0.63 | 0.41 | 0 | 0 | 6.2 |

*mean rank among the 10 LiDAR-only arms run on all 10 sequences: KISS-SLAM, KISS no deskew, GenZ-ICP, CT-ICP, Ours #081 (paper so far), Ours default, + blend (B), + blend + fallback (A), + blend + sectors, + blend + fallback after 4 (C)

#### Oxford Spires (6 sequences)

| arm | sequences | median APE / KISS | geo-mean APE / KISS | LiDAR-only wins | failures (> 5 m) | mean rank* |
|---|---|---|---|---|---|---|
| KISS-SLAM | 6 | 1.00 | 1.00 | 0 | 1 | 11.5 |
| KISS no deskew | 6 | 0.71 | 0.78 | 0 | 1 | 10.0 |
| GenZ-ICP | 6 | 0.29 | 0.25 | 0 | 0 | 8.0 |
| MAD-ICP | 6 | 1.08 | 0.65 | 0 | 0 | 12.3 |
| DLO | 6 | 0.52 | 0.41 | 0 | 0 | 10.0 |
| CT-ICP | 6 | 0.33 | 0.43 | 1 | 1 | 8.2 |
| Traj-LO | 6 | 0.25 | 0.14 | 3 | 0 | 4.5 |
| FAST-LIO2 (IMU) | 6 | 0.13 | 0.12 | – | 0 | – |
| Ours #081 (paper so far) | 6 | 0.28 | 0.18 | 0 | 0 | 6.0 |
| Ours default | 6 | 0.24 | 0.16 | 1 | 0 | 3.8 |
| + blend (B) | 6 | 0.28 | 0.16 | 0 | 0 | 3.0 |
| + blend + fallback (A) | 6 | 0.28 | 0.16 | 0 | 0 | 4.3 |
| + blend + sectors | 6 | 0.27 | 0.17 | 1 | 0 | 4.5 |
| + blend + fallback after 4 (C) | 6 | 0.28 | 0.16 | 0 | 0 | 4.8 |

*mean rank among the 13 LiDAR-only arms run on all 6 sequences: KISS-SLAM, KISS no deskew, GenZ-ICP, MAD-ICP, DLO, CT-ICP, Traj-LO, Ours #081 (paper so far), Ours default, + blend (B), + blend + fallback (A), + blend + sectors, + blend + fallback after 4 (C)

#### Hilti 2021 (6 sequences)

| arm | sequences | median APE / KISS | geo-mean APE / KISS | LiDAR-only wins | failures (> 5 m) | mean rank* |
|---|---|---|---|---|---|---|
| KISS-SLAM | 6 | 1.00 | 1.00 | 0 | 1 | 11.5 |
| KISS no deskew | 6 | 0.58 | 0.49 | 0 | 0 | 9.8 |
| GenZ-ICP | 6 | 0.43 | 0.18 | 1 | 0 | 4.5 |
| MAD-ICP | 6 | 0.44 | 0.42 | 1 | 0 | 7.3 |
| DLO | 6 | 0.78 | 0.32 | 0 | 0 | 8.5 |
| CT-ICP | 6 | 0.58 | 0.21 | 1 | 0 | 5.3 |
| Traj-LO | 6 | 0.45 | 0.78 | 2 | 1 | 4.0 |
| FAST-LIO2 (IMU) | 6 | 0.35 | 0.14 | – | 0 | – |
| Ours #081 (paper so far) | 6 | 0.67 | 0.25 | 0 | 0 | 6.7 |
| Ours default | 6 | 0.61 | 0.24 | 0 | 0 | 6.3 |
| + blend (B) | 6 | 0.57 | 0.24 | 0 | 0 | 5.7 |
| + blend + fallback (A) | 6 | 0.62 | 0.24 | 0 | 0 | 7.2 |
| + blend + sectors | 6 | 0.62 | 0.23 | 1 | 0 | 6.8 |
| + blend + fallback after 4 (C) | 6 | 0.57 | 0.24 | 0 | 0 | 7.3 |

*mean rank among the 13 LiDAR-only arms run on all 6 sequences: KISS-SLAM, KISS no deskew, GenZ-ICP, MAD-ICP, DLO, CT-ICP, Traj-LO, Ours #081 (paper so far), Ours default, + blend (B), + blend + fallback (A), + blend + sectors, + blend + fallback after 4 (C)

#### NTU (eee) (3 sequences)

| arm | sequences | median APE / KISS | geo-mean APE / KISS | LiDAR-only wins | failures (> 5 m) | mean rank* |
|---|---|---|---|---|---|---|
| KISS-SLAM | 3 | 1.00 | 1.00 | 0 | 0 | 12.3 |
| KISS no deskew | 3 | 0.97 | 0.95 | 0 | 0 | 12.0 |
| GenZ-ICP | 3 | 0.60 | 0.42 | 0 | 0 | 7.3 |
| MAD-ICP | 3 | 0.86 | 1.11 | 0 | 0 | 10.0 |
| DLO | 3 | 0.10 | 0.13 | 0 | 0 | 3.3 |
| CT-ICP | 3 | 0.09 | 0.12 | 0 | 0 | 4.0 |
| Traj-LO | 3 | 0.05 | 0.06 | 3 | 0 | 1.0 |
| FAST-LIO2 (IMU) | 3 | 0.05 | 0.06 | – | 0 | – |
| Ours #081 (paper so far) | 3 | 0.46 | 0.49 | 0 | 0 | 8.0 |
| Ours default | 3 | 0.55 | 0.51 | 0 | 0 | 8.0 |
| + blend (B) | 3 | 0.53 | 0.37 | 0 | 0 | 5.3 |
| + blend + fallback (A) | 3 | 0.54 | 0.36 | 0 | 0 | 4.3 |
| + blend + sectors | 3 | 0.55 | 0.47 | 0 | 0 | 8.0 |
| + blend + fallback after 4 (C) | 3 | 0.55 | 0.38 | 0 | 0 | 7.3 |

*mean rank among the 13 LiDAR-only arms run on all 3 sequences: KISS-SLAM, KISS no deskew, GenZ-ICP, MAD-ICP, DLO, CT-ICP, Traj-LO, Ours #081 (paper so far), Ours default, + blend (B), + blend + fallback (A), + blend + sectors, + blend + fallback after 4 (C)

#### NTU (15 new) (15 sequences)

| arm | sequences | median APE / KISS | geo-mean APE / KISS | LiDAR-only wins | failures (> 5 m) | mean rank* |
|---|---|---|---|---|---|---|
| KISS-SLAM | 15 | 1.00 | 1.00 | 1 | 3 | 4.3 |
| Ours default | 15 | 0.65 | 0.52 | 6 | 3 | 2.7 |
| + blend (B) | 15 | 0.51 | 0.44 | 2 | 3 | 2.5 |
| + blend + fallback (A) | 15 | 0.47 | 0.31 | 5 | 2 | 2.7 |
| + blend + fallback after 4 (C) | 15 | 0.49 | 0.32 | 1 | 2 | 2.7 |

*mean rank among the 5 LiDAR-only arms run on all 15 sequences: KISS-SLAM, Ours default, + blend (B), + blend + fallback (A), + blend + fallback after 4 (C)

#### Car (Boreas) (1 sequences)

| arm | sequences | median APE / KISS | geo-mean APE / KISS | LiDAR-only wins | failures (> 5 m) | mean rank* |
|---|---|---|---|---|---|---|
| KISS-SLAM | 1 | 1.00 | 1.00 | 0 | 0 | 6.0 |
| KISS no deskew | 1 | 26.79 | 26.79 | 0 | 1 | 8.0 |
| Ours #081 (paper so far) | 1 | 1.03 | 1.03 | 0 | 0 | 7.0 |
| Ours default | 1 | 0.88 | 0.88 | 0 | 0 | 5.0 |
| + blend (B) | 1 | 0.73 | 0.73 | 1 | 0 | 1.0 |
| + blend + fallback (A) | 1 | 0.73 | 0.73 | 0 | 0 | 3.0 |
| + blend + sectors | 1 | 0.77 | 0.77 | 0 | 0 | 4.0 |
| + blend + fallback after 4 (C) | 1 | 0.73 | 0.73 | 0 | 0 | 2.0 |

*mean rank among the 8 LiDAR-only arms run on all 1 sequences: KISS-SLAM, KISS no deskew, Ours #081 (paper so far), Ours default, + blend (B), + blend + fallback (A), + blend + sectors, + blend + fallback after 4 (C)


### RPE 1 m (NCD + Oxford Spires + Boreas, translation cm / rotation °), median over sequences

| arm | sequences | median RPE t | median RPE r | median RPE t / KISS | median RPE r / KISS |
|---|---|---|---|---|---|
| KISS-SLAM | 17 | 21.33 | 2.727 | 1.00 | 1.00 |
| KISS no deskew | 17 | 10.06 | 0.972 | 0.47 | 0.34 |
| GenZ-ICP | 16 | 7.91 | 0.651 | 0.35 | 0.22 |
| MAD-ICP | 15 | 5.19 | 0.654 | 0.24 | 0.20 |
| DLO | 15 | 6.55 | 0.615 | 0.29 | 0.21 |
| CT-ICP | 16 | 4.78 | 0.670 | 0.30 | 0.25 |
| Traj-LO | 14 | 2.36 | 0.381 | 0.11 | 0.13 |
| FAST-LIO2 (IMU) | 15 | 2.12 | 0.377 | 0.14 | 0.14 |
| COIN-LIO (IMU) | 8 | 2.54 | 0.430 | 0.17 | 0.14 |
| Ours #081 (paper so far) | 17 | 6.43 | 1.022 | 0.39 | 0.37 |
| Ours default | 17 | 6.24 | 0.923 | 0.36 | 0.36 |
| + blend (B) | 17 | 6.19 | 0.876 | 0.35 | 0.36 |
| + blend + fallback (A) | 17 | 6.19 | 0.878 | 0.35 | 0.36 |
| + blend + sectors | 17 | 6.23 | 0.850 | 0.35 | 0.36 |
| + blend + fallback after 4 (C) | 17 | 6.19 | 0.876 | 0.35 | 0.36 |

**Summary (⏳, #162 / #164):** method C never fails where KISS-SLAM does (except spms, the drone at altitude where every arm fails) and roughly halves KISS-SLAM's
error (NCD ×0.41, Spires ×0.16, Hilti ×0.24, NTU eee ×0.38, NTU new ×0.32, Boreas ×0.73, geometric mean of APE / KISS). Against other LiDAR-only methods:
on par with GenZ-ICP / CT-ICP, better than DLO / MAD-ICP on most, **Traj-LO more accurate almost everywhere** (except Hilti, where it diverges once).
The claim is robustness and a large improvement of KISS-SLAM, not the best accuracy.

---

## 2. Oxford Spires — the 7 further sequences with ground truth (8/10, #228 / #229)

Hesai QT64, hand-held; GT `gt-tum.txt` of the dataset (`docs/datasets.md`). Ours 4 seeds; KISS-SLAM 1 run.

| sequence (GT path) | KISS-SLAM APE m · RTE % · RPE 1 m cm · path m | C APE m · RTE % · RPE 1 m cm · path m |
|---|---|---|
| keble-college-02 (294) | 0.376 · 1.31 · 39.8 · 655 | **0.091 ± 0.002 · 0.47 ± 0.02 · 7.9** · 351 |
| keble-college-04 (783) | 1.414 · 1.83 · 93.3 · 2617 | **0.088 ± 0.005 · 0.30 ± 0.01 · 7.7** · 935 |
| keble-college-05 (706) | 1.041 · 1.93 · 78.3 · 2172 | **0.174 ± 0.018 · 0.41 ± 0.02 · 9.5** · 869 |
| observatory-quarter-02 (393) | **0.221** · 0.70 · 24.5 · 616 | 0.248 ± 0.071 · **0.47 ± 0.07 · 6.0** · 420 |
| blenheim-palace-01 (455) | 0.576 · 1.17 · 36.0 · 953 | **0.192 ± 0.019 · 0.55 ± 0.02 · 10.1** · 578 |
| blenheim-palace-05 (386) | 3.285 · 2.86 · 57.7 · 1164 | **0.335 ± 0.016 · 0.73 ± 0.01 · 9.9** · 517 |
| christ-church-05 (816) | 1.721 · 0.85 · 24.0 · 1537 | **0.270 ± 0.025 · 0.27 ± 0.01 · 4.6** · 892 |
| geometric mean C / KISS | | APE ×0.20 · RTE ×0.32 · RPE ×0.17 |

**Summary (⏳):** confirms the first 6: KISS-SLAM loses the hand-held Hesai (path 1.6–3.3× the GT), C does not (+7–23 %, the known overestimate); only
observatory-quarter-02 is an APE tie. Oxford Spires total for the paper: 13 sequences.

---

## 3. Hilti 2021 (12) and 2022 (16) — official protocols, all methods (9/10, #238–#241)

2021: Ouster OS0-64, `evaluate_hilti.py` (pole / prism / imu, 1 s, SE(3)). 2022: Hesai PandarXT-32, `evaluate_hilti2022.py` (measurement tip, 2 s, SE(3);
score 0–100 = points per control point 10 / 6 / 3 / 1 / 0 for < 1 / 3 / 6 / 10 cm / more), identical to the official script to 1e-9. C 4 seeds (`ral239`),
KISS-SLAM 1 run (`kiss239`), the others one run each with the authors' configurations (GenZ-ICP `indoor.yaml`, MAD-ICP `hilti_2021.cfg` / default,
CT-ICP `robust_low_inertia`, Traj-LO `config_ouster` / `config_hesai` — Traj-LO stays with its published configuration, decision M.T. 9/10).
Failures that repeated when re-run alone: GenZ-ICP exp02 / exp06 (out of memory, 56 GB), MAD-ICP exp23 (segfault), Traj-LO exp06 / exp23 (out of memory).
exp23: the last control point is ~77 s after the published bags (completeness 0.94 for every method).

### Full table (generated by `scripts/hilti_table.py`, APE rmse m; "dense" = against the dense IMU trajectory of exp14 / 16 / 18)

| year | sequence | arm | runs | APE rmse m | score /100 | completeness | dense IMU rmse m |
|---|---|---|---|---|---|---|---|
| 2021 | Basement_1 | kiss239 | 1 | 0.055 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_1 | ral239 | 4 | 0.046 ± 0.010 | nan | 1.00 | nan |
| 2021 | Basement_1 | cticp240 | 1 | 0.066 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_1 | genz240 | 1 | 0.069 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_1 | mad240 | 1 | 0.106 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_1 | trajlo240 | 1 | 0.038 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_3 | kiss239 | 1 | 0.076 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_3 | ral239 | 4 | 0.083 ± 0.002 | nan | 1.00 | nan |
| 2021 | Basement_3 | cticp240 | 1 | 0.054 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_3 | genz240 | 1 | 0.050 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_3 | mad240 | 1 | 0.070 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_3 | trajlo240 | 1 | 0.048 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_4 | kiss239 | 1 | 0.072 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_4 | ral239 | 4 | 0.079 ± 0.008 | nan | 1.00 | nan |
| 2021 | Basement_4 | cticp240 | 1 | 0.052 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_4 | genz240 | 1 | 0.061 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_4 | mad240 | 1 | 0.065 ± 0.000 | nan | 1.00 | nan |
| 2021 | Basement_4 | trajlo240 | 1 | 0.065 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_1 | kiss239 | 1 | 0.080 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_1 | ral239 | 4 | 0.062 ± 0.006 | nan | 1.00 | nan |
| 2021 | Campus_1 | cticp240 | 1 | 0.063 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_1 | genz240 | 1 | 0.038 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_1 | mad240 | 1 | 0.252 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_1 | trajlo240 | 1 | 0.057 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_2 | kiss239 | 1 | 0.054 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_2 | ral239 | 4 | 0.054 ± 0.003 | nan | 1.00 | nan |
| 2021 | Campus_2 | cticp240 | 1 | 0.817 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_2 | genz240 | 1 | 0.055 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_2 | mad240 | 1 | 0.150 ± 0.000 | nan | 1.00 | nan |
| 2021 | Campus_2 | trajlo240 | 1 | 0.037 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_1 | kiss239 | 1 | 0.063 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_1 | ral239 | 4 | 0.037 ± 0.002 | nan | 1.00 | nan |
| 2021 | Construction_Site_1 | cticp240 | 1 | 0.034 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_1 | genz240 | 1 | 0.032 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_1 | mad240 | 1 | 0.168 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_1 | trajlo240 | 1 | 0.023 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_2 | kiss239 | 1 | 2.652 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_2 | ral239 | 4 | 0.096 ± 0.010 | nan | 1.00 | nan |
| 2021 | Construction_Site_2 | cticp240 | 1 | 0.067 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_2 | genz240 | 1 | 0.066 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_2 | mad240 | 1 | 0.410 ± 0.000 | nan | 1.00 | nan |
| 2021 | Construction_Site_2 | trajlo240 | 1 | 0.061 ± 0.000 | nan | 1.00 | nan |
| 2021 | IC_Office_1 | kiss239 | 1 | 6.344 ± 0.000 | nan | 1.00 | nan |
| 2021 | IC_Office_1 | ral239 | 4 | 0.074 ± 0.006 | nan | 1.00 | nan |
| 2021 | IC_Office_1 | cticp240 | 1 | 0.061 ± 0.000 | nan | 1.00 | nan |
| 2021 | IC_Office_1 | genz240 | 1 | 0.069 ± 0.000 | nan | 1.00 | nan |
| 2021 | IC_Office_1 | mad240 | 1 | 0.941 ± 0.000 | nan | 1.00 | nan |
| 2021 | IC_Office_1 | trajlo240 | 1 | 0.060 ± 0.000 | nan | 1.00 | nan |
| 2021 | LAB_Survey_2 | kiss239 | 1 | 0.062 ± 0.000 | nan | 0.01 | nan |
| 2021 | LAB_Survey_2 | ral239 | 4 | 0.035 ± 0.000 | nan | 0.01 | nan |
| 2021 | LAB_Survey_2 | cticp240 | 1 | 0.038 ± 0.000 | nan | 0.01 | nan |
| 2021 | LAB_Survey_2 | genz240 | 1 | 0.036 ± 0.000 | nan | 0.01 | nan |
| 2021 | LAB_Survey_2 | mad240 | 1 | 0.035 ± 0.000 | nan | 0.01 | nan |
| 2021 | LAB_Survey_2 | trajlo240 | 1 | 0.026 ± 0.000 | nan | 0.03 | nan |
| 2021 | Office_Mitte_1 | kiss239 | 1 | 4.286 ± 0.000 | nan | 1.00 | nan |
| 2021 | Office_Mitte_1 | ral239 | 4 | 0.248 ± 0.063 | nan | 1.00 | nan |
| 2021 | Office_Mitte_1 | cticp240 | 1 | 0.126 ± 0.000 | nan | 1.00 | nan |
| 2021 | Office_Mitte_1 | genz240 | 1 | 0.117 ± 0.000 | nan | 1.00 | nan |
| 2021 | Office_Mitte_1 | mad240 | 1 | 0.176 ± 0.000 | nan | 1.00 | nan |
| 2021 | Office_Mitte_1 | trajlo240 | 1 | 20553.878 ± 0.000 | nan | 1.00 | nan |
| 2021 | Parking_1 | kiss239 | 1 | 43.160 ± 0.000 | nan | 1.00 | nan |
| 2021 | Parking_1 | ral239 | 4 | 29.094 ± 8.712 | nan | 1.00 | nan |
| 2021 | Parking_1 | cticp240 | 1 | 26.286 ± 0.000 | nan | 1.00 | nan |
| 2021 | Parking_1 | genz240 | 1 | 24.433 ± 0.000 | nan | 1.00 | nan |
| 2021 | Parking_1 | mad240 | 1 | 39.497 ± 0.000 | nan | 1.00 | nan |
| 2021 | Parking_1 | trajlo240 | 1 | 32.299 ± 0.000 | nan | 1.00 | nan |
| 2021 | UZH_Tracking_Area_Run_2 | kiss239 | 1 | 0.585 ± 0.000 | nan | 0.08 | nan |
| 2021 | UZH_Tracking_Area_Run_2 | ral239 | 4 | 0.576 ± 0.002 | nan | 0.08 | nan |
| 2021 | UZH_Tracking_Area_Run_2 | cticp240 | 1 | 0.498 ± 0.000 | nan | 0.08 | nan |
| 2021 | UZH_Tracking_Area_Run_2 | genz240 | 1 | 0.197 ± 0.000 | nan | 0.08 | nan |
| 2021 | UZH_Tracking_Area_Run_2 | mad240 | 1 | 0.188 ± 0.000 | nan | 0.08 | nan |
| 2021 | UZH_Tracking_Area_Run_2 | trajlo240 | 1 | 0.270 ± 0.000 | nan | 0.21 | nan |
| 2022 | exp01_construction_ground_level | kiss239 | 1 | 0.035 ± 0.000 | 41.5 | 1.00 | nan |
| 2022 | exp01_construction_ground_level | ral239 | 4 | 0.030 ± 0.004 | 52.3 | 1.00 | nan |
| 2022 | exp01_construction_ground_level | cticp240 | 1 | 26.539 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp01_construction_ground_level | genz240 | 1 | 1.148 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp01_construction_ground_level | mad240 | 1 | 19.714 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp01_construction_ground_level | trajlo240 | 1 | 0.031 ± 0.000 | 51.5 | 1.00 | nan |
| 2022 | exp02_construction_multilevel | kiss239 | 1 | 28.856 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp02_construction_multilevel | ral239 | 4 | 6.962 ± 2.230 | 0.0 | 1.00 | nan |
| 2022 | exp02_construction_multilevel | cticp240 | 1 | 43.464 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp02_construction_multilevel | mad240 | 1 | 22.504 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp02_construction_multilevel | trajlo240 | 1 | 16174579.246 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp03_construction_stairs | kiss239 | 1 | 9.739 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp03_construction_stairs | ral239 | 4 | 4.501 ± 0.802 | 0.0 | 1.00 | nan |
| 2022 | exp03_construction_stairs | cticp240 | 1 | 17.424 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp03_construction_stairs | genz240 | 1 | 1.678 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp03_construction_stairs | mad240 | 1 | 1.264 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp03_construction_stairs | trajlo240 | 1 | 257760.325 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp04_construction_upper_level | kiss239 | 1 | 1.759 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp04_construction_upper_level | ral239 | 4 | 1.455 ± 0.329 | 0.0 | 1.00 | nan |
| 2022 | exp04_construction_upper_level | cticp240 | 1 | 16.491 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp04_construction_upper_level | genz240 | 1 | 1.717 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp04_construction_upper_level | mad240 | 1 | 4.657 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp04_construction_upper_level | trajlo240 | 1 | 0.784 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp05_construction_upper_level_2 | kiss239 | 1 | 0.701 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp05_construction_upper_level_2 | ral239 | 4 | 0.802 ± 0.525 | 10.0 | 1.00 | nan |
| 2022 | exp05_construction_upper_level_2 | cticp240 | 1 | 1.220 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp05_construction_upper_level_2 | genz240 | 1 | 1.265 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp05_construction_upper_level_2 | mad240 | 1 | 1.420 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp05_construction_upper_level_2 | trajlo240 | 1 | 1.578 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp06_construction_upper_level_3 | kiss239 | 1 | 8.189 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp06_construction_upper_level_3 | ral239 | 4 | 4.612 ± 1.613 | 0.0 | 1.00 | nan |
| 2022 | exp06_construction_upper_level_3 | cticp240 | 1 | 7.627 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp06_construction_upper_level_3 | mad240 | 1 | 5.540 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp07_long_corridor | kiss239 | 1 | 40.518 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp07_long_corridor | ral239 | 4 | 3.130 ± 1.403 | 0.0 | 1.00 | nan |
| 2022 | exp07_long_corridor | cticp240 | 1 | 30.140 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp07_long_corridor | genz240 | 1 | 0.053 ± 0.000 | 26.7 | 1.00 | nan |
| 2022 | exp07_long_corridor | mad240 | 1 | 19.135 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp07_long_corridor | trajlo240 | 1 | 26.962 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp09_cupola | kiss239 | 1 | 56.556 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp09_cupola | ral239 | 4 | 8.454 ± 0.905 | 0.0 | 1.00 | nan |
| 2022 | exp09_cupola | cticp240 | 1 | 11.900 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp09_cupola | genz240 | 1 | 2708.188 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp09_cupola | mad240 | 1 | 3.347 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp09_cupola | trajlo240 | 1 | 55458.210 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp10_cupola_2 | kiss239 | 1 | 124.385 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp10_cupola_2 | ral239 | 4 | 10.300 ± 1.083 | 0.0 | 1.00 | nan |
| 2022 | exp10_cupola_2 | cticp240 | 1 | 12.735 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp10_cupola_2 | genz240 | 1 | 11.812 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp10_cupola_2 | mad240 | 1 | 5.807 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp10_cupola_2 | trajlo240 | 1 | 59238.097 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp11_lower_gallery | kiss239 | 1 | 6.933 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp11_lower_gallery | ral239 | 4 | 2.069 ± 0.320 | 0.0 | 1.00 | nan |
| 2022 | exp11_lower_gallery | cticp240 | 1 | 10.768 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp11_lower_gallery | genz240 | 1 | 0.964 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp11_lower_gallery | mad240 | 1 | 0.960 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp11_lower_gallery | trajlo240 | 1 | 8.794 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp14_basement_2 | kiss239 | 1 | 3.672 ± 0.000 | 0.0 | 1.00 | 3.380 |
| 2022 | exp14_basement_2 | ral239 | 4 | 1.913 ± 0.373 | 0.0 | 1.00 | 1.629 |
| 2022 | exp14_basement_2 | cticp240 | 1 | 4.666 ± 0.000 | 0.0 | 1.00 | 5.900 |
| 2022 | exp14_basement_2 | genz240 | 1 | 0.053 ± 0.000 | 32.5 | 1.00 | 0.108 |
| 2022 | exp14_basement_2 | mad240 | 1 | 0.131 ± 0.000 | 0.0 | 1.00 | 0.162 |
| 2022 | exp14_basement_2 | trajlo240 | 1 | 0.291 ± 0.000 | 0.0 | 1.00 | 0.349 |
| 2022 | exp15_attic_to_upper_gallery | kiss239 | 1 | 7.121 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp15_attic_to_upper_gallery | ral239 | 4 | 4.370 ± 0.583 | 0.0 | 1.00 | nan |
| 2022 | exp15_attic_to_upper_gallery | cticp240 | 1 | 4.634 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp15_attic_to_upper_gallery | genz240 | 1 | 4.429 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp15_attic_to_upper_gallery | mad240 | 1 | 1.112 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp15_attic_to_upper_gallery | trajlo240 | 1 | 76913.959 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp16_attic_to_upper_gallery_2 | kiss239 | 1 | 8.448 ± 0.000 | 0.0 | 1.00 | 11.362 |
| 2022 | exp16_attic_to_upper_gallery_2 | ral239 | 4 | 5.496 ± 0.410 | 0.0 | 1.00 | 4.751 |
| 2022 | exp16_attic_to_upper_gallery_2 | cticp240 | 1 | 4.474 ± 0.000 | 0.0 | 1.00 | 3.926 |
| 2022 | exp16_attic_to_upper_gallery_2 | genz240 | 1 | 7.171 ± 0.000 | 0.0 | 1.00 | 8.240 |
| 2022 | exp16_attic_to_upper_gallery_2 | mad240 | 1 | 1.101 ± 0.000 | 0.0 | 1.00 | 1.091 |
| 2022 | exp16_attic_to_upper_gallery_2 | trajlo240 | 1 | 243269.707 ± 0.000 | 0.0 | 1.00 | 234383.221 |
| 2022 | exp18_corridor_lower_gallery_2 | kiss239 | 1 | 3.339 ± 0.000 | 0.0 | 1.00 | 6.083 |
| 2022 | exp18_corridor_lower_gallery_2 | ral239 | 4 | 3.228 ± 1.164 | 0.0 | 1.00 | 1.220 |
| 2022 | exp18_corridor_lower_gallery_2 | cticp240 | 1 | 3.560 ± 0.000 | 0.0 | 1.00 | 8.310 |
| 2022 | exp18_corridor_lower_gallery_2 | genz240 | 1 | 1.300 ± 0.000 | 0.0 | 1.00 | 0.584 |
| 2022 | exp18_corridor_lower_gallery_2 | mad240 | 1 | 1.903 ± 0.000 | 0.0 | 1.00 | 5.640 |
| 2022 | exp18_corridor_lower_gallery_2 | trajlo240 | 1 | 16143.452 ± 0.000 | 0.0 | 1.00 | 3581983723413917540154343424.000 |
| 2022 | exp21_outside_building | kiss239 | 1 | 1.524 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp21_outside_building | ral239 | 4 | 0.326 ± 0.082 | 0.5 | 1.00 | nan |
| 2022 | exp21_outside_building | cticp240 | 1 | 2.315 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp21_outside_building | genz240 | 1 | 4.632 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp21_outside_building | mad240 | 1 | 0.294 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp21_outside_building | trajlo240 | 1 | 0.202 ± 0.000 | 0.0 | 1.00 | nan |
| 2022 | exp23_the_sheldonian_slam | kiss239 | 1 | 81.281 ± 0.000 | 0.0 | 0.94 | nan |
| 2022 | exp23_the_sheldonian_slam | ral239 | 4 | 17.144 ± 4.105 | 0.0 | 0.94 | nan |
| 2022 | exp23_the_sheldonian_slam | cticp240 | 1 | 15.286 ± 0.000 | 0.0 | 0.94 | nan |
| 2022 | exp23_the_sheldonian_slam | genz240 | 1 | 6.173 ± 0.000 | 0.0 | 0.94 | nan |

### Per year

| | geometric-mean APE (sequences all methods complete) | mean rank | best on |
|---|---|---|---|
| **2021** (12, #241) | GenZ-ICP **0.102** · C 0.136 · CT-ICP 0.145 · MAD-ICP 0.248 · Traj-LO 0.261 · KISS-SLAM 0.381 m | Traj-LO 2.2 · GenZ 2.4 · CT 3.2 · C 3.8 · MAD 4.3 · KISS 5.0 | Traj-LO 7, GenZ 3, CT 1, MAD 1 |
| **2022** (13 common, #240) | **C 1.90** · MAD-ICP 1.93 · GenZ-ICP 2.36 · KISS-SLAM 4.9 · CT-ICP 7.75 · Traj-LO 174 m | **C 2.3 · MAD 2.3** · GenZ 2.9 · KISS 4.1 · CT 4.3 · Traj-LO 4.5 | MAD 6, GenZ 4, C 3, Traj-LO 2, KISS 1 |

C vs KISS-SLAM: 2022 APE ×0.37 (better on 15 / 16), 2021 ×0.36 (12) — closes every KISS-SLAM failure (IC_Office_1 6.34 → 0.07, Office_Mitte_1 4.29 → 0.25,
Construction_Site_2 2.65 → 0.10 m).

**Summary (⏳):** Hilti 2022 (hard hand-held motion, 32-beam Hesai, stairs / corridors / cupolas) is where LiDAR-only odometry generally fails: C has the lowest
average error and is the only method without a collapse (worst 17 m on the 974 s exp23), but all methods stay at metres (challenge score ~0 except exp01:
C 52 / KISS 41.5). MAD-ICP / GenZ-ICP win more single sequences and fail elsewhere; GenZ-ICP reaches 5 cm in the corridors exp07 / exp14 (open: why, must-do
M.T. 9/10). Hilti 2021 (gentle motion, good geometry): every recent method reaches 2–8 cm; GenZ-ICP / Traj-LO are the most accurate, C third.

---

## 4. Car (2/10–8/10)

### 4.1 Boreas — 8 full drives, uncorrected, all methods (8/10, #216, #220, #225)

Velodyne Alpha Prime 128, ~8 km each; GT per scan (applanix); no elevation correction for any method (decision M.T. 7/10). One run per method.
"car config, fast" = the car configuration (blend start + translation trigger) on the real-time stack (options, §6). Odometry only = loop closures removed
(`replay_backend.py none`); the other methods have no closures.

| drive | SLAM APE m · RTE %: KISS-SLAM | C | car config, fast | odometry RTE %: KISS | C | car | CT-ICP | MAD-ICP | GenZ-ICP | odometry APE m: KISS | C | car | CT-ICP | MAD-ICP |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2020-12-01-13-26 | 3.83 · 0.48 | 6.20 · 0.52 | 4.32 · 0.49 | 0.39 | 0.36 | 0.36 | 0.36 | 0.42 | 2.74 | 18.46 | 17.75 | 17.70 | 25.66 | 31.71 |
| 2021-01-15-12-17 | 4.28 · 0.43 | 4.53 · 0.40 | 4.43 · 0.41 | 0.39 | 0.36 | 0.36 | 0.32 | 0.44 | 2.62 | 18.39 | 18.59 | 18.28 | 26.37 | 44.19 |
| 2021-01-19-15-08 | 4.57 · 0.49 | 4.97 · 0.46 | 4.74 · 0.46 | 0.38 | 0.34 | 0.35 | 0.31 | 0.40 | 2.72 | 17.65 | 17.74 | 17.39 | 25.22 | 29.47 |
| 2021-04-08-12-44 | 9.05 · 0.60 | 8.61 · 0.57 | 8.30 · 0.56 | 0.46 | 0.43 | 0.43 | 0.39 | 0.49 | 2.76 | 15.91 | 16.34 | 16.13 | 24.80 | 32.62 |
| 2021-09-07-09-35 | 6.07 · 0.64 | 6.66 · 0.55 | 6.34 · 0.57 | 0.47 | 0.44 | 0.43 | 0.41 | 0.50 | 2.61 | 12.56 | 11.42 | 11.38 | 25.02 | 25.98 |
| 2021-09-14-20-00 | 6.95 · 0.62 | 5.87 · 0.56 | 6.19 · 0.52 | 0.51 | 0.49 | 0.48 | 0.44 | 0.54 | 2.50 | 12.85 | 12.11 | 11.59 | 25.11 | 30.32 |
| 2021-10-15-12-35 | 5.28 · 0.84 | 10.58 · 0.65 | 6.84 · 0.57 | 0.62 | 0.44 | 0.43 | 0.40 | 0.51 | 2.61 | 16.07 | 13.74 | 13.46 | 27.59 | 30.58 |
| 2021-11-16-14-10 | 7.61 · 0.57 | 7.90 · 0.55 | 7.90 · 0.56 | 0.41 | 0.38 | 0.38 | 0.34 | 0.43 | 2.83 | 17.70 | 17.04 | 17.30 | 26.61 | 35.93 |
| **mean** | 5.96 · 0.584 | 6.92 · 0.533 | 6.13 · **0.518** | 0.454 | 0.405 | 0.403 | **0.371** | 0.466 | 2.67 | 16.20 | 15.59 | **15.40** | 25.80 | 32.60 |

Boreas, first 3000 scans of 2021-01-26-11-22 (§1): KISS-SLAM 0.266, C 0.195 m. Height drift on Boreas is a ~+0.1° beam-elevation calibration bias common to
all methods (#218 / #219 / #221: odometry z ×0.49 and RTE ×0.70 with `--elev-offset=0.1`, not used for the paper).

**Summary (⏳):** odometry better than KISS-SLAM on 8 / 8 drives (RTE 0.40 / 0.45 %) and better than MAD-ICP / GenZ-ICP; **CT-ICP drifts less** (0.37 %, 8 / 8)
but has a constant ~25 m APE (open); best odometry APE of all. After loop closures KISS-SLAM keeps the best APE (5.96 vs 6.13 m car config). RPE of methods
without deskew is not comparable (pose-time convention, #085 / #220).

### 4.2 MulRan — 4 sequences (7/10, #208–#217, #223, #224, #227)

Ouster OS1-64 on a car; GT `global_pose.csv` → LiDAR frame (`scripts/mulran_gt.py`). C / car config 4 seeds; others 1 run. RTE %:

| sequence | KISS-SLAM | C | car config | CT-ICP (odo) | GenZ-ICP (odo) | MAD-ICP (odo) |
|---|---|---|---|---|---|---|
| KAIST01 (SLAM / odometry) | 2.57 / 2.59 | **2.47 ± 0.09** / 2.54 | **2.47 ± 0.07** / 2.54 | 2.42 | 2.42 | 2.89 |
| DCC01 | 2.71 / 2.95 | **2.65 ± 0.01** / 2.88 | 2.66 ± 0.01 / 2.89 | 2.89 | 2.80 | 3.22 |
| Riverside01 | **3.57** / 3.77 | 9.98 ± 1.59 / 5.20 | 3.64 ± 0.21 / 3.79 | 3.02 | 3.68 | 4.09 |
| Sejong01 | **4.27** / 4.49 | 8.33 ± 0.05 / 8.50 | 5.38 ± 0.06 / 5.59 | 4.56 | 5.24 | 5.29 |

SLAM APE m: KAIST01 KISS 3.49 / C 3.08 / car 3.14 · DCC01 5.51 / 5.46 / 5.48 · Riverside01 6.41 / 36.42 / 6.79 · Sejong01 165.4 / 261.7 / 174.1.

**Summary (⏳):** on the highway sequences C fails (road markings that move with the car make the image translation ~0, #212); the car configuration
(option) brings Riverside01 to KISS-SLAM level, Sejong01 stays behind — one 22 s stall (#227, **open problem**, decision M.T. 8/10). City sequences: C and
car configuration slightly better than KISS-SLAM.

### 4.3 KITTI 07 (2/10, #084, #085, #207)

The KITTI scans are already motion compensated (#085, sweep-seam test) — no deskew test possible; data deleted 8/10. Locked method C on raw drive 0027:
RTE 0.40 % (KISS-SLAM 0.42, method of 29/9 0.65) — sanity check only.

---

## 5. Car configuration on hand-held / drone (7/10–8/10, #222, #226)

| sequence | KISS-SLAM | C (4 seeds) | car config (s0) | C + blend start only (s0) | C + trigger only (s0) |
|---|---|---|---|---|---|
| christ-church-03 APE m · RTE % | 0.122 · 0.46 | 0.039 ± 0.001 · 0.13 | 0.038 · 0.13 | – | – |
| quad_hard | 0.163 · 0.92 | 0.234 ± 0.029 · 0.91 ± 0.10 | 0.194 · 0.76 | – | – |
| eee_01 ATE m | 2.678 | 1.601 ± 0.036 | 1.962 | – | – |
| eee_02 | 1.486 | 0.812 ± 0.082 | 1.204 | – | – |
| eee_03 | 0.864 | 0.145 ± 0.002 | 0.642 | – | – |
| sbs_01 | 0.976 | 0.370 ± 0.024 | 0.413 | – | – |
| sbs_02 | 1.143 | 0.723 ± 0.290 | 0.844 | – | – |
| rtp_01 | 3.985 | 0.249 ± 0.056 | 3.355 | 3.579 | **0.244** |
| rtp_02 | 3.575 | 0.337 ± 0.016 | 3.355 | – | – |

**Summary (⏳):** the car configuration is neutral on hand-held data but ruins the drone; the culprit is the ICP start from the blend, the translation trigger
alone is harmless (rtp_01 0.244 m). Decision M.T. 7/10: car configuration = option for vehicles only.

---

## 6. Runtime (5/10–6/10, #165, #166, #170–#202)

Wall clock, the method alone on one machine (48 cores), seed 0, no thread budget (decision M.T. 5/10).

| sequence (sensor) | method C, serial | C `--parallel` | after the speed fixes (no result change, #170 / #174 / #176) | real-time stack (option, #202) |
|---|---|---|---|---|
| christ-church-03 (Hesai 64, 10 Hz) | 11.4 Hz | 12.9 Hz | – | – |
| quad_easy (Ouster 128, 10 Hz) | – | 10.3 Hz | 1.11× real time | – |
| Boreas 3000 scans (Velodyne 128, 10 Hz) | 3.9 Hz (764 s) | 4.9 Hz (616 s) | 404 s (0.75×) | **280.9 s for 300 s (1.07×, 11 Hz)** |

Real-time stack = panorama ×4 on 128 beams + image stage in C++ + image 4 scans ahead + range cache + KISS-ICP without the GIL (env `kiss-slam-gil`);
odometry on the 8 Boreas drives identical to C (RTE 0.403 / 0.405 %, §4.1). KISS-SLAM alone on Boreas: 237 s. Memory 1.1–1.2 GB hand-held, 2.4–2.7 GB car.
Rejected for speed: ORB (worse on hand-held), AKAZE (#181–#185), MAGSAC++ / GNC (slower, #236).

---

## 7. Solid-state LiDARs — Livox (2/10–3/10, #095–#116)

TIERS (Avia, Horizon), Mid-360 (Hard Point Cloud Localization). **Summary (⏳):** no realistic setting beats KISS-SLAM on trajectory: the intensity image of one
non-repetitive scan is too sparse; with an exact intra-sweep motion (oracle) the trajectory improves, so the source of motion is the problem, not the deskew.
First positive signal: the rosette's own self-motion on the Avia (#116, rotation median 0.41° vs KISS 0.70°). Out of the paper's scope.

---

## 8. Map sharpness against survey maps (19/9, #048, #049, #051)

Point-to-plane distance of each run's map to the survey map (New College 5 cm; christ-church-03 / -02 the Oxford Spires TLS map), 1 run per arm, the
method of that date ("i3" = image motion). Lower = sharper.

### Deskew only (scans at the ground-truth poses)

| Sequence | Arm | median [cm] | mean [cm] | < 5 cm [%] | < 10 cm [%] | within 0.5 m [%] |
|---|---|---|---|---|---|---|
| quad_easy | KISS-SLAM, no deskew | **2.73** | 4.21 | **71.8** | 90.4 | 99.5 |
|  | KISS-SLAM | 3.14 | 5.18 | 65.7 | 85.1 | 99.3 |
|  | KISS-SLAM, indoor_detail | 3.23 | 5.33 | 64.5 | 84.4 | 99.2 |
|  | i3 + SIFT | 2.88 | 4.72 | 69.2 | 87.5 | 99.3 |
|  | i3 + SURF | 2.77 | 4.45 | 70.9 | 88.9 | 99.5 |
|  | i3 + SURF, rotation only | 2.84 | 4.50 | 70.1 | 88.9 | 99.5 |
| cloister | KISS-SLAM, no deskew | 2.22 | 3.06 | 83.2 | 96.6 | 99.5 |
|  | KISS-SLAM | 2.56 | 3.96 | 75.0 | 91.8 | 98.6 |
|  | KISS-SLAM, indoor_detail | 2.27 | 3.42 | 80.6 | 94.4 | 99.0 |
|  | i3 + SIFT | **1.85** | 2.53 | **89.6** | 97.8 | 99.5 |
|  | i3 + SURF | 1.88 | 2.57 | 89.0 | 97.8 | 99.5 |
|  | i3 + SURF, rotation only | 2.82 | 3.73 | 73.5 | 95.1 | 99.5 |
| christ-church-03 (Hesai) | KISS-SLAM, no deskew | 2.29 | 3.55 | 78.4 | 93.8 | 98.9 |
|  | KISS-SLAM | 4.21 | 6.67 | 55.9 | 78.9 | 98.2 |
|  | KISS-SLAM, indoor_detail | 3.86 | 6.28 | 58.8 | 80.8 | 98.3 |
|  | i3 + SIFT | 2.04 | 3.21 | 81.7 | 95.0 | 98.9 |
|  | i3 + SURF | **2.01** | 3.15 | **82.2** | 95.2 | 98.9 |
|  | i3 + SURF, rotation only | 2.35 | 3.67 | 75.9 | 93.2 | 98.9 |
| christ-church-02 (Hesai) | KISS-SLAM, no deskew | 2.31 | 3.83 | 78.2 | 92.4 | 96.4 |
|  | KISS-SLAM | 5.18 | 7.85 | 48.7 | 73.5 | 95.4 |
|  | KISS-SLAM, indoor_detail | 4.35 | 6.90 | 54.9 | 78.2 | 95.6 |
|  | i3 + SIFT | 2.20 | 3.55 | 80.3 | 93.6 | 96.5 |
|  | i3 + SURF | **2.13** | 3.42 | **81.4** | 94.1 | 96.5 |
|  | i3 + SURF, rotation only | 2.53 | 4.00 | 74.8 | 92.0 | 96.5 |

### The map as built (own poses)

| Sequence | Arm | median [cm] | mean [cm] | < 5 cm [%] | < 10 cm [%] | within 0.5 m [%] |
|---|---|---|---|---|---|---|
| quad_easy | KISS-SLAM, no deskew | 2.06 | 2.93 | 84.4 | 96.5 | 99.6 |
|  | KISS-SLAM | 2.17 | 3.15 | 82.4 | 95.5 | 99.5 |
|  | KISS-SLAM, indoor_detail | 2.14 | 3.13 | 82.6 | 95.4 | 99.5 |
|  | i3 + SIFT | 1.93 | 2.68 | 87.2 | 97.3 | 99.5 |
|  | i3 + SURF | **1.86** | 2.53 | **88.7** | 97.8 | 99.6 |
|  | i3 + SURF, rotation only | 1.94 | 2.66 | 87.1 | 97.6 | 99.6 |
| cloister | KISS-SLAM, no deskew | **6.10** | 9.56 | **44.9** | 63.8 | 34.9 |
|  | KISS-SLAM | 7.25 | 9.99 | 40.4 | 59.7 | 45.6 |
|  | KISS-SLAM, indoor_detail | 6.42 | 9.06 | 43.1 | 63.7 | 85.7 |
|  | i3 + SIFT | 6.84 | 9.06 | 40.9 | 63.3 | 73.2 |
|  | i3 + SURF | 6.93 | 9.37 | 40.6 | 62.2 | 70.5 |
|  | i3 + SURF, rotation only | 6.17 | 9.73 | 44.1 | 63.1 | 70.6 |
| christ-church-03 (Hesai) | KISS-SLAM, no deskew | 3.44 | 4.65 | 65.3 | 90.0 | 98.9 |
|  | KISS-SLAM | 3.60 | 4.92 | 63.2 | 88.4 | 98.9 |
|  | KISS-SLAM, indoor_detail | 3.70 | 4.97 | 62.6 | 88.4 | 98.9 |
|  | i3 + SIFT | **2.17** | 3.10 | **82.9** | 96.5 | 98.9 |
|  | i3 + SURF | 2.26 | 3.23 | 81.1 | 96.0 | 98.9 |
|  | i3 + SURF, rotation only | 2.68 | 3.61 | 76.4 | 95.4 | 98.9 |
| christ-church-02 (Hesai) | KISS-SLAM, no deskew | 7.26 | 10.40 | 37.4 | 61.8 | 90.7 |
|  | KISS-SLAM | 9.11 | 12.24 | 30.6 | 53.5 | 87.7 |
|  | KISS-SLAM, indoor_detail | 13.91 | 16.90 | 21.1 | 38.5 | 59.5 |
|  | i3 + SIFT | 4.70 | 7.25 | 52.4 | 78.3 | 89.3 |
|  | i3 + SURF | 5.00 | 7.59 | 50.0 | 75.3 | 95.2 |
|  | i3 + SURF, rotation only | **4.68** | 6.86 | **52.5** | 77.7 | 95.6 |

**Cloister, own poses: not valid.** Only 35–86 % of the map lies within 0.5 m of the survey and the rigid ICP moved the maps 1–5 m: the 429 m trajectory has drift and no loop closure, so one rigid alignment cannot fit the whole map.  **Stairs** is not in the table: the stairwell interior is not in the survey (25–36 % within 0.5 m even at GT poses).

**christ-church-02, own poses: partly valid.** The 642 m trajectory has drift and no loop closure: the rigid ICP moved the maps 15–208 cm
and 60–95 % of the points lie within 0.5 m (indoor_detail, ATE 3.1 m, is the worst).  **christ-church-03 at 100 % of each scan (#051)**
gives the same numbers as the 10 % sample above within 0.1 percentage point (e.g. SURF 82.2 % / 81.2 %), so 10 % is enough.
