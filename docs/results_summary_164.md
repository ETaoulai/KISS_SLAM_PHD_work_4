# Summary of every full-run arm (#162)

APE / ATE in m (each dataset's official protocol), mean over the arm's seeds (4 for ours, 1 for KISS / no deskew / other methods). **Bold** = best LiDAR-only on the sequence; † = failure (APE > 5 m); – = not run. IMU methods (FAST-LIO2, COIN-LIO) are reference only.

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

## Per dataset (special sequences excluded)

### NCD (10 sequences)

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

### Oxford Spires (6 sequences)

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

### Hilti 2021 (6 sequences)

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

### NTU (eee) (3 sequences)

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

### NTU (15 new) (15 sequences)

| arm | sequences | median APE / KISS | geo-mean APE / KISS | LiDAR-only wins | failures (> 5 m) | mean rank* |
|---|---|---|---|---|---|---|
| KISS-SLAM | 15 | 1.00 | 1.00 | 1 | 3 | 4.3 |
| Ours default | 15 | 0.65 | 0.52 | 6 | 3 | 2.7 |
| + blend (B) | 15 | 0.51 | 0.44 | 2 | 3 | 2.5 |
| + blend + fallback (A) | 15 | 0.47 | 0.31 | 5 | 2 | 2.7 |
| + blend + fallback after 4 (C) | 15 | 0.49 | 0.32 | 1 | 2 | 2.7 |

*mean rank among the 5 LiDAR-only arms run on all 15 sequences: KISS-SLAM, Ours default, + blend (B), + blend + fallback (A), + blend + fallback after 4 (C)

### Car (Boreas) (1 sequences)

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

## RPE 1 m (NCD + Oxford Spires + Boreas, translation cm / rotation °), median over sequences

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
