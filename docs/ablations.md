# Ablations and option tests — consolidated (9/10/2026)

Every ablation of the method and every tested option, newest decisions first in §0, then chronologically. Each section: **date · log entries**
(`docs/experiment_log.md`) · what changes · table · verdict. Conclusions ⏳ until validated by Λ.Γ. Replaces `ablation_165.md`, `results_ablation_081.md`,
`image_options_2026-09-29.md` (git history). Full results of the method: `docs/results.md`. Official protocols only (#061); ours 4 seeds unless noted.

## 0. What is in the locked method C and why (status 9/10)

| component | in C? | evidence (log) | effect (geo-mean APE / KISS-SLAM, §1) |
|---|---|---|---|
| image motion as deskew **and** ICP start | yes (core) | #030, #081, #165 | NCD 0.88 → 0.42, Spires 0.78 → 0.27 |
| two starts (image / constant velocity, > 5°) | yes | #057–#059, #165 | Spires 0.27 → 0.18, Boreas 115 → 12 |
| range-image fallback | yes | #069, #078, #165 | Hilti 0.34 → 0.25, NTU 0.66 → 0.49, Boreas 12 → 1.03 |
| upright SURF + guided matching | yes (implementation) | #086–#089 | Spires 0.18 → 0.16, Boreas 1.03 → 0.88; real time on hand-held |
| blend for the deskew (B) | yes | #131, #137–#140 | NTU 0.51 → 0.37, Boreas 0.88 → 0.73 |
| KISS fallback after 4 consecutive failures (C) | yes | #146–#151, #164 | NTU new 0.44 → 0.32 (spms) |
| near-floor filter, fixed σ = 2.0 | yes | #025–#031, #081 | small, consistent (§2) |
| car configuration (blend as ICP start + translation trigger) | **option** (vehicles) | #211–#227 | MulRan highway fixed, drone ruined (§6) |
| real-time stack (×4, C++, look-ahead, cache, no GIL) | **option** | #186–#202 | Boreas 1.07× real time, same odometry |
| everything in §3–§8 | no | – | – |

---

## 1. Ablation of the locked RA-L method (5/10, #165)

Geometric mean over each group of APE / APE(KISS-SLAM); RPE 1 m median over NCD + Spires; failures = APE > 5 m; last column = sequences better / worse than
the previous row by > 5 % in APE. stairs / dynamic_spinning excluded. Script `scripts/ablation_table_165.py`.

### Cumulative

| row | NCD (10): APE / KISS | Oxford Spires (6): APE / KISS | Hilti (6): APE / KISS | NTU eee (3): APE / KISS | Car Boreas (1): APE / KISS | NTU new (15): APE / KISS | RPE 1 m t / r (NCD + Spires) | failures | vs previous: better / worse (> 5 %) |
|---|---|---|---|---|---|---|---|---|---|
| KISS-SLAM | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 21.41 / 2.805 | 3 | – |
| KISS-SLAM without deskew | 0.88 | 0.78 | 0.49 | 0.95 | 26.79 | – | 10.32 / 0.979 | 3 | 17 / 5 (of 26) |
| + image motion (deskew + ICP start) | 0.42 | 0.27 | 0.32 | 0.65 | 115.52 | – | 7.28 / 1.097 | 1 | 18 / 4 (of 26) |
| + two starts | 0.42 | 0.18 | 0.34 | 0.66 | 12.11 | – | 7.26 / 1.065 | 0 | 3 / 4 (of 26) |
| + range-image fallback | 0.41 | 0.18 | 0.25 | 0.49 | 1.03 | – | 7.25 / 1.053 | 0 | 7 / 2 (of 26) |
| + upright SURF / guided matching (implementation) | 0.42 | 0.16 | 0.24 | 0.51 | 0.88 | 0.52 | 7.01 / 0.982 | 0 | 9 / 6 (of 26) |
| + blend for the deskew (B) | 0.41 | 0.16 | 0.24 | 0.37 | 0.73 | 0.44 | 6.86 / 0.954 | 0 | 9 / 2 (of 26) |
| + KISS fallback after 4 failures (C) = locked method | 0.41 | 0.16 | 0.24 | 0.38 | 0.73 | 0.32 | 6.86 / 0.954 | 0 | 0 / 0 (of 26) |

### Leave-one-out (each against “+ range-image fallback”, #081 / #082)

| row | NCD (10): APE / KISS | Oxford Spires (6): APE / KISS | Hilti (6): APE / KISS | NTU eee (3): APE / KISS | Car Boreas (1): APE / KISS | NTU new (15): APE / KISS | RPE 1 m t / r (NCD + Spires) | failures | vs previous: better / worse (> 5 %) |
|---|---|---|---|---|---|---|---|---|---|
| + range-image fallback | 0.41 | 0.18 | 0.25 | 0.49 | 1.03 | – | 7.25 / 1.053 | 0 | 0 / 0 (of 26) |
| − image as ICP start (deskew only) | 0.76 | 0.55 | 0.70 | 0.40 | – | – | 8.77 / 1.078 | 2 | 5 / 15 (of 25) |
| − image deskew (ICP start only) | 0.51 | 0.32 | 0.28 | 0.56 | – | – | 9.65 / 0.791 | 0 | 4 / 18 (of 25) |
| − near-floor filter | 0.43 | 0.19 | 0.26 | 0.53 | – | – | 7.34 / 1.068 | 0 | 3 / 8 (of 25) |
| − fixed σ (KISS adaptive) | 0.41 | 0.17 | 0.26 | 0.69 | – | – | 7.41 / 1.047 | 0 | 5 / 6 (of 25) |

**Verdict (⏳):** each component earns its place on at least one dataset; the two uses of the image motion are complementary (the ICP start prevents failures,
the deskew brings translation accuracy); C adds nothing where B already works and rescues the drone at altitude (NTU new 0.44 → 0.32).

---

## 2. Ablation of the method of 29/9 — paired, 27 sequences (30/9, #081 / #082)

Method = two starts + range fallback (`surftworangefb`); each ablation changes ONE thing. Wilcoxon over sequences. Image only as ICP start (no deskew):
+31 % APE, RPE +45 % (7–20) — the deskew brings the translation accuracy.

### Paired comparison against the method (27 sequences; RPE: the 18 NCD / Spires sequences)

| ablation | APE: ablation wins – method wins | median APE change | p (Wilcoxon) | RPE 1 s translation: wins, change, p | verdict |
|---|---|---|---|---|---|
| − floor filter | 7 – 20 | +4.2 % | 0.003 | 3 – 15, +2.7 %, 0.005 | the filter helps (small, consistent) |
| adaptive σ (KISS default) | 15 – 12 | −1.8 % | 0.90 | 11 – 7, −0.2 %, 0.77 | tie on average; fixed σ protects NTU / long experiment / Office_Mitte_1 |
| image for deskew only | 6 – 21 | +8.4 % | 0.002 | 2 – 16, +9.0 %, 0.0001 | the image start is the largest part; failures up to 14 m |

### Per sequence (APE, m)

| dataset | sequence | method (two starts + range fallback) | − floor filter | adaptive σ (KISS default) | image for deskew only (ICP from constant velocity) |
|---|---|---|---|---|---|
| Newer College 2020 | 01_short | 0.307 ± 0.018 | 0.335 ± 0.034 | **0.299** ± 0.007 | 0.329 ± 0.050 |
| Newer College 2021 | quad_easy | **0.079** ± 0.001 | 0.082 ± 0.000 | 0.079 ± 0.000 | 0.080 ± 0.000 |
| Newer College 2021 | stairs | **2.074** ± 0.225 | 2.332 ± 0.609 | 2.200 ± 0.642 | 3.399 ± 0.352 |
| Newer College 2021 | cloister | **0.188** ± 0.013 | 0.190 ± 0.021 | 0.189 ± 0.015 | 0.267 ± 0.041 |
| Newer College 2021 | math_easy | 0.108 ± 0.001 | 0.112 ± 0.002 | **0.106** ± 0.003 | 0.114 ± 0.002 |
| Newer College 2021 | underground_easy | 0.067 ± 0.002 | 0.068 ± 0.001 | **0.058** ± 0.000 | 0.071 ± 0.004 |
| Oxford Spires | christ-church-02 | 0.207 ± 0.044 | 0.248 ± 0.089 | **0.185** ± 0.030 | 0.453 ± 0.116 |
| Oxford Spires | christ-church-03 | 0.044 ± 0.001 | 0.045 ± 0.004 | **0.042** ± 0.002 | 0.065 ± 0.004 |
| Newer College 2021 | quad_hard | 0.218 ± 0.012 | 0.241 ± 0.007 | **0.210** ± 0.019 | 0.212 ± 0.036 |
| Newer College 2021 | math_medium | **0.152** ± 0.001 | 0.163 ± 0.002 | 0.153 ± 0.002 | 0.155 ± 0.003 |
| Newer College 2021 | underground_medium | 0.061 ± 0.002 | 0.061 ± 0.003 | **0.058** ± 0.001 | 0.080 ± 0.014 |
| Newer College 2021 | underground_hard | 0.086 ± 0.003 | 0.089 ± 0.002 | **0.083** ± 0.003 | 14.215 ± 1.697 |
| Oxford Spires | keble-college-03 | 0.094 ± 0.001 | 0.101 ± 0.004 | **0.093** ± 0.002 | 4.426 ± 3.524 |
| Oxford Spires | observatory-quarter-01 | 0.080 ± 0.020 | **0.077** ± 0.005 | 0.078 ± 0.022 | 0.262 ± 0.115 |
| Oxford Spires | blenheim-palace-02 | 0.268 ± 0.015 | 0.385 ± 0.017 | 0.262 ± 0.020 | **0.253** ± 0.020 |
| Oxford Spires | bodleian-library-02 | 0.545 ± 0.030 | **0.487** ± 0.070 | 0.528 ± 0.145 | 0.955 ± 0.073 |
| Newer College 2020 | 02_long_experiment | **1.619** ± 0.761 | 1.695 ± 0.838 | 2.302 ± 0.204 | 2.244 ± 0.233 |
| Newer College 2020 | dynamic_spinning | 0.504 ± 0.263 | 0.547 ± 0.378 | **0.451** ± 0.082 | 6.103 ± 3.533 |
| Hilti 2021 | Basement_1 | 0.050 ± 0.008 | 0.043 ± 0.017 | 0.061 ± 0.011 | **0.043** ± 0.016 |
| Hilti 2021 | IC_Office_1 | 0.071 ± 0.006 | 0.117 ± 0.091 | **0.068** ± 0.005 | 7.248 ± 3.610 |
| Hilti 2021 | Office_Mitte_1 | 0.241 ± 0.060 | **0.210** ± 0.054 | 0.303 ± 0.034 | 1.490 ± 2.185 |
| Hilti 2021 | Construction_Site_1 | 0.048 ± 0.004 | 0.047 ± 0.004 | 0.045 ± 0.002 | **0.042** ± 0.006 |
| Hilti 2021 | LAB_Survey_2 | 0.036 ± 0.001 | **0.035** ± 0.000 | 0.036 ± 0.000 | 0.037 ± 0.001 |
| Hilti 2021 | UZH_Tracking_Area_Run_2 | **0.551** ± 0.000 | 0.565 ± 0.015 | 0.564 ± 0.015 | 0.577 ± 0.001 |
| NTU VIRAL | eee_01 | 1.737 ± 0.190 | 1.789 ± 0.118 | 1.983 ± 0.089 | **0.923** ± 0.201 |
| NTU VIRAL | eee_02 | 0.679 ± 0.064 | 0.710 ± 0.042 | 1.174 ± 0.104 | **0.640** ± 0.059 |
| NTU VIRAL | eee_03 | **0.344** ± 0.079 | 0.400 ± 0.041 | 0.495 ± 0.075 | 0.373 ± 0.092 |

---

## 3. Image options (29/9, #069–#076)

All with two starts; defaults unchanged (decision M.T. 28/9: options). → the range fallback entered the method (#078 / #092).

### Summary

| Option | What it does | Result | Verdict |
|---|---|---|---|
| **Range fallback** (`--range=fallback`) | depth-image motion where the intensity image gives none | tested on all 27: better 10, tie 13, worse 4; large gains NTU (−14…−42 %), Office_Mitte_1 (1.44 → 0.24 m), long experiment | **best general option** where the intensity image fails |
| **Range 3rd start** (`--range=candidate`) | depth-image motion as an extra ICP start | 6 sequences: fixes fast rotation (dynamic_spinning 0.46 → 0.30 m) and Office_Mitte_1; no help on NTU / UZH | for fast rotation / wrong motions |
| **Intensity ×1.0** (`--intensity-scale=1.0`) | brighter fixed scale (Hilti / NTU Ouster are dark at ×255/1024) | failures −82…−97 % (Hilti), −37 % (NTU); scores mixed (better 4, worse 3 of 9) | superseded by gain |
| **Per-scan gain** (`--normalise=gain`, branch `intensity_norm`) | each scan scaled so its p99 = 255 | Hilti better or equal on 5 / 6 (Office_Mitte_1 0.23 m); NCD unharmed; NTU mixed | **for Hilti-type (dark) data** |
| **Panorama 2048** (`--panorama-width=2048`) | the Hilti Ouster's own columns | fewer failures (UZH 64 → 31), no better scores | not worth it |
| `reflectivity` field | calibrated field instead of intensity | worse (NTU 1376 failures vs 358) | rejected |

vs two starts (±1 % = tie), better / tie / worse: range fallback 10 / 13 / 4 (27) · range 3rd start 4 / 1 / 1 (6) · ×1.0 4 / 2 / 3 (9) ·
×1.0 + fallback 5 / 1 / 3 (9) · ×1.0 + 3rd start 5 / 0 / 4 (9) · gain 6 / 1 / 5 (12) · gain + 2048 3 / 2 / 1 (6).

Still open: UZH (no image variant beats no deskew — the image motion there is wrong, not missing); the fast-rotation seed that still
fails on dynamic_spinning; per-sequence best settings differ on NTU.

### Complete table

| Dataset | Sequence | KISS | no deskew | two starts | + range fallback | + range 3rd start | ×1.0 | ×1.0 + fallback | ×1.0 + 3rd start | gain | gain + 2048 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Newer College 2020 | 01_short | 0.419 | 0.350 | **0.305** | 0.307 | — | — | — | — | — | — |
| Newer College 2021 | quad_easy | 0.104 | 0.083 | **0.079** | **0.079** | — | — | — | — | — | — |
| Newer College 2021 | stairs | 3.586 | 2.705 | **2.074** | **2.074** | — | — | — | — | — | — |
| Newer College 2021 | cloister | 0.396 | 0.480 | 0.188 | **0.188** | — | — | — | — | — | — |
| Newer College 2021 | math_easy | 0.160 | 0.104 | 0.108 | 0.108 | — | — | — | — | **0.104** | — |
| Newer College 2021 | underground_easy | 0.117 | 0.092 | **0.067** | **0.067** | — | — | — | — | — | — |
| Oxford Spires | christ-church-02 | 0.777 | 0.546 | 0.209 | **0.207** | — | — | — | — | — | — |
| Oxford Spires | christ-church-03 | 0.143 | 0.089 | **0.044** | **0.044** | — | — | — | — | — | — |
| Newer College 2021 | quad_hard | 0.329 | **0.209** | 0.222 | 0.218 | — | — | — | — | 0.216 | — |
| Newer College 2021 | math_medium | 0.253 | 0.164 | 0.154 | **0.152** | — | — | — | — | — | — |
| Newer College 2021 | underground_medium | 0.162 | 0.097 | **0.061** | **0.061** | — | — | — | — | — | — |
| Newer College 2021 | underground_hard | 12.780 | 12.341 | 0.086 | **0.086** | — | — | — | — | 0.087 | — |
| Oxford Spires | keble-college-03 | 9.757 | 11.370 | **0.094** | 0.094 | — | — | — | — | — | — |
| Oxford Spires | observatory-quarter-01 | 0.497 | 0.436 | **0.078** | 0.080 | — | — | — | — | — | — |
| Oxford Spires | blenheim-palace-02 | 0.293 | **0.205** | 0.273 | 0.268 | — | — | — | — | — | — |
| Oxford Spires | bodleian-library-02 | 1.911 | 1.363 | **0.513** | 0.545 | — | — | — | — | — | — |
| Newer College 2020 | 02_long_experiment | **1.275** | 3.498 | 2.113 | 1.619 | — | — | — | — | — | — |
| Newer College 2020 | dynamic_spinning | **0.159** | 20.751 | 0.460 | 0.504 | 0.302 | — | — | — | — | — |
| Hilti 2021 | Basement_1 | 0.055 | 0.078 | 0.058 | **0.050** | — | 0.058 | 0.058 | 0.060 | 0.052 | 0.052 |
| Hilti 2021 | IC_Office_1 | 6.344 | 1.655 | 0.072 | **0.071** | — | 0.076 | 0.076 | 0.074 | 0.074 | 0.072 |
| Hilti 2021 | Office_Mitte_1 | 4.286 | 0.575 | 1.437 | 0.241 | 0.238 | 0.282 | 0.282 | **0.202** | 0.226 | 0.315 |
| Hilti 2021 | Construction_Site_1 | 0.063 | 0.062 | 0.049 | 0.048 | — | 0.043 | 0.044 | **0.041** | 0.045 | 0.046 |
| Hilti 2021 | LAB_Survey_2 | 0.062 | 0.050 | 0.036 | 0.036 | — | 0.035 | 0.035 | **0.035** | 0.035 | 0.036 |
| Hilti 2021 | UZH_Tracking_Area_Run_2 | 0.585 | **0.204** | 0.503 | 0.551 | 0.502 | 0.576 | 0.579 | 0.576 | 0.576 | 0.579 |
| NTU VIRAL | eee_01 | 2.678 | 2.363 | 2.028 | 1.737 | 1.992 | 2.034 | **1.613** | 1.992 | 1.944 | — |
| NTU VIRAL | eee_02 | 1.486 | 1.490 | 0.820 | **0.679** | 0.830 | 1.101 | 0.927 | 1.101 | 0.877 | — |
| NTU VIRAL | eee_03 | 0.864 | 0.841 | 0.592 | **0.344** | 0.584 | 0.435 | 0.367 | 0.434 | 0.679 | — |

---

## 4. Method options 1/10–5/10 (#084–#164)

| option (`run_ncd.py`) | log | result | verdict |
|---|---|---|---|
| upright SURF (`--surf-upright`) + guided matching (±40 columns) | #086–#089 | rotation −3.5 % (8–0), real time on hand-held (12.7 / 10.1 Hz); fixed window fails in fast rotation → only when turning slowly, or prediction from motion | **in the method** |
| panorama ×4 on Ouster 128 | #089, #093, #190–#206 | faster and at least as accurate on 128 beams; "512 rows for every sensor" rejected (NTU ×32 mixed, Hesai 600 columns no) | real-time option |
| bearings for the rotation | #087 | worse | out |
| larger baseline k−2 (`--multi-baseline`) | #090, #130–#133 | +24–35 % (hand-held motion not smooth over 0.3 s) | out |
| range fusion (`--fuse-range`) | #090, #132–#152 | −6 % on one sequence, large cost | out |
| sector weights (`--sectors=8`) | #093, #132–#144 | rotation slightly better, translation worse | out |
| intensity normalisations (gain, CLAHE, r², smooth) | #075, #118–#126 | no consistent gain | out |
| drop "stationary" pairs (`--drop-stationary`) | #130–#135 | no gain | out |
| ICP-failure detector, image instead of ICP in degenerate geometry | #126–#131 | no reliable detector | out |
| **blend for the deskew (B)** `--cv-blend=adaptive --cv-blend-use=deskew` | #131, #137–#140 | car −16…−25 %, drone up to −61 % | **in the method** |
| blend also as ICP start (`--cv-blend-use=both`) | #137, #217, #226 | drone much worse; helps the car on the highway | car option (§6) |
| **KISS fallback in runs of failures (C)** `--fallback=kiss --fallback-after=4` | #143, #146–#151 | spms 48 → 8 m, no harm elsewhere | **in the method** |
| deskew of the constant-velocity winner (`--cv-winner-deskew`) | #163 | no gain | out |
| B.9 joint deskew in the registration | #115, #155–#160 | very good on quad_easy / eee_01, rotation divergences on 5 / 7 | open (decision M.T. 5/10) |

---

## 5. Speed and image rules (6/10, #166–#206)

| change | log | result | verdict |
|---|---|---|---|
| reader thread, contiguous points, ring order by bincount, down-sample reuse | #170, #174, #176 | identical trajectories, Boreas 616 → 404 s | **in `ral_method`** |
| image 4 scans ahead (`KISS_IMAGE_AHEAD`), range cache (`KISS_RANGE_CACHE`) | #186, #201 | identical, faster | real-time option |
| image stage in C++ (`KISS_IMAGE_CPP=all`) | #182–#189 | = another seed | real-time option |
| KISS-ICP without the GIL (env `kiss-slam-gil`) | #189–#202 | faster, identical | real-time option |
| ORB / AKAZE instead of SURF | #181–#185 | ORB real time on the car but APE +11 %, worse on hand-held; AKAZE out | out |
| blend weighted by the scan's inliers (`--cv-blend-inliers`) | #200, #203 | neutral (geo mean 0.998 over 42 + Boreas) | option only |
| panorama rows: 512-row rule, isotropic pixels, saturation (`--panorama-up=auto|saturate`) | #190–#206 | holds only as "×4 on 128 beams" | rule open for 128 beams |
| 4-thread budget | #168 | identical, 3–8 % slower | removed (decision M.T. 5/10) |

---

## 6. Car options (7/10–8/10, #211–#227)

MulRan seed 0 (RTE %, SLAM); "car configuration" = blend as ICP start + translation trigger 0.2.

| MulRan | KISS-SLAM | C | + trigger | + stuck filter | + filter + trigger | + blend start | **+ blend start + trigger (car config)** |
|---|---|---|---|---|---|---|---|
| Sejong01 APE m · RTE % | **165.4 · 4.27** | 259.4 · 8.29 | 179.3 · 5.70 | 514.0 · 7.97 | 197.8 · 6.58 | 178.3 · 5.52 | 175.1 · 5.42 |
| Riverside01 | 6.42 · 3.57 | 40.37 · 11.52 | 6.83 · 3.76 | 22.69 · 7.31 | – | 9.73 · 4.12 | **5.66 · 3.42** |
| KAIST01 | 3.49 · 2.57 | 3.51 · 2.61 | 3.36 · 2.57 | 3.37 · 2.49 | 3.17 · 2.46 | 2.83 · 2.39 | **2.92 · 2.39** |

4 seeds (#223): Riverside01 car config 3.64 ± 0.21 % (= KISS-SLAM), Sejong01 5.38 ± 0.06 % (behind). On hand-held / drone (#222 / #226, `docs/results.md`
§5) the blend start ruins the drone, the trigger alone is harmless. Sejong01 gap = one 22 s stall (#227, open). Boreas elevation offset `--elev-offset`
(#218–#221): z ×0.49, RTE ×0.70 — diagnostic only (decision M.T. 7/10: uncorrected for the paper).

---

## 7. Image alternatives (8/10, #230–#236; branch `matching_intensities`, all options, none in the method)

| test | option | sequence(s) | result | verdict |
|---|---|---|---|---|
| 1D KLT flow along the rings, native panorama | `run_ncd.py klt1d --panorama-up=1` | church_03 / quad_hard / eee_03 | APE 0.064 / 0.416 / 0.805 vs C 0.039 / 0.234 / 0.145 (KISS 0.143 / 0.329 / 0.864) | out (ignores vertical shifts) |
| 2D pyramidal KLT | `klt2d` | quad_hard | 0.253 m (C 0.234 ± 0.029) | ≈ C |
| upright ORB | `uorb` | quad_hard | 0.174 m, RTE 0.75 % (1 seed) | open (4 seeds + more sequences) |
| sub-panoramas 180° / 90° overlap | offline | quad_hard | no better than the full fit (each match spans one period) | not run |
| cubic motion model | `--model=cub` | church_03 | APE 0.044 vs 0.039 ± 0.0004, RPE 4.35 vs 3.93 cm | out |
| MAGSAC++ / GNC-TLS / two-model fitting | `--robust=magsac|gnc|multi` | church_03, quad_hard (offline), Sejong01 | = RANSAC (Sejong RTE 8.36 / 8.32 / 8.15 vs 8.33 %); step 68 / 5.3 / 18 ms vs 3.8 ms | out |

Motion model from 38 ground truths (#233, leave-one-out): constant velocity never best; angular acceleration ("car", the method's) is the right minimum for
hand-held; translation acceleration ("ca") helps on cars (×0.68–0.79 of car) — candidate for the car configuration.
