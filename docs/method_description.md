# Method: LiDAR odometry with the sweep motion measured in the intensity image

*Description of the method as implemented on 25/9/2026 (branch `after_two_start`, commit `165b351`), in its default
configuration. Every step is stated as the code does it; the reason for each design choice points to the entry of
`docs/experiment_log.md` that established it. Sources checked: our code (`kiss_slam/`), KISS-ICP 1.3.0 (C++) and
MapClosures 2.1.0 (C++). All conclusions are ⏳ until validated by Λ.Γ.*

---

## 1. Idea in one paragraph

A hand-held spinning LiDAR moves **during** each 0.1 s sweep, so every scan must be deskewed with the motion of that
sweep. KISS-ICP assumes the motion is the same as in the previous sweep (constant velocity). For a hand-held unit this
guess is worse than no deskew at all, because the rotation of the hand changes from sweep to sweep almost as much as it
is (#012, #019, #045). The ICP itself cannot recover the motion during the sweep: a scan deskewed with a wrong motion
still fits a smooth map (#017).

Our method measures that motion **independently of the geometry**. The intensity panorama of a spinning LiDAR is a
rolling-shutter image in which each column was acquired at a different instant. Matching point features between the
panoramas of two consecutive raw scans, each with its own timestamp, constrains the sensor trajectory during the sweep.
The measured motion is then used for **both** the deskew and the initial guess of the ICP; the rest is KISS-SLAM.

```
raw scan k ─┬─> intensity panorama ─> SURF features ─> matches with scan k−1 ─> RANSAC ─> timed fit ─> Δ̂_k   (§3–§5)
            │                                                                                            │
            └──────────────────────────────> deskew with Δ̂_k (§6) ─> ICP from T_{k−1}·Δ̂_k (§6, §8) <───┘
                                              (+ second start from constant velocity when they disagree, §7)
                                                   │
                                              local maps ─> MapClosures loop closures ─> pose graph (§9)
```

LiDAR only: no IMU, no camera. Only the current and the previous scan are used for the motion.

## 2. Notation and time

- Sweep period $T = 0.1$ s. Point $j$ of a scan has its own acquisition time $t_j$. The scan stamp is the time of its
  **first** point (Hesai, Ouster, measured #061).
- $\Delta_k\in SE(3)$: the motion over sweep $k$, i.e. the pose of the sensor at the end of sweep $k$ in its pose at
  the end of sweep $k-1$ (KISS-ICP's `delta`).
- $T_k\in SE(3)$: the pose of the sensor at the **end** of sweep $k$ in the frame of the current local map.
- Normalised time used by the motion estimator, from the start of the later scan:
  $\tau = (t - t_{\text{start},k})/T$, so points of scan $k-1$ have $\tau\in[-1,0)$ and points of scan $k$ have
  $\tau\in[0,1)$.

## 3. The intensity panorama (per raw scan)

1. Points closer than 1 m are dropped (the operator and the rig).
2. Intensity is scaled to 0–255 and clipped: ×1 for the Hesai (native 0–255); ×255/1024 for the Ouster, whose values
   go from 0 to about 1100 (#041).
3. **Grid:** one row per ring, rows ordered by the ring's mean elevation; $W = 1024$ columns in azimuth. Each point is
   written into its pixel (sensors with 2048 columns: two returns per pixel, the last one is kept). Every pixel stores
   the intensity, the **raw** 3D point in the sensor frame at its own time, and its **time**.
4. Empty pixels are filled by linear interpolation of the intensity along their row. This affects the image only;
   empty pixels carry no 3D point or time.
5. The image is up-sampled 8× vertically (bilinear), because the rings are far fewer than the columns.

*Why:* this is the raw sensor image, with no deskew, so each pixel's time is exact (#018). Rendering alternatives
(ray casting #034, masking pixels fixed to the rig #036, intensity calibration #025) gave no gain in the SLAM result
and are off.

## 4. Features, matches and their 3D/time lifting

1. **SURF** keypoints and descriptors (Hessian threshold 100) on the panorama. SIFT gives nearly the same result but is
   slower and slightly less accurate (#041, #047).
2. Matches between scans $k-1$ and $k$: brute-force L2 nearest neighbour with Lowe's ratio test at 0.75.
3. **Lifting:** each keypoint gets a 3D point and a time. They are bilinearly interpolated over its 4 surrounding
   pixels if all four are valid and their ranges agree within 5 % (same surface). Otherwise the nearest valid pixel in
   the row, up to ±2 columns, is used; if there is none, the match is dropped (#021).
4. **Near-floor filter:** a match is dropped if the two points are within 5 cm of each other in the sensor frame
   ($\|p-q\| < 0.05$ m) **and** it lies on the near floor (below −10° elevation and closer than 5 m). Intensity patterns
   there travel with the sensor (range and incidence fall-off) and would pull the translation toward zero: without the
   filter the measured motion is 8 % short (#023–#027).

A match $i$ is the pair $(p_i, \tau_{p,i})$ in scan $k-1$ and $(q_i,\tau_{q,i})$ in scan $k$. Useful matches saturate at
80–160 per scan pair; more (e.g. dense matching) does not help (#038, #039).

## 5. Motion estimation from the matches

### 5.1 Model
The pose of the sensor at time $\tau$, relative to its pose at $\tau = 0$, is modelled on $SO(3)\times\mathbb R^3$ with a
constant angular acceleration and a constant linear velocity (model "car", #020–#021):
$$
\phi(\tau) = \tau\,\omega + \tfrac{\tau^2}{2}\,\alpha,\qquad s(\tau) = \tau\,v,\qquad
P_x(\tau)\,y = \mathrm{Exp}\big(\phi(\tau)\big)\,y + s(\tau),\qquad x = (\omega, v, \alpha)\in\mathbb R^9 .
$$
*Why:* the hand's rotation changes within two sweeps, which a single velocity cannot follow. Acceleration in
translation as well ("ca") improves rotation but hurts translation (#020).

### 5.2 Residual and Jacobian
A matched feature is the same point in space:
$$
r_i(x) = \mathrm{Exp}\big(\phi(\tau_{p,i})\big)\,p_i + s(\tau_{p,i}) - \mathrm{Exp}\big(\phi(\tau_{q,i})\big)\,q_i - s(\tau_{q,i}) .
$$
Analytical Jacobian, with $y = \mathrm{Exp}(\phi)z$ and the left Jacobian $J_l$ of $SO(3)$:
$$
\frac{\partial\,\mathrm{Exp}(\phi) z}{\partial \phi} = -[y]_\times J_l(\phi),\qquad
J_l(\phi) = I + \tfrac{1-\cos\theta}{\theta^2}[\phi]_\times + \tfrac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2 ,
$$
$$
\frac{\partial r_i}{\partial \omega} = \sum_{\sigma=\pm1}\sigma\,\tau\,\big(-[y]_\times J_l\big),\quad
\frac{\partial r_i}{\partial \alpha} = \sum_{\sigma=\pm1}\sigma\,\tfrac{\tau^2}{2}\,\big(-[y]_\times J_l\big),\quad
\frac{\partial r_i}{\partial v} = \sum_{\sigma=\pm1}\sigma\,\tau\, I_3 ,
$$
with $\sigma=+1,\ \tau=\tau_p,\ z = p_i$ for the first term and $\sigma=-1,\ \tau = \tau_q,\ z = q_i$ for the second.

### 5.3 Estimation
1. **Rigid RANSAC** (times ignored): 3-point Kabsch hypotheses $p \approx Rq + t$; inlier if
   $\|p_i - (Rq_i + t)\| < 0.30$ m. The number of hypotheses adapts at confidence 0.999, with at most 400.
   At least 10 inliers are required.
2. **Timed fit** on the RANSAC inliers. Start: $\omega = \mathrm{Log}\,R$, $v = t$, $\alpha = 0$. The robust nonlinear
   least squares $\min_x \sum \rho\big(r_{i,c}(x)\big)$ uses the soft-$\ell_1$ loss with scale 0.05 m (applied to each
   coordinate of each residual) and the Jacobian above (SciPy trust-region). First 3 rounds with $\alpha \equiv 0$
   ("cv"), then 3 rounds of the full model; after each round inliers are re-selected at $\|r_i\| < 0.10$ m.
   At least 10 inliers must remain.
3. **Output:** $\hat\Delta_k = P_{\hat x}(1)$, the motion over the later sweep. If any step fails, scan $k$ has no
   image motion (§6).

The RANSAC seed is the only random element. Different seeds give practically identical motions (same accuracy) but
noticeably different SLAM runs (APE spread 0.02–0.04 m), which is
why every comparison uses 4 seeds (#037).

## 6. Deskew, initial guess and ICP threshold

**Deskew** (KISS-ICP 1.3.0, unchanged): with $s_j = (t_j - \min t)/(\max t - \min t)$ over the whole scan,
$$
\tilde x_j = \mathrm{Exp}_{SE(3)}\big((s_j - 1)\,\mathrm{Log}_{SE(3)}\hat\Delta_k\big)\, x_j ,
$$
a constant twist over the sweep; the deskewed scan is expressed at its **last** point. Points outside the range limits
are removed after the deskew. Only the endpoint $\hat\Delta_k$ of the fitted curve is used. Deskewing each point along
the full curve gave no measurable gain (#033) and is off.

**Initial guess of the ICP:** $T_k^{(0)} = T_{k-1}\,\hat\Delta_k$. Deskew and initial guess **must come from the same
motion**: the image motion for the deskew with the constant-velocity guess for the start lets the start drift (#030).

**No image motion** (too few matches): no deskew ($\Delta = I$) and $T_k^{(0)} = T_{k-1}$. No deskew beats the
constant-velocity deskew for a hand-held unit (#012).

**Fixed ICP threshold:** $\sigma = 2.0$ m for the whole run. With a good initial guess, KISS's adaptive $\sigma$
collapses from 2.4 to 0.6 m, and the ICP then corrects too little (#031).

## 7. Two starting points (default since #059)

When the image motion and KISS's constant-velocity guess $\Delta_{cv} = T_{k-2}^{-1}T_{k-1}$ disagree by more than **5°**
of rotation, the scan is registered twice:

- (A) deskewed with $\hat\Delta_k$ and started from $T_{k-1}\hat\Delta_k$ (as in §6);
- (B) not deskewed and started from $T_{k-1}\Delta_{cv}$.

The result kept is the one that fits the local map better, by the truncated mean distance of the source points to their
nearest map point:
$$
f(T) = \frac1N\sum_j \min\big(\|T\tilde p_j - \mathrm{NN}(T\tilde p_j)\|,\ 3\sigma\big).
$$
*Why:* repeated façades (Blenheim Palace) produce consistent **wrong** intensity matches that no plausibility test
rejects (#053–#056). The second start catches them without harming the other sequences (#057–#059). The second
registration adds 0.3–12 % run time (#058); the constant-velocity start is kept in about 20 % of the scans registered
twice (#062).
A margin on the fit gain before switching (2 % / 4 %) changes nothing measurable and is not used (#062).

## 8. Registration: point-to-point ICP (KISS-ICP 1.3.0, unchanged)

- **Source:** the deskewed scan, downsampled at $0.5v$ (this is what goes into the map) and again at $1.5v$ (this is
  what the ICP registers). $v$ is the map voxel size.
- **Map:** voxel hash of size $v$, at most 20 points per voxel, a minimum spacing of $\sqrt{v^2/20}$ within a voxel.
  Points farther than the maximum range from the sensor are removed.
- **Association** (every iteration): the nearest map point among the 27 neighbouring voxels, accepted within $3\sigma$.
- **Residual, Jacobian, weight:** with $s_j = T\tilde p_j$ and the left perturbation $\delta = (\rho, \theta)$:
$$
r_j = s_j - m_j,\qquad J_j = \big[\, I_3\ \ -[s_j]_\times \big],\qquad
w_j = \frac{\sigma^2}{\big(\sigma + \|r_j\|^2\big)^2} .
$$
- **Solver:** Gauss–Newton, $\delta = -\big(\sum w_j J_j^\top J_j\big)^{-1}\sum w_j J_j^\top r_j$ (LDLT),
  $T\leftarrow\mathrm{Exp}(\delta)\,T$. At most 500 iterations; stop when $\|\delta\| < 10^{-4}$.
- Intensity does **not** enter the ICP.

## 9. Local maps, loop closures and pose graph (KISS-SLAM + MapClosures 2.1.0)

- **Local maps:** a new local map (node) once the sensor has travelled the splitting distance from the start of the
  current one. The ICP map is re-expressed in the new node's frame. Consecutive nodes are joined by an odometry edge
  with information $I_6$.
- **Loop-closure candidates (MapClosures):** for every finished local map:
  1. **Ground alignment:** the lowest voxel in each 1 m cell is taken as a ground candidate, and the dominant
     direction of their normals gives the rotation. Height, roll and pitch are then refined by iterative weighted
     least squares on the ground points, with weights $e^{-z^2}$.
  2. **Density map:** a bird's-eye image at 0.5 m per pixel of the ground-aligned map (point counts normalised;
     cells below 5 % are set to zero).
  3. **Matching:** ORB features (500) on that image, matched by Hamming distance (at most 50) through a binary search
     tree against all earlier maps except the last 3.
  4. **Alignment:** 2-point 2D RANSAC (3-pixel inlier threshold, 0.999 confidence), lifted to 3D through the two
     ground alignments.

  A candidate needs at least 5 inliers. Only the best one is checked (`top_k = 1`).
- **Verification:** Open3D point-to-plane ICP between the two local maps (threshold $\sqrt3\cdot 0.5$ m). The closure is
  accepted if the voxel overlap of the aligned maps exceeds 0.4. Upstream counted the overlap wrongly (it could exceed
  1 and never reject); this is corrected (#009). The loop edge has information $I_6$. Graph optimisation: g2o
  (Dogleg, Cholmod).
- **Final trajectory:** a second graph with every per-scan pose as a vertex, consecutive odometry edges ($I_6$) and the
  node poses fixed.
- Optional height checks exist (reject a closure whose height disagrees with odometry, split maps by height,
  #013–#016); they are **off** in the configurations below.

## 10. Output: pose time stamps (#061)

Each pose stands for the instant it represents: the **last** point of the sweep for a deskewed scan, the mean point
time for a scan registered without deskew. Each run writes `pose_times.csv` and `*_poses_posetime_tum.txt` with these
times ($\text{stamp} + \text{fraction}\times\text{sweep span}$), besides the upstream TUM file stamped with the scan
start. Evaluation uses the official protocol of each dataset, with no time offset (decision M.T. 25/9):

- Oxford Spires / Newer College: `evo_ape --align --t_max_diff 0.01` (`scripts/evaluate_official.py`);
- Hilti 2021: `scripts/evaluate_hilti.py`;
- NTU VIRAL: `scripts/evaluate_ntu.py`.

## 11. Configuration

| Parameter | Value (paper configuration) | "Indoor detail" | Where |
|---|---|---|---|
| Max range / map voxel $v$ | 100 m / 1.0 m ($r_{\max}/100$) | 50 m / 0.25 m | KISS-ICP |
| Local map splitting distance | 100 m | 15 m | KISS-SLAM |
| ICP threshold $\sigma$ | 2.0 m, fixed | 2.0 m, fixed | `image_deskew.fixed_sigma` |
| Motion model | "car" | "car" | `image_deskew.model` |
| Detector | SURF, Hessian 100 | same | `image_deskew.detector` |
| Sub-pixel lifting | on | on | `image_deskew.subpixel` |
| Near-floor filter | 0.05 m, < −10°, < 5 m | same | `image_deskew.stuck_*` |
| Intensity scale | Hesai 1.0, Ouster 255/1024 | same | `image_deskew.intensity_scale` |
| Two starting points | 5°, no margin | same | `image_deskew.two_start_deg`, `two_start_margin` |
| RANSAC / fit thresholds | 0.30 m / 0.10 m, soft-ℓ1 0.05 m, ≥ 10 inliers | same | `intensity_deskew.py` |
| Loop closure | MapClosures defaults, overlap 0.4, `top_k` 1 | same | `loop_closer` |

The paper configuration (KISS-SLAM defaults) is used for every comparison. The "indoor detail" configuration was used
once for the staircase (#063).

## 12. What the method does not contain

- No IMU, gravity or vertical prior; no information matrices from the ICP (all graph edges have $I_6$).
- No intensity in the ICP cost. An intensity-weighted ICP (ColoredICP in loop closure) and intensity-based point
  selection exist from an earlier, closed approach (selecting the 70 % brightest points made trajectories about 3× worse,
  STATUS §5) and are off.
- No degeneracy detection or handling in the ICP. Per-scan geometric statistics are logged for diagnostics only.
- Options implemented and tested but **off** by default:

  | Option | Entry |
  |---|---|
  | Per-point deskew along the fitted curve | #033 |
  | Ray-cast panorama | #034 |
  | Near-field scale correction | #039 |
  | Plausibility gate | #054–#055 |
  | Range-image motion as fallback or third start | #056–#058 |
  | Rotation smoothing / translation-only / rotation-only ablations | #047 |
  | Switch margin | #062 |
  | Image motion in a parallel process (same trajectory, faster) | #042 |

## 13. Evidence, in brief (official protocols, 4 seeds, 16 sequences; details in the log)

- **Against upstream KISS-SLAM:** better on all 16 sequences on every metric (#059, #061).
- **Against KISS-SLAM without deskew**, the fair baseline (#045): APE −32 %, per-second translation error −38 to
  −42 % (Wilcoxon p ≤ 0.004). Per-second rotation error is a **tie** (8 vs 8 sequences) (#061).
- It rescues sequences where KISS fails: underground_hard 12.8 → 0.09 m, keble-college-03 9.5 → 0.10 m (#052), and
  Blenheim with two starts 3.98 → 0.28 m (#059).
- **Staircase:** solved only with the finer map (APE 0.48 m, and per-second errors better than no deskew with the same
  map, #063).
- **Map sharpness** against survey maps: the sharpest map in 3 of 4 sequences (#048–#051, 1 seed).
- **Speed:** 13.8 scans/s on a desktop in parallel mode (#042).

## 14. Known limitations (open)

- Vertical drift on long routes (Bodleian: the APE is almost all height), no gravity reference (#060).
- Systematic overestimation of the path length, +13 % on average, cause unknown (open_tasks Β.2).
- Rotation is not better than without deskew (#059, #061).
- Few-ring sensors (16 rings, NTU VIRAL: 31 % of scans without image motion) and scenes with poor intensity texture
  (Hilti UZH: 50 %).


### Σταθερό σ = 2.0 στον ICP (#031, #081) — τεκμηρίωση (5/10)

Στον KISS το σ (πύλη αντιστοίχισης 3σ, κλίμακα του πυρήνα Geman–McClure) προσαρμόζεται ανά σάρωση από την απόσταση αρχικής θέσης–αποτελέσματος. Με αρχική θέση την κίνηση της εικόνας η
απόσταση αυτή είναι μικρή και το σ πέφτει ~4×· ο ICP τότε δεν διορθώνει τη μικρή συστηματική απόκλιση της εικόνας (#031: church_02 APE 0.218 → 0.130 m με σ = 2.0). Στο ablation (#081, 27 × 4) το
προσαρμοστικό σ είναι ισοπαλία κατά μέσο όρο αλλά χειρότερο όπου η εικόνα είναι αναξιόπιστη (NTU +14…+73 %, Office_Mitte_1 +26 %)· γι' αυτό η μέθοδος κρατά σ = 2.0 σταθερό. Annealing μέσα στη
σάρωση: ανοιχτό (open_tasks).


## 15. Αλλαγές μετά τις 30/9 (#086–#163) — τι είναι στην προεπιλογή και τι επιλογή

**Στην προεπιλογή (`after_091`, `base092`):** (α) **upright SURF** (χωρίς προσανατολισμό — το πανόραμα δεν περιστρέφεται, #086)· (β) **καθοδηγούμενη αντιστοίχιση** σε παράθυρο ±40 στηλών γύρω από
τη μετατόπιση της προηγούμενης σάρωσης, μόνο σε αργή στροφή, αλλιώς brute force (#087–#089, C++ `_guided_match`)· (γ) εφεδρεία εικόνας απόστασης μόνο όταν αποτυγχάνει η intensity (#078). Όλα τα
υπόλοιπα όπως στις §3–§10.

**Υποψήφιες προσθήκες για το RA-L (5/10):**
- **Β — προσαρμοστική ανάμειξη μόνο για το deskew** (#131, #137–#140). Για τη σάρωση k η κίνηση του deskew είναι slerp / γραμμική ανάμειξη της κίνησης της εικόνας M_k και της σταθερής ταχύτητας C_k
  (το βήμα του ICP της προηγούμενης σάρωσης), με βάρος εικόνας w = v_C / (v_M + v_C), όπου v το μέσο τετραγωνικό σφάλμα στροφής κάθε πρόβλεψης έναντι του βήματος του ICP στις 20 προηγούμενες
  σαρώσεις (αιτιακό, χωρίς κατώφλι). Η αρχή του ICP μένει η M_k: η ανάμειξη και στην αρχή βλάπτει το drone (#137). Ισχύει και για κίνηση από την εικόνα απόστασης. Βάρος εικόνας: φορητή ~0.95,
  drone ~0.4–0.8, όχημα ~0.3.
- **Γ — εφεδρεία KISS σε σειρά αποτυχιών** (#143, #146, #150–#151). Όταν αποτυγχάνουν και οι δύο εικόνες, η σάρωση παίρνει deskew και αρχή από τη σταθερή ταχύτητα (όπως ο KISS) από την 4η
  συνεχόμενη αποτυχία· οι μεμονωμένες αποτυχίες μένουν «καμία κίνηση». Το 4 από τα δεδομένα: μέγιστη σειρά αποτυχιών στη γρήγορη περιστροφή 3, σε ύψος (spms) έως 799.

**Δοκιμασμένα και εκτός μεθόδου:** κανονικοποιήσεις intensity (#118–#126), τομείς (#093, #132–#144), μεγαλύτερη βάση k−2 (#090, #130–#133), σύντηξη απόστασης (#090, #132–#152), «ακίνητα» (#130–#135),
whitening / bearings / cross-check (#093, #087, #132), deskew της σάρωσης που κερδίζει η σταθερή ταχύτητα (#163), Β.9 joint (#115, #155–#160, ανοιχτό).


## 16. Επιλογές για όχημα και πραγματικό χρόνο (6/10–8/10, #170–#225) — εκτός της κλειδωμένης μεθόδου

**ΑΠΟΦΑΣΗ Μ.Τ. 7/10:** και οι δύο είναι παραλλαγές / επιλογές, όχι προεπιλογή.

- **Στοίβα πραγματικού χρόνου** (#186, #201, #202): πανόραμα ×4 στις 128 δέσμες (`--panorama-up=4`), η ίδια εικόνα σε C++ (`KISS_IMAGE_CPP=all`: RANSAC, προσαρμογή soft-L1 LM, scatter — αλλάζει
  το αποτέλεσμα όσο ένας άλλος σπόρος), εικόνα 4 σαρώσεις μπροστά (`KISS_IMAGE_AHEAD`), κρυφή μνήμη εικόνας απόστασης (`KISS_RANGE_CACHE`), KISS-ICP χωρίς GIL (περιβάλλον `kiss-slam-gil`,
  `baselines/kiss_icp_gil.patch`). Boreas 1.07× του πραγματικού χρόνου· στα 8 Boreas ίδια οδομετρία με τη Γ (RTE 0.403 / 0.405 %, #225).
- **Διαμόρφωση οχήματος** (#211–#217, `--cv-blend-use=both --two-start-trans=0.2`): (α) η ανάμειξη του §15 Β και για την **αρχή** του ICP, όχι μόνο για το deskew· (β) **σκανδάλη μετατόπισης**: δύο αρχές
  επιβάλλονται όταν οι μετατοπίσεις εικόνας και σταθερής ταχύτητας διαφέρουν περισσότερο από 20 % της μεγαλύτερης και η ταχύτητα ξεπερνά 0.2 m / σάρωση. Λόγος: σε αυτοκινητόδρομο τα μοτίβα του
  δρόμου στα 8–9 m μένουν ακίνητα στην εικόνα και η μετατόπιση της εικόνας πέφτει ~0 (#212). Στα drone βλάπτει η αρχή από την ανάμειξη, όχι η σκανδάλη (#222, #226) — μόνο για όχημα. Ανοιχτό όριο (#227): σε εικόνα «κολλημένη» με σιγουριά και γεωμετρία εκφυλισμένη κατά μήκος η
  ανάμειξη (βάρος μόνο από τη στροφή) φρενάρει την αρχή ×(1−w) ανά σάρωση και η σκανδάλη, που συγκρίνει την ανάμειξη με το C, δεν πυροδοτεί (Sejong01).
- **Διόρθωση ανύψωσης δεσμών** (`--elev-offset=DEG`, #219 / #221): διαγνωστικό· η ολίσθηση ύψους του Boreas είναι μεροληψία βαθμονόμησης ~+0.1° κοινή σε όλες τις μεθόδους. Για το paper δεν εφαρμόζεται.
- **Δοκιμασμένα χωρίς κέρδος:** φίλτρο «κολλημένων» αντιστοιχίσεων (`--stuck-adaptive`, #213: διορθώνει την εικόνα, πενταπλασιάζει τις αποτυχίες), ανάμειξη με βάρος inliers (#203), ORB / AKAZE (#181–#185).


## 17. Εναλλακτικές της εικόνας που δοκιμάστηκαν και ΔΕΝ μπήκαν (8/10, #230–#236, κλάδος `matching_intensities`)

- **Αντιστοίχιση:** ροή KLT 1D κατά μήκος κάθε δακτυλίου σε πανόραμα χωρίς μεγέθυνση (#230: χειρότερη, αγνοεί την κατακόρυφη μετατόπιση)· ροή KLT 2D (pyramidal LK, γωνίες Shi–Tomasi, #231: ≈ SURF)·
  upright ORB (#231: υποσχόμενο σε ένα σπόρο, ανοιχτό).
- **Μοντέλο κίνησης:** κυβικό (#234: χειρότερο — περισσότεροι άγνωστοι από τις αντιστοιχίσεις)· τα GT δείχνουν ότι η σταθερή ταχύτητα δεν ταιριάζει ποτέ, ότι το «car» είναι σωστό για τη φορητή και ότι η
  επιτάχυνση μετατόπισης («ca») θα ταίριαζε στα οχήματα (#233).
- **Υπο-πανοράματα** 180° με επικάλυψη 90° (#232): κάθε ζεύγος απέχει μία περίοδο, άρα δεν μετρούν τοπική ταχύτητα· όχι καλύτερα από την ολική προσαρμογή.
- **Ανθεκτική εκτίμηση:** MAGSAC++ (για 3D–3D), GNC-TLS, προσαρμογή δύο μοντέλων (#235 / #236): ίδια με RANSAC, πιο αργά. Το RANSAC σταματά νωρίς (πολλά inliers) και η προσαρμογή χρόνου μετά είναι ήδη
  ανθεκτική (soft-L1).

