# KISS-SLAM — Intensity-Aided ICP (PhD / Taoulai)

## 📍 Πού είμαστε (STATUS) και εργαστηριακό ημερολόγιο

**Διάβασε ΠΡΩΤΑ το [`docs/STATUS.md`](docs/STATUS.md)**: τι ξέρουμε (συμπεράσματα με
ημερομηνία), πού στοχεύουμε, τι είναι κλειστό, και λεξιλόγιο. Είναι η σύντομη εικόνα του έργου.

Σύνοψη της πορείας για τον Μ.Τ. (19/9, ενημερωμένη 21/9, 25/9 και 29/9 — §7δ): [`docs/summary_for_MT.md`](docs/summary_for_MT.md).
Το §7β της σύνοψης έχει τα νεότερα ευρήματα (#033–#039) — **πρώτο απ' όλα: το ATE ενός run δεν είναι μετρήσιμο
μέγεθος** (σ = 0.02–0.04 m), οπότε κάθε σύγκριση γίνεται με RPE/μήκος διαδρομής ή με 4+ runs.

**Εκκρεμότητες και ανοιχτά προβλήματα (29/9, ενημερώσεις ως 3/10): [`docs/open_tasks.md`](docs/open_tasks.md)** — από εκεί ξεκινά η επόμενη δουλειά.

**Η μέθοδος αναλυτικά (εξισώσεις, κάθε επιλογή με την εγγραφή της): [`docs/method_description.md`](docs/method_description.md).** Ερώτημα
θεωρητικής αξιολόγησης για ειδικό: [`docs/review_prompt_registration_deskew.md`](docs/review_prompt_registration_deskew.md).

Μετά το [`docs/experiment_log.md`](docs/experiment_log.md), που περιέχει:
- τον **Οδηγό εκτέλεσης** (πού/πώς τρέχει το pipeline, όλες οι CLI παράμετροι & config),
- το **ιστορικό πειραμάτων** αναλυτικά (Στόχος → Μέθοδος → Αποτέλεσμα → Συμπέρασμα, νεότερα πρώτα).

**Σειρά γραφής όταν κλείνει ένα πείραμα ή παίρνεται απόφαση:** πρώτα μία–τρεις γραμμές με
ημερομηνία στο `STATUS.md` (σε απλά λόγια, νούμερα σε παρένθεση), μετά η πλήρης εγγραφή στο
log. Ένα σημείο ανά είδος πληροφορίας — μην αντιγράφεις ολόκληρα συμπεράσματα σε δύο αρχεία.

Συμπεράσματα παίρνουν ✅ μόνο μετά από ρητή επικύρωση του Λ.Γ., με αρχικά και ημερομηνία
(«✅ Λ.Γ. 18/9»)· αλλιώς ⏳. Αποφάσεις και ιδέες του Λ.Γ. καταγράφονται ως «ΑΠΟΦΑΣΗ Λ.Γ.» / «Ιδέα Λ.Γ.».
Ορολογία: «τροχιά» / «θέση και στροφή», όχι «πόζα»· «υπολογίζεται», όχι «βγαίνει».

## Στόχος έρευνας

**Ιδέα:** αξιοποίηση του intensity ώστε να λυθούν τα datasets όπου δυσκολεύεται ο KISS. Η πρώτη υλοποίηση
(φιλτράρισμα «refine») είναι του Μ.Τ. Στα κείμενα γράφεται απλώς «ιδέα», χωρίς έμφαση σε ποιον ανήκει.

Χρήση του **Intensity** των LiDAR σημείων για ενίσχυση του ICP σε γεωμετρικά degenerate
συνθήκες (σκάλες, διάδρομοι), ώστε να μειωθεί το drift/failure έναντι του Ground Truth.

## Περιβάλλον

Το project τρέχει σε **τρία** μηχανήματα (πλήρεις οδηγίες: `docs/experiment_log.md`):

- **Linux** (`photogrammetrylinux`), conda env `manos_kissslam_for_edit`,
  source `/home/photogrammetrylinux/kiss-slam-for-edit`.
- **macOS arm64**, conda env `kissslam`. ⚠️ Απαιτεί `scikit-build-core<1.0` και numpy
  χτισμένο με OpenBLAS (όχι Accelerate).
- **Linux 2** (`photogrammetry`), conda env `kiss-slam-main`, source `/home/photogrammetry/Kiss_SLAM-main`
  (git → GitHub **`ETaoulai/KISS_SLAM_PHD_work_4`**, private, remote `origin`, από 2/10: `main` = όλη η δουλειά ως το #091 (`70a5947`, ο κλάδος `rotation_bearing`).
  Τα παλιά: `KISS_SLAM_PHD_work_3` = remote `archive3` (κλάδοι `main` ως το #080, `after_080`, `rotation_bearing`)· `KISS_SLAM_PHD_work_2` = remote `archive2` (κλάδοι `after_two_start`, `vertical_drift`, `intensity_norm`, `vertical_constraint`, `fast_fallback`)·
  `KISS_SLAM_PHD_work` = remote `archive` (κλάδοι `main` … `gating`, σταματά στο `fa86f3b`).)
  **Ο `main` δεν αλλάζει απευθείας (απόφαση Μ.Τ. 26/9):** κάθε νέα δουλειά σε δικό της κλάδο από τον `origin/main` — τρέχων `after_091` (2/10).
  Προσοχή: `push.default = upstream` — ένας κλάδος που παρακολουθεί τον `origin/main` σπρώχνει στον `main`· νέοι κλάδοι με `git push -u origin <κλάδος>:<κλάδος>` (με ρητό όνομα· χωρίς αυτό ένας κλάδος από τον `origin/main` πάει στον `main`). OpenCV χτισμένο με SURF.
  Δεδομένα στο `/media/photogrammetry/A26C3DDF6C3DAF431/data/` (όχι στο `data/` του repo). Νέα datasets (NCD 2020 long / dynamic_spinning,
  Hilti 2021, NTU VIRAL) οργανωμένα ως σύνδεσμοι στο `/home/photogrammetry/kiss_data/` (ext4) — κατάσταση: `docs/datasets.md`.

## Δομή

- `kiss_slam/` — ο τρέχων (intensity-aware) κώδικας.
- `kiss_slam/original_slam_files/` — **τρεις γενιές, όχι μία.** Πρόσεχε ποια συγκρίνεις:
  - `slam_original.py` / `pipeline_original.py` → **το γνήσιο upstream** KISS-SLAM.
  - `slam.py` / `pipeline.py` → **ενδιάμεση** έκδοση: upstream + τα per-frame διαγνωστικά
    (`_compute_icp_metrics`, `icp_metrics.csv`). **Δεν** είναι upstream.
  - Για «τι πρόσθεσε το intensity» → diff έναντι `original_slam_files/slam.py`.
    Για «τι αποκλίνει από το upstream» → diff έναντι `slam_original.py`.
- `configs/` — έτοιμα configs (`indoor_fast`, `indoor_detail`). Δεν ορίζουν `out_dir`
  επίτηδες, ώστε να δουλεύει το `KISS_SLAM_OUT_DIR`.
- `scripts/run_ab.sh` — τρέχει και τους δύο βραχίονες A/B στο `runs/`.
- `scripts/evaluate_gt.py` — ATE / RPE / z-προφίλ έναντι του Ground Truth.
- `scripts/evaluate_official.py` — **η ΜΟΝΗ αξιολόγηση από 25/9 (#061, ΑΠΟΦΑΣΗ Μ.Τ.):** πρωτόκολλο evo του Oxford Spires (APE `--align --t_max_diff 0.01`,
  RPE 1 m / 1 s), κάθε θέση στη στιγμή που αντιπροσωπεύει, **χωρίς μετατόπιση χρόνου**. Για όλο τον πίνακα: `results_table.py` (προεπιλογή).
  Τα `evaluate_ncd.py` / `evaluate_gt.py` / `results_table.py --legacy` μόνο για αναπαραγωγή παλιών πινάκων.
- **Μέθοδος του paper (ΑΠΟΦΑΣΗ Μ.Τ. 29/9): δύο αρχές + εφεδρεία εικόνας απόστασης** (`surftworangefb`)· ablation στο #081, `docs/results_ablation_081.md`.
  Runs από 29/9 στον εξωτερικό SSD (exFAT, `~/kiss_runs_ssd`, χωρίς symbolic links): `results_table.py --runs=/home/photogrammetry/kiss_runs_ssd`.
- **Επιλογές, όχι προεπιλογή (ΑΠΟΦΑΣΗ Μ.Τ. 28/9):** εικόνα απόστασης ως εφεδρεία (`--range=fallback`, #069/#071· μέρος της μεθόδου του paper) και βάρος στροφής του γράφου
  (`--rotation-weight=100`, #067/#070)· βραχίονες `surftworangefb`, `*rw` στο `results_table.py --all-arms`.
- **Βραχίονες σύγκρισης (ΑΠΟΦΑΣΗ Μ.Τ. 25/9):** KISS-SLAM · KISS-SLAM χωρίς deskew · i3 + SIFT · i3 + SURF · i3 + SURF δύο αρχές · δύο αρχές με
  περιθώριο αλλαγής 2 % / 4 % (#062). Όχι indoor_detail, όχι ablations (εξομάλυνση στροφής, μόνο μετατόπιση) — `results_table.py --all-arms` για όλα.
- `baselines/` + `scripts/run_baseline.py` / `lio_to_tum.py` / `extract_lio_topics.py` — οι άλλες μέθοδοι της σύγκρισης (#083: GenZ-ICP, MAD-ICP,
  CT-ICP, Traj-LO, FAST-LIO2, COIN-LIO), με pinned commits και patches· `baselines/README.md`.
- `scripts/evaluate_hilti.py` — επίσημο πρωτόκολλο του Hilti SLAM Challenge 2021 (τροχιά IMU, pole/prism/imu, 1 s, SE(3), APE)· `docs/datasets.md`.
- `scripts/evaluate_ntu.py` — επίσημο πρωτόκολλο του NTU VIRAL (σώμα + πρίσμα 0.40 m, 0.05 s, SE(3), ATE, πληρότητα)· `docs/datasets.md`.
- `scripts/dump_failed_matches.py` — εικόνες (πανοράματα + αντιστοιχίσεις) κάθε σάρωσης όπου αποτυγχάνει η κίνηση της εικόνας →
  `/home/photogrammetry/kiss_runs/failed_matches/` (ζήτημα Μ.Τ. 28/9)· κατά το run: `run_ncd.py --save-failed`.
- `scripts/eval_motion_npz.py` — σφάλμα ενός αποθηκευμένου npz κίνησης έναντι GT, χωρίς επανεκτίμηση.
- `scripts/measure_variance.sh` — διασπορά της μεθόδου (4 σπόροι × 2 τρόποι κατασκευής)· **τρέξ' το πριν από κάθε σύγκριση**.
- `scripts/analyze_rig_mask.py` — τι είναι καρφωμένο στον σαρωτή (σκιές διάταξης, σιλουέτα χειριστή).
- **Solid-state (2–3/10, #095–#110):** `kiss_slam/tools/livox.py` (Livox `CustomMsg`, ROS1), `kiss_slam/tools/ros2bags.py` (ROS2 bags χωρίς ορισμούς τύπων, σε φακέλους)·
  `scripts/solid_state_check.py`, `solid_state_accum.py`, `mid360_check.py` (εικόνα offline), `score_run_gyro.py`, `evaluate_tiers.py`· configs `livox_voxel{025,01}*.yaml`·
  δεδομένα στον εξωτερικό SSD (`kiss_data_ssd/tiers/`, `kiss_data_ssd/hard_pcl_loc/`, `docs/datasets.md`). Επιλογές `run_ncd.py`: `--motion-file`, `--deskew-from=cv`,
  `--two-start-kiss`, `--range=validate`, `--two-start-always`, `--voxel=auto`, `--panorama-width=auto --panorama-up=auto`, `--detect-scale`.
- `kiss_slam/tools/ncd_pcd.py` — reader του Newer College 2020 (`.pcd` του Ouster, με intensity/ring/απόλυτο χρόνο).
- `scripts/run_ncd.py` / `scripts/evaluate_ncd.py` — ένας βραχίονας (kiss / sift / surf) στο Newer College / αξιολόγηση έναντι GT.
- `data/church_02_cut.bag` — test dataset (Oxford Spires christ-church-02, 240 s, 2402 scans).
- `gt/`, `runs/`, `data/`, `build/` — gitignored.

## Τρεις παγίδες

- **Ο reader του rosbag είναι ΣΕΙΡΙΑΚΟΣ.** `ds[k]` χωρίς να έχουν διαβαστεί τα προηγούμενα επιστρέφει **άλλη σάρωση**.
  Κάθε script που δειγματοληπτεί σαρώσεις διαβάζει από το 0 και επεξεργάζεται επιλεκτικά.
- **Linux 2: ο δίσκος δεδομένων NTFS είναι ΜΟΝΟ για ανάγνωση.** Ο οδηγός `ntfs3` κάνει kernel BUG σε εγγραφές (η διεργασία πεθαίνει,
  μένει zombie ως την επανεκκίνηση· και στον πυρήνα 7.0.0-34). Κάθε έξοδος runs / πινάκων → `/home/photogrammetry/kiss_runs/` (ext4)·
  μεταφορά στον δίσκο δεδομένων μόνο με επαληθευμένο αντίγραφο (#046, #050, #052· απόφαση Μ.Τ. 24/9).
- **Καμία σύγκριση ATE με ένα run.** Η τυπική απόκλιση είναι 0.02–0.04 m (#037). Χρησιμοποίησε RPE και μήκος
  διαδρομής (60–100× σταθερότερα) ή 4+ runs με μέσο ± σ.

## Σημείωση απόδοσης

Το KISS-ICP **δεν χρησιμοποιεί KD-tree**: κάνει nearest-neighbour μέσα σε voxel hash map.
Κάθε `scipy.spatial.KDTree` σε αυτό το repo είναι δική μας προσθήκη και είναι το βασικό
κόστος ανά frame. Βλ. `docs/experiment_log.md`, εγγραφή #005, σημείο 5.
