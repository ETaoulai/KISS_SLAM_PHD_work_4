# 🧪 Εργαστηριακό Ημερολόγιο — Intensity-Aided ICP για KISS-SLAM

> 📍 **Σύντομη εικόνα πρώτα:** [`docs/STATUS.md`](STATUS.md) — πού είμαστε, τι ξέρουμε, τι έπεται.
> Εδώ είναι η **αναλυτική** καταγραφή κάθε πειράματος και ο οδηγός εκτέλεσης.

> **Concept:** Αξιοποίηση του **Intensity** των LiDAR σημείων για ενίσχυση του ICP σε
> **γεωμετρικά degenerate** συνθήκες (σκάλες, διάδρομοι, επίπεδες επιφάνειες), ώστε να
> μειωθεί το drift / failure και άρα το σφάλμα ως προς το Ground Truth.
>
> **Test dataset:** Oxford Spires — `2024-03-18-christ-church-02` (indoor↔outdoor,
> ανέβασμα/κατέβασμα σκάλας), κομμένο rosbag → `data/church_02_cut.bag`.

> ⚠️ **Δύο κανόνες που ισχύουν για ΚΑΘΕ εγγραφή πριν από το #037** (21/9):
> 1. **Το ATE ενός run δεν είναι μετρήσιμο μέγεθος** (σ = 0.023–0.041 m, δηλαδή 20–44 %· #037). Όπου παρακάτω
>    συγκρίνονται δύο βραχίονες με ένα run ο καθένας και διαφορά < ~0.05 m, η σύγκριση **δεν στοιχειοθετείται**.
>    Σταθερά μέτρα: **RPE και μήκος διαδρομής** (σ 60–100× μικρότερο).
> 2. **Ο reader του rosbag είναι ΣΕΙΡΙΑΚΟΣ:** `ds[k]` χωρίς να έχουν διαβαστεί τα προηγούμενα επιστρέφει **άλλη
>    σάρωση** (#038). Κάθε script που δειγματοληπτεί διαβάζει από το 0 και επεξεργάζεται επιλεκτικά.

---

## 🖥️ Οδηγός εκτέλεσης

### Πού τρέχει

Το project τρέχει πλέον σε **τρία** μηχανήματα:

| | Linux (Μανόλης) | macOS arm64 (Λάζαρος) | Linux 2 (από 23/9) |
|---|---|---|---|
| **Host** | `photogrammetrylinux`, x86_64 | Apple Silicon (M-series) | `photogrammetry`, x86_64, 48 πυρήνες, 62 GB |
| **Source dir** | `/home/photogrammetrylinux/kiss-slam-for-edit` | `/Users/lazaros/Code/PhD/Taoulai/Kiss_SLAM` | `/home/photogrammetry/Kiss_SLAM-main` (git → GitHub `ETaoulai/KISS_SLAM_PHD_work_2`, private· από 25/9, πριν `KISS_SLAM_PHD_work`) |
| **Conda env** | `manos_kissslam_for_edit` (Py 3.11) | **`kissslam`** (Py 3.11) | `kiss-slam-main` (Py 3.11) |
| **Build dir** | `build/cp311-cp311-linux_x86_64/` | `build/cp311-cp311-macosx_11_0_arm64/` | `build/cp311-cp311-linux_x86_64/` |
| **Ενεργοποίηση** | `conda activate manos_kissslam_for_edit` | `conda activate kissslam` | `conda activate kiss-slam-main` |

Στο Linux 2 τα δεδομένα είναι στον δίσκο `/media/photogrammetry/A26C3DDF6C3DAF431/data/` (NTFS, ο κύριος δίσκος έχει
~30 GB ελεύθερα), οργανωμένα 23/9 — χάρτης στο `data/README.md` εκεί: `newer_college/` (οι 7 ακολουθίες του paper του
KISS-SLAM, #041), `oxford_spires/2024-03-18-christ-church-0{2,3}/` (**ολόκληρες** ακολουθίες: `rosbag/` + `ground_truth/`
με GT και vilens-slam σε TUM), `runs/`, `software/` (wheel OpenCV με SURF). Τα church bags εκεί είναι οι **πλήρεις**
εγγραφές (church_02: 2 bags, 592 s· church_03: 312 s), όχι το κομμένο `church_02_cut.bag` (240 s) των πειραμάτων ως #039 —
άρα και ο στόχος §2.3 του STATUS (ολόκληρο το christ-church-02) έχει πλέον δεδομένα. Το `data/` και το `runs/` του repo δεν
υπάρχουν σε αυτό το μηχάνημα.

Τα δύο build dirs συνυπάρχουν (το `{wheel_tag}` του scikit-build-core τα κρατά χωριστά),
οπότε **δεν συγκρούονται** αν συγχρονιστεί ο φάκελος.

Εγκατεστημένα στο macOS env: `kiss-icp 1.3.0`, `map_closures 2.1.0`, `open3d 0.20.0`,
`rosbags 0.11.5`, `numpy 2.2.6 (OpenBLAS)`, `scipy 1.17.1`, `kiss-slam 0.0.2` (editable
από αυτό το source).

> ⚠️ **Δύο παγίδες ειδικά για macOS** (τεκμηρίωση: εγγραφή #003):
> 1. **`scikit-build-core` πρέπει να είναι `<1.0`** (χρησιμοποιείται `0.12.2`, ίδιο με το
>    Linux). Η 1.0 καταργεί το `cmake.minimum-version` που δηλώνει το `pyproject.toml`
>    και το build αποτυγχάνει στο metadata generation.
> 2. **Το numpy πρέπει να είναι build με OpenBLAS, όχι Accelerate.** Το pip wheel στο
>    macOS arm64 συνδέεται με Accelerate, που παράγει **ψευδείς** `overflow/invalid value
>    encountered in matmul` προειδοποιήσεις (τα αποτελέσματα είναι σωστά). Λύση:
>    `mamba install -c conda-forge "numpy=2.2.*" "libblas=*=*openblas"`.

### Εγκατάσταση (developer / editable mode)

**Linux:**
```bash
conda activate manos_kissslam_for_edit
cd /home/photogrammetrylinux/kiss-slam-for-edit
make editable          # ισοδύναμο με: pip install --no-build-isolation -ve .
```

**macOS arm64** (αναπαράξιμη συνταγή — δοκιμασμένη 2026-09-17):
```bash
mamba create -y -n kissslam python=3.11
conda activate kissslam

pip install "kiss-icp>=1.2.3" "map_closures>=2.0.2" "open3d>=0.19.0" \
            numpy PyYAML "pydantic>=2" tqdm pydantic-settings rosbags typer matplotlib scipy
pip install "scikit-build-core==0.12.2" pyproject_metadata pathspec pybind11 ninja cmake   # ⚠️ ΟΧΙ 1.x

# numpy με OpenBLAS αντί για Accelerate (αλλιώς ψευδείς matmul warnings)
mamba install -y -c conda-forge "numpy=2.2.*" "libblas=*=*openblas"

cd /Users/lazaros/Code/PhD/Taoulai/Kiss_SLAM
MACOSX_DEPLOYMENT_TARGET=11.0 pip install --no-build-isolation -ve . \
    --config-settings=cmake.define.USE_SYSTEM_EIGEN3=OFF \
    --config-settings=cmake.define.USE_SYSTEM_G2O=OFF \
    --config-settings=cmake.define.USE_SYSTEM_TSL-ROBIN-MAP=OFF
```
Τα `USE_SYSTEM_*=OFF` κάνουν fetch+build τα Eigen / g2o / SuiteSparse / tsl-robin-map
(ίδια ρύθμιση με τα επίσημα macOS wheels). Απαιτεί Xcode CLT. Διάρκεια ~3–4 min.
Χρειάζεται **μόνο** όταν αλλάζει C++· οι αλλαγές σε Python πιάνουν αμέσως.

Το `make editable` χρησιμοποιεί `scikit-build-core` με `editable.rebuild = true`, οπότε
οι αλλαγές σε C++ (`kiss_slam/kiss_slam_pybind/`) ξαναχτίζονται αυτόματα στο import.
Αλλαγές **μόνο σε Python** δεν χρειάζονται rebuild.

**Linux 2** (αναπαράξιμη συνταγή — 23/9· ίδιες εκδόσεις με το macOS env):
```bash
conda create -y -n kiss-slam-main python=3.11 && conda activate kiss-slam-main
pip install "kiss-icp>=1.2.3" "map_closures>=2.0.2" "open3d>=0.19.0" numpy PyYAML "pydantic>=2" tqdm \
            pydantic-settings rosbags typer matplotlib scipy pillow \
            "scikit-build-core==0.12.2" pyproject_metadata pathspec pybind11 ninja cmake   # ⚠️ scikit-build-core ΟΧΙ 1.x
pip install --no-build-isolation -ve . \
    --config-settings=cmake.define.USE_SYSTEM_EIGEN3=OFF \
    --config-settings=cmake.define.USE_SYSTEM_G2O=OFF \
    --config-settings=cmake.define.USE_SYSTEM_TSL-ROBIN-MAP=OFF
```

**OpenCV με SURF** (για `image_deskew.detector: surf`, #041). Το SURF είναι patented και λείπει από **όλα** τα wheels του pip
(`opencv-python`, ακόμη και `opencv-contrib-python`: «This algorithm is patented and is excluded»). Χτίζεται από τον κώδικα
(~20 min με 32 πυρήνες) και αντικαθιστά το `opencv-python`· το SIFT μένει ίδιο (ίδια σημεία):
```bash
ENABLE_CONTRIB=1 ENABLE_HEADLESS=1 CMAKE_ARGS="-DOPENCV_ENABLE_NONFREE=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF" \
    pip wheel --no-deps --no-binary opencv-contrib-python-headless "opencv-contrib-python-headless==5.0.0.93" -w wheels/
pip uninstall -y opencv-python && pip install --no-deps wheels/opencv_contrib_python_headless-5.0.0.93-*.whl
python -c "import cv2; cv2.xfeatures2d.SURF_create(); print('SURF OK')"
```
Έτοιμο wheel για Linux x86_64 / Py 3.11: `/media/photogrammetry/A26C3DDF6C3DAF431/data/opencv_contrib_python_headless-5.0.0.93-cp311-cp311-linux_x86_64.whl`.

### Βασική εντολή

```bash
kiss_slam_pipeline <DATA> [OPTIONS]
```

Για το δικό μας bag:

```bash
kiss_slam_pipeline data/church_02_cut.bag \
    --dataloader rosbag \
    --topic /hesai/pandar \
    --config kiss_slam.yaml \
    --refuse-scans
```

### Παράμετροι CLI (`kiss_slam/tools/cli.py`)

| Flag | Short | Default | Περιγραφή |
|---|---|---|---|
| `DATA` | — | *(required)* | Το αρχείο/φάκελος δεδομένων. Για `.bag` το dataloader μαντεύεται αυτόματα ως `rosbag`. |
| `--dataloader` | — | auto-guess | Ρητός dataloader (`rosbag`, `generic`, `ouster`, `mcap`, `kitti`, …). |
| `--topic` | `-t` | — | **Μόνο για rosbag.** Το PointCloud2 topic. Εδώ: `/hesai/pandar`. |
| `--config` | — | defaults | YAML config. Παράγεται με `kiss_slam_dump_config`. |
| `--visualize` | `-v` | `False` | Live Open3D visualizer. |
| `--refuse-scans` | `-rs` | `False` | Στο τέλος ξανα-διαβάζει όλα τα scans και φτιάχνει global occupancy map (2D+3D). **Διπλασιάζει τον χρόνο.** |
| `--n-scans` | `-n` | `-1` (όλα) | Πλήθος scans προς επεξεργασία. Χρήσιμο για γρήγορα smoke tests (`-n 200`). |
| `--jump` | `-j` | `0` | Από ποιο scan index να ξεκινήσει. Χρήσιμο για να στοχεύσουμε **μόνο το τμήμα της σκάλας**. |
| `--use-intensity` / `--no-use-intensity` | — | *(config)* | **A/B πειραματικός βραχίονας.** `--no-use-intensity` = vanilla KISS-SLAM baseline· `--use-intensity` = intensity-aided. Υπερισχύει του `intensity.enabled` στο YAML. Αν παραλειφθεί, ισχύει η τιμή του config (default: **disabled**). |
| `--image-deskew` / `--no-image-deskew` | — | *(config)* | **Η μέθοδος (#027–#032):** deskew κάθε σάρωσης με την κίνηση που μετρά η εικόνα intensity, η ίδια κίνηση ως αρχική θέση του ICP, σ σταθερό 2.0. Υπερισχύει του `image_deskew.enabled` στο YAML (default: **disabled**). Χωρίς `--motion-file` η κίνηση υπολογίζεται **online** από τη σάρωση (χρειάζεται `intensity` + `ring` στο bag· ~60 ms/σάρωση επιπλέον). Δεν συνδυάζεται με `--use-intensity` (ValueError). |
| `--image-detector` | — | *(config)* | Μόνο με `--image-deskew` (online): χαρακτηριστικά των πανοραμάτων, `sift` ή `surf` (#041). Το `surf` θέλει OpenCV με `OPENCV_ENABLE_NONFREE` (Εγκατάσταση). Υπερισχύει του `image_deskew.detector` (default `sift`). |
| `--motion-file` | — | *(config)* | Μόνο με `--image-deskew`: `.npz` με προϋπολογισμένες κινήσεις από το `scripts/precompute_i3_motion.py` (π.χ. `runs/i3_motion_car_sp_st0.05_floor.npz`) αντί για online εκτίμηση· δείκτης = αύξων αριθμός σάρωσης (να ξεκινά από το ίδιο scan, χωρίς `--jump`). Υπερισχύει του `image_deskew.motion_file`. |
| `--sequence` | `-s` | — | Μόνο για sequence-based dataloaders (KITTI κ.λπ.). Δεν χρειάζεται εδώ. |
| `--meta` | `-m` | — | Μόνο για Ouster pcap. |

### Παραγωγή / επεξεργασία config

```bash
kiss_slam_dump_config          # γράφει kiss_slam.yaml με τα defaults
```

Σημαντικά πεδία (βλ. `kiss_slam/config/config.py`):

| Πεδίο | Default | Σχόλιο για indoor/σκάλα |
|---|---|---|
| `out_dir` | `slam_output` | Φάκελος εξόδου. |
| `odometry.preprocessing.max_range` | 100.0 | **→ 50.0** για indoor (μειώνει αυτόματα και το voxel_size). |
| `odometry.preprocessing.deskew` | — | Το bag έχει per-point `timestamp` ⇒ αξίζει `true`. |
| `odometry.mapping.voxel_size` | `max_range/100` | 0.5 m στα 50 m range. |
| `local_mapper.voxel_size` | 0.5 | Ανάλυση του local map voxel grid. |
| `local_mapper.splitting_distance` | 100.0 | **→ 10–20 m** για indoor, αλλιώς όλη η σεκάνς γίνεται 1–2 nodes και δεν υπάρχουν loop closures. |
| `loop_closer.overlap_threshold` | 0.4 | Κατώφλι αποδοχής closure. |
| `loop_closer.top_k` | 1 | Πόσους υποψηφίους του ανιχνευτή επαληθεύει ο ICP· με > 1 κρατιούνται **όλοι** οι δεκτοί. 1 = upstream. (#014) |
| `loop_closer.max_height_disagreement` | null | Απόρριψη closure όταν η διαφορά ύψους που ορίζει διαφέρει από της odometry πάνω από τόσα m (κατά την κατακόρυφο του χάρτη). null = upstream. (#016) |
| `deskew_refine.passes` | 1 | Περάσματα deskew: >1 ⇒ deskew ξανά με την κίνηση της ίδιας της σάρωσης και νέος ICP. 1 = upstream. ⚠️ ασταθές (#017). |
| `local_mapper.splitting_height` | null | Νέος χάρτης και όταν το ύψος αλλάξει > τόσα m (κατά την κατακόρυφο από το έδαφος του MapClosures). null = upstream. (#014) |
| `pose_graph_optimizer.max_iterations` | 10 | g2o iterations. |
| **`intensity.enabled`** | **`false`** | **Ο διακόπτης A/B.** `false` = ακριβώς το upstream KISS-SLAM. |
| `intensity.keep_ratio` | 0.70 | Ποσοστό σημείων που κρατά το `_intensity_filter`. |
| `intensity.min_intensity` | 0.05 | Κατώφλι «bright». ⚠️ Βλ. εύρημα #5 — με αυτά τα δεδομένα είναι πρακτικά ανενεργό. |
| `intensity.lambda_geometric` | 0.90 | ColoredICP geometry↔photometry trade-off στο loop closure (1.0 = καθαρή γεωμετρία). |
| **`image_deskew.enabled`** | **`false`** | **Η μέθοδος (#027–#032):** κίνηση από την εικόνα intensity ως deskew + αρχική θέση ICP. `false` = upstream. Αποκλείει το `intensity.enabled`. Απαιτεί `preprocessing.deskew: true`. |
| `image_deskew.model` | `car` | Μοντέλο κίνησης μέσα στη σάρωση: `cv` σταθερή ταχύτητα (#019), `car` επιτάχυνση μόνο στη στροφή (#021), `ca` σταθερή επιτάχυνση (#020). |
| `image_deskew.subpixel` | `true` | Παρεμβολή σημείου/χρόνου μέσα στο pixel (#021). |
| `image_deskew.stuck_min` | 0.05 | Απόρριψη αντιστοιχίσεων με \|p − q\| < τόσα m («ίδιο σημείο» στο πλαίσιο του αισθητήρα, #025–#026). `null` = καμία. |
| `image_deskew.stuck_floor_only` | `true` | Το `stuck_min` μόνο στο κοντινό δάπεδο (#027): κάτω από `stuck_elev_deg` (−10°) και πιο κοντά από `stuck_range_m` (5 m). |
| `image_deskew.use_as_initial_guess` | `true` | Η κίνηση της εικόνας και ως αρχική θέση του ICP (`last_pose · M`, #030). `false` = μόνο deskew· από τη γραμμή εντολών `run_ncd.py --image-start=false` (#080). |
| `image_deskew.fixed_sigma` | 2.0 | Το σ του ICP σταθερό σε αυτή την τιμή, χωρίς προσαρμογή (#031). `null` = προσαρμοστικό σ του KISS. |
| `image_deskew.motion_file` | `null` | `.npz` από το `precompute_i3_motion.py`· `null` = online εκτίμηση (χρειάζεται `intensity` + `ring` ανά σημείο). |

### Τι παράγεται (`slam_output/<timestamp>/`, + symlink `latest`)

| Αρχείο | Περιεχόμενο |
|---|---|
| `<seq>_poses.<ext>` | Εκτιμώμενη τροχιά (KITTI/TUM format). |
| `<seq>_gt.<ext>` | Ground truth — **μόνο αν ο dataloader έχει `gt_poses`. Ο rosbag ΔΕΝ έχει ⇒ δεν γράφεται.** |
| `result_metrics.log` | ATE/RPE — **μόνο αν υπάρχει GT. Τώρα κενό από metrics.** |
| **`slam_config.yaml`** | **Ολόκληρο το `KissSLAMConfig`, άρα και το `intensity.*` ⇒ κάθε output dir δηλώνει σε ποιον βραχίονα A/B ανήκει.** |
| `icp_metrics.csv` | Per-frame διαγνωστικά (custom, βλ. `_ICP_CSV_FIELDS` στο `pipeline.py`). |
| `icp_metrics.png` | RMS/mean/max error, point counts, inlier ratio, adaptive σ. |
| `motion_metrics.png` | Frame-to-frame μετατόπιση/στροφή + model deviation. |
| `geometry_metrics.png` | Linearity/planarity/sphericity/condition/omnivariance. |
| `trajectory.png` | Top-down τροχιά με κόκκινες γραμμές στα loop closures. |
| `trajectory.g2o`, `local_maps/local_map_graph.g2o` | Pose graphs. |
| `local_maps/plys/*.ply` | Τα local maps ως point clouds. |
| `occupancy_grid/` | Μόνο με `--refuse-scans`. |

### Πού γράφονται τα αποτελέσματα

Default `out_dir = slam_output` (σχετικά με το cwd), με symlink `slam_output/latest`.

Το `KissSLAMConfig` έχει `env_prefix="kiss_slam_"`, οπότε:

```bash
KISS_SLAM_OUT_DIR=/tmp/my_run kiss_slam_pipeline ...
```

> ⚠️ **Προτεραιότητα (δοκιμασμένο):** αν το YAML config ορίζει `out_dir`, **αυτό κερδίζει**
> και η μεταβλητή περιβάλλοντος αγνοείται — το `load_config` περνά το YAML ως init kwargs
> στο pydantic-settings, και τα init kwargs έχουν μεγαλύτερη προτεραιότητα από τα env vars.
> Γι' αυτό τα `configs/*.yaml` **δεν ορίζουν** `out_dir`: έτσι η έξοδος κατευθύνεται ανά run
> από το `KISS_SLAM_OUT_DIR`, που είναι απαραίτητο για να μην μπερδεύονται οι δύο βραχίονες.

### Χρήσιμο smoke test (πριν από κάθε πλήρες run)

```bash
kiss_slam_pipeline data/church_02_cut.bag -t /hesai/pandar -n 200 --no-use-intensity
```

### Έτοιμα configs (`configs/`)

| Αρχείο | `mapping.voxel_size` | σημεία/scan | ms/frame (baseline) | Πότε |
|---|---|---|---|---|
| `configs/indoor_fast.yaml` | 0.5 | ~2.500 | ~46 | γρήγορες επαναλήψεις, debugging |
| `configs/indoor_detail.yaml` | **0.25** | ~5.970 | ~172 | **συμπερασματικά runs** |

Και τα δύο: `max_range=50`, `deskew=true`, `splitting_distance=15 m`, `intensity.enabled=false`
(ο βραχίονας ορίζεται από CLI). Τεκμηρίωση επιλογών: εγγραφή #004.

### A/B πείραμα (οι δύο βραχίονες)

```bash
# A — baseline (vanilla KISS-SLAM)
KISS_SLAM_OUT_DIR=runs/base kiss_slam_pipeline data/church_02_cut.bag -t /hesai/pandar \
    --config configs/indoor_detail.yaml --no-use-intensity

# B — intensity-aided
KISS_SLAM_OUT_DIR=runs/int  kiss_slam_pipeline data/church_02_cut.bag -t /hesai/pandar \
    --config configs/indoor_detail.yaml --use-intensity
```

Ίδιο config και στα δύο· **μόνο** η σημαία αλλάζει. Ο βραχίονας τυπώνεται στην αρχή
(`KissSLAM| Experiment arm: …`) και καταγράφεται στο `slam_config.yaml` του output dir.
Ο intensity βραχίονας είναι ντετερμινιστικός (seeded RNG), οπότε επαναλήψεις του ίδιου
βραχίονα δίνουν πανομοιότυπη τροχιά.

### Τρέχουσες εντολές (5/10)

Προεπιλογή (`base092`): `python scripts/run_ncd.py surf <bag/φάκελος> <out> --seed=<s> --config=<threads4.yaml> --two-start=5 --range=fallback [--parallel] <επιλογές της ακολουθίας από all_seqs.tsv>`.
Υποψήφιες: Β + `--cv-blend=adaptive --cv-blend-use=deskew`· Γ + `--cv-blend=adaptive --cv-blend-use=deskew --fallback=kiss --fallback-after=4`. Όλες οι επιλογές από 3/10: `CLAUDE.md`.
Β.9: από τον worktree `python scripts/worktree_run.py scripts/run_ncd.py surf ... --ct=joint --ct-lambda=0.1 --parallel` (κλάδος `b9_joint_new`, C++: `scripts/build_ct_accumulate.sh`).
Batches: systemd μονάδες (`systemd-run --user ... -p MemoryMax=… -p OOMPolicy=continue`), launchers στο `kiss_runs/`, runs στο `kiss_runs_ssd/`.

### Η μέθοδος με μία εντολή (`--image-deskew`, #027–#032)

```bash
# online: η κίνηση κάθε σάρωσης υπολογίζεται από την εικόνα intensity μέσα στο pipeline
KISS_SLAM_OUT_DIR=runs/church_02_img kiss_slam_pipeline data/church_02_cut.bag -t /hesai/pandar \
    --config configs/indoor_detail.yaml --image-deskew

# με προϋπολογισμένες κινήσεις (ίδιο αποτέλεσμα, χωρίς το κόστος του εκτιμητή)
KISS_SLAM_OUT_DIR=runs/church_02_img kiss_slam_pipeline data/church_02_cut.bag -t /hesai/pandar \
    --config configs/indoor_detail.yaml --image-deskew --motion-file runs/i3_motion_car_sp_st0.05_floor.npz
```

Ισοδύναμο με `scripts/precompute_i3_motion.py --model=car --subpixel --stuck=0.05 --floor-only` +
`scripts/run_i3_deskew.py … 0 init fixed` (τα scripts παραμένουν και δουλεύουν όπως πριν). Οι ρυθμίσεις της
μεθόδου (`image_deskew.*`, πίνακας παρακάτω) γράφονται στο `slam_config.yaml` του run. Στο τέλος τυπώνεται πόσες
σαρώσεις πήραν κίνηση από την εικόνα και πόσες έπεσαν στην ταυτοτική (`KissSLAM| image motion: …`).
Έλεγχος ισοδυναμίας: `python tests/test_image_deskew.py` (upstream με `enabled=false`· ίδιες θέσεις με το
override του `run_i3_deskew.py`· ίδιες κινήσεις online και από npz).

Μέτρηση διασποράς (#037) — πόσο αλλάζει το ATE όταν αλλάζει **μόνο** ο σπόρος του RANSAC:

```bash
bash scripts/measure_variance.sh                      # 4 σπόροι × 2 τρόποι κατασκευής, ~1 ώρα, ένα run τη φορά
bash scripts/measure_variance.sh "0 1 2" splat        # υποσύνολο
```

Κάθε (τρόπος, σπόρος) → `runs/var_<τρόπος>_s<σπόρος>`· στο τέλος πίνακας με το `evaluate_gt.py`. Χωρίς αυτό το νούμερο,
διαφορές ±0.02–0.05 m ανάμεσα σε παραλλαγές δεν κρίνονται.

Διόρθωση της μεροληψίας κοντινού πεδίου (#039), **online και αιτιακή** — στο config ή στο precompute:

```yaml
image_deskew:
  trans_min_range: 8.0      # m· null = ανενεργό (προεπιλογή)
  trans_mode: auto          # auto (τρέχων διάμεσος) | magnitude | vector
```
```bash
python scripts/precompute_i3_motion.py … --trans-min=8 --trans-mode=auto
python scripts/eval_motion_npz.py runs/<x>.npz gt/<seq>_gt-tum.txt runs/<run της ακολουθίας>
```

**Δύο αρχικές θέσεις για τον ICP — προεπιλογή από το #059** (`image_deskew.two_start_deg: 5`, απόφαση Μ.Τ. 25/9): όταν η κίνηση της εικόνας και η
σταθερή ταχύτητα διαφέρουν > 5° σε στροφή, η σάρωση καταχωρίζεται και από τις δύο αρχές και κρατείται το καλύτερο ταίριασμα στον τοπικό χάρτη (#057–#058).
`two_start_deg: null` (ή `run_ncd.py --two-start=none`) = μία αρχή, όπως κάθε αποτέλεσμα πριν από το #059. Προαιρετικά `range_motion: candidate | fallback`
(εικόνα απόστασης, #058), `gate_*` / `fallback` (έλεγχος αληθοφάνειας, #054–#055), `save_rejected_dir`.

Ανιχνευτής, σπόρος και παράλληλη εκτέλεση (#041, #042):

```yaml
image_deskew:
  detector: sift                # sift | surf (θέλει OpenCV με OPENCV_ENABLE_NONFREE)
  surf_hessian_threshold: 100.0 # μόνο surf: μεγαλύτερο = λιγότερα, ισχυρότερα σημεία (400 ≈ πλήθος του SIFT)
  surf_upright: false           # μόνο surf: χωρίς προσανατολισμό (U-SURF)
  parallel: false               # κλάδος fast_test: εικόνα σε χωριστή διεργασία, παράλληλα με τον ICP· ίδια τροχιά
  seed: 0                       # σπόρος του RANSAC του εκτιμητή (#037)
```
```bash
python scripts/precompute_i3_motion.py … --detector=surf [--surf-hessian=400] [--surf-upright]   # κατάληξη _surf[_h400][_up]
```
**Κλάδος `rotation_bearing` (#086–#091)** — προεπιλογές εκεί: `surf_upright: true`, `guided_matching_window: 40`, `guided_prediction: shift` (#088).
Θέλει το C++ module: `bash scripts/build_guided_match.sh` (αλλιώς έκδοση Python, πιο αργή).
```bash
python scripts/run_ncd.py … --no-upright --guided=none        # η μέθοδος όπως πριν από το #087
python scripts/run_ncd.py … --guided-predict=shift|motion|hybrid   # πρόβλεψη του παραθύρου (#088 / #089)· ανοιχτό ποια
python scripts/run_ncd.py … --surf-hessian=200 --panorama-up=4     # ταχύτητα σε 128 δέσμες (#091), επιλογές
python scripts/run_ncd.py … --oracle-deskew=<oracle_motion.npz>    # deskew από το GT (#086, διαγνωστικό)
python scripts/run_ncd.py … --rot-cv=0.05      # στροφή εικόνας σταθμισμένη με τη σταθερή ταχύτητα (#092, image_deskew.rotation_cv_weight)· όχι προεπιλογή
python scripts/analyse_rotation_blend.py <run> <oracle_motion.npz> …   # offline: σφάλμα στροφής ανά σάρωση με σταθμίσεις (#092)
python scripts/run_ncd.py … --panorama-width=auto --panorama-up=auto   # πανόραμα από τον αισθητήρα (#093): ίδια ακρίβεια, ×1.7 ταχύτερο· προτείνεται ως προεπιλογή
python scripts/run_ncd.py … --sectors=8 | --whiten=0.03,0.003,0.008 | --cross-check | --detect-scale=0.75   # #093, απορρίφθηκαν (επιλογές)
python scripts/analyse_motion_windows.py <oracle_motion.npz> <run> …   # offline: σφάλμα ανά σάρωση και σε 10 σαρώσεις (#093)· κρίνει μόνο αρνητικά
```
Στον κλάδο `fast_test` το RANSAC σταματά νωρίτερα και το fit χρόνου έχει αναλυτική Ιακωβιανή (#042)· για αναπαραγωγή
αποτελεσμάτων πριν από αυτόν: `intensity_deskew.RANSAC_CONF = None`, `intensity_deskew.FIT_JAC = "2-point"`.

Προαιρετική μάσκα pixel καρφωμένων στον σαρωτή (#036, απενεργοποιημένη από προεπιλογή):
`scripts/analyze_rig_mask.py [βήμα] [--bag=]` → `runs/rig_masks_<bag>.npz` (κλειδιά `narrow`, `edges`, `union`, `hole`) και
`docs/figures/rig_annotated.png`· μετά `precompute_i3_motion.py … --mask=runs/rig_masks_church02.npz --mask-key=union`
(κατάληξη `_mask-union` στο npz). Στον κώδικα: `intensity_deskew.PIXEL_MASK` (None = off).

### Newer College 2020 (Ouster OS1-64, #041)

Οι ακολουθίες του paper του KISS-SLAM. Λήψη: φόρμα στη σελίδα του dataset → σύνδεσμος Google Drive. Στο Linux 2:
`/media/photogrammetry/A26C3DDF6C3DAF431/data/newer_college/` (δομή του Drive: `2020/01_short_experiment/…`,
`2021/collection N - …/`), με `manifest.tsv` (αρχεία, id, μέγεθος) και `download.sh` (συνεχίζει διακοπείσες λήψεις·
το δημόσιο quota του Drive μπλοκάρει συχνά τα μεγάλα bags — τότε από τον browser, συνδεδεμένος).

```bash
cd <seq>/raw_format && for z in ouster_zip_files/*.zip; do unzip -q -n "$z" -d .; done   # → ouster_scan/*.pcd
python scripts/run_ncd.py <kiss|sift|surf> <seq dir> <out dir> [n_scans] [--config=<yaml>] [--seed=N] [--parallel]
python scripts/evaluate_ncd.py <seq dir> <out dir> [<out dir> …]      # ανά run και ανά βραχίονα (μέσος ± σ)
```
Reader: `kiss_slam/tools/ncd_pcd.py` (intensity × 255/1024, ring, **απόλυτος** χρόνος = χρονοσφραγίδα σάρωσης + `t`· GT ανά
σάρωση στο σύστημα του LiDAR). Το όνομα του out dir μέχρι το πρώτο «_» είναι ο βραχίονας (`sift_s2` → `sift`).
⚠️ Τα bags του 2021 (topic `/os_cloud_node/points`) μέσα από το `read_point_cloud_raw` θα έδιναν το `t` του Ouster
**σχετικό** (από την αρχή κάθε σάρωσης) και το intensity σε κλίμακα 0–~1100 — τα δύο προβλήματα του #041. Χρειάζονται
την ίδια προσαρμογή πριν τρέξουν με `--image-deskew`.

### Αξιολόγηση με το επίσημο πρωτόκολλο (evo, #061 — η μόνη αξιολόγηση από 25/9, ΑΠΟΦΑΣΗ Μ.Τ.)

Όπως το benchmark του Oxford Spires (`scripts/localisation_benchmark/*.py` του `ori-drs/oxford_spires_dataset`):
`evo_ape tum gt_lidar.txt <εκτίμηση>_tum.txt --align --t_max_diff 0.01` — APE μετατόπισης (RMSE) μετά από στερεή ευθυγράμμιση SE(3),
αντιστοίχιση χρονοσφραγίδων εντός 10 ms, **χωρίς μετατόπιση χρόνου**. Επιπλέον RPE του evo σε 1 m και σε 10 θέσεις (1 s).

```bash
python scripts/evaluate_official.py <GT> <run dir> [<run dir> …] --frame=spires|ncd2021|ncd2020 [--out=<dir>] [-v]
python scripts/results_table.py                        # όλες οι ακολουθίες, οι 7 βραχίονες → /home/photogrammetry/kiss_runs/results_official.md/.csv
python scripts/compare_arms.py /home/photogrammetry/kiss_runs/results_official.csv "<βραχίονας A>" "<βραχίονας B>"
```
**Χρόνος κάθε θέσης.** Η χρονοσφραγίδα της σάρωσης είναι το **πρώτο** σημείο της περιστροφής (Hesai, Ouster, pcd 2020)· μια σάρωση με
deskew εκφράζεται στο **τελευταίο** (kiss_icp 1.3.0), μια χωρίς deskew κοντά στον μέσο χρόνο των σημείων. Από 25/9 κάθε run γράφει
`pose_times.csv` και `*_poses_posetime_tum.txt` (θέσεις στη δική τους στιγμή)· στα παλαιότερα runs ο χρόνος ανακατασκευάζεται από τη
σύμβαση του βραχίονα (config + `two_start.csv`). Με `-v` / `--out` γράφονται τα TUM της εκτίμησης και του GT (στο σύστημα του LiDAR), ώστε
κάθε αριθμός να αναπαράγεται με την εντολή `evo_ape` που τυπώνεται. Το `evaluate_ncd.py` (και `--offset=best`) μένει για τα παλιά νούμερα.

### Λήψη Ground Truth (Oxford Spires)

Το GT είναι **τροχιά σε TUM format**, παραγόμενη με ICP registration των Hesai clouds
πάνω στον TLS χάρτη (ακρίβεια ~1–2 cm).

```bash
# prerequisites
git clone https://github.com/ori-drs/oxford_spires_dataset.git
cd oxford_spires_dataset && pip install .
huggingface-cli login          # απαιτείται λογαριασμός HuggingFace

python scripts/dataset_download.py \
    --patterns "sequences/2024-03-18-christ-church-02/processed/trajectory/gt-tum.txt"
```

Επιβεβαιωμένο ότι το αρχείο υπάρχει στο HuggingFace repo `ori-drs/oxford_spires_dataset`
(**2.24 MB**), μαζί με `vilens-slam-tum.txt`, `hba-tum.txt`, `colmap-tum.txt`.

| Χαρακτηριστικό | Τιμή |
|---|---|
| Μορφή | TUM: `timestamp tx ty tz qx qy qz qw` |
| Timestamps | **Unix time σε δευτερόλεπτα** (nanosecond precision) — ταιριάζει άμεσα με το πεδίο `timestamp` του bag |
| Πλαίσιο | pose του **base frame** στο world frame (base ορισμένο ως προς το LiDAR sensor frame ⇒ **χρειάζονται τα extrinsics** από το wiki «Sensors») |
| Χρονικό παράθυρο του κομμένου bag | `1710754268.978` → `+240.0 s` |

> ⚠️ **Δύο εργασίες ευθυγράμμισης πριν από οποιοδήποτε ATE:** (α) χρονικό crop/interpolation
> του GT στα 2402 timestamps των scans, (β) εφαρμογή του `base → lidar` extrinsic, αλλιώς
> το ATE θα περιέχει σταθερό offset άσχετο με το drift.

---

## 💡 Ιδέες προς διερεύνηση

### Ι-1 (2026-09-18, Λ.) — Intensity ως πανοραμική εικόνα: διευθύνσεις + ICP

**Κατάσταση 21/9: ΑΠΟΡΡΟΦΗΘΗΚΕ.** Εξελίχθηκε στη μέθοδο Ι-3 (κίνηση από την εικόνα ως deskew + αρχική θέση ICP).
Μένει ανοιχτός ο ρόλος της σε ευρωστία και loop closure, μαζί με την Ι-2.


**Ιδέα.** Το intensity μιας σάρωσης δεν είναι απλώς «χρώμα ανά σημείο» αλλά μια
**πανοραμική εικόνα** γύρω από το σημείο λήψης: κάθε pixel = μια **διεύθυνση** στον χώρο.
Όταν πετάμε τα μακρινά σημεία από τον ICP (π.χ. > 50 m), να πετάμε μόνο το XYZ τους και να
**κρατάμε τη διεύθυνσή τους** στην εικόνα. Η registration (scan-to-scan ή scan-to-map) τότε
χρησιμοποιεί **και** ICP στα φιλτραρισμένα σημεία **και** διανύσματα διευθύνσεων από
αντιστοιχίσεις στις εικόνες intensity των δύο σημείων λήψης.

**Γιατί στέκει.**
- Η διεύθυνση ενός μακρινού σημείου σχεδόν δεν αλλάζει με μετατόπιση λίγων cm· αλλάζει
  όταν στρίβεις → καθαρή μέτρηση **στροφής**. Φυσική αποσύζευξη: στροφή από διευθύνσεις,
  θέση από τον ICP στα κοντινά.
- Σε degenerate γεωμετρία (διάδρομος, σκάλα) η υφή του intensity (πινακίδες, ακμές
  σκαλοπατιών, αλλαγές υλικού) δίνει περιορισμούς εκεί που η γεωμετρία δεν δίνει.
- Χρησιμοποιεί το intensity ως **ανεξάρτητη μέτρηση**, όχι ως κριτήριο επιλογής σημείων
  (όπως η τρέχουσα μέθοδος).

**Επιφυλάξεις.**
- Σε αυτό το dataset τα μακρινά σημεία υπάρχουν κυρίως **έξω**· στα εσωτερικά scans
  900–1099 μόνο 27/200 είχαν 1–2 σημεία > 50 m. Στη σκάλα η αξία θα έρθει από την εικόνα
  intensity συνολικά, όχι από τα μακρινά.
- Το intensity εξαρτάται από απόσταση/γωνία πρόσπτωσης· σταθερό μεταξύ διαδοχικών scans
  (0.1 s), λιγότερο scan-to-map. Η per-scan κανονικοποίηση (#001, εύρημα 4) πρέπει να φύγει.
- Hesai QT64: 64 δακτύλιοι (πεδίο `ring`) × ~850–1.200 στήλες → χαμηλή κατακόρυφη ανάλυση.
  Οι διευθύνσεις θέλουν deskew (υπάρχει per-point `timestamp`).
- Νέο module (εικόνα, features, αντιστοίχιση, κοινή βελτιστοποίηση)· ο ICP του KISS είναι
  σε C++ → χρειάζεται δικός μας βελτιστοποιητής.

**Σχετική βιβλιογραφία (από μνήμη — ⚠️ να επιβεβαιωθεί πριν αναφερθεί):**
COIN-LIO (Pfreundschuh et al., ICRA 2024 — εικόνα intensity + φωτομετρικό σφάλμα, ρητά για
degenerate περιβάλλοντα)· Intensity-SLAM (Wang, Wang, Xie, RA-L 2021)· Shan et al., «Robust
Place Recognition using an Imaging Lidar» (ICRA 2021). Πιθανή πρωτοτυπία: η **ρητή χρήση των
διευθύνσεων των μακρινών σημείων που απορρίπτονται από τον ICP** ως περιορισμών στροφής.

**Προτεινόμενο πρώτο τεστ («υπάρχει σήμα;»).** Εικόνες intensity από διαδοχικά scans →
αντιστοιχίσεις → στροφή μόνο από διευθύνσεις → σύγκριση με τη στροφή του GT και του ICP.
Παραλλαγή χωρίς νέες εξαρτήσεις: **phase correlation** (FFT, numpy) μεταξύ πανοραμάτων —
οριζόντια μετατόπιση της εικόνας = yaw.

**Status:** ⏳ πρώτο τεστ #011 — ως εκτιμητής στροφής δεν ξεπερνά τον ICP με ίσο deskew· οι αντιστοιχίσεις
όμως δουλεύουν σταθερά (υποψήφια για ευρωστία / loop closure).

---

### Ι-2 (2026-09-18, Λ.) — Χάρτες όψεων: κατακόρυφα αναπτύγματα αριστερά/δεξιά

**Κατάσταση 21/9: ΑΝΟΙΧΤΗ, αδοκίμαστη.** Χρειάζεται dataset με επάλληλους ίδιους ορόφους· αυτό δεν την δοκιμάζει
(η διαδρομή του ορόφου δεν περνά πάνω από του ισογείου). STATUS §2.7(δ).


**Ιδέα.** Το KISS αναγνωρίζει επανεπισκέψεις με **κατόψεις** (εικόνες πυκνότητας), που δεν μπορούν να ξεχωρίσουν
επίπεδα όπως οι όροφοι ενός κτηρίου. Εκτός από την κάτοψη, να φτιάχνονται **χάρτες όψεων**: κατακόρυφα
αναπτύγματα αριστερά και δεξιά ως προς την τροχιά. *Κρατιέται για αργότερα.*

**Σκέψεις (Claude, 18/9).**
- **Κρατά ό,τι χάνει η κάτοψη.** Η κάτοψη δίνει (x, y, yaw)· μια όψη δίνει (θέση κατά μήκος, ύψος, κλίση στο
  επίπεδό της). Κάτοψη + δύο όψεις → και οι 6 κινήσεις από τρεις 2D αντιστοιχίσεις, όπως κάτοψη και όψεις σε
  αρχιτεκτονικό σχέδιο. Η διαφορά ύψους αριστερής–δεξιάς όψης δίνει το roll.
- **Στο εσωτερικό, η πληροφορία είναι στους τοίχους** (πόρτες, παράθυρα, κόγχες, τόξα), όχι στην κάτοψη, όπου
  οι τοίχοι είναι απλές γραμμές. Οι όροφοι φαίνονται ως χωριστές ζώνες ύψους· η σκάλα ως διαγώνιος.
- **Μπορεί να φέρει intensity.** Η ορθή προβολή δίνει εικόνα intensity **ανεξάρτητη από τη θέση λήψης** (σε
  αντίθεση με το πανόραμα της Ι-1, που είναι κεντρικό στον αισθητήρα και παραμορφωμένο). Τα παράθυρα και τα
  τόξα της #008 θα εμφανίζονταν αδιάστρεβλα.
- **Προτεινόμενη βελτίωση: επίπεδα προβολής δεμένα στους κύριους τοίχους, όχι στην τροχιά.** Δύο περάσματα από
  διαφορετική διαδρομή — ή σε αντίθετη φορά, όπου αριστερά/δεξιά ανταλλάσσονται — θα έδιναν τότε την ίδια εικόνα.
  Κτήρια όπως η εκκλησία και τα κλιμακοστάσια έχουν λίγες κυρίαρχες διευθύνσεις τοίχων.
- **Δυσκολίες:** επιλογή των σημείων που προβάλλονται (λωρίδα κοντά στον τοίχο, αλλιώς αναμειγνύονται έπιπλα
  και μακρινοί τοίχοι)· ο κατακόρυφος άξονας θέλει βαρύτητα (ευθυγράμμιση εδάφους ή το IMU του bag)· καμπύλη
  τροχιά μέσα σε έναν τοπικό χάρτη.
- **Καταμερισμός με την Ι-1:** Ι-1 (πανόραμα, ανά σάρωση) → odometry και στροφή· Ι-2 (όψεις, ανά χάρτη) →
  loop closure και ύψος· κάτοψη (υπάρχει) → (x, y, yaw).
- **Αυτό το dataset δεν τη δοκιμάζει καλά:** η διαδρομή του ορόφου δεν περνά πάνω από του ισογείου (1.7 % < 5 m
  σε κάτοψη) και τα 2 closures είναι γνήσια. Χρειάζεται dataset με επάλληλους όμοιους ορόφους.

**Status:** ⏳ ιδέα για αργότερα.


---

### Ι-3 (2026-09-18, Λ.) — Deskew από τις εικόνες intensity

**Κατάσταση 21/9: ΕΓΙΝΕ ΚΑΙ ΕΙΝΑΙ Η ΜΕΘΟΔΟΣ.** Υλοποιημένη στον KISS-SLAM (`--image-deskew`), τρεις ακολουθίες,
μία ρύθμιση (#027–#032). Οι επιμέρους βελτιώσεις της έχουν κατάσταση στη λίστα στο τέλος αυτής της ενότητας.


**Ερώτημα Λ.Γ.:** μπορούν οι εικόνες intensity να βοηθήσουν στο deskew (χωρίς IMU);

**Γιατί είναι εύλογο:** στο πανόραμα κάθε **στήλη** είναι μια **χρονική στιγμή** της περιστροφής — το πανόραμα είναι
ουσιαστικά κάμερα «rolling shutter». Αντιστοιχίσεις χαρακτηριστικών intensity ανάμεσα σε δύο διαδοχικές σαρώσεις,
η καθεμία με τον δικό της χρόνο, περιορίζουν την κίνηση **μέσα** στη σάρωση. Είναι ανεξάρτητη πληροφορία από τη
γεωμετρία: ο ICP «απορροφά» τη στρέβλωση επειδή οι λείες επιφάνειες γλιστρούν, ενώ ένα σημειακό χαρακτηριστικό
(γωνία παραθύρου, κάγκελο) δεσμεύει τη θέση τη στιγμή που μετρήθηκε. Λύνει το κυκλικό πρόβλημα του #017.

**Φθηνός έλεγχος (offline, χωρίς SLAM):** για κάθε ζεύγος σαρώσεων k, k+1 στις δύσκολες περιοχές: SIFT στα πανοράματα
των **ακατέργαστων** σαρώσεων → ζεύγη 3D σημείων p (χρόνος s_p) και q (χρόνος 1+s_q) → εκτίμηση σταθερής ταχύτητας ξ
(6 παράμετροι) με RANSAC: exp(s_p·ξ)·p ≈ exp((1+s_q)·ξ)·q. Σύγκριση της ξ με την αληθινή κίνηση (GT) και με τη
μαντεψιά του KISS (προηγούμενη κίνηση). Αν είναι σαφώς πιο κοντά στο GT → deskew με αυτήν μέσα στο SLAM.

**Επιφυλάξεις:** 64 γραμμές = χαμηλή κατακόρυφη ανάλυση· στη σκάλα το intensity έχει λίγο σχέδιο (#009)· η Ι-1
έδωσε στροφή 0.33–0.35° ανά ζεύγος, οριακό για deskew που χρειάζεται ~0.2°. Βιβλιογραφία (π.χ. COIN-LIO, που
χρησιμοποιεί εικόνες intensity μαζί με IMU) — να ελεγχθεί η πρωτοτυπία.


**Πιθανές βελτιώσεις (18/9, από την αναλυτική περιγραφή του αλγορίθμου) — κατάσταση 21/9:**
1. ✘ **Deskew με ολόκληρη την καμπύλη κίνησης** — **δοκιμάστηκε (#033)**: καμία ουσιαστική διαφορά. Με τα δικά μας
   μεγέθη η διαφορά θέσης ενός σημείου στα 10 m είναι ~3 cm, μικρότερη από τον θόρυβο του ICP. Ο κώδικας μένει
   (`deskew_curve`), η προεπιλογή παραμένει η ομοιόμορφη κατανομή.
2. ⏳ **Κατώφλι RANSAC ανάλογο της απόστασης** — **αδοκίμαστο, το μόνο που μένει ζωντανό από τη λίστα.** Σήμερα 0.30 m
   για όλα, της τάξης της κίνησης ανά σάρωση. Προσοχή στο όριο του #038 (κορεσμός στα ~0.50°).
3. ⏳ **Αμφίδρομος έλεγχος (cross-check)** στην αντιστοίχιση SIFT — **αδοκίμαστο.**
4. ✘ **Εικόνα με ακτίνα ανά pixel (Ι-5)** — **έγινε (#034, #037)**: βελτιώνει το RPE με βεβαιότητα (−10 % μετατόπιση,
   −12 % στροφή, 4+4 runs), το ATE όχι. Προαιρετικό (`RENDER = "raycast"`), ανενεργό από προεπιλογή.
   ✘ **Πυκνή αντιστοίχιση (RoMa)** — **δοκιμάστηκε και ΕΚΛΕΙΣΕ (#039)**: ίδιος διάμεσος με το SIFT, χειρότερη ουρά και
   θέση, 224 ms–66 s ανά ζεύγος σε Mac/MPS. Αν ποτέ χρειαστεί πυκνή συνταύτιση σε πραγματικό χρόνο: EDM ή
   EfficientLoFTR σε CUDA.

### Ι-4 (2026-09-18) — Χάρτης βάθους ως δεύτερη πηγή χαρακτηριστικών

Η ίδια διαδικασία (πανόραμα → χαρακτηριστικά → κίνηση) πάνω στην εικόνα **απόστασης** αντί για φωτεινότητα, ως δεύτερη,
ανεξάρτητη μέτρηση που θα συνδυαζόταν με την εικόνα intensity στις εκφυλισμένες κατευθύνσεις.
**Κατάσταση 21/9: ΜΕΡΙΚΩΣ ΔΟΚΙΜΑΣΜΕΝΗ και σε αναμονή.** Η σύντηξη μέσα στο SLAM κατέρρευσε (#023, ATE 5–12 m) με αιτία
τη μεροληψία κλίμακας της εικόνας· `scripts/test_i4_degeneracy_fusion.py`. Τώρα που η μεροληψία είναι κατανοητή και
διορθώσιμη (#039), η γραμμή ξαναγίνεται βιώσιμη — βλ. STATUS §2.6.

### Ι-5 (2026-09-19, Λ.) — Η εικόνα «από το pixel προς την επιφάνεια» (ray casting)

Αντί να ρίχνεται κάθε σημείο στο πλησιέστερο pixel (splat, 42 % γεμάτα pixel και μοτίβο κενών σταθερό ως προς τον
σαρωτή), κάθε pixel να είναι **ακτίνα** που παίρνει τιμή με παρεμβολή ανάμεσα στα δύο δείγματα του δακτυλίου του.
**Κατάσταση 21/9: ΕΓΙΝΕ (#034), ΕΠΙΒΕΒΑΙΩΘΗΚΕ ΜΕ 4+4 RUNS (#037).** 71 % έγκυρα pixel· RPE μετατόπισης −10 % (t = 11),
RPE στροφής −12 % (t = 31), καμία επικάλυψη· το ATE δεν ξεχωρίζει. `RENDER = "raycast"`, `--render=raycast` —
**ανενεργό από προεπιλογή** μέχρι να αποφασιστεί ρητά.

**Νέο που προέκυψε (21/9, #039) — δεν ήταν σε καμία λίστα:** η μετατόπιση που μετρά η εικόνα είναι **συστηματικά
κοντή**, με σφάλμα που εξαρτάται από την **απόσταση** της αντιστοίχισης (74 % στα < 5 m, 100 % στα > 12 m), κοινό σε
SIFT και RoMa. Υπάρχει αυτοβαθμονομούμενη διόρθωση χωρίς GT (`trans_min_range`, `trans_mode=auto`), που φέρνει την
κλίμακα σε ~1.00 σε τρεις ακολουθίες — αλλά **δεν βελτιώνει το SLAM**, γιατί την τελική θέση τη βγάζει ο ICP.
Παραμένει ως φαινόμενο προς δημοσίευση και ως εργαλείο για το §2.6 του STATUS (εκφυλισμένες κατευθύνσεις).

---

## 📒 Εγγραφές πειραμάτων (νεότερα πρώτα)

---

### 2026-10-06 — #188 KISS-ICP χωρίς το GIL στις κλήσεις C++ (ίδιος κώδικας 1.3.0, ξαναχτισμένος): ίδιες τροχιές· SURF + C++ 380 → 343 s (0.87×)

**Σχετικά:** #186, #187 · ΑΠΟΦΑΣΗ Μ.Τ. 6/10 (δοκιμή σε απομονωμένο περιβάλλον) · κλάδος `speed_test` · πηγή KISS-ICP v1.3.0 (GitHub, `kiss_runs/gil188/kiss-icp`), διόρθωση μίας γραμμής σε 6 bindings
(`py::call_guard<py::gil_scoped_release>()`: `_update` ×2, `_add_points`, `_preprocess`, `_align_points_to_map`, `_voxel_down_sample`) — `baselines/kiss_icp_gil.patch` · χτισμένο **μόνο** στο κλωνοποιημένο περιβάλλον
`kiss-slam-gil` (το `kiss-slam-main` ανέγγιχτο) · `kiss_runs/gil188_launch.sh` · Boreas ×8, `--parallel`, 4 μπροστά, σπόρος 0

| run | πριν (`kiss-slam-main`) | με KISS-ICP χωρίς GIL | × πραγματικού | τροχιά |
|---|---|---|---|---|
| SURF + C++ | 379.5 s · process_scan 359.2 s | **343.1 s · 335.5 s** | **0.87×** | ταυτόσημη με `cppsurf183` |
| SURF, Python | 386.7 s · 370.5 s | 370.8 s · 363.0 s | 0.81× | ταυτόσημη με `ral178` |

Έλεγχος πρώτα σε 80 σαρώσεις στα δύο περιβάλλοντα: ταυτόσημες.

**Συμπέρασμα ⏳:** η σειριοποίηση από το GIL ήταν πράγματι μέρος του κόστους: −10 % με C++ στην εικόνα, −4 % χωρίς, ίδια αποτελέσματα. Με SURF το όχημα φτάνει 0.87× του πραγματικού χρόνου. Για χρήση: το patch
του KISS-ICP πρέπει να συνοδεύει τον κώδικα (όπως τα patches των άλλων μεθόδων). Άλλες κλήσεις που κρατούν το GIL: οι δικές μας pybind του KISS-SLAM (`_integrate_frame` ~9 ms, χάρτης).

---

### 2026-10-06 — #187 ICP με 8 / 16 νήματα: ίδια ακριβώς τροχιά, κανένα κέρδος χρόνου

**Σχετικά:** #186 · κλάδος `speed_test` · `kiss_runs/icp187_launch.sh`, configs `kiss_runs/threads{8,16}.yaml` (= `threads4.yaml` με `max_num_threads` 8 / 16) · SURF + C++, 4 μπροστά, Boreas ×8, `--parallel`, σπόρος 0

| ICP νήματα | χρόνος · process_scan | APE m · RTE % · RPE 1 m cm / ° |
|---|---|---|
| 4 (`ahcpp186`) | 379.5 s · 359.2 s | 0.195 · 0.28 · 3.84 / 0.152 |
| 8 | 372.6 s · 352.8 s | ίδια (ταυτόσημη) |
| 16 | 377.8 s · 358.2 s | ίδια (ταυτόσημη) |

**Συμπέρασμα ⏳:** περισσότερα νήματα στον ICP δεν βοηθούν (−2 % / −0.5 %, θόρυβος)· ο ICP του KISS δεν είναι το όριο όπως μετρά το προφίλ, ή το όριο είναι η σειριοποίηση στην κύρια διεργασία (GIL: οι κλήσεις του
KISS κρατούν το GIL, ενώ νήματα ανάγνωσης / αποστολής περιμένουν). Με SURF το όχημα μένει ~0.79–0.81× του πραγματικού χρόνου.

---

### 2026-10-06 — #185 / #186 Προφίλ με την εικόνα σε C++ και εικόνα 4 σαρώσεις μπροστά: η εικόνα δεν είναι πια κορεσμένη· η αναμονή ήταν από αιχμές· κέρδος μικρό (Python −4 %)

**Σχετικά:** #179, #183 · κλάδος `speed_test` · `kiss_runs/prof185_launch.sh` (`prof185_{surf,orb}_{main,worker}.{prof,txt}`), `ah186_launch.sh` · `5bd235c` (`KISS_IMAGE_AHEAD`, προεπιλογή 4) · Boreas ×8, `--parallel`, σπόρος 0

| διεργασία εικόνας, ms / σάρωση (C++) | SURF | ORB |
|---|---|---|
| ανίχνευση | ~38 | ~4 |
| πανόραμα | 10.3 | 8.8 |
| αντιστοίχιση + προσαρμογή (guided 6.4 / 7.6, fit 4.9 / 8.7, RANSAC 1.8 / 1.6) | 15.1 | 20.1 |
| εφεδρεία απόστασης (μ. ό.· ~190 ms όταν τρέχει) | 24.6 | 2.9 |
| **σύνολο · απασχόληση** | **86 · 68 %** | **52 · 54 %** |

| κύρια διεργασία, ms / σάρωση | SURF | ORB |
|---|---|---|
| αναμονή για την εικόνα | 38.5 | 14.6 |
| ICP | 41.8 | 37.7 |
| αραίωση + deskew του KISS + χάρτες | 33.6 | 32.7 |
| KD-tree δύο αρχών + loop closures | 4.8 | 3.6 |
| **process_scan** | 126.5 | 96.5 |

Η εικόνα (86 ms) δεν είναι κορεσμένη, αλλά υπολογιζόταν μόνο μία σάρωση μπροστά· στις σαρώσεις με εφεδρεία η κύρια περίμενε. #186: ουρά 4 σαρώσεων (μία διεργασία, ίδια σειρά):

| run | πριν | με 4 μπροστά | τροχιά |
|---|---|---|---|
| SURF + C++ | 373.8 s (process_scan 365.9) | 379.5 s (359.2) | ταυτόσημη με `cppsurf183` |
| SURF, Python | 404.1 s | **386.7 s** (370.5) | ταυτόσημη με `ral178` |

**Συμπέρασμα ⏳:** ίδια αποτελέσματα, μικρό κέρδος (−4 % στην Python, ~0 με C++). Η σάρωση κοστίζει ~120 ms αντί για τα ~88 της «δικής» της δουλειάς: πιθανώς τα νήματα της κύριας διεργασίας (ανάγνωση ~10 ms,
αποστολή στη διεργασία ~6 ms) μοιράζονται το GIL με τις κλήσεις του KISS (pybind, κρατούν το GIL). Το υπόλοιπο Python της εικόνας είναι πια ~10–15 ms — μικρό περιθώριο από περισσότερη C++.

---

### 2026-10-06 — #184 ORB + C++: στο όχημα RPE καλύτερο (−10 %) αλλά APE +11 % (4 σπόροι), 0.98–1.04× πραγματικού· στη φορητή χειρότερο (quad_easy RPE +23 %)

**Σχετικά:** #088, #181, #183 · κλάδος `speed_test` · `kiss_runs/orb184_launch.sh` · `orb`, `KISS_IMAGE_CPP=all`, `--parallel`, ένα run κάθε φορά· αναφορά: SURF (`bfc150`, 4 σπόροι, Python)

| ακολουθία | εκδοχή | χρόνος · × πραγματικού | APE m | RTE % | RPE 1 m cm / ° | RRE °/100 m | μήκος m |
|---|---|---|---|---|---|---|---|
| quad_easy (199 s) | SURF (4 σπόροι) | 179 s · 1.11× (#176) | 0.077–0.078 | 0.26 | 5.34–5.45 / 0.570–0.576 | 1.058–1.076 | 264.4–264.9 |
| | **ORB + C++** (σ. 0) | 121.2 s · 1.64× | **0.081** | 0.27 | **6.68 / 0.673** | 1.154 | 268.5 |
| christ-church-03 (312 s) | SURF (4 σπόροι) | 224 s · 1.39× (#178) | 0.039–0.040 | 0.13–0.14 | 3.88–3.97 / 0.704–0.708 | 0.645–0.706 | 284.9–285.5 |
| | **ORB + C++** (σ. 0) | 148.9 s · 2.10× | **0.043** | 0.14 | 3.92 / **0.753** | 0.742 | 287.6 |
| Boreas (300 s) | SURF (4 σπόροι) | 404 s · 0.74× | 0.182–0.202 (μ. ό. 0.195) | 0.26–0.28 | 3.61–3.88 / 0.155–0.162 | 0.129–0.137 | 1323.7–1324.4 |
| | **ORB + C++** (4 σπόροι) | 287–305 s · 0.98–1.04× | **0.210–0.223 (0.217)** | 0.27 | **3.32–3.40 / 0.134–0.142** | 0.117–0.119 | 1322.9–1323.0 |

**Συμπέρασμα ⏳:** το ORB δεν είναι καθαρή βελτίωση: στο όχημα τοπικά ακριβέστερο (RPE −10 % / −13 %) αλλά APE +11 % (έξω από τους σπόρους)· στη φορητή χειρότερο (quad_easy RPE +23 % / +17 %, μήκος +1.5 %,
church_03 στροφή +7 %, APE +5–10 %) — όπως στο #088. Ο πραγματικός χρόνος στο όχημα είναι οριακός (0.98–1.04×). Επιλογή μόνο ανά αισθητήρα (όχημα), απόφαση Μ.Τ.

---

### 2026-10-06 — #182 / #183 Νήματα του OpenBLAS και η εικόνα σε C++: **ORB + C++ σε πραγματικό χρόνο στο όχημα (287 s, 1.04×)**· SURF + C++ 374 s (0.80×), ακρίβεια μέσα στους σπόρους

**Σχετικά:** #179 (προφίλ), #181 (ORB) · κλάδος `speed_test` (`95c2fd1`: `kiss_slam/cpp/image_fit.cpp`, `scripts/build_image_fit.sh`, `KISS_IMAGE_CPP=off|exact|all`) · `kiss_runs/blas182_launch.sh`, `cpp183_launch.sh` ·
Boreas ×8, `--parallel`, σπόρος 0, ένα run κάθε φορά
C++: (1) διασπορά του πανοράματος — ταυτόσημη (293 / 293 σαρώσεις offline)· (2) υποθέσεις του RANSAC (Kabsch 3 σημείων + έλεγχος inliers, οι ίδιες κληρώσεις) — ίδια inliers· (3) η χρονική προσαρμογή soft_l1
(ίδιο κόστος και Jacobian, Levenberg-Marquardt με αυστηρότερη ανοχή): σε 12 προσαρμογές ≤ 1e-6 από πλήρως συγκλίνουσα scipy, ενώ η scipy με την προεπιλεγμένη ανοχή σταματά έως 5e-4 μακριά (ίδιο κόστος έως 1e-9) —
μετά τη σάρωση η ακολουθία αποκλίνει όπως με άλλον σπόρο (πρόβλεψη, κληρώσεις RANSAC).

| run | χρόνος · process_scan | × πραγματικού | APE m | RTE % | RPE 1 m cm / ° | RRE °/100 m | τροχιά |
|---|---|---|---|---|---|---|---|
| SURF, Python (`ral178`) | 404.1 s · 396.1 s | 0.74× | 0.182 (σπόροι 0.196–0.202) | 0.27 | 3.64 / 0.155 (3.61–3.88 / 0.157–0.162) | 0.130 | – |
| SURF, `OPENBLAS_NUM_THREADS=1` (#182) | 416.6 s · 408.9 s | 0.72× | ίδια | | | | ταυτόσημη |
| SURF, C++ μόνο το ταυτόσημο (`exact`) | 396.4 s · 388.7 s | 0.76× | ίδια | | | | ταυτόσημη |
| **SURF, C++ όλα (`all`)** | **373.8 s · 365.9 s** | **0.80×** | 0.195 | 0.28 | 3.84 / 0.152 | 0.134 | σαν άλλος σπόρος |
| ORB, Python (`orb181`) | 302.2 s · 294.4 s | 0.99× | 0.211 | 0.27 | 3.27 / 0.132 | 0.119 | – |
| **ORB, C++ όλα** | **287.4 s · 279.3 s** | **1.04×** | 0.213 | 0.27 | 3.32 / 0.134 | 0.117 | σαν άλλος σπόρος |

**Συμπέρασμα ⏳:** 1. Τα νήματα του OpenBLAS δεν είναι η αιτία (ίδια τροχιά, ίδιος χρόνος). 2. Η C++ κερδίζει 7.5 % με SURF (404 → 374 s) με ακρίβεια μέσα στους σπόρους· το ταυτόσημο μέρος μόνο του 2 %.
3. **ORB + C++: πρώτη φορά πραγματικός χρόνος στο όχημα (1.04× με την εκκίνηση, 10.7 Hz ανά σάρωση)**· η κύρια διεργασία (~93 ms / σάρωση) είναι πλέον το όριο. SURF + C++ μένει στο 0.80× (η εικόνα το όριο).

---

### 2026-10-06 — #181 Άλλοι ανιχνευτές στο όχημα (Boreas ×8, `--parallel`): ORB σε πραγματικό χρόνο (302 s) με καλύτερο RPE· upright AKAZE πιο αργό και χειρότερο

**Σχετικά:** #088 (ORB στη φορητή: στροφή +8–10 %, offline), #178 · κλάδος `speed_test` (`b4f09b8`, `f0f8ba0`) · ΑΠΟΦΑΣΗ / ζήτημα Μ.Τ. 6/10 («upright AKAZE», ταχύτεροι περιγραφείς) · `kiss_runs/det181_launch.sh`,
`akaze181_launch.sh` · Γ με άλλον ανιχνευτή (η εφεδρεία απόστασης μένει SURF, πια με δικό της L2 matcher — πριν, με δυαδικό ανιχνευτή θα ταίριαζε τους SURF της απόστασης με Hamming)

Χρόνος ανίχνευσης σε πανόραμα του Boreas (μόνο του): ×8 upright SURF 20–23 ms (430–630 σημεία), **upright AKAZE 83–96 ms (79–128)**, ORB 5–6 ms (2300–2600)· ×4: 6–12 / 34–49 / 4–5 ms.

| ανιχνευτής | χρόνος · × πραγματικού | APE m | RTE % | RPE 1 m cm / ° | RRE °/100 m | δύο αρχές |
|---|---|---|---|---|---|---|
| upright SURF (`ral178`) | 404.1 s · 0.74× | 0.182 (σπόροι 0.196–0.202) | 0.27 | 3.64 / 0.155 (3.61–3.88 / 0.157–0.162) | 0.130 | 31 |
| **ORB** (`orb181`) | **302.2 s · 0.99×** | 0.211 | 0.27 | **3.27 / 0.132** | **0.119** | 15 |
| upright AKAZE (`akaze181`, M-LDB 61 bytes → 64) | 722.7 s · 0.42× | 0.207 | 0.29 | 4.42 / 0.182 | 0.139 | 95 |
| KISS-SLAM | 236.9 s · 1.27× | 0.266 | 0.28 | 4.14 / 0.192 | 0.149 | – |

**Συμπέρασμα ⏳:** στο όχημα το ORB είναι 25 % ταχύτερο και τοπικά ακριβέστερο (RPE −10 % / −15 %)· APE λίγο πάνω από τους σπόρους του SURF (ένα run). Το AKAZE απορρίπτεται (2× πιο αργό, χειρότερο RPE).
Για αλλαγή ανιχνευτή (ανά αισθητήρα ή παντού) χρειάζονται σπόροι στο όχημα και η φορητή (το #088 ήταν offline, 200 σαρώσεις).

---

### 2026-10-06 — #180 Η εικόνα σε δύο στάδια (χαρακτηριστικά μπροστά από την αντιστοίχιση) + κρυφή μνήμη απόστασης: ίδιες τροχιές, **όχι ταχύτερο**

**Σχετικά:** #179 · κλάδος `speed_test` (`63cd608`, `b1a4a2d`) · `kiss_runs/stg180_launch.sh`, `stgb180_launch.sh` · Γ, ×8, `--parallel`, σπόρος 0
Έλεγχος offline (1350 σαρώσεις του Boreas, 151 με εφεδρεία απόστασης): οι κινήσεις της παλιάς και της νέας διαδρομής ταυτόσημες bit προς bit (και με τα χαρακτηριστικά μέσω pickle).

| εκδοχή | χρόνος · process_scan | CPU | τροχιά έναντι `ral178_s0` |
|---|---|---|---|
| μία διεργασία εικόνας (`ral178`, #178) | 404.1 s · 396.1 s | 19.3 | – |
| (α) δύο διεργασίες: χαρακτηριστικά σε δεύτερη, μέσω της κύριας | **448.7 s** · 440.8 s | 16.9 | ταυτόσημη |
| (β) νήμα χαρακτηριστικών μέσα στη διεργασία της εικόνας | **415.5 s** · 407.6 s | 17.7 | ταυτόσημη |

**Συμπέρασμα ⏳:** (α) πιο αργό: η κύρια διεργασία (ήδη το δεύτερο όριο, ~87 ms) αντέγραφε κάθε σάρωση δύο φορές και έπαιρνε πίσω τα χαρακτηριστικά (~25 MB / σάρωση). (β) καμία βελτίωση: το SURF
χρησιμοποιεί ήδη τα 8 νήματα του OpenCV, και η υπόλοιπη δουλειά (πανόραμα, RANSAC, least squares) είναι Python / numpy που κρατά το GIL — δύο νήματα στη ίδια διεργασία δεν τρέχουν παράλληλα.
Η εικόνα δεν παραλληλίζεται έτσι χωρίς αλλαγή στον κώδικά της (π.χ. εκτίμηση σε C++). Το όχημα μένει ~0.74× του πραγματικού χρόνου.

---

### 2026-10-06 — #179 Προφίλ και των δύο διεργασιών στο Boreas (×8, `--parallel`): η εικόνα είναι το όριο (απασχολημένη 77 %, 102 ms / σάρωση)· η κύρια διεργασία χωρίς αναμονή ~87 ms

**Σχετικά:** #170–#178 · κλάδος `speed_test` (κώδικας = `ral_method` + άγκιστρο `KISS_PROFILE_WORKER`, `4da9b9a`) · `kiss_runs/wprof179_launch.sh`, `kiss_runs/prof179_{main,worker}.{prof,txt}` ·
Γ, ×8, `--parallel`, σπόρος 0· 404.1 s συνολικά, process_scan 394.8 s (όσο το #178: το προφίλ δεν κοστίζει αισθητά)

| διεργασία εικόνας (ζωντανή 394.9 s, **απασχολημένη 304.9 s = 77 %**) | s | ms / σάρωση |
|---|---|---|
| **ανίχνευση SURF** (`detectAndCompute`, 1024 × 1024, 3356 κλήσεις) | **132.2** | **39 / κλήση** |
| πανόραμα (`grid` 35.1, από αυτά σειρά δακτυλίων 6.4) | 44.7 | 13 |
| αντιστοίχιση + εκτίμηση (`match_motion`: least squares 33.7, RANSAC 30.8, guided C++ 15.7) | 92.6 | 28 |
| εφεδρεία απόστασης (356 κλήσεις `range_features` × 193 ms — δύο πανοράματα απόστασης ανά αποτυχία) | 80.0 | 23 (μ. ό.) |
| **σύνολο `motion()`** | **304.9** | **102** |

| κύρια διεργασία | s | ms / σάρωση |
|---|---|---|
| **αναμονή για την εικόνα** | **138.7** | **46** |
| ICP | 122.6 | 41 |
| deskew του KISS + αραίωση (2 / σάρωση μετά το #176) | 24.5 + 35.6 | 20 |
| χάρτης SLAM + χάρτης ICP | 26.1 + 18.7 | 15 |
| loop closures, KD-tree δύο αρχών, λοιπά | ~25 | 8 |
| **χωρίς την αναμονή** | **~256** | **~85–87** |

**Συμπέρασμα ⏳:** η διεργασία της εικόνας (102 ms) είναι πιο αργή από την κύρια (~87 ms) και την καθυστερεί· το μεγαλύτερο κομμάτι είναι η ανίχνευση SURF στο πανόραμα 1024 × 1024 (39 ms, διπλάσιο από το ×4).
Αν η εικόνα κατέβει κάτω από ~85 ms / σάρωση, το όχημα δεσμεύεται από την κύρια διεργασία: ~11.5 Hz, δηλ. ~1.05–1.1× του πραγματικού χρόνου, χωρίς αλλαγή αποτελέσματος. Τρόπος: τα χαρακτηριστικά
(πανόραμα + SURF) κάθε σάρωσης εξαρτώνται μόνο από τη σάρωση → σε χωριστή διεργασία (ή δύο), η αντιστοίχιση / εκτίμηση / εφεδρεία στη σημερινή.

---

### 2026-10-06 — #177 Χρόνος του KISS-SLAM μόνου στο μηχάνημα: 2.4–4.7× πραγματικού χρόνου στη φορητή, 1.27× στο όχημα· η μέθοδός μας 1.7–2.2× πιο αργή

**Σχετικά:** #166, #176 · κλάδος `speed_test` (με νήμα ανάγνωσης και διόρθωση του reader του Boreas) · `kiss_runs/kisst177_launch.sh` · βραχίονας `kiss`, `threads4.yaml`, σπόρος 0, ένα run κάθε φορά

| ακολουθία (διάρκεια δεδομένων) | KISS-SLAM: χρόνος · Hz ανά σάρωση · × πραγματικού · CPU · RSS | Γ `--parallel`: χρόνος · × πραγματικού | Γ / KISS |
|---|---|---|---|
| quad_easy (199 s) | 83.3 s · 28 Hz · **2.39×** · 23.4 · 1.10 GB | 179.1 s (#176) · 1.11× | 2.2× |
| christ-church-03 (312 s) | 66.1 s · 58 Hz · **4.72×** · 22.3 · 1.03 GB | 241.3 s (#166, πριν τις βελτιώσεις) · 1.29× | 3.7× |
| Boreas, 3000 σαρώσεις (300 s) | 236.9 s · 13 Hz · **1.27×** · 19.3 · 2.71 GB | 402.1 s (#176) · 0.75× | 1.7× |

Η τροχιά του KISS στο Boreas είναι ταυτόσημη με το `kiss_s0` (1/10)· τα 1381 s εκείνου ήταν από batch και από τη μετατροπή του reader (#170). Ο KISS-SLAM χρησιμοποιεί κι αυτός 19–23 πυρήνες (TBB, numpy).

**Συμπέρασμα ⏳:** ο KISS-SLAM είναι σε πραγματικό χρόνο παντού, και στο όχημα (13 Hz). Η μέθοδός μας κοστίζει 1.7–3.7× τον χρόνο του KISS (η εικόνα: ανίχνευση, αντιστοίχιση, εκτίμηση, εφεδρεία,
και η δεύτερη καταχώριση των δύο αρχών)· στη φορητή μένει σε πραγματικό χρόνο, στο όχημα όχι (0.75×). Για το paper: πίνακας χρόνου KISS / Γ ανά αισθητήρα.

---

### 2026-10-06 — #176 Νήμα ανάγνωσης, επαναχρησιμοποίηση της αραίωσης, φθηνότερο φίλτρο του Boreas: ίδιες τροχιές· Boreas 442 → 402 s, quad_easy 194 → 179 s (πάνω από τον πραγματικό χρόνο μαζί με την εκκίνηση)

**Σχετικά:** #170–#175 · κλάδος `speed_test`, `05debb9` · `kiss_runs/rd176_launch.sh` · Γ, `--parallel`, σπόρος 0
(Α) οι σαρώσεις διαβάζονται από νήμα στο παρασκήνιο, 4 μπροστά, με την ίδια σειρά (όλα τα datasets, από το `b9_joint_new` #159)· (Β) η αραίωση 0.5 m του ICP ξαναχρησιμοποιείται για τον χάρτη του SLAM
όταν τα μεγέθη ταυτίζονται (όχημα: ICP voxel 1.0 → 0.5 = voxel του χάρτη· στη φορητή διαφορετικά, υπολογίζεται όπως πριν)· (Γ) φίλτρο του reader του Boreas `any(xyz != 0)` αντί `norm > 0` (ίδια σημεία, 3000 / 3000)

| run | χρόνος · process_scan | Hz συνολικά (με εκκίνηση) | × πραγματικού | CPU | τροχιά |
|---|---|---|---|---|---|
| Boreas ×8, πριν (`pan175`) | 442.4 s · 400.9 s | 6.8 | 0.68 | 18.9 | – |
| **Boreas ×8, #176** | **402.1 s · 394.3 s** | **7.5** | **0.75** | 19.5 | **ταυτόσημη (0)** |
| quad_easy, πριν (`lockpar166`, χωρίς #174) | 193.6 s · 164.3 s | 10.3 | 1.03 | – | – |
| **quad_easy, #176 (+ #174)** | **179.1 s · 171.8 s** | **11.1** | **1.11** | 15.1 | **ταυτόσημη (0)** |

**Συμπέρασμα ⏳:** ίδιες τροχιές, κέρδος 7–9 % στον συνολικό χρόνο (η ανάγνωση βγαίνει από τον κρίσιμο δρόμο· το process_scan αλλάζει λίγο, στο quad_easy +5 % από τον ανταγωνισμό για πυρήνες).
Συνολικά από το #166: Boreas 616.5 → 402.1 s (−35 %, 0.49 → 0.75 του πραγματικού χρόνου), quad_easy 193.6 → 179.1 s. Η φορητή είναι πια άνετα σε πραγματικό χρόνο με `--parallel`.

---

### 2026-10-06 — #175 Επαλήθευση του ταχύτερου πανοράματος (Boreas, ×8, `--parallel`): ταυτόσημη τροχιά, 434 → 401 s στο process_scan (7.5 Hz) — λιγότερο κέρδος από το αναμενόμενο

**Σχετικά:** #171 (`fix171`), #174 · κλάδος `speed_test` · `kiss_runs/pan175_launch.sh` · Γ, ×8, `--parallel`, σπόρος 0

| run | χρόνος · process_scan → Hz ανά σάρωση | × πραγματικού (300 s) | CPU | APE m · RTE % · RPE 1 m cm / ° | τροχιά |
|---|---|---|---|---|---|
| πριν από όλα (`lockpar166`) | 616.5 s · 560.6 s → 5.4 | 0.49 | – | 0.182 · 0.27 · 3.64 / 0.155 | – |
| + reader (`fix171`) | 475.9 s · 434.1 s → 6.9 | 0.63 | 17.4 | ίδια | ταυτόσημη |
| **+ πανόραμα (`pan175`)** | **442.4 s · 400.9 s → 7.5** | 0.68 | 18.9 | 0.182 · 0.27 · 3.64 / 0.155 | **ταυτόσημη (0, και στροφή)** |

**Συμπέρασμα ⏳:** η αλλαγή του #174 δεν αλλάζει το αποτέλεσμα (επιβεβαιώθηκε σε πλήρες run). Κέρδος 33 s (11 ms / σάρωση) αντί για ~60 s που περιμέναμε από το σειριακό προφίλ: η κύρια διεργασία
(ICP, voxel, χάρτες, reader ~93 ms / σάρωση στο #172) ή η ταυτόχρονη χρήση ~19 πυρήνων από τις δύο διεργασίες είναι πλέον το όριο. Σύνολο από το #166: 616.5 → 442.4 s (−28 %). Το όχημα μένει στο 0.68 του πραγματικού χρόνου.

---

### 2026-10-06 — #174 Ταχύτερο πανόραμα: η σειρά των δακτυλίων με `bincount` — ίδια εικόνα bit προς bit, Boreas 26 → 8 ms

**Σχετικά:** #173 · κλάδος `speed_test` · `kiss_slam/intensity_deskew.py`: νέα `ring_order()` (ένα πέρασμα `np.bincount` για τη μέση ανύψωση ανά δακτύλιο, αντί για μάσκα ανά δακτύλιο και `np.unique`),
χρησιμοποιείται στα `grid` και `raycast_grid` (και άρα και στο πανόραμα απόστασης της εφεδρείας)

Ανά βήμα (σάρωση του Boreas, 220 000 σημεία): σειρά δακτυλίων 9.0 ms + `np.unique` 4.4 + ανύψωση με `linalg.norm` 5.3 → μαζί ~1 ms· τα υπόλοιπα (στήλη 1.2, γέμισμα πίνακα 3.8, γέμισμα γραμμών 1.2,
μεγέθυνση 0.3) μένουν.

| αισθητήρας (σαρώσεις) | `panorama()` πριν → μετά | ίδια έξοδος (εικόνα, P, T, valid· intensity και απόσταση) |
|---|---|---|
| Boreas, Velodyne 128 (10) | 25.1–31.3 → 7.9–9.3 ms (**×3.2**) | ναι, bit προς bit, 10 / 10 |
| NCD 2020, Ouster OS1-64 (4) | 6.8–7.3 → 3.1–3.8 ms (×2.1) | ναι, 4 / 4 |
| NTU, Ouster OS1-16 (4) | 1.3 → 0.7 ms (×1.8) | ναι, 4 / 4 |

**Συμπέρασμα ⏳:** καθαρή βελτίωση ταχύτητας χωρίς αλλαγή αποτελέσματος (οι εικόνες είναι ταυτόσημες, άρα και οι τροχιές). Στο Boreas ~18 ms ανά πανόραμα λιγότερα (~21 ms / σάρωση με την εφεδρεία).
Εκκρεμεί run επαλήθευσης (ταυτόσημη τροχιά, χρόνος).

---

### 2026-10-06 — #173 Προφίλ της εικόνας στο Boreas (σειριακά, ×4): 92 ms / σάρωση — το ένα τρίτο στο πανόραμα, κυρίως η σειρά των δακτυλίων· η μεταφορά στη διεργασία μόνο 6 ms

**Σχετικά:** #170–#172 · κλάδος `speed_test` · `kiss_runs/ser173_launch.sh`, `kiss_runs/prof173_boreas_ser.{prof,txt}` · Γ, `--panorama-up=4`, σειριακά, σπόρος 0, με cProfile ·
559.6 s συνολικά, process_scan 521.4 s (5.8 Hz)· η εφεδρεία απόστασης χρησιμοποιήθηκε σε 261 σαρώσεις (524 προσπάθειες)

| εικόνα (`motion`, 3000 σαρώσεις) | s | ms / σάρωση |
|---|---|---|
| **σύνολο** | **275.9** | **92** |
| πανόραμα (`grid` + γέμισμα + μεγέθυνση, 3524 κλήσεις) | 96.0 | 32 |
| — από αυτά **η σειρά των δακτυλίων** (`[elev[ring == r].mean() for r in rings]`, 128 μάσκες × 220 000 σημεία) | **~35 + ufunc** | ~12–15 |
| ανίχνευση SURF (`detectAndCompute`) | 69.3 | 23 |
| αντιστοίχιση + εκτίμηση (`match_motion`: RANSAC 25.9, `fit_time` / least_squares 33.7, guided 12.0) | 79.1 | 26 |
| εφεδρεία απόστασης (`_range`, 524 κλήσεις × 124 ms — μέσα της πανόραμα + SURF) | 77.6 | 26 |

Υπόλοιπο της σάρωσης: ICP 112.0 s (37 ms), voxel 49.6 s, deskew του KISS 23.3 s, χάρτες 35.9 s, reader 29.3 s.
Μικρο-μετρήσεις (σάρωση του Boreas, 219 771 σημεία): pickle + unpickle των ορισμάτων της διεργασίας 6.3 ms (10.5 MB)· σειρά δακτυλίων με τη λίστα 13.4 ms, με `np.bincount` 0.3 ms, **ίδια σειρά**.

**Συμπέρασμα ⏳:** η μεταφορά στη διεργασία δεν είναι το πρόβλημα (6 ms). Η σειρά των δακτυλίων με `bincount` δίνει ακριβώς την ίδια εικόνα και γλιτώνει ~13 ms ανά πανόραμα (≈ 15 % της εικόνας).
Τα υπόλοιπα (SURF, RANSAC / least squares, εφεδρεία απόστασης στο 17 % των σαρώσεων) είναι πραγματική δουλειά της μεθόδου.

---

### 2026-10-06 — #172 Boreas με πανόραμα ×2 (+ διόρθωση, προφίλ): πιο αργό από το ×4 και λίγο λιγότερο ακριβές· η αναμονή της εικόνας δεν εξαρτάται από το μέγεθος του πανοράματος

**Σχετικά:** #170, #171 · κλάδος `speed_test` · `kiss_runs/up2172_launch.sh`, προφίλ `kiss_runs/prof172_boreas_up2.{prof,txt}` · Γ, `--panorama-up=2` (1024 × 256), `--parallel`, σπόρος 0, με cProfile

| run (μετά τη διόρθωση) | χρόνος · process_scan → Hz | APE m | RTE % | RPE 1 m cm / ° | RPE 1 s cm / ° | z RMSE m |
|---|---|---|---|---|---|---|
| ×8 (`fix171`) | 475.9 s · 434.1 s → 6.9 | 0.182 | 0.27 | 3.64 / 0.155 | 4.11 / 0.164 | 0.141 |
| ×4 (`fixup4171`) | 426.1 s · 385.4 s → 7.8 | 0.201 | 0.28 | 3.78 / 0.156 | 4.14 / 0.163 | 0.152 |
| **×2 (`up2172`, με προφίλ)** | **452.8 s · 410.0 s → 7.3** | 0.202 | 0.28 | **4.10 / 0.167** | 4.67 / 0.173 | 0.152 |

(×8, σπόροι 1–3: APE 0.196–0.202, RPE 1 m 3.61–3.88 / 0.157–0.162)

Προφίλ της κύριας διεργασίας (×2): αναμονή για την εικόνα **132.6 s (44 ms / σάρωση)** — όσο και με ×8 πριν τη διόρθωση (127 s)· ICP 122.6 s (40 ms)· voxel 58.9 s· deskew του KISS 24.1 s· χάρτης SLAM 24.8 s,
χάρτης ICP 16.7 s· reader 33.6 s (ο `ascontiguousarray` 4.0 s)· η μετατροπή του #170 εξαφανίστηκε (`preprocess` 26.8 s αντί 190.3 s).

**Συμπέρασμα ⏳:** 1. Το ×2 δεν είναι ταχύτερο από το ×4 (το προφίλ προσθέτει λίγο, αλλά η αναμονή της εικόνας μένει ~44 ms / σάρωση) και το RPE 1 m είναι λίγο έξω από τους σπόρους προς το χειρότερο.
2. Η διεργασία της εικόνας χρειάζεται ~140 ms / σάρωση ανεξάρτητα από το πανόραμα (×8 / ×2) → το κόστος της είναι ανά σημείο (μεταφορά ~230 000 σημείων στη διεργασία, προβολή, εφεδρεία απόστασης στο 31 %), όχι ανά pixel.
Επόμενο: προφίλ της εικόνας μέσα στη διεργασία της (ή σειριακό run με προφίλ). Το ×4 μένει η καλύτερη επιλογή του πανοράματος για το όχημα.

---

### 2026-10-06 — #171 Boreas μετά τη διόρθωση του reader: ταυτόσημη τροχιά, 6.9 Hz (από 5.4)· με πανόραμα ×4 7.8 Hz, ακρίβεια μέσα στους σπόρους — ακόμη όχι πραγματικός χρόνος

**Σχετικά:** #170 (`fc45188`), #166, #169 · κλάδος `speed_test` · `kiss_runs/fix171_launch.sh` · Γ, `--parallel`, σπόρος 0, ένα run κάθε φορά

| run | χρόνος · process_scan → Hz | × πραγματικού (300 s) | CPU | APE m | RTE % | RPE 1 m cm / ° | z RMSE m | τροχιά έναντι `lockpar166_s0` |
|---|---|---|---|---|---|---|---|---|
| πριν (`lockpar166_s0`, ×8) | 616.5 s · 560.6 s → 5.4 | 0.49 | – | 0.182 | 0.27 | 3.64 / 0.155 | 0.141 | – |
| **διόρθωση, ×8 (`fix171`)** | **475.9 s · 434.1 s → 6.9** | 0.63 | 17.4 | 0.182 | 0.27 | 3.64 / 0.155 | 0.141 | **ταυτόσημη (0 m)** |
| **διόρθωση + ×4 (`fixup4171`)** | **426.1 s · 385.4 s → 7.8** | 0.70 | 18.7 | 0.201 | 0.28 | 3.78 / 0.156 | 0.152 | διαφορετική (άλλη εικόνα) |

(σπόροι 1–3 της Γ, ×8: APE 0.196–0.202, RTE 0.26–0.28, RPE 3.61–3.88 / 0.157–0.162)

**Συμπέρασμα ⏳:** η διόρθωση δεν αλλάζει την τροχιά και κερδίζει 23 % (126 s στο process_scan, λιγότερο από τα 165 s της μετατροπής — μέρος του κέρδους «τρώγεται» από μεγαλύτερη αναμονή της εικόνας).
Το ×4 κερδίζει άλλα 11 % χωρίς απώλεια ακρίβειας (μέσα στους σπόρους). Το όχημα μένει στο 0.63–0.70 του πραγματικού χρόνου· επόμενο όριο πιθανότατα η εικόνα και ο ICP (42 ms / σάρωση).

---

### 2026-10-05 — #170 Προφίλ ενός run του Boreas (Γ, `--parallel`): το 29 % του χρόνου ήταν μετατροπή σημείων από μη συνεχή πίνακα — διορθώθηκε στον reader

**Σχετικά:** #166, #169 · κλάδος `speed_test` · `kiss_runs/prof170_launch.sh`, `kiss_runs/profile_wrap.py` (το `run_ncd.py` τελειώνει με `os._exit`, άρα το `-m cProfile` δεν γράφει τίποτα), `kiss_runs/prof170_boreas.{prof,txt}` ·
μόνο η κύρια διεργασία (η εικόνα στη δική της, εδώ ως αναμονή)

| πού (κύρια διεργασία, 3000 σαρώσεις, 606 s συνολικά, process_scan 569 s) | s | ms / σάρωση |
|---|---|---|
| `kiss_icp` `preprocess`: **μετατροπή `_Vector3dVector` του `p[:, :3]` (μη συνεχής) σημείο προς σημείο** | **165.5** | 55 |
| ICP (`_align_points_to_map`, 3031 κλήσεις με τις δύο αρχές) | 127.9 | 42 |
| αναμονή για την κίνηση της εικόνας (`future.result`) | 127.0 | 42 |
| `voxel_down_sample` (C++ 52.1 + Python 7.3) | 59.5 | 20 |
| deskew του KISS (`_preprocess`, C++) | 24.7 | 8 |
| χάρτης του SLAM (`_integrate_frame`) + χάρτης του ICP (`_update`) | 23.8 + 15.4 | 13 |
| reader του Boreas (ανάγνωση `.bin`, φίλτρο) | 27.7 | 9 |
| loop closures (11 κόμβοι) + KD-tree δύο αρχών (31) | 12.0 + 5.9 | – |

Μικρο-μέτρηση (200 000 σημεία): `_Vector3dVector` σε slice 59.3 ms, σε συνεχή πίνακα 2.0 ms, ίδιες τιμές. Μόνο ο reader του Boreas επέστρεφε slice (`kiss_slam/tools/boreas.py`)·
διόρθωση `np.ascontiguousarray(p[:, :3])` (`fc45188`) — οι τιμές δεν αλλάζουν, άρα η τροχιά πρέπει να είναι ταυτόσημη.

**Συμπέρασμα ⏳:** το όχημα ήταν αργό κυρίως από ένα σφάλμα απόδοσης του reader (−150 s αναμένονται, ~5.3 → ~7 Hz), όχι από τη μέθοδο. Μετά τη διόρθωση η εικόνα (42 ms αναμονή / σάρωση ήδη) γίνεται
πιθανότατα το όριο — εκεί βοηθά το μικρότερο πανόραμα (#169). Επαλήθευση: ένα run με τη διόρθωση (ταυτόσημη τροχιά, χρόνος).

---

### 2026-10-05 — #169 Πανόραμα από τον αισθητήρα (auto) στη Γ: Boreas μόνο 11 % ταχύτερο, ίδια ακρίβεια· observatory-01 ξανά χαλά (ύψος)

**Σχετικά:** #093 / #096 (ο κανόνας), #166 (χρόνος της Γ) · κλάδος `speed_test` · ΑΠΟΦΑΣΗ Μ.Τ. 5/10 (auto πρώτα, μόνο `--parallel`) · `kiss_runs/auto169_launch.sh` · Γ + `--panorama-width=auto --panorama-up=auto --parallel`, σπόρος 0, ένα run κάθε φορά

| ακολουθία | πανόραμα | χρόνος · process_scan | APE m | RTE % | RPE 1 m cm / ° | RRE °/100 m | z RMSE m |
|---|---|---|---|---|---|---|---|
| Boreas, Γ ×8 (`lockpar166_s0`; `bfc150` s1–s3) | 1024 × 1024 | 616.5 s · 560.6 s (5.4 Hz) | 0.182 (0.196–0.202) | 0.27 (0.26–0.28) | 3.64 / 0.155 (3.61–3.88 / 0.157–0.162) | 0.130 | 0.141 |
| **Boreas, auto** | **1800 × 128** | **551.5 s · 515.8 s (5.8 Hz)** | 0.206 | 0.27 | 3.81 / 0.151 | 0.129 | 0.146 |
| observatory-01, Γ (`bfc150` s0–s3) | 1024 × 8 | 771.8 s (s0, σε batch) | 0.061–0.077 | 0.25–0.27 | 5.54–5.63 / 0.727–0.733 | 0.540–0.576 | 0.033–0.057 |
| **observatory-01, auto** | **600 × 161** | 234.7 s (51 Hz) | **0.261** | **0.46** | 5.38 / 0.682 | **1.354** | **0.235** |

**Συμπέρασμα ⏳:**
1. **Boreas:** 4.5× λιγότερα pixel αλλά μόνο **−11 % χρόνος** (5.4 → 5.8 Hz ανά σάρωση)· ακρίβεια ίδια (APE μέσα στους 4 σπόρους). Το πανόραμα **δεν** είναι το κύριο κόστος στο όχημα — ούτε ο KISS-SLAM
   είναι γρήγορος εκεί (`kiss_s0` 1381 s, σε batch). Επόμενο: προφίλ ενός run του Boreas.
2. **observatory-01:** το πρόβλημα του #096 αναπαράγεται με τη Γ: APE 0.261 (Γ 0.061–0.077), z RMSE 0.235 (0.03–0.06), RRE 2.4× — απόκλιση ύψους / κλίσης, ενώ το τοπικό RPE είναι καλύτερο (−3 % / −7 %).
   Ο κανόνας auto δεν γίνεται προεπιλογή.

---

### 2026-10-05 — #168 Χρόνος με όριο 4 νημάτων ανά βιβλιοθήκη: ίδια τροχιά, 3–8 % πιο αργά· πραγματικός χρόνος στη φορητή με `--parallel`, όχι στο όχημα

**Σχετικά:** #166 (χωρίς όριο), commit `25e487f` (`KISS_THREADS`) · `kiss_runs/thr168_launch.sh` · `KISS_THREADS=4 OMP / OPENBLAS / MKL / NUMEXPR_NUM_THREADS=4`, ICP 4 (`threads4.yaml`) · σπόρος 0, ένα run κάθε φορά

| ακολουθία (σαρώσεις, διάρκεια) | | χρόνος (χωρίς όριο, #165 / #166) | process_scan → Hz ανά σάρωση | × πραγματικού (με εκκίνηση) | CPU (πυρήνες) | max RSS | τροχιά έναντι χωρίς όριο |
|---|---|---|---|---|---|---|---|
| quad_easy (1991, 199 s) | σειριακά | 259.4 s (240.5) | 225.9 s → 8.8 Hz | 0.77 | 7.4 | 1.23 GB | ταυτόσημη |
| | `--parallel` | 203.8 s (193.6) | 176.1 s → **11.3 Hz** | 0.98 | 9.5 | 1.14 GB | ταυτόσημη |
| christ-church-03 (3123, 312 s) | σειριακά | 289.3 s (274.4) | 264.1 s → **11.8 Hz** | 1.08 | 6.0 | 1.16 GB | ταυτόσημη |
| | `--parallel` | 256.2 s (241.3) | 235.6 s → **13.3 Hz** | 1.22 | 6.9 | 1.10 GB | ταυτόσημη |
| Boreas (3000, 300 s) | σειριακά | 781.9 s (763.8) | 748.2 s → 4.0 Hz | 0.38 | 8.2 | 2.56 GB | ταυτόσημη |
| | `--parallel` | 638.6 s (616.5) | 604.5 s → 5.0 Hz | 0.47 | 10.7 | 2.51 GB | ταυτόσημη |

**Συμπέρασμα ⏳:**
1. Το όριο νημάτων αλλάζει μόνο την ταχύτητα (τροχιές ταυτόσημες, 0 m)· κόστος 3–8 %.
2. Το όριο είναι **ανά βιβλιοθήκη**, όχι συνολικό: η διεργασία χρησιμοποιεί 6–11 πυρήνες (με `--parallel` η διεργασία της εικόνας έχει δικές της δεξαμενές). Για συνολικό όριο πυρήνων χρειάζεται `taskset` (δεν δοκιμάστηκε).
3. **Πραγματικός χρόνος στη φορητή:** Hesai 11.8 Hz σειριακά / 13.3 Hz `--parallel`· Ouster 128 11.3 Hz ανά σάρωση με `--parallel` (με την εκκίνηση 0.98×), όχι σειριακά (8.8 Hz). **Όχι στο όχημα** (4–5 Hz).

---

### 2026-10-05 — #167 Οι δύο γραμμές του ablation που έλειπαν για το όχημα (Boreas × 4): η εικόνα μόνη αποτυγχάνει, η εφεδρεία απόστασης τη σώζει

**Σχετικά:** #165 (ablation), #127 / #164 (Boreas) · κλάδος `ral_method`, ρητές επιλογές (`--no-upright --guided=none --range=none --cv-blend=off --fallback=identity --fallback-after=1`) · `kiss_runs/abl167_launch.sh`
(η πρώτη εκκίνηση έτρεξε χωρίς τις επιλογές — `export -f job OLD` δεν εξήγαγε τη μεταβλητή· σταμάτησε μετά από ~5 λεπτά, διορθώθηκε, ξανά) · `--last=3000 --intensity-scale=1.0`, ATE / RPE με evo

| γραμμή (Boreas, 3000 σαρώσεις) | APE m (4 σπόροι) | μ. ό. | RTE % | RPE 1 m cm / ° | APE / KISS |
|---|---|---|---|---|---|
| KISS-SLAM (1 run) | 0.266 | 0.266 | 0.28 | 4.14 / 0.192 | 1.00 |
| + κίνηση εικόνας (`ablsurf167`) | 7.50 · 13.81 · 39.37 · 62.24 | 30.7 | 3.45–10.05 | 6.78–7.65 / 0.277–0.427 | **115.5** |
| + δύο αρχές (`ablsurftwo167`) | 3.27 · 3.30 · 3.23 · 3.09 | 3.22 | 1.04–1.37 | 5.91–6.26 / 0.210–0.222 | **12.1** |
| + εφεδρεία απόστασης (`surftworangefb`, υπάρχοντα) | 0.272 · 0.271 · 0.280 · 0.270 | 0.273 | 0.36–0.37 | 6.33–6.55 / 0.241–0.244 | 1.03 |

**Συμπέρασμα ⏳:** στο όχημα η εικόνα intensity αποτυγχάνει στο 31 % των σαρώσεων (935 / 3000 από την εικόνα απόστασης στο `surftworangefb_s0`)· χωρίς εφεδρεία ο ICP ξεκινά τότε από μηδενική κίνηση
στα ~10 m/s και η τροχιά σπάει (APE 7–62 m). Οι δύο αρχές περιορίζουν τη ζημιά (3.2 m, σταθερά), μόνο η εφεδρεία απόστασης φέρνει το όχημα στο επίπεδο του KISS. Το RPE 1 m δεν σπάει τόσο — η αποτυχία είναι
λίγα μεγάλα άλματα, όχι γενικό σφάλμα. Ο πίνακας `docs/ablation_165.md` ξαναγράφτηκε με τη στήλη του οχήματος πλήρη.

---

### 2026-10-05 — #166 Χρόνος εκτέλεσης της κλειδωμένης μεθόδου (Γ) έναντι της παλιάς προεπιλογής: ίδιο κόστος· πραγματικός χρόνος στη φορητή, όχι στο όχημα (3.9–4.9 Hz)

**Σχετικά:** #161, #165 · ΑΠΟΦΑΣΗ Μ.Τ. 5/10 (δοκιμές χρόνου) · κλάδος `ral_method` · `kiss_runs/time166_launch.sh` · σπόρος 0, ένα run κάθε φορά, μηχάνημα κατά τα άλλα αδρανές, νήματα χωρίς όριο
(«παλιά» = `--cv-blend=off --fallback=identity --fallback-after=1`, η συμπεριφορά του base092)

| ακολουθία (σαρώσεις, διάρκεια) | εκδοχή | χρόνος (με εκκίνηση) | Hz | × πραγματικού χρόνου | max RSS | APE m · RTE % · RPE 1 m cm / ° |
|---|---|---|---|---|---|---|
| christ-church-03 (3123, 312 s) | παλιά σειριακά | 287.0 s | 10.9 | 1.09 | 1.18 GB | 0.039 · 0.13 · 3.71 / 0.702 |
| | παλιά `--parallel` | 241.2 s | 12.9 | 1.29 | 1.10 GB | ίδια |
| | **Γ σειριακά** | **274.4 s** | **11.4** | 1.14 | 1.16 GB | 0.039 · 0.14 · 3.97 / 0.708 |
| | **Γ `--parallel`** | **241.3 s** | **12.9** | 1.29 | 1.09 GB | ίδια |
| Boreas (3000, 300 s) | παλιά σειριακά | 774.3 s | 3.9 | 0.39 | 2.42 GB | 0.239 · 0.34 · 4.90 / 0.212 |
| | παλιά `--parallel` | 616.8 s | 4.9 | 0.49 | 2.72 GB | ίδια |
| | **Γ σειριακά** | **763.8 s** | **3.9** | 0.39 | 2.66 GB | 0.182 · 0.27 · 3.64 / 0.155 |
| | **Γ `--parallel`** | **616.5 s** | **4.9** | 0.49 | 2.55 GB | ίδια |
| quad_easy (1991, 199 s) | **Γ `--parallel`** | **193.6 s** | **10.3** | 1.03 | 1.14 GB | 0.078 · 0.26 · 5.45 / 0.575 |

Σειριακά και `--parallel` δίνουν **ταυτόσημη** τροχιά· η Γ ταυτόσημη με το `bfc150_s0` του #164 (στις τρεις ακολουθίες).

**Διόρθωση στο #161** (η εγγραφή μένει ως έχει): εκεί οι δύο χρόνοι είναι ανεστραμμένοι — κατά τα logs `classic161_s0` (σειριακά) = 192.9 s και `classicpar161_s0` (`--parallel`) = 244.5 s.
Με τον χρόνο της Γ σειριακά στο quad_easy (#165, 240.5 s) και παράλληλα (193.6 s), το 192.9 s σειριακά του #161 δεν συμφωνεί με τα υπόλοιπα (στο church_03 και στο Boreas το `--parallel`
είναι σταθερά 15–20 % ταχύτερο)· πιθανότατα άλλο φορτίο στο μηχάνημα σε ένα από τα δύο runs. Για το quad_easy αξιόπιστο είναι μόνο το 193.6 s (`--parallel`, Γ) αυτής της εγγραφής.

**Συμπέρασμα ⏳:**
1. Η ανάμειξη και η εφεδρεία KISS **δεν κοστίζουν** (church_03 274 έναντι 287 s, Boreas 764 έναντι 774 s σειριακά· ίσο παράλληλα — μέσα στον θόρυβο του χρόνου).
2. **Πραγματικός χρόνος στις φορητές** (Ouster 128 και Hesai, 10 Hz): 10.3–12.9 Hz με `--parallel`, 10.9–11.4 Hz σειριακά στο Hesai.
3. **Όχι στο όχημα:** Boreas 3.9 Hz σειριακά, 4.9 Hz παράλληλα (0.4–0.5 του πραγματικού χρόνου), μνήμη 2.4–2.7 GB. Πιθανή αιτία το πανόραμα 1024 × 1024 (128 δακτύλιοι × 8)· δοκιμή ×2 / ×4 προτείνεται (#089, #096).

---

### 2026-10-05 — #164 Η επιλογή Γ σε όλες τις 42 ακολουθίες + Boreas × 4: = Β σε 30, διορθώνει το spms, καμία απόκλιση

**Σχετικά:** #149 (Α), #150–#151 (Γ, υπο-δοκιμή) · ΑΠΟΦΑΣΗ Μ.Τ. 5/10 · launcher `kiss_runs/bfc164_launch.sh` (144 νέα + 28, 172 / 172) · βραχίονας `bfc150` · πίνακες
`kiss_runs/results_official_allarms_164.{md,csv}`, **`docs/results_summary_164.md`** (όλοι οι βραχίονες, ο Γ προστέθηκε)

**Η Γ διαφέρει από τη Β (> 0.5 %) μόνο σε 13 ακολουθίες** (ATE m, μ. ό. 4 σπόρων — KISS / προεπιλογή / Β / Α / **Γ**, Γ έναντι Β):
eee_01 2.678 / 1.483 / 1.553 / 1.454 / **1.601** (+3 %)· eee_02 1.486 / 0.836 / 0.790 / 0.797 / **0.812** (+3 %)· eee_03 0.864 / 0.360 / 0.142 / 0.140 / **0.145** (+2 %)· sbs_01 0.976 / 0.354 / 0.367 / 0.390 /
**0.370** (+1 %)· sbs_02 1.143 / 0.759 / 0.767 / 0.603 / **0.723** (−6 %)· sbs_03 1.214 / 0.793 / 0.815 / 0.850 / **0.810** (−1 %)· rtp_01 3.985 / 0.264 / 0.240 / 0.220 / **0.249** (+4 %)· rtp_02 3.575 / 2.604 / 0.335 / 0.327 /
**0.337** (+1 %)· rtp_03 3.408 / 0.232 / 0.215 / 0.227 / **0.226** (+5 %)· **spms_01 8.743 / 8.646 / 8.047 / 10.700 / 6.668 (−17 %)**· **spms_02 15.40 / 48.30 / 48.10 / 7.298 / 8.329 (−83 %)**· **spms_03 9.518 / 26.30 /
17.60 / 0.410 / 0.818 (−95 %)**· dynamic_spinning (ειδική) 0.159 / 0.171 / 0.111 / 1.415 / **0.114** (+3 %). Στις άλλες 30 ταυτόσημη με τη Β (η εφεδρεία δεν ενεργοποιείται).
**Ανά dataset** (γεωμ. μέσος APE / KISS, Β → Γ): NCD 0.41 → 0.41, Spires 0.16 → 0.16, Hilti 0.24 → 0.24, NTU eee 0.37 → 0.38, **νέες NTU 0.44 → 0.32** (Α 0.31), όχημα 0.73 → 0.73· RPE 1 m (17)
ίδιο. (Η χειρότερη «μέση θέση» της Γ στον πίνακα είναι τεχνούργημα: σε ισοπαλία με τη Β η κατάταξη τη βάζει τελευταία λόγω σειράς.)
**Συμπέρασμα ⏳:** η Γ κρατά ό,τι δίνει η Β (όχημα, drone, ουδέτερη στη φορητή) και προσθέτει τη διόρθωση της αποτυχίας σε ύψος (spms έως −95 %), χωρίς την απόκλιση της Α στη γρήγορη
περιστροφή. Κόστος: μια παράμετρος (4, από τα δεδομένα) και ±1–5 % σε eee / rtp (μέσα στη διασπορά). Υποψήφια για κλείδωμα (ΑΠΟΦΑΣΗ Μ.Τ.).

---

### 2026-10-05 — #163 Deskew της σάρωσης που κερδίζει η σταθερή ταχύτητα (δύο αρχές) με την κίνηση του ICP: όχι

**Σχετικά:** #057–#059, #108 / #119 · ζήτημα Μ.Τ. 5/10 · νέα επιλογή `image_deskew.two_start_cv_deskew` (`--cv-winner-deskew`): η σάρωση που μπαίνει στον χάρτη, όταν κερδίζει η αρχή σταθερής ταχύτητας
(χωρίς deskew), διορθώνεται με το βήμα του ICP (P_{k−1}⁻¹P_k, κλιμακωμένο σε μία σάρωση από τους χρόνους θέσης), εκφρασμένη στη στιγμή της θέσης· χωρίς δεύτερη καταχώριση · `kiss_runs/cvd163_launch.sh`

**Πότε κερδίζει η σταθερή ταχύτητα** (base092 σπόρος 0, 42 ακολουθίες, `two_start.csv`): 0.02–4.4 % των σαρώσεων — quad_hard 4.4 %, math_medium 4.4 %, UZH 3.1 %, rtp_01 2.7 %, stairs / spms_01 / keble / bodleian /
Office_Mitte_1 ~2 %, οι περισσότερες < 0.2 %· απόκλιση εικόνας–CV διάμεσος 5.5–10°· η αρχή της εικόνας ταιριάζει στον χάρτη 2–86 % χειρότερα (συνήθως 2–4 %· 1.3–1.9 σε rtp / spms / eee, όπου η εφεδρεία
απόστασης χρησιμοποιείται συχνά).
**Ένα run, σπόρος 0** (APE m · RTE % · RPE 1 m cm / °): quad_hard — προεπιλογή 0.212–0.261 · 0.81–0.88 · 8.18–8.54 / 1.023–1.059· **με την επιλογή 0.226 · 0.97 · 8.47 / 1.102** (πρώτη διαφορά στη σάρωση 138)·
math_medium — 0.150–0.157 · 0.93–0.97 · 12.87–12.97 / 2.440–2.460· **0.154 · 0.96 · 12.80 / 2.455** (από τη σάρωση 54).
**Συμπέρασμα ⏳:** καμία βελτίωση· στο quad_hard RTE και στροφή λίγο έξω από τη διασπορά προς το χειρότερο. Πιθανώς όταν κερδίζει η σταθερή ταχύτητα η κίνηση είναι ακανόνιστη και το βήμα του ICP
(από το τέλος της προηγούμενης ως τη μέση αυτής) δεν περιγράφει καλά τη σάρωση. Δεν μπαίνει στη μέθοδο.

---

### 2026-10-05 — #161 Η κλασική μέθοδος: ίδιο αποτέλεσμα με το base092 και ταχύτητα μόνη της (σειριακά 8–9 Hz, παράλληλα 10–12 Hz)

**Σχετικά:** #087 (πραγματικός χρόνος με `--parallel`), #092 (base092) · ΑΠΟΦΑΣΗ Μ.Τ. 5/10 («άφησε το Β.9 joint ανοιχτό, πίσω στην κλασική μέθοδο») · κύριος checkout `after_091` · `kiss_runs/classic161_launch.sh`

quad_easy, σπόρος 0, μόνη στο μηχάνημα, ρύθμιση base092 (`--two-start=5 --range=fallback`):
| | χρόνος | μέση συχνότητα / process_scan | APE · RTE · RPE 1 m · μήκος | διαφορά από base092_s0 |
|---|---|---|---|---|
| base092_s0 (σε batch με άλλα runs) | 620.2 s | – | 0.078 · 0.27 · 5.47 / 0.592 · 265.9 | – |
| **σειριακά** | **244.5 s** (≈ 8.1 Hz με εκκίνηση) | 9 Hz / 107 ms | ίδια | 2.1·10⁻¹³ m |
| **`--parallel`** (εικόνα σε διεργασία) | **192.9 s** (≈ 10.3 Hz) | 12 Hz / 84 ms | ίδια | 1.8·10⁻¹³ m |

**Συμπέρασμα ⏳:** ο κώδικας της κλασικής μεθόδου δίνει ακριβώς το αποτέλεσμα των runs του πίνακα (καμία από τις αλλαγές των #126–#160 δεν την επηρεάζει χωρίς τις επιλογές τους). Πραγματικός χρόνος με
`--parallel` (10.3 Hz συνολικά, 12 Hz ανά σάρωση)· σειριακά λίγο κάτω (8–9 Hz).

---

### 2026-10-05 — #160 Υπο-δοκιμή του Β.9 joint (7 × 4): μεγάλα κέρδη σε quad_easy / eee_01, αποκλίσεις στροφής σε 5 / 7

**Σχετικά:** #155 (ένα run στα quad_easy / stairs), #159 (έκδοση πραγματικού χρόνου) · κλάδος `b9_joint_new`, `--ct=joint --ct-lambda=0.1 --parallel` (χωρίς δύο αρχές) · `kiss_runs/ctj160_launch.sh` ·
βραχίονας `ctj159`

Μ. ό. ± σ [ανά σπόρο], 4 σπόροι. APE / ATE m · RTE % · RPE 1 m cm / ° · μήκος m:

| ακολουθία | KISS APE | προεπιλογή | ανάμειξη deskew | **Β.9 joint** |
|---|---|---|---|---|
| quad_easy | 0.108 | 0.077 ± 0.001 · 0.263 · 5.48 / 0.592 · 266 | 0.077 · 0.260 · 5.40 / 0.574 · 265 | **0.068 ± 0.001 · 0.220 · 3.62 / 0.363 · 249** |
| cloister | 0.181 | 0.180 ± 0.016 · 0.297 · 6.24 / 0.711 · 451 | 0.180 · 0.295 · 6.19 / 0.690 · 450 | 0.484 ± 0.012 · 0.703 · 4.85 / 0.866 · 424 |
| underground_hard | 3.846 | 0.091 ± 0.001 · 0.247 · 5.90 / 1.088 · 251 | 0.091 · 0.247 · 5.93 / 1.116 · 252 | 3.726 ± 0.224 · 2.990 · 7.14 / 1.342 · 236 |
| christ-church-03 | 0.122 | 0.039 ± 0.000 · 0.125 · 3.79 / 0.700 · 285 | 0.039 · 0.133 · 3.93 / 0.705 · 285 | 0.355 ± 0.026 · 0.648 · 4.33 / 0.927 · 269 |
| keble-03 | 0.854 | 0.089 ± 0.003 · 0.677 · 8.19 / 1.040 · 347 | 0.090 · 0.663 · 8.09 / 1.031 · 343 | 11.54 ± 7.87 [16.7 9.4 1.36 18.7] · 35.2 · 12.39 / 4.668 · 297 |
| eee_01 | 2.678 | 1.483 ± 0.149 | 1.553 ± 0.127 | **0.569 ± 0.222** [0.37 0.71 0.81 0.39] |
| Boreas | 0.266 | 0.233 ± 0.008 · 0.350 · 4.96 / 0.210 · 1333 | 0.195 · 0.270 · 3.71 / 0.158 · 1324 | 35.0 ± 22.2 [29.3 31.6 66.0 13.3] · 15.3 · 4.97 / 0.749 · 1320 |

**Συμπέρασμα ⏳:** το Β.9 joint κερδίζει πολύ εκεί όπου η κίνηση είναι πλούσια σε γεωμετρία και η εικόνα πυκνή (quad_easy RPE −34 / −39 %, eee_01 −62 %), αλλά σε 5 / 7 η τροχιά αποκλίνει. Κοινό
χαρακτηριστικό: η στροφή — RPE στροφής +22 % (cloister), +23 % (underground_hard), +32 % (christ-church-03), ×4.5 (keble), ×3.6 (Boreas)· η μετατόπιση συχνά καλύτερη (cloister −22 %) ή ίση (Boreas).
Πιθανές αιτίες προς διάγνωση: (α) ο περιορισμός ταχύτητας (λ = 0.1, από τα Livox) και η πρώτης τάξης Ιακωβιανή του τραβούν τη στροφή· (β) χωρίς τις δύο αρχές / την εφεδρεία KISS, μια λάθος
αρχή δεν διορθώνεται (underground_hard = KISS)· (γ) οι 30 επαναλήψεις χωρίς σύγκλιση. Η προεπιλογή δεν αλλάζει.

---

### 2026-10-05 — #159 Β.9 σε πραγματικό χρόνο: 10.5 Hz στο quad_easy, ίδια τροχιά (κλάδος `b9_joint_new`)

**Σχετικά:** #158 · ζήτημα Μ.Τ. 5/10 («μπορείς πιο γρήγορα;») · `b9_joint_new` `a236940` · `kiss_runs/ct159_launch.sh`

**Profile του #158** (κύρια διεργασία, 120 s, 0.111 s / σάρωση): C++ βρόχος 30 ms (~28 επαναλήψεις / σάρωση)· deskew όλης της σάρωσης ×3 σε Python 16 ms· υπόλοιπα εικόνας σε κάθε επανάληψη ~8 ms·
ανάγνωση bag 11 ms· βήμα περιορισμών (αντικείμενα `Rotation` της scipy, 125 000 κλήσεις) 7 ms· δειγματοληψία voxel 7 ms· αναμονή εικόνας 4 ms· άλλα ~25 ms.
**Αλλαγές (ίδιο αποτέλεσμα):** τοποθέτηση / deskew όλης της σάρωσης και υπόλοιπα εικόνας σε C++ (ίσα με την Python ως 10⁻¹⁴ / 10⁻¹⁷)· δικές μου log / exp περιστροφής αντί για αντικείμενα της scipy
(exp ως 2·10⁻¹⁶· log ως 7.7·10⁻¹⁰ rad, μόνο κοντά στις 180°)· ανάγνωση σαρώσεων από νήμα στο παρασκήνιο, με τη σειρά (`--parallel`).
**quad_easy, σπόρος 0, μόνο του:** **195.0 s, 10.5 σαρώσεις / s κατά μέσο όρο**· APE 0.068 · RTE 0.22 · RPE 3.66 / 0.370 · μήκος 248.8· μέγιστη διαφορά θέσης από το #158: 1.2·10⁻¹¹ m.
Πορεία: ~830 → 444.6 → 288.9 → 226.2 → **195.0 s** (×4.3). Όριο τώρα: η κίνηση της εικόνας στη διεργασία (~0.09 s / σάρωση, SURF + αντιστοίχιση + προσαρμογή).

---

### 2026-10-05 — #158 Ο εσωτερικός βρόχος του Β.9 σε C++: ~8.8–9.4 Hz, ίδια τροχιά (νέος κλάδος `b9_joint_new`)

**Σχετικά:** #156, #157 · ζήτημα Μ.Τ. 5/10 («βήμα 2 σε νέο κλάδο») · **κλάδος `b9_joint_new`** (από `b9_joint`, `origin/b9_joint_new`): `8f6ed5c`, `eca0254` · `kiss_slam/cpp/ct_accumulate.cpp`,
`scripts/build_ct_accumulate.sh` (→ `kiss_slam/_ct_accumulate*.so`, gitignored)· η έκδοση Python μένει (`KISS_CT_PYTHON=1`) · `kiss_runs/ct158_launch.sh`

Σε C++ (μονονηματικό, ντετερμινιστικό): τοποθέτηση κάθε σημείου στον χρόνο του (Rodrigues), πλησιέστερο σημείο του χάρτη σε πλέγμα, άθροιση του συστήματος Gauss–Newton 12 × 12· σε Python
μένουν οι περιορισμοί του ECTLO, η ενημέρωση και τα λίγα υπόλοιπα της εικόνας. Έλεγχος: συνθετική σκηνή — ίδιο αποτέλεσμα με την Python ως 10⁻¹⁷· πλέγμα έναντι KD-tree (200 000 σημεία χάρτη,
20 000 ερωτήματα) — ίδιοι γείτονες 20 000 / 20 000, αθροίσματα ως 10⁻¹⁴. **Πρώτη εκδοχή** (κελί = ακτίνα, 2√3 · voxel = 3.5 m): 27 μεγάλα κελιά ανά ερώτημα — πιο αργή (5.3 Hz)· **διόρθωση** (#158b):
κελιά μισού voxel, αναζήτηση σε κελύφη προς τα έξω με ακριβή διακοπή (κέλυφος r ≥ (r−1) κελιά μακριά), ~3× ταχύτερη.
**quad_easy, σπόρος 0, μόνο του, `--parallel`:** **226.2 s** (≈ 8.8 σαρώσεις / s συνολικά μαζί με την εκκίνηση, ~9.4 σταθερά)· APE 0.068 · RTE 0.22 · RPE 3.66 / 0.370 · μήκος 248.8 — μέγιστη διαφορά
θέσης από το #157: 5.6·10⁻¹³ m.
**Πορεία ταχύτητας** (quad_easy, μόνο του): πρωτότυπο ~830 s (2.4 Hz) → #156 444.6 s → #157 288.9 s → **#158 226.2 s**· ×3.7 χωρίς καμία αλλαγή αποτελέσματος. Σχεδόν πραγματικός χρόνος (10 Hz).

---

### 2026-10-05 — #157 Β.9 με την εικόνα σε παράλληλη διεργασία: 6.9 Hz, ίδια τροχιά (κλάδος `b9_joint`)

**Σχετικά:** #156 · ζήτημα Μ.Τ. 5/10 («ξεκίνα τις αλλαγές ταχύτητας, δοκιμή στην ίδια ακολουθία και σπόρο») · `b9_joint`: `31150fd`, `ba1e010` · `kiss_runs/ct157_launch.sh`

Αλλαγές: (1) η διεργασία της εικόνας επιστρέφει κίνηση, inliers **και τα ζεύγη** (η joint τα χρειάζεται)· (2) διεργασία «spawn» από worktree: φόρτωνε τον κύριο checkout (το `_kiss_slam_editable.pth`
ξαναμπαίνει σε κάθε νέο διερμηνέα — πρώτη δοκιμή: σφάλμα, η παλιά `_motion_worker` επέστρεφε μόνο την κίνηση)· τώρα το `worktree_run.py` δίνει στις διεργασίες
`scripts/_python_worktree.sh` (`python -S`, ο worktree πρώτος, μετά τα site-packages)· η διεργασία γράφει ποιο αρχείο φόρτωσε (`…/Kiss_SLAM-b9/kiss_slam/intensity_deskew.py`).
**quad_easy, σπόρος 0, μόνο του, `--parallel`:** 288.9 s (≈ 6.9 σαρώσεις / s· #156 444.6 s, #155 ~830 s)· APE 0.068 · RTE 0.22 · RPE 3.66 / 0.370 · μήκος 248.8 — μέγιστη διαφορά θέσης από το #156: 0 m.
Οι βραχίονες του κύριου checkout δεν επηρεάζονται (καμία αλλαγή στο `after_091`).
**Συμπέρασμα ⏳:** ×2.9 από το πρωτότυπο χωρίς αλλαγή αποτελέσματος· για 10 Hz μένει η καταχώριση σε C++ (KD-tree → πίνακας voxel του KISS).

---

### 2026-10-05 — #156 Ταχύτητα του Β.9: profile και τρεις διορθώσεις χωρίς αλλαγή μεθόδου — ~1.9× ταχύτερο, ίδια τροχιά (νέος κλάδος `b9_joint`)

**Σχετικά:** #155 · ζήτημα Μ.Τ. 5/10 («πολύ πιο αργό από πραγματικό χρόνο;», «σε νέο κλάδο b9 joint») · **κλάδος `b9_joint`** (από `deskew_in_icp`, worktree `/home/photogrammetry/Kiss_SLAM-b9`,
`origin/b9_joint`): `8c58758` (σύνδεση `ct_max_iter`, ήταν μη δεσμευμένη), `990e1b9` (#156) · profile με cProfile (`scratchpad/profile_run.py`, 180 s) · `kiss_runs/ct156_launch.sh`

**Profile** (B.9 joint, quad_easy, μόνο του, 436 σαρώσεις σε 180 s = 0.41 s / σάρωση, 2.40 Hz): καταχώριση συνεχούς χρόνου 0.30 s (72 %) — αναζήτηση γειτόνων `cKDTree.query(workers=-1)` ~0.09 s
(550 000 εκκινήσεις νημάτων Python), τοποθέτηση σημείων με `Slerp` / `Rotation` της scipy ~0.08 s, `np.unique(axis=0)` στη δειγματοληψία voxel ~0.05 s, άλγεβρα Gauss–Newton και τα υπόλοιπα ~0.08 s·
κίνηση εικόνας 0.09 s (21 %, ίδια με τη μέθοδο)· ανάγνωση / χάρτης 0.02 s.
**Διορθώσεις** (`kiss_slam/ct_registration.py`): slerp σε κλειστή μορφή R(s) = R_b exp(s log(R_bᵀ R_e)) με Rodrigues (διαφορά από τη scipy 1.8·10⁻¹⁴· 5000 σημεία σε 0.6 ms)· δειγματοληψία voxel με ένα κλειδί
int64 (ίδια σημεία, ελεγμένο)· `workers=1` στο KD-tree (ίδιοι γείτονες).
**Έλεγχος** (quad_easy, σπόρος 0): χρόνος **444.6 s** μόνο του (≈ 4.5 σαρώσεις / s· πριν ~830 s μόνο του κατά το profile, 875 s με 6 παράλληλα)· APE 0.068 · RTE 0.22 · RPE 3.66 / 0.370 · μήκος 248.8 —
ίδια με το #155· μέγιστη διαφορά θέσης ανά σάρωση 5.4·10⁻¹² m.
**Συμπέρασμα ⏳:** ~1.9× ταχύτερο χωρίς καμία αλλαγή αποτελέσματος. Για 10 Hz: η εικόνα σε παράλληλη διεργασία (όπως `--parallel`, −0.09 s) και η καταχώριση σε C++ με την αναζήτηση στον
πίνακα voxel του KISS.

---

### 2026-10-05 — #155 Β.9 joint σε περιστρεφόμενο αισθητήρα (ένα run): η καταχώριση συνεχούς χρόνου με τις αντιστοιχίσεις της εικόνας ως υπόλοιπα — quad_easy RPE −33 / −37 %, stairs μήκος = GT

**Σχετικά:** #115 (πρωτότυπο, Livox ~2× χειρότερο από τον KISS· joint δεν είχε δοκιμαστεί), #128 / #129 (η εικόνα κερδίζει όπου ο ICP αποτυγχάνει) · ζήτημα Μ.Τ. 5/10 · worktree `deskew_in_icp`
(`scripts/worktree_run.py`), `--ct=cv | image | joint --ct-lambda=0.1` (χωρίς δύο αρχές, με εφεδρεία απόστασης) · launcher `kiss_runs/ct155_launch.sh` · βραχίονες `ctcv155`, `ctimage155`, `ctjoint155`

Ένα run, σπόρος 0, έναντι 4 σπόρων. APE m · RTE % · RPE 1 m cm / ° · μήκος m:

| | quad_easy (GT 246.6 m) | stairs (GT 57.0 m) |
|---|---|---|
| προεπιλογή (4) | 0.077–0.078 · 0.26–0.27 · 5.46–5.51 / 0.589–0.593 · 265.6–266.1 | 1.49–2.33 · – · 36.5–38.7 / 6.52–7.64 · 94.3–98.3 |
| ανάμειξη deskew (4) | 0.077–0.078 · 0.26 · 5.34–5.45 / 0.570–0.576 · 264.4–264.9 | 1.37–2.05 · – · 31.8–40.6 / 6.55–8.32 · 91.6–98.3 |
| CT μόνο LiDAR (αρχή σταθ. ταχύτητας) | κατάρρευση (σήμα 6 στη σάρωση 1280 / 1991) | 5.233 · – · 163.1 / 55.0 · 217.2 (αποκλίνει) |
| CT, αρχή εικόνας | 0.076 · 0.23 · 4.07 / 0.372 · 250.6 | 2.468 · – · 64.6 / 16.6 · 113.7 |
| **CT joint** | **0.068 · 0.22 · 3.66 / 0.370 · 248.8** | **1.083 · – · 20.7 / 5.53 · 56.4** |

Χρόνος: ~2.1–2.3 σαρώσεις / s (Python, KD-tree)· quad_easy 848–875 s.
**Συμπέρασμα ⏳:** σε περιστρεφόμενο αισθητήρα η καταχώριση συνεχούς χρόνου με αρχή την εικόνα είναι ήδη πολύ καλύτερη από τη μέθοδο στο quad_easy (RPE −26 / −37 %), και η εκδοχή joint
προσθέτει ακόμη (APE −12 %, RPE μετατόπισης −33 %)· στο stairs η joint είναι η πρώτη ρύθμιση που δίνει σωστό μήκος (56.4 έναντι 57.0 m, η προεπιλογή ~95) και RPE −44 / −20 %. Η εικόνα
είναι απαραίτητη: χωρίς αυτήν η CT αποκλίνει (stairs) ή καταρρέει (quad_easy). Σε αντίθεση με τα Livox (#115) εδώ η εικόνα είναι πυκνή και ακριβής. Ένα run — επόμενο: υπο-δοκιμή·
και η κατάρρευση (σήμα 6) να διερευνηθεί.

---

### 2026-10-05 — #152 / #153 Σύντηξη απόστασης μόνο σε αδύναμες σαρώσεις (ένα run) · christ-church-02 με 8 σπόρους

**Σχετικά:** #135 (σύντηξη πάντα), #140 (απώλεια christ-church-02) · ανοιχτά ερωτήματα 5 / 7 (Μ.Τ. 5/10) · νέα τιμή `image_deskew.fuse_range: weak` (`--fuse-range=weak`): οι αντιστοιχίσεις της
εικόνας απόστασης μόνο όταν οι αντιστοιχίσεις intensity < ½ του διάμεσου των 100 προηγούμενων της ίδιας ακολουθίας (στα πρώτα 150 του underground_hard: 12 / 150 σαρώσεις) · launcher
`kiss_runs/q152_launch.sh` · βραχίονες `frw152`, `base092_s4–7`, `bd137_s4–7`

**#152** (APE m · RTE % · RPE 1 m cm / °): cloister — προεπιλογή 0.164–0.203 · 0.29–0.32 · 6.20–6.27 / 0.704–0.721· σύντηξη πάντα 0.206–0.232 · 0.33–0.34 · 6.50–6.55 / 0.722–0.741· **σε αδύναμες 0.202 · 0.33 ·
6.30 / 0.713**. underground_hard — 0.090–0.093 · 0.24–0.25 · 5.77–6.03 / 1.075–1.109· πάντα 0.084–0.087 · 0.22–0.23 · 5.56–5.68 / 0.989–1.040· **σε αδύναμες 0.089 · 0.24 · 5.77 / 1.051**. Boreas —
0.225–0.240 · 0.33–0.37 · 4.76–5.19 / 0.199–0.216· πάντα 0.209–0.861 · 0.29–0.55 · 4.03–4.41 / 0.145–0.155· **σε αδύναμες 0.232 · 0.34 · 4.67 / 0.192**.
**#153 christ-church-02, 8 σπόροι** (APE / z RMSE m · RPE): προεπιλογή 0.126 / 0.107, 0.196 / 0.185, 0.166 / 0.147, 0.152 / 0.137, 0.221 / 0.211, 0.187 / 0.175, 0.196 / 0.183, 0.192 / 0.179 — μ. ό. 0.179 ± 0.030,
διάμεσος 0.190· ανάμειξη 0.257 / 0.242, 0.250 / 0.239, 0.256 / 0.243, 0.146 / 0.132, **0.482 / 0.471**, 0.185 / 0.169, 0.205 / 0.191, 0.191 / 0.175 — 0.247 ± 0.103, διάμεσος 0.227· Mann–Whitney (ανάμειξη > προεπιλογή)
p = 0.064· RPE 1 m 5.14–5.33 / 0.702–0.711 έναντι 5.30–5.60 / 0.698–0.725.
**Συμπέρασμα ⏳:** (#152) η σύντηξη μόνο σε αδύναμες σαρώσεις μειώνει τη βλάβη του cloister αλλά και τα κέρδη (underground_hard −3 % αντί −7 %, όχημα στροφή −9 % αντί −28 %)· δεν είναι καλύτερη
συνολικά από το «ποτέ» — όχι πρώτη προτεραιότητα. (#153) με 8 σπόρους η βλάβη της ανάμειξης στο christ-church-02 μοιάζει περισσότερο πραγματική (διάμεσος 0.19 → 0.23 m, p = 0.06), αποκλειστικά
στο ύψος, με έναν σπόρο 0.48 m· το τοπικό σφάλμα δεν αλλάζει. Μηχανισμός ανοιχτός (βάρος εικόνας 0.95 εκεί, #140).

---

### 2026-10-05 — #151 Υπο-δοκιμή της επιλογής Γ (εφεδρεία kiss από την 4η συνεχόμενη αποτυχία), 7 × 4: διορθώνει τη γρήγορη περιστροφή, κρατά το spms

**Σχετικά:** #149, #150 · launcher `kiss_runs/bfc151_launch.sh` · βραχίονας `bfc150`

ATE / APE m, μ. ό. ± σ [ανά σπόρο]· KISS ένα run:

| ακολουθία | KISS | προεπιλογή | ανάμειξη (Β) | ζεύγος (Α) | **Γ** |
|---|---|---|---|---|---|
| dynamic_spinning | 0.159 | 0.171 ± 0.091 | 0.111 ± 0.017 | 1.415 ± 1.502 [0.12 2.74 2.69 0.11] | **0.111 ± 0.017** (= Β) |
| spms_01 | 8.743 | 8.646 ± 1.219 | 8.047 ± 0.618 | 10.654 ± 2.351 | **6.668 ± 1.144** [6.21 5.50 8.20 6.75] |
| spms_02 | 15.391 | 48.257 ± 3.622 | 48.137 ± 4.007 | **7.297 ± 0.939** | 8.329 ± 0.745 |
| spms_03 | 9.518 | 26.265 ± 5.519 | 17.642 ± 11.716 | **0.410 ± 0.133** | 0.818 ± 0.353 |
| eee_01 | 2.678 | 1.483 ± 0.149 | 1.553 ± 0.127 | **1.454 ± 0.055** | 1.601 ± 0.036 |
| rtp_01 | 3.985 | 0.264 ± 0.021 | 0.240 ± 0.058 | **0.220 ± 0.020** | 0.249 ± 0.056 |
| Boreas | 0.266 | 0.233 ± 0.008 | 0.195 ± 0.009 | 0.196 ± 0.007 | 0.195 ± 0.009 (= Β) |

RPE 1 m (cm / °): dynamic_spinning προεπιλογή 14.58 / 3.186, Β 13.17 / 3.130, Α 15.93 / 3.460, **Γ 13.17 / 3.130**· Boreas Β 3.71 / 0.158, Α 3.69 / 0.158, Γ 3.71 / 0.158.
**Συμπέρασμα ⏳:** η Γ είναι ακριβώς η Β όπου οι αποτυχίες είναι μεμονωμένες (dynamic_spinning, Boreas) και παίρνει το κέρδος του spms (μισό ή περισσότερο από το Α στα 02 / 03, καλύτερο στο 01).
Κόστος έναντι του Α: spms_02 / 03 λίγο χειρότερα (οι πρώτες 3 σαρώσεις κάθε σειράς χωρίς κίνηση), eee_01 +10 % (σειρές αποτυχιών ≤ 10 — μέρος τους κάτω από το 4). Καμία απόκλιση σε κανέναν
σπόρο. Υποψήφια προεπιλογή μαζί με Α / Β (απόφαση Μ.Τ.).

---

### 2026-10-05 — #154 Διόρθωση του #130: η «κλίμακα του ICP ~0.8» ήταν τεχνούργημα του μέτρου — το GT της σάρωσης είναι σωστό, τα βήματα του ICP είναι μακρύτερα

**Σχετικά:** #130 (διορθώνεται: «η κλίμακα του ICP ~0.8 σε 01_short / dynamic_spinning / math_medium / stairs — έλεγχος του GT»), #039 (μήκος διαδρομής +5 %) · ανοιχτό ερώτημα 4 (Μ.Τ. 5/10) · offline

base092 σπόρος 0. GT = `official_eval/<seq>/gt_lidar.txt` (10 Hz, κανένα κενό > 0.11 s), στις στιγμές του run· διάμεσοι:

| ακολουθία | |t| GT σάρωσης / απόσταση GT στη σάρωση | βήμα ICP / βήμα GT (μήκος) | βήμα ICP / GT σάρωσης | διαδρομή ICP / GT |
|---|---|---|---|---|
| 01_short | 1.000 | 1.091 | 1.143 | 1.146 |
| dynamic_spinning | 1.000 | 1.163 | 1.164 | 1.478 |
| math_medium | 1.000 | 1.029 | 1.019 | 1.036 |
| stairs | 1.000 | 1.416 | 1.418 | 1.610 |
| quad_easy | 1.000 | 1.053 | 1.054 | 1.076 |
| cloister | 1.000 | 1.039 | 1.040 | 1.049 |

**Συμπέρασμα ⏳:** το GT της σάρωσης (`make_oracle_motion.py`) συμφωνεί ακριβώς με το GT της τροχιάς. Η «κλίμακα» του #130 ήταν η προβολή t · t_GT / |t_GT|², που μικραίνει όταν η κατεύθυνση έχει
θόρυβο: τα βήματα του ICP τρεμοπαίζουν σε κατεύθυνση (#128: cos ~0.96) και είναι στην πραγματικότητα 3–42 % μακρύτερα — ο ίδιος θόρυβος φουσκώνει το μήκος διαδρομής (έως 1.6× στο stairs, το
+5 % του #039). Για την εικόνα (cos 0.996) η προβολή ≈ το μήκος: η κοντή μετατόπισή της (#130) μένει πραγματική.

---

### 2026-10-05 — #150 Επιλογή Γ: εφεδρεία «kiss» μόνο σε σειρά αποτυχιών (από την 4η συνεχόμενη) — κρατά το spms, αφαιρεί τη βλάβη στη γρήγορη περιστροφή (ένα run)

**Σχετικά:** #149 (επιλογές Α / Β / Γ, open_tasks), #146 · νέα επιλογή `image_deskew.fallback_kiss_after` (`--fallback-after=N`, 1 = #146) · launcher `kiss_runs/bfc150_launch.sh` · βραχίονας `bfc150`
(ανάμειξη για deskew + `--fallback=kiss --fallback-after=4`)

**Το N από τα δεδομένα** (σειρές αποτυχιών της εικόνας intensity, base092 σπόρος 0): dynamic_spinning 36 αποτυχημένες σαρώσεις σε 23 σειρές, **μέγιστη 3**· spms_01 2004 / 416, μέγιστη 123, 72 % σε σειρές ≥ 5·
spms_02 3066 / 101, μέγιστη 799, 97 %· spms_03 1874 / 260, μέγιστη 376, 81 %· eee_01 247 / 146, μέγιστη 10, 10 %· rtp_01 1277 / 392, μέγιστη 23, 55 %· Office_Mitte_1 καμία. N = 4: ποτέ στο dynamic_spinning,
σχεδόν όλες οι αποτυχίες του spms.
**Ένα run, σπόρος 0:** dynamic_spinning APE **0.102**, RPE 12.74 / 3.169 — ταυτόσημο με την ανάμειξη σπόρο 0 (ζεύγος #149: 0.115 / **2.744 / 2.687** / 0.112)· spms_02 ATE **7.218** (ζεύγος 7.30 ± 0.94,
KISS 15.39)· spms_03 **0.992** (ζεύγος 0.41 ± 0.13, εφεδρεία μόνη 0.90 ± 0.95 με σπόρους 0.33–2.31, KISS 9.52).
**Συμπέρασμα ⏳:** η εφεδρεία σε σειρά αποτυχιών κρατά το κέρδος του spms και δεν ενεργοποιείται στη γρήγορη περιστροφή. Επόμενο: υπο-δοκιμή.

---

### 2026-10-05 — #149 Το ζεύγος «ανάμειξη για deskew + εφεδρεία kiss» σε όλες τις 42 ακολουθίες (+ Boreas) × 4

**Σχετικά:** #138, #140, #147, #148 · launcher `kiss_runs/bf149_launch.sh` (136 νέα + 36, 172 / 172) · πίνακες `kiss_runs/results_official_allarms_149.{md,csv}` (27 ακολουθίες· ετικέτα `bf148`)
και πρωτόκολλο NTU για τις 15 νέες · Boreas από το #148

APE / ATE m (μ. ό. ± σ, 4 σπόροι· KISS ένα run), RTE %, RPE 1 m cm / ° — προεπιλογή, ανάμειξη μόνη, ζεύγος:

| sequence | KISS | default | blend | PAIR | % pair | RTE def/pair | RPE 1 m def → pair |
|---|---|---|---|---|---|---|---|
| 01_short | 0.348 | 0.301 ± 0.012 | 0.304 | 0.304 ± 0.016 | +1 | 0.72 / 0.73 | 10.70 / 1.839 → 10.25 / 1.838 |
| quad_easy | 0.108 | 0.077 ± 0.000 | 0.077 | 0.077 ± 0.000 | +0 | 0.26 / 0.26 | 5.49 / 0.591 → 5.40 / 0.574 |
| stairs | 0.458 | 1.923 ± 0.415 | 1.643 | 1.643 ± 0.289 | -15 | – | 37.26 / 7.040 → 36.63 / 7.455 |
| cloister | 0.181 | 0.180 ± 0.016 | 0.180 | 0.180 ± 0.025 | +0 | 0.30 / 0.30 | 6.24 / 0.711 → 6.19 / 0.690 |
| math_easy | 0.173 | 0.103 ± 0.001 | 0.103 | 0.103 ± 0.002 | +0 | 0.51 / 0.51 | 7.84 / 1.256 → 7.65 / 1.269 |
| underground_easy | 0.067 | 0.066 ± 0.002 | 0.065 | 0.065 ± 0.002 | -0 | 0.28 / 0.28 | 4.61 / 0.628 → 4.59 / 0.625 |
| christ-church-02 | 3.022 | 0.160 ± 0.029 | 0.227 | 0.227 ± 0.054 | +42 | 0.28 / 0.31 | 5.26 / 0.706 → 5.32 / 0.706 |
| christ-church-03 | 0.122 | 0.039 ± 0.000 | 0.039 | 0.039 ± 0.001 | +1 | 0.12 / 0.13 | 3.78 / 0.700 → 3.93 / 0.705 |
| quad_hard | 0.163 | 0.233 ± 0.021 | 0.234 | 0.234 ± 0.029 | +0 | 0.86 / 0.91 | 8.31 / 1.042 → 8.58 / 1.084 |
| math_medium | 0.888 | 0.153 ± 0.003 | 0.157 | 0.157 ± 0.004 | +2 | 0.94 / 0.97 | 12.93 / 2.453 → 12.64 / 2.458 |
| underground_medium | 0.096 | 0.057 ± 0.003 | 0.059 | 0.059 ± 0.002 | +3 | 0.22 / 0.22 | 4.73 / 0.793 → 4.77 / 0.786 |
| underground_hard | 3.846 | 0.091 ± 0.001 | 0.091 | 0.091 ± 0.003 | -0 | 0.25 / 0.25 | 5.90 / 1.088 → 5.93 / 1.116 |
| keble-college-03 | 0.854 | 0.089 ± 0.003 | 0.090 | 0.090 ± 0.001 | +1 | 0.68 / 0.66 | 8.19 / 1.040 → 8.09 / 1.031 |
| observatory-quarter-01 | 0.172 | 0.072 ± 0.008 | 0.067 | 0.067 ± 0.007 | -7 | 0.27 / 0.26 | 5.58 / 0.730 → 5.58 / 0.731 |
| blenheim-palace-02 | 0.540 | 0.214 ± 0.009 | 0.197 | 0.198 ± 0.006 | -8 | 0.54 / 0.50 | 7.77 / 0.923 → 7.54 / 0.878 |
| bodleian-library-02 | 2.547 | 0.649 ± 0.063 | 0.564 | 0.564 ± 0.100 | -13 | 1.04 / 0.98 | 9.27 / 1.115 → 9.39 / 1.103 |
| 02_long_experiment | 1.275 | 2.190 ± 0.187 | 1.901 | 1.901 ± 0.720 | -13 | 1.07 / 1.03 | 11.43 / 2.871 → 11.36 / 2.890 |
| dynamic_spinning | 0.159 | 0.171 ± 0.091 | 0.111 | 1.415 ± 1.502 | +728 | – | 14.58 / 3.186 → 15.93 / 3.460 |
| Basement_1 | 0.055 | 0.053 ± 0.005 | 0.046 | 0.046 ± 0.010 | -14 | – | – |
| IC_Office_1 | 6.344 | 0.073 ± 0.003 | 0.074 | 0.074 ± 0.006 | +1 | – | – |
| Office_Mitte_1 | 4.286 | 0.185 ± 0.054 | 0.248 | 0.248 ± 0.063 | +34 | – | – |
| Construction_Site_1 | 0.063 | 0.042 ± 0.003 | 0.037 | 0.042 ± 0.008 | +1 | – | – |
| LAB_Survey_2 | 0.062 | 0.035 ± 0.000 | 0.035 | 0.035 ± 0.000 | +0 | – | – |
| UZH_Tracking_Area_Run_2 | 0.585 | 0.576 ± 0.001 | 0.576 | 0.576 ± 0.002 | -0 | – | – |
| eee_01 | 2.678 | 1.483 ± 0.149 | 1.553 | 1.454 ± 0.054 | -2 | – | – |
| eee_02 | 1.486 | 0.836 ± 0.038 | 0.790 | 0.797 ± 0.082 | -5 | – | – |
| eee_03 | 0.864 | 0.360 ± 0.073 | 0.142 | 0.140 ± 0.002 | -61 | – | – |
| nya_01 | 0.736 | 0.355 ± 0.005 | 0.359 | 0.360 ± 0.009 | +1 | – | – |
| nya_02 | 1.508 | 0.191 ± 0.021 | 0.200 | 0.200 ± 0.020 | +5 | – | – |
| nya_03 | 1.052 | 0.519 ± 0.005 | 0.536 | 0.535 ± 0.010 | +3 | – | – |
| sbs_01 | 0.976 | 0.354 ± 0.022 | 0.367 | 0.390 ± 0.062 | +10 | – | – |
| sbs_02 | 1.143 | 0.759 ± 0.187 | 0.767 | 0.603 ± 0.225 | -21 | – | – |
| sbs_03 | 1.214 | 0.793 ± 0.071 | 0.815 | 0.850 ± 0.050 | +7 | – | – |
| rtp_01 | 3.985 | 0.264 ± 0.021 | 0.240 | 0.220 ± 0.020 | -17 | – | – |
| rtp_02 | 3.575 | 2.605 ± 0.174 | 0.335 | 0.327 ± 0.008 | -87 | – | – |
| rtp_03 | 3.408 | 0.232 ± 0.005 | 0.215 | 0.227 ± 0.002 | -2 | – | – |
| tnp_01 | 2.132 | 2.151 ± 0.060 | 2.168 | 2.153 ± 0.139 | +0 | – | – |
| tnp_02 | 2.786 | 2.820 ± 0.110 | 2.762 | 2.791 ± 0.107 | -1 | – | – |
| tnp_03 | 2.613 | 1.086 ± 0.283 | 1.118 | 1.228 ± 0.250 | +13 | – | – |
| spms_01 | 8.743 | 8.646 ± 1.219 | 8.047 | 10.654 ± 2.351 | +23 | – | – |
| spms_02 | 15.391 | 48.257 ± 3.622 | 48.137 | 7.297 ± 0.939 | -85 | – | – |
| spms_03 | 9.518 | 26.265 ± 5.519 | 17.642 | 0.410 ± 0.133 | -98 | – | – |
| Boreas | 0.266 | 0.233 ± 0.008 | 0.195 | 0.196 ± 0.007 | -16 | 0.35 / 0.27 | 4.96 / 0.210 → 3.69 / 0.158 |

Σύνοψη (41 χωρίς τις ειδικές): ζεύγος έναντι προεπιλογής πέρα από 1σ καλύτερο / χειρότερο / μέσα **7 / 3 / 31**, διάμεσος μεταβολής +0.2 %· **καλύτερο από τον KISS σε 36 / 41** (KISS καλύτερος:
quad_hard, 02_long, spms_01, tnp_01 / tnp_02 ≈ ίσα).
**dynamic_spinning ανά σπόρο** (APE m · RPE 1 m cm / °): προεπιλογή 0.305 / 0.129 / 0.102 / 0.147· ανάμειξη 0.102 / 0.103 / 0.104 / 0.136· **ζεύγος 0.115 / 2.744 / 2.687 / 0.112** (RPE 13.8–18.6 / 3.44–3.49).
**Συμπέρασμα ⏳:** (1) Το ζεύγος κρατά τα κέρδη και των δύο μερών (όχημα −16 %, drone eee_03 / rtp_02 −61 / −87 %, spms −85 / −98 %) και είναι ουδέτερο στη φορητή μονάδα (31 / 41 μέσα στη
διασπορά). (2) Απώλειες: christ-church-02 (η αστάθεια ύψους, #140), Office_Mitte_1 (από την ανάμειξη, #140), spms_01 (+23 %, από την εφεδρεία, #147). (3) **Νέο πρόβλημα στην ειδική
dynamic_spinning:** με την εφεδρεία, 2 / 4 σπόροι αποκλίνουν (2.7 m)· εκεί η εικόνα αποτυγχάνει σε σαρώσεις γρήγορης περιστροφής, όπου η σταθερή ταχύτητα είναι λάθος αρχή και το «χωρίς
deskew + μηδενική κίνηση» ήταν ασφαλέστερο. Πριν από προεπιλογή: ΑΠΟΦΑΣΗ Μ.Τ. — και πιθανός περιορισμός της εφεδρείας (π.χ. μόνο όταν η εικόνα λείπει σε συνεχόμενες σαρώσεις).

---

### 2026-10-04 — #148 Υπο-δοκιμή του ζεύγους «ανάμειξη για deskew + εφεδρεία kiss» (9 × 4): τα δύο κέρδη χωρίς αλληλεπίδραση

**Σχετικά:** #138, #147 · launcher `kiss_runs/bf148_launch.sh` · βραχίονας `bf148` (`--cv-blend=adaptive --cv-blend-use=deskew --fallback=kiss`)

Μ. ό. ± σ, 4 σπόροι. APE m · RTE % · RPE 1 m cm / ° (NTU: ATE):

| ακολουθία | προεπιλογή | ανάμειξη | εφεδρεία kiss | **ζεύγος** |
|---|---|---|---|---|
| quad_easy | 0.077 · 0.263 · 5.48 / 0.592 | 0.077 · 0.260 · 5.40 / 0.574 | – | 0.077 · 0.260 · 5.40 / 0.574 (= ανάμειξη) |
| cloister | 0.180 · 0.297 · 6.24 / 0.711 | 0.180 · 0.295 · 6.19 / 0.690 | – | = ανάμειξη |
| underground_hard | 0.091 · 0.247 · 5.90 / 1.088 | 0.091 · 0.247 · 5.93 / 1.116 | – | = ανάμειξη |
| christ-church-03 | 0.039 · 0.125 · 3.79 / 0.700 | 0.039 · 0.133 · 3.93 / 0.705 | – | = ανάμειξη |
| keble-03 | 0.089 · 0.677 · 8.19 / 1.040 | 0.090 · 0.663 · 8.09 / 1.031 | – | = ανάμειξη |
| eee_01 | 1.483 ± 0.149 | 1.553 ± 0.127 | 1.348 ± 0.209 | 1.454 ± 0.055 (−2 %) |
| spms_02 | 48.257 ± 3.622 | 48.137 ± 4.007 | 6.596 ± 0.813 | **7.297 ± 0.939 (−85 %)** |
| spms_03 | 26.265 ± 5.519 | 17.642 ± 11.716 | 0.902 ± 0.945 | **0.410 ± 0.133 (−98 %)** |
| Boreas | 0.233 · 0.350 · 4.96 / 0.210 | 0.195 · 0.270 · 3.71 / 0.158 | – | **0.196 · 0.270 · 3.69 / 0.158** |

**Συμπέρασμα ⏳:** τα δύο μέρη δρουν σε ξένες μεταξύ τους σαρώσεις: όπου η εικόνα δεν αποτυγχάνει το ζεύγος είναι ταυτόσημο με την ανάμειξη, όπου αποτυγχάνει παίρνει το κέρδος της
εφεδρείας (spms_03 και πιο σταθερό, ± 0.13 έναντι ± 0.95). Το όχημα κρατά το −16…−26 %. Υποψήφιο για νέα προεπιλογή → όλες οι 42 × 4.

---

### 2026-10-04 — #147 Υπο-δοκιμή της εφεδρείας «kiss» (7 × 4): spms_02 −86 %, spms_03 −97 %, spms_01 +17 %, αλλού ίδιο ή λίγο καλύτερο

**Σχετικά:** #146 · launcher `kiss_runs/fbk147_launch.sh` (26 + 2 υπάρχοντα) · βραχίονας `fbk146` (`--fallback=kiss`)

ATE (NTU) / APE (Hilti) m, μ. ό. ± σ (4 σπόροι) [ανά σπόρο]· KISS ένα run:

| ακολουθία | εφεδρείες απόστασης (σ. 0) | KISS | προεπιλογή | ανάμειξη deskew | εφεδρεία kiss |
|---|---|---|---|---|---|
| spms_01 | 747 | 8.743 | 8.646 ± 1.219 | 8.047 ± 0.618 | **10.152 ± 0.877 (+17 %)** [10.34 9.23 9.75 11.28] |
| spms_02 | 129 | 15.391 | 48.257 ± 3.622 | 48.137 ± 4.007 | **6.596 ± 0.813 (−86 %)** [6.14 6.85 7.62 5.78] |
| spms_03 | 541 | 9.518 | 26.265 ± 5.519 | 17.642 ± 11.716 | **0.902 ± 0.945 (−97 %)** [2.31 0.56 0.33 0.41] |
| rtp_01 | 1038 | 3.985 | 0.264 ± 0.021 | 0.240 ± 0.058 | 0.263 ± 0.021 (−1 %) |
| tnp_01 | 3 | 2.132 | 2.151 ± 0.060 | 2.168 ± 0.090 | 2.081 ± 0.060 (−3 %) |
| eee_01 | 293 | 2.678 | 1.483 ± 0.149 | 1.553 ± 0.127 | 1.348 ± 0.209 (−9 %) |
| Office_Mitte_1 | – | 4.286 | 0.185 ± 0.054 | 0.248 ± 0.063 | 0.185 ± 0.054 (0 %, ταυτόσημο) |

**Συμπέρασμα ⏳:** όπου η εικόνα αποτυγχάνει σε μεγάλα τμήματα (spms_02 / 03, ύψος) η εφεδρεία μετατρέπει την αποτυχία σε κέρδος έναντι του KISS (6.6 έναντι 15.4, 0.9 έναντι 9.5 m)· όπου
οι αποτυχίες είναι σποραδικές (rtp_01, tnp_01, eee_01) ίδιο ή λίγο καλύτερο· χωρίς αποτυχίες ταυτόσημο. Μία απώλεια: spms_01 +17 % (~1.3σ), όπου και η προεπιλογή ≈ KISS. Υποψήφια για
προεπιλογή μαζί με την ανάμειξη για deskew (ανεξάρτητες: η μία δρα στις σαρώσεις χωρίς εικόνα, η άλλη στις σαρώσεις με εικόνα).

---

### 2026-10-04 — #144 Ο συνδυασμός (ανάμειξη για deskew + στροφή από τομείς) σε όλες τις 27 × 4: όχι σαφώς καλύτερος από την ανάμειξη μόνη

**Σχετικά:** #140, #142 · launcher `kiss_runs/cmb144_launch.sh` (84 νέα + 28 υπάρχοντα, 112 / 112) · πίνακας `kiss_runs/results_official_allarms_144.{md,csv}` (ετικέτες `cmb141`, `fbk146`)

APE m (μ. ό., σ για προεπιλογή / συνδυασμό), % έναντι προεπιλογής· RTE % προεπιλογή / ανάμειξη / συνδυασμός· RPE 1 m cm / ° ίδια σειρά:

| ακολουθία | KISS | προεπιλογή | ανάμειξη | συνδυασμός | RTE | RPE 1 m |
|---|---|---|---|---|---|---|
| 01_short | 0.348 | 0.301 ± 0.012 | 0.304 (+1) | 0.299 ± 0.004 (−1) | 0.72 / 0.73 / 0.73 | 10.70 / 1.839 · 10.25 / 1.838 · 10.12 / 1.839 |
| quad_easy | 0.108 | 0.077 ± 0.000 | 0.077 (0) | **0.075 (−2)** | 0.26 / 0.26 / 0.25 | 5.49 / 0.591 · 5.40 / 0.574 · **5.31 / 0.533** |
| cloister | 0.181 | 0.180 ± 0.016 | 0.180 (0) | 0.182 ± 0.008 (+1) | 0.30 / 0.30 / 0.28 | 6.24 / 0.711 · 6.19 / 0.690 · 6.23 / 0.668 |
| math_easy | 0.173 | 0.103 ± 0.001 | 0.103 (0) | 0.105 (+2) | 0.51 / 0.51 / 0.54 | 7.84 / 1.256 · 7.65 / 1.269 · 7.56 / 1.277 |
| underground_easy | 0.067 | 0.066 ± 0.002 | 0.065 | 0.065 | 0.28 / 0.28 / 0.27 | 4.61 / 0.628 · 4.59 / 0.625 · 4.62 / 0.614 |
| quad_hard | 0.163 | 0.233 ± 0.021 | 0.234 (0) | **0.200 ± 0.007 (−14)** | 0.86 / 0.91 / **0.77** | 8.31 / 1.042 · 8.55 / 1.071 · 8.60 / 1.044 |
| math_medium | 0.888 | 0.153 ± 0.003 | 0.157 (+2) | 0.157 (+3) | 0.94 / 0.97 / 0.91 | 12.93 / 2.453 · 12.64 / 2.458 · 12.90 / 2.453 |
| underground_medium | 0.096 | 0.057 ± 0.003 | 0.059 (+3) | 0.056 (−2) | 0.22 / 0.22 / 0.22 | 4.73 / 0.793 · 4.77 / 0.786 · 4.76 / 0.778 |
| underground_hard | 3.846 | 0.091 ± 0.001 | 0.091 | 0.089 (−2) | 0.25 / 0.25 / 0.25 | 5.90 / 1.088 · 5.93 / 1.116 · 6.04 / 1.127 |
| christ-church-02 | 3.022 | 0.160 ± 0.029 | 0.227 (+42) | 0.200 ± 0.029 (+25) | 0.28 / 0.31 / 0.30 | 5.26 / 0.706 · 5.32 / 0.706 · 5.62 / 0.732 |
| christ-church-03 | 0.122 | 0.039 ± 0.000 | 0.039 (+1) | 0.040 (+2) | 0.12 / 0.13 / 0.13 | 3.78 / 0.700 · 3.93 / 0.705 · 3.97 / 0.725 |
| keble-03 | 0.854 | 0.089 ± 0.003 | 0.090 (+1) | 0.090 (+1) | 0.68 / 0.66 / **0.62** | 8.19 / 1.040 · 8.09 / 1.031 · 8.00 / 1.012 |
| observatory-01 | 0.172 | 0.072 ± 0.008 | 0.067 (−7) | 0.076 ± 0.012 (+6) | 0.27 / 0.26 / 0.26 | 5.58 / 0.730 · 5.58 / 0.731 · 5.61 / 0.731 |
| blenheim-02 | 0.540 | 0.214 ± 0.009 | 0.197 (−8) | **0.196 (−9)** | 0.54 / 0.49 / 0.49 | 7.77 / 0.923 · 7.53 / 0.876 · **7.40 / 0.850** |
| bodleian-02 | 2.547 | 0.649 ± 0.063 | 0.564 (−13) | 0.642 ± 0.039 (−1) | 1.04 / 0.97 / 1.02 | 9.27 / 1.115 · 9.39 / 1.102 · 9.63 / 1.121 |
| 02_long | 1.275 | 2.190 ± 0.187 | 1.901 (−13) | 2.048 ± 0.396 (−6) | 1.07 / 1.03 / 1.07 | 11.43 / 2.871 · 11.36 / 2.890 · 11.37 / 2.888 |
| Basement_1 | 0.055 | 0.053 ± 0.005 | 0.046 (−14) | **0.036 ± 0.006 (−32)** | – | – |
| IC_Office_1 | 6.344 | 0.073 ± 0.003 | 0.074 (+1) | 0.078 (+6) | – | – |
| Office_Mitte_1 | 4.286 | 0.185 ± 0.054 | 0.248 (+34) | 0.206 ± 0.040 (+12) | – | – |
| Construction_Site_1 | 0.063 | 0.042 ± 0.003 | 0.037 (−12) | 0.043 (+2) | – | – |
| LAB_Survey_2 | 0.062 | 0.035 ± 0.000 | 0.035 | 0.036 (+2) | – | – |
| UZH_Tracking_Area_Run_2 | 0.585 | 0.576 ± 0.001 | 0.576 | 0.571 (−1) | – | – |
| eee_01 | 2.678 | 1.483 ± 0.149 | 1.553 (+5) | 1.607 ± 0.154 (+8) | – | – |
| eee_02 | 1.486 | 0.836 ± 0.038 | 0.790 (−6) | 0.811 ± 0.062 (−3) | – | – |
| eee_03 | 0.864 | 0.360 ± 0.073 | **0.142 (−60)** | 0.272 ± 0.135 (−24) | – | – |
| *stairs (ειδική)* | 0.458 | 1.923 ± 0.415 | 1.643 (−15) | 2.257 ± 0.666 (+17) | – | 37.26 / 7.040 · 36.63 / 7.455 · 36.60 / 7.269 |
| *dynamic_spinning (ειδική)* | 0.159 | 0.171 ± 0.091 | 0.111 (−35) | 0.146 ± 0.059 (−15) | – | 14.58 / 3.186 · 13.17 / 3.130 · 13.83 / 3.214 |

Σύνοψη (25 χωρίς τις ειδικές): APE πέρα από 1σ καλύτερο / χειρότερο / μέσα — ανάμειξη 3 / 2 / 20, συνδυασμός 4 / 3 / 18· RPE 1 m διάμεσος — ανάμειξη −0.5 % / 0 %, συνδυασμός 0 % / +0.1 %.
Boreas (#142): ανάμειξη 0.195 · 0.270 · 3.71 / 0.158, συνδυασμός 0.205 · 0.265 · 3.58 / 0.153.
**Συμπέρασμα ⏳:** ο συνδυασμός δεν ξεπερνά συνολικά την ανάμειξη μόνη· τα κέρδη στροφής των τομέων (quad_easy, cloister, keble, blenheim) και τα κέρδη quad_hard / Basement_1
αντισταθμίζονται από τις μικρές απώλειες των τομέων (christ-church-02 / -03 RPE, math_easy RTE) και από το eee_03 (η «καλή» κατάσταση 0.14 m σε λιγότερους σπόρους). Η ανάμειξη μόνη μένει
η απλούστερη και ασφαλέστερη υποψήφια· οι τομείς ως επιλογή.

---

### 2026-10-04 — #146 Εφεδρεία «kiss» μόνο για τις σαρώσεις όπου αποτυγχάνει η εικόνα: διορθώνει το spms χωρίς την κατάρρευση του underground_hard

**Σχετικά:** #143, #145, #108 / #119 · νέα τιμή `image_deskew.fallback: kiss` (`--fallback=kiss`): η σάρωση χωρίς κίνηση εικόνας → deskew και αρχή του ICP από τη σταθερή ταχύτητα· οι δύο
αρχές αμετάβλητες (σε αντίθεση με το `two_start_kiss`) · launcher `kiss_runs/fbk146_launch.sh` · βραχίονας `fbk146`

Ένα run, σπόρος 0. ATE m (NTU) / APE m · RTE % · RPE 1 m cm / ° (NCD):
spms_02 — KISS 15.391· προεπιλογή 46.19 (σ. 0)· `--two-start-kiss` 8.579· **`--fallback=kiss` 6.138**.
spms_03 — KISS 9.518· προεπιλογή 30.06 (σ. 0, 18.9–30.9 σε 4)· `--two-start-kiss` 0.415· **`--fallback=kiss` 2.312**.
underground_hard — προεπιλογή 0.090–0.093 · 0.24–0.25 · 5.77–6.03 / 1.075–1.109· `--two-start-kiss` (#108) 4.623 / 4.584 / 0.087 / 3.342 · RTE 3.58 / 3.61 / 0.26 / 2.66· **`--fallback=kiss` 0.093 · 0.25 ·
5.89 / 1.075 — ακριβώς ο σπόρος 0 της προεπιλογής** (καμία αποτυχία της εικόνας εκεί, 0 εφεδρείες απόστασης).
**Συμπέρασμα ⏳:** η εφεδρεία ενεργεί μόνο όπου αποτυγχάνει η εικόνα· αλλού η μέθοδος μένει ίδια bit προς bit. Στο spms: καλύτερη από τον KISS και στις δύο (6.1 έναντι 15.4, 2.3 έναντι 9.5 m)·
το spms_03 έχει δύο καταστάσεις (0.4 / ~2–30 m), ένα run δεν αρκεί. Επόμενο: υπο-δοκιμή στις ακολουθίες με αποτυχίες της εικόνας.

---

### 2026-10-04 — #145 Εφεδρεία = KISS στο spms: 46–52 → 8.6 m και 19–31 → 0.4 m (ένα run) · διόρθωση του #143 (κενά στο GT)

**Σχετικά:** #143 (διορθώνεται), #108 (`--two-start-kiss`) · launcher `kiss_runs/kfl143_launch.sh` · βραχίονας `kfl143` (προεπιλογή + `--two-start-kiss`: όπου αποτυγχάνει η εικόνα, deskew
και αρχή του ICP από τη σταθερή ταχύτητα, όπως ο KISS· η δεύτερη αρχή επίσης ο KISS)

**ATE (m, πρωτόκολλο NTU):** spms_02 — KISS 15.391· προεπιλογή 46.19 / 44.36 / 50.22 / 52.26· ανάμειξη deskew 43.35 / 52.97 / 49.09 / 47.15· **εφεδρεία KISS 8.579**.
spms_03 — KISS 9.518· προεπιλογή 30.06 / 30.86 / 18.85 / 25.29· ανάμειξη 20.46 / **0.395** / 25.92 / 23.79· **εφεδρεία KISS 0.415**. Πληρότητα 100 %.
**Διόρθωση του #143:** το GT των spms έχει κενά (Leica χάνει το πρίσμα σε ύψος): spms_02 15 κενά > 0.5 s, σύνολο 89 / 395 s, μέγιστο 40 s· spms_03 9 κενά, **177 / 386 s, μέγιστο 105 s**·
nya_02 κανένα. Η δική μου ευθυγράμμιση στο #143 παρεμβάλλει το GT πάνω από τα κενά: τα «μήκη διαδρομής GT» (698 / 312 m), οι λόγοι μήκους και τα σφάλματα ανά λεπτό εκεί είναι
λάθος. Ισχύουν τα επίσημα ATE (που συνδέουν μόνο εκτιμήσεις σε 0.05 s από δείγμα GT) και η διάγνωση ανά ύψος (αποτυχίες της εικόνας 84–97 % πάνω από 15 m, εφεδρεία «καμία κίνηση»).
**Συμπέρασμα ⏳:** η εφεδρεία = KISS αφαιρεί την αποτυχία της μεθόδου όπου η εικόνα δεν δουλεύει, και στο spms η μέθοδος γίνεται καλύτερη από τον KISS (8.6 έναντι 15.4, 0.4 έναντι 9.5 m)· το
spms_03 της ανάμειξης (σπόρος 1, 0.395) δείχνει ότι εκεί υπάρχουν δύο καταστάσεις. Στις 27 το #119 δεν βρήκε τοπικό κέρδος (και κατάρρευση στο underground_hard) — άρα όχι ως
προεπιλογή χωρίς υπο-δοκιμή στις 15 νέες NTU και στις 27.

---

### 2026-10-04 — #142 Υπο-δοκιμή του συνδυασμού (ανάμειξη για deskew + στροφή από τομείς), 7 × 4

**Σχετικά:** #138, #139, #141 · launcher `kiss_runs/cmb142_launch.sh` · βραχίονας `cmb141`

Μ. ό. (4 σπόροι), % έναντι προεπιλογής. APE m · RTE % · RPE 1 m cm / ° (NTU: ATE).

| ακολουθία | προεπιλογή | ανάμειξη deskew | στροφή τομείς | **συνδυασμός** |
|---|---|---|---|---|
| quad_easy | 0.077 · 0.263 · 5.48 / 0.592 ± 0.002 | 0.077 · 0.260 · 5.40 / 0.574 (−3 %) | 0.076 · 0.258 · 5.35 / 0.548 (−7 %) | **0.075 · 0.250 · 5.31 / 0.533 (−10 %)** |
| cloister | 0.180 · 0.297 · 6.24 / 0.711 ± 0.008 | 0.180 · 0.295 · 6.19 / 0.690 | 0.169 · 0.298 · 6.20 / 0.678 | 0.182 · **0.285** · 6.24 / **0.668 (−6 %)** |
| underground_hard | 0.091 · 0.247 · 5.90 / 1.088 ± 0.016 | 0.091 · 0.247 · 5.93 / 1.116 | 0.089 · 0.253 · 6.06 / 1.122 | 0.089 · 0.247 · 6.04 / 1.127 (+4 %) |
| christ-church-03 | 0.039 · 0.125 · 3.79 / 0.700 ± 0.003 | 0.039 · 0.133 · 3.93 / 0.705 | 0.043 · 0.125 · 3.72 / 0.712 | 0.040 · 0.130 · 3.97 / 0.725 (+5 / +3 %) |
| keble-03 | 0.089 · 0.677 · 8.19 / 1.040 ± 0.004 | 0.090 · 0.663 · 8.09 / 1.031 | 0.090 · 0.637 · 8.17 / 1.022 | 0.090 · **0.615** · 8.00 / **1.012** |
| eee_01 | 1.483 ± 0.149 | 1.553 | 1.585 | 1.607 (+8 %) |
| Boreas | 0.233 · 0.350 · 4.96 / 0.210 ± 0.008 | 0.195 · 0.270 · 3.71 / 0.158 | 0.235 · 0.363 · 4.75 / 0.193 | **0.205 · 0.265 · 3.58 / 0.153** |

**Συμπέρασμα ⏳:** ο συνδυασμός δίνει τη μεγαλύτερη βελτίωση στροφής στη φορητή όπου και τα δύο μέρη βοηθούν (quad_easy −10 %, cloister −6 %, keble −3 % με RTE −9 %) και κρατά το
κέρδος του οχήματος (RPE −28 / −27 %, RTE −24 %)· όπου και τα δύο μέρη έβλαπταν λίγο (underground_hard, christ-church-03) αθροίζονται και οι μικρές απώλειες (+3…+5 %). eee_01 μέσα στη
διασπορά. Χειρότερο στοιχείο: christ-church-03 (RPE +5 %, στροφή +3 %).

---

### 2026-10-04 — #143 Διάγνωση: γιατί η μέθοδος είναι χειρότερη από τον KISS στο spms (NTU) — η εικόνα αποτυγχάνει σε ύψος και η εφεδρεία είναι «καμία κίνηση»

**Σχετικά:** #141, #108 / #119 (εφεδρεία = KISS) · μόνο υπάρχοντα runs (σπόρος 0)

**Τροχιά έναντι GT** (θέση, ευθυγράμμιση Umeyama, χωρίς το πρίσμα — ~ATE)· σφάλμα ανά λεπτό (m):
spms_02 (GT 395 s, 698 m, ύψος 0–33 m): KISS 16.1 (z 14.3), μήκος 931 m, 26 / 8 / 12 / 19 / 11 / 8 / 16· προεπιλογή 43.7 (z 18.4), 920 m, 75 / 34 / 39 / 24 / 24 / 49 / 58· ανάμειξη 40.2.
spms_03 (386 s, 312 m, 0–35 m): KISS 16.0, 713 m· προεπιλογή 31.1, 643 m, 19 / 15 / 23 / 16 / 35 / 55 / 60· ανάμειξη 27.1. nya_02 (0–5 m): KISS 1.51, προεπιλογή 0.26.
**Ανά ύψος (προεπιλογή, σπόρος 0):** spms_02 0–5 m: 455 σαρώσεις, 1501 σημεία / σάρωση, inliers 81, αποτυχίες εικόνας 7 %· 5–15 m: 135, 3146, 0, 65 %· **15–40 m: 3032, 1214, 0, 97 %**.
spms_03 0–5 m: 3 %· 5–15 m: 25 %· **15–40 m: 84 %**. nya_02 0–15 m: αποτυχίες 0.2–0.3 %. Εφεδρεία απόστασης: 129 (spms_02) / 541 (spms_03) σαρώσεις. Οι δύο αρχές σχεδόν ποτέ CV (< 1 %).
**Συμπέρασμα ⏳:** σε ύψος ο οριζόντιος OS1-16 βλέπει λίγα και μακρινά σημεία· η εικόνα intensity δεν δίνει αντιστοιχίσεις. Η μέθοδος τότε (`image_deskew.fallback: identity`, η
ρύθμιση από το #012) δεν κάνει deskew και ξεκινά τον ICP από μηδενική κίνηση — ενώ το drone κινείται — σε ~3000 σαρώσεις· ο KISS έχει τουλάχιστον τη σταθερή ταχύτητα. Και οι δύο
αποτυγχάνουν (μήκος 1.3–2.3× του GT), η μέθοδος περισσότερο. Διόρθωση προς δοκιμή: εφεδρεία = KISS (σταθερή ταχύτητα για deskew και αρχή, #108 `--two-start-kiss`), που στις
27 δεν βοήθησε τοπικά (#119) γιατί εκεί οι αποτυχίες είναι λίγες.

---

### 2026-10-04 — #141 Συνδυασμός «ανάμειξη για deskew + στροφή από τομείς» (ένα run) · τα 15 νέα NTU VIRAL στη δοκιμή (σε εξέλιξη)

**Σχετικά:** #138, #139, #140 · ζήτημα Μ.Τ. 4/10 («δοκίμασε τον συνδυασμό, και όλα στις νέες NTU») · launcher `kiss_runs/ntu141_launch.sh` · βραχίονας `cmb141`
(`--cv-blend=adaptive --cv-blend-use=deskew --sectors=8 --sectors-part=rotation`)· οι 15 νέες ακολουθίες στο `kiss_runs/all_seqs.tsv` (42 γραμμές· η λίστα των 27 στο `all_seqs_27.tsv`)

**Συνδυασμός, ένα run, έναντι 4 σπόρων ανά βραχίονα** (APE m · RTE % · RPE 1 m cm / ° · μήκος m):
quad_easy — προεπιλογή 0.077–0.078 · 0.26–0.27 · 5.46–5.51 / 0.589–0.593 · 265.6–266.1· ανάμειξη 0.077–0.078 · 0.26 · 5.34–5.45 / 0.570–0.576 · 264.4–264.9· τομείς 0.076–0.077 · 0.25–0.26 ·
5.34–5.38 / 0.546–0.551 · 264.8–265.4· **συνδυασμός 0.076 · 0.25 · 5.33 / 0.535 · 263.9**.
Boreas — προεπιλογή 0.225–0.240 · 0.33–0.37 · 4.76–5.19 / 0.199–0.216· ανάμειξη 0.182–0.202 · 0.26–0.28 · 3.61–3.88 / 0.155–0.162· τομείς 0.220–0.252 · 0.35–0.40 · 4.56–4.92 / 0.189–0.199·
**συνδυασμός 0.189 · 0.27 · 3.62 / 0.152**.
**Συμπέρασμα ⏳:** στη φορητή μονάδα τα δύο κέρδη στροφής αθροίζονται (0.535° έναντι 0.570–0.576 / 0.546–0.551 — έξω από τις περιοχές και των δύο)· στο όχημα ο συνδυασμός = η ανάμειξη
(στροφή 0.152, μόλις κάτω από την περιοχή της). Επόμενο: υπο-δοκιμή 7 × 4.

**15 νέες NTU VIRAL — ATE m (πρωτόκολλο NTU, μ. ό. ± σ, 4 σπόροι· KISS ένα run):**

| ακολουθία | KISS | προεπιλογή | ανάμειξη για deskew | στροφή από τομείς | σύντηξη απόστασης |
|---|---|---|---|---|---|
| nya_01 | 0.736 | 0.355 ± 0.005 | 0.359 ± 0.009 (+1 %) | 0.349 ± 0.008 (-2 %) | 0.367 ± 0.007 (+3 %) |
| nya_02 | 1.508 | 0.191 ± 0.021 | 0.200 ± 0.020 (+5 %) | 0.187 ± 0.025 (-2 %) | 0.160 ± 0.013 (-16 %) |
| nya_03 | 1.052 | 0.519 ± 0.005 | 0.536 ± 0.010 (+3 %) | 0.461 ± 0.036 (-11 %) | 0.528 ± 0.003 (+2 %) |
| sbs_01 | 0.976 | 0.354 ± 0.022 | 0.367 ± 0.034 (+4 %) | 0.340 ± 0.036 (-4 %) | 0.383 ± 0.026 (+8 %) |
| sbs_02 | 1.143 | 0.759 ± 0.187 | 0.767 ± 0.238 (+1 %) | 0.659 ± 0.160 (-13 %) | 0.746 ± 0.160 (-2 %) |
| sbs_03 | 1.214 | 0.793 ± 0.071 | 0.815 ± 0.040 (+3 %) | 0.837 ± 0.049 (+6 %) | 0.814 ± 0.050 (+3 %) |
| rtp_01 | 3.985 | 0.264 ± 0.021 | 0.240 ± 0.058 (-9 %) | 0.336 ± 0.082 (+27 %) | 0.306 ± 0.028 (+16 %) |
| rtp_02 | 3.575 | 2.605 ± 0.174 | 0.335 ± 0.009 (-87 %) | 2.359 ± 0.319 (-9 %) | 2.821 ± 0.229 (+8 %) |
| rtp_03 | 3.408 | 0.232 ± 0.005 | 0.215 ± 0.002 (-7 %) | 0.231 ± 0.002 (-0 %) | 0.239 ± 0.003 (+3 %) |
| tnp_01 | 2.132 | 2.151 ± 0.060 | 2.168 ± 0.090 (+1 %) | 2.081 ± 0.097 (-3 %) | 2.190 ± 0.088 (+2 %) |
| tnp_02 | 2.786 | 2.820 ± 0.110 | 2.762 ± 0.081 (-2 %) | 2.912 ± 0.042 (+3 %) | 2.963 ± 0.079 (+5 %) |
| tnp_03 | 2.613 | 1.086 ± 0.283 | 1.118 ± 0.342 (+3 %) | 1.029 ± 0.278 (-5 %) | 1.246 ± 0.438 (+15 %) |
| spms_01 | 8.743 | 8.646 ± 1.219 | 8.047 ± 0.618 (-7 %) | 8.502 ± 0.617 (-2 %) | 10.628 ± 2.374 (+23 %) |
| spms_02 | 15.391 | 48.257 ± 3.622 | 48.137 ± 4.007 (-0 %) | 50.901 ± 1.986 (+5 %) | 48.016 ± 3.520 (-0 %) |
| spms_03 | 9.518 | 26.265 ± 5.519 | 17.642 ± 11.716 (-33 %) | 14.973 ± 13.967 (-43 %) | 27.997 ± 5.887 (+7 %) |

Πληρότητα 99–100 % σε όλα (spms_01 99 % και για τον KISS). Διάμεσος μεταβολής / πέρα από 1σ (καλύτερο / χειρότερο / μέσα): ανάμειξη +1 % · 2 / 1 / 12· τομείς −2 % · 1 / 0 / 14·
σύντηξη +3 % · 1 / 6 / 8.
**Συμπέρασμα ⏳:** (1) Η μέθοδος έναντι KISS: πολύ καλύτερη σε nya / sbs / rtp (2–15×), ίση στο tnp, χειρότερη στο spms (spms_02 48 έναντι 15 m, spms_03 26 έναντι 9.5 m) — το spms είναι η
πρώτη ομάδα όπου η εικόνα βλάπτει συστηματικά· χρειάζεται διάγνωση. (2) Η ανάμειξη για deskew: ουδέτερη στις περισσότερες, σώζει το rtp_02 (2.61 → 0.34 m, σ 0.009 έναντι 0.174)·
στο drone ξανά το κέρδος έρχεται σε μία ακολουθία (όπως eee_03), αλλού ίδια. (3) Τομείς για στροφή: μικρό μέσο κέρδος, μία απώλεια (rtp_01 +27 %). (4) Σύντηξη απόστασης: όχι.

---

### 2026-10-04 — #140 Ανάμειξη μόνο για το deskew σε όλες τις 27 × 4: ουδέτερη κατά μέσο όρο εκτός οχήματος — κέρδη στο drone eee_03 / Hilti, απώλειες christ-church-02 / Office_Mitte_1

**Σχετικά:** #137, #138 · launcher `kiss_runs/bd140_launch.sh` (108 + Boreas, οι υπάρχοντες σπόροι παραλείφθηκαν· 112 / 112, τέλος 02:05) · πίνακας `kiss_runs/results_official_allarms_140.{md,csv}`
(βραχίονας `bd137`· στο `results_table.py` και οι ετικέτες των `blend131`, `multit131`, `sec132`, `fr132`, `ds132`, `sr135`)

APE m (μ. ό. ± σ, 4 σπόροι), KISS (ένα run) · προεπιλογή → ανάμειξη για deskew · RTE % · RPE 1 m cm / °:

| ακολουθία | KISS | προεπιλογή | ανάμειξη deskew | % APE | RTE | RPE 1 m |
|---|---|---|---|---|---|---|
| 01_short | 0.348 | 0.301 ± 0.012 | 0.304 ± 0.016 | +1 | 0.72 / 0.73 | 10.70 / 1.839 → 10.25 / 1.838 |
| quad_easy | 0.108 | 0.077 ± 0.000 | 0.077 ± 0.000 | 0 | 0.26 / 0.26 | 5.49 / 0.591 → 5.40 / 0.574 |
| cloister | 0.181 | 0.180 ± 0.016 | 0.180 ± 0.025 | 0 | 0.30 / 0.30 | 6.24 / 0.711 → 6.19 / 0.690 |
| math_easy | 0.173 | 0.103 ± 0.001 | 0.103 ± 0.002 | 0 | 0.51 / 0.51 | 7.84 / 1.256 → 7.65 / 1.269 |
| underground_easy | 0.067 | 0.066 ± 0.002 | 0.065 ± 0.002 | 0 | 0.28 / 0.28 | 4.61 / 0.628 → 4.59 / 0.625 |
| quad_hard | 0.163 | 0.233 ± 0.021 | 0.234 ± 0.029 | 0 | 0.86 / 0.91 | 8.31 / 1.042 → 8.55 / 1.071 |
| math_medium | 0.888 | 0.153 ± 0.003 | 0.157 ± 0.004 | +2 | 0.94 / 0.97 | 12.93 / 2.453 → 12.64 / 2.458 |
| underground_medium | 0.096 | 0.057 ± 0.003 | 0.059 ± 0.002 | +3 | 0.22 / 0.22 | 4.73 / 0.793 → 4.77 / 0.786 |
| underground_hard | 3.846 | 0.091 ± 0.001 | 0.091 ± 0.003 | 0 | 0.25 / 0.25 | 5.90 / 1.088 → 5.93 / 1.116 |
| christ-church-02 | 3.022 | 0.160 ± 0.029 | **0.227 ± 0.054** | **+42** | 0.28 / 0.31 | 5.26 / 0.706 → 5.32 / 0.706 |
| christ-church-03 | 0.122 | 0.039 ± 0.000 | 0.039 ± 0.001 | +1 | 0.12 / 0.13 | 3.78 / 0.700 → 3.93 / 0.705 |
| keble-03 | 0.854 | 0.089 ± 0.003 | 0.090 ± 0.001 | +1 | 0.68 / 0.66 | 8.19 / 1.040 → 8.09 / 1.031 |
| observatory-01 | 0.172 | 0.072 ± 0.008 | 0.067 ± 0.007 | −7 | 0.27 / 0.26 | 5.58 / 0.730 → 5.58 / 0.731 |
| blenheim-02 | 0.540 | 0.214 ± 0.009 | **0.197 ± 0.007** | −8 | 0.54 / 0.49 | 7.77 / 0.923 → 7.53 / 0.876 |
| bodleian-02 | 2.547 | 0.649 ± 0.063 | 0.564 ± 0.100 | −13 | 1.04 / 0.97 | 9.27 / 1.115 → 9.39 / 1.102 |
| 02_long | 1.275 | 2.190 ± 0.187 | 1.901 ± 0.718 | −13 | 1.07 / 1.03 | 11.43 / 2.871 → 11.36 / 2.890 |
| Basement_1 | 0.055 | 0.053 ± 0.005 | 0.046 ± 0.010 | −14 | – | – |
| IC_Office_1 | 6.344 | 0.073 ± 0.003 | 0.074 ± 0.006 | +1 | – | – |
| Office_Mitte_1 | 4.286 | 0.185 ± 0.054 | 0.248 ± 0.063 | +34 | – | – |
| Construction_Site_1 | 0.063 | 0.042 ± 0.003 | **0.037 ± 0.002** | −12 | – | – |
| LAB_Survey_2 | 0.062 | 0.035 ± 0.000 | 0.035 ± 0.000 | 0 | – | – |
| UZH_Tracking_Area_Run_2 | 0.585 | 0.576 ± 0.001 | 0.576 ± 0.002 | 0 | – | – |
| eee_01 | 2.678 | 1.483 ± 0.149 | 1.553 ± 0.126 | +5 | – | – |
| eee_02 | 1.486 | 0.836 ± 0.038 | 0.790 ± 0.088 | −6 | – | – |
| eee_03 | 0.864 | 0.360 ± 0.073 | **0.142 ± 0.002** | **−60** | – | – |
| *stairs (ειδική)* | 0.458 | 1.923 ± 0.415 | 1.643 ± 0.289 | −15 | – | 37.26 / 7.040 → 36.63 / 7.455 |
| *dynamic_spinning (ειδική)* | 0.159 | 0.171 ± 0.091 | 0.111 ± 0.017 | −35 | – | 14.58 / 3.186 → 13.17 / 3.130 |
| Boreas (#138) | 0.266 | 0.233 ± 0.008 | **0.195 ± 0.009** | **−16** | 0.35 / 0.27 | 4.96 / 0.210 → 3.71 / 0.158 |

Σύνοψη (25 ακολουθίες χωρίς τις ειδικές): APE καλύτερο 8, χειρότερο 7, ίδιο (±1 %) 10· πέρα από 1σ καλύτερο 3 (eee_03, Construction_Site_1, blenheim), χειρότερο 2 (christ-church-02,
Office_Mitte_1)· RPE 1 m (16 ακολουθίες με RPE) διάμεσος −0.5 % μετατόπιση, 0 % στροφή, καλύτερο σε 10 / 8.
**Συμπέρασμα ⏳:** σε φορητή μονάδα ουδέτερη (η εικόνα κρατά ~95 % του βάρους), με δύο απώλειες του Hesai / Hilti κοντά στη διασπορά (christ-church-02 ~1.5σ, Office_Mitte_1 ~1σ — και οι δύο έχουν
μεγάλη διασπορά)· κέρδη στο drone eee_03 και στα Hilti Construction_Site / Basement. Το καθαρό κέρδος είναι στο όχημα (Boreas, #138). Ως προεπιλογή: απόφαση Μ.Τ.· τα christ-church-02 /
Office_Mitte_1 θέλουν έλεγχο (βάρη ανά σάρωση) πριν.
**Έλεγχος των δύο απωλειών (προσθήκη 4/10):** βάρη της ανάμειξης ανακατασκευασμένα από τις αποθηκευμένες κινήσεις / θέσεις (ίδιος κανόνας), μ. ό. 4 σπόρων — βάρος εικόνας / σαρώσεις < 0.8 /
< 0.5: christ-church-02 0.95 / 3 % / 0.7 %· christ-church-03 0.95 / 4 % / 0 %· quad_easy 0.95 / 2 % / 0 %· **Office_Mitte_1 0.84 / 24 % / 3 %**· eee_03 0.44 / 99 % / 58 %· Boreas 0.29 / 100 % / 86 %.
christ-church-02 ανά σπόρο (APE / z RMSE m): προεπιλογή 0.126 / 0.107, 0.196 / 0.185, 0.166 / 0.147, 0.152 / 0.137· ανάμειξη 0.257 / 0.242, 0.250 / 0.239, 0.256 / 0.243, 0.146 / 0.132 — RPE (5.30–5.33 /
0.70–0.71) και μήκος ίδια. Office_Mitte_1 (APE m): προεπιλογή 0.180 / 0.153 / 0.143 / 0.262· ανάμειξη 0.245 / 0.182 / 0.233 / 0.333.
**Συμπέρασμα ⏳:** στο christ-church-02 η ανάμειξη σχεδόν δεν αλλάζει το deskew (βάρος 0.95, όπως στις ουδέτερες quad_easy / christ-church-03)· η απώλεια είναι το ύψος (z RMSE ≈ APE), με
δύο καταστάσεις ανά σπόρο (~0.13 / ~0.25 m) — η γνωστή αστάθεια ύψους της ακολουθίας (#060), που 4 σπόροι δεν ξεχωρίζουν από τύχη. Στο Office_Mitte_1 η ανάμειξη δρα (24 % των σαρώσεων
< 0.8)· οι περιοχές επικαλύπτονται (0.143–0.262 έναντι 0.182–0.333 m, 9 σημεία ελέγχου) — πιθανή μικρή βλάβη, αβέβαιη. Τα κέρδη έρχονται εκεί όπου το βάρος πέφτει πολύ (eee_03 0.44,
Boreas 0.29).

---

### 2026-10-03 — #139 Υπο-δοκιμή: στροφή μόνο από τομείς (7 × 4) — στροφή καλύτερη σε 4 / 6, μετατόπιση πια ουδέτερη εκτός underground_hard / christ-church-03

**Σχετικά:** #134, #135 · launcher `kiss_runs/sr139_launch.sh` · βραχίονας `sr135` (`--sectors=8 --sectors-part=rotation`)· σύγκριση με base092 (Boreas base127) και τους πλήρεις τομείς `sec132`

Μ. ό. ± σ (4 σπόροι). APE m · RTE % · RPE 1 m cm / ° (NTU: ATE).

| ακολουθία | προεπιλογή | πλήρεις τομείς | στροφή μόνο από τομείς |
|---|---|---|---|
| quad_easy | 0.077 ± 0.001 · 0.263 · 5.48 / 0.592 | 0.078 · 0.255 · 5.70 / 0.551 | **0.076 · 0.258 · 5.35 / 0.548** |
| cloister | 0.180 ± 0.016 · 0.297 · 6.24 / 0.711 | 0.180 · 0.292 · 6.24 / 0.685 | 0.169 ± 0.022 · 0.298 · 6.20 / **0.678** |
| underground_hard | 0.091 ± 0.001 · 0.247 · 5.90 / 1.088 | 0.090 · 0.253 · 6.14 / 1.113 | 0.089 · 0.253 · 6.06 / 1.122 |
| christ-church-03 | 0.039 ± 0.000 · 0.125 · 3.79 / 0.700 | 0.044 · 0.130 · 3.75 / 0.712 | 0.043 ± 0.002 · 0.125 · 3.72 / 0.712 |
| keble-03 | 0.089 ± 0.003 · 0.677 · 8.19 / 1.040 | 0.094 · 0.627 · 8.36 / 1.024 | 0.090 · 0.637 · 8.17 / **1.022** |
| eee_01 | 1.483 ± 0.149 | 1.544 ± 0.133 | 1.585 ± 0.154 |
| Boreas | 0.233 ± 0.008 · 0.350 · 4.96 / 0.210 | 0.234 · 0.360 · 4.84 / 0.195 | 0.235 · 0.363 · 4.75 / **0.193** |

**Συμπέρασμα ⏳:** η μετατόπιση από την κανονική προσαρμογή αφαιρεί το κόστος των πλήρων τομέων στο quad_easy / keble (RPE 5.35 / 8.17 έναντι 5.70 / 8.36), και η στροφή μένει καλύτερη σε
4 / 6 (−2…−8 %). Δεν διορθώνει το underground_hard (+3 %, κι εκεί η στροφή χειρότερη) ούτε το christ-church-03 (APE +9 %). Μικρότερο και λιγότερο καθαρό κέρδος από το #138· δεν
προχωρά στις 27 μόνο του — πιθανός συνδυασμός με την ανάμειξη για το deskew μετά το #140.

---

### 2026-10-03 — #138 Υπο-δοκιμή της ανάμειξης μόνο για το deskew (9 × 4): όχημα −16…−25 %, drone eee_03 −60 %, φορητή ουδέτερη

**Σχετικά:** #131, #133, #137 · launcher `kiss_runs/bd138_launch.sh` · βραχίονας `bd137` (`--cv-blend=adaptive --cv-blend-use=deskew`)· σύγκριση με base092 (Boreas base127) και την πλήρη ανάμειξη `blend131`

Μ. ό. ± σ (4 σπόροι). APE m · RTE % · RPE 1 m cm / ° (NTU: ATE).

| ακολουθία | προεπιλογή | πλήρης ανάμειξη | ανάμειξη μόνο deskew |
|---|---|---|---|
| quad_easy | 0.077 ± 0.001 · 0.263 · 5.48 / 0.592 | 0.076 · 0.268 · 5.38 / 0.570 | 0.077 · 0.260 · 5.40 / 0.574 |
| cloister | 0.180 ± 0.016 · 0.297 · 6.24 / 0.711 | 0.202 · 0.302 · 6.32 / 0.699 | 0.180 ± 0.025 · 0.295 · 6.19 / 0.690 |
| underground_hard | 0.091 ± 0.001 · 0.247 · 5.90 / 1.088 | 0.090 · 0.247 · 6.04 / 1.138 | 0.091 · 0.247 · 5.93 / 1.116 |
| christ-church-03 | 0.039 ± 0.000 · 0.125 · 3.79 / 0.700 | 0.039 · 0.122 · 3.77 / 0.706 | 0.039 · 0.133 · 3.93 / 0.705 |
| keble-03 | 0.089 ± 0.003 · 0.677 · 8.19 / 1.040 | 0.091 · 0.645 · 8.08 / 1.028 | 0.090 · 0.663 · 8.09 / 1.031 |
| eee_01 | 1.483 ± 0.149 | 1.966 ± 0.079 | 1.553 ± 0.127 |
| eee_02 | 0.836 ± 0.038 | 1.270 ± 0.070 | 0.805 ± 0.078 |
| eee_03 | 0.360 ± 0.073 | 0.654 ± 0.010 | **0.142 ± 0.002** |
| Boreas | 0.233 ± 0.008 · 0.350 · 4.96 / 0.210 | 0.202 · 0.278 · 3.67 / 0.161 | **0.195 ± 0.009 · 0.270 · 3.71 / 0.158** |

**Συμπέρασμα ⏳:** η ανάμειξη μόνο για το deskew κρατά όλο το κέρδος του οχήματος (−16…−25 %), αφαιρεί τη βλάβη στο drone και μάλιστα κόβει το eee_03 κατά 60 % (με σ 0.002 έναντι 0.073: η
προεπιλογή εκεί είναι ασταθής ανάμεσα στους σπόρους)· στη φορητή μονάδα ουδέτερη (η εικόνα παίρνει ~95 % του βάρους), με μικρές αλλαγές ±1–6 %. Πρώτη αλλαγή του φετινού κύκλου που δεν
βλάπτει καμία πλατφόρμα στην υπο-δοκιμή· υποψήφια για τις 27 × 4.

---

### 2026-10-03 — #135 Υπο-δοκιμή σύντηξης εικόνας απόστασης / «ακίνητων» (7 × 4)· στροφή μόνο από τομείς (ένα run): κρατά το κέρδος στροφής χωρίς το κόστος μετατόπισης

**Σχετικά:** #132, #134 · launcher `kiss_runs/stage135_launch.sh` · βραχίονες `fr132` (`--fuse-range`), `ds132` (`--drop-stationary`), `sr135` (`--sectors=8 --sectors-part=rotation`)

Μ. ό. ± σ (4 σπόροι). APE m · RTE % · RPE 1 m cm / ° (NTU: ATE).

| ακολουθία | προεπιλογή | σύντηξη απόστασης | χωρίς «ακίνητα» |
|---|---|---|---|
| quad_easy | 0.077 ± 0.001 · 0.263 · 5.48 / 0.592 | 0.078 · 0.265 · 5.52 / 0.601 | 0.077 · 0.260 · 5.48 / 0.601 |
| cloister | 0.180 ± 0.016 · 0.297 · 6.24 / 0.711 | **0.216 ± 0.011 · 0.338** · 6.53 / 0.731 | 0.178 ± 0.025 · 0.310 · 6.33 / 0.696 |
| underground_hard | 0.091 ± 0.001 · 0.247 · 5.90 / 1.088 | **0.086 · 0.228 · 5.61 / 1.013** | 0.090 · 0.250 · 5.84 / 1.067 |
| christ-church-03 | 0.039 ± 0.000 · 0.125 · 3.79 / 0.700 | **0.036 · 0.120 · 3.64 / 0.668** | 0.038 · 0.128 · 3.81 / 0.701 |
| keble-03 | 0.089 ± 0.003 · 0.677 · 8.19 / 1.040 | 0.091 · 0.683 · 7.98 / 1.025 | 0.087 · 0.653 · 8.03 / 1.039 |
| eee_01 | 1.483 ± 0.149 | 1.633 ± 0.129 | 1.293 ± 0.153 |
| Boreas | 0.233 ± 0.008 · 0.350 · 4.96 / 0.210 | 0.377 ± 0.323 · 0.365 · **4.22 / 0.151** | 0.230 · 0.347 · 4.94 / 0.211 |

Boreas, σύντηξη ανά σπόρο (APE / RPE / RTE): 0.209 / 4.13 / 0.149 / 0.29 · 0.222 / 4.41 / 0.155 / 0.33 · **0.861** / 4.32 / 0.155 / 0.55 · 0.214 / 4.03 / 0.145 / 0.29 — ένα άλμα στον σπόρο 2.
**Στροφή μόνο από τομείς, quad_easy, ένα run:** APE 0.077 · RTE 0.260 · RPE 5.38 / **0.546** (πλήρεις τομείς 0.077 · 0.250 · 5.72 / 0.550).
**Συμπέρασμα ⏳:** (1) Η σύντηξη απόστασης βοηθά εκεί όπου η εικόνα intensity δυσκολεύεται (underground_hard, Boreas: στροφή −7 / −28 %) αλλά βλάπτει στο cloister (+20 % APE, +13 % RTE) και
στο eee_01, και έδωσε ένα άλμα στο Boreas — δεν είναι γενική. (2) Τα «ακίνητα»: όλα μέσα στη διασπορά (μόνο eee_01 −13 %, ~1σ) — όχι. (3) Η στροφή από τομείς κρατά όλο το κέρδος στροφής
(−8 %) χωρίς το +4 % της μετατόπισης — επόμενο: υπο-δοκιμή 7 × 4.

---

### 2026-10-03 — #137 Πού βλάπτει η ανάμειξη στο drone: στην αρχή του ICP, όχι στο deskew ούτε στη μετατόπιση (#136: αναφορές για τα βάρη, offline)

**Σχετικά:** #131, #133, #134 · ζήτημα Μ.Τ. 3/10 («άλλες ιδέες για καλύτερη ανάμειξη») · `scripts/analyse_blend_reference.py` (#136, έξοδος `kiss_runs/blendref136.txt`) · νέες επιλογές
`image_deskew.cv_blend_part: full | rotation` (`--cv-blend-part`), `image_deskew.cv_blend_use: both | deskew` (`--cv-blend-use`) · launchers `kiss_runs/blend137{,b}_launch.sh`

**#136, offline, στροφή ανά σάρωση (° διάμεσος, 2 σπόροι, έναντι GT / γυροσκοπίου στο drone), εικόνα / CV / βήμα ICP | ανάμειξη με βάρη από: ICP / τρίγωνο αποκλίσεων («three-cornered hat», καμία
εκτίμηση ως αλήθεια) / μόνο εικόνα:** eee_01 0.457 / 0.48 / **0.44** | 0.446 / 0.453 / 0.457· eee_02 0.426 / 0.415 / **0.389** | 0.405 / 0.415 / 0.415· eee_03 0.433 / 0.432 / **0.376** | 0.402 / 0.410 / 0.415·
Boreas 0.137 / 0.148 / **0.104** | **0.120** / 0.123 / 0.132· quad_easy 0.588 / 1.61 / 0.53 | **0.555** / 0.587 / 0.557· cloister 0.365 / 1.59 / 0.55 | **0.320** / 0.362 / 0.340· underground_hard 0.526 / 2.87 / 0.70 |
**0.499** / 0.525 / 0.580· bodleian 0.800 / 2.42 / 1.05 | **0.768** / 0.801 / 0.862. Βάρος εικόνας (ICP / hat / εικόνα): φορητή ~0.95 / ~1.00 / ~0.85, Boreas 0.65 / 0.72 / 0.92, drone ~0.77 / ~0.87 / ~0.84.
Ο ICP του drone δεν είναι χειρότερος στη στροφή ανά σάρωση· όλα κοντά στο πάτωμα του γυροσκοπίου (~0.4°). Οι δύο νέες αναφορές δεν ξεπερνούν τον ICP.
**Συνθετικός έλεγχος του κανόνα:** εικόνα θορυβώδης / CV ακριβής → βάρος εικόνας 0.00 (σωστό)· εικόνα ακριβής / CV θορυβώδης → 0.67 αντί ~1: ο θόρυβος της αναφοράς (ICP) προστίθεται και στις
δύο προβλέψεις και τραβά το βάρος προς 0.5.
**#137, eee_02 (ATE m, NTU):** προεπιλογή 0.887 / 0.844 / 0.808 / 0.806 (0.836 ± 0.038)· ανάμειξη 1.375 / 1.241 / 1.225 / 1.240 (1.270)· **μόνο στροφή 1.281**· **μόνο deskew (αρχή ICP = εικόνα) 0.851**.
**Συμπέρασμα ⏳:** η βλάβη στο drone προέρχεται από την αρχή του ICP: σε σάρωση 16 δεσμών η λεκάνη σύγκλισης είναι στενή και μια αρχή τραβηγμένη προς τη σταθερή ταχύτητα οδηγεί σε άλλη
λύση· το deskew ανέχεται το ίδιο σφάλμα.
**Boreas, μόνο deskew (ένα run)** — APE m · RPE 1 m cm / ° · RTE % · RRE °/100 m · μήκος m: KISS 0.266 · 4.14 / 0.192 · 0.28 · 0.149 · 1326.4· προεπιλογή (4) 0.225–0.240 · 4.76–5.19 / 0.199–0.216 ·
0.33–0.37 · 0.175–0.217 · 1332–1335· πλήρης ανάμειξη (4) 0.195–0.207 · 3.48–3.92 / 0.156–0.168 · 0.27–0.29 · 0.134–0.142 · 1324–1325· **μόνο deskew 0.182 · 3.64 / 0.155 · 0.27 · 0.130 · 1323.9**.
Το κέρδος του οχήματος μένει ολόκληρο (και λίγο καλύτερο APE)· η ανάμειξη μόνο για το deskew είναι η υποψήφια παραλλαγή για υπο-δοκιμή.

---

### 2026-10-03 — #134 Τομείς σε 7 × 4 (στροφή καλύτερη, μετατόπιση χειρότερη)· ανάμειξη στο drone χειρότερη και σε eee_02 / 03· σύντηξη απόστασης / «ακίνητα» στο underground_hard: καλύτερα

**Σχετικά:** #132, #133 · launchers `kiss_runs/stage133_launch.sh`, `kiss_runs/blend134_launch.sh` · βραχίονες `sec132` (`--sectors=8`), `fr132`, `ds132`, `blend131`

Μ. ό. ± σ (4 σπόροι· ένα run όπου n = 1). APE m · RTE % · RPE 1 m cm / ° (NTU: ATE).

| ακολουθία | προεπιλογή | τομείς | |
|---|---|---|---|
| quad_easy | 0.077 ± 0.001 · 0.263 · 5.48 / 0.592 ± 0.002 | 0.078 · 0.255 · 5.70 / **0.551** | στροφή −7 %, μετατ. +4 % |
| cloister | 0.180 ± 0.016 · 0.297 · 6.24 / 0.711 ± 0.008 | 0.180 · 0.292 · 6.24 / **0.685** | στροφή −4 % |
| underground_hard | 0.091 ± 0.001 · 0.247 · 5.90 / 1.088 ± 0.016 | 0.090 · 0.253 · 6.14 / 1.113 | +4 / +2 % |
| christ-church-03 | 0.039 ± 0.000 · 0.125 · 3.79 / 0.700 | 0.044 ± 0.002 · 0.130 · 3.75 / 0.712 | APE +13 % |
| keble-03 | 0.089 ± 0.003 · 0.677 ± 0.070 · 8.19 / 1.040 | 0.094 · 0.627 · 8.36 / **1.024** | RTE −7 %, στροφή −2 % |
| eee_01 | 1.483 ± 0.149 | 1.544 ± 0.133 | +4 % (μέσα σε σ) |
| Boreas | 0.233 ± 0.008 · 0.350 · 4.96 / 0.210 ± 0.008 | 0.234 · 0.360 · 4.84 / **0.195** | στροφή −7 % |

**Ανάμειξη, drone (ATE m):** eee_02 0.836 ± 0.038 → **1.270 ± 0.070 (+52 %)**· eee_03 0.360 ± 0.073 → **0.654 ± 0.010 (+82 %)** (eee_01 +33 %, #133).
**Ένα run, underground_hard** (προεπιλογή 0.091 ± 0.001 · 0.247 ± 0.005 · 5.90 ± 0.11 / 1.088 ± 0.016): σύντηξη απόστασης **0.086 · 0.230 · 5.56 / 0.989** (−6 / −7 / −6 / −9 %)·
χωρίς «ακίνητα» 0.089 · 0.240 · 5.75 / 1.035 (−3 / −3 / −3 / −5 %). Κίνηση της εικόνας (στροφή ανά σάρωση / 10 σαρώσεις °, μετατόπιση 10 σαρώσεων mm): 0.524 / 2.30 / 109 →
0.494 / 2.06 / 92 (σύντηξη), 0.517 / 2.16 / 92 («ακίνητα»).
**Συμπέρασμα ⏳:** (1) Οι τομείς βελτιώνουν σταθερά τη στροφή (4 / 6 −2…−7 %) και χειροτερεύουν τη μετατόπιση — όπως offline (#130: η μετατόπιση στηρίζεται στις πυκνές κοντινές
αντιστοιχίσεις)· ιδέα: στροφή από την προσαρμογή με τομείς, μετατόπιση από την κανονική. (2) Η ανάμειξη βλάπτει σε όλο το drone (+33…+82 %) — η αναφορά της (ο ICP του OS1-16) δεν
αρκεί· ως τώρα βοηθά μόνο στο όχημα. (3) Στο underground_hard, όπου ο ICP δυσκολεύεται περισσότερο από το quad_easy, η καλύτερη κίνηση της εικόνας περνά στην τροχιά (σύντηξη −6…−9 %,
στροφή ~6σ)· επόμενο: υπο-δοκιμή σε 7 × 4.

---

### 2026-10-03 — #133 Υπο-δοκιμή του #131 (7 ακολουθίες × 4): η ανάμειξη βελτιώνει πολύ το όχημα (−13…−26 %), ουδέτερη στη φορητή, χειρότερη στο drone (+33 %)· μετατόπιση k−2 ουδέτερη

**Σχετικά:** #131 · ΑΠΟΦΑΣΗ Μ.Τ. 3/10 (υπο-δοκιμή πριν από τις 27) · launcher `kiss_runs/arms131_launch.sh` · βραχίονες `blend131` (`--cv-blend=adaptive`), `multit131`
(`--multi-baseline=translation`)· αναφορά base092 (Boreas: base127, ίδιος κώδικας)· KISS από τον πίνακα `results_official_allarms_121` (ένα run)

Μ. ό. ± σ, 4 σπόροι. APE m · RTE % · RPE 1 m cm / ° (NTU: μόνο ATE του πρωτοκόλλου).

| ακολουθία | KISS | προεπιλογή | ανάμειξη | μετατόπιση k−2 |
|---|---|---|---|---|
| quad_easy | 0.104 · 0.28 · 12.08 / 1.140 | 0.077 ± 0.001 · 0.263 · 5.48 / 0.592 | 0.076 · 0.268 · **5.38 / 0.570** | 0.076 · 0.268 · 5.40 / 0.589 |
| cloister | 0.396 · 0.77 · 19.47 / 2.883 | 0.180 ± 0.016 · 0.297 · 6.24 / 0.711 | 0.202 ± 0.004 · 0.302 · 6.32 / 0.699 | 0.178 · 0.297 · 6.20 / 0.710 |
| underground_hard | 12.78 · 14.83 · 241.6 / 17.24 | 0.091 ± 0.001 · 0.247 · 5.90 / 1.088 | 0.090 · 0.247 · 6.04 / 1.138 | 0.091 · 0.250 · 5.88 / 1.090 |
| christ-church-03 | 0.143 · 0.50 · 16.27 / 2.727 | 0.039 ± 0.000 · 0.125 · 3.79 / 0.700 | 0.039 · 0.122 · 3.77 / 0.706 | 0.040 · 0.130 · 3.89 / 0.707 |
| keble-03 | 9.757 · 26.33 · 60.52 / 4.480 | 0.089 ± 0.003 · 0.677 ± 0.070 · 8.19 / 1.040 | 0.091 · 0.645 · 8.08 / 1.028 | 0.093 · 0.640 · 8.04 / 1.042 |
| eee_01 (drone) | 2.678 | 1.483 ± 0.149 | **1.966 ± 0.079** | 1.519 ± 0.087 |
| Boreas (όχημα) | 0.266 · 0.28 · 4.14 / 0.192 | 0.233 ± 0.008 · 0.350 ± 0.018 · 4.96 / 0.210 | **0.202 ± 0.005 · 0.278 ± 0.010 · 3.67 / 0.161** | 0.236 · 0.355 · 4.68 / 0.205 |

**Συμπέρασμα ⏳:** (1) Στο όχημα η ανάμειξη είναι το μεγαλύτερο κέρδος της μεθόδου ως τώρα εκεί (#084 / #085: η μέθοδος ήταν χειρότερη από τον KISS)· τώρα καλύτερη και από τον KISS σε
όλα τα μέτρα εκτός του RTE (0.278 έναντι 0.28, ίσο). Η εικόνα σε ομαλή κίνηση έχει θόρυβο που η σταθερή ταχύτητα δεν έχει· τα βάρη τον μειώνουν χωρίς κατώφλι. (2) Στη φορητή μονάδα
η εικόνα παίρνει ~100 % του βάρους (#131), άρα ίδιο αποτέλεσμα, εκτός από μικρές αλλαγές (quad_easy στροφή −4 %, underground_hard +5 %). (3) Στο drone eee_01 +33 % (≈ 3σ): ο ICP
του OS1-16 (16 δέσμες) είναι αδύναμη αναφορά για τα βάρη — το ίδιο όριο με το stairs (#131). Χρειάζεται έλεγχος σε eee_02 / 03 πριν από οτιδήποτε. (4) Η μετατόπιση από k−2 δεν περνά
στην τροχιά πουθενά (ο ICP δίνει τη μετατόπιση).

---

### 2026-10-03 — #132 Επανέλεγχος με πλήρες run των αποφάσεων που πάρθηκαν μόνο offline: οι τομείς βελτιώνουν τη στροφή (−7 %)· whitening / bearings επιβεβαιώνονται ✗

**Σχετικά:** #087 (bearings), #090 (σύντηξη απόστασης), #093 (whitening, τομείς, cross-check), #123 / #124 (log, rangefit), #130 («ακίνητα») · ΑΠΟΦΑΣΗ Μ.Τ. 3/10 (πρώτα ένα πλήρες run) ·
νέες επιλογές `image_deskew.fuse_range` (`--fuse-range`), `image_deskew.drop_stationary` (`--drop-stationary`: ο κανόνας του #130 μόνο όταν |t| > 2 × διάμεσο υπόλοιπο — σε ακινησία
ο πρώτος κανόνας πετούσε τα μισά ζεύγη, 7 → 115 mm· διορθωμένο: ακίνητες 7.3 → 7.3 mm, κινούμενες 15.5 → 13.6 mm, κλίμακα 0.90 → 0.93) · launcher `kiss_runs/retest132_launch.sh`

Σπόρος 0, ένα run, έναντι base092 (4 σπόροι). **quad_easy, επίσημο evo** (APE m · RPE 1 m cm / ° · RRE °/100 m · μήκος m, GT ~264) και κίνηση της εικόνας έναντι GT (στροφή ανά σάρωση /
10 σαρώσεις °, μετατόπιση 10 σαρώσεων mm):

| | APE | RPE 1 m | RRE | μήκος | εικόνα: στροφή / 10 | μετατ. 10 |
|---|---|---|---|---|---|---|
| base092 s0–s3 | 0.077–0.078 | 5.46–5.51 / 0.589–0.593 | 1.05–1.08 | 265.6–266.1 | 0.588 / 2.44 | 142 |
| whitening (#093) | 0.082 | 6.12 / 0.681 | 1.213 | 271.2 | 0.809 / 4.54 | 142 |
| **τομείς 8 (#093)** | 0.077 | 5.72 / **0.550** | **1.037** | 267.2 | **0.537** / 2.46 | 224 |
| cross-check (#093) | 0.078 | 5.52 / 0.598 | 1.066 | 265.6 | 0.603 / 2.41 | 136 |
| bearings > 5 m (#087) | 0.077 | 5.73 / 0.639 | 1.081 | 266.9 | 0.624 / 2.00 | 142 |
| σύντηξη απόστασης (#090) | 0.078 | 5.55 / 0.603 | 1.048 | 266.3 | 0.551 / 1.91 | 103 |
| χωρίς «ακίνητα» (#130) | 0.078 | 5.48 / 0.600 | 1.058 | 265.6 | 0.590 / 2.15 | 102 |

**Office_Mitte_1 (Hilti, APE m):** base092 0.180 / 0.153 / 0.143 / 0.262 (0.185 ± 0.054)· log I 0.254· rangefit 0.174.
**Συμπέρασμα ⏳:** (1) whitening και bearings χειρότερα και στην τροχιά — η offline απόφαση ισχύει. (2) Οι τομείς: η στροφή της τροχιάς −7 % (0.550 έναντι 0.589–0.593°, διασπορά ±0.002)
και RRE −2 %, με μετατόπιση +4 % — offline είχαν απορριφθεί με τη στροφή 10 σαρώσεων· άξιοι υπο-δοκιμής. (3) Σύντηξη απόστασης και «ακίνητα» βελτιώνουν την κίνηση της εικόνας
(μετατόπιση 10 σαρώσεων −28 %) χωρίς αλλαγή στην τροχιά του quad_easy: εκεί τη μετατόπιση τη δίνει ο ICP — η δοκιμή τους χρειάζεται ακολουθίες όπου ο ICP δυσκολεύεται.
(4) Hilti: ένα run μέσα στη διασπορά (σ = 0.054 m με 9 σημεία ελέγχου) — δεν κρίνει.

---

### 2026-10-03 — #121 Αποτέλεσμα: κανονικοποίηση του intensity σε πλήρη runs — ομαλή καμπύλη όχι· I · r² μόνο στο OS0-128

**Σχετικά:** #118, #123–#126 · launcher `kiss_runs/rnorm121_launch.sh` (worktree `deskew_in_icp`) · βραχίονες `rsmooth121` (27 × 4), `rsq121` (21 Ouster × 4) · πίνακας
`kiss_runs/results_official_allarms_121.{md,csv}` (192 / 192 runs)

APE (m, 4 σπόροι), προεπιλογή / ομαλή / I·r²: quad_hard 0.233 / 0.198 / **0.179**· math_medium 0.153 / 0.155 / **0.145**· underground_medium 0.057 / 0.059 / 0.055· underground_hard 0.091 /
0.093 / 0.089· quad_easy, cloister, math_easy, underground_easy ίδια· 02_long 2.19 / 1.77 ± 0.82 / 2.32· Hesai (μόνο ομαλή): christ-church-02 0.160 / 0.199, christ-church-03 0.039 / 0.041,
keble 0.089 / 0.093, observatory 0.072 / **0.209**, blenheim 0.214 / 0.232, bodleian 0.649 / **0.438**· Hilti: Office_Mitte_1 0.185 / 0.272 / 0.200, IC_Office_1 0.073 / 0.077 / 0.081,
Basement_1 0.053 / 0.058 / 0.049, Construction_Site_1 0.042 / 0.043 / 0.040· NTU eee_01 1.48 / 1.80 / 1.77, eee_02 0.84 / 1.01 / 0.99, eee_03 0.360 / 0.389 / **0.306**.
RPE 1 m (NCD / Spires): I·r² −1…−4 % στα OS0-128, ομαλή +1…+7 % στα Hesai. (stairs, dynamic_spinning: ειδικές συνθήκες, χωριστά — χειρότερα και τα δύο.)
**Συμπέρασμα ⏳:** η ομαλή καμπύλη, ασφαλής offline (#118), είναι χειρότερη στην τροχιά σχεδόν παντού — ξανά το ανά σάρωση δεν προβλέπει την τροχιά. Το I·r² βοηθά λίγο στο
OS0-128 και βλάπτει στο OS1-16· η κανονικοποίηση εξαρτάται από τον αισθητήρα. Προεπιλογή αμετάβλητη.

---

### 2026-10-03 — #131 Προσαρμοστική ανάμειξη εικόνας και σταθερής ταχύτητας (χωρίς κατώφλι): όχημα −12 %, drone −2…−7 %, φορητή μικτό (stairs χειρότερο)

**Σχετικά:** #127 · ζήτημα Μ.Τ. 3/10 («καλύτερα / προσαρμοστικά κατώφλια στο όχημα»· «όχι όλα φορητά, έλεγξε 3») · `scripts/analyse_gated_deskew.py` (track-best, blend),
`scripts/score_blend_gyro.py` (NTU, γυροσκόπιο του Ouster) · `kiss_runs/gate131.txt`, `kiss_runs/blend131_eee_0{1,2,3}.txt`

Για τη σάρωση k, το πρόσφατο σφάλμα κάθε πρόβλεψης (εικόνα M_j, σταθερή ταχύτητα C_j) έναντι του βήματος του ICP P_{j−1}⁻¹P_j στις w προηγούμενες (αιτιακό): «track-best» η
μικρότερη· «blend» slerp / γραμμική με βάρος εικόνας v_C / (v_I + v_C). Στροφή (°) / μετατόπιση (mm) ανά σάρωση έναντι του GT της σάρωσης, μ. ό. 4 σπόρων, εικόνα → blend w = 20:
Boreas 0.137 / 26.1 → **0.120 / 22.0** (CV 0.147 / 29.7, όριο από το GT 0.104 / 25.5· track-best 0.137 — κανένα κέρδος)· cloister 0.365 → 0.321· underground_medium 0.422 → 0.386·
quad_easy 0.589 → 0.556· keble 0.730 → 0.695· church_03 0.470 → 0.452· math_medium 1.76 → 1.71· 01_short 1.02 → 1.02· **stairs 1.03 / 12.8 → 1.09 / 19.9**· **dynamic_spinning 1.24 → 1.36**.
NTU drone, έναντι του γυροσκοπίου (σαρώσεις με στροφή > 0.2°), εικόνα / CV / blend: eee_01 0.455 / 0.480 / 0.444· eee_02 0.421 / 0.441 / 0.406· eee_03 0.429 / 0.430 / 0.400 (βάρος εικόνας ~0.77·
το γυροσκόπιο του Ouster θέτει πάτωμα ~0.4°).
**Συμπέρασμα ⏳:** η ανάμειξη με βάρη από τα πρόσφατα σφάλματα είναι γενικός κανόνας χωρίς κατώφλι και βοηθά στο όχημα και λίγο στο drone· στη φορητή μονάδα μικρό κέρδος, εκτός
όπου ο ICP αποτυγχάνει (η αναφορά του βάρους είναι ο ICP). Ανά σάρωση μόνο — η τροχιά θέλει πλήρη runs.


**Υλοποίηση και δοκιμή στο quad_easy (προσθήκη 3/10):** επιλογές `image_deskew.cv_blend: adaptive` (`--cv-blend=adaptive`, παράθυρο 20) και `image_deskew.multi_baseline`
(`--multi-baseline`, `--multi-baseline=translation`), όλες ανενεργές από προεπιλογή. Ένα run, σπόρος 0, έναντι base092 (4 σπόροι: APE 0.077, RPE 1 m 5.46–5.51 cm / 0.589–0.593°):
ανάμειξη 0.076 / 5.37 cm / 0.571°· k−2 κοινή προσαρμογή 0.084 / 6.79 / 0.784 ✗ (στροφή ανά σάρωση 0.59 → 0.95°: το deskew θέλει τη στροφή ανά σάρωση)· **k−2 μόνο μετατόπιση**
0.076 / 5.44 / 0.592 (στροφή ανά σάρωση 0.591, μετατόπιση 10 σαρώσεων 142 → 70 mm). Πλήρη runs `blend131`, `multit131` (27 × 4 + Boreas × 4) μετά το #121:
`kiss_runs/arms131_launch.sh`.

---

### 2026-10-03 — #130 Γιατί η μετατόπιση της εικόνας είναι κοντή, και η διόρθωση: μεγαλύτερη βάση (k−2 → k) — −39…−48 % σε 10 σαρώσεις στα Ouster

**Σχετικά:** #023, #039 (μεροληψία κοντινού πεδίου, μόνο Hesai), #090 (k−2: απορρίφθηκε με το σφάλμα στροφής ανά σάρωση), #128 · ζήτημα Μ.Τ. 3/10 («γιατί αυτή η σταθερή
απόκλιση», «μακρινές / καλύτερο φιλτράρισμα / η πιο συνεπής κίνηση αντί για μέσο») · scripts `analyse_translation_scale.py`, `analyse_translation_consensus.py` ·
`kiss_runs/scale130_*.txt`, `scale130b_*_multi.txt`

**Μέτρηση (base092, 14 ακολουθίες):** κλίμακα της εικόνας 0.74–0.985, πάντα < 1· σε 10 σαρώσεις το 77–95 % του σφάλματος κατά μήκος της κίνησης· τα σφάλματα ανά σάρωση του ICP
αρνητικά συσχετισμένα (−0.2…−0.7, αλληλοαναιρούνται), της εικόνας όχι (ίδιο πρόσημο → αθροίζονται).
**Αιτία (επανάληψη της κίνησης, 150–300 σαρώσεις, ζεύγη inliers):** κάθε ζεύγος έναντι της κίνησης του GT στον χρόνο κάθε σημείου (λ = μέρος της αληθινής μετατόπισης):
quad_easy 0–2 / 2–4 / 4–8 / 8–16 / > 16 m: 0.83 / 0.92 / 0.99 / 1.04 / 1.12 (όλα 0.96)· underground_hard 0.91 / 0.94 / 0.99 / 1.02· stairs 0.90 / 0.92 / 0.96· math_medium 0.91 / 0.80 / 0.87 /
0.94 (όλα 0.84, 27 % των ζευγών < 0.2)· church_03 (Hesai) 0.98 / 1.00 / 1.00 / 0.99. Η προσαρμογή κονταίνει περαιτέρω (0.96 → 0.93· 0.84 → 0.71· Hesai 0.99 → 0.996).
Όχι η καθοδηγούμενη αντιστοίχιση (χωρίς: 0.919 έναντι 0.925)· όχι ξεχωριστή ομάδα ακίνητων ζευγών (μία πλατιά κορυφή).
**Διορθώσεις, σφάλμα μετατόπισης 10 σαρώσεων (mm, 300 σαρώσεις, σπόρος 0):**

| | μέθοδος | χωρίς «ακίνητα» | συναίνεση (2.5 MAD) | > 8 m | μακρινότερο 30 % | mode | **k−2 → k** |
|---|---|---|---|---|---|---|---|
| quad_easy | 115 | 85 | 111 | 114 | 75 | 125 | **60** |
| underground_hard | 82 | 63 | 82 | 101 | 51 | 91 | **48** |
| stairs | 72 | 59 | 71 | 100 | 77 | 85 | **44** |
| math_medium | 107 | 98 | 103 | 135 | 108 | 145 | **96** |
| church_03 (Hesai) | 32 | 37 | 31 | 87 | 73 | 42 | **30** |

k−2: κλίμακα 0.95–0.97 (Hesai 0.98)· στροφή ανά σάρωση χειρότερη (quad_easy 0.51 → 0.75°, όπως #090), **στροφή 10 σαρώσεων καλύτερη** σε quad_easy (2.16 → 1.52°), underground_hard
(1.63 → 1.16°), stairs (2.37 → 2.14°), ίση στο church_03, χειρότερη στο math_medium (2.16 → 2.44°). Βαθμονόμηση της κλίμακας έναντι του ICP (αιτιακός διάμεσος 100 σαρώσεων,
14 ακολουθίες): cloister 115 → 42 mm (ICP 54), quad_easy 144 → 65, keble 78 → 64 (ICP 73)· stairs 86 → 126 (αντιγράφει την κλίμακα 0.81 του ICP εκεί).
**Συμπέρασμα ⏳:** η κοντή μετατόπιση είναι κυρίως φαινόμενο Ouster: κοντινές αντιστοιχίσεις + προσαρμογή με μετατόπιση στο μέγεθος των κατωφλίων. Η μεγαλύτερη βάση τη διορθώνει
χωρίς ρύθμιση και χωρίς τον ICP· το #090 την απέρριψε με το σφάλμα ανά σάρωση, που (όπως #092) δεν είναι το μέγεθος της τροχιάς. Επόμενο: πλήρη runs (`multi_baseline`),
και η στροφή από δύο σαρώσεις + μετατόπιση από k−2 ως παραλλαγή. Ανοιχτό: η κλίμακα του ICP ~0.8 σε 01_short / dynamic_spinning / math_medium / stairs — έλεγχος του GT της σάρωσης.

---

### 2026-10-03 — #129 Ανιχνευτής αποτυχίας του ICP από τη συνεπή απόκλιση από την εικόνα: βρίσκει το stairs, βλάπτει αλλού

**Σχετικά:** #128 · `scripts/analyse_icp_failure_detector.py` · `kiss_runs/detect129.txt`

Ανά σάρωση d_k = R_{k−1}(t_ICP − t_εικ.), σε παράθυρο 10: |Σd|, συνέπεια ρ = |Σd| / Σ|d|, και |Σd| / διαδρομή. Σημαία → μετατόπιση της εικόνας. 14 × 4, σφάλμα 10 σαρώσεων (mm):
stairs ICP 200 → **97** («> 20 % της διαδρομής»: 84 % σημαία, 93 % σωστά· όριο από το GT 89)· αλλού 0…+36 % (bodleian 77 → 106, dynamic_spinning 89 → 121, math_medium 115 → 134)·
με ρ > 0.4 μαζί: stairs 155, αλλού 0…+14 %. Σχετικό κατώφλι (× διάμεσος της ακολουθίας): στο stairs σχεδόν ποτέ (ο ICP αποτυγχάνει σε όλο). **Ακόμη και η επιλογή από το GT
χειρότερη από τον ICP σε 11 / 14.**
**Συμπέρασμα ⏳:** η εναλλαγή ανά σάρωση σπάει την αλληλοαναίρεση των σφαλμάτων του ICP και προσθέτει τη συστηματική απόκλιση της εικόνας (#130). Η εικόνα ως αντικατάσταση
μόνο σε ολική αποτυχία· αλλιώς ως αρχή / deskew (όπως σήμερα) ή ως ήπιος περιορισμός μέσα στον ICP (Β.9 joint).

---

### 2026-10-03 — #128 Η εικόνα αντί του ICP στην «αδύναμη» κατεύθυνση της γεωμετρίας (#023 ξανά, ιδέα Μ.Τ.): όχι — η γεωμετρία δεν βρίσκει πού αποτυγχάνει ο ICP

**Σχετικά:** #023 · ιδέα Μ.Τ. 3/10 («καλή αρχή της εικόνας απορροφάται σε κακό ICP») · `scripts/analyse_degenerate_image.py` (H = mean(n nᵀ), voxel 0.25 m, 20 γείτονες· cache
`degeneracy_H.npz` δίπλα στο `oracle_motion.npz`) · `kiss_runs/degen128_*.txt`

Θέση και στροφή k στο τέλος της σάρωσης k → βήμα ICP P_{k−1}⁻¹P_k έναντι της κίνησης του GT. Σφάλμα μετατόπισης 10 σαρώσεων στα παράθυρα με το ασθενέστερο 25 % (mm, μ. ό. 4 σπόρων),
ICP / σύντηξη στην αδύναμη κατεύθυνση / μόνο εικόνα: stairs 232 / 211 / **83**· 01_short 104 / 99 / **86**· keble_03 69 / 60 / **50**· quad_hard 67 / **61** / 76· church_03 34 / 33 / 33·
και ο ICP καλύτερος σε 9: church_02 40 / 47 / 48, math_easy 60 / 61 / 90, math_medium 106 / 109 / 133, quad_easy 57 / 58 / 151, underground_medium 48 / 58 / 97, underground_hard 61 / 73 /
120, dynamic_spinning 138 / 153 / 203, cloister 57 / 111 / 127, bodleian 66 / 143 / 139. Χαμηλή ιδιοτιμή και σε cloister / bodleian, όπου η εικόνα διπλασιάζει το σφάλμα.
Το σφάλμα ανά σάρωση του ICP (28–100 mm) φουσκώνει από τον θόρυβο της θέσης, που αναιρείται στο επόμενο βήμα — το μέγεθος σύγκρισης είναι των 10 σαρώσεων.
**Συμπέρασμα ⏳:** η εικόνα κερδίζει όπου ο ICP αποτυγχάνει (stairs: ύψος, όχι γεωμετρική εκφύλιση — όπως #023), όχι όπου η γεωμετρία είναι αδύναμη. Συνέχεια: #129.

---

### 2026-10-03 — #127 Εικόνα μόνο όπου η κίνηση είναι ακανόνιστη, αλλιώς σταθερή ταχύτητα (ιδέα Μ.Τ.): σε φορητή μονάδα όχι· στο όχημα σχεδόν ισοπαλία

**Σχετικά:** #084 / #085 (όχημα), #092 · ιδέα Μ.Τ. 3/10 · `scripts/analyse_gated_deskew.py` · `kiss_runs/gate127.txt`, `gate127_boreas.txt` · launcher `kiss_runs/boreas127_launch.sh`
(Boreas 2021-01-26-11-22, 3000 σαρώσεις, base127_s0–3, νέο `oracle_motion.npz` από το GT ανά σάρωση)

Σταθερή ταχύτητα C_k = P_{k−2}⁻¹P_{k−1}· διακόπτης: εικόνα όταν η γωνία C_k⁻¹M_k > c × διάμεσος των 50 προηγούμενων (c = 1.5 / 2 / 3, χωρίς απόλυτο κατώφλι). 14 ακολουθίες × 4:
στροφή ανά σάρωση εικόνα / CV ανά τρίτο της αληθινής αλλαγής κίνησης — quad_easy 0.57 / 0.96 · 0.57 / 1.59 · 0.63 / 2.86°· underground_hard 0.43 / 1.42 · 0.52 / 2.83 · 0.66 / 5.12· church_03
0.36 / 0.88 · 0.49 / 1.82 · 0.56 / 3.14· μόνη εξαίρεση 01_short ακανόνιστο (1.93 / 1.71). Η επιλογή ανά σάρωση από το GT διαλέγει εικόνα 64–98 %· ο διακόπτης χειροτερεύει ανά σάρωση
και σε 10 σαρώσεις (το CV σε 10 σαρώσεις «τηλεσκοπεί» — ψευδώς καλό). **Boreas (όχημα, raw):** ήρεμο 0.045 / 0.035°, μέσο 0.17 / 0.15, ακανόνιστο 0.29 / 0.34· σύνολο 0.137 / 0.147°·
ο διακόπτης 0.148–0.154°, το όριο από το GT 0.103°.
**Συμπέρασμα ⏳:** σε φορητή μονάδα η εικόνα πάντα· στο όχημα η διαφορά είναι μικρή και ο απλός διακόπτης δεν βρίσκει τις σωστές σαρώσεις. Το KITTI δεν ελέγχεται (διορθωμένες σαρώσεις, #085).

---

### 2026-10-03 — #126 Όλες οι κανονικοποιήσεις του intensity σε NTU eee_01 (πλήρη runs) και Hilti: καμία καλύτερη από την προεπιλογή

**Σχετικά:** #118–#125, #121 · ζήτημα Μ.Τ. 3/10 · `kiss_runs/rnorm126_launch.sh` (worktree `deskew_in_icp`), `scripts/score_motion_gyro.py` (στροφή έναντι του γυροσκοπίου του Ouster,
εξωτερική βαθμονόμηση και χρονική απόκλιση από τις θέσεις του ICP) · `kiss_runs/rnorm126_gyro_office.txt`

**eee_01, ATE NTU (m, 4 σπόροι):** προεπιλογή 1.483 ± 0.149· I·r 1.648 (+11 %)· rangefit 1.680 (+13 %)· log(I·r²) 1.737 (+17 %)· ομαλή 1.797 (+21 %)· log I 1.819 (+23 %)·
log(I·r^k) 1.845 (+24 %)· (I·r²: #121, σε εξέλιξη). Offline στο eee_01 άκυρο: οι πρώτες 600 σαρώσεις ακίνητες (0.02° / σάρωση).
**Hilti, APE (m, 4 σπόροι), προεπιλογή / ομαλή:** Construction_Site_1 0.042 / 0.043· Office_Mitte_1 0.185 ± 0.054 / 0.272 ± 0.065· IC_Office_1 0.073 / 0.077· LAB_Survey_2 0.0351 / 0.0354·
Basement_1 0.053 / 0.058· UZH 0.576 / 0.577. **Γυροσκόπιο, Office_Mitte_1, 600 σαρώσεις** (στροφή 10 σαρώσεων): προεπιλογή 2.84°, I·r 2.67, I·r² 2.71, ομαλή 2.87, rangefit 2.51, log 2.47,
log(I·r²) 2.39, log(I·r^k) 2.42.
**Συμπέρασμα ⏳:** στο NTU καμία κανονικοποίηση δεν βοηθά· στο Hilti το γυροσκόπιο δείχνει μικρό κέρδος στροφής με log / rangefit, ενώ τα πλήρη runs της ομαλής χειρότερα. Μένει η
προεπιλογή.

---

### 2026-10-03 — #125 log(1 + I · r^k) με τον εκθέτη της σάρωσης: μετατόπιση καλύτερη, στροφή χειρότερη παντού — όχι

**Σχετικά:** #123, #124 · ζήτημα Μ.Τ. 3/10 («και log με το rangefit») · κλάδος `deskew_in_icp`: `intensity_normalisation: logrf` · `kiss_runs/rnorm125.txt`

600 σαρώσεις, σπόρος 0. Σε 10 σαρώσεις έναντι της προεπιλογής, στροφή / μετατόπιση: quad_easy **+29 / −21 %** (log I −11 / −33, rangefit −29 / −34)· cloister **+40 / −24** (+16 / −17, −16 / −8)·
underground_hard **+21 / −23** (−11 / −22, −10 / −15)· christ-church-03 **+23 / +26** (+18 / +15, +14 / +5). Inliers Office_Mitte_1 154 → 250, eee_01 207 → 196.
**Συμπέρασμα ⏳:** όπως το log(I · r²) (#123): η συμπίεση μετά από διόρθωση απόστασης σβήνει τη λεπτή υφή που δίνει τη στροφή· η μετατόπιση βελτιώνεται αλλά την τροχιά τη δίνει
ο ICP (#039), ενώ η εικόνα μετρά κυρίως στη στροφή (#086). Όχι.

---

### 2026-10-03 — #124 Διόρθωση του #118: κανένας περιστρεφόμενος αισθητήρας μας δεν δίνει intensity αντισταθμισμένο ως προς την απόσταση — μέτρηση ανά αισθητήρα· εκθέτης πτώσης από τη σάρωση

**Σχετικά:** #118 (διορθώνεται: «το Hesai δίνει ήδη ανακλαστικότητα» — λάθος), #075 (πεδίο `reflectivity` των Ouster: απορρίφθηκε) · ζήτημα Μ.Τ. 3/10 · κλάδος `deskew_in_icp`:
`intensity_normalisation: rangefit` (`fit_falloff`) · `kiss_runs/rnorm124_launch.sh`

**Μέτρηση** (κάθε 50ή σάρωση, διάμεσος του πεδίου που χρησιμοποιεί ο reader μας ανά απόσταση 1–2 / 2–4 / 4–8 / 8–16 / 16–32 m): Hesai QT64 `intensity` 250 / 241 / 187 / 67 / 26
(≈ r^−1.5 πέρα από 4 m, **κορεσμός 255** κοντά)· Ouster OS0-128 `intensity` 64 / 251 / 280 / 78 / 19 (≈ r^−2 πέρα από 8 m)· OS1-64 — / 367 / 426 / 483 / 194· Hilti OS0-64 22 / 80 / 65 / 22 / 8·
NTU OS1-16 51 / 89 / 188 / 183 / 86 — **όλοι ακατέργαστο σήμα**· Livox Mid-360 14 / 59 / 32 / 24 / 19 (σχεδόν σταθερό: αντισταθμισμένο)· Avia 1 / 32 / 49 / 1 (ασαφές, μικρός χώρος).
Κάθε bag των Ouster έχει και πεδίο `reflectivity` (του firmware) — #075: χειρότερο.
**Τι αλλάζει στο #118:** το I · r² βλάπτει στο Hesai επειδή η πτώση του είναι ηπιότερη (r^−1.5) και το κοντινό πεδίο κορεσμένο, όχι επειδή είναι ήδη αντισταθμισμένο. Γενικός
κανόνας: ο εκθέτης από τη μετρούμενη πτώση, όχι από τη μάρκα → `rangefit`: I · r^k, k από την κλίση log–log της ίδιας της σάρωσης από το φωτεινότερο κελί και πέρα (0–3).
**Offline έλεγχος σε εξέλιξη** (όπως #118 / #123).

**Αποτέλεσμα (προσθήκη 3/10, `kiss_runs/rnorm124.txt`)** — στροφή / μετατόπιση σε 10 σαρώσεις έναντι της προεπιλογής: quad_easy r² −32 / −38 %, ομαλή −17 / −12, **rangefit −29 / −34**·
cloister −20 / −13, −14 / +1, **−16 / −8**· underground_hard −12 / −18, −9 / −2, **−10 / −15**· christ-church-03 (Hesai) +29 / +57, +2 / +10, **+14 / +5**. Inliers Office_Mitte_1 154 → 204 / 302 /
243, eee_01 207 → 176 / 326 / 227. **Συμπέρασμα ⏳:** ο εκθέτης από τη σάρωση κρατά ~90 % του κέρδους του r² στο Ouster χωρίς επιλογή εκθέτη και μισεί τη βλάβη στο Hesai, χωρίς να τη
μηδενίζει (ο κορεσμός κοντά περιορίζει κάθε πολλαπλασιαστική διόρθωση)· η ομαλή καμπύλη μένει η ουδέτερη στο Hesai με τα περισσότερα inliers. Κανένας κανόνας δεν βοηθά παντού.

---

### 2026-10-03 — #123 Κι άλλες κανονικοποιήσεις του intensity offline (I · r, log I, log(I · r²)): καμία καλύτερη από τις I · r² (Ouster) / ομαλή (ασφαλής)

**Σχετικά:** #118 · ΑΠΟΦΑΣΗ Μ.Τ. 3/10 («δοκίμασε κι άλλες, π.χ. I · r ή log») · κλάδος `deskew_in_icp`: `intensity_normalisation: range1 | log | logr2` · `kiss_runs/rnorm123.txt`

600 σαρώσεις, σπόρος 0. Στροφή σε 10 σαρώσεις έναντι της προεπιλογής (%): quad_easy I·r −23, I·r² **−32**, log −11, log(I·r²) +42, ομαλή −17· cloister −14 / **−20** / +16 / +48 / −14·
underground_hard −5 / **−12** / −11 / +27 / −9· christ-church-03 (Hesai) +9 / +29 / +18 / +22 / **+2**. Μετατόπιση: το log βοηθά σε cloister (103 → 85 mm) και underground_hard
(98 → 76). Inliers Office_Mitte_1 154 → 256 / 204 / 284 / 235 / **302**, eee_01 207 → 252 / 176 / 273 / 159 / **326**.
**Συμπέρασμα ⏳:** το I · r² μένει το καλύτερο στο Ouster, η ομαλή καμπύλη η ασφαλέστερη (Hesai σχεδόν ίσο, τα περισσότερα inliers)· το log(I · r²) χειρότερο παντού. Στο Hesai καμία
κανονικοποίηση δεν βοηθά. Τα πλήρη runs του #121 ελέγχουν ήδη τις δύο καλύτερες.

---

### 2026-10-03 — #122 Αυτο-κίνηση της ροζέτας στην τροχιά του Avia: καλύτερη διάμεση στροφή από τον KISS, αλλά APE ~2× χειρότερο· ο κανόνας εφεδρείας μισεί την ουρά offline

**Σχετικά:** #116 (ιδέα Μ.Τ.) · `scripts/rosette_selfmotion.py --out` (κινήσεις, περιορισμοί, υπόλοιπο ανά σάρωση) · αρχεία `kiss_runs_ssd/tiers/indoor02/motions/self{,cvfb,rot}_avia_gap0.25.npz`
· launchers `kiss_runs/self122{,b}_launch.sh` · `kiss_runs/self122{,b}_eval.txt`

**Ουρά (offline, έναντι γυροσκοπίου):** το σφάλμα πέφτει με τους περιορισμούς (διάμεσος ανά τεταρτημόριο 1.54 / 0.53 / 0.34 / 0.26°, συσχέτιση −0.57). Κανόνας σχετικός (χωρίς ρύθμιση
ανά αισθητήρα): **λιγότεροι από το μισό του διάμεσου περιορισμών της ακολουθίας → η προηγούμενη αυτο-κίνηση** (σταθερή ταχύτητα, 27 % των σαρώσεων): p50 0.41 → **0.33°**, p90 2.00 →
**1.01°** — καλύτερο από τη στροφή ανά σάρωση του KISS (0.70 / 1.90°).

**Τροχιά** (voxel 0.1 m, προσαρμοστικό σ, δύο αρχές, 1 σπόρος· APE [m], RPE 1 s στροφή p50 / RMSE): KISS **0.415, 0.63 / 1.56°**· αυτο-κίνηση ακατέργαστη 0.925, 0.64 / 3.63· με τον κανόνα
0.844, **0.52** / 2.20· deskew σταθ. ταχύτητας + αυτο-κίνηση ως αρχή 0.827, 0.58 / 2.15· ακατέργαστη + δεύτερη αρχή KISS 1.024, 0.63 / 4.01· μόνο η στροφή της αυτο-κίνησης (μετατόπιση
σταθερής ταχύτητας του KISS) 0.975 / ως αρχή 0.878, 0.53 / 2.4–2.6· oracle γυροσκοπίου 0.437, 0.51 / 1.14.
**Συμπέρασμα ⏳:** η αυτο-κίνηση δίνει τη διάμεση στροφή ανά δευτερόλεπτο κοντά στο oracle (0.52 έναντι 0.51, KISS 0.63), αλλά οι σαρώσεις που ξεφεύγουν από τον κανόνα κρατούν
RMSE και APE ~2× χειρότερα από τον KISS. Όχι η μετατόπιση (μόνο στροφή: ίδιο). Ανοιχτό: καλύτερος ανιχνευτής κακών σαρώσεων (π.χ. συμφωνία με την κίνηση του ICP / της προηγούμενης)
ή στάθμιση αντί για αντικατάσταση· μόνο Avia (πραγματική ροζέτα), 1 ακολουθία, 1 σπόρος.

---

### 2026-10-03 — #120 Αποτέλεσμα του #109 (10 ακολουθίες × 4): ο έλεγχος από την εικόνα απόστασης αφαιρεί την κατάρρευση του #108 αλλά δεν κερδίζει σημαντικά· «πάντα και οι δύο» χειρότερη τοπικά

**Σχετικά:** #108, #109, #119 · βραχίονες `valid109`, `always109` · `kiss_runs/results_official_allarms_109.{md,csv}`, `kiss_runs/comparisons_109.txt`

stairs, math_easy, christ-church-03, quad_hard, underground_hard, observatory-01, dynamic_spinning, Office_Mitte_1, UZH, eee_01 × 4 σπόροι, έναντι της προεπιλογής (`base092`):

| | RPE 1 m στροφή | RPE 1 m μετατόπιση | APE / σκορ |
|---|---|---|---|
| δεύτερη αρχή = KISS (#108) | 1–6, +0.5 % | 2–5, +0.7 % | 4–4, 0 % |
| + έλεγχος από απόσταση (`valid109`) | 4–3, −0.8 % | 5–2, −1.3 % | **6–4, −2.0 %** (p 0.92) |
| πάντα και οι δύο (`always109`) | **0–7, +7.2 % (p 0.016)** | 2–5, +2.0 % | 4–6, +1.6 % |

APE ανά ακολουθία (προεπιλογή / #108 / valid / always): underground_hard 0.091 / **3.16** / 0.088 / 0.093· Office_Mitte_1 0.185 / 0.278 / **0.176 ± 0.026** / 0.187· dynamic_spinning 0.171 /
0.124 / 0.215 ± 0.205 / **0.109 ± 0.003**· quad_hard 0.233 / 0.214 / 0.206 / **0.174**· eee_01 1.48 / 1.35 / 1.58 / 1.90· stairs 1.92 / 2.01 / 1.99 / 2.29.
**Συμπέρασμα ⏳:** ο έλεγχος από την εικόνα απόστασης είναι η ασφαλέστερη μορφή της ιδέας (χωρίς την κατάρρευση του underground_hard, σφιχτότερο Office_Mitte_1) αλλά τα κέρδη του δεν
είναι σημαντικά· «πάντα και οι δύο» δίνει τα καλύτερα dynamic_spinning / quad_hard με κόστος τοπικής στροφής παντού (+7 %) και 2× ICP. Καμία ως προεπιλογή· η ιδέα «δάπεδο ανθεκτικότητας»
δεν βελτιώνει ουσιαστικά τους περιστρεφόμενους.

---

### 2026-10-03 — #116 Κίνηση μέσα στη σάρωση από την ίδια τη σάρωση (αυτο-επικάλυψη της ροζέτας): Avia διάμεσος 0.41° (καλύτερο από KISS 0.70°) με βαριά ουρά· Horizon χειρότερο

**Σχετικά:** **Ιδέα Μ.Τ. 3/10** (το μοτίβο σάρωσης των solid-state, «ροζέτα»)· εκτιμητής: προσαρμογή αυτο-συνέπειας · κλάδος `deskew_in_icp`, `scripts/rosette_selfmotion.py` (`cf848f1`) ·
`kiss_runs/rosette116.txt`

Η ροζέτα ξαναπερνά τις ίδιες κατευθύνσεις μέσα σε μία σάρωση (κάθε πέταλο από το κέντρο) — ένας περιστρεφόμενος ποτέ. Με κίνηση σταθερής ταχύτητας (ω, v) κάθε σημείο στο πλαίσιο της
αρχής της σάρωσης, x_i = Exp(s_i ω) p_i + s_i v· η σωστή κίνηση κάνει τη σάρωση συνεπή: σημεία με χρονική απόσταση ≥ gap πάνω στις ίδιες επιφάνειες (σημείο-προς-επίπεδο, κάθετη
από 8 γείτονες, Geman–McClure, Gauss–Newton από μηδέν, 10 επαναλήψεις)· μόνο η διαφορά χρόνων s_i − s_k μπαίνει. Βαθμολόγηση έναντι του γυροσκοπίου του Livox (423 σαρώσεις).

| | p50 / p90 [°] | συσχέτιση | αποτυχίες |
|---|---|---|---|
| Avia, gap 0.25 | **0.412 / 2.00** | 0.72 | 0 |
| Avia, gap 0.25 + α | 0.396 / 2.20 | 0.75 | 0 |
| Avia, gap 0.5 | 0.485 / 2.58 | 0.62 | 0 |
| Horizon, gap 0.25 | 0.966 / 2.54 | 0.62 | 0 |
| Horizon, gap 0.5 | 0.977 / 3.64 | 0.43 | 26 |
| (KISS ανά σάρωση, #100) | Avia 0.70 / 1.90 · Horizon 0.79 / 1.86 | | |

(Στις πρώτες 60 σαρώσεις Avia 0.35 / 0.60 — αισιόδοξο.) **Συμπέρασμα ⏳:** στο Avia (πραγματική ροζέτα) ο διάμεσος είναι ο καλύτερος ρεαλιστικός που έχουμε (0.41 έναντι KISS 0.70,
εικόνα 0.67) χωρίς καμία αποτυχία, αλλά η ουρά είναι βαριά (p90 2.0°)· στο Horizon (6 γραμμές που σαρώνουν πλάγια — λίγη επανεπίσκεψη, μισοί περιορισμοί) χειρότερο από τον KISS.
Επόμενο: η αυτο-κίνηση ως deskew του Avia στην τροχιά (αρχείο κινήσεων), και περιορισμός της ουράς (π.χ. σταθερή ταχύτητα όπου λίγοι περιορισμοί / μεγάλο υπόλοιπο).

---

### 2026-10-03 — #115 Β.9 (deskew μέσα στην καταχώριση): πρωτότυπο — στα Livox του TIERS ~2× χειρότερο από τον KISS, ανεξάρτητα από την αρχή (εικόνα / σταθερή ταχύτητα)

**Σχετικά:** open_tasks Β.9 · ΑΠΟΦΑΣΗ Μ.Τ. 3/10: νέος κλάδος για το Β.9 → **`deskew_in_icp`** (από `after_091`), σε git worktree `/home/photogrammetry/Kiss_SLAM-b9` (τα runs του
κύριου checkout συνεχίζουν)· εκτέλεση με `scripts/worktree_run.py` · `kiss_slam/ct_registration.py`, `--ct=image|cv|joint`, `--ct-lambda`, `--ct-max-iter` · `kiss_runs/ct115*_eval.txt`

Δύο θέσεις ανά σάρωση (αρχή / τέλος), κάθε σημείο στον χρόνο του (slerp / γραμμική), ο χάρτης, ο πυρήνας και το σ του KISS, οι περιορισμοί θέσης και ταχύτητας του ECTLO. Συνθετικός
έλεγχος: ο λύτης βρίσκει γνωστή κίνηση ακριβώς. **Σφάλμα που διορθώθηκε:** ο γείτονας αναζητούνταν ως 3σ (ως 6 m) αντί για τα 27 voxels γύρω από το σημείο του KISS
(min(3σ, 2√3 voxel)) — πρώτα runs APE 2–3.6 m. Μετά τη διόρθωση (TIERS, voxel 0.1 m, 1 σπόρος, APE / RMSE στροφής 1 s): KISS Avia 0.415 / 1.56°, Horizon 0.611 / 2.09°· Β.9 αρχή
σταθ. ταχύτητας λ = 0.1: **0.882 / 2.30°, 0.988 / 3.90°**· αρχή εικόνας 0.933 / 2.68°, 0.858 / 4.53°· λ = 0: 1.84 m / Horizon αποκλίνει (11.5 m)· λ = 0.01: 1.16 / 1.77· λ = 1: 1.24 / 1.19·
λ = 10: 2.9 / 2.9 m. **Συμπέρασμα ⏳:** οι περιορισμοί κρατούν την καταχώριση (χωρίς αυτούς αποκλίνει)· το καλύτερο λ = 0.1 (του ECTLO) αλλά το πρωτότυπο μένει ~2× χειρότερο από τον
KISS — όχι θέμα ρύθμισης· πιθανώς η απλή Gauss–Newton σε Python (σταθερές επαναλήψεις, πρώτης τάξης Ιακωβιανές, KD-tree) έναντι της ICP του KISS. Η εκδοχή «joint» (αντιστοιχίσεις
εικόνας στο κόστος) και οι περιστρεφόμενοι (#117) σταμάτησαν· να ξαναγίνουν όταν η βάση CT φτάσει τον KISS.

---

### 2026-10-03 — #119 Αποτέλεσμα του #108 στις 27 × 4: η δεύτερη αρχή = KISS δεν βοηθά στους περιστρεφόμενους· τοπικά λίγο χειρότερη, και μία κατάρρευση (underground_hard)

**Σχετικά:** #108 · βραχίονας `kissfloor108` έναντι `base092` · `kiss_runs/results_official_allarms_108.{md,csv}`, `kiss_runs/comparisons_108.txt`

Όλες οι 27: RPE 1 m στροφή χειρότερη σε 16 / 18 (+0.5 %, p = 0.006), μετατόπιση 12 / 17 (+0.6 %, p = 0.022), RPE 1 s στροφή 15 / 18 (p = 0.003)· APE / σκορ 12–13 (ίσο). Κέρδη:
dynamic_spinning 0.171 ± 0.091 → **0.124 ± 0.010**, eee_01 1.48 → 1.35, bodleian 0.65 → 0.60, Basement_1 0.053 → 0.045. Απώλειες: **underground_hard 0.091 ± 0.001 → 3.16 ± 2.13 m**
(σπόροι αποκλίνουν), Office_Mitte_1 0.185 → 0.278, christ-church-02 0.160 → 0.198.
**Συμπέρασμα ⏳:** στους περιστρεφόμενους το deskew της εικόνας είναι συνήθως σωστό, οπότε ο ανταγωνισμός με τον KISS με κριτήριο το ταίριασμα στον χάρτη προσθέτει λάθος επιλογές
συχνότερα από ό,τι πιάνει κακές κινήσεις — **όχι ως προεπιλογή**. Στα Livox (#108, #114) κόβει μέρος της βλάβης χωρίς να φτάνει τον KISS.

---

### 2026-10-03 — #118 Κανονικοποίηση του intensity ως προς την απόσταση, ξανά: I · r² βοηθά στο Ouster (offline −12…−32 %), βλάπτει στο Hesai· η ομαλή καμπύλη ασφαλής, ×1.6–2 inliers στα Hilti / NTU

**Σχετικά:** #025 (πίνακας ανά κελί απόστασης × γωνίας, παλιά pipeline: χειρότερο), ΑΠΟΦΑΣΗ Μ.Τ. 3/10 («έλεγξε ξανά την κανονικοποίηση απόστασης») · κλάδος `deskew_in_icp`
(`afa4593`): `image_deskew.intensity_normalisation: range2 | range_smooth` · launcher `kiss_runs/rnorm118_launch.sh` · `kiss_runs/rnorm118.txt`

`range2` = I · r²· `range_smooth` = I / f(r), f ο διάμεσος της ίδιας της σάρωσης ανά λογαριθμικό κελί απόστασης, ομαλός (χωρίς τα σκαλοπάτια του #025)· και τα δύο σε p99 = 255.
600 σαρώσεις, σπόρος 0, προεπιλογή κατά τα άλλα. Στροφή / μετατόπιση σε 10 σαρώσεις (p50): quad_easy 2.03° / 128 mm → **r² 1.38 / 79** → ομαλή 1.68 / 113· cloister 1.02 / 103 →
**0.82 / 89** → 0.87 / 104· underground_hard 2.10 / 98 → **1.85 / 80** → 1.91 / 96· christ-church-03 (Hesai) 1.41 / 30 → **1.81 / 48 (χειρότερο)** → 1.43 / 33. Inliers: Office_Mitte_1
154 → 204 → **302**, eee_01 207 → 176 → **326**· αποτυχίες ίδιες.
**Συμπέρασμα ⏳:** εξαρτάται από το τι αναφέρει ο αισθητήρας — Ouster: σήμα που πέφτει με την απόσταση (r² διορθώνει πραγματικό φαινόμενο)· Hesai: ανακλαστικότητα ήδη
αντισταθμισμένη (r² υπερδιορθώνει). Η ομαλή καμπύλη της σάρωσης είναι ασφαλής παντού και αυξάνει τα inliers, με μικρότερο κέρδος. Offline μόνο — το #093 έδειξε ότι κέρδη offline
στο Ouster 128 δεν έφτασαν την τροχιά· χρειάζονται πλήρη runs.

---

### 2026-10-03 — #111 Πανόραμα με μόνο τις στήλες του αισθητήρα (μεγέθυνση 8 όπως πριν): Hesai τοπικά καλύτερο 6–0 χωρίς τη βλάβη ύψους· APE μικτό, Hilti 1–5

**Σχετικά:** #096 (ο πλήρης κανόνας) · `--panorama-width=auto` · launcher `kiss_runs/autow111_launch.sh` · βραχίονας `autow111` · `kiss_runs/results_official_allarms_111.{md,csv}`

Μόνο εκεί όπου οι στήλες του αισθητήρα διαφέρουν από 1024: Hesai (600) και Hilti (2048), 12 × 4. **Hesai:** RPE 1 m στροφή 6–0, −5.4 % (p = 0.031), μετατόπιση 6–0, −3.5 % (0.031) — όπως ο
πλήρης κανόνας (−5.7 / −5.2 %) — **και z RMSE ίσο** (+0.8 %, 3–3· πλήρης κανόνας +20.9 %). APE 2–4 (+3.9 %): christ-church-02 0.160 ± 0.029 → **0.512 ± 0.053** (σταθερό σε 4 σπόρους),
observatory 0.072 → 0.065 (πλήρης κανόνας 0.234), bodleian 0.649 → 0.607. **Hilti:** σκορ 1–5 (+2.0 %, p 0.31), Office_Mitte_1 0.185 → 0.239, Basement_1 0.053 → 0.041.
**Συμπέρασμα ⏳:** οι στήλες του αισθητήρα είναι η αιτία της τοπικής βελτίωσης στο Hesai και αφαιρούν τη βλάβη ύψους του πλήρους κανόνα, αλλά το christ-church-02 χειροτερεύει
σταθερά σε APE και το Hilti δεν κερδίζει. Ούτε αυτό ως προεπιλογή χωρίς εξήγηση του christ-church-02 (απόφαση Μ.Τ.).

---

### 2026-10-03 — #114 Livox (TIERS): «πάντα και οι δύο αρχές» — λίγο καλύτερο από το #108, ακόμη όχι στο επίπεδο του KISS

**Σχετικά:** #108, #109 · launcher `kiss_runs/livox109_launch.sh` · runs `kiss_runs_ssd/tiers/indoor02/img01al_*` · `kiss_runs/livox109_eval.txt`

Voxel 0.1 m, κινήσεις εικόνας του #095–#100, `--two-start-kiss --two-start-always` (κάθε σάρωση από την εικόνα ΚΑΙ από τον KISS, κρατείται το καλύτερο ταίριασμα στον χάρτη), 1 σπόρος.
APE [m] (RMSE στροφής 1 s): Avia K2 δικές της 0.530 (2.53°) [#108: 0.545 (2.97°)], K2 ICP **0.464 (1.76°)** [0.473 (2.39°)]· Horizon μία σάρωση 0.607 (2.23°) [0.603 (2.33°)], K2 δικές της
0.657 (3.12°) [0.631 (2.67°)], K2 ICP 1.044 (2.83°) [1.147 (3.08°)]· KISS μόνος Avia 0.415 (1.56°), Horizon 0.611 (2.09°).
**Συμπέρασμα ⏳:** η επιλογή ανά σάρωση με το ταίριασμα στον χάρτη δεν ξεχωρίζει πλήρως το κακό deskew της εικόνας — οι δύο αρχές δίνουν σχεδόν ίδιο ταίριασμα· στα Livox ο KISS μόνος μένει
ο καλύτερος.

---

### 2026-10-03 — #113 Mid-360 (συνέχεια του #106): ο KISS αποτυγχάνει σε κάθε voxel που δοκιμάστηκε — η σύγκριση με την εικόνα εκεί δεν κρίνεται

**Σχετικά:** #105, #106, #110 · `kiss_runs/hpl106_eval.txt`

KISS (η ρύθμιση του paper με voxel 0.25 m, `configs/livox_voxel025.yaml`), outdoor_hard_01, επίσημο evo: με deskew APE **51.7 m**, μήκος 2150 m (GT 999), RPE 1 s 10.1° (4 σπόροι
ταυτόσημοι)· χωρίς deskew APE 463 m. Μαζί με 1 m (APE 76 m, #105) και auto 0.93 m (90 m, #110), και 0.1 m που τερματίστηκε από μνήμη (#106): **ο KISS μόνο-LiDAR δεν ακολουθεί αυτή
την ακολουθία** (γωνιακή ταχύτητα ως 504 °/s). Το deskew βοηθά πολύ (52 έναντι 463 m). **Συμπέρασμα ⏳:** στο Mid-360 δεν υπάρχει λειτουργική βάση για να κριθεί η εικόνα· ένα
ακριβές deskew θα ήταν η προϋπόθεση (το oracle της εικόνας έχει μισή ουρά από τη σταθερή ταχύτητα, #105), αλλά η εικόνα αποτυγχάνει στο 77–92 % των σαρώσεων. Ανοιχτό (Β.10).

---

### 2026-10-03 — #112 Αντιστοίχιση σάρωσης σε εικόνα intensity του χάρτη (Β.12α): χειρότερη από τη σάρωση-σε-σάρωση, ακόμη και με τέλειο χάρτη

**Σχετικά:** open_tasks Β.11 / Β.12α (LLOL / UPSLAM) · `scripts/map_pano_check.py` (`dcd277a`) · launcher `kiss_runs/map112_launch.sh` · έξοδος `kiss_runs/map112_*.txt`

#### Μέθοδος
Χάρτης = οι W προηγούμενες σαρώσεις, deskew με την κίνηση του ICP και θέση από τον ICP (run `base092_s0`), στο πλαίσιο της αρχής της σάρωσης k· πλέγμα αζιμούθιο × ύψος με τις στήλες του
αισθητήρα και γραμμές ανά ring (× 8)· ανά pixel η μπροστινή επιφάνεια (≤ 5 % από την πλησιέστερη απόσταση), μέση intensity και θέση. Η τρέχουσα σάρωση ακατέργαστη στο ίδιο πλέγμα·
καθοδηγούμενη αντιστοίχιση σε μηδενική μετατόπιση· προσαρμογή p_χάρτη = T0 · καμπύλη(t_q) · q με την αρχή T0 ελεύθερη (σταθερή στη θέση του ICP: το σφάλμα του ICP περνά στην κίνηση).
Οροφή: χάρτης από το GT και T0 ακριβής. 300 σαρώσεις, σφάλμα στροφής έναντι GT ανά σάρωση και σε 10 σαρώσεις.

#### Αποτέλεσμα (p50, ανά σάρωση / σε 10 σαρώσεις)

| | christ-church-03 | quad_easy |
|---|---|---|
| σάρωση-σε-σάρωση (η μέθοδος) | **0.445 / 1.36°** | **0.514 / 2.17°** |
| χάρτης ICP, 5 / 10 σαρώσεις | 0.681 / 2.41° · 0.689 / 2.85° | 1.366 / 6.79° · 1.389 / 7.15° |
| οροφή: χάρτης GT, T0 ακριβής | 0.432 / 1.66° | 0.839 / 3.83° |

#### Συμπέρασμα ⏳
**Όχι σε περιστρεφόμενους αισθητήρες.** Έναντι στατικού χάρτη η κίνηση μέσα στη σάρωση φαίνεται από μία μόνο σάρωση (η στροφή που αυξάνει με τον χρόνο-αζιμούθιο)· η σάρωση-σε-σάρωση
μοιράζεται μία καμπύλη σε δύο σαρώσεις (διπλό χρονικό εύρος) και τα σφάλματά της αλληλοαναιρούνται εν μέρει σε διαδοχικές σαρώσεις. Ακόμη και η οροφή είναι ίση ανά σάρωση και χειρότερη
σε 10 σαρώσεις· με τον χάρτη του ICP 1.5–2.7× χειρότερη. (Στο quad_easy το GT ανά σάρωση είναι θορυβώδες όπως του math_easy, #094 — η οροφή εκεί υποτιμάται.) Η ιδέα μένει μόνο για
αραιούς αισθητήρες (Livox), όπου η συσσώρευση με σωστή αντιστάθμιση βοηθά (#097).

---

### 2026-10-03 — #110 Voxel από τον αισθητήρα για τη βάση των solid-state (`--voxel=auto`) — υλοποίηση, δοκιμή σε εξέλιξη

**Σχετικά:** #099–#102, #105 · ιδέα από `Improving_Intensity_Driven_LiDAR_SLAM_UPSLAM_LLOL_ECTLO.md` (§7: v(r) = max(v_min, r Δθ)) · `run_ncd.py --voxel=auto` (`45d4c89`) ·
launcher `kiss_runs/vox110_launch.sh`

Ο KISS χρειάστηκε 1 m στο Hesai, 0.1 m στα Livox (#102), και το 1 m του paper απέτυχε στο Mid-360 (#105): ένας κανόνας αντί για ρύθμιση ανά dataset, όπως το πανόραμα του #093.
**Κανόνας:** v = √20 · διάμεση απόσταση · απόσταση σημείων (√(εμβαδόν πεδίου / σημεία)) από την πρώτη σάρωση, όριο 0.05–2 m· ο KISS κρατά ως 20 σημεία ανά voxel, άρα √20.
Τιμές: Avia **0.105 m** (το 0.1 που δούλεψε στο #102), Horizon 0.076, Mid-360 0.93 (έξω, απόσταση 8.7 m)· για αναφορά σε περιστρεφόμενους: christ-church-03 0.25, quad_easy 0.14,
Office_Mitte_1 0.17, eee_01 0.81 m. Μόνο για τη βάση των solid-state (η βάση των περιστρεφόμενων μένει η ρύθμιση του paper).

**Αποτέλεσμα** (`kiss_runs/vox110_eval.txt`, APE [m], RPE 1 s στροφή p50 / RMSE): Avia auto 0.105 m → 0.493, 0.57 / 1.33° (χειροκίνητο 0.1 m: 0.415, 0.63 / 1.56°)· Horizon auto 0.076 m →
1.157, 0.77 / 1.50° (0.1 m: 0.611, 0.86 / 2.09°)· Mid-360 auto 0.93 m → **APE 90 m, αποτυχία** (όπως το 1 m, #105· 2 σπόροι ίδιοι). **Συμπέρασμα ⏳:** ο κανόνας βρίσκει την κλίμακα στα
Livox (στροφή καλύτερη, APE κοντά ή χειρότερο), αλλά στο Mid-360 η αποτυχία δεν είναι θέμα πυκνότητας — η έντονη κίνηση· όχι γενική λύση.

---

### 2026-10-03 — #109 Έλεγχος της κίνησης της εικόνας ανά σάρωση από την εικόνα απόστασης· πάντα και οι δύο αρχές — υλοποίηση, δοκιμή σε εξέλιξη

**Σχετικά:** #108 (το κατώφλι 5° αφήνει σφάλματα 1–3° χωρίς έλεγχο) · ιδέα από το ίδιο κείμενο (§3: ο κλάδος απόστασης για επικύρωση) · επιλογές `range_motion: validate`
(κίνηση απόστασης σε κάθε σάρωση + εφεδρεία), `validate_k` (3), `two_start_always` (`45d4c89`)· `run_ncd.py --range=validate [--validate-k=3] --two-start-always` · launcher
`kiss_runs/valid109_launch.sh` · βραχίονες `valid109`, `always109`

**Κανόνας χωρίς κλίμακα:** αν η στροφή της εικόνας διαφέρει από της εικόνας απόστασης περισσότερο από k × τον κυλιόμενο διάμεσο αυτής της διαφοράς (200 σαρώσεις, ≥ 20),
η σάρωση περνά από την επιλογή των αρχών (εικόνα / απόσταση / KISS, καλύτερο ταίριασμα στον χάρτη) ακόμη κάτω από τις 5°. «Always»: κάθε σάρωση από όλες τις αρχές (~2× ICP).
Δοκιμή ανάπτυξης quad_hard 300 σαρώσεις: 34 σαρώσεις στάλθηκαν από τον έλεγχο, ο KISS κέρδισε 6 / 61. **Δοκιμή:** 8 προβληματικές ακολουθίες (dynamic_spinning, UZH, stairs,
Office_Mitte_1, eee_01, observatory_01, quad_hard, underground_hard) + christ-church-03, math_easy × 4 σπόροι × 2 βραχίονες, έναντι `base092` και `kissfloor108` — σε εξέλιξη.

---

### 2026-10-02 — #108 Δεύτερη αρχή = ο ίδιος ο KISS (εγγύηση ανθεκτικότητας): στα Livox κόβει το μεγαλύτερο μέρος της βλάβης, όχι όλη — 27 × 4 σε εξέλιξη

**Σχετικά:** open_tasks Β.12(β) · `image_deskew.two_start_kiss`, `run_ncd.py --two-start-kiss` (`5344ba7`) · launchers `kiss_runs/{kissfloor108,livox108}_launch.sh` · βραχίονας
`kissfloor108` · `kiss_runs/livox108_eval.txt`

Η αρχή «cv» των δύο αρχών είχε deskew = I· τώρα deskew ΚΑΙ αρχή σταθερής ταχύτητας = ο KISS, και η εφεδρεία όπου αποτυγχάνει η εικόνα επίσης ο KISS. **TIERS Livox (voxel 0.1 m,
1 σπόρος, APE [m]):** Avia εικόνα K2 (δικές της) 1.265 → **0.545**, K2 (ICP) 1.151 → **0.473** (KISS 0.415)· Horizon K2 (δικές της) 0.813 → **0.631**, μία σάρωση 0.799 → **0.603**
(KISS 0.611), K2 (ICP) 0.855 → 1.147 (χειρότερο). RMSE στροφής 1 s ακόμη χειρότερο από τον KISS (2.3–3.1 έναντι 1.6–2.1°). **Γιατί όχι πλήρης εγγύηση:** η δεύτερη αρχή
καταχωρείται μόνο όταν εικόνα και σταθερή ταχύτητα διαφωνούν > 5°· τα σφάλματα της εικόνας στα Livox είναι 1–3° → #109. **27 × 4 σε εξέλιξη.**

---

### 2026-10-02 — #107 Mid-360: μεγέθυνση ×3 / ×4 πριν από την ανίχνευση — μικρό κέρδος, μεγάλο κόστος

**Σχετικά:** #105 · `scripts/mid360_check.py --ds=3,4` · `kiss_runs/mid360_107_K{1,2,3}.txt`

Αποτυχίες (×2 → ×3 → ×4, από 1499): μία σάρωση 1299 → 1238 → 1227· K2 δικές της 1158 → 1208 → 1159· K2 GT 719 → 628 → 615· K3 GT 486 → 447 → 460. Σφάλμα p50 περίπου ίδιο
(K2 δικές της 0.81 → 0.76 → 0.78°). Χρόνος εικόνας 45–196 ms ανά σάρωση (φορτωμένο μηχάνημα). **Πέρα από ×2 δεν βοηθά:** το όριο στο Mid-360 είναι η αραιότητα μιας–δύο
σαρώσεων, όχι η κλίμακα ανίχνευσης → το πανόραμα χάρτη (Β.12α).

---

### 2026-10-02 — #106 Mid-360: KISS με voxel 0.25 / 0.1 m — το 0.1 m τερματίστηκε από έλλειψη μνήμης· σταματάμε στο 0.25 m (ΑΠΟΦΑΣΗ Μ.Τ.)

**Σχετικά:** #105 · launcher `kiss_runs/hpl106_launch.sh` · runs `kiss_runs_ssd/hard_pcl_loc/outdoor_hard_01/kiss{025,01}*`

Οι 5 runs με 0.1 m τερματίστηκαν (signal 9) στη σάρωση ~980 / 5147 (19 %): ~9–10 GB η καθεμία και αυξανόμενη (1 km έξω, λεπτό voxel), πάνω από το όριο των 40 GB της μονάδας μαζί με
άλλες δουλειές. **ΑΠΟΦΑΣΗ Μ.Τ. 3/10: έως 0.25 m** (το 0.1 m χρειάστηκε σε μικρό εσωτερικό χώρο, #102). Τα 0.25 m σε εξέλιξη.

---

### 2026-10-02 — #105 Mid-360 (γρήγορη κίνηση χειρός): ο KISS με voxel 1 m αποτυγχάνει· η εικόνα αποτυγχάνει στο 77–92 % των σαρώσεων, αλλά όπου πετυχαίνει έχει μισή ουρά από τη σταθερή ταχύτητα

**Σχετικά:** #095–#103 · `scripts/mid360_check.py` · launcher `kiss_runs/hpl105_launch.sh` · `kiss_runs/hpl105_eval.txt`, `kiss_runs/mid360_105_K{1,2,3,5}.txt`

**KISS (ρύθμιση paper, voxel 1 m), 4 σπόροι, επίσημο evo:** APE **76 m**, μήκος 33–81 km σε διαδρομή 999 m· χωρίς deskew APE 110 m (ντετερμινιστικό, 4 ίδια). **Εικόνα** (1500 πρώτες
σαρώσεις, έναντι της κίνησης του GT ανά σάρωση, που υπάρχει στο πλαίσιο του LiDAR): στροφή ανά σάρωση p50 1.0°, p90 5.1°, p99 17.8°· τέλεια σταθερή ταχύτητα (από το GT) p50 0.60°,
**p90 4.04°**. Μία σάρωση (pixel 1°, 355 × 59): 87–99 % αποτυχίες· συσσώρευση με δικές της εκτιμήσεις K2 / K3 (ανίχνευση ×2): 77 / 83 % αποτυχίες, 0.81 / 0.93°, p90 2.7 / 3.2°·
με αντιστάθμιση από το GT (oracle) K2 / K5: 48 / 24 % αποτυχίες, 0.71 / 0.63°, p90 1.9 / 2.2°. **Συμπέρασμα ⏳:** στο Mid-360 ο KISS του paper αποτυγχάνει· η εικόνα θα βοηθούσε στην
ουρά (p90 μισό από της σταθερής ταχύτητας) αλλά αποτυγχάνει στις περισσότερες σαρώσεις — αραιότητα.

---

### 2026-10-02 — #104 Νέο solid-state dataset με έντονη κίνηση (Hard Point Cloud Localization, Livox Mid-360) και reader για ROS2 bags χωρίς ορισμούς τύπων

**Σχετικά:** #095–#103 · [Zenodo 10122133](https://zenodo.org/records/10122133) (CC BY 4.0) · `kiss_slam/tools/ros2bags.py`, `run_ncd.py` (`7cfafe4`) · `docs/datasets.md`

`outdoor_hard_01` (a + b, 1.64 + 1.35 GB, MD5 ελεγμένα): Mid-360 (360°, μη επαναλαμβανόμενη), 684 s, 999 m, γωνιακή ταχύτητα p95 73, p99 176, max 504 °/s (TIERS Indoor02: p99 52).
GT στο πλαίσιο του LiDAR, 10 Hz, ίδιο ρολόι (βελτιστοποίηση LiDAR + IMU, όχι ανεξάρτητη). ROS2 sqlite σε δύο φακέλους χωρίς ορισμούς τύπων· `/livox/points` PointCloud2 με χρόνο
ανά σημείο (`t` uint32 ns), intensity, line. Reader `Ros2Bags`: όλοι οι φάκελοι ως μία ακολουθία, τύποι ROS2 Humble, ίδιος εγκαταστάσιμος reader με το `RosbagDataset`· οι χρόνοι
θέσεων ταυτίζονται με του GT.

---

### 2026-10-02 — #103 Livox: deskew σταθερής ταχύτητας + εικόνα μόνο ως αρχή του ICP — ίδιο με τον KISS, όχι χειρότερο· το κέρδος του oracle είναι στο deskew

**Σχετικά:** #102 · ζήτημα Μ.Τ. 2/10 («deskew με σταθερή ταχύτητα και η εικόνα στις σαρώσεις» → επιλογή Μ.Τ.: CV deskew, εικόνα ως αρχή ICP) · νέα επιλογή
`image_deskew.deskew_from: image | cv` (`run_ncd.py --deskew-from=cv`· προεπιλογή image = αμετάβλητο· όπου αποτυγχάνει η εικόνα, ακριβώς ο KISS) · launcher
`kiss_runs/livox103_launch.sh` · runs `kiss_runs_ssd/tiers/indoor02/img01cv_*` · `kiss_runs/livox103_eval.txt`

#### Αποτέλεσμα (voxel 0.1 m, 1 σπόρος, προσαρμοστικό σ, δύο αρχές· APE [m], RPE 1 s στροφή p50 / RMSE [°])

| κινήσεις εικόνας | Avia: CV deskew + εικόνα αρχή | Avia: deskew εικόνας (#102) | Horizon: CV deskew + εικόνα αρχή | Horizon: deskew εικόνας (#102) |
|---|---|---|---|---|
| (KISS, #102) | 0.415, 0.63 / 1.56 | | 0.611, 0.86 / 2.09 | |
| δικές της εκτιμήσεις K2 | 0.532, 0.58 / 1.95 | 1.265, 0.95 / 3.83 | **0.611**, 0.93 / 2.29 | 0.813, 1.09 / 2.87 |
| αντιστάθμιση από ICP K2 | **0.414**, 0.65 / 1.62 | 1.151, 0.78 / 3.51 | 0.644, 0.98 / 2.71 | 0.855, 1.06 / 3.46 |
| μία σάρωση | — | — | 0.946, 0.91 / 2.31 | 0.799, 1.09 / 2.68 |
| oracle γυροσκοπίου K5 | 0.408, 0.58 / 1.16 | 0.437, **0.51 / 1.14** | 0.595, 0.80 / 1.77 | **0.508, 0.67 / 1.38** |

#### Συμπέρασμα ⏳
1. **Με deskew σταθερής ταχύτητας η εικόνα ως αρχή δεν βλάπτει πια** (Avia 0.414 έναντι 0.415, Horizon 0.611 = 0.611) — η βλάβη του #102 ήταν από το deskew της εικόνας —
   **αλλά ούτε βοηθά:** οι ρεαλιστικές κινήσεις φτάνουν μόνο τον KISS (RMSE στροφής ίδιο ή λίγο χειρότερο). Η μία σάρωση στο Horizon χειρότερη (0.95).
2. **Το κέρδος του oracle είναι στο deskew, όχι στην αρχή:** Horizon oracle ως deskew 0.508 m / 1.38° έναντι ως αρχή 0.595 / 1.77°.
3. Στα Livox, σε αυτή την ακολουθία, **η καλύτερη ρύθμιση μένει ο KISS με voxel 0.1 m**· η εικόνα θα βοηθούσε μόνο ως ακριβές deskew (oracle), που οι ρεαλιστικές κινήσεις δεν φτάνουν.

---

### 2026-10-02 — #102 Livox, voxel 0.1 m: ο KISS δουλεύει (APE 0.42 / 0.61 m)· το deskew της εικόνας (ρεαλιστικές κινήσεις) χειροτερεύει την τροχιά, μόνο το oracle βοηθά λίγο

**Σχετικά:** #101 · **ΑΠΟΦΑΣΗ Μ.Τ. 2/10: voxel 0.1 m** · `configs/livox_voxel01{,_nodeskew}.yaml` (η ρύθμιση του paper με μόνο το voxel 0.1 m) · launcher
`kiss_runs/livox102_launch.sh` · runs `kiss_runs_ssd/tiers/indoor02/{kiss01,kiss01nd,img01_*}_s0` · αξιολόγηση `scripts/evaluate_tiers.py` → `kiss_runs/livox102_eval.txt`

#### Αποτέλεσμα (TIERS Indoor02, 1 σπόρος· APE [m] έναντι mocap, RPE 1 s στροφή p50 / RMSE [°] έναντι γυροσκοπίου, μήκος έναντι GT· GT 22.3 m)

| | Avia | Horizon |
|---|---|---|
| **KISS (deskew σταθερής ταχύτητας)** | **0.415**, 0.63 / 1.56, +16.6 % | **0.611**, 0.86 / 2.09, +23.7 % |
| KISS χωρίς deskew | 0.562, 0.58 / 1.88, +16.3 % | 1.221, 0.96 / 2.45, +27.8 % |
| εικόνα, δικές της εκτιμήσεις K2 | 1.265, 0.95 / 3.83, +24.3 % | 0.813, 1.09 / 2.87, +36.1 % |
| εικόνα, αντιστάθμιση από ICP K2 | 1.151, 0.78 / 3.51, +23.6 % | 0.855, 1.06 / 3.46, +41.6 % |
| εικόνα, μία σάρωση | — | 0.799, 1.09 / 2.68, +38.5 % |
| εικόνα, oracle γυροσκοπίου K5 | 0.437, **0.51 / 1.14**, +18.5 % | **0.508, 0.67 / 1.38**, +21.5 % |

Μετατόπιση ρολογιού (από το run KISS) +70 / +50 ms· το APE εξαρτάται πλέον από αυτήν (στο 0.25 m ήταν επίπεδο — οι τροχιές δεν έμοιαζαν με το GT).

#### Συμπέρασμα ⏳
1. **Στα Livox (εσωτερικός χώρος) ο KISS θέλει voxel 0.1 m:** APE 0.42 / 0.61 m, έναντι 2.2–2.4 m με 0.25 m (#101) και αποτυχίας με 1 m (#099). Το deskew σταθερής ταχύτητας
   βοηθά (χωρίς: 0.56 / 1.22 m).
2. **Το deskew από τις ρεαλιστικές κινήσεις της εικόνας χειροτερεύει την τροχιά** (APE ×2–3 στο Avia, +30–40 % στο Horizon, RMSE στροφής ×1.4–2.4).
3. **Ακόμη και το oracle κερδίζει λίγο:** Horizon APE 0.61 → 0.51, RMSE στροφής 2.09 → 1.38°· Avia APE ίσο, RMSE 1.56 → 1.14°. Με λεπτό voxel η σταθερή ταχύτητα είναι ήδη κοντά.
4. **Η καλύτερη ρύθμιση για Avia και Horizon (αυτή η ακολουθία): KISS με voxel 0.1 m και το δικό του deskew.** Η εικόνα όπως είναι δεν προσφέρει στα Livox· ένα σημείο για το
   paper (η μέθοδος αφορά περιστρεφόμενους αισθητήρες) — απόφαση Μ.Τ. Μία ακολουθία, ένας σπόρος, 42 s.

---

### 2026-10-02 — #101 Livox, voxel 0.25 m: όλες οι ρεαλιστικές ρυθμίσεις αποτυγχάνουν στην τροχιά — η σύγκριση δεν κρίνεται

**Σχετικά:** #100, #102 · `run_ncd.py --motion-file=<npz>` (προϋπολογισμένες κινήσεις εικόνας, και στα Livox) · `configs/livox_voxel025_nodeskew.yaml` · `scripts/evaluate_tiers.py`
(APE έναντι mocap με μία μετατόπιση ρολογιού ανά αισθητήρα από το run KISS· RPE 1 s στροφή έναντι του γυροσκοπίου του Livox· απόσταση 1 s και μήκος έναντι mocap) ·
`kiss_runs/livox101_eval.txt`

Βραχίονες (voxel 0.25 m, 1 σπόρος, προσαρμοστικό σ σε όλους): KISS / KISS χωρίς deskew / η μέθοδος (deskew + αρχή ICP + δύο αρχές) με τις κινήσεις των #095–#100. **Το GT
είναι επίπεδη πλατφόρμα εδάφους** (22.3 m σε 42 s, 6.3 × 2.8 m, εύρος ύψους 0.02 m)· ο KISS με 0.25 m δίνει 34–40 m, εύρος 7–12 m και 2.5–3 m ύψος — APE 1.6–3.9 m σε όλους,
σχεδόν ανεξάρτητο της μετατόπισης ρολογιού (−1…+1 s). Μόνο το oracle γυροσκοπίου < 1 m (0.83 / 0.97). Η στροφή ανά σάρωση 0.7° (#100) δεν αρκούσε: RMSE 1 s 5–6°.
**Συμπέρασμα:** δεν κρίνεται — ο ICP με 0.25 m δεν δίνει τροχιά εδώ· → #102 (0.1 m).

---

### 2026-10-02 — #100 Livox: KISS με voxel 0.25 m δουλεύει· η εικόνα με αντιστάθμιση από αυτόν φτάνει μόνο την ακρίβεια του ίδιου του ICP, όχι του oracle

**Σχετικά:** #097, #099 · **ΑΠΟΦΑΣΗ Μ.Τ. 2/10: voxel 0.25 m για τα Livox** · `configs/livox_voxel025.yaml` (η ρύθμιση του paper με μόνο το voxel 0.25 m, χάρτης και τοπικοί χάρτες) ·
`scripts/score_run_gyro.py` (στροφή ανά σάρωση ενός run έναντι του γυροσκοπίου του Livox) · launchers `kiss_runs/{kiss100,icp100}_launch.sh` · runs `kiss_runs_ssd/tiers/indoor02/kiss025_*`
· έξοδος `kiss_runs/solid_state100.txt`

#### Αποτέλεσμα
**KISS, στροφή ανά σάρωση έναντι γυροσκοπίου** (p50 / p90, συσχέτιση· στροφή ανά σάρωση 0.83°): Avia voxel 1 m 6.8 / 28°, 0.19 → **0.25 m 0.70 / 1.90°, 0.70**· Horizon 4.5 / 15°, 0.22 →
**0.79 / 1.86°, 0.81**.

**Εικόνα με συσσώρευση και αντιστάθμιση από τις θέσεις αυτού του KISS** (σφάλμα στροφής p50 / p90, αποτυχίες / 422, inliers):

| | K = 2 (ανίχνευση ×2) | K = 3 | K = 5 (ανίχνευση ×2) | oracle γυροσκοπίου K = 5 (#097) | ο ίδιος ο ICP |
|---|---|---|---|---|---|
| Avia | 0.67 / 1.85°, 70, 69 | 0.69 / 2.66°, 58, 99 | 0.72 / 2.97°, 35, 150 | 0.31 / 0.89°, 0, 162 | 0.70 / 1.90° |
| Horizon | 0.71 / 2.89°, 37, 81 | 0.86 / 3.09°, 47, 89 | 0.92 / 3.32°, 32, 107 | 0.34 / 0.83°, 0, 167 | 0.79 / 1.86° |

#### Συμπέρασμα ⏳
1. **Ο KISS δουλεύει στα Livox με voxel 0.25 m** (στροφή ανά σάρωση 0.7–0.8°)· με 1 m όχι (#099). Βάση για τα solid-state: `livox_voxel025.yaml`.
2. **Η αντιστάθμιση από τον ICP δεν αποκλίνει** (σε αντίθεση με τις εκτιμήσεις της εικόνας, #097): K = 5 → 35 / 32 αποτυχίες, 107–150 inliers. **Αλλά το σφάλμα μένει στο επίπεδο του
   ίδιου του ICP** (0.67–0.92° έναντι 0.70 / 0.79°), μακριά από το oracle (0.31–0.34°): το τρέμουλο του ICP τοποθετεί λάθος το συσσωρευμένο περιεχόμενο και η εικόνα το κληρονομεί.
   Το p90 χειρότερο από του ICP (1.9–3.3° έναντι 1.9°).
3. **Σε αυτή τη μορφή η εικόνα δεν προσθέτει πληροφορία στα Livox.** Ανοιχτό (Β.10): αντιστάθμιση που να μην εξαρτάται από τον ICP ανά σάρωση (π.χ. ομαλή τροχιά από
   πολλές σαρώσεις ή μόνο η στροφή)· ή σύγκριση στην τροχιά (KISS με / χωρίς deskew της εικόνας), αφού ένα σφάλμα ανά σάρωση ίδιο με του ICP δεν λέει αν βοηθά το deskew.

---

### 2026-10-02 — #099 Livox στο pipeline (reader `CustomMsg`)· αντιστάθμιση από τον ICP: ο KISS με τη ρύθμιση του paper αποτυγχάνει στα Livox — η δοκιμή δεν κρίνεται

**Σχετικά:** #097, open_tasks Β.10(ε) · `kiss_slam/tools/livox.py` (`22bbc48`) · launcher `kiss_runs/icp099_launch.sh` · runs `kiss_runs_ssd/tiers/indoor02/kiss_{avia,horizon}_s0`
· έξοδος `kiss_runs/solid_state099.txt`

#### Μέθοδος
(α) Reader `LivoxRosbag`: τα bytes του `livox_ros_driver/CustomMsg` απευθείας (2 ms / σάρωση, ταυτόσημο με το rosbags), χρόνος σημείου = επικεφαλίδα + offset_time,
χρόνος συσκευής → ROS με τη μετατόπιση του πρώτου μηνύματος, μηδενικά σημεία έξω (~1200 / 24000)· `run_ncd.py` τον επιλέγει για topics `CustomMsg` (μόνο βραχίονας kiss).
(β) KISS (ρύθμιση του paper, `threads4.yaml`: voxel 1 m) σε Avia και Horizon, 423 σαρώσεις. (γ) Ο έλεγχος του #097 με αντιστάθμιση από τις θέσεις του KISS (M_j = P_{j−1}⁻¹ P_j).

#### Αποτέλεσμα
- **Η στροφή ανά σάρωση του ίδιου του KISS έναντι του γυροσκοπίου:** Avia **6.8° (p90 28°)**, Horizon **4.5° (p90 15°)**, συσχέτιση 0.2 — ο KISS με voxel 1 m **αποτυγχάνει**
  στα Livox σε αυτόν τον εσωτερικό χώρο (στενό πεδίο, λίγα σημεία μετά το voxel — όπως η σκάλα, #063).
- Αντιστάθμιση από αυτές τις θέσεις: 50–83 % αποτυχίες, 2–3.4° — χειρότερη και από τις εκτιμήσεις της εικόνας (#097).

#### Συμπέρασμα ⏳
1. Ο reader δουλεύει· τα Livox μπαίνουν στο pipeline (βραχίονας kiss).
2. **Η δοκιμή «αντιστάθμιση από τον ICP» δεν κρίνεται** — ο ICP της βάσης δεν δουλεύει εδώ. Πρώτα ένας ICP που δουλεύει στα Livox (ρύθμιση εσωτερικού χώρου / προεπιλογή
   του KISS-ICP με voxel από το max range), μετά ξανά ο έλεγχος.

---

### 2026-10-02 — #097 Solid-state: συσσώρευση με αντιστάθμιση κίνησης — με σωστή αντιστάθμιση η εικόνα των Livox γίνεται καλύτερη του Ouster· με τις δικές της εκτιμήσεις αποκλίνει

**Σχετικά:** #095, open_tasks Β.10(α) · script `scripts/solid_state_accum.py` · έξοδος `kiss_runs/solid_state096.txt`, κινήσεις `kiss_runs_ssd/solid096/` · TIERS Indoor02, 423 σαρώσεις

#### Μέθοδος
Η εικόνα της σάρωσης k από τις σαρώσεις k−K+1…k, όλες στο πλαίσιο της αρχής της k: οι παλαιότερες με deskew από τη δική τους κίνηση και μεταφορά με τις επόμενες, η τρέχουσα
με την πρόβλεψη σταθερής ταχύτητας. **Η εικόνα μόνο τοποθετεί τα χαρακτηριστικά:** 3D σημείο και χρόνος από τα ακατέργαστα σημεία της ΝΕΟΤΕΡΗΣ σάρωσης (≤ 1.5 px), άρα η
προσαρμογή, το deskew και ο ICP ίδια. Αντιστάθμιση: «estimate» = οι εκτιμήσεις της ίδιας της εικόνας (όπως θα έτρεχε η μέθοδος)· «gyro» = στροφή από το IMU του Livox (oracle
στροφής, μετατόπιση 0). K = 2 / 3 / 5, ανίχνευση ×1 / ×2. Αναφορά: γυροσκόπιο του αισθητήρα, με το bias αφαιρεμένο από το πρώτο 1 s ακινησίας (0.05–0.1°/s· το #095 ξαναβαθμολογήθηκε:
αλλαγές ≤ 0.02°, ίδιο).

#### Αποτέλεσμα (σφάλμα στροφής ανά σάρωση 0.1 s, p50 / p90· στροφή ανά σάρωση ~0.8°)

| | Avia | Horizon |
|---|---|---|
| μία σάρωση (#095, καλύτερη) | 3.3 / 9.0°, 85 αποτυχίες | 0.78 / 2.30°, 19 αποτυχίες |
| K = 2, estimate (καλύτερη) | 0.87 / 4.31°, 93 αποτυχίες | 0.65 / 2.49°, 38 |
| K = 3, estimate | 1.6 / 6.5°, 129 | 1.4 / 5.2°, 76 |
| K = 5, estimate | 3.2 / 8.0°, 253 | 2.8 / 8.6°, 142 |
| K = 3, gyro | 0.42 / 1.61°, **0**, 105 inliers | 0.36 / 1.08°, **0**, 136 inliers |
| **K = 5, gyro** | **0.31 / 0.89°, 0, 162 inliers, συσχέτιση 0.93** | **0.34 / 0.83°, 0, 167 inliers, 0.96** |

(Ouster OS0 της ίδιας διαδρομής, #095: 0.63° έναντι mocap — άλλη αναφορά.)

#### Συμπέρασμα ⏳
1. **Η πυκνότητα λύνεται με συσσώρευση με αντιστάθμιση:** με σωστή στροφή (oracle) το Avia πάει από «χωρίς σήμα» σε 0.31° και το Horizon σε 0.34°, 0 αποτυχίες — η εικόνα
   των Livox μπορεί να μετρήσει την κίνηση μέσα στη σάρωση.
2. **Με τις εκτιμήσεις της ίδιας της εικόνας η συσσώρευση αποκλίνει** (K ≥ 3: σφάλματα 1.4–3.7°, 30–75 % αποτυχίες): ένα λάθος χαλάει την επόμενη εικόνα — ανάδραση. Μόνο K = 2
   βοηθά λίγο (Horizon 0.78 → 0.65°).
3. **Επόμενο:** η αντιστάθμιση από τον **ICP** (οι καταχωρημένες θέσεις των προηγούμενων σαρώσεων, όπως ο τοπικός χάρτης του KISS) αντί για την εικόνα — χρειάζεται reader
   `CustomMsg` στο pipeline (Β.10(ε)). Η εικόνα γίνεται «εικόνα τοπικού χάρτη intensity»· η σάρωση k ταιριάζει σε αυτήν.

---

### 2026-10-02 — #096 Πανόραμα από τον αισθητήρα σε όλες τις 27 × 4: Hesai καλύτερο τοπικά (6–0), OS1-64 / Hilti / NTU μικτά — όχι ως προεπιλογή ακόμη

**Σχετικά:** #093 (συνέχεια· εκεί πλήρη runs μόνο NCD 2021) · launcher `kiss_runs/autowh093full_launch.sh` · βραχίονες `base092`, `autowh093` (NCD 2021: `up2093`) · πίνακες
`kiss_runs/results_official_allarms_093b.{md,csv}`, `kiss_runs/comparisons_093b.txt` · ο κανόνας με κλασματική μεγέθυνση από σημεία > 5 m (`67ab9fd`: στο Hesai ο ακέραιος γύριζε
2 ↔ 3 ανάλογα με τη σκηνή)

#### Αποτέλεσμα (ο κανόνας έναντι της προεπιλογής, μέσος 4 σπόρων ανά ακολουθία)

| dataset (αισθητήρας, επιλογή) | RPE 1 m στροφή | RPE 1 m μετατόπιση | APE / επίσημο σκορ |
|---|---|---|---|
| Oxford Spires (Hesai, 600 × 2.5) | **6–0, −5.7 %** (p = 0.031) | **6–0, −5.2 %** (0.031) | 3–3, +1.1 % |
| NCD 2021 (Ouster 128, 1024 × 2) | 7–2, −1.6 % | 6–3, −1.3 % | 4–5, +0.2 % |
| NCD 2020 (OS1-64, 1024 × 1.55) | 0–3, +0.8 % | 0–3, +4.0 % | 2–1 |
| Hilti (Ouster 64, 2048 × 8) | — | — | 2–4, +5.7 % |
| NTU (OS1-16, 1024 × 6) | — | — | 2–1, −10 % |
| **όλες οι 27** | 13–5, −2.3 % (p = 0.09) | 12–6, −1.9 % | **13–14, +0.2 %** |

Μεγάλες αλλαγές APE (μέσος ± σ): observatory-01 0.072 ± 0.008 → 0.234 ± 0.087 (αποτυχίες εικόνας 1 → 2.3)· dynamic_spinning 0.171 → 0.311 (αποτυχίες 36 → 56)· Office_Mitte_1
0.185 → 0.295· eee_01 1.48 → 1.81 (αποτυχίες 249 → 387)· eee_02 0.84 → 0.75, Basement_1 0.053 → 0.042.

#### Συμπέρασμα ⏳
1. Στο Hesai ο κανόνας βελτιώνει την τοπική ακρίβεια σταθερά (στροφή και μετατόπιση 6 / 6) και είναι ~×1.7 ταχύτερος.
2. **Όχι ως γενική προεπιλογή ακόμη:** στο OS1-64 χειρότερος (0–3, dynamic_spinning +60 % αποτυχίες), στο NTU eee_01 περισσότερες αποτυχίες (249 → 387), Hilti 2–4 στο σκορ,
   και ένα observatory με 3× APE. Σε όλες τις 27 το APE ίσο (13–14). Κοινό στους χειρότερους: λιγότερες γραμμές πανοράματος (OS1-64 99, NTU 95 έναντι 128–512 πριν).
3. Ανοιχτό (απόφαση Μ.Τ.): ο κανόνας μόνο για τις στήλες (στο Hesai η κύρια αιτία, #093) με μεγέθυνση όπως πριν· ή ελάχιστο ύψος εικόνας.
4. Αναλυτικός πίνακας (όλα τα μέτρα, μέσος ± σ ανά ακολουθία, επιλογή του κανόνα, Wilcoxon ανά dataset): `docs/results_panorama_rule_096.md`
   (`scripts/table_panorama_rule.py`). Επιπλέον εκεί: στο Hesai το z RMSE χειροτερεύει (2–4, διάμεσος +21 %) παρά την τοπική βελτίωση.

---

### 2026-10-02 — #095 Solid-state LiDAR (TIERS Indoor02): η εικόνα των Livox δίνει κίνηση με σφάλμα όσο η ίδια η κίνηση — ανεπαρκής πυκνότητα· Ouster OS0 όπως πάντα

**Σχετικά:** open_tasks Β.10 (νέο) · ΑΠΟΦΑΣΗ Μ.Τ. 2/10: η μέθοδος πρέπει να δουλεύει και σε solid-state · dataset TIERS (`docs/datasets.md`) · script
`scripts/solid_state_check.py` · έξοδος `kiss_runs/solid_state095{,_acc,_gyro}.txt`, κινήσεις `kiss_runs_ssd/solid095/` · μόνο εκτίμηση κίνησης, χωρίς SLAM

#### Στόχος
Αν η κίνηση από την εικόνα intensity λειτουργεί σε Livox (μη επαναλαμβανόμενη σάρωση, χωρίς rings, μικρό οπτικό πεδίο) πριν από οποιαδήποτε αλλαγή στη μέθοδο.

#### Μέθοδος
TIERS Indoor02 (42 s, 423 σαρώσεις ανά LiDAR, ταυτόχρονα Livox Avia, Livox Horizon, Ouster OS0, OS1, VLP-16, mocap). Livox (`livox_ros_driver/CustomMsg`, χρόνος ανά σημείο):
εικόνα αζιμούθιο × ύψος μέσα στο οπτικό πεδίο, pixel = √(εμβαδόν πεδίου / σημεία) (ένα σημείο ανά pixel — ο κανόνας του #093 χωρίς rings)· κενά intensity με κανονικοποιημένη
συνέλιξη, 3D σημείο και χρόνος από το πλησιέστερο σημείο ≤ 1.5 px· ίδιος εκτιμητής (SURF upright, car, brute force). Παραλλαγές: pixel ×1 / ×2, ανίχνευση στην εικόνα ×1 / ×2,
**μη επικαλυπτόμενα** παράθυρα K = 1 / 2 / 3 σαρώσεων ως μία σάρωση K × 0.1 s (κυλιόμενα θα μοιράζονταν σαρώσεις → κίνηση προς το μηδέν). Αναφορά: για τα Livox **το
γυροσκόπιο του ίδιου αισθητήρα** (ίδιο ρολόι συσκευής, άξονες του LiDAR — χωρίς μετατόπιση χρόνου ή εξωτερική βαθμονόμηση)· για το Ouster το mocap (μετατόπιση χρόνου και
στροφή LiDAR–σώματος εκτιμημένες από τα δεδομένα· στα Livox η ίδια εκτίμηση αποτύγχανε, γι' αυτό το γυροσκόπιο). Οι επικεφαλίδες των LiDAR του TIERS έχουν χρόνο συσκευής
(δευτερόλεπτα από την εκκίνηση), όχι ROS.

#### Αποτέλεσμα

| αισθητήρας, ρύθμιση | αποτυχίες | inliers | σφάλμα στροφής p50 / p90 | στροφή ανά σάρωση | συσχέτιση μέτρου |
|---|---|---|---|---|---|
| **Ouster OS0** (μέθοδος, 2048 × 4.16) | **0 / 422** | **745** | **0.63 / 1.30°** (mocap) | 0.98° | 0.91 |
| Avia, K = 1 (καλύτερη) | 85 / 422 | 22 | 3.28 / 9.01° | 0.74° | −0.12 (καμία) |
| Avia, K = 2, pixel ×2, ανίχνευση ×2 | 75 / 210 | 29 | 0.95 / 3.65° | 1.20° | 0.85 |
| Avia, K = 3 (καλύτερη) | 56 / 140 | 29 | 1.17 / 3.69° | 1.51° | 0.74 |
| **Horizon, K = 1, ανίχνευση ×2** | **19 / 422** | **43** | **0.78 / 2.31°** | 0.72° | 0.57 |
| Horizon, K = 2 (καλύτερη) | 37 / 210 | 30 | 1.03 / 3.12° | 1.15° | 0.62 |

Pixel του Avia 0.45° (158 στήλες), του Horizon 0.30° (277)· pixel ×2 → 79–139 στήλες, έως 99 % αποτυχίες. Στις πρώτες 40 σαρώσεις το Horizon είχε 0.49° (0 αποτυχίες)·
στο σύνολο χειρότερα. Η αναφορά mocap για τα Livox έδινε απαισιόδοξα νούμερα (μετατόπιση χρόνου στο όριο της αναζήτησης, συσχέτιση ≈ 0).

#### Συμπέρασμα ⏳
1. **Σε Livox η κίνηση της εικόνας όπως είναι δεν αρκεί:** το σφάλμα είναι όσο η ίδια η κίνηση ανά σάρωση (Horizon 0.78° σε 0.72°, Avia 0.95° σε 1.2° με K = 2) με 5–40 %
   αποτυχίες· ένα deskew από αυτήν θα ήταν χειρότερο από τη σταθερή ταχύτητα. Το Ouster OS0 της ίδιας διαδρομής: 745 inliers, 0 αποτυχίες.
2. **Το όριο είναι η πυκνότητα:** 14–43 inliers έναντι 745· το Horizon (στενό πεδίο, πυκνότερο) είναι καλύτερο από το Avia, και μία σάρωση καλύτερη από τη συσσώρευση
   (η κίνηση μέσα στο παράθυρο θολώνει την εικόνα). Στο Avia η μία σάρωση δεν έχει καθόλου σήμα.
3. Ανοιχτά (Β.10): συσσώρευση **με αντιστάθμιση κίνησης** (τοπική εικόνα-χάρτης από τις προηγούμενες εκτιμήσεις), εικόνα απόστασης ως δεύτερη πηγή, άλλοι ανιχνευτές
   για αραιές εικόνες· και το Ouster με αναφορά το δικό του IMU για ίδια σύγκριση.

---

### 2026-10-02 — #094 Διαγνωστικά των εξωτερικών κριτικών: καμία ουσιαστική υστέρηση / μεροληψία jerk, σχεδόν καθόλου aliases· το GT του math_easy θορυβώδες ανά σάρωση

**Σχετικά:** #086, #090, #093 · `KISS_SLAM_Technical_Critique_and_Experimental_Roadmap.md`, `intensity_deskew_review_and_plan.md`, `stress_test_results_and_improvements.md`
(δίσκος δεδομένων, 25–28/9) · **ΑΠΟΦΑΣΗ Μ.Τ. 2/10: όχι πυκνότερη πηγή του ICP (αλλάζει τη βάση σύγκρισης)· ενδιαφέρον μόνο η αποτελεσματικότητα του deskew και η
ενσωμάτωση της εικόνας στον ICP** · script `scripts/diagnose_image_motion.py` · έξοδος `kiss_runs/diagnose094.txt` · μόνο υπάρχοντα runs (`base092_s0`, `surftwofbv2b_s0`)

#### Στόχος
Δύο διαγνωστικά που προτείνουν οι κριτικές πριν από υλοποίηση: (α) αν η κίνηση της εικόνας (δύο σαρώσεις, μοντέλο car) έχει τη μεροληψία υστέρησης (−α/2) ή jerk (−j/12)
που θα αφαιρούσε εκτιμητής με υστέρηση μίας σάρωσης (k−1, k, k+1)· (β) αν υπάρχουν μεταφορικά aliases (ένα σκαλοπάτι, περίοδος πρόσοψης) που θα έπιανε κατώφλι ταχύτητας.

#### Μέθοδος
(α) Ανά σάρωση σφάλμα στροφής e = rotvec(Dᵀ M) έναντι της κίνησης του GT μέσα στη σάρωση, παλινδρόμηση (3 άξονες μαζί, |e| < 5°) στη μεταβολή του GT dr = r_k − r_{k−1}
και στη δεύτερη διαφορά d2r = r_{k+1} − 2r_k + r_{k−1}· κλίση, R². (β) Ταχύτητα της κίνησης της εικόνας ανά σάρωση έναντι GT (ή του ICP όπου δεν υπάρχει GT ανά σάρωση:
Office_Mitte_1, eee_01)· ιστόγραμμα |t_εικόνας − t_αναφοράς|.

#### Αποτέλεσμα
- **Υστέρηση / jerk** (6 καθαρές ακολουθίες: christ-church-02 / -03, keble-03, bodleian-02, quad_easy, underground_hard): κλίση σε dr −0.09…+0.02 (cv χωρίς α: −0.5), σε d2r
  −0.06…−0.01 (πρόβλεψη −0.083), **R² ≤ 0.05 η καθεμία**· μαζί R² 0.03–0.15 (Hesai 0.13–0.15: το σφάλμα συσχετίζεται λίγο με τη μεταβολή της ΕΠΟΜΕΝΗΣ σάρωσης — άκρο του
  παραθύρου). Cloister R² 0.11. Ό,τι θα αφαιρούσε ένας εκτιμητής με υστέρηση: **≤ 7 % του RMS** του σφάλματος.
- math_easy (κλίση −0.66, R² 0.47) και dynamic_spinning (R² 0.43): το μοτίβο (αρνητικό σε dr, θετικό σε d2r) είναι αυτό που δίνει **θόρυβος του GT ανά σάρωση**. Έλεγχος:
  μεταβολή ταχύτητας του GT μεταξύ σαρώσεων διάμεσος **0.35 m/s στο math_easy** (μέγιστη ταχύτητα 5.6 m/s) έναντι 0.05 christ-church-03, 0.03 cloister. **Το GT του math_easy δεν
  αρκεί για σφάλμα ανά σάρωση** — τα νούμερα ανά σάρωση του math_easy στα #086 / #092 / #093 περιέχουν θόρυβο του GT (τα συμπεράσματα στηρίζονται και σε ≥ 7 άλλες).
- **Aliases:** εικόνα > 2.5 m/s: 0 σε christ-church-02 / -03, keble, quad, cloister· 7 / 5006 bodleian, 6 / 1905 underground_hard, 22 / 1159 dynamic_spinning (γρήγορη περιστροφή)·
  Office_Mitte_1 / eee_01 15 / 19 σαρώσεις > 2× ICP. Το |t_εικόνας − t_GT| φθίνει μονότονα, **καμία δεύτερη κορυφή σε περίοδο δομής** (0.3 m).

#### Συμπέρασμα ⏳
1. **Εκτιμητής με υστέρηση μίας σάρωσης (k−1, k, k+1): όχι** — η μεροληψία που θα αφαιρούσε εξηγεί ≤ 15 % της διασποράς (≤ 7 % του RMS) και, όπως έδειξαν τα #092 / #093,
   μικρότερο σφάλμα εικόνας δεν περνά στην τροχιά. Μαζί με το #090 (k−2: χειρότερο) κλείνει η κατεύθυνση «περισσότερος χρόνος στην προσαρμογή».
2. **Κατώφλι ταχύτητας / μετατόπισης για aliases: όχι** — < 0.5 % των σαρώσεων (2 % στο dynamic_spinning), χωρίς υπογραφή περιοδικής δομής· και (Μ.Τ. 2/10) κάθε
   τέτοιο κατώφλι εξαρτάται από την πλατφόρμα (αυτοκίνητο 10–30 m/s, χειρός ~1.5, drone ενδιάμεσα), αντίθετα με τη γενική, χωρίς ρυθμίσεις κατεύθυνση του #093.
3. Το GT του math_easy (και του dynamic_spinning) δεν χρησιμοποιείται για σφάλμα ανά σάρωση.

---

### 2026-10-02 — #093 Πανόραμα από τον αισθητήρα (στήλες + τετράγωνα pixel): ίδια ακρίβεια, 1.7× ταχύτερο· whitening, τομείς, cross-check, κλίμακα ανίχνευσης: όχι

**Σχετικά:** #086, #087, #091 (`--panorama-up=4`), #092 · ιδέες του `ideas_for_test.md` (Μ.Τ. 2/10) · κλάδος `after_091` · επιλογές (`7a7d2f8`, `5b64461`, προεπιλογές
αμετάβλητες): `image_deskew.{fit_sectors, whiten, cross_check, detect_scale}`, `panorama_width: auto`, `panorama_up: auto`· `run_ncd.py --sectors= --whiten= --cross-check
--detect-scale= --panorama-width=auto --panorama-up=auto` · τα παράθυρα της καθοδηγούμενης αντιστοίχισης ως γωνίες (στήλες στα 1024, `_px`· ίδια στα 1024) ·
script `scripts/analyse_motion_windows.py` · launchers `kiss_runs/{off093,auto093,autowh093,ds093,ds093b,up2093,speed093}_launch.sh` · runs `kiss_runs_ssd/offline093/`,
`kiss_runs_ssd/speed093/` · πίνακες `kiss_runs/offline093_{motion,up,autowh,ds}.txt`, `kiss_runs/results_official_allarms_093.{md,csv}`, `kiss_runs/comparisons_093.txt`

#### Στόχος
Οι φθηνές ιδέες του `ideas_for_test.md` για την ακρίβεια της κίνησης της εικόνας (whitening απόστασης–γωνίας, ισοκατανομή ανά αζιμούθιο, αμφίδρομος έλεγχος,
τετράγωνα pixel) και, μετά από ερώτημα του Μ.Τ., **γενικός κανόνας χωρίς ρύθμιση ανά αισθητήρα** για το μέγεθος του πανοράματος· κλίμακα ανίχνευσης για ταχύτητα.

#### Μέθοδος
(α) **Offline**: πρώτες 600 σαρώσεις, σπόρος 0, σφάλμα της κίνησης της εικόνας έναντι της κίνησης του GT μέσα στη σάρωση (`oracle_motion.npz`) ανά σάρωση **και
συντιθέμενο σε 10 σαρώσεις (1 s)** — μετά το #092 το σφάλμα που μετρά είναι το συσχετισμένο σε πολλές σαρώσεις. 8 ακολουθίες (4 Hesai, 4 Ouster 128) + dynamic_spinning
(OS1-64)· Hilti / NTU μόνο αποτυχίες και inliers (χωρίς στροφή GT ανά σάρωση). (β) Whitening: υπόλοιπο στη βάση απόσταση / αζιμούθιο / ύψος, σ = (0.03 m, 0.003 rad,
0.008 ή 0.004 rad)· μία soft_l1 ανά σημείο με IRLS (με |A r| ως υπόλοιπο ο least_squares έφτανε το όριο επαναλήψεων: 1 s / σάρωση). (γ) Τομείς: βάρος ανά αντιστοίχιση ώστε
κάθε τομέας (8 / 4) να έχει ίσο βάρος. (δ) Cross-check στο ίδιο παράθυρο ανάποδα. (ε) **Κανόνας πανοράματος:** στήλες = του αισθητήρα (360° / διάμεσο βήμα αζιμουθίου
ενός ring), μεγέθυνση = round(διάμεσο βήμα ύψους μεταξύ rings / πλάτος στήλης)· από την πρώτη σάρωση. (στ) Κλίμακα ανίχνευσης 0.75 / 0.5 (πανόραμα μικρότερο μόνο για
το SURF, σημεία πίσω σε πλήρη ανάλυση). (ζ) Πλήρη runs: μεγέθυνση 2 (= ο κανόνας στο Ouster 128) έναντι προεπιλογής, όλες οι 9 NCD 2021 × 4 σπόροι. (η) Ταχύτητα: αδρανές
μηχάνημα, ένα run τη φορά, 400 σαρώσεις, `--parallel`, christ-church-03 από το ασυμπίεστο bag.

#### Αποτέλεσμα
**Μετρήσεις αισθητήρων** (μία σάρωση): Hesai QT64 600 στήλες (0.600°, διπλή επιστροφή), rings 1.56° → **600 × 3**· Ouster 128 (NCD 2021) 1024, 0.71° → **1024 × 2**·
OS1-64 (NCD 2020) 1024, 0.54° → **1024 × 2**· Ouster 64 Hilti **2048**, 1.39° → **2048 × 8**· OS1-16 NTU 1024, 2.18° → **1024 × 6**. Σήμερα 1024 × 8 σε όλα: το Hesai
υπερδειγματοληπτείται οριζόντια (κενές στήλες με παρεμβολή, το moiré του #029), το Hilti χάνει το μισό.

**Offline (σφάλμα στροφής συντιθέμενο σε 10 σαρώσεις, έναντι προεπιλογής):** whitening χειρότερο ή ίσο σχεδόν παντού (0 έως +140 %· μόνο christ-church-03 καλύτερο, −8 έως −14 %)· τομείς
καλύτεροι ανά σάρωση, χειρότεροι στις 10 (quad +7 %, bodleian +8 %) και στη μετατόπιση· cross-check ουδέτερο (−4 έως +9 %). **Κανόνας πανοράματος:**

| ακολουθία | επιλογή | στροφή / 10 σαρώσεις [°] | μετατόπιση / 10 |
|---|---|---|---|
| bodleian-02 / christ-church-02 / -03 / keble-03 (Hesai) | 600 × 3 | 2.278 → 1.807 / 1.658 → 1.153 / 1.406 → 0.834 / 2.008 → 1.889 | −21 / −27 / −13 / −13 % |
| underground_hard / math_easy / quad_easy / cloister (Ouster 128) | 1024 × 2 | −23 / −18 / −30 / −22 % | −24 / −24 / −43 / −42 % |
| dynamic_spinning (OS1-64) | 1024 × 2 | 2.713 → 2.896 (+7 %) | +5 % |

Μόνο μεγέθυνση στα 1024 (χωρίς τις στήλες) στο Hesai: 4 / 6 / 12 μικτά (±10 %) — το Hesai ακολουθεί τον κανόνα μόνο με τις δικές του στήλες. Ouster 128: 1 / 3 / 4
χειρότερα από 2 και στις 4. Hilti 2048 × 8: inliers 154 → 226, αποτυχίες 1 / 1· NTU 1024 × 6: 207 → 170, 1 / 1.

**Κλίμακα ανίχνευσης** (πάνω στον κανόνα, 10 σαρώσεις): 0.75 christ-church-03 +15 %, math_easy +3 %· 0.5 +71 % / +19 %, αποτυχίες dynamic_spinning 4 → 79, eee_01 1 → 52.

**Πλήρη runs, NCD 2021, μεγέθυνση 2 έναντι προεπιλογής (9 ακολουθίες × 4 σπόροι):** RPE 1 m μετατόπιση −1.3 % (6–3), στροφή −1.6 % (7–2), RPE 1 s 7–2, RTE −2.4 %,
APE +0.2 % (4–5) — **όλα μη σημαντικά** (p ≥ 0.13). Π.χ. quad_easy στροφή 0.591 → 0.611 °/m, cloister 0.711 → 0.705, quad_hard 1.042 → 1.008 (APE 0.233 → 0.196),
underground_hard 1.088 → 1.046.

**Ταχύτητα** (400 σαρώσεις, πραγματικός χρόνος): christ-church-03 προεπιλογή 31 s = **12.9 Hz** (#087: 12.7), κανόνας 18 s = **22 Hz**, + κλίμακα 0.75 25 Hz· math_easy
10.3 Hz (#087: 10.1) → **17 Hz** → 20 Hz.

**ATE σε 600 σαρώσεις Hilti / NTU: δεν μετριέται** — Office_Mitte_1 έχει 2 από τα 9 σημεία αποτύπωσης στα πρώτα 60 s· στο eee_01 το GT αρχίζει στα 47 s (130 θέσεις,
ATE ίδιο ως το mm σε τρεις ρυθμίσεις).

#### Συμπέρασμα ⏳
1. **Whitening, τομείς, cross-check: όχι** (offline χειρότερα ή ουδέτερα). Μένουν επιλογές.
2. **Κανόνας πανοράματος από τον αισθητήρα** (στήλες του αισθητήρα, τετράγωνα pixel, από την πρώτη σάρωση, χωρίς ρύθμιση ανά dataset): offline 8 / 9 καλύτερα
   (−6 έως −41 % στη στροφή 10 σαρώσεων), αλλά **στα πλήρη runs του Ouster 128 η τροχιά δεν αλλάζει** (−1.5 %, μη σημαντικό). Όπως το #092: ο ICP και ο χάρτης
   απορροφούν το σφάλμα της εικόνας — **ούτε το σφάλμα 10 σαρώσεων προβλέπει την τροχιά**· τα offline μέτρα κρίνουν μόνο αρνητικά. Το κέρδος του κανόνα είναι
   **η ταχύτητα: ×1.7 (22 / 17 Hz) με ίδια ακρίβεια** και ένας κανόνας για κάθε αισθητήρα. **Προτείνεται ως προεπιλογή — απόφαση Μ.Τ.** Πριν: πλήρη runs στο Hesai,
   Hilti, NTU, OS1-64 (το μόνο offline χειρότερο).
3. **Κλίμακα ανίχνευσης: όχι** — 0.5 βλάπτει (αποτυχίες), 0.75 +10–15 % ταχύτητα με απώλεια ακρίβειας· ο κανόνας έχει ήδη μικρύνει την εικόνα.

---

### 2026-10-02 — #092 Στάθμιση της στροφής της εικόνας με τη σταθερή ταχύτητα: λιγότερο σφάλμα ανά σάρωση, όχι καλύτερη τροχιά

**Σχετικά:** #086 (δεύτερο deskew, εξομάλυνση), #087, open_tasks Β.5 · κλάδος `after_091` · επιλογή `image_deskew.rotation_cv_weight`,
`run_ncd.py --rot-cv=<w>` (`05bdfae`, προεπιλογή 0 = αμετάβλητο) · script `scripts/analyse_rotation_blend.py` · launcher `kiss_runs/rotcv092_launch.sh` ·
βραχίονες `base092` (προεπιλογή του `after_091`, ίδιος κώδικας), `rotcv05`, `rotcv10` · πίνακες `kiss_runs/results_official_allarms_092.{md,csv}`,
`kiss_runs/comparisons_092.txt`

#### Στόχος
Η τελευταία ιδέα του Β.5: στάθμιση της στροφής της εικόνας με μια δεύτερη εκτίμηση, ώστε να μειωθεί ο θόρυβος χωρίς bias του #086 (0.5–0.85° / σάρωση).

#### Μέθοδος
(α) **Offline**, χωρίς runs: στις καταγεγραμμένες κινήσεις των runs `guided` του #087 (8 ακολουθίες × 2 σπόροι), σε κάθε σάρωση με deskew από την εικόνα,
σφάλμα στροφής έναντι της κίνησης του GT μέσα στη σάρωση (`oracle_motion.npz`) για R(w) = R_img exp(w log(R_imgᵀ R_x)), με R_x (i) την κίνηση του ICP της
ίδιας σάρωσης (μετά το deskew) ή (ii) την προηγούμενη κίνηση του ICP (σταθερή ταχύτητα), w = 0…1· και w ανά σάρωση από τα inliers, n0 / (n0 + inliers).
(β) **Πλήρη runs:** w = 0.05 / 0.1 στη θέση της στροφής της εικόνας, για deskew ΚΑΙ αρχή του ICP (#030), έναντι της προεπιλογής με τον ίδιο κώδικα, 8 × 2.

#### Αποτέλεσμα
**(α) Offline** (διάμεσος σφάλματος στροφής ανά σάρωση, σχετικά με την εικόνα· διάμεσος των 16 runs):

| w | 0.1 | 0.2 | 0.3 | 0.5 | 1.0 |
|---|---|---|---|---|---|
| με τον ICP της ίδιας σάρωσης | 0.999 | 0.997 | 1.009 | 1.036 | 1.212 |
| με τη σταθερή ταχύτητα | **0.928** | 1.001 | 1.221 | 1.815 | 3.422 |

- Τα σφάλματα εικόνας και ICP της ίδιας σάρωσης είναι ισχυρά συσχετισμένα (r = 0.69–0.94): ο ICP κληρονομεί το σφάλμα του deskew — γι' αυτό απέτυχε και
  το δεύτερο deskew (#086).
- Με τη σταθερή ταχύτητα (σφάλμα μόνη της 1.3–2.8°, 3–5× της εικόνας): βέλτιστο w 0.05–0.12 ανά ακολουθία· cloister 0.361 → 0.292 (−19 %), quad_easy −11 %,
  keble −8 %, bodleian −7 %, christ-church-03 −2 %, math_easy −2 %. Το w από τα inliers όχι καλύτερο από σταθερό w.

**(β) Πλήρη runs** (έναντι `base092`, 8 ακολουθίες, μέσος 2 σπόρων):

| | RPE 1 m στροφή | RPE 1 s στροφή | RPE 1 m μετατόπιση | RTE | APE |
|---|---|---|---|---|---|
| w = 0.05 | −0.7 % (5–3) | **−1.5 % (7–1, p = 0.039)** | +0.5 % (4–4) | +0.5 % (4–4) | −1.0 % (6–2) |
| w = 0.1 | +0.6 % (4–4) | −1.8 % (5–3) | +0.6 % (4–4) | +0.6 % (3–5) | +1.1 % (3–5) |

RPE 1 m στροφή [°/m] base / 0.05 / 0.1: quad_easy 0.591 / 0.574 / 0.556, cloister 0.712 / 0.683 / 0.681, christ-church-02 0.705 / 0.714 / 0.728,
christ-church-03 0.703 / 0.712 / 0.726, math_easy 1.256 / 1.260 / 1.275, keble 1.042 / 1.027 / 1.028, bodleian 1.115 / 1.104 / 1.111.
Μέσα στα runs (σπόρος 0) η στροφή του ICP ανά σάρωση έναντι GT μειώνεται με w = 0.1: bodleian 1.042 → 0.993°, cloister 0.547 → 0.506, keble 0.860 → 0.806,
quad_easy 0.529 → 0.483, math_easy 0.896 → 0.871, christ-church-02 / 03 ίδια. Έλεγχος: `base092` έναντι `guided` (#087), ίδια ρύθμιση εκτός του κανόνα
αργής στροφής: όλα εντός ±2.4 %, p ≥ 0.25 — αναπαραγωγή.

#### Συμπέρασμα ⏳
1. **Η στάθμιση με τη σταθερή ταχύτητα μειώνει το σφάλμα ανά σάρωση (−7 % offline, −3…−9 % στον ICP μέσα στα runs), αλλά όχι το σφάλμα της τροχιάς**
   (RPE 1 m στροφή ±1 %, μη σημαντικό). Μικρό κέρδος μόνο στο RPE 1 s (w = 0.05: −1.5 %, 7–1)· με w = 0.1 οι christ-church χειροτερεύουν (+3 %).
   **Όχι προεπιλογή·** μένει ως επιλογή (`--rot-cv`).
2. Η στάθμιση με τον ICP της ίδιας σάρωσης δεν κάνει τίποτα (σφάλματα συσχετισμένα) — κλείνει μαζί με το δεύτερο deskew (#086).
3. **Τι δείχνει για το κενό στροφής:** ο θόρυβος ανά σάρωση που αφαιρεί το χαμηλοπερατό φίλτρο αλληλοαναιρείται σε 1 m (μια λάθος θέση μπαίνει σε δύο
   διαδοχικές κινήσεις με αντίθετο πρόσημο). Το oracle (#086) κερδίζει 40 % στο RPE 1 m, άρα διορθώνει **συσχετισμένο σε πολλές σαρώσεις** σφάλμα του deskew,
   όχι το τρέμουλο ανά σάρωση. Το επόμενο ερώτημα είναι το χαμηλής συχνότητας μέρος του σφάλματος της εικόνας (π.χ. συστηματικό σε τμήματα της διαδρομής
   ή εξαρτημένο από τη σκηνή), όχι ο θόρυβος.

---

### 2026-10-02 — #091 Πρόβλεψη από κίνηση σε 4 γρήγορες ακολουθίες· υβριδικό όχι καλύτερο (διόρθωση #089)· ταχύτητα σε 128 δέσμες· γιατί χειροτερεύει το Office_Mitte_1

**Σχετικά:** #087–#089 · κλάδος `rotation_bearing` (`40bab98`) · νέες επιλογές `run_ncd.py --surf-hessian=<τιμή> --panorama-up=<n>` (`image_deskew.panorama_up`,
εκτός προεπιλογής) · launcher `kiss_runs/fast089_launch.sh` · βραχίονες `surftwofbv2b` (κανόνας #088), `surftwofbv3` (κίνηση), `surftwofbv4` (υβριδικό) ·
πίνακες `kiss_runs/results_official_allarms_089.*` · **ΑΠΟΦΑΣΗ Μ.Τ. 2/10:** η ×4 και η πρόβλεψη από κίνηση μένουν επιλογές· η προεπιλογή μένει ανοιχτή

#### 1. Τέσσερις γρήγορες ακολουθίες, 4 σπόροι (APE m · RPE 1 m cm · RPE στροφής °/m · RTE %)

| | μέθοδος | κανόνας #088 | κίνηση | υβριδικό |
|---|---|---|---|---|
| underground_hard | 0.086 · 5.71 · 1.090 · 0.253 | 0.091 · 5.90 · 1.088 · 0.248 | **0.087 · 5.60 · 1.023 · 0.234** | 0.090 · 5.92 · 1.102 · 0.248 |
| quad_hard | 0.218 · 8.16 · 1.087 · 0.893 | 0.233 · 8.31 · 1.042 · 0.859 | **0.192 · 7.76 · 1.001 · 0.736** | 0.234 · 8.32 · 1.038 · 0.866 |
| underground_medium | 0.061 · 4.84 · 0.858 · 0.223 | 0.057 · 4.73 · 0.793 · 0.224 | 0.058 · 4.72 · 0.807 · 0.229 | 0.057 · 4.74 · 0.793 · 0.226 |
| dynamic_spinning | 0.504 · 20.4 · 3.373 · — | 0.171 · 14.6 · 3.186 · — | **0.094 · 11.9 · 3.015 · —** | 0.134 · 14.6 · 3.171 · — |

Το υβριδικό είναι σχεδόν ίδιο με τον κανόνα και στις τέσσερις — **όχι καλύτερο συνολικά** (Μ.Τ. 2/10· διορθώνει το συμπέρασμα 2 του #089). Η κίνηση κερδίζει
στις γρήγορες, χάνει στο Office_Mitte_1 (#089: 0.315 έναντι 0.185).

#### 2. Ταχύτητα (wall clock, 400 σαρώσεις, `--parallel`, μία διεργασία τη φορά)
- Προεπιλογή πανοράματος (UP = 8): quad_hard μέθοδος 8.3 / κανόνας 10.6 / κίνηση 10.0 / υβριδικό 10.5 Hz· underground_hard (Ouster 128) 5.3 / 8.1 / 7.6 / 8.1 Hz.
  KISS-SLAM μόνος στο underground_hard: 39 Hz — το όριο είναι η εικόνα.
- **Κατώφλι Hessian** (200 σαρώσεις, χρόνος εικόνας / διάμεσο σφάλμα στροφής έναντι GT): underground_hard 100: 110 ms / 0.556° · 200: 87 ms / 0.556° · 400: 77 ms / 0.578° ·
  800: 65 ms / 0.633°· christ-church-03 100–800: 70 → 60 ms, 0.435–0.452°. Πλήρης μέθοδος με 200: underground_hard 8.75 / 8.24 Hz (κανόνας / κίνηση), quad_hard 11.2 / 10.7.
- **Κατακόρυφη μεγέθυνση του πανοράματος** (`UP`, γραμμές ανά ring): underground_hard UP 8 → 4: **98 → 73 ms, στροφή 0.556 → 0.501° (90 %: 1.29 → 1.04°)**·
  UP 4 + Hessian 200: 68 ms, 0.532°. Πλήρης μέθοδος με UP 4: underground_hard **10.4 / 9.7 Hz**, quad_hard 12.7 / 12.1 Hz.

#### 3. Γιατί χειροτερεύει το Office_Mitte_1 με την πρόβλεψη από κίνηση
- **Ακρίβεια πρόβλεψης** (όλες οι 2641 σαρώσεις, αντιστοιχίσεις brute force): στήλη 3.0 (μετατόπιση) / 2.6 px (κίνηση), γραμμή 0.06 / 0.34 rings· εκτός παραθύρου
  (±40 / ±4) 0.8 / 2.1 % και 0.9 / 2.2 % — ίδιο.
- **Κίνηση ανά σάρωση:** ίδια στους δύο βραχίονες (διάμεσο 0.06°)· διαφέρει σε 241 σαρώσεις, όλες γρήγορη στροφή (γυροσκόπιο 3.7° / σάρωση έναντι 1.3 τυπικό).
- **Έναντι του γυροσκοπίου του bag** (`/os_cloud_node/imu`, 100 Hz, άξονες = σαρωτή, βάθρο από τα πρώτα 2 s, η κίνηση της σάρωσης k στο [t_k − 0.01, t_k + 0.09] s):
  στροφή εικόνας διάμεσο 0.566 (κανόνας) / 0.566° (κίνηση), στις γρήγορες 1.13 / 1.12°· τελική (ICP) 0.582 / 0.577°, άθροισμα 2307 / 2288°. **Καμία διαφορά ανά σάρωση.**
  (Το γυροσκόπιο κρίνει μόνο σχετικά: η διάμεση απόκλιση 0.57° είναι κοινή σε όλους.)
- **Ανά σημείο αποτύπωσης** (9 σημεία σε 264 s, μέσο 4 runs, m): η διαφορά είναι σχεδόν όλη στο σημείο 8 (t = 212 s): κανόνας 0.352, κίνηση **0.750**, υβριδικό 0.434,
  μέθοδος 0.343, DLO 0.141. Το τμήμα 7 → 8 (195 → 212 s, 13.81 m) κονταίνει σε όλους μας: κανόνας −0.35, κίνηση **−0.75**, υβριδικό −0.42, μέθοδος −0.20,
  DLO +0.01 m. Εκεί γρήγορο περπάτημα (1.4–1.6 m/s, 197–205 s) και στάση με απότομη στροφή (ως 9.6° / σάρωση)· 47 από τις 241 σαρώσεις που διαφέρουν.
  Σφάλμα στροφής εικόνας στο τμήμα 2.1 (κανόνας) / 2.3° (κίνηση) έναντι 0.57 σε όλη την ακολουθία· άθροισμα σφάλματος yaw στο τμήμα ανά run: κανόνας
  −2.9 / −2.7 / −2.7 / −5.4°, κίνηση +0.7 / −4.8 / +0.7 / −7.6°.

#### Συμπέρασμα ⏳
1. Η πρόβλεψη από κίνηση βοηθά σε γρήγορη στροφή (quad_hard −18 % APE, −14 % RTE· underground_hard −6 % RPE στροφής· dynamic_spinning −45 %). Το υβριδικό δεν
   προσφέρει κάτι πέρα από τον κανόνα.
2. Το Office_Mitte_1 δεν δείχνει κακή πρόβλεψη: ανά σάρωση οι βραχίονες είναι ίσοι. Η διαφορά είναι ένα τμήμα γρήγορου περπατήματος όπου η κίνηση της εικόνας
   είναι κακή για όλους (≈ 2°/σάρωση) και ο ICP κονταίνει τη διαδρομή· η κίνηση εκεί δίνει άλλες αρχές και μεγαλύτερη διασπορά yaw. Με 9 σημεία, ένα τμήμα
   κρίνει το APE — ασθενής ένδειξη. Το πραγματικό πρόβλημα (κοινό σε όλους μας, όχι στο DLO) είναι το γρήγορο περπάτημα.
3. ×4 πανόραμα στις 128 δέσμες: ταχύτερο και ακριβέστερο σε 200 σαρώσεις· underground_hard πάνω από 10 Hz. Δεν έχει δοκιμαστεί σε όλες τις 27.
4. **Ανοιχτό:** προεπιλογή πρόβλεψης (κανόνας ή κίνηση) και ×4 — μετά από περισσότερους σπόρους στο Office_Mitte_1 ή τα άλλα Hilti / όλες τις 27 (`docs/open_tasks.md` §Γ).

---

### 2026-10-02 — #090 Μεγαλύτερη βάση (k−2) και σύντηξη εικόνας απόστασης στην προσαρμογή της κίνησης: όχι

**Σχετικά:** #086, #089 · κλάδος `rotation_bearing` · `ScanMotionEstimator(multi_baseline=, fuse_range=)`, `_joint`, `match_motion.last_pairs` (εκτός προεπιλογής)

Ιδέες για την ακρίβεια της κίνησης (το όριο του #086): (1) αντιστοιχίσεις και k−2 ↔ k (χρόνοι −2…−1 περίοδοι, ίδια καμπύλη κίνησης) σε κοινή προσαρμογή·
(2) οι αντιστοιχίσεις της εικόνας απόστασης k−1 ↔ k μαζί με του intensity σε κοινή προσαρμογή, σε κάθε σάρωση. 200 σαρώσεις, σφάλμα στροφής έναντι GT
(christ-church-03 / bodleian-02, βάση η προεπιλογή του #088): βάση 0.440 / 0.776°· **k−2: 0.594 / 0.960° (+35 / +24 %)**· **απόσταση: 0.412 / 0.774° (−6 % / 0)**·
και τα δύο 0.566 / 0.909°.

**Συμπέρασμα ⏳:** η μεγαλύτερη βάση χειροτερεύει — το μοντέλο (σταθερή ταχύτητα + γωνιακή επιτάχυνση) δεν περιγράφει χειροκίνητη κίνηση σε 0.3 s· άρα το
σφάλμα ανά σάρωση δεν είναι θόρυβος μέτρησης που μειώνεται με περισσότερο χρόνο, αλλά η ακανόνιστη κίνηση μέσα στη σάρωση. Η σύντηξη απόστασης: μικρό
κέρδος σε μία ακολουθία, κόστος χαρακτηριστικών απόστασης σε κάθε σάρωση — όχι ως προεπιλογή.

---

### 2026-10-02 — #089 Καθοδηγούμενη αντιστοίχιση με πρόβλεψη ανά σημείο από την κίνηση· υβριδικό (μετατόπιση σε αργή, κίνηση σε γρήγορη στροφή)

**Σχετικά:** #087, #088 · κλάδος `rotation_bearing` · `image_deskew.guided_prediction: shift | motion | hybrid`, `run_ncd.py --guided-predict=` ·
`predict_pixels` στο `intensity_deskew.py` · launchers `kiss_runs/{v3test089,v4test089,v2test089o}_launch.sh` · βραχίονες `surftwofbv3` (motion), `surftwofbv4` (hybrid) ·
πίνακες `kiss_runs/results_official_allarms_089.*`

#### Μέθοδος
Πρόβλεψη ανά σημείο: το 3D σημείο κάθε σημείου-κλειδιού της προηγούμενης σάρωσης μετακινείται με την κίνηση της προηγούμενης σάρωσης (σταθερή ταχύτητα,
q = Rᵀ(p − t)) και προβάλλεται στο νέο πανόραμα (στήλη από αζιμούθιο· γραμμή από ύψος, παρεμβολή στα διάμεσα ύψη των rings της τρέχουσας σάρωσης)·
το παράθυρο (±40 στήλες, ±4 rings) εκεί. Σημεία χωρίς 3D σημείο / σάρωση χωρίς προηγούμενη κίνηση: η κοινή μετατόπιση του #088. Υβριδικό: κοινή
μετατόπιση όταν |μετατόπιση| ≤ 20 στήλες, πρόβλεψη από κίνηση όταν μεγαλύτερη (αντί για brute force).

#### Αποτέλεσμα
- **Ακρίβεια πρόβλεψης** (αντιστοιχίσεις brute force, πού κατέληξαν· η κοινή μετατόπιση από την ΠΡΟΗΓΟΥΜΕΝΗ σάρωση): christ-church-03 στήλες 2.7 → 2.0 px,
  γραμμές 0.10 → 0.34 rings· bodleian 3.8 → 4.3 px, 0.13 → 0.35· **dynamic_spinning (γρήγορο) 7.2 → 4.1 px, γραμμές 4.4 → 1.7 rings** — η κατακόρυφη
  μετακίνηση 4.4 rings, έξω από το παράθυρο ±4, εξηγεί την αποτυχία του σταθερού παραθύρου (#088).
- **200 / 1040 σαρώσεις, έναντι GT:** dynamic_spinning γρήγορο τμήμα: αποτυχίες 39 (brute) / 39 (κανόνας) / **7 (κίνηση)**, inliers 88 / 92 / 171, σφάλματα > 5°
  18 / 62 / 14· christ-church-03 / bodleian ίδια με τον κανόνα (0.437 / 0.784°).
- **Πλήρης μέθοδος, 4 σπόροι (APE m):**

| | μέθοδος | κανόνας #088 | κίνηση | **υβριδικό** | DLO |
|---|---|---|---|---|---|
| dynamic_spinning | 0.504 ± 0.263 | 0.171 ± 0.091 | **0.094 ± 0.007** | 0.134 ± 0.034 | 4.450 |
| Office_Mitte_1 | 0.241 ± 0.060 | **0.185 ± 0.054** | 0.315 ± 0.040 | 0.206 ± 0.057 | 0.120 |
| stairs | 2.074 ± 0.225 | 1.923 ± 0.415 | 1.855 ± 0.454 | 2.015 ± 0.186 | 0.175 |
| eee_01 | 1.737 ± 0.190 | 1.483 ± 0.149 | 1.502 ± 0.167 | 1.483 ± 0.149 | 0.220 |
| eee_02 | **0.679 ± 0.064** | 0.836 ± 0.038 | 0.810 ± 0.014 | 0.836 ± 0.038 | 0.149 |
| eee_03 | 0.344 ± 0.079 | 0.360 ± 0.073 | 0.319 ± 0.129 | 0.360 ± 0.073 | 0.226 |

#### Συμπέρασμα ⏳
1. Η πρόβλεψη από κίνηση κάνει την καθοδηγούμενη αντιστοίχιση να δουλεύει σε γρήγορη στροφή (dynamic_spinning 0.094 m — καλύτερο κάθε μεθόδου της σύγκρισης
   εκτός Traj-LO 0.080), αλλά χειροτερεύει το Office_Mitte_1 (0.315).
2. Το υβριδικό είναι ασφαλές: ίδιο με τον κανόνα σε αργή κίνηση (NTU ταυτόσημο, Office_Mitte_1 0.206), καλύτερο σε γρήγορη (0.134). **Προτείνεται ως
   προεπιλογή — απόφαση Μ.Τ.**
3. Το NTU (eee_02 +23 % έναντι της μεθόδου) δεν εξαρτάται από τον τρόπο πρόβλεψης· ανοιχτό.

---

### 2026-10-02 — #088 Νέα προεπιλογή: upright SURF + καθοδηγούμενη αντιστοίχιση μόνο σε αργή στροφή — δοκιμή σε 4 δύσκολες· ORB (όχι)

**Σχετικά:** #086, #087 · **ΑΠΟΦΑΣΗ Μ.Τ. 2/10:** upright + καθοδηγούμενη ως προεπιλογή, πρώτα δοκιμή σε 2 μικρές δύσκολες (+ 1 Hilti, 1 NTU), όχι ακόμη
όλες · κλάδος `rotation_bearing` · `config.py`: `surf_upright = True`, `guided_matching_window = 40` (προεπιλογές)· `run_ncd.py --no-upright --guided=none`
(η μέθοδος ως #087)· `intensity_deskew.py`: `GUIDED_SHIFT` / `GUIDED_MAX_SHIFT` / ξανά με brute force σε αποτυχία· `detector: orb` + C++ `guided_match_hamming` ·
launchers `kiss_runs/v2test088{,b,c}_launch.sh` · βραχίονες `surftwofbv2` (σταθερό παράθυρο), `surftwofbv2b` (τελικό) · πίνακες `kiss_runs/results_official_allarms_088.*`

#### Αποτέλεσμα
1. **Πρώτη εκδοχή (σταθερό παράθυρο ±40 στήλες γύρω από την ίδια θέση)**, 4 σπόροι: stairs 1.99 ± 0.50 (μέθοδος 2.07 ± 0.23), Office_Mitte_1 0.223 ± 0.069
   (0.241 ± 0.060), eee_01 1.551 ± 0.187 (1.737 ± 0.190· αποτυχίες εικόνας 409 → 254 / run) — αλλά **dynamic_spinning 5.29 ± 0.55 m (μέθοδος 0.50): αποτυχία.**
   Στο τέλος του dynamic_spinning η στροφή φτάνει 21° / σάρωση (διάμεσος 5.8°)· τα σημεία βγαίνουν από το παράθυρο και η καθοδηγούμενη δίνει λάθος
   αντιστοιχίσεις που ταιριάζουν σε λάθος κίνηση (δεν πιάνονται ως αποτυχία).
2. **Στο γρήγορο τμήμα του dynamic_spinning** (1040 σαρώσεις, σφάλμα στροφής έναντι GT ανά σάρωση): brute force 39 αποτυχίες / 18 σφάλματα > 5°· σταθερό
   παράθυρο 188 / 76· παράθυρο στην προβλεπόμενη θέση (διάμεση μετατόπιση στηλών της προηγούμενης σάρωσης) 165 / 63· + διεύρυνση (40 + |μετατόπιση|/2) +
   ξανά με brute force σε αποτυχία 39 / 68 (οι λάθος «επιτυχημένες» κινήσεις μένουν).
3. **Τελικός κανόνας:** καθοδηγούμενη μόνο όταν |μετατόπιση προηγούμενης| ≤ 20 στήλες (~7° / σάρωση, ~70°/s), αλλιώς brute force· παράθυρο στην
   προβλεπόμενη θέση, διευρυνόμενο· ξανά με brute force σε αποτυχία. 4 σπόροι: **dynamic_spinning 0.171 ± 0.091 m** (RPE 1 m 14.6 cm / 3.19°· μέθοδος
   0.504 / 20.4 / 3.37· KISS 0.159), **stairs 1.923 ± 0.415** (37.3 cm / 7.04°).
4. **ORB** (5000 σημεία, ακμή / patch 15, Hamming· C++ `guided_match_hamming`), 200 σαρώσεις: σφάλμα στροφής 0.464 / 0.873° έναντι 0.428 / 0.793 του upright
   SURF + καθοδηγούμενης (+8–10 %)· brute force ORB 0.488 / 1.126°. Ταχύτερο υπό ίδιο φόρτο (−19 έως −44 %). Απορρίφθηκε (ακρίβεια)· μένει ως επιλογή.

#### Συμπέρασμα ⏳
1. Η νέα προεπιλογή με τον κανόνα αργής στροφής: dynamic_spinning −66 %, eee_01 −11 %, stairs / Office_Mitte_1 ίδια· στις 8 του #087 στροφή −3.5 %, RTE −2 %·
   πραγματικός χρόνος (12.7 / 10.1 Hz, μετρημένο πριν από τον κανόνα· ο κανόνας αλλάζει μόνο τις γρήγορες σαρώσεις).
2. Η καθοδηγούμενη αντιστοίχιση χωρίς πρόβλεψη είναι επικίνδυνη σε γρήγορη στροφή — η δοκιμή σε δύσκολες ακολουθίες πριν από τις 27 το έπιασε.
3. Επόμενο: όλες οι 27 × 4 με τη νέα προεπιλογή (απόφαση Μ.Τ.)· τα Office_Mitte_1 / eee_01 να ξανατρέξουν με τον τελικό κανόνα.

---

### 2026-10-02 — #087 Στροφή από bearings (όχι), καθοδηγούμενη αντιστοίχιση σε C++ (λίγο καλύτερη), επιτάχυνση: πραγματικός χρόνος (12.7 / 10.1 Hz)

**Σχετικά:** #086 · ΑΠΟΦΑΣΗ Μ.Τ. 2/10: «δοκίμασε τα σε νέο κλάδο» → **κλάδος `rotation_bearing`** (από `after_080`) · νέες επιλογές
`image_deskew.{rotation_from_bearings, guided_matching_window}`, `run_ncd.py --bearings=<m> --guided=<px>` · C++ `kiss_slam/cpp/guided_match.cpp`
(`scripts/build_guided_match.sh` → `kiss_slam/_guided_match*.so`, gitignored· χωρίς αυτό: έκδοση Python) · `lookup_batch`, `_rotmats` στο
`intensity_deskew.py` · launcher `kiss_runs/guided087_launch.sh` · πίνακες `kiss_runs/results_official_allarms_087.{md,csv}`

#### Στόχος
Δύο ιδέες για το κενό στροφής του #086 (το σφάλμα της κίνησης της εικόνας): (α) στροφή από τις κατευθύνσεις (bearings) των αντιστοιχίσεων, ανεξάρτητη
από το βάθος· (β) καθοδηγούμενη αντιστοίχιση (μόνο σε παράθυρο γύρω από την ίδια θέση στο πανόραμα). Μετά, ζήτημα Μ.Τ.: έκδοση C++ και ταχύτητα.

#### Μέθοδος και αποτέλεσμα
- **Έλεγχος 200 σαρώσεων** (christ-church-03 / bodleian-02, upright SURF, σφάλμα στροφής έναντι GT ανά σάρωση): bearings (> 5 m, μόνο οι παράμετροι
  στροφής, soft_l1) 0.436 → 0.483° / 0.824 → 0.804° — **χειρότερο / ίσο, απορρίφθηκε**· καθοδηγούμενη (±40 στήλες, ±4 rings) inliers +68 / +53 %, σφάλμα
  −5 %. C++ (πλέγμα κελιών, OpenMP, ορθογώνιο παράθυρο, όλοι οι υποψήφιοι): αντιστοίχιση 18.6 → 6.4 ms (×3), σφάλμα 0.428 / 0.793°.
- **Πλήρης μέθοδος, 8 ακολουθίες × 2 σπόροι** (upright + καθοδηγούμενη έναντι μεθόδου): RPE 1 m στροφή 0.934 → 0.876 (8–0, −3.5 %, p = 0.008), RPE 1 s
  στροφή 7–1 (−3.8 %), μετατόπιση 7–1 (−3.3 %), RTE 7–1 (−2.0 %, p = 0.02), APE 6–2 (−4.8 %). Έναντι upright μόνο: −1.5 % στροφή, 6–7 / 8, μη σημαντικό.
  DLO 0.595, oracle 0.530.
- **Ανάλυση του σφάλματος στροφής** (καταγεγραμμένες κινήσεις upright, 8 ακολουθίες): κατά μήκος του άξονα της αληθινής στροφής 0.19–0.46°, κάθετα 0.31–0.55°
  — περίπου ισοτροπικό, όχι σφάλμα κλίμακας / χρόνου· δεν αυξάνεται με την ταχύτητα, εκτός underground_hard (υποεκτίμηση στις γρήγορες, r = 0.84)·
  σε 5 / 8 ακολουθίες κυρίως γύρω από τον κατακόρυφο (yaw 67–79 %). **Φίλτρο «κολλημένων» σημείων παντού** (όχι μόνο δάπεδο): καμία αλλαγή (ίδια ως το
  τελευταίο ψηφίο) — δεν επιβιώνουν του RANSAC.
- **Επιτάχυνση:** lookup διανυσματικό (`lookup_batch`, ίδια αποτελέσματα με το ανά σημείο ως τη στρογγυλοποίηση: ≤ 2.4·10⁻⁷ s σε απόλυτους χρόνους,
  < 10⁻⁹ m)· στροφές με Rodrigues αντί για αντικείμενα scipy στην προσαρμογή (διαφορά 2·10⁻¹⁶). Βήμα εικόνας (upright + καθοδηγούμενη): 76.5 → 69.9 ms.
- **Χρόνος ρολογιού** (400 σαρώσεις, `--parallel`, αδρανές μηχάνημα· christ-church-03 από το ασυμπίεστο `_lio_bags/church_03.bag` — η ανάγνωση του bz2
  bag μόνη 73 ms / σάρωση):

| | christ-church-03 (Hesai) | math_easy (Ouster 128) |
|---|---|---|
| μέθοδος (πρωί 2/10) | 8.4 Hz (bz2) | 7.0 Hz |
| κανονικό SURF + νέο lookup / προσαρμογή | 8.8 | — |
| upright | 12.2 | 9.9 |
| **upright + καθοδηγούμενη (C++)** | **12.7** | **10.1** |

#### Συμπέρασμα ⏳
1. **Πραγματικός χρόνος:** upright + καθοδηγούμενη + επιταχύνσεις → 12.7 / 10.1 Hz, με λίγο καλύτερη ακρίβεια (στροφή −3.5 %, RTE −2 %). Υποψήφια νέα
   προεπιλογή — απόφαση Μ.Τ.· να επιβεβαιωθεί στις 27 × 4.
2. **Το κενό στροφής έναντι DLO μένει** (0.88 έναντι 0.60 °/m· oracle 0.53). Αποκλείστηκαν: έλλειψη αντιστοιχίσεων, θόρυβος βάθους (bearings), σφάλμα
   κλίμακας / χρόνου (εκτός γρήγορων περιστροφών), κολλημένα σημεία. Ανοιχτό· το yaw κυριαρχεί — ίσως η σύνδεση στήλης–χρόνου του πανοράματος.
3. Για το paper: οι χρόνοι από ασυμπίεστα δεδομένα (ή χωρίς την ανάγνωση)· τα bz2 του Spires προσθέτουν 73 ms.

---

### 2026-10-02 — #086 Το κενό στροφής: η ακρίβεια της κίνησης της εικόνας (oracle), όχι ο ICP· τέσσερις διορθώσεις χειρότερες· upright SURF· διόρθωση ταχύτητας (~8 Hz)

**Σχετικά:** #068, #082, #085 (DLO) · ΑΠΟΦΑΣΗ Μ.Τ. 1/10: διερεύνηση του κενού στροφής (άρση της παύσης για αυτό) · κλάδος `after_080` · νέες επιλογές
`image_deskew.{deskew_rotation, redeskew, deskew_motion_file}`, `run_ncd.py --deskew-rotation=cv --redeskew --model=cv --oracle-deskew=<npz> --surf-upright`·
καταγραφή `image_motions.npz` σε κάθε run · scripts `make_oracle_motion.py`, `analyse_image_motion.py` · launchers `kiss_runs/{rot086,oracle086,upright086}_launch.sh` ·
πίνακες `kiss_runs/results_official_allarms_086.{md,csv}`, `kiss_runs/comparisons_086.txt`

#### Στόχος
Έναντι του DLO (#085) η τοπική στροφή μας είναι χειρότερη (RPE 1 m 1.08 έναντι 0.66 °/m, 5–12). Στο #082 η στροφή ήταν καλύτερη χωρίς deskew → υπόθεση:
ο θόρυβος στροφής της εικόνας (#068) περνά από το deskew.

#### Μέθοδος
8 ακολουθίες NCD / Spires (quad_easy, math_easy, cloister, underground_hard, christ-church-02/03, keble-03, bodleian-02) × 2 σπόροι, μία αλλαγή τη
φορά έναντι της μεθόδου (4 σπόροι). (α) Τέσσερις διορθώσεις: μοντέλο cv αντί car· deskew με μετατόπιση εικόνας + στροφή σταθερής ταχύτητας· δεύτερο
deskew με την κίνηση του ICP· εξομάλυνση στροφής 3 σαρώσεων (#047). (β) **Oracle:** deskew από το GT (κίνηση κατά τη σάρωση, πρώτο → τελευταίο σημείο,
`make_oracle_motion.py`, GT του `load_gt`), αρχή ICP / δύο αρχές / χάρτης αμετάβλητα. (γ) Η κίνηση της εικόνας ανά σάρωση έναντι του GT
(`analyse_image_motion.py`, 3 ακολουθίες). (δ) Profiling του βήματος της εικόνας· χρόνος ρολογιού. (ε) Upright SURF. Έλεγχος: η μέθοδος με τον νέο κώδικα
αναπαράγει το προηγούμενο run ακριβώς (christ-church-03: 0.044 m, 3.82 cm, 0.710 °/m).

#### Αποτέλεσμα
**(α) Όλες οι διορθώσεις χειρότερες** (έναντι μεθόδου, 8 ακολουθίες): στροφή σταθ. ταχύτητας RPE στροφής +98 % (0–8), APE +40 %· δεύτερο deskew +18 % (1–7),
+23 %· εξομάλυνση +55 % (0–8), +36 %· μοντέλο cv +31 % (0–8), +26 %.

**(β) Oracle** (διάμεσοι των 8):

| | RPE 1 m στροφή [°/m] | RPE 1 s στροφή | RPE 1 m μετατόπιση [cm] | RTE [%] | APE [m] |
|---|---|---|---|---|---|
| μέθοδος | 0.934 | 0.902 | 6.12 | 0.31 | 0.101 |
| χωρίς deskew (#082) | 0.723 | 0.657 | 10.11 | 0.44 | 0.141 |
| DLO | 0.595 | 0.522 | 6.31 | 0.49 | 0.262 |
| **oracle deskew** | **0.530** | **0.522** | **5.28** | 0.32 | 0.147 |
| upright SURF | 0.903 | 0.862 | 6.14 | 0.31 | 0.098 |

**(γ) Σφάλμα της κίνησης της εικόνας** (έναντι GT ανά σάρωση): christ-church-03 διάμεσος 0.48° (στροφή ανά σάρωση 1.68°), bodleian 0.85°, math_easy 0.83°·
**χωρίς bias** (< 0.1°), μέγεθος σωστό στο Hesai (1.05 / 0.98 μετατόπιση), ~8–15 % μικρότερο στο Ouster 128 (ανθεκτικές μετρήσεις· οι ανά άξονα κλίσεις
ελαχίστων τετραγώνων 0.05–0.77 στο math_easy ήταν τεχνούργημα λίγων ακραίων σαρώσεων). Μικρότερο σφάλμα σε σαρώσεις με πολλά inliers (0.11° > 400), αλλά
**όχι αιτιακά**: το upright SURF ανεβάζει τα inliers +30 % και το σφάλμα μένει (0.48 → 0.48°, 0.85 → 0.83°).

**(δ) Χρόνος:** το βήμα της εικόνας 106 ms / σάρωση (christ-church-03), 162 ms (math_easy): SURF ~55 %, αντιστοίχιση ~15 %, προσαρμογή κίνησης ~14 %,
πανόραμα ~8 %, lookup (βρόχος Python) 5–13 %. **Χρόνος ρολογιού της μεθόδου 8.2 Hz** στο christ-church-03 (400 σαρώσεις, `--parallel`, αδρανές μηχάνημα),
ενώ το pipeline αναφέρει 27 Hz: η «Average Frequency» μετρά μόνο τον ICP. **Τα 32 / 6 / 26 Hz του #080 ήταν λάθος**· διορθώθηκαν STATUS, open_tasks, σύνοψη Λ.Γ.
Χαμηλότερο κατώφλι SURF: λίγα επιπλέον inliers (+3–5 %), +5–25 % χρόνος — απορρίφθηκε.

**(ε) Upright SURF** (χωρίς προσανατολισμό σημείων — τα πανοράματα δεν περιστρέφονται): −26 έως −30 % χρόνος, +22–31 % inliers· στη μέθοδο στροφή 8–0
(−2.7 %, p = 0.008), APE 6–2 (−3.3 %), μετατόπιση 6–2, RTE 4–4.

#### Συμπέρασμα ⏳
1. **Ο ICP δεν είναι το όριο:** με τέλειο deskew η ίδια καταχώριση ξεπερνά το DLO (στροφή 0.53 έναντι 0.60, μετατόπιση 5.3 έναντι 6.3 cm). **Το όριο είναι
   η ακρίβεια της κίνησης της εικόνας μέσα στη σάρωση** — αξίζει έως ~40 % στη στροφή.
2. Η στροφή της εικόνας είναι η καλύτερη διαθέσιμη για deskew (κάθε υποκατάστατο χειρότερο)· το σφάλμα της είναι θόρυβος χωρίς bias, όχι έλλειψη αντιστοιχίσεων.
3. Πιθανή αιτία: η κίνηση υπολογίζεται από αντιστοιχίσεις 3D–3D (θόρυβος απόστασης, λάθος βάθος σε ακμές). **Επόμενη ιδέα:** στροφή από τις κατευθύνσεις
   (bearings) των αντιστοιχίσεων — ανεξάρτητη από το βάθος — και μετατόπιση από τα 3D.
4. **Upright SURF:** δωρεάν βελτίωση (ταχύτερο και λίγο ακριβέστερο)· προς απόφαση Μ.Τ. για προεπιλογή.
5. **Ταχύτητα: ~8 Hz, όχι πραγματικός χρόνος** στο Hesai· το «γρήγορη» δεν είναι επιχείρημα για το paper μέχρι να επιταχυνθεί η εικόνα.

---

### 2026-10-02 — #085 KITTI: οι σαρώσεις είναι διορθωμένες (διόρθωση του #084) + διόρθωση γωνίας· Boreas (όχημα, raw)· DLO στις 26

**Σχετικά:** [#084](#2026-10-01--084-kitti-odometry-07-όχημα-η-μέθοδος-χειρότερη-από-τον-kiss-slam--σε-ομαλή-κίνηση-η-σταθερή-ταχύτητα-αρκεί) (διορθώνεται εδώ), #082, #083 ·
κλάδος `after_080` · `kiss_slam/tools/kitti_raw.py` (`correct=`), `kiss_slam/tools/boreas.py`, `scripts/boreas_gt.py`, `scripts/comparison_table.py`,
`baselines/lio_docker` (DLO, εικόνα `kiss-lio:3`), `scripts/lio_to_tum.py --raw-scan` · launchers `kiss_runs/{kitti085,kitti085c,boreas085,dlo085}_launch.sh` ·
πίνακες `kiss_runs/results_official_allarms_085.{md,csv}`, `docs/results_comparison_085.md`

#### 1. KITTI — διόρθωση του #084
- **Δοκιμή ραφής:** μια σάρωση αρχίζει και τελειώνει κοιτάζοντας πίσω, 0.1 s μετά· αν δεν είναι διορθωμένη, το έδαφος πίσω από το όχημα
  εμφανίζεται πιο μακριά κατά v · 0.1 s στη μία πλευρά της ραφής (~1 m στα 10 m/s). Μετρήθηκε σε 44 σαρώσεις του 07 (κάτω lasers, οριζόντια
  απόσταση εκατέρωθεν), έναντι της ταχύτητας του GT: κλίση **0.0014 s πίσω, 0.0008 s μπροστά** (αδιόρθωτη: 0.104 s). **Οι σαρώσεις του KITTI
  (sync = extract) είναι διορθωμένες ως προς την κίνηση.** Η πρόταση του #084 «καμία δεν είναι διορθωμένη» ήταν λάθος (συμπέρασμα από το ότι το
  deskew του KISS μείωνε το APE, χωρίς μέτρηση).
- **Η μέθοδος χωρίς deskew** (`--deskew=false`, #082): RTE 0.78–0.81 %, RPE 8.7 cm / 0.096 °/m — ίδια με τον KISS χωρίς deskew (0.76 %, 8.4 / 0.097).
- **Διόρθωση κατακόρυφης γωνίας** του Velodyne του KITTI (0.205°, `kiss_icp_pybind._correct_kitti_scan`, όπως IMLS-SLAM / CT-ICP / KISS-ICP·
  `run_ncd.py --kitti-correction`· αζιμούθιο / ring / χρόνος αμετάβλητα): RTE KISS-SLAM 0.54 → **0.42**, μέθοδος 0.77 → 0.65, μέθοδος χωρίς deskew
  0.79 → 0.69, KISS χωρίς deskew 0.76 → 0.71 %. Η σειρά ίδια. Ανοιχτό: γιατί το deskew του KISS μειώνει το drift (0.42 έναντι 0.71) ενώ χειροτερεύει
  την τοπική στροφή (0.198 έναντι 0.093 °/m), αφού οι σαρώσεις είναι ήδη διορθωμένες.

#### 2. Boreas (όχημα, raw σαρώσεις)
- `boreas-2021-01-26-11-22`, οι πρώτες 3000 σαρώσεις (311 s, 1.34 km· Velodyne Alpha Prime 128 δέσμες· δημόσιο AWS, χωρίς εγγραφή). Ring και χρόνος
  σημείου στα αρχεία· GT `applanix/lidar_poses.csv` στο πλαίσιο του LiDAR· σύμβαση στροφής **επιλέχθηκε από τα δεδομένα** (η ταχύτητα GNSS στο
  πλαίσιο του LiDAR κρατά μία κατεύθυνση, διασπορά 0.6°, ~45.7° ≈ τα 42.6° του `T_applanix_lidar`). Δοκιμή ραφής: κλίση 0.087 s (θορυβώδης) → raw.

| Boreas | RTE [%] | RRE [°/100 m] | APE [m] | RPE 1 m [cm] / [°] |
|---|---|---|---|---|
| KISS-SLAM | **0.28** | **0.15** | **0.27** | **4.1** / 0.19 |
| μέθοδος (s0–s3) | 0.36–0.37 | 0.19–0.20 | 0.27–0.28 | 6.3–6.6 / 0.24 |
| μέθοδος χωρίς deskew | 2.6–2.9 | 0.57–0.63 | 6.1–6.5 | 4.2–4.4 / 0.07 |
| KISS χωρίς deskew | 3.13 | 0.69 | 7.13 | 4.2 / 0.07 |

#### 3. DLO (Direct LiDAR Odometry, RA-L 2022)
- Η ρύθμιση των συγγραφέων με `imu: false` (σύγκριση μόνο LiDAR)· μέσω ROS / Docker, bags σε πραγματικό χρόνο· 26 / 27 (01_short μόνο .pcd).
  Χρόνος θέσης: σάρωση χωρίς deskew → μέσος χρόνος (`--raw-scan`).
- Έναντι της δικής μας odometry (26 ακολουθίες): APE 16–10 υπέρ μας (p = 0.42), RPE 1 m 8–9 (0.46), **RTE 11–4 υπέρ μας (0.04)**, **στροφή 5–12
  υπέρ DLO (0.03)**. Ανά dataset: Spires 6–0, Hilti 4–2, NCD 2021 5–4 υπέρ μας· **NTU 0–3 υπέρ DLO**. Μεγαλύτερες διαφορές: dynamic_spinning
  0.50 / 4.45, underground_hard 0.09 / 0.57 (υπέρ μας)· stairs 2.07 / 0.18, eee_01 1.74 / 0.22 (υπέρ DLO). **Χειρότερο DLO 4.45 m — καμία αποτυχία.**
- Πλήρης πίνακας όλων των μεθόδων ανά ακολουθία: `docs/results_comparison_085.md`.

#### Συμπέρασμα ⏳
1. Το KITTI είναι διορθωμένο ως προς την κίνηση: μόνο έλεγχος λογικής, όχι στόχος· εκεί το deskew μας πρέπει να είναι κλειστό.
2. Σε όχημα (KITTI, Boreas) η μέθοδος ≈ KISS ή λίγο χειρότερη· τα κέρδη μας είναι φαινόμενο φορητής μονάδας.
3. Το DLO είναι η πιο σχετική σύγκριση μόνο LiDAR: ίδιας κατηγορίας, ισάξιο· εμείς λιγότερο drift και καλύτεροι σε χειροκίνητα, εκείνο καλύτερη
   τοπική στροφή και NTU (16 δέσμες). Το «μόνοι χωρίς αποτυχία» δεν ισχύει πια — «μικρότερο χειρότερο σφάλμα» (2.07 έναντι 4.45 m).

---

### 2026-10-01 — #084 KITTI odometry 07 (όχημα): η μέθοδος χειρότερη από τον KISS-SLAM — σε ομαλή κίνηση η σταθερή ταχύτητα αρκεί

**Σχετικά:** #045 (deskew σταθερής ταχύτητας σε φορητή μονάδα), #083 · κλάδος `after_080` · reader `kiss_slam/tools/kitti_raw.py`, GT `scripts/kitti_gt.py`,
`run_ncd.py` (φάκελος με `velodyne_points/`, `--first/--last`, `--intensity-scale=255`) · δεδομένα `/media/photogrammetry/Extreme SSD/kitti/`
(raw drive 2011_09_30_0027 sync + extract, calib, `data_odometry_poses.zip`· από το επίσημο s3 του KITTI, CC BY-NC-SA) · launcher
`kiss_runs/kitti084_launch.sh` · runs `~/kiss_runs_ssd/kitti/07_{sync,raw}/`

#### Στόχος
Ζήτημα Μ.Τ. 1/10: έλεγχος σε KITTI (Velodyne HDL-64E σε όχημα), το dataset που αναφέρουν σχεδόν όλα τα papers LiDAR odometry.

#### Μέθοδος
- **Δεδομένα:** το odometry 07 (694.7 m, 1101 σαρώσεις, αστικό) = raw drive 2011_09_30_0027, σαρώσεις sync 0–1100· και οι 1111 σαρώσεις extract.
- **Ring / χρόνος σημείου** (το KITTI δεν τα αποθηκεύει): τα σημεία είναι αποθηκευμένα laser προς laser (πάνω laser +2.8° πρώτο, αζιμούθιο από
  το μπροστά, βήμα +0.18°) → νέο ring όπου το αζιμούθιο 0–360° από το μπροστά πέφτει > 180° (64 rings ακριβώς, −23.6° το κάτω)· χρόνος =
  έναρξη + (180° − yaw)/360° · (λήξη − έναρξη), η σύμβαση του `kiss_icp` (`KITTIRawDataset.get_timestamps`) με τους μετρημένους χρόνους κάθε σάρωσης.
- **GT:** οι θέσεις του odometry 07 (κάμερα 0) στο πλαίσιο του Velodyne, T = Tr⁻¹ P Tr, Tr = R_rect_00 · T_cam0_velo, χρόνος = `timestamps.txt` των sync
  (μέση σάρωσης)· μήκος 694.4 m.
- Βραχίονες: η μέθοδος (δύο αρχές + εφεδρεία) × 4 σπόροι, KISS-SLAM, KISS χωρίς deskew· ρύθμιση όπως παντού (`threads4.yaml`). Επίσημη αξιολόγηση
  (#061) + RTE / RRE (#083). Η κίνηση της εικόνας υπολογίζεται σε 1100 / 1101 σαρώσεις.

#### Αποτέλεσμα

| | RTE [%] | RRE [°/100 m] | APE [m] | RPE 1 m [cm] | RPE 1 m [°] |
|---|---|---|---|---|---|
| μέθοδος (sync, s0–s3) | 0.74–0.77 | 0.46–0.50 | 0.72–0.84 | 12.3–12.6 | 0.32–0.33 |
| μέθοδος (extract, s0–s3) | 0.76–0.84 | 0.48–0.53 | 0.66–0.79 | 12.5–12.6 | 0.32–0.35 |
| KISS-SLAM (sync / extract) | **0.54 / 0.53** | **0.34 / 0.34** | **0.40 / 0.41** | 9.1 / 9.1 | 0.20 / 0.20 |
| KISS χωρίς deskew | 0.76 / 0.75 | 0.35 / 0.34 | 0.61 / 0.62 | **8.4 / 8.5** | **0.10 / 0.09** |

**Sync = extract:** οι σαρώσεις sync και extract της ίδιας στιγμής είναι ίδιες σημείο προς σημείο (ίδιος αριθμός, 0.0 mm)· το extract έχει μόνο
λίγες σαρώσεις επιπλέον στα άκρα. Καμία δεν είναι διορθωμένη ως προς την κίνηση (το deskew του KISS βελτιώνει και τις δύο, APE 0.61 → 0.40).
Η αρχική περιγραφή «sync = διορθωμένες» (στον reader, στη συζήτηση με τον Μ.Τ.) ήταν λάθος· διορθώθηκε. Άρα ένα αποτέλεσμα, μετρημένο δύο φορές.

#### Συμπέρασμα ⏳
1. **Σε όχημα η μέθοδος χειροτερεύει τον KISS-SLAM** (RTE +40 %, APE ×1.9, RPE +37 %). Η κίνηση είναι ομαλή, η πρόβλεψη σταθερής ταχύτητας του KISS
   σωστή, και το deskew του KISS δουλεύει (0.61 → 0.40 m)· η κίνηση της εικόνας δεν φέρνει πληροφορία, μόνο τον θόρυβό της (ιδίως στροφής,
   RPE στροφής 0.33 έναντι 0.20°/m — ο θόρυβος του #068 / #082).
2. **Αντίθετα από τη φορητή μονάδα** (#045: εκεί το deskew σταθερής ταχύτητας είναι χειρότερο από καθόλου). Για το paper: πεδίο εφαρμογής —
   χειροκίνητες / απότομες κινήσεις· σε όχημα ο KISS-SLAM αρκεί. Πιθανή βελτίωση (σε παύση): χρήση της κίνησης της εικόνας μόνο όταν διαφωνεί
   έντονα με τη σταθερή ταχύτητα.

---

### 2026-10-01 — #083 Σύγκριση με άλλες μεθόδους στις 27: λιγότερο ακριβής από Traj-LO / FAST-LIO2 / COIN-LIO, ισάξια με GenZ-ICP / CT-ICP, η μόνη χωρίς καμία αποτυχία

**Σχετικά:** #061 (επίσημο πρωτόκολλο), #081–#082 · κλάδος `after_080` · στήσιμο και patches: `baselines/README.md` · scripts
`scripts/run_baseline.py`, `scripts/lio_to_tum.py`, `scripts/extract_lio_topics.py` · launchers `kiss_runs/{baselines083,cticp083,cticprobust083,lio083,lio083b}_launch.sh`
· runs `~/kiss_runs_ssd/<ακολουθία>/{genz,mad,trajlo,cticp,cticpdriving,fastlio,fastlioblind1,coinlio,surftworangefbodo}_s0` · πίνακες
`kiss_runs/results_official_allarms_083.{md,csv}`, `kiss_runs/comparisons_083.txt` · σύνοψη `docs/summary_for_LG_2026-10-01.md`

#### Στόχος
Για το paper (open_tasks §Δ, πρώτο κενό): άλλες μέθοδοι στα ίδια δεδομένα με το ίδιο πρωτόκολλο. Μόνο LiDAR: GenZ-ICP (RA-L 2025),
MAD-ICP (RA-L 2024), CT-ICP (ICRA 2022) και Traj-LO (RA-L 2024) — οι δύο τελευταίες συνεχούς χρόνου, οι πραγματικοί ανταγωνιστές
(`literature_i3.md`). Ζήτημα Μ.Τ. 1/10: και μέθοδοι με IMU ως αναφορά — FAST-LIO2 και COIN-LIO (ICRA 2024, intensity + IMU, το πλησιέστερο
προηγούμενο έργο).

#### Μέθοδος
- **Ρυθμίσεις:** η δημοσιευμένη ρύθμιση των συγγραφέων για κάθε dataset (GenZ-ICP pretuned· MAD-ICP dataset cfg· Traj-LO `config_{ouster,ntu,hesai}`·
  CT-ICP profile· FAST-LIO2 `ouster64` / `velodyne` με τη βαθμονόμηση LiDAR–IMU κάθε dataset: `/tf_static` του NCD 2020, `configs/sensor.yaml`
  του Spires, NTU· COIN-LIO `mapping_newer_college.launch`). Όπου η ρύθμιση αποτυγχάνει προφανώς, δεύτερη ρύθμιση των ίδιων συγγραφέων: CT-ICP
  driving (11.8 Hz) → robust_low_inertia (αποτυγχάνει το driving σε σχεδόν όλες τις χειροκίνητες: 01_short 464 m, math_medium 488 m)·
  FAST-LIO2 stairs blind 4 → 1 m.
- **Ίδια είσοδος:** GenZ-ICP, MAD-ICP, CT-ICP διαβάζουν από τους δικούς μας readers (stamp κεφαλίδας, πλαίσιο LiDAR)· Traj-LO και οι LIO από τα bags
  (Traj-LO: χρόνος σημείου Ouster = header + t όπως ο δικός μας reader, όχι header − 0.1 s). Χρόνος κάθε θέσης για το #061: GenZ / MAD χωρίς
  deskew → μέσος χρόνος σάρωσης (`config.yml` deskew false, όπως ο «KISS χωρίς deskew»)· Traj-LO, CT-ICP, LIO → ακριβής χρόνος
  (`*_poses_posetime_tum.txt`). LIO: θέση IMU → θέση LiDAR με τον εξωτερικό προσανατολισμό του config.
- **Δική μας για δίκαιη σύγκριση:** όλες οι άλλες είναι odometry μόνο → και η δική μας χωρίς κλεισίματα βρόχου (`replay_backend.py <run> none`,
  βραχίονας `surftworangefbodo`, 108 runs)· διαφέρει από το SLAM μόνο σε 01_short και long experiment.
- **Τεχνικά:** Traj-LO, CT-ICP, FAST-LIO2 με μικρά patches (headless runner, stream από pipe, Hesai reader· `baselines/patches/`). Spires / Hilti
  bz2 → πρώτα μόνο τα topics LiDAR + IMU (το `rosbag play` δεν προλάβαινε, το FAST-LIO2 δεν έπαιρνε σαρώσεις). Runs σε μονάδες systemd με
  όριο μνήμης (το Traj-LO στα 46 GB σκότωσε στις 30/9 όλες τις διεργασίες της εφαρμογής· `OOMPolicy=continue` χρειάζεται, αλλιώς σταματά όλη η μονάδα).
- **Νέα μετρική RTE / RRE** (KITTI, `kiss_icp.metrics.sequence_error`, 100–800 m, % και °/100 m) στο `evaluate_official.py`· μόνο σε διαδρομή
  ≥ 100 m (16 ακολουθίες NCD / Spires). **Διόρθωση NTU:** σε πυκνή εκτίμηση (Traj-LO, ανά 40 ms) το evo έχανε θέσεις → μία θέση ανά δείγμα GT,
  **μόνο όταν αποτυγχάνει η επίσημη αντιστοίχιση**· μια πρώτη εκδοχή το εφάρμοζε πάντα και μετακινούσε το NTU APE ως 0.4 % — διορθώθηκε,
  3992 τιμές ίδιες με τον πίνακα του #082. Τροχιά που δεν κινείται (CT-ICP driving, eee_01) → αποτυχία, όχι κατάρρευση του πίνακα.

#### Αποτέλεσμα

| μέθοδος | διάμεσος APE [m] | διάμεσος RPE 1 m [cm] | διάμεσος RTE [%] / RRE [°/100 m] | ακολουθίες | αποτυχίες (APE > 5 m) |
|---|---|---|---|---|---|
| **δική μας** (SLAM / odometry) | 0.19 / 0.19 | 8.1 | 0.43 / 1.07 | 27 | **0** (χειρότερη stairs 2.07) |
| KISS-SLAM | 0.42 | 21.4 | 0.92 / 2.38 | 27 | 3 |
| GenZ-ICP | 0.15 | 8.0 | 0.38 / 0.91 | 27 | 1 (dynamic_spinning 15.6) |
| MAD-ICP | 0.45 | 5.2 | 0.55 / 1.02 | 26 (segfault στο long) | 3 |
| CT-ICP (robust) | 0.13 | 5.9 | 0.45 / 1.16 | 27 | 3 (christ-church-02 21.8, underground_hard 9.6, dynamic_spinning 10.8) |
| Traj-LO | 0.07 | 2.5 | 0.20 / 0.74 | 25 (01_short .pcd· long > 46 GB) | 1 (Office_Mitte_1 1632) |
| FAST-LIO2 (IMU) | 0.08 | 2.4 | 0.23 / 0.88 | 26 (01_short χωρίς bag) | 1 (stairs 733· blind 1 m: 97) |
| COIN-LIO (IMU) | 0.05 | 2.7 | 0.23 / 1.02 | 9 (μόνο Ouster OS0-128, NCD 2021) | 0 |

Ζεύγη Wilcoxon, η δική μας odometry έναντι (νίκες δικής μας – άλλης, p· έναντι KISS-SLAM το SLAM):

| έναντι | APE | RPE 1 m | RTE |
|---|---|---|---|
| KISS-SLAM | 25–2, −42 %, < 0.0001 | 17–1 | 16–0, −45 % |
| GenZ-ICP | 12–15, 0.90 | 10–8, 0.52 | 6–10, +23 %, 0.02 |
| MAD-ICP | 20–6, −57 %, 0.002 | 4–13, 0.15 | 11–4, 0.08 |
| CT-ICP | 10–17, 0.56 | 5–13, 0.44 | 5–11, 0.53 |
| Traj-LO | 3–22, +78 %, 0.005 | 0–16, +137 % | 0–14, +54 % |
| FAST-LIO2 | 3–23, +78 %, 0.0004 | 1–16 | 0–15, +36 % |
| COIN-LIO | 0–9, 0.004 | 0–9 | 0–8 |

Ταχύτητα / μνήμη (όχι σε ίδιες συνθήκες φόρτου): CT-ICP robust 0.4–1.3 Hz με 16 νήματα (18 ώρες στο long experiment), 15–23 GB στις μεγάλες·
Traj-LO > 46 GB στο long experiment· δική μας ~30 Hz, λίγα GB.

#### Συμπέρασμα ⏳
1. **Δεν είμαστε οι ακριβέστεροι:** Traj-LO (συνεχούς χρόνου, μόνο LiDAR) και οι LIO έχουν ~2× μικρότερο RTE / APE και ~3× μικρότερο RPE.
   Ισάξιοι με GenZ-ICP και CT-ICP, σαφώς καλύτεροι από MAD-ICP και KISS-SLAM.
2. **Είμαστε οι μόνοι χωρίς αποτυχία** σε 27 ακολουθίες / 5 datasets / 4 αισθητήρες, και τρέχουμε σε όλες. Κάθε ακριβέστερη μέθοδος που καλύπτει
   όλα τα datasets αποτυγχάνει τουλάχιστον μία φορά ή δεν τρέχει.
3. **Το intensity σώζει τη σκάλα** (stairs): FAST-LIO2 733 / 97 m, COIN-LIO 0.22 m — η αρχική ιδέα του έργου, σε άλλη μέθοδο.
4. **Πρόταση πλαισίου για το paper** (απόφαση Μ.Τ. / Λ.Γ.): ανθεκτικότητα και απλότητα, όχι καλύτερη ακρίβεια (`summary_for_LG_2026-10-01.md` §4–5).

---

### 2026-09-30 — #082 Η κίνηση της εικόνας μόνο ως αρχή του ICP, χωρίς deskew: το 2 × 2 (deskew × αρχή) σε 27 × 4 — η αρχή για την ανθεκτικότητα και τη στροφή, το deskew για τη μετατόπιση

**Σχετικά:** [#081](#2026-09-30--081-ablation-της-μεθόδου-του-paper-δύο-αρχές--εφεδρεία-η-αρχή-του-icp-από-την-εικόνα-μετρά-περισσότερο-φίλτρο-δαπέδου-μικρό-αλλά-σταθερό-σταθερό-σ-ισοπαλία-κατά-μέσο-όρο),
#030, #068 · κλάδος `after_080` · νέα επιλογή `run_ncd.py --deskew=false` (`image_deskew.use_for_deskew`) · βραχίονας `surftwofbnodeskew` · runs
`~/kiss_runs_ssd/<ακολουθία>/surftwofbnodeskew_s0–3` · script `kiss_runs/nodeskew082_launch.sh` · πίνακες `kiss_runs/results_official_allarms_082.{md,csv}`,
`kiss_runs/comparisons_082.txt`

#### Στόχος
Ζήτημα Μ.Τ. 30/9: η κίνηση της εικόνας μόνο ως αρχική θέση του ICP, χωρίς deskew — το αντίστροφο του «μόνο deskew» του #081. Μαζί συμπληρώνουν το 2 × 2
(deskew από την εικόνα ναι / όχι × αρχή του ICP από την εικόνα / σταθερή ταχύτητα) που εξηγεί τι προσφέρει κάθε χρήση.

#### Μέθοδος
Μέθοδος (δύο αρχές + εφεδρεία) με `--deskew=false`: η σάρωση δεν διορθώνεται (δ = I), ο ICP ξεκινά από `last_pose · M`· οι δύο αρχές συγκρίνουν αυτήν με τη
σταθερή ταχύτητα όπως πάντα. Πρώτα 2 ακολουθίες (christ-church-02, underground_hard), μετά όλες οι 27 × 4 σπόροι (108 runs, εξωτερικός SSD). Το batch
διακόπηκε μία φορά (13:36, ο πυρήνας σκότωσε το Traj-LO του #083 στα 46 GB και μαζί όλες τις διεργασίες της εφαρμογής)· συνεχίστηκε σε δική του μονάδα
systemd (`systemd-run --user`, MemoryMax 40G), 108 / 108 με τροχιά. Ζεύγη Wilcoxon ανά ακολουθία.

#### Αποτέλεσμα

Διάμεσοι στις 27 (APE m / RPE 1 m μετατόπιση cm / RPE 1 m στροφή °):

| | αρχή από εικόνα | αρχή από σταθερή ταχύτητα |
|---|---|---|
| **deskew από εικόνα** | **0.188** / **10.65** / 1.84 (μέθοδος) | 0.329 / 12.95 / 2.46 (#081) |
| **χωρίς deskew** | 0.206 / 12.68 / **1.48** (νέο) | 0.436 / 16.52 / 2.22 (KISS χωρίς deskew) |

Ζεύγη (νίκες A – νίκες B, διάμεσος (A−B)/B, p):

| A έναντι B | APE (27) | RPE 1 m μετατόπιση (18) | RPE 1 m στροφή (18) |
|---|---|---|---|
| μόνο αρχή έναντι μεθόδου | 7–20, +31 %, 0.044 | 4–14, +45 %, 0.004 | **12–6, −18 %**, 0.067 (1 s: 13–5, −17 %, 0.048) |
| μόνο αρχή έναντι κανενός | 21–6, −17 %, 0.001 | 18–0, −8 %, < 0.0001 | 17–1, −9 %, 0.0002 |
| μόνο αρχή έναντι μόνο deskew | 13–14, +0.4 %, 0.43 | 9–9, +1 %, 0.77 | **15–3, −34 %, 0.0007** |
| μόνο deskew έναντι κανενός | 19–8, −23 %, 0.12 | 14–4, −29 %, 0.003 | 7–11, +3 %, 0.42 |

christ-church-02 / underground_hard (APE m): μέθοδος 0.207 / 0.086 · μόνο αρχή 0.875 ± 0.295 / 0.122 · μόνο deskew 0.453 / 14.2 · κανένα 0.546 / 12.3.

#### Συμπέρασμα ⏳
1. **Οι δύο χρήσεις είναι συμπληρωματικές.** Η αρχή του ICP αποτρέπει τις αποτυχίες (underground_hard 12.3 → 0.12 m μόνη της) και βελτιώνει τη στροφή· το
   deskew βελτιώνει τη μετατόπιση (RPE −29 έως −31 %, μήκος διαδρομής). Μαζί: η καλύτερη μετατόπιση και το καλύτερο APE.
2. **Το deskew με την κίνηση της εικόνας χειροτερεύει τη στροφή κατά ~17 %** (μόνο αρχή έναντι μεθόδου, 12–6 / 13–5). Είναι ο θόρυβος στροφής ανά σάρωση του
   #068 (§Β.5 του open_tasks), που περνά από το deskew· ποσοτικοποιείται εδώ για πρώτη φορά σε όλες τις 27. Ιδέα για αργότερα (σε παύση): deskew μόνο με τη
   μετατόπιση της εικόνας και τη στροφή του ICP, ή εξομάλυνση της στροφής (#047).
3. Στη χειροκίνητη Hesai (christ-church-02) το deskew είναι το μεγαλύτερο μέρος· στο underground_hard η αρχή — η συνεισφορά εξαρτάται από τη σκηνή.

---

### 2026-09-30 — #081 Ablation της μεθόδου του paper (δύο αρχές + εφεδρεία): η αρχή του ICP από την εικόνα μετρά περισσότερο· φίλτρο δαπέδου μικρό αλλά σταθερό· σταθερό σ ισοπαλία κατά μέσο όρο

**Σχετικά:** #025–#027 (φίλτρο δαπέδου), #030 (deskew και αρχή μαζί), #031 (σταθερό σ), #069 / #078 (εφεδρεία), #080 (`--image-start=false`) · κλάδος `after_080` ·
νέες επιλογές `run_ncd.py --stuck=none|<m>` (`image_deskew.stuck_min`), `--sigma=adaptive|<m>` (`image_deskew.fixed_sigma`) · βραχίονες
`surftwofbnostuck`, `surftwofbadaptive`, `surftwofbcvstart` · runs `~/kiss_runs_ssd/<ακολουθία>/…_s0–3` (εξωτερικός SSD, exFAT) · script
`/home/photogrammetry/kiss_runs/ablation_launch.sh` · πίνακες `kiss_runs/results_official_allarms_081.{md,csv}`, `kiss_runs/comparisons_081.txt`,
`docs/results_ablation_081.md`

#### Στόχος
Για το paper (open_tasks §Δ): ablation ανά συνιστώσα της μεθόδου όπως δημοσιεύεται. **ΑΠΟΦΑΣΗ Μ.Τ. 29/9: η μέθοδος = δύο αρχές + εφεδρεία εικόνας
απόστασης** (`surftworangefb`, #069 / #078). Κάθε βραχίονας αλλάζει ΕΝΑ πράγμα: (α) χωρίς φίλτρο δαπέδου· (β) προσαρμοστικό σ του KISS αντί για
σταθερό 2.0· (γ) η κίνηση της εικόνας μόνο για deskew, ο ICP ξεκινά από σταθερή ταχύτητα. Τα «δύο αρχές on–off» (#059), εφεδρεία (#069), βάρος
στροφής / gain (#070–#079) υπάρχουν ήδη.

#### Μέθοδος
- 27 ακολουθίες × 4 σπόροι × 3 βραχίονες = 324 runs, ρύθμιση όπως στο #079 (`threads4.yaml`, `--two-start=5 --range=fallback` + η αλλαγή), 12 παράλληλα.
  Αξιολόγηση: επίσημο πρωτόκολλο κάθε dataset (`results_table.py --all-arms --runs=/home/photogrammetry/kiss_runs_ssd`), ζεύγη Wilcoxon ανά ακολουθία
  (`compare_arms.py`) έναντι `surftworangefb` (4 σπόροι).
- **Εγγραφή στον εξωτερικό SSD** (`/media/photogrammetry/Extreme SSD/kiss_runs`, σύνδεσμος `~/kiss_runs_ssd`): ο `/` ήταν 100 % (4.2 GB ελεύθερα).
- **Το πρώτο batch χάθηκε ολόκληρο (29/9 17:30 → 30/9 02:33):** το exFAT δεν έχει symbolic links· το `kiss_icp` φτιάχνει τον σύνδεσμο `latest` πριν γράψει
  τα αποτελέσματα, στο τέλος κάθε run → `PermissionError`, καμία τροχιά. Δεν φάνηκε επειδή (1) το πρότυπο των launchers τυπώνει `exit $?` μέσα σε
  `echo "$(date) …"`, όπου το `$(date)` μηδενίζει το `$?` → πάντα «exit 0»· (2) ένα run που κατέρρευσε έχει κι αυτό τη γραμμή `wall` του `/usr/bin/time`.
  Διορθώσεις: `SlamPipeline._get_results_dir` (ο σύνδεσμος `latest` προαιρετικός)· το launcher κρατά `rc=$?` πρώτα και μετρά ένα run ως έτοιμο μόνο αν
  υπάρχει `*_poses_tum.txt`· το `results_table.py` το ίδιο. Δεύτερο batch 30/9 02:36 → 11:42, 324 / 324 με τροχιά.

#### Αποτέλεσμα

Ζεύγη ανά ακολουθία έναντι της μεθόδου (νίκες αλλαγής – νίκες μεθόδου, διάμεσος (A−B)/B, p):

| αλλαγή | APE (27) | RPE 1 s μετατόπιση (18) | RPE 1 s στροφή (18) | z RMSE (18) |
|---|---|---|---|---|
| χωρίς φίλτρο δαπέδου | 7–20, +4.2 %, 0.003 | 3–15, +2.7 %, 0.005 | 3–14, +2.6 %, 0.009 | 4–12, +2.5 %, 0.030 |
| προσαρμοστικό σ | 15–12, −1.8 %, 0.90 | 11–7, −0.2 %, 0.77 | 8–10, +0.0 %, 0.88 | 12–6, −1.7 %, 0.61 |
| εικόνα μόνο για deskew | 6–21, +8.4 %, 0.002 | 2–16, +9.0 %, 0.0001 | 5–13, +6.9 %, 0.014 | 5–13, +37.8 %, 0.015 |

Χαρακτηριστικές ακολουθίες (APE m, μέσος ± σ, 4 σπόροι· όλες στο `docs/results_ablation_081.md`):

| ακολουθία | μέθοδος | χωρίς φίλτρο | προσαρμ. σ | μόνο deskew |
|---|---|---|---|---|
| underground_hard | 0.086 ± 0.003 | 0.089 | 0.083 | **14.215 ± 1.697** |
| keble-college-03 | 0.094 ± 0.001 | 0.101 | 0.093 | **4.426 ± 3.524** |
| IC_Office_1 | 0.071 ± 0.006 | 0.117 ± 0.091 | 0.068 | **7.248 ± 3.610** |
| dynamic_spinning | 0.504 ± 0.263 | 0.547 | 0.451 | **6.103 ± 3.533** |
| christ-church-02 | 0.207 ± 0.044 | 0.248 | 0.185 | 0.453 |
| blenheim-palace-02 | 0.268 ± 0.015 | **0.385** | 0.262 | 0.253 |
| stairs | 2.074 ± 0.225 | 2.332 | 2.200 | 3.399 |
| 02_long_experiment | 1.619 ± 0.761 | 1.695 | **2.302** | 2.244 |
| Office_Mitte_1 | 0.241 ± 0.060 | 0.210 | 0.303 | 1.490 ± 2.185 |
| eee_01 (NTU) | 1.737 ± 0.190 | 1.789 | **1.983** | **0.923 ± 0.201** |
| eee_02 (NTU) | 0.679 ± 0.064 | 0.710 | **1.174** | 0.640 |
| eee_03 (NTU) | 0.344 ± 0.079 | 0.400 | **0.495** | 0.373 |

#### Συμπέρασμα ⏳
1. **Η κίνηση της εικόνας ως αρχή του ICP είναι η σημαντικότερη συνιστώσα.** Μόνο για deskew: χειρότερα σε 21 / 27, με αποτυχίες 4–14 m σε τέσσερις
   ακολουθίες. Επιβεβαιώνει το #030 (deskew και αρχή πρέπει να συμφωνούν) σε 27 ακολουθίες. Στην odometry του long experiment (#080, μία αρχή) η αρχή
   δεν άλλαζε τίποτα — εδώ φαίνεται ότι ο ρόλος της είναι οι δύσκολες στιγμές, όχι το μέσο σφάλμα.
2. **Εξαίρεση το NTU (Ouster 16 δεσμών):** με αρχή σταθερής ταχύτητας 0.92 / 0.64 / 0.37 m έναντι 1.74 / 0.68 / 0.34 (eee_01–03). Η εικόνα 16 γραμμών
   δίνει αναξιόπιστη κίνηση (αποτυγχάνει σε 20–31 % των σαρώσεων, #069)· περιορισμός της μεθόδου για αραιούς αισθητήρες, για το paper.
3. **Το φίλτρο δαπέδου βοηθά λίγο αλλά σταθερά** (+4 % APE χωρίς αυτό, 15 / 18 στο RPE)· μεγαλύτερο στο blenheim (0.27 → 0.39 m) και στο church_02.
4. **Σταθερό σ: ισοπαλία κατά μέσο όρο, προστασία στις δύσκολες.** Το προσαρμοστικό σ του KISS είναι ελαφρά καλύτερο σε πολλές εύκολες (≤ 0.01 m) και
   χειρότερο σε NTU (+14–73 %), long experiment (+42 %) και Office_Mitte_1 (+26 %). Το επιχείρημα του #031 (με καλή αρχή οι διορθώσεις του ICP είναι μικρές,
   το σ συρρικνώνεται και ο ICP δεν μπορεί να διορθώσει μια λάθος κίνηση) στέκει εκεί όπου η κίνηση της εικόνας είναι αναξιόπιστη.
5. **Για τα runs:** κάθε νέος δίσκος δοκιμάζεται με ένα σύντομο run ΩΣ ΤΟ ΤΕΛΟΣ πάνω του· ένα run μετρά μόνο αν έγραψε τροχιά. Τα παλιά launchers στο
   `kiss_runs/` έχουν ακόμη το σφάλμα του «exit 0».

---

### 2026-09-29 — #080 Odometry μόνο: γιατί ο KISS έχει μικρότερο APE στο long experiment — όχι σφάλμα κατεύθυνσης, όχι η αρχή του ICP· ταχύτητα της πλήρους ρύθμισης

**Σχετικά:** [#070](#2026-09-28--070-βάρος-στροφής-100-στον-γράφο-κόμβων-επιβεβαίωση-με-πραγματικά-runs), [#078](#2026-09-29--078-γρήγορη-εφεδρεία-εικόνας-απόστασης-υπολογίζεται-μόνο-όταν-αποτυγχάνει-η-εικόνα-intensity--κόστος-0-όπου-δεν-χρειάζεται),
[#079](#2026-09-29--079-πλήρης-ρύθμιση-σε-όλες-τις-27-δύο-αρχές--gain--γρήγορη-εφεδρεία--βάρος-στροφής-100--23-3-έναντι-χωρίς-deskew--395-) · κλάδος `fast_fallback` ·
νέα επιλογή `run_ncd.py --image-start=false` (= `image_deskew.use_as_initial_guess: false`) · runs `…/{02_long_experiment,newer_college_01_short}/surfcv_s0–3` ·
scripts `/home/photogrammetry/kiss_runs/{cvstart_launch.sh,speed_launch.sh}` · odometry μόνο: `replay_backend.py <run> none` → `kiss_runs/backend_replay/none/`

#### Στόχος
Ερώτηση Μ.Τ. 29/9: (α) πόσο γρήγορη είναι η μέθοδος με την εφεδρεία· (β) πώς εξηγείται ότι η odometry μόνη (χωρίς κλεισίματα βρόχου) του KISS είναι
καλύτερη από τη δική μας στο long experiment. Δύο έλεγχοι για το (β): (1) πού συσσωρεύεται το σφάλμα κατεύθυνσης κατά μήκος της διαδρομής· (2) η ίδια
odometry με το ICP να ξεκινά από τη σταθερή ταχύτητα του KISS (η εικόνα μόνο για το deskew) — αν η απόκλιση πέσει στο επίπεδο του KISS, φταίει η αρχή.

#### Μέθοδος
- **Ταχύτητα:** έξι runs διαδοχικά σε αδρανές μηχάνημα (`--parallel`), δύο αρχές και πλήρης ρύθμιση (#079), church_03 / math_easy / eee_03· «Average
  Frequency / Runtime» του pipeline.
- **Odometry μόνο:** αναπαραγωγή του back-end χωρίς κλεισίματα (`none`). Σφάλμα κατεύθυνσης: ευθυγράμμιση στα πρώτα 30 s, διαφορά yaw
  (R_est · R_gtᵀ) στο πλαίσιο του κόσμου, αφαιρείται ο μέσος των πρώτων 30 s· ανά παράθυρο 60 s πόσο προστίθεται. Επίσης APE οριζόντια / κατακόρυφα, κλίμακα
  (Sim3), πλήθος βημάτων με |Δz| > 0.2 m ανά σάρωση.
- **Αρχή από σταθερή ταχύτητα:** μία αρχή εικόνας (`surf`) με `--image-start=false`, 4 σπόροι, long experiment και 01_short· σύγκριση με `surf`
  (long) / `surfrw` (01_short· το βάρος στροφής δεν αγγίζει την odometry). Δοκιμή πρώτα σε 150 σαρώσεις.

#### Αποτέλεσμα

**Ταχύτητα** (αδρανές μηχάνημα, ένα run τη φορά):

| ακολουθία | δύο αρχές | πλήρης ρύθμιση |
|---|---|---|
| church_03 (Hesai) | 33 Hz (30 ms) | 32 Hz (31 ms) |
| math_easy (NCD 2021, 128 δέσμες) | 7 Hz (137 ms) | 6 Hz (157 ms) |
| eee_03 (NTU, 16 δέσμες) | 32 Hz (31 ms) | 26 Hz (38 ms) |

**Odometry μόνο, long experiment** (APE επίσημο· κατεύθυνση: RMS και τέλος):

| run | APE [m] | κατεύθυνση RMS / τέλος [°] | βήματα \|Δz\| > 0.2 m | κλίμακα Sim3 | SLAM APE [m] |
|---|---|---|---|---|---|
| KISS | 1.092 | 2.24 / +1.35 | 5311 (20 %) | 0.9956 | 1.275 |
| χωρίς deskew | 2.085 | 2.67 / +3.87 | 173 | 0.9949 | 3.498 |
| surf s0–s3 | 1.37 / 1.37 / **14.66 / 14.59** | 2.17–2.31 / +1.9…+2.8 | 58–67 | 0.9957 | 1.97–2.13 |
| **surf, αρχή σταθ. ταχύτητας** s0–s3 | 1.38 / 1.46 / 1.38 / 1.37 | 2.28–2.34 / +2.9…+5.7 | 63–73 | — | 0.63–2.31 |
| πλήρης s0 / s1 | 1.277 / 1.348 | 2.22–2.24 / +2.7 | 49–56 | 0.9957 | 0.381 / 0.374 |

**Odometry μόνο, 01_short:** KISS 0.920, χωρίς deskew 0.770, surf (rw) 0.670–0.692, αρχή σταθ. ταχύτητας 0.661–0.700, πλήρης 0.660–0.676 m·
κατεύθυνση RMS 1.2–1.8° σε όλους.

**Ανά παράθυρο 60 s (long):** το σφάλμα κατεύθυνσης που προστίθεται ανά λεπτό είναι ±2–6° σε όλους, πρόσημο αλλάζει συνεχώς. Η διαφορά «δύο αρχές − KISS»
ανά λεπτό: μέσος +0.05°, τυπική απόκλιση 2.3° (t = 0.14). Η δική μας συσχετίζεται με του «χωρίς deskew» (r = 0.83) περισσότερο απ' ό,τι με του KISS
(0.38)· οι σπόροι μας διαφέρουν ≤ 0.1° ανά λεπτό.

#### Συμπέρασμα ⏳
1. **Δεν υπάρχει συστηματική απόκλιση κατεύθυνσης.** Το σφάλμα κατεύθυνσης κάνει τυχαίο περίπατο με το ίδιο μέγεθος σε όλους (RMS 2.2–2.3°)· η τελική
   διαφορά (+1.35° έναντι +2.7°) είναι μέσα σε αυτό. **Αποσύρεται** η εκτίμηση «0.056 έναντι 0.043°/λεπτό» που δόθηκε προφορικά νωρίτερα: δεν είναι
   σημαντική διαφορά. Ούτε κλίμακα (όλοι 0.996).
2. **Η αρχή του ICP δεν είναι η αιτία.** Με αρχή από σταθερή ταχύτητα η odometry μένει στο 1.37–1.46 m (long) / 0.66–0.70 m (01_short) — ίδια με την αρχή
   εικόνας. Η κίνηση της εικόνας δεν περνά συστηματικό σφάλμα μέσω της αρχής.
3. **Η διαφορά 1.09 έναντι ~1.35 m είναι μία πραγματοποίηση τυχαίου περιπάτου ανά μέθοδο**, όχι ιδιότητα της μεθόδου — οι σπόροι μας δεν τη μετακινούν
   επειδή το σφάλμα καθορίζεται από τα δεδομένα (ίδιο ανά λεπτό σε όλους τους σπόρους). Στο 01_short η δική μας odometry είναι καλύτερη (0.67 έναντι 0.92).
4. **Ο KISS αναπηδά κατακόρυφα στο long experiment:** 20 % των σαρώσεων αλλάζουν ύψος > 0.2 m σε 0.1 s, διαδρομή 5695 έναντι 3067 m (και στο πραγματικό run,
   όχι μόνο στην αναπαραγωγή)· το APE τα απορροφά (μέσος όρος γύρω από το σωστό). Η δική μας τροχιά: 0.2 %.
5. **Η μία αρχή εικόνας χωρίς κλεισίματα ξεφεύγει καθ' ύψος σε 2 / 4 σπόρους** (+63 m από το λεπτό 38)· με κλεισίματα διορθώνεται. Δύο αρχές και πλήρης
   ρύθμιση δεν το κάνουν. Με αρχή σταθερής ταχύτητας 0 / 4. Το SLAM APE της μίας αρχής χωρίς βάρος στροφής κυμαίνεται πολύ (0.63–2.31 m)· είναι το πρόβλημα
   του γράφου του #070, όχι της αρχής.
6. **Ταχύτητα:** η πλήρης ρύθμιση κοστίζει 0 % (church_03), +15 % (math_easy), +23 % (eee_03, όπου η εφεδρεία χρησιμοποιείται συχνά). church_03 και eee_03
   ~3× πραγματικός χρόνος· το math_easy (Ouster 128 δεσμών) 6–7 Hz, **κάτω από τα 10 Hz του αισθητήρα** σε κάθε ρύθμιση.

---

### 2026-09-29 — #079 Πλήρης ρύθμιση σε όλες τις 27: δύο αρχές + gain + γρήγορη εφεδρεία + βάρος στροφής 100 — 23–3 έναντι «χωρίς deskew» (−39.5 %)

**Σχετικά:** [#070](#2026-09-28--070-βάρος-στροφής-100-στον-γράφο-κόμβων-επιβεβαίωση-με-πραγματικά-runs), [#071](#2026-09-28--071-εικόνα-απόστασης-ως-εφεδρεία-σε-όλες-τις-27-ακολουθίες-146-έναντι-των-δύο-αρχών-234-έναντι-του-χωρίς-deskew-συνδυασμός-με-τη-διόρθωση-του-γράφου),
[#076](#2026-09-29--076-αυτόματη-κανονικοποίηση-intensity-ανά-σάρωση-gain-σε-runs-hilti-καλύτερο-σε-4--6-ncd-χωρίς-βλάβη-ntu-μικτό-2048-στήλες-χωρίς-κέρδος),
[#078](#2026-09-29--078-γρήγορη-εφεδρεία-εικόνας-απόστασης-υπολογίζεται-μόνο-όταν-αποτυγχάνει-η-εικόνα-intensity--κόστος-0-όπου-δεν-χρειάζεται) · κλάδος `fast_fallback` ·
runs `…/surftwofull_s0–3` (108, 0 αποτυχίες) · script `/home/photogrammetry/kiss_runs/full_launch.sh` · πίνακας `docs/results_official_allarms.md`

#### Στόχος
Πρόταση Μ.Τ. 29/9: gain και εφεδρεία σε όλες τις ακολουθίες — μπορεί μία ρύθμιση να χρησιμοποιηθεί παντού;

#### Μέθοδος
`--two-start=5 --normalise=gain --range=fallback --rotation-weight=100` (η εφεδρεία στη γρήγορη μορφή του #078), 27 ακολουθίες × 4 σπόροι. Επίσημο πρωτόκολλο
κάθε dataset· σύγκριση ανά ακολουθία.

#### Αποτέλεσμα (επίσημο σκορ [m], 4 σπόροι)

| ακολουθία | KISS | χωρίς deskew | δύο αρχές | + εφεδρεία (#071) | **πλήρης** |
|---|---|---|---|---|---|
| 01_short / quad_easy / stairs | 0.419 / 0.104 / 3.586 | 0.350 / 0.083 / 2.705 | 0.305 / 0.079 / 2.074 | 0.307 / 0.079 / 2.074 | **0.303 / 0.078 / 1.812** |
| cloister / math_easy / underground_easy | 0.396 / 0.160 / 0.117 | 0.480 / 0.104 / 0.092 | 0.188 / 0.108 / 0.067 | **0.188** / 0.108 / 0.067 | 0.243 / **0.104 / 0.066** |
| christ-church-02 / -03 | 0.777 / 0.143 | 0.546 / 0.089 | 0.209 / 0.044 | 0.207 / 0.044 | **0.192 / 0.044** |
| quad_hard / math_medium / underground_medium / underground_hard | 0.329 / 0.253 / 0.162 / 12.78 | **0.209** / 0.164 / 0.097 / 12.34 | 0.222 / 0.154 / 0.061 / 0.086 | 0.218 / **0.152** / 0.061 / **0.086** | 0.215 / **0.152 / 0.059** / 0.087 |
| keble / observatory / blenheim / bodleian | 9.76 / 0.497 / 0.293 / 1.911 | 11.37 / 0.436 / **0.205** / 1.363 | 0.094 / 0.078 / 0.273 / 0.513 | 0.094 / 0.080 / 0.268 / 0.545 | **0.092 / 0.076** / 0.265 / **0.498** |
| NCD long experiment / dynamic_spinning | 1.275 / **0.159** | 3.498 / 20.75 | 2.113 / 0.460 | 1.619 / 0.504 | **0.361** / 0.533 |
| Hilti Basement / IC / Office_Mitte / Construction / LAB / UZH | 0.055 / 6.34 / 4.29 / 0.063 / 0.062 / 0.585 | 0.078 / 1.66 / 0.575 / 0.062 / 0.050 / **0.204** | 0.058 / 0.072 / 1.437 / 0.049 / 0.036 / 0.503 | **0.050 / 0.071** / 0.241 / 0.048 / 0.036 / 0.551 | 0.052 / 0.074 / **0.222 / 0.043 / 0.035** / 0.577 |
| NTU eee_01 / 02 / 03 | 2.68 / 1.49 / 0.86 | 2.36 / 1.49 / 0.84 | 2.03 / 0.82 / 0.59 | **1.74 / 0.68 / 0.34** | 1.76 / 0.84 / 0.38 |

| πλήρης έναντι… (27) | καλύτερη / ίση (±1 %) / χειρότερη | διάμεσος | Wilcoxon p |
|---|---|---|---|
| KISS | 26 / 0 / 1 | −43.3 % | < 0.0001 |
| **KISS χωρίς deskew** | **23 / 1 / 3** | **−39.5 %** | **< 0.0001** |
| δύο αρχές | 17 / 4 / 6 | −2.3 % | 0.032 |
| δύο αρχές + εφεδρεία (#071) | 14 / 4 / 9 | −1.1 % | 0.49 |

#### Συμπέρασμα ⏳
1. **Η πλήρης ρύθμιση είναι η ισχυρότερη μία ρύθμιση ως τώρα:** έναντι του «χωρίς deskew» 23–3 (−39.5 %, p < 0.0001· δύο αρχές −29 %, με εφεδρεία −37 %)·
   καλύτερη σε 18 από 27· το long experiment 2.11 → **0.36 m** (από το βάρος στροφής).
2. **Το gain προσθέτει λίγο συνολικά** (έναντι της εφεδρείας μόνης 14–9, p 0.49): βοηθά Hilti, σκάλα (2.07 → 1.81), church_02, bodleian· **χειροτερεύει το NTU**
   (και τα τρία χειρότερα από την εφεδρεία μόνη, eee_02 0.68 → 0.84) και το cloister (0.19 → 0.24).
3. Πρόταση: **εφεδρεία + βάρος 100 ως ο ασφαλής γενικός πυρήνας**· το gain μόνο για σκοτεινούς αισθητήρες (Hilti). Επιλογές, όχι προεπιλογές (απόφαση Μ.Τ.
   28/9)· προς απόφαση Μ.Τ.

---

### 2026-09-29 — #078 Γρήγορη εφεδρεία εικόνας απόστασης: υπολογίζεται μόνο όταν αποτυγχάνει η εικόνα intensity — κόστος 0 όπου δεν χρειάζεται

**Σχετικά:** [#069](#2026-09-26--069-εικόνα-απόστασης-όπου-αποτυγχάνει-η-εικόνα-intensity-δύο-αρχές-ntu-1542--office_mitte_1-σταθεροποιείται--διόρθωση-του-068),
[#071](#2026-09-28--071-εικόνα-απόστασης-ως-εφεδρεία-σε-όλες-τις-27-ακολουθίες-146-έναντι-των-δύο-αρχών-234-έναντι-του-χωρίς-deskew-συνδυασμός-με-τη-διόρθωση-του-γράφου)
· **κλάδος `fast_fallback`** · `ScanMotionEstimator._range` · runs `…/{surftwotime,surftworangefbfast}_s*` · script `/home/photogrammetry/kiss_runs/fastfb_launch.sh`

#### Στόχος
Ερώτημα Μ.Τ.: η εφεδρεία κάνει τα runs πιο αργά. Μέτρηση (runs των #071 / #065): ×1.14–×1.92, **χειρότερα όπου δεν χρησιμοποιήθηκε καθόλου** (math_easy ×1.92,
quad_easy ×1.75, church_03 ×1.61 με 0 χρήσεις). Αιτία: σε κατάσταση «fallback» το πανόραμα απόστασης, τα σημεία του, οι αντιστοιχίσεις, το RANSAC και το
χρονικό ταίριασμα υπολογίζονταν σε **κάθε** σάρωση και πετιούνταν όταν η εικόνα intensity πετύχαινε.

#### Μέθοδος
«Fallback» τεμπέλικο: κρατείται μόνο η προηγούμενη ωμή σάρωση· τα δύο πανοράματα απόστασης και η κίνηση υπολογίζονται μόνο όταν η εικόνα intensity αποτύχει.
Η κατάσταση «candidate» (τρίτη αρχή) μένει ως είχε. Έλεγχοι: (1) μόνο ο εκτιμητής στο eee_03· (2) runs δίπλα-δίπλα υπό το ίδιο φορτίο: δύο αρχές με / χωρίς
εφεδρεία σε math_easy, quad_easy (σπόρος 0)· εφεδρεία στο eee_03, 4 σπόροι.

#### Αποτέλεσμα
Εκτιμητής, eee_03: χρήσεις εικόνας απόστασης 351 (ταχεία) / 354 (παλιά), χωρίς κίνηση 214 / 212. Χρόνος runs: math_easy **512 s** έναντι 513 s χωρίς εφεδρεία·
quad_easy **452** / 450 s — ίδιο APE (0.1078, 0.0790). eee_03 ATE: ταχεία **0.400 ± 0.040** (0.351, 0.449, 0.398, 0.403) έναντι παλιάς 0.344 ± 0.079 (0.333, 0.238,
0.385, 0.419)· δύο αρχές χωρίς εφεδρεία 0.592.

#### Συμπέρασμα ⏳
1. **Η εφεδρεία δεν κοστίζει πια τίποτα όπου δεν χρησιμοποιείται** (ίδιος χρόνος και ίδιο αποτέλεσμα ως το bit).
2. Όπου χρησιμοποιείται, συμπεριφέρεται όπως πριν (ίδιος αριθμός χρήσεων)· το RANSAC της απόστασης τραβά πλέον τυχαίους αριθμούς μόνο στις αποτυχίες, άρα οι
   κινήσεις διαφέρουν όπως με άλλον σπόρο — eee_03 0.40 ± 0.04 έναντι 0.34 ± 0.08 m, εντός της διασποράς των σπόρων.

---

### 2026-09-29 — #077 Κατακόρυφος περιορισμός χωρίς IMU: το ύψος της bodleian ακολουθεί την κλίση, αλλά ούτε το έδαφος ούτε οι τοίχοι τη μετρούν αρκετά

**Σχετικά:** [#072](#2026-09-28--072-bodleian-το-κατακόρυφο-drift-της-odometry-δεν-είναι-θέμα-ανάλυσης-του-χάρτη-η-υπόθεση-του-068-απορρίπτεται) · «Where we stand» 3b ·
**κλάδος `vertical_constraint`** · script `scripts/analyse_ground_normal.py` · μόνο ανάλυση (runs δύο αρχών του #059, ωμές σαρώσεις της bodleian)

#### Στόχος
Ζήτημα Μ.Τ. 29/9: διερεύνηση του κατακόρυφου σφάλματος και ενός περιορισμού. Πρώτα: από πού έρχεται το ύψος της bodleian· μετά: μπορεί το LiDAR μόνο του να
μετρήσει το «πάνω» αρκετά καλά (≪ 1°) για περιορισμό;

#### Μέθοδος
1. Γωνία ανάμεσα στο εκτιμώμενο και το αληθινό «πάνω» (μετά την ευθυγράμμιση κόσμου και σώματος, #067) και μεταβολή ύψους ανά 60 s, δύο αρχές σ0–3 και «χωρίς
   deskew».
2. **Έδαφος:** ανά σάρωση (κάθε 10η, 501), RANSAC για το μεγαλύτερο σχεδόν οριζόντιο επίπεδο κάτω από τον αισθητήρα (1–15 m, κώνος 35°, 5 cm), χωρίς GT· σύγκριση
   με το αληθινό «πάνω»· και οι κάθετες σε παγκόσμιο σύστημα μέσω GT, μέσοι όροι σε 5 / 30 / 60 s (το έδαφος είναι επίπεδο κατά μέσο όρο;).
3. **Τοίχοι:** έως 4 μεγάλα σχεδόν κατακόρυφα επίπεδα ανά σάρωση (1–30 m)· «πάνω» = η κατεύθυνση κάθετη στις κάθετές τους (χρειάζονται ≥ 2 μη παράλληλοι).

#### Αποτέλεσμα
**1. Το ύψος ακολουθεί την κλίση:** σφάλμα κλίσης 0.5–0.9° τον περισσότερο χρόνο, 1.2–2.8° σε διαστήματα (κυρίως 360–480 s)· εκεί το ύψος χάνει −1.4 έως −1.7 m
ανά 60 s σε 70–100 m (tan 1° × 85 m ≈ 1.5 m)· σε διαστήματα με ~0.55° μόνο ±0.3 m. Ίδιο σε όλους τους σπόρους και στο «χωρίς deskew».

**2. Έδαφος** (501 σαρώσεις, επίπεδο στο 99 %): σφάλμα ανά σάρωση διάμεσος **1.52°**, p90 3.32° (σταθερή απόκλιση μόνο 0.25°· τα μεγάλα επίπεδα όχι καλύτερα,
1.44°). Μέσος όρος σε 5 / 30 / 60 s: διάμεσος 1.42° / 1.20° / **0.93°**, p90 3.19° / 2.51° / 2.30° — **ο μέσος όρος δεν βοηθά: οι αποκλίσεις είναι πραγματικό
ανάγλυφο (κλίσεις, ράμπες), όχι θόρυβος.**

**3. Τοίχοι:** 3–4 μεγάλοι τοίχοι ανά σάρωση, αλλά **όλοι παράλληλοι** (αζιμούθιο ±10°: δρόμοι και διάδρομοι με δύο απέναντι προσόψεις) — κανένα «πάνω» σε
καμία σάρωση του ελέγχου. Παράλληλοι τοίχοι κατά μήκος της διαδρομής καθορίζουν μόνο την κλίση **εγκάρσια** (roll)· την κλίση **κατά μήκος** (pitch), που
είναι αυτή που χαλάει το ύψος, δεν τη βλέπουν.

#### Συμπέρασμα ⏳
1. Το κατακόρυφο σφάλμα της bodleian είναι **σφάλμα κλίσης** της odometry (pitch κατά μήκος της διαδρομής), όχι σφάλμα ύψους χωρίς κλίση.
2. **Κανένα από τα απλά σήματα του LiDAR δεν μετρά το «πάνω» αρκετά καλά εκεί:** το έδαφος έχει πραγματικές κλίσεις (~1° ακόμη και σε 60 s — όσο το σφάλμα
   που θέλουμε να διορθώσουμε)· οι τοίχοι είναι παράλληλοι στη διαδρομή και τυφλοί στο pitch. Ένας περιορισμός από το έδαφος θα έσπρωχνε την κλίση προς το
   ανάγλυφο.
3. Επιλογές: (α) κατακόρυφες ακμές (γωνίες κτηρίων, στύλοι) — καθορίζουν και τις δύο κλίσεις, πιο δύσκολη εξαγωγή, αβέβαιο αν φτάνουν ≪ 0.5°· (β) **βαρύτητα
   από IMU** μόνο για roll / pitch (τα datasets έχουν IMU) — η σωστή λύση, αλλά αλλάζει το «μόνο LiDAR» της μεθόδου· απόφαση Μ.Τ. / Λ.Γ.

---

### 2026-09-29 — #076 Αυτόματη κανονικοποίηση intensity ανά σάρωση (gain) σε runs: Hilti καλύτερο σε 4 / 6, NCD χωρίς βλάβη, NTU μικτό· 2048 στήλες χωρίς κέρδος

**Σχετικά:** [#075](#2026-09-29--075-φωτεινότητα-της-εικόνας-στο-hilti--ntu-10-σε-runs-μικτό-reflectivity--πανόραμα-2048-στηλών--αυτόματη-κλίμακα-ανά-σάρωση-)
· **κλάδος `intensity_norm`**, commit `d356e2f`: `image_deskew.intensity_normalisation` (none | gain | gain_clahe) και `image_deskew.panorama_width`, προεπιλογή
όπως πριν· `run_ncd.py --normalise=`, `--panorama-width=` · runs `…/{surftwogain,surftwogainw}_s0–3` · script `/home/photogrammetry/kiss_runs/gain_launch.sh`

#### Στόχος
Απόφαση Μ.Τ. 29/9: η κανονικοποίηση ανά σάρωση (#075) ως επιλογή στον κώδικα και πραγματικά runs, πριν από την κατακόρυφη διερεύνηση.

#### Μέθοδος
Έλεγχοι πριν από τα runs (μόνο ο εκτιμητής, UZH): προεπιλογή → 452 αποτυχίες (ακριβώς όπως πριν: η προεπιλογή δεν άλλαξε)· gain → 60 (όπως η δοκιμή του
#075)· gain + 2048 στήλες → 25. Runs: δύο αρχές + gain σε Hilti × 6, NTU × 3, NCD quad_hard / math_easy / underground_hard (έλεγχος ότι δεν βλάπτει)· δύο αρχές
+ gain + 2048 στήλες σε Hilti × 6· 4 σπόροι, 72 runs, 0 αποτυχίες.

#### Αποτέλεσμα (επίσημο σκορ [m], 4 σπόροι· αποτυχίες εικόνας ×0.25 / gain / gain+2048)

| ακολουθία | KISS | χωρίς deskew | δύο αρχές ×0.25 | ×1.0 | **gain** | gain + 2048 | αποτυχίες |
|---|---|---|---|---|---|---|---|
| Basement_1 | 0.055 | 0.078 | 0.058 | 0.058 | **0.052** | 0.052 | 31 / 1 / 1 |
| IC_Office_1 | 6.344 | 1.655 | **0.072** | 0.076 | 0.074 | 0.072 | 6 / 1 / 1 |
| Office_Mitte_1 | 4.286 | 0.575 | 1.437 | 0.282 | **0.226** | 0.315 | 20 / 2 / 1 |
| Construction_Site_1 | 0.063 | 0.062 | 0.049 | **0.043** | 0.045 | 0.046 | 481 / 14 / 9 |
| LAB_Survey_2 | 0.062 | 0.050 | 0.036 | **0.035** | 0.035 | 0.036 | 1 / 1 / 1 |
| UZH | 0.585 | **0.204** | 0.503 | 0.576 | 0.576 | 0.579 | 450 / 64 / 31 |
| NTU eee_01 / 02 / 03 | 2.68 / 1.49 / 0.86 | 2.36 / 1.49 / 0.84 | 2.03 / **0.82** / 0.59 | 2.03 / 1.10 / **0.44** | **1.94** / 0.88 / 0.68 | — | 1039/696 · 646/401 · 562/404 |
| NCD quad_hard / math_easy / underground_hard | 0.33 / 0.16 / 12.78 | **0.21** / 0.10 / 12.34 | 0.22 / 0.11 / **0.086** | — | 0.22 / **0.10** / 0.087 | — | — |

NCD quad_hard, RPE 1 s: 9.18 → 8.32 cm με gain.

#### Συμπέρασμα ⏳
1. **Hilti: η κανονικοποίηση ανά σάρωση δουλεύει χωρίς κλίμακα ανά αισθητήρα** — καλύτερη ή ίση σε 5 από 6 (Office_Mitte_1 1.44 → 0.23 m, σταθερό).
2. **NCD: καμία βλάβη** (ίδιο ή λίγο καλύτερο)· **NTU μικτό** (καλύτερο στο eee_01, χειρότερο στα eee_02/03, όπου μένει καλύτερη η σταθερή κλίμακα με εφεδρεία, #071).
3. **UZH μένει άλυτο:** οι αποτυχίες πέφτουν 450 → 31–64 αλλά το σκορ όχι — εκεί η κίνηση της εικόνας είναι λάθος, όχι απούσα.
4. **Οι 2048 στήλες μειώνουν κι άλλο τις αποτυχίες χωρίς να βελτιώνουν το σκορ** — δεν αξίζουν.
5. Λιγότερες αποτυχίες βοηθούν μόνο αν οι κινήσεις που τις αντικαθιστούν είναι καλές. Πρόταση: gain για δεδομένα σαν του Hilti· επιλογή, όχι προεπιλογή.

---

### 2026-09-29 — #075 Φωτεινότητα της εικόνας στο Hilti / NTU: ×1.0 σε runs (μικτό), reflectivity ✗, πανόραμα 2048 στηλών ✓, αυτόματη κλίμακα ανά σάρωση ✓

**Σχετικά:** [#074](#2026-09-29--074-η-κλίμακα-intensity-255-1024-είναι-λάθος-για-τα-ouster-του-hilti-και-του-ntu-σκοτεινά-πανοράματα-65-σημεία-αντί-για-1400)
· κλάδος `vertical_drift` · runs `…/{surftwos1,surftworangefbs1,surftworangecands1}_s0–3` (108, 0 αποτυχίες) · δοκιμές μόνο του εκτιμητή κίνησης
(scripts στο scratchpad, **καμία αλλαγή αρχείου**) · πίνακας `docs/results_official_allarms.md`

#### Στόχος
Ζητήματα Μ.Τ. 29/9: η σωστή κλίμακα intensity σε πραγματικά runs· το πεδίο `reflectivity` αντί για `intensity` (επιλογή 1)· άλλο μέγεθος πανοράματος στο
Hilti (ιδέα Μ.Τ.)· αυτόματη κανονικοποίηση ανά σάρωση (επιλογή 2).

#### Αποτέλεσμα
**1. Runs με ×1.0** (δύο αρχές, + εφεδρεία, + τρίτη αρχή· 4 σπόροι· επίσημο σκορ [m]): Office_Mitte_1 τρίτη αρχή ×1.0 **0.202** (καλύτερο), Construction_Site_1
**0.041**, LAB_Survey_2 0.035· αλλά NTU eee_02 δύο αρχές 0.820 → 1.101 και UZH 0.503 → 0.576 (χειρότερα, αν και οι αποτυχίες έπεσαν 646 → 368 / 450 → 90)·
NTU: καλύτερο το ×0.25 + εφεδρεία στα eee_02 / eee_03 (0.679 / 0.344), το ×1.0 + εφεδρεία στο eee_01 (1.613). **Καμία σταθερή κλίμακα δεν είναι παντού
καλύτερη — πιο φωτεινό ενισχύει και τον θόρυβο.**

**2. Αιτία της σκοτεινής εικόνας (μία σάρωση ανά αισθητήρα):** διάμεσος raw intensity NCD OS0-128 242 · Hilti LAB 69 · Construction 44 · UZH **34** · NTU 107·
με ×255/1024 κάτω από 30 / 255: 30 % / 64 % / 86 % / **94 %** / 55 % των pixel. Επιστροφές: Construction μόνο 39 % (ανοιχτός χώρος). Το Ouster του Hilti
λειτουργεί σε 2048 στήλες (πιθανώς μικρότερο σήμα ανά βολή — δεν επαληθεύτηκε).

**3. `reflectivity` (16 bit, p99 → 255 ανά αισθητήρα):** αποτυχίες UZH 165, Construction 113, NTU eee_03 **1376** — χειρότερα από intensity ×1.0 (83 / 16 / 358)·
το NTU κορεννύεται στα 65 535. **Απορρίπτεται.**

**4. Πανόραμα 2048 στηλών** (intensity ×1.0): αποτυχίες UZH 83 → **35**, Construction 16 → **10**, LAB 0 → 0. Στο 1024 δύο επιστροφές ανά pixel και η μία χάνεται.

**5. Αυτόματη κανονικοποίηση ανά σάρωση** (αποτυχίες· και σφάλμα κίνησης ανά σάρωση έναντι GT, διάμεσος):

| | σταθερή (σήμερα) | **gain (p99 → 255)** | gain + CLAHE | CLAHE |
|---|---|---|---|---|
| UZH / Construction / LAB | 447 / 473 / 0 | **60 / 13 / 0** | 89 / 21 / 0 | 87 / 23 / 0 |
| NTU eee_03 | 566 | 415 | 418 | 445 |
| NCD math_easy: στροφή / μετατόπιση | 0.836° / 4.33 cm | 0.820° / 4.28 | 0.791° / **4.25** | **0.785°** / 4.32 |
| Spires church_03 (Hesai) | 0.484° / 1.20 cm | 0.484° / 1.20 (ίδιο) | **0.479° / 1.16** | **0.479° / 1.16** |

#### Συμπέρασμα ⏳
1. Η σταθερή κλίμακα ανά κατασκευαστή δεν στέκει· **η αυτόματη κανονικοποίηση ανά σάρωση (gain) αφαιρεί τις περισσότερες αποτυχίες του Hilti χωρίς ρύθμιση**
   ανά αισθητήρα και δεν αλλάζει το Hesai· το CLAHE προσθέτει λίγη ακρίβεια (NCD / Hesai) με περισσότερες αποτυχίες στο Hilti.
2. **Πανόραμα με τις στήλες του αισθητήρα** (2048 στο Hilti) βοηθά επιπλέον.
3. Το `reflectivity` απορρίπτεται.
4. Επόμενο: επιλογές `intensity_normalisation` και `panorama_width` στον κώδικα (νέος κλάδος, όχι προεπιλογή) και πραγματικά runs.

---

### 2026-09-29 — #074 Η κλίμακα intensity ×255/1024 είναι λάθος για τα Ouster του Hilti και του NTU: σκοτεινά πανοράματα, 65 σημεία αντί για ~1400

**Σχετικά:** [#041](#2026-09-23--041) (η κλίμακα ×255/1024, μετρημένη στο Ouster του Newer College· αποδεκτή από τον Μ.Τ. 25/9, ⏳ Λ.Γ.) · [#069](#2026-09-26--069-εικόνα-απόστασης-όπου-αποτυγχάνει-η-εικόνα-intensity-δύο-αρχές-ntu-1542--office_mitte_1-σταθεροποιείται--διόρθωση-του-068)
· open_tasks Β.10(γ), «Where we stand» 6 · κλάδος `vertical_drift` · εικόνες `/home/photogrammetry/kiss_runs/failed_matches/` · μόνο ο εκτιμητής κίνησης
(`scripts/dump_failed_matches.py`), κανένα run SLAM

#### Στόχος
Γιατί αποτυγχάνει η εικόνα στο Hilti UZH (50 % των σαρώσεων);

#### Μέθοδος
Στατιστικά των αποτυχιών (`rejected.csv`) και επιθεώρηση των εικόνων. Μετά ο εκτιμητής κίνησης μόνος του (ίδιες ρυθμίσεις και σπόρος με τα runs) με
`--intensity-scale` 0.5 και 1.0 αντί για 255/1024.

#### Αποτέλεσμα
Σημεία SURF ανά πανόραμα στις αποτυχίες (διάμεσος): UZH **65**, Construction_Site_1 195, NTU eee_03 358 (16 γραμμές), Blenheim 1325· ταιριάσματα μετά το ratio
test 19 / 31 / 47 / 126. Η εικόνα του UZH είναι σχεδόν μαύρη· τα λίγα ταιριάσματα είναι στο κοντινό δάπεδο και τα αφαιρεί το φίλτρο δαπέδου. Το Ouster του
Hilti δίνει intensity p99 ≈ 440 (του NCD ~1100)· με ×255/1024 σχεδόν όλο το πανόραμα πέφτει κάτω από ~30 / 255.

| ακολουθία | αποτυχίες με ×255/1024 | ×0.5 | ×1.0 |
|---|---|---|---|
| Hilti UZH | 452 / 894 | 209 | **83** (σημεία στις αποτυχίες: 65 → 1443) |
| Hilti Construction_Site_1 | 481 / 1994 | — | **16** |
| NTU eee_03 | 565 / 1813 | — | **358** |

#### Συμπέρασμα ⏳
1. **Η κλίμακα intensity είναι ανά αισθητήρα, όχι ανά κατασκευαστή:** η ×255/1024 (#041) ταιριάζει στο Ouster του Newer College, αλλά κάνει τα πανοράματα του
   Hilti (και εν μέρει του NTU) πολύ σκοτεινά. Με ×1.0 οι αποτυχίες πέφτουν κατά 82–97 % στο Hilti και 37 % στο NTU (το υπόλοιπο πιθανώς οι 16 γραμμές).
2. **Επηρεάζει την αποδεκτή επιλογή (δ) του Μ.Τ. 25/9** (×255/1024): σωστή για το NCD, όχι γενικά. Προς απόφαση Μ.Τ. / Λ.Γ.
3. Επόμενο: runs του Hilti και του NTU με `--intensity-scale=1.0` (μόνο runs, καμία αλλαγή κώδικα)· εναλλακτικά αυτόματη κλίμακα ανά αισθητήρα
   (π.χ. από το p99 του intensity) — μικρή αλλαγή κώδικα.

---

### 2026-09-29 — #073 Εικόνα απόστασης ως τρίτη αρχή: dynamic_spinning 0.46 → 0.30 m (3 από 4 σπόρους ≈ KISS), Office_Mitte_1 σταθεροποιείται

**Σχετικά:** [#058](#2026-09-25--058-δύο-αρχικές-θέσεις-σε-16-ακολουθίες--εικόνα-απόστασης-ως-εφεδρεία--τρίτη-αρχή--γιατί-δουλεύει-με-έλεγχο-των-αποφάσεων-στο-gt)
(`range_motion = candidate`), [#068](#2026-09-26--068-οι-υπόλοιπες-εκκρεμότητες-με-τα-υπάρχοντα-runs-μήκος--τρέμουλο-στροφή--θόρυβος-ανά-θέση-γρήγορη-περιστροφή-bodleian)
(γρήγορη περιστροφή), [#069](#2026-09-26--069-εικόνα-απόστασης-όπου-αποτυγχάνει-η-εικόνα-intensity-δύο-αρχές-ntu-1542--office_mitte_1-σταθεροποιείται--διόρθωση-του-068) ·
«Where we stand» 4 · κλάδος `vertical_drift` · runs `…/surftworangecand_s0–3` · script `/home/photogrammetry/kiss_runs/rangecand_launch.sh`

#### Στόχος
Απόφαση Μ.Τ. 29/9 («Where we stand» 4, καμία αλλαγή κώδικα): η εικόνα απόστασης ως **τρίτη** αρχική θέση του ICP όπου η εικόνα intensity κάνει λάθος στη
γρήγορη περιστροφή.

#### Μέθοδος
Δύο αρχές + `--range=candidate`, 4 σπόροι, σε dynamic_spinning, Office_Mitte_1, UZH, NTU eee_01–03: 24 runs, 0 αποτυχίες. Επίσημο πρωτόκολλο.

#### Αποτέλεσμα (επίσημο σκορ [m])

| ακολουθία | KISS | χωρίς deskew | δύο αρχές | + εφεδρεία (#071) | **+ τρίτη αρχή** |
|---|---|---|---|---|---|
| dynamic_spinning | 0.159 | 20.751 | 0.460 ± 0.139 | 0.504 ± 0.263 | **0.302 ± 0.235** (σπόροι 0.141, 0.220, 0.651, 0.197) |
| Office_Mitte_1 | 4.286 | 0.575 | 1.437 ± 1.208 | 0.241 ± 0.060 | **0.238 ± 0.094** |
| UZH | 0.585 | 0.204 | 0.503 | 0.551 | 0.502 |
| NTU eee_01 / 02 / 03 | 2.68 / 1.49 / 0.86 | 2.36 / 1.49 / 0.84 | 2.03 / 0.82 / 0.59 | **1.74 / 0.68 / 0.34** | 1.99 / 0.83 / 0.58 |

dynamic_spinning, RPE 1 s στροφής: τρίτη αρχή **2.97°** (δύο αρχές 3.11°, KISS 3.98°).

#### Συμπέρασμα ⏳
1. **Η τρίτη αρχή διορθώνει τη γρήγορη περιστροφή:** στο dynamic_spinning 3 από 4 σπόρους φτάνουν τον KISS (0.14–0.22 m έναντι 0.159), με την καλύτερη στροφή
   όλων· ένας σπόρος αποτυγχάνει (0.65 m).
2. **Office_Mitte_1: σταθεροποιείται όπως με την εφεδρεία** (0.238 ± 0.094). Άρα εκεί παίζουν ρόλο και οι λάθος κινήσεις (που διορθώνει η τρίτη αρχή) και οι
   αποτυχίες (που διορθώνει η εφεδρεία) — συμπληρώνει τη διόρθωση του #069 στο #068.
3. **NTU / UZH: κανένα κέρδος** — η τρίτη αρχή δεν δίνει κίνηση όπου η εικόνα intensity αποτυγχάνει· εκεί βοηθά η εφεδρεία (NTU) ή η κλίμακα intensity (#074).
4. Οι δύο χρήσεις της εικόνας απόστασης είναι συμπληρωματικές· σήμερα η ρύθμιση δέχεται μία (`fallback` ή `candidate`). Και οι δύο μαζί = μικρή αλλαγή
   κώδικα, αν το ζητήσει ο Μ.Τ.

---

### 2026-09-28 — #072 Bodleian: το κατακόρυφο drift της odometry ΔΕΝ είναι θέμα ανάλυσης του χάρτη (η υπόθεση του #068 απορρίπτεται)

**Σχετικά:** [#068](#2026-09-26--068-οι-υπόλοιπες-εκκρεμότητες-με-τα-υπάρχοντα-runs-μήκος--τρέμουλο-στροφή--θόρυβος-ανά-θέση-γρήγορη-περιστροφή-bodleian)
(Β.1(β): υπόθεση «χοντρός χάρτης»), [#063](#2026-09-25--063-η-σκάλα-λύνεται-δύο-αρχές--indoor_detail-open_tasks-β3-μία-φορά) (σκάλα) · `docs/where_we_stand_2026-09-28.md` πρόβλημα 3a ·
κλάδος `vertical_drift` · runs `/home/photogrammetry/kiss_runs/oxford_spires_full/bodleian_02/{surftwodetail,surftwov05,kissdetailnodeskew,kissnodeskewv05}_s*` ·
configs `configs/indoor_detail(_nodeskew).yaml`, `/home/photogrammetry/kiss_runs/{paper,nodeskew}_voxel05_threads4.yaml`

#### Στόχος
Απόφαση Μ.Τ. 28/9 (3a): μία φορά, διαγνωστικά — μειώνει ένας πυκνότερος χάρτης το κατακόρυφο drift της odometry στη bodleian;

#### Μέθοδος
(A) indoor_detail (voxel 0.25 m, εμβέλεια 50 m, χάρτες 15 m) και (B) η ρύθμιση του paper με μόνο το voxel στα 0.5 m, για δύο αρχές (4 σπόροι) και KISS χωρίς
deskew (1)· + το υπάρχον KISS indoor_detail (#052). 10 runs, 0 αποτυχίες. Επίσημο πρωτόκολλο + διάσπαση ύψους (`analyse_vertical.py`). Οι χάρτες 15 m του (A)
έδωσαν 2–3 loop closures στις δύο αρχές· χωρίστηκαν με `replay_backend.py` (χωρίς closures / βάρος ×100).

#### Αποτέλεσμα

| bodleian | APE [m] | z RMSE [m] | xy [m] | ύψος στο τέλος | RPE 1 s | τρέμουλο (μήκος) |
|---|---|---|---|---|---|---|
| paper (voxel 1 m), δύο αρχές | 0.513 ± 0.087 | 0.497 | 0.126 | −2.90 m | 9.69 cm / 1.18° | +18.9 % |
| voxel 0.5 m, δύο αρχές | 0.599 ± 0.067 | 0.586 | 0.122 | −4.29 m | 8.41 cm / 1.18° | +14.6 % |
| indoor_detail, δύο αρχές | 0.499 ± 0.040 | 0.480 | 0.135 | −2.84 m | **7.07 cm** / 1.18° | **+10.2 %** |
| paper, χωρίς deskew | 1.363 | 1.333 | 0.284 | −5.46 m | 15.47 cm / 0.64° | +50.0 % |
| voxel 0.5 m, χωρίς deskew | 2.367 | 2.286 | 0.614 | −8.24 m | 14.67 cm / 0.77° | +40.1 % |
| indoor_detail, χωρίς deskew | 2.421 | 2.078 | 1.242 | −8.48 m | 13.66 cm / 0.86° | +29.2 % |
| indoor_detail, KISS | 2.547 | 0.763 | 2.430 | −3.76 m | 35.37 cm / 2.95° | +68.4 % |

indoor_detail, δύο αρχές, ανά back end: όπως έτρεξε 0.499 · χωρίς closures **0.504** · βάρος ×100 0.505 m (οι επαναλήψεις αναπαράγουν τα runs ως 1 mm).

#### Συμπέρασμα ⏳
1. **Η υπόθεση του #068 απορρίπτεται:** ο πυκνότερος χάρτης βελτιώνει τα τοπικά μέτρα (RPE 1 s −27 %, τρέμουλο στο μισό) αλλά **όχι το ύψος** (z RMSE
   0.48–0.59 m και στις τρεις ρυθμίσεις για τη μέθοδο· για το «χωρίς deskew» χειρότερο). Οι closures δεν παίζουν ρόλο εδώ (χωρίς αυτές 0.504 m).
2. Το κατακόρυφο drift της bodleian είναι της odometry και δεν εξηγείται από την ανάλυση του χάρτη. Μένει η λύση 3b: **κατακόρυφος περιορισμός**
   (επίπεδο εδάφους / βαρύτητα, π.χ. από την ευθυγράμμιση εδάφους του MapClosures) — νέος κώδικας.

---

### 2026-09-28 — #071 Εικόνα απόστασης ως εφεδρεία σε όλες τις 27 ακολουθίες: 14–6 έναντι των δύο αρχών, 23–4 έναντι του «χωρίς deskew»· συνδυασμός με τη διόρθωση του γράφου

**Σχετικά:** [#069](#2026-09-26--069-εικόνα-απόστασης-όπου-αποτυγχάνει-η-εικόνα-intensity-δύο-αρχές-ntu-1542--office_mitte_1-σταθεροποιείται--διόρθωση-του-068) (6 ακολουθίες) ·
[#070](#2026-09-28--070-βάρος-στροφής-100-στον-γράφο-κόμβων-επιβεβαίωση-με-πραγματικά-runs) · open_tasks Β.10(γ) · κλάδος `vertical_drift` · runs `…/surftworangefb_s0–3` ·
script `/home/photogrammetry/kiss_runs/rw_rangefb_launch.sh` · πίνακας `docs/results_official_allarms.md`

#### Στόχος
Απόφαση Μ.Τ. 28/9 (πρόβλημα 2 του «Where we stand»): η εικόνα απόστασης ως εφεδρεία (#069) στις υπόλοιπες 21 ακολουθίες — χειροτερεύει κάπου;

#### Μέθοδος
84 runs (δύο αρχές + `--range=fallback`, 4 σπόροι, back end όπως πριν: βάρος 1), 0 αποτυχίες. Μαζί με τα 24 του #069: 27 ακολουθίες. Επίσημο πρωτόκολλο
κάθε dataset· σύγκριση ανά ακολουθία (`compare_arms.py`). Για τις δύο ακολουθίες με loop closures, ο συνδυασμός με βάρος στροφής ×100 υπολογίστηκε με
επανάληψη του back end (`replay_backend.py rotw=100`· αναπαράγει τα runs ως 3 mm).

#### Αποτέλεσμα
Έναντι των δύο αρχών (27): APE **14–6** (7 ίδιες), διάμεσος −0.2 %, p 0.07· RPE 1 m στροφής 13–3 (p 0.03)· τα υπόλοιπα μέτρα ίδια. Μεγάλα κέρδη: Office_Mitte_1
−83 % (1.437 ± 1.208 → 0.241 ± 0.060), eee_03 −42 %, NCD long experiment −23 % (2.113 → 1.619 ± 0.761), eee_02 −17 %, eee_01 −14 %, Basement_1 −14 %.
Απώλειες: UZH +9.6 %, dynamic_spinning +9.8 % (και οι δύο αμετάβλητα προβλήματα, #069), bodleian +6.2 % (εντός της διασποράς ± 0.087)· όλα τα άλλα ≤ 2.3 %.
Σαρώσεις χωρίς κίνηση: π.χ. Construction_Site_1 481 → 16, eee_01 1039 → 409, long experiment 30 → 6.

| με εφεδρεία έναντι… (27) | APE | RPE 1 s μετατ. | RPE 1 s στροφή | μήκος | z RMSE |
|---|---|---|---|---|---|
| KISS χωρίς deskew | **23–4, −36.9 %, p < 0.0001** | 14–4, −39.2 % | 9–9 | 16–2 | 16–2, −35.1 % |
| KISS | 25–2, −42.3 % | 17–1 | 18–0 | 18–0 | 15–3 |

(δύο αρχές χωρίς εφεδρεία έναντι χωρίς deskew: 22–5, −29 %, #065.)

**Συνδυασμός με βάρος στροφής ×100 (offline):** NCD long experiment **0.385 ± 0.014 m** (z 0.140)· 01_short 0.305 ± 0.011 m.

#### Συμπέρασμα ⏳
1. **Η εικόνα απόστασης ως εφεδρεία δεν χειροτερεύει ουσιαστικά πουθενά** (απώλειες ≤ 10 % μόνο σε δύο ακολουθίες με γνωστά, ανεξάρτητα προβλήματα) και
   κερδίζει πολύ όπου αποτυγχάνει η εικόνα intensity. Έναντι του «χωρίς deskew»: 23–4 (−37 %) αντί 22–5 (−29 %).
2. Με τη διόρθωση του γράφου (#070) οι δύο αλλαγές συνδυάζονται χωρίς σύγκρουση: long experiment 2.11 → **0.39 m**.
3. **Πρόταση: νέα προεπιλογή = δύο αρχές + εικόνα απόστασης ως εφεδρεία + βάρος στροφής 100** (απόφαση Μ.Τ.). Κόστος: ~2× υπολογισμός χαρακτηριστικών.

---

### 2026-09-28 — #070 Βάρος στροφής ×100 στον γράφο κόμβων: επιβεβαίωση με πραγματικά runs

**Σχετικά:** [#067](#2026-09-26--067-κατακόρυφο-drift-στο-ncd-long-experiment-το-προκαλεί-το-pose-graph-βάρη-i₆-όχι-η-odometry--διόρθωση-offline-στροφή-100)
(αιτία + offline δοκιμή) · open_tasks Β.1(α) · κλάδος `vertical_drift`, commit `46d588e` (`pose_graph_optimizer.rotation_weight`, προεπιλογή 1 = upstream;
`run_ncd.py --rotation-weight`) · runs `…/{kiss,kissnodeskew,sift,surf,surftwo}rw_s*` · script `/home/photogrammetry/kiss_runs/rw_rangefb_launch.sh`

#### Στόχος
Απόφαση Μ.Τ. 28/9 (πρόβλημα 1 του «Where we stand»): η διόρθωση του #067 στον κώδικα, με πραγματικά runs.

#### Μέθοδος
Νέα ρύθμιση: πληροφορία diag(1, 1, 1, w, w, w) σε κάθε ακμή του γράφου κόμβων (odometry μεταξύ κόμβων και loop closures)· η εξομάλυνση ανά σάρωση μένει I₆.
Έλεγχος: quad_easy (χωρίς closures) με w = 100 → ίδια τροχιά με πριν (μέγιστη διαφορά 0.05 mm, στρογγυλοποίηση του αρχείου). Runs με w = 100 στις δύο
ακολουθίες με loop closures (NCD long experiment, 01_short), όλοι οι βραχίονες, 4 σπόροι όπου υπάρχουν: 28 runs, 0 αποτυχίες.

#### Αποτέλεσμα (APE / z RMSE [m], μέσος των σπόρων)

| βραχίονας | long: πριν (I₆) | long: offline ×100 (#067) | **long: runs ×100** | 01_short: πριν → ×100 |
|---|---|---|---|---|
| KISS | 1.276 / 1.172 | 0.458 / 0.206 | **0.458 / 0.206** | 0.419 → 0.397 |
| χωρίς deskew | 3.498 / 3.352 | 0.609 / 0.117 | **0.607 / 0.117** | 0.350 → 0.325 |
| SIFT | 1.277 / 1.035 | 0.373 / 0.140 | **0.373 / 0.140** | 0.304 → 0.302 |
| SURF | 2.020 / 1.833 | 0.448 / 0.263 | **0.448 / 0.263** | 0.305 → 0.304 |
| SURF δύο αρχές | 2.113 / 2.061 | 0.391 / 0.139 | **0.392 / 0.139** (σπόροι 0.37–0.42) | 0.305 → 0.304 |

Τα τοπικά μέτρα (RPE ανά δευτερόλεπτο) αμετάβλητα — αλλάζει μόνο το back end.

#### Συμπέρασμα ⏳
1. **Επιβεβαιώνεται:** τα πραγματικά runs δίνουν ό,τι προέβλεψε η offline επανάληψη, ως το χιλιοστό. Στο long experiment το σφάλμα ύψους πέφτει από ~2 m σε
   0.14 m και όλοι οι βραχίονες βελτιώνονται· στο 01_short τίποτα δεν χειροτερεύει. Στις υπόλοιπες 25 ακολουθίες (χωρίς closures) η ρύθμιση δεν αλλάζει τίποτα.
2. **Πρόταση: `rotation_weight = 100` ως προεπιλογή** (απόφαση Μ.Τ.)· είναι διόρθωση του back end του KISS-SLAM, ανεξάρτητη από τη μέθοδο.

---

### 2026-09-26 — #069 Εικόνα απόστασης όπου αποτυγχάνει η εικόνα intensity (δύο αρχές): NTU −15…−42 %, Office_Mitte_1 σταθεροποιείται · διόρθωση του #068

**Σχετικά:** [#058](#2026-09-25--058-δύο-αρχικές-θέσεις-σε-16-ακολουθίες--εικόνα-απόστασης-ως-εφεδρεία--τρίτη-αρχή--γιατί-δουλεύει-με-έλεγχο-των-αποφάσεων-στο-gt)
(εικόνα απόστασης), [#068](#2026-09-26--068-οι-υπόλοιπες-εκκρεμότητες-με-τα-υπάρχοντα-runs-μήκος--τρέμουλο-στροφή--θόρυβος-ανά-θέση-γρήγορη-περιστροφή-bodleian)
(Β.10(β): **διορθώνεται** εδώ) · open_tasks Β.10(γ) · κλάδος `vertical_drift` · runs `…/surftworangefb_s0–3` · script `/home/photogrammetry/kiss_runs/rangefb_launch.sh`

#### Στόχος
Όπου η εικόνα intensity δεν δίνει κίνηση (NTU 16 ακτίνες: 20–31 % των σαρώσεων· UZH: 50 %), να δίνει η εικόνα απόστασης (`--range=fallback`, #058).

#### Μέθοδος
Δύο αρχές + εικόνα απόστασης ως εφεδρεία, 4 σπόροι, σε NTU eee_01–03, Hilti UZH_Tracking_Area_Run_2 και Office_Mitte_1, NCD dynamic_spinning: 24 runs,
0 αποτυχίες. Επίσημο πρωτόκολλο κάθε dataset.

#### Αποτέλεσμα (επίσημο σκορ [m], 4 σπόροι)

| ακολουθία | KISS | χωρίς deskew | δύο αρχές | **+ εικόνα απόστασης** | σαρώσεις χωρίς κίνηση |
|---|---|---|---|---|---|
| NTU eee_01 | 2.678 | 2.363 | 2.028 ± 0.098 | **1.737 ± 0.190** | 1039 → 409 |
| NTU eee_02 | 1.486 | 1.490 | 0.820 ± 0.073 | **0.679 ± 0.064** | 646 → 188 |
| NTU eee_03 | 0.864 | 0.841 | 0.592 ± 0.041 | **0.344 ± 0.079** | 562 → 216 |
| Hilti Office_Mitte_1 | 4.286 | 0.575 | 1.437 ± 1.208 | **0.241 ± 0.060** (σπόροι 0.18–0.31) | 20 → 1 |
| Hilti UZH | 0.585 | **0.204** | 0.503 | 0.551 | 450 → 112 |
| dynamic_spinning | **0.159** | 20.75 | 0.460 ± 0.139 | 0.504 ± 0.263 | 72 → 66 |

#### Συμπέρασμα ⏳
1. **Η εικόνα απόστασης ως εφεδρεία βελτιώνει και τις τρεις NTU (−15 έως −42 %)** και κόβει τις σαρώσεις χωρίς κίνηση κατά 60–70 %.
2. **Office_Mitte_1: η αστάθεια των σπόρων εξαφανίζεται** (0.18–0.31 m και στους 4). **Διόρθωση του #068 (Β.10(β)):** εκεί γράφτηκε ότι οι κακοί σπόροι
   οφείλονται σε λάθος κίνηση της εικόνας που «αποφασίζει ο σπόρος». Το παρόν δείχνει ότι αρκεί να δοθεί κίνηση στις ~20 σαρώσεις όπου η εικόνα
   intensity **αποτυγχάνει** (χωρίς deskew στη δύσκολη στιγμή) — οι αποτυχίες, όχι μόνο οι λάθος κινήσεις, έριχναν τους κακούς σπόρους.
3. **UZH και dynamic_spinning: κανένα κέρδος.** Στο UZH ο «χωρίς deskew» μένει καλύτερος (0.20 m)· στο dynamic_spinning το πρόβλημα είναι η γρήγορη
   περιστροφή (#068), όχι οι αποτυχίες (72 → 66).
4. Επόμενο (απόφαση Μ.Τ.): η εικόνα απόστασης ως εφεδρεία σε **όλες τις 27** ακολουθίες (κόστος: διπλός υπολογισμός χαρακτηριστικών) — αν δεν
   χειροτερεύει αλλού, υποψήφια προεπιλογή.

---

### 2026-09-26 — #068 Οι υπόλοιπες εκκρεμότητες με τα υπάρχοντα runs: μήκος = τρέμουλο, στροφή = θόρυβος ανά θέση, γρήγορη περιστροφή, bodleian

**Σχετικά:** open_tasks Β.1(β), Β.2, Β.4, Β.5, Β.10(α)(β) · [#067](#2026-09-26--067-κατακόρυφο-drift-στο-ncd-long-experiment-το-προκαλεί-το-pose-graph-βάρη-i₆-όχι-η-odometry--διόρθωση-offline-στροφή-100) ·
κλάδος `vertical_drift` · μόνο ανάλυση των runs του #061–#065 (επίσημο πρωτόκολλο), κανένα νέο run

#### Στόχος
Ζήτημα Μ.Τ.: διερεύνηση όλων των ανοιχτών εκκρεμοτήτων και λύσεων.

#### Αποτέλεσμα και συμπέρασμα ⏳ ανά εκκρεμότητα

**Β.2 — η υπερεκτίμηση του μήκους είναι τρέμουλο ανά σάρωση, όχι σφάλμα κλίμακας.** Μήκος εκτίμησης / GT με δειγματοληψία κάθε 1 / 10 / 50 / 200
σαρώσεις (18 ακολουθίες με πλήρες GT, σπόρος 0), διάμεσος: δύο αρχές **+8.3 / +0.2 / +0.1 / +0.1 %**· χωρίς deskew +20.3 / +0.6 / +0.2 / 0.0 %· KISS
+52.6 / +1.2 / +0.3 / +0.1 %. Η μετατόπιση είναι σωστή· η εκτίμηση τρεμοπαίζει λίγα εκατοστά γύρω από την αληθινή διαδρομή. Όπου η περίσσεια
μένει και σε αραιή δειγματοληψία, το run απέτυχε (KISS σε stairs / underground_hard, χωρίς deskew στο dynamic_spinning). Συνέπεια: η στήλη «μήκος
έναντι GT» μετρά **τρέμουλο** — εκεί η μέθοδος έχει το μικρότερο. **Β.2 κλείνει.**

**Β.4 — η ισοπαλία στη στροφή είναι θόρυβος ανά θέση, όχι drift.** RPE στροφής σε 0.1 / 1 / 10 s (18 ακολουθίες, όλοι οι σπόροι): δύο αρχές καλύτερες σε
**3 / 9 / 9**. Στα 10 s η μέθοδος κερδίζει πολύ όπου ο KISS δυσκολεύεται (stairs 13.3° έναντι 43.3°, underground_hard 1.4° / 38.0°, dynamic_spinning
4.2° / 62.9°, keble 1.2° / 4.2°) και χάνει στις «εύκολες» (Blenheim 1.07° / 0.42°, bodleian 1.18° / 0.78°) — εκεί η διαφορά είναι **ίδια σε 0.1, 1 και 10 s**,
άρα δεν συσσωρεύεται: θόρυβος προσανατολισμού ~0.7° ανά θέση, συνεπής με την υπόθεση του #045 (το σφάλμα στροφής της εικόνας ανά σάρωση περνά
μέσα από το deskew στη θέση). Στα 0.1 s τα σφάλματα είναι στο επίπεδο του ίδιου του GT (ένα δείγμα GT απέχει 0.4–1.4° από το μέσο των γειτόνων του,
μαζί με την αληθινή κίνηση) και ο βραχίονας χωρίς deskew συγκρίνεται με παρεμβεβλημένο GT — **τα 0.1 s δεν είναι αξιόπιστη σύγκριση**. Πιθανές λύσεις:
(α) δεύτερο deskew με την κίνηση που έδωσε ο ICP (T_{k−1}⁻¹T_k) και νέος ICP· (β) εξομάλυνση της στροφής (#047: καλύτερο RPE στροφής, λίγο χειρότερο
ATE)· (γ) στάθμιση της στροφής της εικόνας με το πλήθος / την κατανομή των αντιστοιχιών.

**Β.5 (υπόλοιπο) — τα διαφορετικά υποσύνολα GT δεν αλλάζουν τίποτα.** Όλοι οι βραχίονες με GT παρεμβεβλημένο στις θέσεις τους (αντί για το ταίριασμα του
evo): δύο αρχές έναντι χωρίς deskew APE 15–3, −38.3 %, p 0.001 (ίδιο με το επίσημο), RPE μετατόπισης 14–4, στροφή 9–9. **Β.5 κλείνει.**

**Β.10(α) — dynamic_spinning: η εικόνα σπάει στα ~10° ανά σάρωση.** Drift ανά παράθυρο 10 s: δύο αρχές 0.05–0.09 m στα πρώτα 90 s (KISS 0.11–0.25 m),
**1.1–1.8 m στα τελευταία 20 s** όπου η GT περιστροφή φτάνει ~100°/s (≈10° ανά σάρωση)· το SURF μίας αρχής σπάει και σε άλλα παράθυρα στα 70–94°/s.

**Β.10(β) — Office_Mitte_1: μία δύσκολη στιγμή, ίδια για όλους.** Σκορ Hilti ανά σπόρο: SURF 0.20 / 0.23 / 1.57 / 2.59, δύο αρχές 0.18 / 0.72 / 2.00 / 2.84 m
(χωρίς deskew 0.57, KISS 4.29) — **δικόρυφο**. Όλα τα κακά runs αποκλίνουν στο **t ≈ 181–184 s** (0.9–1.1 m και 9–14° σε 1 s έναντι καλού σπόρου)· εκεί η
εικόνα δίνει 14–16° ανά σάρωση και ο σπόρος του RANSAC αποφασίζει αν η κίνηση είναι σωστή. Κοινό με το (α): **πάνω από ~10° ανά σάρωση η κίνηση της
εικόνας γίνεται αναξιόπιστη**. Λύσεις προς δοκιμή: εικόνα απόστασης ως τρίτη αρχή (`--range=candidate`, #058)· στη γρήγορη στροφή προτίμηση της
σταθερής ταχύτητας.

**Β.1(β) — bodleian: άλλος μηχανισμός από το long experiment.** Καμία closure· το σφάλμα ύψους συσσωρεύεται αργά παντού (−0.2 έως −0.5 m ανά 30 s στην
αρχή) και γρηγορότερα στα 360–450 s (−0.5 έως −1.0 m ανά 30 s), όπου το GT κατεβαίνει και ανεβαίνει 0.7–0.8 m (σκαλοπάτια / ράμπες)· ίδιο στους δύο
βραχίονες. Υπόθεση: ο **χοντρός χάρτης** της ρύθμισης του paper (voxel 1 m, πηγή ICP 1.5 m) δεν «βλέπει» μικρές υψομετρικές αλλαγές — όπως στη σκάλα
(#063). Δοκιμή: bodleian με indoor_detail, μία φορά (όπως το Β.3).

---

### 2026-09-26 — #067 Κατακόρυφο drift: στο NCD long experiment το προκαλεί το pose graph (βάρη I₆), όχι η odometry — διόρθωση offline: στροφή ×100

**Σχετικά:** [#060](#2026-09-25--060-bodleian-γιατί-χειροτερεύει-το-ate-με-δύο-αρχές-ύψος-όχι-λάθος-επιλογές) (bodleian), [#065](#2026-09-26--065-οι-πέντε-βραχίονες-σε-11-νέες-ακολουθίες-ncd-2020-hilti-2021-ntu-viral--σύνολο-27-ακολουθίες-με-τα-επίσημα-πρωτόκολλα)
(long experiment: APE 2.1 m, z RMSE 2.1 m) · open_tasks Β.1 · **κλάδος `vertical_drift`** · scripts `scripts/analyse_vertical.py`,
`scripts/replay_backend.py` · αποτελέσματα `/home/photogrammetry/kiss_runs/backend_replay/`, `backend_replay_067.json`

#### Στόχος
Από πού έρχεται το κατακόρυφο σφάλμα των μεγάλων διαδρομών: από την odometry (κλίση ή κατακόρυφη μετατόπιση) ή από το back end;

#### Μέθοδος
1. **Διάσπαση του σφάλματος ύψους** (`analyse_vertical.py`): για διαδοχικές θέσεις, ακριβώς
   $\Delta z_{est}-\Delta z_{gt} = e_z^\top(R_{est}-R_{gt})d_{est} + e_z^\top R_{gt}(d_{est}-d_{gt})$ (στροφή / μετατόπιση), μετά την ευθυγράμμιση του evo
   και την αφαίρεση σταθερής στροφής ανάμεσα στο σύστημα του αισθητήρα και του GT (στο quad_easy 0.8–0.9° — χωρίς αυτή οι δύο όροι βγαίνουν ±3.7 m
   και αλληλοαναιρούνται). Αποτέλεσμα: στις μεγάλες διαδρομές οι δύο όροι αλληλοαναιρούνται μερικώς (ο ICP ευθυγραμμίζει θέσεις) — **η διάσπαση
   από μόνη της δεν αποδίδει αιτία**· δείχνει όμως απότομα, κοινά σε όλους τους βραχίονες άλματα ύψους ±2–3 m σε 30 s.
2. **Odometry μόνο έναντι τελικής τροχιάς:** η τροχιά χωρίς loop closures ανασυντίθεται από τις ακμές odometry του `trajectory.g2o`. (Ένα πρώτο
   «ταυτόσημα» ήταν λάθος του δικού μου ελέγχου — αντικαθιστούσα `pose_times` σε λάθος module· διορθώθηκε.)
3. **Επανάληψη του back end offline** (`replay_backend.py`): odometry ανά σάρωση + γράφος κόμβων με odometry και loop closures από τα αρχεία του run,
   ξαναχτισμένος **σταδιακά όπως στο KissSLAM** (κόμβος, closures του κόμβου που τελείωσε, 10 επαναλήψεις Dogleg) και με το ίδιο τελικό εξομαλυντικό
   ανά σάρωση. **Αναπαράγει κάθε run ως 2 cm** (≤ 8 mm σε 17 από 18), με τις ίδιες σταθερές σαρώσεις. Παραλλαγές: χωρίς closures· έλεγχος ύψους
   0.3 / 0.5 m· closures μόνο σε x, y, yaw· 4-DoF γράφος (όπως VINS-Mono)· ίδιος γράφος λυμένος μαζικά ως τη σύγκλιση· 30 / 100 / 1000 επαναλήψεις
   g2o· **πληροφορία diag(1, 1, 1, w, w, w)** σε κάθε ακμή του γράφου κόμβων (στροφή ×w).

#### Αποτέλεσμα
Ποιες ακολουθίες έχουν loop closures με τη ρύθμιση του paper (τοπικοί χάρτες 100 m, το MapClosures αγνοεί τους 3 τελευταίους): **μόνο** 01_short (6) και
NCD 2020 long experiment (6–7)· στις άλλες 25 το αποτέλεσμα είναι καθαρή odometry.

NCD 2020 long experiment, APE / z RMSE [m], μέσος των σπόρων:

| βραχίονας | χωρίς closures | όπως έτρεξε (I₆) | 4-DoF | μαζικά ως σύγκλιση | g2o 1000 επαν. | **στροφή ×100** | στροφή ×10⁴ |
|---|---|---|---|---|---|---|---|
| KISS | 1.09 / 0.24 | 1.28 / 1.17 | 0.59 / 0.21 | 0.58 / 0.19 | 1.28 / 1.17 | **0.46 / 0.21** | 0.44 / 0.21 |
| χωρίς deskew | 2.09 / 0.17 | 3.50 / 3.35 | 1.39 / 0.14 | 2.68 / 2.45 | 3.54 / 3.40 | **0.61 / 0.12** | 0.44 / 0.12 |
| SIFT | 1.37 / 0.18 | 1.28 / 1.04 | 0.43 / 0.16 | 0.42 / 0.13 | 1.37 / 1.22 | **0.37 / 0.14** | 0.38 / 0.14 |
| SURF | 8.00 / 7.33 | 2.02 / 1.83 | 3.46 / 3.13 | 0.70 / 0.48 | 2.06 / 1.87 | **0.45 / 0.26** | 0.61 / 0.45 |
| SURF δύο αρχές | 1.39 / 0.19 | 2.11 / 2.06 | 0.52 / 0.18 | 0.52 / 0.15 | 2.16 / 2.10 | **0.39 / 0.14** | 0.38 / 0.14 |

01_short (δύο αρχές): χωρίς closures 0.675, όπως έτρεξε 0.305, στροφή ×100 0.304, ×10⁴ 0.308 m. Έλεγχος ύψους 0.3 m: ανάμεικτο (KISS 2.33, χωρίς deskew
0.13)· closures μόνο σε x, y, yaw: καμία βελτίωση (1.8–2.5 m). Η διαφορά ύψους κάθε closure από την odometry: 0.0–0.5 m. SURF σπόροι 2–3: η **odometry**
αποτυγχάνει κατακόρυφα (z RMSE 14.5 m χωρίς closures)· οι closures τη σώζουν.

#### Συμπέρασμα ⏳
1. **Στο NCD long experiment το ύψος το χαλάει το pose graph, όχι η odometry:** χωρίς closures z RMSE 0.17–0.24 m· με closures 1.0–2.1 m. Οι closures
   διορθώνουν το οριζόντιο (xy 1.36 → 0.34 m) αλλά ο γράφος πληρώνει τη διόρθωση με κλίσεις.
2. **Ο μηχανισμός είναι τα βάρη, όχι η σύγκλιση ή οι ίδιες οι closures:** περισσότερες επαναλήψεις g2o δεν αλλάζουν τίποτα (έχει συγκλίνει)· το ίδιο
   πρόβλημα λυμένο με σφάλμα στροφής σε ακτίνια (≈ ×4 βάρος) δίνει 0.52 m, και στο g2o το ×4 το αναπαράγει ακριβώς. Με πληροφορία I₆ και σφάλμα στροφής
   = διάνυσμα τετραδονίου (≈ θ/2), η στροφή κοστίζει ελάχιστα έναντι της μετατόπισης· σε κόμβους 100 m μια κλίση 1 mrad μετακινεί το άκρο 10 cm — ο
   γράφος λυγίζει τους κόμβους αντί να κατανείμει μετατόπιση.
3. **Διόρθωση (offline): πληροφορία στροφής ×100 στον γράφο κόμβων** — δύο αρχές APE 2.11 → **0.39 m** (z 2.06 → 0.14), KISS 1.28 → 0.46, SIFT → 0.37,
   χωρίς deskew 3.50 → 0.61· το 01_short αμετάβλητο. Το 4-DoF δεν χρειάζεται (όχι καλύτερο, χειρότερο όπου η κλίση της odometry είναι λάθος).
4. **Η bodleian (#060) είναι άλλος μηχανισμός:** καμία closure εκεί — το drift ύψους είναι της odometry. Μένει ανοιχτό.
5. Επόμενο: ρύθμιση (π.χ. `pose_graph_optimizer.rotation_weight`, προεπιλογή 1 = upstream) και επιβεβαίωση με πραγματικά runs· απόφαση Μ.Τ.

---

### 2026-09-26 — #066 Διόρθωση του #065: οι αποτυχίες της εικόνας στο NTU VIRAL είναι 20–31 %, όχι 31–57 %

**Σχετικά:** [#065](#2026-09-26--065-οι-πέντε-βραχίονες-σε-11-νέες-ακολουθίες-ncd-2020-hilti-2021-ntu-viral--σύνολο-27-ακολουθίες-με-τα-επίσημα-πρωτόκολλα)
(Συμπέρασμα 3)

Στο #065 γράφτηκε «31–57 % αποτυχίες της εικόνας» για το NTU VIRAL. Λάθος υπολογισμός: διαιρέθηκαν οι αποτυχίες με λάθος πλήθος σαρώσεων. Από τα logs
(δύο αρχές, σπόρος 0): eee_01 **1041 / 3987 = 26 %**, eee_02 **647 / 3210 = 20 %**, eee_03 **566 / 1814 = 31 %**. Η στήλη «αποτυχίες εικόνας» του
πίνακα του #065 δίνει μέσους όρους των 4 σπόρων (1039, 646, 562) και είναι σωστή. Τα υπόλοιπα συμπεράσματα δεν αλλάζουν.

---

### 2026-09-26 — #065 Οι πέντε βραχίονες σε 11 νέες ακολουθίες (NCD 2020, Hilti 2021, NTU VIRAL) · σύνολο 27 ακολουθίες με τα επίσημα πρωτόκολλα

**Σχετικά:** [#064](#2026-09-25--064-νέα-datasets-ncd-2020-long--dynamic_spinning-hilti-2021-ntu-viral-και-τα-επίσημα-εργαλεία-αξιολόγησής-τους--το-time-offset-του-dynamic_spinning-δεν-αφορά-το-lidar)
(datasets, εργαλεία), [#061](#2026-09-25--061-αξιολόγηση-με-το-επίσημο-πρωτόκολλο-evo--κάθε-θέση-στη-δική-της-στιγμή--κλείνει-το-ανοιχτό-πρόβλημα-του-045046) ·
κλάδος `after_two_start` · runs `/home/photogrammetry/kiss_runs/{newer_college_2020,hilti_2021,ntu_viral}/` · script `/home/photogrammetry/kiss_runs/newseq_launch.sh` ·
πίνακας `docs/results_official.md`

#### Στόχος
Οι βραχίονες σύγκρισης (απόφαση Μ.Τ. 25/9) στα νέα datasets: NCD 2020 02_long_experiment (44 min, 26 560 σαρώσεις) και dynamic_spinning, Hilti 2021 × 6
(OS0-64), NTU VIRAL eee_01–03 (OS1-16, 16 ακτίνες). Χωρίς περιθώριο στις δύο αρχές (#062).

#### Μέθοδος
150 runs (+ 4 ήδη υπάρχοντα, ίδιες ρυθμίσεις): KISS και KISS χωρίς deskew 1 run (ντετερμινιστικοί), SIFT, SURF (μία αρχή), SURF δύο αρχές × 4 σπόροι·
ρύθμιση του paper (threads4)· 12 παράλληλα, 0 αποτυχίες, κανένα kernel BUG. Αξιολόγηση με το επίσημο πρωτόκολλο κάθε dataset (`results_table.py`):
NCD evo APE (+ RPE, μήκος)· Hilti το APE του challenge· NTU το ATE του πρίσματος (πληρότητα 100 % σε όλα).

#### Αποτέλεσμα
Επίσημο σκορ [m] (μέσος ± σ των 4 σπόρων):

| ακολουθία | KISS | χωρίς deskew | SIFT | SURF | δύο αρχές | αποτυχίες εικόνας (δύο αρχές) |
|---|---|---|---|---|---|---|
| NCD 02_long_experiment | **1.275** | 3.498 | 1.276 ± 0.676 | 2.020 ± 0.075 | 2.113 ± 0.202 | 30 / 26 560 |
| NCD dynamic_spinning | **0.159** | 20.751 | 1.390 ± 0.190 | 0.774 ± 0.134 | 0.460 ± 0.139 | 72 |
| Hilti Basement_1 | 0.055 | 0.078 | 0.056 ± 0.003 | **0.049 ± 0.013** | 0.058 ± 0.010 | 31 |
| Hilti IC_Office_1 | 6.344 | 1.655 | 0.098 ± 0.017 | **0.072 ± 0.002** | **0.072 ± 0.006** | 6 |
| Hilti Office_Mitte_1 | 4.286 | **0.575** | 4.400 ± 0.908 | 1.147 ± 1.154 | 1.437 ± 1.208 | 20 |
| Hilti Construction_Site_1 | 0.063 | 0.062 | 0.058 ± 0.007 | 0.050 ± 0.004 | **0.049 ± 0.004** | 481 |
| Hilti LAB_Survey_2 | 0.062 | 0.050 | 0.037 ± 0.000 | **0.036 ± 0.001** | **0.036 ± 0.001** | 1 |
| Hilti UZH_Tracking_Area_Run_2 | 0.585 | **0.204** | 1.289 ± 0.523 | 0.506 ± 0.000 | 0.503 ± 0.000 | 450 / 895 |
| NTU eee_01 | 2.678 | 2.363 | **1.400 ± 0.172** | 1.964 ± 0.094 | 2.028 ± 0.098 | 1039 |
| NTU eee_02 | 1.486 | 1.490 | 3.952 ± 2.174 | 0.880 ± 0.191 | **0.820 ± 0.073** | 646 |
| NTU eee_03 | 0.864 | 0.841 | **0.471 ± 0.043** | 0.540 ± 0.034 | 0.592 ± 0.041 | 562 |

NCD 2020 (πλήρες GT): 02_long — RPE 1 s: δύο αρχές 11.5 cm / 2.58°, χωρίς deskew 11.2 cm / 2.00°, KISS 27.8 cm / 3.49°· μήκος +13.0 % / +37.4 % / +85.7 %·
z RMSE 2.06 / 3.35 / 1.17 m. dynamic_spinning — δύο αρχές 19.3 cm / 3.11°, KISS 18.3 cm / 3.98°· μήκος +17.7 % / +52.6 %.

Σύγκριση ανά ακολουθία (`compare_arms.py`):

| δύο αρχές έναντι… | APE | RPE 1 s μετατ. | RPE 1 s στροφή | μήκος |
|---|---|---|---|---|
| χωρίς deskew, 11 νέες | 9–2, −29 %, p 0.08 | — | — | — |
| KISS, 11 νέες | 8–3, −24 %, p 0.15 | — | — | — |
| **χωρίς deskew, 27 ακολουθίες** | **22–5, −29 %, p 0.0007** | 14–4, −38 %, p 0.003 | **9–9** | 16–2, p < 0.0001 |
| **KISS, 27** | **24–3, −42 %, p 0.0001** | 17–1 | 18–0 | 18–0 |
| SURF μία αρχή, 27 | 13–14, p 0.68 | 15–3, −0.5 % | 16–1, −0.6 % | 16–2, −4 % |

SURF έναντι SIFT (27): APE 17–10 (p 0.12), RPE μετατόπισης 18–0, στροφής 18–0, μήκος 18–0.

#### Συμπέρασμα ⏳
1. **Στις 27 ακολουθίες (5 datasets, 4 αισθητήρες) η μέθοδος κερδίζει τη δίκαιη βάση (KISS χωρίς deskew) στο επίσημο σκορ 22–5 (−29 %, p = 0.0007)
   και τον KISS 24–3**· η στροφή ανά δευτερόλεπτο μένει ισοπαλία (9–9) — επιβεβαίωση του #059/#061 σε ανεξάρτητα δεδομένα.
2. **Hilti:** το καλύτερο σκορ σε 4 από 6 (IC_Office_1 6.34 → 0.07 m του KISS, LAB_Survey_2 0.036)· **χάνει** στο Office_Mitte_1 (μεγάλη διασπορά
   σπόρων: 1.1–1.4 ± 1.2 m, κάποιοι σπόροι αποτυγχάνουν) και στο UZH (η εικόνα αποτυγχάνει στο 50 % των σαρώσεων· χωρίς deskew 0.20 m).
3. **NTU (16 ακτίνες):** καλύτερο σε 3/3 έναντι KISS, αλλά με 31–57 % αποτυχίες της εικόνας· εδώ το SIFT είναι καλύτερο σε 2/3.
4. **Όπου χάνει από τον KISS:** dynamic_spinning (γρήγορη σταθερή περιστροφή = η υπόθεση του KISS) και 02_long_experiment: τοπικά ίσο με το χωρίς deskew
   (11.5 cm/s) και πολύ καλύτερο μήκος (+13 % έναντι +37 %), αλλά APE 2.1 m με z RMSE 2.1 m — **κατακόρυφο drift σε μεγάλη διαδρομή**, όπως η
   bodleian (#060). Το ανοιχτό Β.1 (βαρύτητα / ύψος) είναι ο κύριος περιορισμός.
5. Δύο αρχές έναντι μίας: ίδιο APE (13–14) με μικρά αλλά σταθερά κέρδη στα τοπικά μέτρα και στο μήκος.

---

### 2026-09-25 — #064 Νέα datasets (NCD 2020 long / dynamic_spinning, Hilti 2021, NTU VIRAL) και τα επίσημα εργαλεία αξιολόγησής τους · το time offset του dynamic_spinning δεν αφορά το LiDAR

**Σχετικά:** [#061](#2026-09-25--061-αξιολόγηση-με-το-επίσημο-πρωτόκολλο-evo--κάθε-θέση-στη-δική-της-στιγμή--κλείνει-το-ανοιχτό-πρόβλημα-του-045046) · κλάδος
`after_two_start` · `docs/datasets.md` · `scripts/evaluate_hilti.py`, `scripts/evaluate_ntu.py` · δεδομένα `/home/photogrammetry/kiss_data/` (σύνδεσμοι, ext4)

#### Στόχος
Ζήτημα Μ.Τ.: οργάνωση των datasets που κατέβηκαν (Newer College 2020 long experiment και dynamic_spinning, Hilti SLAM Challenge 2021 × 6, NTU VIRAL
eee_01–03) ώστε να τρέξουν τα ίδια τεστ, με το επίσημο πρωτόκολλο αξιολόγησης **κάθε** dataset.

#### Μέθοδος
1. **Οργάνωση χωρίς εγγραφή στον NTFS** (#046): δέντρο συνδέσμων στο ext4 (`kiss_data/`, 31 σύνδεσμοι, 0 σπασμένοι)· τα zip του NTU αποσυμπιέστηκαν
   στο ext4. Έλεγχοι: κάθε bag ανοίγει· τα 16 bags του long experiment συνεχόμενα (επικάλυψη 0.04 s), 26 560 σαρώσεις = θέσεις του GT· τα αρχεία Hilti
   ίδια σε μέγεθος με το Hugging Face.
2. **Λήψεις (από τα επίσημα αποθετήρια):** `calibration.yaml` του Hilti (5.4 KB, Hugging Face `Hilti-Research`), GT του NTU VIRAL (`ntu-aris/ntuviral_gt`,
   4.9 MB).
3. **Hilti 2021** (`evaluate_hilti.py`): αναπαράγει το `evaluation-evo/evaluation.py` — τροχιά του IMU (LiDAR `os_sensor` → IMU από τη βαθμονόμηση,
   ~180° γύρω από το x, 13 cm), × μετατόπιση κατά τον τύπο του GT (`_pole` άκρη κονταριού, `_prism`, `_imu`), αντιστοίχιση εντός 1 s, SE(3), APE.
4. **NTU VIRAL** (`evaluate_ntu.py`): αναπαράγει το `ntuviral_evaluate.ipynb` του tutorial — τροχιά σώματος (οριζόντιος OS1-16 → σώμα), **+ πρίσμα
   0.40 m**, 0.05 s, SE(3), ATE, πληρότητα (< 90 % με ATE < 20 m → ∞).
5. **dynamic_spinning:** έλεγχος του `time_offsets.csv` του dataset και δοκιμή ευαισθησίας ±55 ms (ζήτημα Μ.Τ.).

#### Αποτέλεσμα
- **Ταύτιση με τα επίσημα εργαλεία:** Hilti — ίδια νούμερα ως το τελευταίο ψηφίο σε όλα τα παραδείγματα του αποθετηρίου (pole, prism, δύο πυκνά IMU)·
  NTU — ίδια με το notebook στα 18 δείγματα FAST-LIO2 (μέγιστη διαφορά 4·10⁻¹⁶ m). Η μετατροπή LiDAR→IMU του Hilti επιβεβαιώνεται από τη στροφή
  (διάμεσο σφάλμα 0.7° με βαθμονόμηση, 179.7° χωρίς).
- **Πρώτα runs (1 σπόρος):** Hilti LAB_Survey_2 APE 0.035 m (παράδειγμα hdl_graph_slam του dataset 0.053)· UZH_Tracking_Area_Run_2 0.503 m (η εικόνα
  αποτυγχάνει σε 453 / 895 σαρώσεις)· NTU eee_03 SURF δύο αρχές 0.548 m, KISS 0.864 m (FAST-LIO2 με IMU: 0.102)· αποτυχίες εικόνας 31 % με 16 ακτίνες.
- **dynamic_spinning, time_offsets.csv:** κατά το dataset είναι οι μετατοπίσεις RealSense IMU / Ouster IMU ως προς τις κάμερες· το GT είναι ανά σάρωση
  LiDAR με χρονοσφραγίδες ακριβώς πάνω στις σαρώσεις (0.000 ms). Ευαισθησία (4 σπόροι όπου υπάρχουν):

  | βραχίονας | APE [m]: χωρίς / +55 / −55 ms | RPE 1 s στροφή [°]: ίδια σειρά |
  |---|---|---|
  | KISS | 0.159 / 0.166 / 0.165 | 3.98 / 8.43 / 6.73 |
  | KISS χωρίς deskew | 20.75 / 20.78 / 21.07 | 28.4 / 30.6 / 30.0 |
  | SIFT | 1.390 / 1.579 / 1.579 | 3.73 / 8.74 / 7.14 |
  | SURF | 0.774 / 0.790 / 0.788 | 3.29 / 7.98 / 6.31 |
  | SURF δύο αρχές | 0.460 / 0.498 / 0.486 | 3.11 / 8.06 / 6.05 |

- **Σφάλμα στο GT του dynamic_spinning:** 40 ακριβώς διπλές γραμμές (ίδιος χρόνος, ίδια θέση) στην αρχή και στο τέλος — έσπαγαν την παρεμβολή SLERP.
  Διόρθωση στο `evaluate_ncd.load_gt`: κρατείται η πρώτη κάθε διπλής χρονοσφραγίδας· καμία άλλη από τις 18 ακολουθίες δεν έχει (αμετάβλητες).

#### Συμπέρασμα ⏳
1. Τα νέα datasets αξιολογούνται **με το δικό τους επίσημο πρωτόκολλο**, με εργαλεία που ταυτίζονται αριθμητικά με τα επίσημα.
2. **Το time offset του dynamic_spinning δεν εφαρμόζεται:** αφορά κάμερες/IMU· εφαρμοσμένο στο LiDAR διπλασιάζει το σφάλμα στροφής όλων των βραχιόνων
   και προς τις δύο κατευθύνσεις.
3. **Το dynamic_spinning είναι η πρώτη ακολουθία όπου ο KISS κερδίζει στο APE** (0.159 έναντι 0.460 m των δύο αρχών)· η εικόνα κερδίζει στη στροφή ανά
   δευτερόλεπτο (3.11° έναντι 3.98°). Γρήγορη σταθερή περιστροφή = η υπόθεση σταθερής ταχύτητας του KISS· να εξεταστεί με όλα τα αποτελέσματα (#065).

---

### 2026-09-25 — #062 Δύο αρχές με περιθώριο αλλαγής (2 % / 4 %): καμία μετρήσιμη διαφορά

**Σχετικά:** [#060](#2026-09-25--060-bodleian-γιατί-χειροτερεύει-το-ate-με-δύο-αρχές-ύψος-όχι-λάθος-επιλογές) (πρόταση), [#059](#2026-09-25--059-δύο-αρχικές-θέσεις-ως-προεπιλογή-4-σπόροι-σε-16-ακολουθίες),
[#061](#2026-09-25--061-αξιολόγηση-με-το-επίσημο-πρωτόκολλο-evo--κάθε-θέση-στη-δική-της-στιγμή--κλείνει-το-ανοιχτό-πρόβλημα-του-045046) (αξιολόγηση) ·
κλάδος `after_two_start` · `image_deskew.two_start_margin`, `run_ncd.py --two-start-margin` · runs `/home/photogrammetry/kiss_runs/<ακολουθία>/surftwom2_s0–3`,
`surftwom4_s0–3` · script `/home/photogrammetry/kiss_runs/margin062_launch.sh` · πίνακας `docs/results_official.md`

#### Στόχος
Ζήτημα Μ.Τ. (open_tasks Β.6): στις δύο αρχές, αλλαγή από την αρχή της εικόνας σε άλλη αρχή **μόνο αν το ταίριασμα στον χάρτη βελτιώνεται περισσότερο
από 2–4 %** — θα έκοβε τις 1–3 βλαβερές αλλαγές ανά run του #060.

#### Μέθοδος
Νέα ρύθμιση `two_start_margin` (κλάσμα): η άλλη αρχή κρατείται μόνο αν fit_άλλης < (1 − περιθώριο) · fit_εικόνας· 0 = όπως πριν (#057–#060). 128 runs:
περιθώριο 2 % και 4 %, 16 ακολουθίες × 4 σπόροι, ρύθμιση του paper (threads4), 12 παράλληλα, 20:10 τέλος, 0 αποτυχίες. Αξιολόγηση με το επίσημο
πρωτόκολλο (#061), σύγκριση ανά ακολουθία (`compare_arms.py`).

#### Αποτέλεσμα
Αλλαγές σε σταθερή ταχύτητα (όλα τα runs): χωρίς περιθώριο 2521 από 12 205 σαρώσεις με διαφωνία (20.7 %) · 2 %: 1894 (15.5 %) · 4 %: 1511 (12.3 %).

| έναντι «δύο αρχές» (16 ακολουθίες) | APE | RPE 1 m μετατ. | RPE 1 s μετατ. | RPE 1 s στροφή | μήκος |
|---|---|---|---|---|---|
| περιθώριο 2 % | 9–7, −1.0 %, p 0.14 | 9–7, p 0.14 | 12–4, −0.4 %, p 0.07 | 10–6, p 0.30 | 10–6, p 0.07 |
| περιθώριο 4 % | 7–9, +0.3 %, p 0.90 | 9–7, p 0.35 | 10–6, p 0.35 | 7–9, p 0.59 | 6–10, p 0.71 |

Έναντι KISS χωρίς deskew και τα δύο όπως οι δύο αρχές: APE 13–3 (−34 %, p ≤ 0.004), RPE μετατόπισης −39…−43 %, στροφή 8–8, μήκος 14–15 / 16.
Κρίσιμες ακολουθίες (APE, 4 σπόροι): Blenheim 0.273 ± 0.021 · 0.269 ± 0.023 · 0.260 ± 0.003 m (χωρίς / 2 % / 4 %)· bodleian 0.513 · 0.487 · 0.491 m·
keble 0.094 · 0.092 · 0.095 m· underground_hard 0.086 · 0.086 · 0.088 m.

#### Συμπέρασμα ⏳
1. **Το περιθώριο δεν αλλάζει τίποτα μετρήσιμο**: κόβει το 25–40 % των αλλαγών (τις οριακές), χωρίς επίδραση πάνω από 1 % σε κανένα μέτρο (p ≥ 0.07)·
   η διόρθωση του Blenheim μένει. Οι οριακές αλλαγές ήταν ουδέτερες — επιβεβαίωση του #060.
2. **Πρόταση: η προεπιλογή μένει χωρίς περιθώριο** (απλούστερη, ίδια αποτελέσματα)· το 2 % είναι ισοδύναμη επιλογή αν προτιμηθούν λιγότερες
   διπλές καταχωρίσεις. Απόφαση Μ.Τ.

---

### 2026-09-25 — #063 Η σκάλα λύνεται: δύο αρχές + indoor_detail (open_tasks Β.3, μία φορά)

**Σχετικά:** [#045](#2026-09-23--045-δύο-ακόμη-βάσεις-του-kiss-χωρίς-deskew-indoor_detail-και-η-χρονική-μετατόπιση-της-αξιολόγησης) (μόνο το
indoor_detail έλυνε τη σκάλα), [#059](#2026-09-25--059-δύο-αρχικές-θέσεις-ως-προεπιλογή-4-σπόροι-σε-16-ακολουθίες), [#061](#2026-09-25--061-αξιολόγηση-με-το-επίσημο-πρωτόκολλο-evo--κάθε-θέση-στη-δική-της-στιγμή--κλείνει-το-ανοιχτό-πρόβλημα-του-045046)
(αξιολόγηση) · κλάδος `after_two_start` · runs `/home/photogrammetry/kiss_runs/newer_college_2021/stairs/surftwodetail_s0–3`,
`kissdetailnodeskew_s0` · scripts `/home/photogrammetry/kiss_runs/b3_062_launch.sh` · νέο config `configs/indoor_detail_nodeskew.yaml`

#### Στόχος
Ζήτημα Μ.Τ. (μία φορά, εκτός των βραχιόνων σύγκρισης): η σκάλα του NCD 2021 έμενε άλυτη με κάθε βραχίονα στη ρύθμιση του paper (ATE ~2.1 m και με
δύο αρχές, #059)· μόνο ο KISS με `indoor_detail` τη «έλυνε» (0.46 m, #045). Λύνεται με δύο αρχές + indoor_detail;

#### Μέθοδος
`run_ncd.py surf … --config=configs/indoor_detail.yaml --two-start=5`, σπόροι 0–3 (voxel 0.25 m, εμβέλεια 50 m, χάρτες 15 m). Δίκαιη βάση στο ίδιο
config: KISS **χωρίς deskew** (`configs/indoor_detail_nodeskew.yaml` = indoor_detail με `deskew: false`), 1 run (ντετερμινιστικός). Αξιολόγηση με το
επίσημο πρωτόκολλο (#061).

#### Αποτέλεσμα (stairs, 57 m)

| Βραχίονας | runs | APE [m] | RPE 1 s [cm] | RPE 1 s [°] | μήκος έναντι GT | z RMSE [m] |
|---|---|---|---|---|---|---|
| KISS (paper) | 2 | 3.586 | 100.9 | 27.73 | +318 % | 2.333 |
| KISS χωρίς deskew (paper) | 1 | 2.705 | 67.9 | 13.71 | +183 % | 2.327 |
| SURF δύο αρχές (paper) | 4 | 2.074 ± 0.225 | 21.3 ± 0.6 | 5.01 ± 0.16 | +64 % | 1.770 |
| KISS indoor_detail | 1 | 0.458 | 12.05 | 3.92 | +35.1 % | 0.341 |
| KISS indoor_detail χωρίς deskew | 1 | 0.536 | 9.90 | 2.88 | +20.3 % | 0.460 |
| **SURF δύο αρχές indoor_detail** | 4 | 0.478 ± 0.048 | **6.35 ± 0.35** | **2.48 ± 0.03** | **+10.5 %** | 0.386 ± 0.036 |

Χρόνος: ~740 s ανά run (δύο αρχές σε 61–66 σαρώσεις, +3.6–4.1 %)· KISS χωρίς deskew 117 s.

#### Συμπέρασμα ⏳ (1 ακολουθία· βάσεις 1 run)
1. **Η σκάλα λύνεται** με δύο αρχές + indoor_detail: APE 2.07 → 0.48 m. Η αποτυχία ήταν της ρύθμισης (voxel 1 m, εμβέλεια 100 m), όχι της μεθόδου.
2. **Έναντι της δίκαιης βάσης στο ίδιο config (χωρίς deskew) η εικόνα κερδίζει τοπικά:** RPE μετατόπισης −36 %, **στροφής −14 %** (εδώ και η στροφή,
   σε αντίθεση με την ισοπαλία των 16 ακολουθιών), μήκος +20 % → +10.5 %. Στο APE οι τρεις βραχίονες του indoor_detail (0.46–0.54 m) δεν
   διακρίνονται με ένα run βάσης (σ ~0.05 m).
3. Ανοιχτό: αν το indoor_detail βοηθά την εικόνα και σε άλλες εσωτερικές ακολουθίες — εκτός των βραχιόνων σύγκρισης (απόφαση Μ.Τ. 25/9)· μόνο αν
   το ζητήσει ο Μ.Τ.

---

### 2026-09-25 — #061 Αξιολόγηση με το επίσημο πρωτόκολλο (evo) · κάθε θέση στη δική της στιγμή · κλείνει το ανοιχτό πρόβλημα του #045/#046

**Σχετικά:** [#045](#2026-09-23--045-δύο-ακόμη-βάσεις-του-kiss-χωρίς-deskew-indoor_detail-και-η-χρονική-μετατόπιση-της-αξιολόγησης),
[#046](#2026-09-24--046-διόρθωση-του-044-το-κόλλημα-στην-έξοδο-ήταν-kernel-bug-του-οδηγού-ntfs--ανοιχτό-πρόβλημα-από-το-045) §2,
[#059](#2026-09-25--059-δύο-αρχικές-θέσεις-ως-προεπιλογή-4-σπόροι-σε-16-ακολουθίες) · κλάδος `after_two_start` ·
`scripts/evaluate_official.py`, `results_table.py --official` · πίνακας `docs/results_official.md` (αντίγραφο του
`/home/photogrammetry/kiss_runs/results_official.md/.csv`) · TUM εκτιμήσεων και GT για το evo: `/home/photogrammetry/kiss_runs/official_eval/`

#### Στόχος
Ζήτημα Μ.Τ.: τα τεστ να τρέχουν όπως περιγράφουν τα επίσημα εργαλεία αξιολόγησης (evo). Μέχρι τώρα κάθε run αξιολογούνταν στη δική του
καλύτερη χρονική μετατόπιση (#045), ελαφρά αισιόδοξο (#046 §2, STATUS / open_tasks Β.5).

#### Μέθοδος
1. **Το επίσημο πρωτόκολλο.** Στο αποθετήριο του Oxford Spires (`ori-drs/oxford_spires_dataset`, `scripts/localisation_benchmark/*.py`)
   όλες οι μέθοδοι αξιολογούνται με `evo_ape tum gt_lidar.txt <εκτίμηση> --align --t_max_diff 0.01`: APE μετατόπισης (RMSE) μετά από στερεή
   ευθυγράμμιση SE(3), αντιστοίχιση χρονοσφραγίδων εντός 10 ms, **χωρίς μετατόπιση χρόνου**. Τα papers του Newer College δίνουν το ίδιο APE
   και RPE του evo σε 1 m. Υλοποίηση με το API του evo 1.37.1 (ήδη στο env)· το APE ταυτίζεται με την εντολή `evo_ape` (church_03,
   0.027893 m και στα δύο).
2. **Τι στιγμή αντιπροσωπεύει κάθε θέση — μετρημένο.** Η χρονοσφραγίδα της σάρωσης είναι το **πρώτο** σημείο (Hesai: πρώτο σημείο +0.0 ms,
   τελευταίο +99.9 ms· Ouster 2021 ίδια· pcd 2020 +98.3 ms). Το kiss_icp 1.3.0 εκφράζει μια σάρωση με deskew στο **τελευταίο** σημείο
   (`Preprocessing.cpp`: exp((s − 1)·log δ)· επιβεβαίωση στο `tests/test_pose_times.py`). Άρα όλα τα runs μέχρι σήμερα έγραφαν τη θέση του
   τέλους με τη χρονοσφραγίδα της αρχής — 0.1 s νωρίτερα. Μια σάρωση **χωρίς** deskew είναι θολή· το στερεό ταίριασμα πέφτει κοντά στον
   μέσο χρόνο των σημείων (Hesai 0.500, Ouster 2021 0.475, 2020 0.501 της περιστροφής).
3. **Χρόνος θέσης ανά σάρωση στον κώδικα:** `KissSLAM.pose_time_fractions` (1 με deskew από κίνηση· μέσος χρόνος των σημείων εντός εμβέλειας
   χωρίς κίνηση: deskew off, αποτυχία εικόνας, αρχή σταθερής ταχύτητας των δύο αρχών, πρώτες σαρώσεις)· το pipeline γράφει
   `pose_times.csv` και `*_poses_posetime_tum.txt`. Το `*_poses_tum.txt` μένει όπως ήταν. **Τα παλιά runs** δεν ξανατρέχουν: ο χρόνος
   ανακατασκευάζεται από τη σύμβαση του βραχίονα (config + `two_start.csv`)· έναντι του ακριβούς: διαφορά ≤ 0.7 ms (church_03 SURF, 300
   σαρώσεις), ≤ 3.9 ms (math_easy χωρίς deskew).
4. **Ποια δείγματα GT υπάρχουν.** Newer College: 10 Hz πάνω σε κάθε χρονοσφραγίδα σάρωσης. Oxford Spires: πλέγμα 20 Hz **και** οι χρονοσφραγίδες
   ~77 % των σαρώσεων· για το υπόλοιπο 23 % το κοντινότερο δείγμα απέχει 17–24 ms και το evo πετά τη θέση (στήλη «assoc.», 0.64–0.77 στα
   Spires). Κενά στο GT: ≤ 0.3 s. Οι θέσεις χωρίς deskew (+0.05 s) πέφτουν ανάμεσα στα δείγματα· όταν συνδέεται λιγότερο από το μισό, το GT
   παρεμβάλλεται (SLERP) στους χρόνους των θέσεων και το run σημειώνεται (στήλη «GT interp.»: μόνο ο «KISS χωρίς deskew», 16/16).
5. **Μέτρα:** APE (επίσημο)· RPE 1 m (evo, όλα τα ζεύγη, ζεύγη από το GT)· RPE 1 s (ίδιο σφάλμα με το evo, ζεύγη ανά χρόνο — το evo δεν έχει
   βήμα σε δευτερόλεπτα· ταυτίζεται με το evo «10 frames, all pairs» όπου συνδέονται όλες οι θέσεις)· μήκος, z RMSE.
6. Όλα τα υπάρχοντα runs (16 ακολουθίες, 16–34 runs η καθεμία), 130 s.

#### Αποτέλεσμα
Σύγκριση βραχιόνων (`compare_arms.py`, μονάδα η ακολουθία, 16 ακολουθίες):

| δύο αρχές έναντι… | APE | RPE 1 m μετατ. | RPE 1 m στροφή | RPE 1 s μετατ. | RPE 1 s στροφή | μήκος |
|---|---|---|---|---|---|---|
| KISS χωρίς deskew | 13–3, −32 %, p 0.004 | 13–3, −42 %, p 0.002 | 8–8, p 0.74 | 13–3, −38 %, p 0.003 | 8–8, p 0.86 | 14–2, p 0.0002 |
| KISS | 16–0 | 16–0 | 16–0 | 16–0 | 16–0 | 16–0 |
| SURF μία αρχή | 8–8, p 0.98 | 10–6 | 12–4, p 0.03 | 13–3, p 0.04 | 14–1, p 0.009 | 14–2 |

Πόσο αισιόδοξη ήταν η καλύτερη μετατόπιση ανά run (διάμεσος λόγος επίσημο / #045 ανά ακολουθία): APE 0.996–1.002 σε όλους· RPE 1 s μετατόπισης
0.99–1.02· **RPE 1 s στροφής: εικόνα 1.01–1.02, χωρίς deskew 1.01, KISS 1.11, indoor_detail 1.15.**

Οι καλύτερες μετατοπίσεις του #045 εξηγούνται από τη σύμβαση: εικόνα +0.100 s στις quad / cloister / underground (= τελευταίο σημείο), χωρίς
deskew ~0.05 s λιγότερο (πρόβλεψη 0.050). Το υπόλοιπο είναι **χρονική μετατόπιση του ίδιου του GT**, ίδια για όλους τους βραχίονες:
maths institute ~−60 ms (εικόνα +0.035 / +0.040), 01_short ~−120 ms (−0.025), Oxford Spires −5 έως −20 ms (+0.080…+0.095).

Παράδειγμα (δύο αρχές, επίσημο): bodleian APE 0.513 ± 0.087 m (#060, με παρεμβολή σε όλες τις σαρώσεις: 0.79) — το evo κρατά μόνο το 64 % των
θέσεων που έχουν δείγμα GT· church_02 0.209 ± 0.045, Blenheim 0.273 ± 0.021 m.

#### Συμπέρασμα ⏳
1. **Τα συμπεράσματα του #059 στέκουν με το επίσημο πρωτόκολλο:** έναντι KISS χωρίς deskew APE −32 %, RPE μετατόπισης −38…−42 % (p ≤ 0.003),
   μήκος 14–2, στροφή ισοπαλία (8–8). Έναντι KISS 16–0 παντού.
2. **Η «χρονική μετατόπιση ανά run» ήταν η σύμβαση του χρόνου της θέσης, όχι ελεύθερη παράμετρος.** Με κάθε θέση στη στιγμή της, χωρίς
   αναζήτηση, η εικόνα χάνει 1–2 % στο RPE στροφής· ο KISS 11 % (το deskew του είναι λάθος, η αναζήτηση το «μπάλωνε»). Το ανοιχτό πρόβλημα
   του #045/#046 **κλείνει**· προεπιλογή αξιολόγησης από εδώ: `evaluate_official.py` / `results_table.py --official`.
3. **Ανοιχτά:** (α) στα Spires οι βραχίονες αξιολογούνται σε λίγο διαφορετικά υποσύνολα (64–77 % με deskew, 100 % παρεμβολή χωρίς) — όπως στο
   benchmark· ένα κοινό υποσύνολο θα ήταν αυστηρότερο. (β) Η χρονική μετατόπιση του GT στα maths institute (−60 ms) και στο 01_short (−120 ms)
   ανεβάζει το RPE στροφής όλων εκεί — να αναφερθεί, όχι να διορθωθεί.

---

### 2026-09-25 — #060 Bodleian: γιατί χειροτερεύει το ATE με δύο αρχές (ύψος, όχι λάθος επιλογές)

**Σχετικά:** [#059](#2026-09-25--059-δύο-αρχικές-θέσεις-ως-προεπιλογή-4-σπόροι-σε-16-ακολουθίες) · κλάδος `gating` ·
script `/home/photogrammetry/kiss_runs/analyse_bodleian.py`

#### Στόχος
Στο #059 η bodleian-library-02 ήταν η μόνη ακολουθία όπου οι δύο αρχές χειροτέρεψαν το ATE (0.623 ± 0.044 → 0.791 ± 0.135 m) ενώ το RPE βελτιώθηκε
(10.17 → 9.90 cm). Είναι λάθος επιλογές της μεθόδου ή κάτι άλλο;

#### Μέθοδος
1. **Αποφάσεις έναντι GT** (σπόροι 1–3, `two_start.csv`): κάθε σάρωση όπου κρατήθηκε η αρχή σταθερής ταχύτητας (CV) χαρακτηρίζεται ωφέλιμη / βλαβερή /
   ουδέτερη, ανάλογα με το αν η κίνηση που υπολογίστηκε από την CV είναι πιο κοντά στο GT από αυτή της εικόνας.
2. **Προφίλ σφάλματος στον χρόνο** (4 + 4 σπόροι): σφάλμα θέσης ανά 50 s και χειρότερο τοπικό drift 30 s.
3. **Διάσπαση του ATE** σε οριζόντιο (xy) και κατακόρυφο (z) μέρος· drift προσανατολισμού και ύψους από την αρχή ως το τέλος (αγκύρωση στα πρώτα 20 s).

#### Αποτέλεσμα
Αποφάσεις (ανά σπόρο): 810–822 σαρώσεις με διαφωνία > 5°, κρατήθηκε CV σε 120–141· **ωφέλιμες 82–91, βλαβερές 1–3**, ουδέτερες 37–49. Στις ωφέλιμες
το σφάλμα της κίνησης πέφτει (διάμεσος) 3.24° → 1.68° και 29.1 → 15.5 cm. Κέρδος ταιριάσματος στον χάρτη: ωφέλιμες 8.9 % (p10 1.4 %), βλαβερές 3.8 %
(Blenheim: 19.0 % / 1.9 %).

Προφίλ: και οι δύο βραχίονες έχουν μεγάλο σφάλμα στην αρχή (1.3–2.0 m) και στο τέλος (1.1–1.8 m) και μικρό στη μέση (0.1–0.3 m)· το χειρότερο τοπικό drift
30 s είναι στα πρώτα 30 s (5–8 m στους περισσότερους σπόρους **και των δύο**). Στα πρώτα 60 s γίνονται μόνο 5–6 αλλαγές σε CV.

Διάσπαση ATE [m]:

| | ATE | xy RMSE | z RMSE | μήκος έναντι GT |
|---|---|---|---|---|
| SURF σ0–3 | 0.57–0.68 | 0.15–0.17 | 0.55–0.66 | +31.5 έως +32.4 % |
| δύο αρχές σ0–3 | 0.67–0.96 | 0.12–0.17 | 0.65–0.94 | +29.4 έως +29.9 % |
| KISS χωρίς deskew σ0 | 1.36 | 0.28 | 1.33 | +50.0 % |

Drift προσανατολισμού αρχή→τέλος: −0.5° έως +0.1° και στους δύο. Drift ύψους: SURF +0.3 έως +4.1 m, δύο αρχές −1.8 έως +9.8 m (μεγάλη διασπορά και στους δύο).

#### Συμπέρασμα ⏳
- Η διαφορά στο ATE **δεν οφείλεται σε λάθος επιλογές** της μεθόδου: οι αλλαγές σε CV είναι σχεδόν όλες ωφέλιμες ή ουδέτερες (1–3 βλαβερές ανά run).
- Είναι **σχεδόν ολόκληρη κατακόρυφη**: το οριζόντιο σφάλμα είναι ίδιο (0.12–0.17 m), ο προσανατολισμός ίδιος, το z RMSE 0.60 → 0.78 m. Σε διαδρομή 690 m
  χωρίς βαρύτητα στο μοντέλο, μια μικρή κλίση στα πρώτα δευτερόλεπτα (όπου και οι δύο βραχίονες έχουν τοπικό drift 5–8 m) γίνεται μέτρα ύψους στο τέλος·
  το ποιος σπόρος «πέφτει» πιο στραβά είναι σχεδόν τυχαίο (διασπορά ύψους −1.8…+9.8 m).
- Τα τοπικά μέτρα (RPE, μήκος) βελτιώνονται με τις δύο αρχές· η bodleian **δεν** είναι αντεπιχείρημα για την προεπιλογή του #059.
- Ανοιχτό: τι γίνεται στα πρώτα 30 s της bodleian (και οι δύο βραχίονες) — πιθανή αιτία το ξεκίνημα από στάση/χειριστή· και ένα κατακόρυφο μέτρο (π.χ.
  κλίση ως προς τη βαρύτητα) στο εργαλείο αξιολόγησης. Ένα περιθώριο στο κέρδος ταιριάσματος (> 2–4 %) θα έκοβε τις λίγες βλαβερές αλλαγές, αλλά είναι πολύ
  λίγες για να αλλάξουν το ATE.

---

### 2026-09-25 — #059 Δύο αρχικές θέσεις ως προεπιλογή: 4 σπόροι σε 16 ακολουθίες

**Σχετικά:** [#058](#2026-09-25--058-δύο-αρχικές-θέσεις-σε-16-ακολουθίες--εικόνα-απόστασης-ως-εφεδρεία--τρίτη-αρχή--γιατί-δουλεύει-με-έλεγχο-των-αποφάσεων-στο-gt)
(σπόρος 0) · κλάδος `gating` · πίνακες `docs/results_all_best_offset.md`, `docs/comparisons_best_offset.txt`

#### Στόχος
Απόφαση Μ.Τ. 25/9: οι δύο αρχικές θέσεις γίνονται **προεπιλογή** (`image_deskew.two_start_deg = 5`· `none` = μία αρχή, όπως πριν), η εικόνα απόστασης μένει
προαιρετική· έλεγχος με το πρωτόκολλο του #037 (4 σπόροι) σε όλες τις 16 ακολουθίες.

#### Μέθοδος
69 runs: δύο αρχές σπόροι 1–3 σε 16 ακολουθίες (+ blenheim σ3)· SURF μίας αρχής (`--two-start=none`) σπόροι 1–3 στις 8 νεότερες (+ blenheim σ2–3). Έτσι και
οι δύο βραχίονες έχουν 4 σπόρους παντού. Κάθε run στη βέλτιστη χρονική μετατόπιση (#045).

#### Αποτέλεσμα
Μέσος ± σ (4 σπόροι), ATE [m] / RPE μετατόπισης 1 s [cm]:

| ακολουθία | SURF (μία αρχή) | δύο αρχές |
|---|---|---|
| 01_short | 0.264 ± 0.011 / 7.64 | 0.264 ± 0.015 / 7.62 |
| quad_easy | 0.079 / 6.15 | 0.079 / 6.15 |
| stairs | 2.10 ± 0.23 / 20.50 ± 0.78 | 2.06 ± 0.22 / 21.64 ± 0.73 |
| cloister | 0.197 ± 0.024 / 6.65 | 0.189 ± 0.014 / 6.68 |
| math_easy | 0.079 / 6.07 | 0.081 / 6.08 |
| underground_easy | 0.065 / 4.85 | 0.067 / 4.89 |
| christ-church-02 | 0.210 ± 0.049 / 5.54 | 0.211 ± 0.046 / 5.52 |
| christ-church-03 | 0.046 / 3.60 | 0.046 / 3.59 |
| quad_hard | 0.219 ± 0.018 / 9.32 | 0.221 ± 0.012 / 9.29 |
| math_medium | 0.108 / 10.19 | 0.106 / 10.00 |
| underground_medium | 0.059 / 5.01 | 0.061 / 5.08 |
| underground_hard | 0.087 / 6.08 | 0.087 / 6.11 |
| keble-college-03 | 0.096 / 8.49 | 0.095 / 8.39 |
| observatory-quarter-01 | 0.075 ± 0.006 / 6.20 | 0.079 ± 0.022 / 6.12 |
| **blenheim-palace-02** | **3.98 ± 2.42** / 9.86 | **0.275 ± 0.017** / 8.50 |
| **bodleian-library-02** | **0.623 ± 0.044** / 10.17 | **0.791 ± 0.135** / 9.90 |

Ζεύγη ανά ακολουθία (16 ακολουθίες, 4 σπόροι): δύο αρχές έναντι SURF — RPE μετατόπισης 9–7 (p = 0.43), στροφής 11–5 (p = 0.06), ATE 6–9 (p = 0.78),
μήκος 13–3 (−1.4 %, p = 0.025)· έναντι «χωρίς deskew» — ATE 14–2 (−32 %, p = 0.002), RPE μετατόπισης 13–3 (−36 %, p = 0.002), στροφή 8–8· έναντι KISS —
16–0 σε όλα τα μέτρα· έναντι SIFT — RPE και μήκος 16–0, ATE 10–6.

#### Συμπέρασμα ⏳
1. **Με 4 σπόρους επιβεβαιώνεται:** οι δύο αρχικές θέσεις διορθώνουν το Blenheim (3.98 ± 2.42 → 0.275 ± 0.017 m — το SURF αποτυγχάνει σε 3 από 4 σπόρους)
   και δεν αλλάζουν ουσιαστικά 14 από τις άλλες 15 (καμία σημαντική διαφορά σε RPE / ATE· μήκος λίγο καλύτερο).
2. **bodleian-library-02: το ATE χειροτερεύει** (0.62 ± 0.04 → 0.79 ± 0.14 m, διαφορά ~2 σ) **ενώ το RPE βελτιώνεται** (10.17 → 9.90 cm) — προς διερεύνηση
   (821 σαρώσεις με δύο καταχωρίσεις στο σ0, οι περισσότερες από όλες τις ακολουθίες). stairs: RPE μετατόπισης +1.1 cm (~1.5 σ).
3. Έναντι της δίκαιης βάσης (KISS χωρίς deskew) η μέθοδος με δύο αρχές: ATE −32 % και RPE μετατόπισης −36 % σε 16 ακολουθίες (p = 0.002).

---

### 2026-09-25 — #058 Δύο αρχικές θέσεις σε 16 ακολουθίες · εικόνα απόστασης ως εφεδρεία / τρίτη αρχή · γιατί δουλεύει, με έλεγχο των αποφάσεων στο GT

**Σχετικά:** [#057](#2026-09-25--057-icp-από-δύο-αρχικές-θέσεις-διορθώνει-blenheim-keble-και-underground_hard-μαζί--εικόνα-απόστασης-καμία-λάθος-στροφή-στο-blenheim)
(οι δύο αρχικές θέσεις σε 3 ακολουθίες) · κλάδος `gating` · runs `/home/photogrammetry/kiss_runs/**/{surftwo,surftwolog,surfrangefb,surftworange}_s*`
· ανάλυση `/home/photogrammetry/kiss_runs/analyse_058.py`

#### Στόχος
Ζητήματα Μ.Τ. 25/9: (1) οι δύο αρχικές θέσεις σε όλες τις 16 ακολουθίες και το κόστος χρόνου· (2) η εικόνα απόστασης ως τρίτη αρχική θέση· (3) η εικόνα
απόστασης **μόνο όταν το intensity αποτυγχάνει**· εξήγηση σε βάθος του γιατί βοηθούν οι δύο αρχικές θέσεις.

#### Μέθοδος
- Κώδικας: `ScanMotionEstimator(range_motion=...)` — κίνηση και από πανόραμα απόστασης (log απόσταση + CLAHE, SURF κατώφλι 10, δικό του RNG, ώστε η
  εκτίμηση του intensity να μένει ίδια)· «fallback»: όταν το intensity αποτυγχάνει ή απορρίπτεται (gate 25/20)· «candidate»: τρίτη αρχική θέση.
  Η καταχώριση πολλών αρχών γενικεύεται σε εικόνα / απόσταση / σταθερή ταχύτητα· `two_start.csv` καταγράφει κάθε τέτοια σάρωση (ταίριασμα κάθε αρχής,
  ποια κρατήθηκε, η κίνηση στην οποία συνέκλινε η καθεμία)· μετράται ο επιπλέον χρόνος.
- 26 runs: δύο αρχές σε 13 ακολουθίες (σ0)· δύο αρχές με καταγραφή σε blenheim / keble / underground_hard (επανάληψη του #057 — ίδιες τροχιές ως
  1·10⁻¹² ✓)· εφεδρεία απόστασης και τρεις αρχές σε blenheim (σ0–2), keble, underground_hard.
- **Έλεγχος των αποφάσεων στο GT:** για κάθε σάρωση με πολλές αρχές, ποια αρχή συνέκλινε πλησιέστερα στην αληθινή κίνηση (σφάλμα στροφής σε ° + 10 ×
  σφάλμα θέσης σε m), και αν αυτή κρατήθηκε. **Το GT δεν χρησιμοποιείται από τη μέθοδο** — κρίνει μόνο ο τοπικός χάρτης του SLAM.

#### Αποτέλεσμα
**(1) 16 ακολουθίες, σπόρος 0** — ATE [m] / RPE μετατόπισης 1 s [cm], SURF → δύο αρχές · σαρώσεις με δύο καταχωρίσεις (σταθερή ταχύτητα κρατήθηκε) · επιπλέον χρόνος:

| ακολουθία | SURF | δύο αρχές | δύο φορές (CV) | + χρόνος |
|---|---|---|---|---|
| 01_short | 0.270 / 7.63 | 0.277 / 7.65 | 21 (13) | 0.3 % |
| quad_easy | 0.079 / 6.13 | 0.079 / 6.15 | 82 (26) | 2.6 % |
| stairs | 1.932 / 20.45 | 2.175 / 21.41 | 97 (31) | 1.9 % |
| cloister | 0.194 / 6.61 | 0.189 / 6.70 | 69 (15) | 1.0 % |
| math_easy | 0.076 / 6.07 | 0.079 / 6.09 | 11 (3) | 0.3 % |
| underground_easy | 0.065 / 4.93 | 0.066 / 4.96 | 16 (1) | 0.3 % |
| christ-church-02 | 0.254 / 5.53 | 0.261 / 5.51 | 142 (8) | 2.2 % |
| christ-church-03 | 0.046 / 3.59 | 0.045 / 3.60 | 103 (4) | 1.7 % |
| quad_hard | 0.209 / 9.35 | 0.220 / 9.28 | 238 (40) | 7.7 % |
| math_medium | 0.109 / 10.31 | 0.105 / 9.98 | 389 (97) | 12.2 % |
| underground_medium | 0.060 / 5.03 | 0.059 / 5.08 | 95 (10) | 1.6 % |
| underground_hard | 0.087 / 6.09 | 0.086 / 6.06 | 400 (33) | 4.9 % |
| keble-college-03 | 0.095 / 8.54 | 0.095 / 8.38 | 339 (58) | 8.8 % |
| observatory-quarter-01 | 0.074 / 6.16 | 0.065 / 6.13 | 60 (16) | 2.2 % |
| **blenheim-palace-02** | **5.298** / 9.72 | **0.299** / 8.49 | 185 (147) | 3.9 % |
| bodleian-library-02 | 0.623 / 10.06 | 0.959 / 9.83 | 821 (129) | 12.3 % |

Ζεύγη ανά ακολουθία (16): δύο αρχές έναντι SURF — RPE μετατόπισης 9–7 (p = 0.41), στροφής 11–5 (p = 0.07), ATE 8–8, μήκος 14–2 (p < 0.001, −1.7 %)·
έναντι «χωρίς deskew» — ATE 14–2 (−29 %, p = 0.002), RPE μετατόπισης 13–3 (−36 %, p = 0.002), στροφή ισοπαλία.

**(2), (3) Εικόνα απόστασης** — ATE [m] ανά σπόρο (RPE μετατόπισης [cm]):

| | δύο αρχές | gate 25/20 + απόσταση ως εφεδρεία | τρεις αρχές (+ απόσταση) |
|---|---|---|---|
| blenheim-palace-02 | 0.30, 0.26, 0.27 (8.3–8.7) | 0.41, 0.37, 0.46 (9.8–9.9)· απόσταση σε 15–26 σαρώσεις | **0.27, 0.26, 0.27 (8.2–8.4)** |
| keble-college-03 | 0.09 (8.4) | 0.09 (8.5) | 0.09 (8.2) |
| underground_hard | 0.09 (6.1) | 0.09 (6.1) | 0.09 (6.0) |

**(4) Οι αποφάσεις έναντι του GT:**
- **blenheim-palace-02 (δύο αρχές):** 185 σαρώσεις με δύο καταχωρίσεις, κρατήθηκε η σταθερή ταχύτητα σε 147. Η αρχή της εικόνας συνέκλινε > 10 μακριά από την
  αλήθεια σε **18** σαρώσεις — σε **17 από τις 18 κρατήθηκε η άλλη** (π.χ. σάρωση 445: εικόνα 24.1 → σταθερή ταχύτητα 2.7). Σφάλμα του αποτελέσματος p99:
  αρχή εικόνας 18.8, **κρατημένο 7.7**, καλύτερο δυνατό 7.4. Η πλησιέστερη στην αλήθεια κρατήθηκε σε 82 %.
- **keble-college-03:** η εικόνα δεν συνέκλινε ποτέ > 10 μακριά· οι αρχές έδιναν σχεδόν ίδια αποτελέσματα (κρατημένο p99 8.8, εικόνα 9.4)· «η πλησιέστερη»
  μόνο σε 56 % επειδή οι διαφορές είναι μικρές — δεν μετράει.
- **underground_hard:** στις σαρώσεις 1486–1489 (οι ίδιες που έριξαν το gate, #055) **όλες** οι αρχές συνέκλιναν 13–29 μακριά — ο ICP κράτησε την εικόνα και
  η τροχιά συνήλθε στις επόμενες σαρώσεις (ATE 0.09 m).
- Με τρεις αρχές κρατήθηκε συχνά η απόσταση (33–215 σαρώσεις), χωρίς ουσιαστική διαφορά στο αποτέλεσμα.

#### Συμπέρασμα ⏳ (1 σπόρος στις 13, 3 στο Blenheim)
1. **Οι δύο αρχικές θέσεις διορθώνουν το Blenheim (5.30 → 0.30 m) χωρίς να χαλάσουν τις άλλες 15:** καμία σημαντική διαφορά από το SURF σε RPE / ATE
   (μήκος ελαφρά καλύτερο 14/16). Εξαίρεση προς παρακολούθηση: bodleian 0.62 → 0.96 m ATE με καλύτερο RPE (1 σπόρος· #037: το ATE ενός run δεν κρίνει).
   Κόστος χρόνου 0.3–12 % (όσο πιο γρήγορη κίνηση, τόσο περισσότερες σαρώσεις δύο φορές).
2. **Γιατί δουλεύει (επιβεβαιωμένο στο GT):** ο ICP συγκλίνει στο πλησιέστερο τοπικό ελάχιστο· η αρχή της εικόνας είναι σχεδόν πάντα καλύτερη, αλλά στις
   επαναλαμβανόμενες προσόψεις δίνει μια ψευδή συναίνεση εκτός της περιοχής σύγκλισης· η σταθερή ταχύτητα δεν είναι ποτέ τόσο λάθος. Αφού συγκλίνουν και οι
   δύο, η λάθος λύση αφήνει σημεία μακριά από τις επιφάνειες του χάρτη — έτσι ο χάρτης (όχι κατώφλι, όχι GT) βρίσκει τη σωστή: στο Blenheim 17 από τις 18
   επικίνδυνες σαρώσεις. Όταν όλες οι αρχές είναι κακές (underground_hard 1486–1489) δεν βοηθά, αλλά ούτε χειροτερεύει.
3. **Εικόνα απόστασης:** ως εφεδρεία όταν το intensity αποτυγχάνει/απορρίπτεται — διορθώνει το Blenheim (0.37–0.46 m) αλλά λιγότερο από τις δύο αρχές·
   ως τρίτη αρχή — ίδιο ή ελάχιστα καλύτερο από τις δύο αρχές (Blenheim 0.26–0.27, RPE 8.2–8.4), με το κόστος μιας δεύτερης εικόνας σε κάθε σάρωση.
   **Πρόταση: οι δύο αρχικές θέσεις ως προεπιλογή· η απόσταση ως προαιρετική επέκταση.**

---

### 2026-09-25 — #057 ICP από δύο αρχικές θέσεις: διορθώνει Blenheim, keble και underground_hard μαζί · εικόνα απόστασης: καμία λάθος στροφή στο Blenheim

**Σχετικά:** [#055](#2026-09-24--055-όρια-25--20-το-blenheim-μένει-διορθωμένο-αλλά-13-απορρίψεις-ρίχνουν-keble-και-underground_hard--φταίει-η-εφεδρεία),
[#056](#2026-09-25--056-εναλλακτικές-για-την-εφεδρεία-σταθερή-επιτάχυνση--εικόνα-απόστασης-χρειάζεται-δουλειά-μερίδιο-αντιστοιχιών---offline-έλεγχοι)
· κλάδος `gating` · runs `/home/photogrammetry/kiss_runs/**/surftwo_s*`, `surfgate25id_s*`

#### Στόχος
Ζητήματα Μ.Τ. 25/9: (1) δύο αρχικές θέσεις για τον ICP· (2) εικόνα απόστασης + intensity μαζί στις σαρώσεις που αποτυγχάνουν· (3) όταν ο έλεγχος απορρίπτει,
**καθόλου deskew** (ταυτοτική, χωρίς σταθερή ταχύτητα).

#### Μέθοδος
- **(1)** `image_deskew.two_start_deg` (`--two-start=5`): όταν η κίνηση της εικόνας και η σταθερή ταχύτητα του KISS διαφέρουν > 5° σε στροφή, η σάρωση
  καταχωρίζεται δύο φορές — (A) deskew + αρχή από την εικόνα, (B) χωρίς deskew, αρχή από τη σταθερή ταχύτητα — και κρατείται εκείνη με το μικρότερο
  περικομμένο μέσο της απόστασης κάθε σημείου από τον τοπικό χάρτη (KD-tree μόνο σε αυτές τις σαρώσεις). Χωρίς έλεγχο αληθοφάνειας.
- **(2)** offline, ίδιο RANSAC + fit χρόνου: εικόνα απόστασης = log απόσταση 1–60 m με τοπική αντίθεση (CLAHE 4×16), SURF κατώφλι 10· «μαζί» = οι
  αντιστοιχίσεις των δύο εικόνων σε μία εκτίμηση.
- **(3)** `--gate --gate-rot=25 --gate-drot=20 --fallback=identity`. **Σφάλμα του launcher στην πορεία:** η πρώτη εκτέλεση έχασε το `--fallback=identity`
  (περνούσαν μόνο 6 λέξεις ανά εργασία) — τα 5 runs ήταν επανάληψη του «gate + σταθερή ταχύτητα» (ταυτόσημα ως 5·10⁻¹³, άρα ντετερμινιστικά)· ξανάτρεξαν.

#### Αποτέλεσμα
**SLAM, ATE [m] / RPE μετατόπισης 1 s [cm] ανά σπόρο:**

| | χωρίς deskew | SURF | gate 25/20 + σταθ. ταχύτητα | gate 25/20 + χωρίς deskew | **δύο αρχικές θέσεις** |
|---|---|---|---|---|---|
| blenheim-palace-02 | 0.20 / 8.5 | 5.30, 5.15 | 0.59, 0.62, 0.37 | 0.35, 0.35, 0.33 / 9.7 | **0.30, 0.26, 0.27 / 8.3–8.7** |
| keble-college-03 | 11.37 | 0.10 / 8.5 | 3.93 | 2.19 | **0.09 / 8.4** |
| underground_hard | 12.34 | 0.09 / 6.1 | 3.16 | 0.09 / 6.1 | **0.09 / 6.1** |

Δύο αρχικές θέσεις: 175–400 σαρώσεις καταχωρίστηκαν δύο φορές ανά run· η αρχή σταθερής ταχύτητας κρατήθηκε σε 33–58 (Blenheim) και 141–148 (keble,
underground_hard) από αυτές.

**Εικόνα απόστασης (offline), σφάλμα στροφής p90 / max [°] / σαρώσεις > 5°:**

| | intensity | απόσταση | μαζί |
|---|---|---|---|
| blenheim 396–479 | 6.67 / 35.5 / 12 | **1.88 / 4.3 / 0** (33 αποτυχίες) | 1.73 / 52.2 / 2 |
| keble 1131–1169 | 3.61 / 6.0 / 2 | 3.34 / 4.4 / 0 | 3.46 / 5.1 / 1 |
| underground_hard 1467–1505 | 5.66 / 17.4 / 5 | 2.16 / 18.1 / 3 | 2.71 / 16.0 / 3 |

#### Συμπέρασμα ⏳ (1–3 σπόροι, 3 ακολουθίες)
1. **Οι δύο αρχικές θέσεις διορθώνουν και τις τρεις** (Blenheim 5.2 → 0.26–0.30 m, με RPE ίσο του «χωρίς deskew»· keble, underground_hard ανέπαφα) —
   χωρίς κατώφλι για το τι είναι «λάθος»: αποφασίζει το ταίριασμα στον χάρτη. Η καλύτερη λύση ως τώρα.
2. **«Χωρίς deskew» ως εφεδρεία είναι καλύτερη από τη σταθερή ταχύτητα** (Blenheim 0.33–0.35, underground_hard 0.09) αλλά δεν σώζει το keble (2.19 m).
3. **Η εικόνα απόστασης (με CLAHE) δεν ξεγελιέται από τις επαναλαμβανόμενες προσόψεις** (Blenheim: καμία στροφή > 5°) και σε γρήγορη κίνηση έχει μικρότερη
   ουρά σφάλματος από το intensity· αποτυγχάνει όμως συχνότερα. Η απλή συγχώνευση κληρονομεί τη λάθος συναίνεση του intensity (52°)· καλύτερα ως
   **διασταύρωση** ή ως τρίτη αρχική θέση.
4. Επόμενα: οι δύο αρχικές θέσεις σε όλες τις 16 ακολουθίες (+ κόστος χρόνου)· η εικόνα απόστασης ως διασταύρωση / τρίτη αρχική θέση.

---

### 2026-09-25 — #056 Εναλλακτικές για την εφεδρεία: σταθερή επιτάχυνση ✗, εικόνα απόστασης (χρειάζεται δουλειά), μερίδιο αντιστοιχιών ✗ — offline έλεγχοι

**Σχετικά:** [#055](#2026-09-24--055-όρια-25--20-το-blenheim-μένει-διορθωμένο-αλλά-13-απορρίψεις-ρίχνουν-keble-και-underground_hard--φταίει-η-εφεδρεία)
(η εφεδρεία σταθερής ταχύτητας ως αδύναμος κρίκος) · κλάδος `gating` · μόνο offline, χωρίς runs SLAM

#### Στόχος
Ιδέες Μ.Τ. 25/9: (α) **σταθερή επιτάχυνση** αντί για σταθερή ταχύτητα όταν ο έλεγχος απορρίπτει· (β) **εικόνα απόστασης** για αντιστοιχίσεις όπου το intensity αποτυγχάνει.

#### Αποτέλεσμα
**(α) Πρόβλεψη της επόμενης κίνησης από τις προηγούμενες (αληθινές, από GT), σφάλμα στροφής p50 / p90 [°]:**

| | keble-03 | underground_hard | blenheim-02 | church_03 |
|---|---|---|---|---|
| ταυτοτική (παλιά εφεδρεία) | 1.84 / 4.12 | 3.11 / 7.75 | 1.13 / 2.19 | 1.67 / 3.87 |
| σταθερή ταχύτητα | **1.74 / 3.87** | **2.37 / 5.06** | **1.04 / 2.08** | **1.47 / 3.03** |
| σταθερή επιτάχυνση | 2.61 / 6.24 | 3.02 / 6.43 | 1.47 / 3.18 | 1.99 / 4.36 |
| μισή επιτάχυνση | 2.02 / 4.90 | 2.51 / 5.31 | 1.16 / 2.46 | 1.59 / 3.41 |

Η σταθερή επιτάχυνση είναι **χειρότερη** από τη σταθερή ταχύτητα στη στροφή παντού: η στροφή του χεριού ταλαντώνεται από σάρωση σε σάρωση (#019), και η
παρέκταση της μεταβολής της υπερβάλλει. (Η μετατόπιση είναι ίδια ή λίγο καλύτερη με τη μισή επιτάχυνση.)

**(β) Εικόνα απόστασης** (log απόσταση 1–60 m → 0–255, το ίδιο πανόραμα, SURF), blenheim σαρώσεις 440–445: SURF κατώφλι 100 → **15–31 σημεία ανά σάρωση,
6–12 αντιστοιχίσεις** (το intensity: ~1390 σημεία, 137–162 αντιστοιχίσεις)· με κατώφλι 10 → 200–350 σημεία, 49–73 αντιστοιχίσεις. Η λογαριθμική εικόνα
απόστασης είναι πολύ ομαλή· χρειάζεται άλλη αναπαράσταση (ακμές / τοπική αντίθεση) πριν κριθεί.

**(γ) Νέο εύρημα για το Blenheim:** στο σημείο της αποτυχίας το intensity **δεν** έχει λίγα χαρακτηριστικά — 137–162 αντιστοιχίσεις ratio test ανά σάρωση·
μετά τους ελέγχους μένουν ~21–27. Άρα οι περισσότερες αντιστοιχίσεις είναι **λάθος**, και οι καταστροφικές στροφές είναι λάθος συναίνεση του RANSAC —
πιθανή αιτία η επανάληψη στις προσόψεις (όμοια παράθυρα). **Το μερίδιο των αντιστοιχιών που συμφωνούν δεν ξεχωρίζει** τις κακές σαρώσεις: κακές
0.16–0.25· καλές p10 0.13–0.32, διάμεσος 0.21–0.41 (blenheim, keble, underground_hard).

#### Συμπέρασμα ⏳
1. Η σταθερή επιτάχυνση ως εφεδρεία **απορρίπτεται** (χειρότερη πρόβλεψη στροφής σε 4/4 ακολουθίες).
2. Η εικόνα απόστασης είναι ρεαλιστική ιδέα (Ι-4) αλλά χρειάζεται καλύτερη αναπαράσταση — έργο έρευνας, όχι γρήγορη διόρθωση.
3. Η αποτυχία του Blenheim οφείλεται σε **λάθος αντιστοιχίσεις που συμφωνούν μεταξύ τους**, όχι σε έλλειψη αντιστοιχιών· κανένα απλό μέτρο της εικόνας
   (πλήθος, μερίδιο, στροφή) δεν τις ξεχωρίζει καθαρά. Η πιο γερή λύση μένει ο **ICP από δύο αρχικές θέσεις** (αποφασίζει ο χάρτης).

---

### 2026-09-24 — #055 Όρια 25° / 20°: το Blenheim μένει διορθωμένο, αλλά 1–3 απορρίψεις ρίχνουν keble και underground_hard — φταίει η εφεδρεία

**Σχετικά:** [#054](#2026-09-24--054-έλεγχος-αληθοφάνειας-της-κίνησης-της-εικόνας--εφεδρεία-σταθερής-ταχύτητας-διορθώνει-το-blenheim-αλλά-10--8-χαλούν-τις-γρήγορες-ακολουθίες)
(ο ίδιος έλεγχος με 10° / 8°· εκεί «σε εξέλιξη») · κλάδος `gating` · runs `/home/photogrammetry/kiss_runs/**/surfgate25_s*`

#### Αποτέλεσμα (ATE [m] / RPE μετατόπισης 1 s [cm]· χωρίς deskew · SURF χωρίς έλεγχο · 10°/8° · 25°/20°)
| ακολουθία | χωρίς deskew | SURF | 10° / 8° | 25° / 20° | απορρίψεις στα 25/20 |
|---|---|---|---|---|---|
| blenheim-palace-02 | 0.205 / 8.46 | 5.223 / 9.82 | 0.384 / 9.82 | **0.529 / 11.30** (σ0–2: 0.59, 0.62, 0.38) | 84–91 |
| keble-college-03 | 11.37 / 21.78 | 0.096 / 8.54 | 12.56 / 10.21 | **3.929** / 8.96 | 1 |
| underground_hard | 12.34 / 129.1 | 0.087 / 6.09 | 1.782 / 13.46 | **3.162** / 6.56 | 3 |
| underground_medium | 0.098 / 8.88 | 0.060 / 5.03 | 1.060 / 8.90 | 0.060 / 5.03 | 0 |
| math_medium | 0.134 / 7.99 | 0.109 / 10.31 | 1.251 / 10.35 | 0.116 / 10.32 | 4 (αποτυχίες) |
| quad_hard | 0.206 / 14.29 | 0.209 / 9.35 | 0.238 / 11.09 | 0.208 / 9.36 | 32 |
| christ-church-02 / -03 | 0.547 / 0.089 | 0.210 / 0.046 | 0.281 / 0.051 | 0.254 / 0.045 | 0 / 0 |

**Οι απορρίψεις που έριξαν keble και underground_hard:** keble σάρωση 1150 — εικόνα 15.7° (μεταβολή 22.9°), αληθινή 8.7°· underground_hard 1486 —
εικόνα 25.5°, αληθινή 11.2° (και 1488: εικόνα 13.3°, αληθινή 11.3°). Οι κινήσεις ήταν **λάθος κατά 7–14°** — ο ICP συνέρχεται από αυτές (χωρίς έλεγχο
ATE 0.10 / 0.09 m)· δεν συνέρχεται από τη **σταθερή ταχύτητα** στη μέση γρήγορης στροφής (~11° ανά σάρωση). Στο Blenheim οι κινήσεις ήταν λάθος κατά
27–47° — εκεί ο ICP δεν συνέρχεται και η εφεδρεία βοηθά.

#### Συμπέρασμα ⏳
1. Κανένα σταθερό όριο δεν καλύπτει και τα δύο: το κρίσιμο δεν είναι αν η κίνηση της εικόνας είναι λάθος, αλλά **από ποια αρχική θέση συνέρχεται ο ICP**.
2. **Η εφεδρεία σταθερής ταχύτητας είναι ο αδύναμος κρίκος** σε γρήγορη κίνηση.
3. Επόμενο (πρόταση): **ICP από δύο αρχικές θέσεις** (εικόνα και σταθερή ταχύτητα) στις ύποπτες σαρώσεις, κρατώντας εκείνη με το καλύτερο ταίριασμα
   στον χάρτη — χωρίς κατώφλι που να αποφασίζει τι είναι «λάθος».

---

### 2026-09-24 — #054 Έλεγχος αληθοφάνειας της κίνησης της εικόνας + εφεδρεία σταθερής ταχύτητας: διορθώνει το Blenheim, αλλά 10° / 8° χαλούν τις γρήγορες ακολουθίες

**Σχετικά:** [#053](#2026-09-24--053-η-αποτυχία-στο-blenheim-palace-02-λίγες-αντιστοιχίσεις--στροφές-3050--surf-με-περισσότερα-χαρακτηριστικά-δεν-αρκεί)
(η διάγνωση και η πρόταση) · κλάδος **`gating`** · runs `/home/photogrammetry/kiss_runs/**/surfgate_s*`, `surfcheck_s0`

#### Στόχος
Απόφαση Μ.Τ. 24/9: υλοποίηση των προτάσεων 1 (έλεγχος αληθοφάνειας) και 2 (εφεδρεία) του #053 σε νέο κλάδο· έλεγχος ότι δεν αλλάζει τίποτα όπου
η μέθοδος ήδη δούλευε· Blenheim· **φύλαξη των πανοραμάτων intensity των ζευγών που απορρίπτονται** για οπτικό έλεγχο.

#### Μέθοδος
- `ScanMotionEstimator`: απορρίπτει κίνηση με < `gate_min_matches` αντιστοιχίσεις, στροφή > `gate_max_rotation_deg` σε μία σάρωση, ή στροφή που
  διαφέρει από την τελευταία αποδεκτή κατά > `gate_max_rotation_change_deg` (κάθε None = ανενεργό, η προεπιλογή)· `image_deskew.fallback =
  "constant_velocity"`: χωρίς deskew, αρχική θέση ICP από τη σταθερή ταχύτητα του KISS (πριν: ταυτοτική). Για κάθε σάρωση που απορρίπτεται ή
  αποτυγχάνει, στο `<run>/rejected_pairs/`: `scanNNNNN_a_previous.png`, `_b_current.png` (τα πανοράματα), `_c_matches.png` (οι αντιστοιχίσεις του
  ratio test), και γραμμή στο `rejected.csv` (σάρωση, χρόνος, αιτία, αντιστοιχίσεις, σημεία ανά εικόνα, στροφή).
- **Κατώφλι αντιστοιχιών εγκαταλείφθηκε:** με 30, στις 600 πρώτες σαρώσεις του Blenheim απορρίφθηκαν 352 (λίγες αντιστοιχίσεις παντού), ενώ οι
  δύο καταστροφικές (30°, 50°) είχαν 26 και 20. Προεπιλογή του `--gate`: μόνο στροφή ≤ 10°, μεταβολή ≤ 8°.
- 16 ακολουθίες × σπόρος 0 + Blenheim σπόροι 1–2· έλεγχος «gate off» σε church_03, quad_easy.

#### Αποτέλεσμα
- **Gate off = ο παλιός κώδικας:** τροχιές ίδιες ως 1.4·10⁻¹³ (church_03), 9.9·10⁻¹⁴ (quad_easy).
- **Blenheim: ATE 5.22 → 0.38 m** (μέσος 3 σπόρων)· απορρίψεις 145–153 ανά run (80–86 από αυτές αποτυχίες του εκτιμητή).
- **Ίδιο περίπου** (ATE / RPE μετατόπισης, χωρίς → με gate): 01_short 0.264 / 7.64 → 0.265 / 7.66· quad_easy ίδιο· math_easy, underground_easy,
  observatory ίδια· church_02 0.210 → 0.281, church_03 0.046 → 0.051 m (2–25 απορρίψεις).
- **Χαλάει τις γρήγορες:** keble-college-03 **0.096 → 12.56 m**, underground_hard 0.087 → 1.78, underground_medium 0.060 → 1.06, math_medium
  0.109 → 1.25 m (53–231 απορρίψεις, σχεδόν όλες από στροφή / μεταβολή, όχι αποτυχίες).
- **Αιτία:** η πραγματική στροφή ανά σάρωση (GT) φτάνει 13.5–21.6° (max) σε quad_hard, underground_medium/hard, keble· μεταβολή έως 11–15°. Τα όρια
  10° / 8° απορρίπτουν **σωστές** κινήσεις, και η σταθερή ταχύτητα είναι κακή ακριβώς εκεί. Στο Blenheim οι επιβλαβείς τιμές ήταν 30° και 50°, ενώ η
  αληθινή κίνηση ≤ 7.7°.

#### Συμπέρασμα ⏳
Ο έλεγχος + η εφεδρεία διορθώνουν την αποτυχία του #053, αλλά **σταθερά όρια 10° / 8° είναι πολύ στενά για φορητό αισθητήρα σε γρήγορη κίνηση**.
Σε εξέλιξη: όρια **25° / 20°** (πάνω από κάθε αληθινή τιμή σε 16 ακολουθίες, κάτω από τα 30° / 50° του Blenheim). Αν δεν αρκεί: ICP από δύο
αρχικές θέσεις όταν διαφωνούν (#053, πρόταση 3), χωρίς κατώφλια.

---

### 2026-09-24 — #053 Η αποτυχία στο blenheim-palace-02: λίγες αντιστοιχίσεις → στροφές 30–50° · SURF με περισσότερα χαρακτηριστικά δεν αρκεί

**Σχετικά:** [#052](#2026-09-24--052-8-νέες-ακολουθίες-ncd-2021-mediumhard-4-τοποθεσίες-oxford-spires--8-βραχίονες--το-bug-του-ntfs-επιμένει)
(όπου φάνηκε η αποτυχία) · κλάδος `map_sharpness` · runs `/home/photogrammetry/kiss_runs/oxford_spires_full/{blenheim_02,church_03}/`

#### Στόχος
Γιατί το SURF στο blenheim-palace-02 έχει ATE 5.3 m με καλό RPE (#052)· ζητήματα Μ.Τ. 24/9: δεύτερος σπόρος, SURF «με περισσότερα χαρακτηριστικά».

#### Μέθοδος
(α) Σφάλμα θέσης και στροφής ανά 1 s κατά μήκος της τροχιάς· (β) δεύτερος σπόρος· (γ) η κίνηση της εικόνας για τις σαρώσεις 400–480 απευθείας
έναντι του GT· (δ) κατώφλι Hessian του SURF 100 → 50 / 25 / 10 στο ίδιο κομμάτι, και πλήρη runs με 25 (σ0) και 10 (σ0, σ1) + church_03 με 10.

#### Αποτέλεσμα
- **Ένα γεγονός, όχι σταδιακό drift:** στα 43–46 s σφάλμα στροφής έως **42° μέσα σε 1 s**· μετά λάθος κατεύθυνση ως το τέλος (απόσταση από GT
  20–56 m). Κανένα κλείσιμο βρόχου σε κανένα run — δεν φταίει ένωση. **Ο σπόρος 1 αποτυγχάνει στο ίδιο σημείο** (44.6 s, ATE 5.15 m) → συστηματικό.
- **Η κίνηση της εικόνας εκεί** (σαρώσεις 400–470): 15–43 αντιστοιχίσεις (αλλού 80–250)· 10 σαρώσεις με σφάλμα > 5°, 7 αποτυχίες (ταυτοτική)·
  δύο καταστροφικές: σάρωση 445 (GT 2.7°, εικόνα **30.0°**), σάρωση 456 (GT 3.0°, εικόνα **50.1°**). Ο ICP ξεκινά από αυτές και κλειδώνει λάθος.
- **Κατώφλι Hessian** στο κομμάτι: οι αντιστοιχίσεις μένουν ~26–27 σε κάθε κατώφλι (περισσότερα σημεία, όχι περισσότερες αξιόπιστες
  αντιστοιχίσεις)· σφάλματα > 20° υπάρχουν σε 100 / 50 / 25, όχι σε 10.
- **Πλήρη runs:** ATE — κατώφλι 100: 5.30 (σ0), 5.15 (σ1)· **25: 0.31 (σ0)· 10: 0.31 (σ0), 5.50 (σ1)**· χωρίς deskew 0.21. Αποτυχίες εικόνας 81–83 →
  58–63. church_03 με 10: ίδιο με 100 (ATE 0.046, RPE 3.52 έναντι 3.59 cm).

#### Συμπέρασμα ⏳
1. Η αποτυχία προέρχεται από **σημείο με φτωχή υφή intensity**: λίγες αντιστοιχίσεις, και κάποιες σαρώσεις δίνουν αδύνατη στροφή (30–50° σε 0.1 s)·
   ο ICP δεν συνέρχεται από τόσο λάθος αρχική θέση.
2. **Περισσότερα χαρακτηριστικά SURF μειώνουν τις αποτυχίες (−25 %) αλλά δεν διορθώνουν** (1 από 3 runs αποτυγχάνει ξανά)· δεν βλάπτουν αλλού.
3. **Προτεινόμενη διόρθωση** (δεν έχει γίνει): έλεγχος αληθοφάνειας κάθε κίνησης της εικόνας (≥ ~30 αντιστοιχίσεις, στροφή ≤ ~10° ανά σάρωση,
   όχι απότομο άλμα από την προηγούμενη) και, όταν απορρίπτεται, **αρχική θέση από τη σταθερή ταχύτητα του KISS** αντί για ταυτοτική. Επόμενα:
   ICP από δύο αρχικές θέσεις όταν διαφωνούν· εικόνα απόστασης (Ι-4) για περισσότερες αντιστοιχίσεις όπου το intensity είναι επίπεδο.

---

### 2026-09-24 — #052 8 νέες ακολουθίες (NCD 2021 medium/hard, 4 τοποθεσίες Oxford Spires) × 8 βραχίονες · το bug του NTFS επιμένει

**Σχετικά:** [#047](#2026-09-24--047-4-σπόροι-στατιστικός-έλεγχος-σε-8-ακολουθίες-ablation-της-κίνησης-της-εικόνας-ο-loader-του-01_short)
(οι ίδιοι έλεγχοι σε 8 ακολουθίες), [#046](#2026-09-24--046-διόρθωση-του-044-το-κόλλημα-στην-έξοδο-ήταν-kernel-bug-του-οδηγού-ntfs--ανοιχτό-πρόβλημα-από-το-045)
και [#050](#2026-09-24--050-νέες-θέσεις-των-αποτελεσμάτων-του-048049--ο-δίσκος-ntfs-μετά-την-επανεκκίνηση) (το bug) · πίνακες
`docs/results_all_best_offset.md`, `docs/results_all.md`, έλεγχοι `docs/comparisons_best_offset.txt`

#### Στόχος
Ζήτημα Μ.Τ. 24/9: οι νέες λήψεις στους φακέλους τους και ένα run (σπόρος 0) ανά ακολουθία με όλους τους βραχίονες, στους πίνακες.

#### Μέθοδος
- **Δεδομένα:** NCD 2021 quad_hard, math_medium, underground_medium, underground_hard (bags = μέγεθος του Drive· GT από το Drive με
  έγκριση Μ.Τ., 0.7 MB· κάθε νέα ακολουθία σε δικό της υποφάκελο, αλλιώς ο loader τη συγχωνεύει με το math_easy)· Oxford Spires
  keble-college-03, observatory-quarter-01, blenheim-palace-02, bodleian-library-02 (κάθε bag ανοίγει και καλύπτει το GT του).
- 8 ακολουθίες × 8 βραχίονες (KISS, χωρίς deskew, indoor_detail, SIFT, SURF, SURF μόνο μετατόπιση / μόνο στροφή / εξομάλυνση 3), σπόρος 0,
  ρύθμιση του paper· αξιολόγηση στη βέλτιστη χρονική μετατόπιση ανά run (#045).
- **Το bug του ntfs3 επιμένει στον πυρήνα 7.0.0-34:** 24/9 17:32 `kernel BUG at fs/iomap/buffered-io.c:1061` σκότωσε το blenheim_02 SURF μόνο
  μετατόπιση ενώ έγραφε· ξανάτρεξε με έξοδο σε ext4. **Απόφαση Μ.Τ. 24/9: ο δίσκος δεδομένων μόνο για ανάγνωση**· όλες οι έξοδοι στο
  `/home/photogrammetry/kiss_runs/` (CLAUDE.md, «Τρεις παγίδες»).
- **Ταχύτερη αξιολόγηση:** RPE διανυσματοποιημένο (ίδια νούμερα, 21×) + cache ανά run· ο πίνακας βέλτιστης μετατόπισης 45 min → 149 s
  (1 s με cache)· οι 65 γραμμές που υπήρχαν ήδη ταυτίζονται.

#### Αποτέλεσμα
**Νέες ακολουθίες** — RPE μετατόπισης 1 s [cm] / ATE [m]:

| ακολουθία (μήκος GT) | KISS | χωρίς deskew | indoor_detail | SIFT | SURF | μόνο στροφή | μόνο μετατόπιση | εξομάλυνση 3 |
|---|---|---|---|---|---|---|---|---|
| quad_hard (237 m) | 18.99 / 0.33 | 14.29 / 0.21 | 13.01 / 0.16 | 12.27 / **1.68** | **9.35** / 0.21 | 9.63 / 0.19 | 13.06 / 0.21 | 10.26 / 0.21 |
| math_medium (304 m) | 18.18 / 0.22 | **7.99** / 0.13 | 17.03 / 0.89 | 10.97 / 0.11 | 10.31 / 0.11 | 9.55 / 0.12 | 8.93 / 0.21 | 9.25 / 0.11 |
| underground_medium (174 m) | 14.45 / 0.16 | 8.88 / 0.10 | 8.50 / 0.10 | 5.08 / 0.05 | **5.03** / 0.06 | 5.76 / 0.09 | 10.92 / 1.20 | 6.13 / 0.07 |
| underground_hard (237 m) | 296.48 / **12.79** | 129.08 / **12.34** | 20.02 / 3.84 | 6.46 / 0.09 | **6.09 / 0.09** | 7.26 / 0.12 | 31.16 / 6.48 | 9.13 / 0.11 |
| keble-college-03 (284 m) | 51.80 / **9.48** | 21.78 / **11.37** | 38.83 / 0.87 | 8.56 / 0.09 | **8.54** / 0.10 | 8.10 / 0.11 | 22.02 / 2.30 | 11.59 / 0.13 |
| observatory-quarter-01 (397 m) | 17.51 / 0.49 | 9.69 / 0.44 | 11.99 / 0.17 | 6.64 / 0.10 | **6.16 / 0.07** | 5.50 / 0.09 | 11.42 / 0.44 | 7.90 / 0.13 |
| blenheim-palace-02 (392 m) | 17.41 / 0.30 | **8.46 / 0.20** | 13.35 / 0.55 | 21.26 / **3.69** | 9.72 / **5.30** | 8.63 / 5.10 | 9.73 / 0.19 | 10.12 / 0.40 |
| bodleian-library-02 (690 m) | 46.34 / 2.93 | 15.36 / 1.36 | 32.19 / 4.46 | 11.34 / **21.42** | **10.06 / 0.62** | 9.15 / 0.68 | 17.04 / 1.23 | 14.40 / 1.16 |

**Όλες οι 16 ακολουθίες** (SURF έναντι βάσης, νίκες SURF–βάσης · διάμεσος σχετικής διαφοράς · Wilcoxon p):

| έναντι | RPE μετατ. | RPE στροφής | ATE | μήκος |
|---|---|---|---|---|
| KISS | 16–0 · −65 % · <0.001 | 16–0 · −57 % · <0.001 | 15–1 · −50 % · 0.003 | 16–0 · −82 % · <0.001 |
| **χωρίς deskew** | **13–3 · −36 % · 0.003** | 8–8 · +7 % · 0.94 | **14–2 · −34 % · 0.004** | 13–3 · −32 % · 0.005 |
| indoor_detail | 15–1 · −40 % · 0.002 | 15–1 · −49 % · 0.002 | 12–4 · −58 % · 0.09 | 15–1 · −66 % · 0.002 |
| SIFT | 16–0 · −6 % · <0.001 | 16–0 · −9 % · <0.001 | 9–7 · 0.56 | 16–0 · −16 % · <0.001 |

Ablation (έναντι SURF): μόνο μετατόπιση χειρότερο (ATE +105 %, p = 0.005)· εξομάλυνση χειρότερη (RPE +16 %, p = 0.002)· μόνο στροφή: ίδιο RPE,
μήκος καλύτερο 16/16, ATE χειρότερο 13/16 (+10 %, p = 0.04).

#### Συμπέρασμα ⏳ (1 σπόρος στις νέες)
1. **Εκεί όπου ο KISS αποτυγχάνει, η εικόνα τον σώζει:** underground_hard (KISS 12.8 m, χωρίς deskew 12.3 m → SURF 0.09 m) και keble-college-03
   (9.5 / 11.4 m → 0.10 m). Αυτό είναι το ζητούμενο της ιδέας.
2. **Σε 16 ακολουθίες** τα ευρήματα του #047 ισχύουν και ενισχύονται: έναντι του «χωρίς deskew» ATE 14/16 (−34 %), RPE μετατόπισης 13/16 (−36 %),
   στροφή ισοπαλία· SURF > SIFT σε RPE / μήκος 16/16.
3. **Νέο πρόβλημα: αποτυχίες ATE της εικόνας με καλό RPE** — blenheim-palace-02 (SURF 5.30 m, SIFT 3.69 m, έναντι 0.20 m χωρίς deskew) και
   SIFT σε bodleian-library-02 (21.4 m) και quad_hard (1.68 m). Μικρό σφάλμα ανά δευτερόλεπτο αλλά μεγάλο ATE → πιθανώς ένα τοπικό άλμα ή λάθος
   ένωση. 1 σπόρος: δεν ξέρουμε αν είναι συστηματικό. **Επόμενο: διερεύνηση του blenheim-palace-02** (πού, πότε, πόσοι σπόροι).

---

### 2026-09-24 — #051 Ευκρίνεια χάρτη στο christ-church-02 · το church_03 με το 100 % των σαρώσεων

**Σχετικά:** [#049](#2026-09-24--049-ευκρίνεια-χάρτη-στο-christ-church-03-hesai-έναντι-του-tls-χάρτη-του-oxford-spires) (church_03 με 10 %),
[#050](#2026-09-24--050-νέες-θέσεις-των-αποτελεσμάτων-του-048049--ο-δίσκος-ntfs-μετά-την-επανεκκίνηση) (δίσκος) · κλάδος `map_sharpness` ·
runs `…/data/runs/map_sharpness/church_02/`, `…/church_03_full/` (δίσκος δεδομένων, 19 GB)

#### Στόχος
Απόφαση Μ.Τ. 24/9: αποθήκευση στον δίσκο δεδομένων (τώρα με χώρο), το church_03 με το **100 %** κάθε σάρωσης, και το **church_02**.

#### Μέθοδος
Όπως #049 (6 βραχίονες, σπόρος 0, voxel 5 cm, χάρτης TLS του Christ Church σε 2 cm) αλλά χωρίς δειγματοληψία (`--save-fraction` = 1).
Έξοδος στον δίσκο NTFS (ntfs3, πυρήνας 7.0.0-34). GT church_02: `gt-tum_church_2.txt`, `--frame=spires`.

#### Αποτέλεσμα — ποσοστό σημείων εντός 5 cm (διάμεσος)

| | χωρίς deskew | KISS | indoor_detail | SIFT | SURF | SURF μόνο στροφή |
|---|---|---|---|---|---|---|
| church_02, μόνο deskew (GT) | 78.2 (2.31 cm) | 48.7 (5.18) | 54.9 (4.35) | 80.3 (2.20) | **81.4 (2.13)** | 74.8 (2.53) |
| church_02, χάρτης όπως φτιάχνεται | 37.4 (7.26) | 30.6 (9.11) | 21.1 (13.91) | 52.4 (4.70) | 50.0 (5.00) | **52.5 (4.68)** |
| church_03 100 %, μόνο deskew (GT) | 78.3 | 56.0 | 58.8 | 81.7 | **82.2** | 75.9 |
| church_03 100 %, χάρτης όπως φτιάχνεται | 65.3 | 63.2 | 62.6 | **82.9** | 81.2 | 76.4 |

- **church_03: 100 % ≈ 10 %** (διαφορές ≤ 0.1 ποσοστιαίας μονάδας) → η δειγματοληψία 10 % του #049 αρκεί.
- **church_02, χάρτης όπως φτιάχνεται: μερικώς έγκυρο** — τροχιά 642 m με drift χωρίς κλείσιμο βρόχου· το ICP μετακίνησε τους χάρτες
  15–208 cm, εντός 0.5 m 60–95 %. Η σειρά (εικόνα 50–53 % > χωρίς deskew 37 % > KISS 31 % > indoor_detail 21 %) συμφωνεί με το ATE.
- **Κανένα kernel BUG του ntfs3** στα 19 GB εγγραφής (12 διεργασίες παράλληλα).

#### Συμπέρασμα ⏳ (1 σπόρος)
1. **Και στο church_02 το deskew της εικόνας δίνει τον πιο ευκρινή χάρτη** (SURF 81.4 % εντός 5 cm έναντι 78.2 % χωρίς deskew, 48.7 % KISS)·
   το deskew του KISS θολώνει τον χάρτη περισσότερο από κάθε άλλη ακολουθία.
2. Σε σύνολο 4 ακολουθιών (quad, cloister, church_02, church_03): η εικόνα είναι η πιο ευκρινής σε 3, ισοπαλία με το «χωρίς deskew» στο quad·
   το KISS είναι το χειρότερο παντού· «μόνο στροφή» πάντα κάτω από το πλήρες.
3. Ο δίσκος NTFS δέχτηκε μεγάλο παράλληλο φορτίο εγγραφής χωρίς το bug του #046 — ένδειξη ότι ο πυρήνας 7.0.0-34 το διορθώνει.

---

### 2026-09-24 — #050 Νέες θέσεις των αποτελεσμάτων του #048–#049 · ο δίσκος NTFS μετά την επανεκκίνηση

**Σχετικά:** [#048](#2026-09-24--048-ευκρίνεια-του-χάρτη-έναντι-του-τοπογραφικού-χάρτη-αναφοράς-quad_easy-cloister),
[#049](#2026-09-24--049-ευκρίνεια-χάρτη-στο-christ-church-03-hesai-έναντι-του-tls-χάρτη-του-oxford-spires) (οι διαδρομές τους άλλαξαν),
[#046](#2026-09-24--046-διόρθωση-του-044-το-κόλλημα-στην-έξοδο-ήταν-kernel-bug-του-οδηγού-ntfs--ανοιχτό-πρόβλημα-από-το-045) (το bug του ntfs3)

- **Επανεκκίνηση (24/9, απόφαση Μ.Τ.):** πυρήνας 7.0.0-31 → **7.0.0-34**· οι διεργασίες-zombie του #046 χάθηκαν. Ο δίσκος NTFS προσαρτάται
  ξανά με τον **ntfs3**, χωρίς σφάλματα στο kernel log. Έλεγχος μόνο με ανάγνωση: 30/30 αρχεία Newer College στο ακριβές μέγεθος του Drive,
  15 302 σαρώσεις του 01_short, τα bags των church αμετάβλητα, 135/135 τροχιές runs αναγνώσιμες.
- **Μεταφορά** (ο δίσκος συστήματος ήταν στο 100 %): `/home/photogrammetry/kiss_runs/map_sharpness/` (25 GB) → **`…/data/runs/map_sharpness/`**,
  `/home/photogrammetry/kiss_runs/prior_maps/` (3.1 GB) → **`…/data/prior_maps/`**. rsync, έλεγχος byte-προς-byte (0 διαφορές), μετά διαγραφή
  των αρχικών. Ο δίσκος συστήματος: 3.7 → 32 GB ελεύθερα. **Κανένα kernel BUG του ntfs3 στα 28 GB εγγραφής** — ένδειξη, όχι απόδειξη, ότι ο
  νέος πυρήνας δεν έχει το bug (και πριν οι μεγάλες σειριακές εγγραφές, π.χ. λήψεις, δεν το προκαλούσαν).
- Στο ext4 έμειναν οι μικρές επανεκτελέσεις του #046 (`/home/photogrammetry/kiss_runs/`, 0.2 GB)· οι φάκελοί τους συγκρούονται με τους
  μισογραμμένους του δίσκου NTFS.

---

### 2026-09-24 — #049 Ευκρίνεια χάρτη στο christ-church-03 (Hesai) έναντι του TLS χάρτη του Oxford Spires

**Σχετικά:** [#048](#2026-09-24--048-ευκρίνεια-του-χάρτη-έναντι-του-τοπογραφικού-χάρτη-αναφοράς-quad_easy-cloister) (ίδια μέθοδος, Newer College) ·
κλάδος `map_sharpness` · πίνακας `docs/map_sharpness.md` · χάρτης `/home/photogrammetry/kiss_runs/prior_maps/christ-church-merged-cloud-1cm.pcd`
(Oxford Spires στο Hugging Face, `ground_truth_map/christ-church/merged-cloud-1cm.pcd`, 2.74 GB, λήψη με έγκριση Μ.Τ. 24/9)

#### Στόχος
Ζήτημα Μ.Τ. 24/9: ο έλεγχος του #048 για όλους τους βραχίονες στο church_03, με σπόρο 0, και ο χάρτης αναφοράς του.

#### Μέθοδος
Όπως το #048: 6 βραχίονες (KISS, χωρίς deskew, indoor_detail, SIFT, SURF, SURF μόνο στροφή), `--topic=/hesai/pandar
--intensity-scale=1.0`, σαρώσεις με deskew σε voxel 5 cm. **Νέο:** κρατείται τυχαίο 10 % των σημείων κάθε σάρωσης
(`--save-fraction=0.1`, `diagnostics.save_deskewed_fraction`)· ο δίσκος συστήματος ήταν στο 100 % (6.5 GB ελεύθερα· ένα run του cloister
είχε γράψει 2.1 GB). Ο χάρτης TLS (114 M σημεία, χωρίς normals) περικόπτεται γύρω από το run και αραιώνεται σε 2 cm πριν από τον υπολογισμό
normals (`--prior-voxel=0.02`). `--frame=spires`.

**Δύο αστοχίες στην πορεία (χωρίς επίπτωση στα αποτελέσματα):** (α) η πρώτη εκτέλεση διακόπηκε σκόπιμα πριν γράψει, για να μη γεμίσει ο δίσκος
συστήματος· (β) το αρχείο «FINISHED» εκείνης της εκτέλεσης υπήρχε ακόμη, οπότε η βαθμολόγηση ξεκίνησε πριν τελειώσουν 4 runs — ξανάγινε με
όλα τα runs τελειωμένα (οι αριθμοί των «δικών θέσεων» ταυτίστηκαν).

#### Αποτέλεσμα — ποσοστό σημείων εντός 5 cm από τον χάρτη TLS (διάμεσος απόστασης σε παρένθεση)

| | χωρίς deskew | KISS | indoor_detail | SIFT | SURF | SURF μόνο στροφή |
|---|---|---|---|---|---|---|
| **μόνο deskew (θέσεις GT)** | 78.4 (2.29 cm) | 55.9 (4.21) | 58.8 (3.86) | 81.7 (2.04) | **82.2 (2.01)** | 75.9 (2.35) |
| **ο χάρτης όπως φτιάχνεται** | 65.3 (3.44) | 63.2 (3.60) | 62.6 (3.70) | **82.9 (2.17)** | 81.1 (2.26) | 76.4 (2.68) |

Εντός 0.5 m: 98.2–98.9 % σε όλα (ο χάρτης TLS καλύπτει το church_03)· το τελικό ICP μετακίνησε τους χάρτες ≤ 5 cm.

#### Συμπέρασμα ⏳ (1 σπόρος)
1. **Στο church_03 η εικόνα δίνει σαφώς τον πιο ευκρινή χάρτη**, και στους δύο τρόπους: ως χάρτης όπως φτιάχνεται 81–83 % εντός 5 cm έναντι
   62–65 % για κάθε μορφή του KISS — η μεγαλύτερη διαφορά από τις τρεις ακολουθίες του ελέγχου.
2. **Το deskew του KISS θολώνει έντονα** (55.9 % έναντι 78.4 % χωρίς deskew στο GT)· το indoor_detail δεν το διορθώνει (58.8 %).
3. **«Μόνο στροφή» και εδώ χειρότερο από το πλήρες** (75.9 έναντι 82.2 %) — επιβεβαιώνει το #048: η μετατόπιση μέσα στη σάρωση μετρά για τον χάρτη.
4. Σε σύνολο 3 ακολουθιών (#048, #049): η εικόνα είναι η πιο ευκρινής σε cloister και church_03, ισοπαλία με το «χωρίς deskew» στο quad_easy.

---

### 2026-09-24 — #048 Ευκρίνεια του χάρτη έναντι του τοπογραφικού χάρτη αναφοράς (quad_easy, cloister)

**Σχετικά:** [#047](#2026-09-24--047-4-σπόροι-στατιστικός-έλεγχος-σε-8-ακολουθίες-ablation-της-κίνησης-της-εικόνας-ο-loader-του-01_short)
(ablation «μόνο στροφή») · κλάδος `map_sharpness` · πίνακας `docs/map_sharpness.md` (csv στο `docs/map_sharpness/`) · runs
`/home/photogrammetry/kiss_runs/map_sharpness/` · χάρτης `/home/photogrammetry/kiss_runs/prior_maps/new-college-combined-5cm-v2.ply`
(Drive του Newer College, 509 MB, λήψη με έγκριση Μ.Τ. 24/9)

#### Στόχος
Ζήτημα (m) του καταλόγου ελέγχων, απόφαση Μ.Τ. 24/9: μέτρο που κρίνει **το ίδιο το deskew**, ανεξάρτητο από το πρόβλημα της χρονικής
μετατόπισης του RPE (#046): πόσο «λεπτές» βγαίνουν οι επιφάνειες όταν οι σαρώσεις κάθε βραχίονα μπαίνουν στον χάρτη, έναντι του
τοπογραφικού χάρτη του dataset. Για 2 από τις μικρότερες ακολουθίες.

#### Μέθοδος
- `diagnostics.save_deskewed_voxel` (`run_ncd.py --save-frames=0.05`): κάθε σάρωση όπως έγινε deskew (σύστημα αισθητήρα, voxel 5 cm) →
  `deskewed_frames.npz`. 6 βραχίονες × σπόρος 0: KISS, KISS χωρίς deskew, KISS indoor_detail, SIFT, SURF, SURF μόνο στροφή.
- `scripts/map_sharpness.py`: χάρτης = σαρώσεις στις θέσεις (α) του **GT** (στη βέλτιστη μετατόπιση του run) — μετρά ΜΟΝΟ το deskew — ή
  (β) του ίδιου του run (ευθυγράμμιση στο GT)· μετά ένα άκαμπτο point-to-plane ICP όλου του χάρτη στον χάρτη αναφοράς· απόσταση
  point-to-plane 2 εκατ. τυχαίων σημείων· εκτός όσα απέχουν > 0.5 m (περιοχές εκτός τοπογράφησης).
- **Το stairs δεν μετριέται:** το εσωτερικό του κλιμακοστασίου δεν είναι στον χάρτη αναφοράς (25–36 % εντός 0.5 m ακόμη και στο GT).
  Αντ' αυτού το **cloister** (429 m), στον ίδιο χάρτη. Έλεγχος της μεθόδου στο quad_easy: στο GT 99.5 % εντός 0.5 m, διάμεσος 5.6 cm
  χωρίς ICP — το σύστημα του GT είναι του χάρτη (και το T_base_os-sensor βελτιώνει: 6.1 → 5.6 cm).

#### Αποτέλεσμα (`docs/map_sharpness.md`)
**Μόνο deskew (σαρώσεις στο GT)** — διάμεσος [cm] / ποσοστό < 5 cm:

| | χωρίς deskew | KISS | indoor_detail | SIFT | SURF | SURF μόνο στροφή |
|---|---|---|---|---|---|---|
| quad_easy | **2.73 / 71.8** | 3.14 / 65.7 | 3.23 / 64.5 | 2.88 / 69.2 | 2.77 / 70.9 | 2.84 / 70.1 |
| cloister | 2.22 / 83.2 | 2.56 / 75.0 | 2.27 / 80.6 | **1.85 / 89.6** | 1.88 / 89.0 | 2.82 / 73.5 |

**Ο χάρτης όπως φτιάχνεται (δικές του θέσεις):** quad_easy — SURF **1.86 / 88.7**, SIFT 1.93 / 87.2, μόνο στροφή 1.94 / 87.1, χωρίς deskew
2.06 / 84.4, indoor_detail 2.14 / 82.6, KISS 2.17 / 82.4. **Cloister: μη έγκυρο** — 35–86 % εντός 0.5 m και το ICP μετακίνησε τους
χάρτες 1–5 m: η τροχιά των 429 m έχει drift χωρίς κλείσιμο βρόχου, ένα άκαμπτο ICP δεν ταιριάζει όλο τον χάρτη.

#### Συμπέρασμα ⏳ (1 σπόρος, 2 ακολουθίες)
1. **Το deskew του KISS θολώνει τον χάρτη** και στις δύο (και στο GT: 65.7 έναντι 71.8 %, 75.0 έναντι 83.2 % εντός 5 cm) — ανεξάρτητη
   επιβεβαίωση του #045/#047 χωρίς RPE.
2. **Η εικόνα δίνει τον πιο ευκρινή χάρτη στο cloister** (SIFT / SURF 89–90 % εντός 5 cm έναντι 83 % χωρίς deskew) και στον χάρτη όπως
   φτιάχνεται στο quad (SURF 88.7 έναντι 84.4 %)· στο quad, μόνο με deskew (GT), ισοπαλία με το «χωρίς deskew» (70.9 έναντι 71.8 %).
3. **«Μόνο στροφή» ≠ πλήρες για την ευκρίνεια:** στο cloister 73.5 % έναντι 89.0 % — η μετατόπιση μέσα στη σάρωση (~10 cm στο βάδισμα)
   θολώνει τον χάρτη αν δεν διορθωθεί, αν και στα RPE / ATE του #047 δεν φάνηκε. Άρα η πρόταση του #047 να γίνει «μόνο στροφή» η
   προεπιλογή **δεν στέκει** όπου μετρά ο χάρτης.
4. Όρια: οι αποστάσεις είναι κοντά στο «πάτωμα» του χάρτη των 5 cm (~1–2 cm)· οι θέσεις του GT έχουν δικό τους θόρυβο (#045) — γι' αυτό
   στο quad ο χάρτης με τις δικές του θέσεις βγαίνει πιο ευκρινής απ' ό,τι στο GT· 1 σπόρος. Για γερό συμπέρασμα: 4 σπόροι, χάρτης 1 cm,
   και στο math_easy (χάρτης maths-institute, 184 MB).

---

### 2026-09-24 — #047 4 σπόροι, στατιστικός έλεγχος σε 8 ακολουθίες, ablation της κίνησης της εικόνας· ο loader του 01_short

**Σχετικά:** [#045](#2026-09-23--045-δύο-ακόμη-βάσεις-του-kiss-χωρίς-deskew-indoor_detail-και-η-χρονική-μετατόπιση-της-αξιολόγησης)
(η υπόθεση για τη στροφή που ελέγχεται εδώ), [#046](#2026-09-24--046-διόρθωση-του-044-το-κόλλημα-στην-έξοδο-ήταν-kernel-bug-του-οδηγού-ntfs--ανοιχτό-πρόβλημα-από-το-045)
(4 runs χάθηκαν στο bug του ntfs3 και ξανάτρεξαν) · κλάδος `ablation_seeds` · πίνακες `docs/results_all_best_offset.md`,
`docs/results_all.md`, έλεγχοι `docs/comparisons_best_offset.txt` · runs στους φακέλους του #043–#045 και `/home/photogrammetry/kiss_runs/`

#### Στόχος
Απόφαση Μ.Τ. 24/9: (γ) έλεγχος της βάσης έναντι του paper, (β) περισσότεροι σπόροι + στατιστικός έλεγχος, (ε) ablation: ποιο κομμάτι
της κίνησης της εικόνας βοηθά. Αφορμή το #045: η εικόνα κερδίζει στη μετατόπιση αλλά όχι στη στροφή — υπόθεση «θορυβώδης στροφή».

#### Μέθοδος
- **(γ)** ATE με το `evo` (το εργαλείο του paper) έναντι του δικού μας· KISS-SLAM στο 01_short με τον loader NCD του kiss_icp
  (`kiss_slam_pipeline --dataloader ncd`: x,y,z μόνο, όλα τα 65 536 σημεία, συνθετικοί χρόνοι).
- **(β)** SIFT / SURF σπόροι 2–3 στις 7 ακολουθίες που είχαν 2 → 4 σπόροι παντού. `scripts/compare_arms.py`: μονάδα η ακολουθία
  (μέσος των σπόρων), ζεύγη ανά ακολουθία, νίκες, διάμεσος σχετικής διαφοράς, Wilcoxon signed-rank (δίπλευρο· με 8 ακολουθίες το
  ελάχιστο p = 0.0078, μόνο όταν ένας βραχίονας κερδίζει και τις 8).
- **(ε)** `image_deskew.use_parts` / `rotation_smoothing` (για deskew ΚΑΙ αρχική θέση, #030): SURF μόνο μετατόπιση, μόνο στροφή,
  στροφή = μέσος των 3 τελευταίων (0.3 s)· 2 σπόροι × 8 ακολουθίες.
- Όλα στην καλύτερη χρονική μετατόπιση ανά run (#045· ανοιχτό πρόβλημα #046). Ρύθμιση του paper.

#### Αποτέλεσμα
**(γ) Βάση και εργαλείο.** Το ATE μας = του evo (διαφορά ≤ 0.003 m σε 6 ακολουθίες). Ο KISS-SLAM αναπαράγει τον Πίνακα V: stairs
3.582 (paper 3.58), math_easy 0.150 (0.15), cloister 0.396 (0.40). Στο 01_short, με τον loader του kiss_icp: **ATE 0.314 m (paper
0.30)** — με τον δικό μας reader 0.391. Η διαφορά του 01_short είναι λοιπόν ο loader (συνθετικοί χρόνοι / όλα τα σημεία), όχι η ρύθμιση
ή η αξιολόγηση. quad_easy (0.119 έναντι 0.16) και ορυχείο (0.170 έναντι 0.12) μένουν χωρίς εξήγηση.

**(β) SURF έναντι βάσεων, 8 ακολουθίες, 4 σπόροι** (νίκες SURF–βάσης · διάμεσος σχετικής διαφοράς · p):

| έναντι | RPE μετατ. | RPE στροφής | ATE | μήκος | z RMSE |
|---|---|---|---|---|---|
| KISS-SLAM | 8–0 · −64 % · 0.008 | 8–0 · −67 % · 0.008 | 8–0 · −47 % · 0.008 | 8–0 · −80 % · 0.008 | 8–0 · −51 % · 0.008 |
| **KISS χωρίς deskew** | **7–1 · −34 % · 0.039** | **4–4 · +0.4 % · 0.84** | **8–0 · −26 % · 0.008** | 6–2 · −37 % · 0.039 | 8–0 · −25 % · 0.008 |
| SIFT | 8–0 · −6 % · 0.008 | 8–0 · −9 % · 0.008 | 5–3 · −1 % · 0.64 | 8–0 · −14 % · 0.008 | 6–2 · −2 % · 0.37 |

**(ε) Ablation** (παραλλαγή έναντι πλήρους SURF):

| παραλλαγή | RPE μετατ. | RPE στροφής | ATE | μήκος |
|---|---|---|---|---|
| μόνο μετατόπιση (χωρίς στροφή) | 1–7 · **+74 %** · 0.023 | 4–4 · 1.0 | 0–8 · **+105 %** · 0.008 | 1–7 · +71 % · 0.016 |
| μόνο στροφή (χωρίς μετατόπιση) | 4–4 · +1 % · 0.64 | 6–2 · −0.4 % · 0.64 | 1–7 · +8 % · 0.055 | **8–0 · −15 % · 0.008** |
| στροφή εξομαλυμένη (3) | 2–6 · +12 % · 0.055 | 3–5 · +9 % · 0.84 | 1–7 · +18 % · 0.20 | 1–7 · +28 % · 0.039 |
| μόνο μετατόπιση έναντι **χωρίς deskew** | 1–7 · +7 % · 0.20 | 5–3 · 0.55 | 2–6 · 0.38 | 1–7 · 0.20 |

RPE μετατόπισης [cm] / ATE [m] ανά ακολουθία (χωρίς deskew · SURF · μόνο στροφή · μόνο μετατόπιση): church_02 11.53/0.547 · 5.54/0.210 ·
5.39/0.219 · 12.70/0.798 — church_03 7.31/0.089 · 3.60/0.046 · 3.76/0.059 · 7.53/0.091 — cloister 11.72/0.478 · 6.65/0.197 · 7.06/0.200 ·
12.98/0.492 — math_easy 4.51/0.087 · 6.07/0.079 · 5.50/0.069 · 5.34/0.165 (πλήρης πίνακας: `docs/results_all_best_offset.md`).

#### Συμπέρασμα ⏳
1. **Έναντι της δίκαιης βάσης (KISS χωρίς deskew) η εικόνα κερδίζει με στατιστική σημασία σε ATE (8/8, −26 %), z (8/8) και RPE
   μετατόπισης (7/8, −34 %)· στη στροφή ισοπαλία (4–4).** Με 4 σπόρους το ATE ξεχωρίζει πλέον (στο #045, με 2, όχι).
2. **Η αξία της εικόνας είναι η ΣΤΡΟΦΗ μέσα στη σάρωση.** Χωρίς αυτήν (μόνο μετατόπιση) το αποτέλεσμα πέφτει στο επίπεδο του «χωρίς
   deskew» (καμία σημαντική διαφορά)· μόνο με τη στροφή είναι όσο καλό όσο το πλήρες, και με μήκος διαδρομής καλύτερο σε 8/8 (−15 %).
   Η μετατόπιση της εικόνας (με τη μεροληψία κοντινού πεδίου του #039) προσθέτει λίγο ή τίποτα.
3. **Η υπόθεση του #045 («θορυβώδης στροφή») απορρίπτεται:** η εξομάλυνση χειροτερεύει (μήκος +28 %, p = 0.039), συνεπές με το #019
   (η στροφή αλλάζει από σάρωση σε σάρωση σχεδόν όσο είναι). Το γιατί η εικόνα δεν κερδίζει στο RPE στροφής μένει ανοιχτό.
4. **SURF > SIFT** σε RPE και μήκος 8/8 (p = 0.008), όχι στο ATE.
5. Υποψήφιο επόμενο: «μόνο στροφή» ως προεπιλογή (ίδιο RPE / ATE, μικρότερη περίσσεια μήκους)· χρειάζεται 4 σπόρους για να κριθεί.

---

### 2026-09-24 — #046 Διόρθωση του #044: το «κόλλημα στην έξοδο» ήταν kernel BUG του οδηγού NTFS · ανοιχτό πρόβλημα από το #045

**Σχετικά:** [#044](#2026-09-23--044-oxford-spires-christ-church-02---03-ολοκληρες-οι-εγγραφες-ρυθμιση-του-paper) (λάθος διάγνωση),
[#045](#2026-09-23--045-δύο-ακόμη-βάσεις-του-kiss-χωρίς-deskew-indoor_detail-και-η-χρονική-μετατόπιση-της-αξιολόγησης) (ανοιχτό πρόβλημα) ·
κλάδος `ablation_seeds`

#### 1. Το «κόλλημα στην έξοδο» του #044 — λάθος διάγνωση
Στο #044 γράφτηκε ότι ένα run (church_03, KISS, σπόρος 0) «κόλλησε στην έξοδο» από τα νήματα μιας δεξαμενής νημάτων. **Λάθος.**
Το kernel log δείχνει **5 kernel oops από την εκκίνηση**, όλα `kernel BUG at fs/iomap/buffered-io.c:1061` μέσα στο
`ntfs_file_write_iter [ntfs3]`: η διεργασία πέθανε **μέσα στον πυρήνα** την ώρα που έγραφε στον δίσκο δεδομένων (NTFS,
`/dev/nvme0n1p3`, οδηγός `ntfs3`, πυρήνας 7.0.0-31). Το κύριο νήμα χάνεται, τα υπόλοιπα μένουν σε futex, η διεργασία μένει zombie
και **δεν σκοτώνεται** ως την επανεκκίνηση.

| Ώρα | Run | Συνέπεια |
|---|---|---|
| 23/9 18:28 | church_03 KISS σ0 (#044) | η τροχιά είχε γραφτεί· ταυτίζεται με τον σ1 (1.6·10⁻¹³) → έγκυρη |
| 24/9 00:41 | 01_short, KISS με τον loader του kiss_icp (έλεγχος του paper) | χάθηκε |
| 24/9 01:08 | cloister SURF μόνο μετατόπιση σ1 (#047) | χάθηκε |
| 24/9 01:13 | church_02 SIFT σ3 (#047) | χάθηκε |
| 24/9 01:31 | 01_short SURF εξομάλυνση 3 σ1 (#047) | χάθηκε |

Το `os._exit(0)` που προστέθηκε στο `run_ncd.py` για το υποτιθέμενο κόλλημα δεν βοηθά (δεν φτάνει ποτέ εκεί) — μένει, είναι αβλαβές.
**Μέτρα:** τα 4 χαμένα runs ξανατρέχουν με έξοδο στον δίσκο Linux (`/home/photogrammetry/kiss_runs/`, ext4)· ο δίσκος NTFS μόνο
διαβάζεται· ο `results_table.py` μετρά ένα run μόνο αν το log του έχει την τελική γραμμή του `/usr/bin/time`. **Προτάσεις (αλλαγές
συστήματος, απόφαση Μ.Τ.):** επανεκκίνηση· mount του NTFS με `ntfs-3g` αντί για `ntfs3`, ή εγγραφές μόνο σε ext4· `chkdsk` από
Windows.

#### 2. Ανοιχτό πρόβλημα από το #045: η χρονική στιγμή της θέσης κάθε βραχίονα
Η αναζήτηση της καλύτερης μετατόπισης ανά run (`evaluate_ncd.py --offset=best`, #045) είναι ελαφρά αισιόδοξη: η μετατόπιση
επιλέγεται με το ίδιο μέτρο που αναφέρεται. Σωστή λύση: μετατόπιση ανά βραχίονα από τη σύμβαση του deskew (αναλυτικά), ή επιλογή στο
μισό της ακολουθίας και αξιολόγηση στο άλλο. **Αναβάλλεται — απόφαση Μ.Τ. 24/9 (υπερβολικό κόστος προς το παρόν).** STATUS §2.8.

---

### 2026-09-23 — #045 Δύο ακόμη βάσεις του KISS (χωρίς deskew· indoor_detail) και η χρονική μετατόπιση της αξιολόγησης

**git commit:** κλάδος `baselines_timeshift` · runs στους ίδιους φακέλους (`kissnodeskew_s0`, `kissdetail_s0`) · πίνακες
`docs/results_all.md` (χρόνοι όπως καταγράφηκαν) και **`docs/results_all_best_offset.md`** (κάθε run στη δική του καλύτερη μετατόπιση)

#### Στόχος
Ζήτημα Μ.Τ.: ο σκέτος KISS-SLAM σε όλες τις 8 ακολουθίες, μία φορά, (α) με `configs/indoor_detail.yaml` (voxel 0.25 m, εμβέλεια 50 m,
χάρτες 15 m, deskew on) και (β) με τη ρύθμιση του paper **χωρίς deskew** (`configs/kiss_paper_nodeskew.yaml`).

#### Μέθοδος
`run_ncd.py kiss … --config=…`, 8 runs παράλληλα. Κατά την ανάλυση: το RPE στροφής ενός φορητού αισθητήρα **διπλασιάζεται μέσα σε
50 ms μετατόπισης χρόνου**, και οι βραχίονες διαφέρουν στη στιγμή της σάρωσης που αντιπροσωπεύει η θέση τους (deskew on / off / εικόνα).
Άρα σε μία σταθερή μετατόπιση (0) η σύγκριση ευνοεί όποιον βραχίονα τύχει να ταιριάζει. `evaluate_ncd.py --offset=best`: κάθε run
αξιολογείται στη μετατόπιση (−0.15…+0.25 s, βήμα 5 ms) που ελαχιστοποιεί το δικό του RPE στροφής 1 s· η μετατόπιση αναφέρεται.
Διόρθωση στην πορεία: το μήκος του GT υπολογίζεται πλέον από τα **ακατέργαστα** δείγματα του GT· από GT παρεμβεβλημένο στους χρόνους
των σαρώσεων μίκραινε με τη μετατόπιση (01_short: −8 % στα −45 ms), γιατί η παρεμβολή εξομαλύνει τον θόρυβο του ίδιου του GT.

Παράδειγμα (church_03, RPE στροφής 1 s σε μετατόπιση −0.10 / 0 / +0.05 / +0.10 s): χωρίς deskew 3.48 / 1.33 / **0.58** / 1.49° ·
εικόνα + SURF 4.59 / 2.64 / 1.42 / **0.68**°. Καλύτερη μετατόπιση: KISS και χωρίς deskew +0.045–0.060 s, εικόνα +0.095–0.100 s στα
Oxford Spires· 0 / −0.005 / +0.035 στο math_easy.

#### Αποτέλεσμα — κάθε run στη δική του καλύτερη μετατόπιση (`docs/results_all_best_offset.md`)

| Ακολουθία | RPE 1 s [cm]: χωρίς deskew · indoor_detail · KISS · SIFT · SURF | RPE 1 s [°]: ίδια σειρά | ATE [m]: ίδια σειρά |
|---|---|---|---|
| 01_short | 8.47 · 15.47 · 20.54 · 8.10 · **7.64** | **0.552** · 0.978 · 1.039 · 0.681 · 0.633 | 0.326 · 0.901 · 0.388 · **0.262** · 0.264 |
| quad_easy | 6.27 · 9.67 · 10.02 · 6.99 · **6.13** | **0.370** · 0.944 · 0.814 · 0.767 · 0.671 | 0.082 · 0.108 · 0.106 · 0.082 · **0.079** |
| stairs | 68.18 · **11.77** · 100.71 · 23.62 · 20.70 | 13.13 · **3.10** · 27.13 · 5.32 · 4.35 | 2.715 · **0.456** · 3.582 · 1.748 · 2.153 |
| cloister | 11.72 · 8.39 · 19.22 · 6.97 · **6.62** | 0.974 · 1.512 · 2.233 · 0.831 · **0.768** | 0.478 · **0.175** · 0.393 · 0.188 · 0.179 |
| math_easy | **4.51** · 8.58 · 10.34 · 6.63 · 6.08 | **0.286** · 1.070 · 1.181 · 0.796 · 0.728 | 0.087 · 0.188 · 0.150 · 0.090 · **0.077** |
| underground_easy | 6.48 · 6.13 · 10.28 · 4.93 · **4.89** | 0.773 · 1.588 · 1.976 · 0.690 · **0.631** | 0.092 · 0.071 · 0.122 · 0.066 · **0.065** |
| church_02 | 11.53 · 17.81 · 23.96 · 5.89 · **5.54** | 0.835 · 2.100 · 2.310 · 0.786 · **0.721** | 0.547 · 3.088 · 0.788 · 0.330 · **0.234** |
| church_03 | 7.31 · 10.93 · 14.51 · 3.69 · **3.58** | **0.578** · 2.033 · 2.234 · 0.704 · 0.674 | 0.089 · 0.123 · 0.146 · **0.045** · 0.046 |

Μήκος διαδρομής έναντι GT (ίδιο στους δύο πίνακες): χωρίς deskew +2.7…+28 % (stairs +180 %)· indoor_detail +8…+56 %· KISS +18…+92 %
(stairs +319 %)· SURF +3.5…+16 % (stairs +66 %).

Καλύτερος ανά ακολουθία (8): RPE μετατόπισης SURF 6 · χωρίς deskew 1 (math) · indoor_detail 1 (stairs)· RPE στροφής χωρίς deskew 4 ·
SURF 3 · indoor_detail 1· ATE SURF 4 · SIFT 2 · indoor_detail 2. (1 run ανά βάση του KISS — ντετερμινιστικός· 2–4 σπόροι για την εικόνα.)

#### Συμπέρασμα ⏳
1. **Το deskew του KISS («όπως η προηγούμενη σάρωση») βλάπτει σε φορητό αισθητήρα σε ΟΛΕΣ τις 8 ακολουθίες**: χωρίς αυτό ο KISS
   βελτιώνεται παντού, συχνά ~2× (church_02 24.0 → 11.5 cm/s). Επιβεβαίωση του #012 σε 3 datasets / 3 αισθητήρες. **Η δίκαιη βάση
   σύγκρισης είναι ο KISS χωρίς deskew, όχι ο KISS.**
2. **Έναντι αυτής, η εικόνα κερδίζει στη μετατόπιση** (RPE 6/8· church 5.5 έναντι 11.5, 3.6 έναντι 7.3 cm/s) και στο μήκος (5/8),
   **όχι στη στροφή** (χωρίς deskew καλύτερο σε 4/8: 01_short, quad, math, church_03). Υπόθεση: το σφάλμα στροφής της εικόνας ανά σάρωση
   (0.6–0.8° σε κίνηση 1–1.5°) περνά ως θόρυβος μέσα από το deskew και την αρχική θέση. Επόμενο: εξομάλυνση / στάθμιση της στροφής.
3. **Η σκάλα λύνεται μόνο με το indoor_detail** (ATE 0.46 m έναντι 1.7–3.6)· η εικόνα δεν έχει τρέξει ποτέ με αυτό σε αυτήν την ακολουθία.
4. **Μεθοδολογία:** κάθε σύγκριση βραχιόνων με διαφορετικό deskew χρειάζεται μετατόπιση χρόνου ανά βραχίονα (ή ίδιο σύστημα χρόνου θέσης)·
   αλλιώς το RPE στροφής είναι μεροληπτικό. Τα RPE στροφής των #041–#044 (μετατόπιση 0) αδικούν την εικόνα έναντι του KISS.

---

### 2026-09-23 — #044 Oxford Spires christ-church-02 / -03, ΟΛΟΚΛΗΡΕΣ οι εγγραφές, ρύθμιση του paper

**git commit:** κλάδος `ouster_2021` · runs `/media/photogrammetry/A26C3DDF6C3DAF431/data/runs/oxford_spires_full/` · συνολικός
πίνακας όλων των ακολουθιών: `docs/results_all.md` / `.csv` (`scripts/results_table.py`)

#### Στόχος
Οι πλήρεις εγγραφές (church_02: 592 s σε 2 bags, 5928 σαρώσεις· church_03: 312 s, 3123) με το ίδιο πρωτόκολλο με το Newer College
(#041, #043), ώστε όλος ο πίνακας να είναι συγκρίσιμος. **Όχι** το `indoor_detail.yaml` ούτε το κομμένο bag των #027–#039.

#### Μέθοδος
KISS / εικόνα + SIFT / εικόνα + SURF × 2 σπόροι, config προεπιλογής του KISS-SLAM, 4 νήματα ICP· `run_ncd.py --topic=/hesai/pandar
--intensity-scale=1.0` (Hesai: intensity 0–255, απόλυτοι χρόνοι)· `evaluate_ncd.py --frame=spires`, χωρίς μετατόπιση χρόνου
(όλοι οι βραχίονες ίδια).

#### Αποτέλεσμα
| Ακολουθία (μήκος GT) | Βραχίονας | ATE [m] | RPE 1 s [cm] | RPE 1 s [°] | μήκος έναντι GT | z RMSE [m] | KITTI [%] |
|---|---|---|---|---|---|---|---|
| church_02 (642 m) | KISS | 0.797 | 24.23 | 2.563 | +92 % | 0.438 | 1.15 |
| | SIFT | 0.354 ± 0.197 | 7.32 ± 0.03 | 2.225 | +13.5 % | 0.310 ± 0.205 | 0.92 ± 0.08 |
| | SURF | **0.259** ± 0.025 | **7.05** ± 0.04 | **2.199** | **+11.2 %** | **0.219** ± 0.030 | **0.88** |
| church_03 (266 m) | KISS | 0.169 | 14.75 | 2.802 | +79 % | 0.106 | 0.65 |
| | SIFT | 0.108 ± 0.002 | 5.56 ± 0.02 | 2.655 | +11.7 % | 0.039 | 0.59 |
| | SURF | 0.108 ± 0.000 | **5.40** ± 0.00 | **2.641** | **+10.7 %** | 0.040 | 0.59 |

Καμία ένωση (χάρτες 100 m). Το ATE του SIFT στο church_02 διαφέρει 0.2 m ανάμεσα στους 2 σπόρους (#037: ATE ενός run δεν είναι
μέτρηση). Ένα run (church_03, KISS, σπόρος 0) τελείωσε όλη τη δουλειά (αρχεία γραμμένα) αλλά η διεργασία κόλλησε στην έξοδο:
9 νήματα της δεξαμενής νημάτων σε futex, zombie ~2 h· σκοτώθηκε, η τροχιά του ταυτίζεται με του σπόρου 1 ως 1.6·10⁻¹³.

#### Συμπέρασμα ⏳
Στη ρύθμιση του paper, στις πλήρεις εγγραφές, η εικόνα μειώνει το RPE μετατόπισης **~2.7×** (24.2 → 7.1 cm, 14.8 → 5.4 cm) και την
περίσσεια μήκους από 79–92 % σε 11–14 %· το SURF είναι πάλι λίγο καλύτερο από το SIFT. Και στις 8 ακολουθίες των τριών datasets το
RPE μετατόπισης της εικόνας είναι μικρότερο του KISS (`docs/results_all.md`). Με `indoor_detail.yaml` (#031) ο KISS είχε ATE 0.319 στο
κομμένο church_02· εδώ, με voxel 1.0 m, 0.797 στο πλήρες.

---

### 2026-09-23 — #043 Newer College 2021 (Ouster OS0-128): η εικόνα βελτιώνει 4 από 5 ακολουθίες· η σκάλα μένει άλυτη

**git commit:** κλάδος `ouster_2021`, `2c26875` (+ διόρθωση του evaluator για το GT του ορυχείου) · runs
`/media/photogrammetry/A26C3DDF6C3DAF431/data/runs/newer_college_2021/<ακολουθία>/`

#### Στόχος
Οι 5 ακολουθίες του 2021 του paper του KISS-SLAM, τρεις βραχίονες (KISS, εικόνα + SIFT, εικόνα + SURF), 2 σπόροι ο καθένας
(απόφαση Μ.Τ. 23/9).

#### Μέθοδος
Οι τρεις διορθώσεις του κλάδου `ouster_2021`: (1) απόλυτοι χρόνοι σημείων στο `read_point_cloud_raw` (ακέραιο `t` = ns, και
σχετικοί χρόνοι + χρονοσφραγίδα μηνύματος)· (2) `image_deskew.intensity_scale` = 255/1024· (3) `evaluate_ncd.py` με GT TUM /
csv, `--frame=ncd2021` (T_base_os-sensor: 0.001, 0, 0.091 m), παρεμβολή GT στους χρόνους των σαρώσεων, KITTI από το
`sequence_error` του kiss_icp. Έλεγχος στο stairs (σαρώσεις 100–299, κίνηση 1.46° / 38 mm): SIFT 0.77° / 11 mm, SURF 0.69° /
10 mm. Οι χρονοσφραγίδες των σαρώσεων ταυτίζονται με του GT (0.0 ms). Cloister και math_easy (2 bags η καθεμία) διαβάζονται ως
μία ακολουθία (2788 / 2160 σαρώσεις, κενό ≤ 0.134 s). Config προεπιλογής του KISS-SLAM, 4 νήματα ICP, 12 runs παράλληλα.
Το GT του ορυχείου («tum_format») έχει sec και nsec σε χωριστές στήλες (9 στήλες)· ο evaluator το αναγνωρίζει πλέον.

#### Αποτέλεσμα
Μέσος ± σ (2 σπόροι· ο KISS είναι ντετερμινιστικός). **Καμία ένωση** σε κανένα run: οι ακολουθίες (57–429 m) είναι μικρές για
χάρτες των 100 m.

| Ακολουθία (μήκος GT) | Βραχίονας | ATE [m] | RPE 1 s [cm] | RPE 1 s [°] | Μήκος [m] | z RMSE [m] | KITTI [%] |
|---|---|---|---|---|---|---|---|
| quad_easy (247 m) | KISS | 0.119 | 10.56 | **1.416** | 291.8 | 0.089 | **0.42** |
| | SIFT | 0.096 ± 0.000 | 7.70 ± 0.06 | 1.890 | 274.6 | 0.070 | 0.49 |
| | SURF | **0.094** ± 0.001 | **6.88** ± 0.03 | 1.893 | **268.3** | **0.069** | 0.47 |
| stairs (57 m) | KISS | 3.582 | 100.66 | 27.224 | 238.6 | 2.332 | — |
| | SIFT | **1.747** ± 0.106 | 23.64 ± 1.08 | 5.425 | 100.6 | **1.606** | — |
| | SURF | 2.151 ± 0.312 | **20.72** ± 0.33 | **4.461** | **94.5** | 1.889 | — |
| cloister (429 m) | KISS | 0.396 | 19.57 | 2.641 | 580.3 | 0.306 | 0.90 |
| | SIFT | 0.216 ± 0.007 | 8.55 ± 0.01 | 2.082 | 456.5 | 0.119 | 0.78 |
| | SURF | **0.205** ± 0.019 | **8.21** ± 0.01 | **2.055** | **453.0** | **0.108** | **0.76** |
| math_easy (264 m) | KISS | 0.150 | 10.34 | 1.181 | 310.4 | 0.051 | 0.63 |
| | SIFT | 0.101 ± 0.001 | 6.92 ± 0.07 | 0.924 | 278.6 | 0.034 | 0.51 |
| | SURF | **0.090** ± 0.000 | **6.31** ± 0.00 | **0.869** | **272.8** | **0.028** | **0.49** |
| underground_easy (162 m) | KISS | 0.170 | 10.76 | 2.453 | 192.0 | 0.055 | 0.70 |
| | SIFT | 0.132 ± 0.000 | 6.12 ± 0.03 | 2.093 | 169.5 | 0.028 | 0.61 |
| | SURF | **0.130** ± 0.002 | **6.07** ± 0.04 | **2.052** | **168.9** | **0.027** | **0.60** |

(KITTI: τμήματα 100–800 m, άρα δεν ορίζεται στο stairs. ms/σάρωση 48–498 με 12 runs μαζί — όχι μέτρηση ταχύτητας.)

- **4 ακολουθίες (quad, cloister, math, ορυχείο):** η εικόνα μειώνει RPE μετατόπισης 29–58 %, περίσσεια μήκους (π.χ. cloister
  +35 % → +6 %), z RMSE 22–65 %, ATE 13–48 %.
- **Εξαίρεση: RPE στροφής στο quad_easy χειρότερο με την εικόνα** (1.89° έναντι 1.42°, και KITTI 0.47–0.49 έναντι 0.42 %),
  ενώ στις άλλες τρεις βελτιώνεται. Αιτία άγνωστη.
- **stairs:** ο KISS καταρρέει (ATE 3.6 m, RPE 1 m ανά δευτερόλεπτο, μήκος 239 έναντι 57 m)· η εικόνα κόβει RPE ~4–5× και μήκος
  ~2.5× αλλά η τροχιά μένει κακή (ATE 1.7–2.2 m, μήκος +65–76 %).
- **SURF έναντι SIFT:** RPE μετατόπισης καλύτερο σε 4/5 πέρα από τη σ (quad 6.88 / 7.70, stairs 20.7 / 23.6, cloister 8.21 / 8.55,
  math 6.31 / 6.92)· στο ορυχείο ίσο (6.07 / 6.12). Αποτυχίες εικόνας 1–2 έναντι 1–13.

#### Συμπέρασμα ⏳
Με τις τρεις διορθώσεις η μέθοδος τρέχει σε δεύτερο αισθητήρα Ouster (OS0-128, 128 ακτίνες) χωρίς άλλη ρύθμιση και βελτιώνει τον
KISS-SLAM σε 4 από 5 ακολουθίες σχεδόν σε όλα τα μέτρα· το SURF είναι σταθερά λίγο καλύτερο από το SIFT (όπως στο 01_short, #041).
Ανοιχτά: (α) γιατί χειροτερεύει η στροφή στο quad_easy· (β) η σκάλα του 2021 — εδώ πράγματι «δύσκολη» γεωμετρία, σε αντίθεση με
το church_02 (#022) — υποψήφια για το αρχικό concept (intensity μόνο όπου εκφυλίζεται ο ICP). Με 2 σπόρους το ATE του stairs
(σ 0.1–0.3 m) δεν ξεχωρίζει SIFT από SURF.

---

### 2026-09-23 — #042 Κλάδος `fast_test`: τρεις επιταχύνσεις του βήματος της εικόνας (στόχος τα 10 Hz)

**git commit:** κλάδος `fast_test` — `0bb60df` (1), `e42b57b` (2), `0004cf5` (3) · μηχάνημα `photogrammetry`

#### Στόχος
Ο αισθητήρας δίνει 10 σαρώσεις/s, άρα ο προϋπολογισμός είναι 100 ms ανά σάρωση για όλα (εικόνα + ICP). Να μειωθεί ο χρόνος
του βήματος της εικόνας **χωρίς να αλλάξει η μέθοδος**.

#### Μέθοδος
Προφίλ (cProfile) σε 30 σαρώσεις του 01_short (#041), SIFT: ανίχνευση 66 ms (42 %), RANSAC 35 ms (22 %: πάντα 400 υποθέσεις,
μία-μία σε Python), fit χρόνου 33 ms (21 %: 6 λύσεις `least_squares` με αριθμητικές παραγώγους), πανόραμα 12 ms,
αναζήτηση + ταίριασμα 6 ms. Τρεις αλλαγές, καθεμία με διακόπτη που επαναφέρει την παλιά συμπεριφορά:

1. **RANSAC με πρόωρη διακοπή** (`RANSAC_CONF = 0.999`, `RANSAC_BATCH = 32`): σταματά μετά από
   log(1 − 0.999) / log(1 − w³) υποθέσεις (w = ποσοστό inliers της καλύτερης ως τώρα· ~50 για w = 0.5), ποτέ πάνω από 400·
   οι υποθέσεις υπολογίζονται και κρίνονται 32 τη φορά σε numpy (`kabsch_batch`, `inliers_batch`). `RANSAC_CONF = None` =
   το παλιό loop με τις ίδιες τυχαίες επιλογές.
2. **Αναλυτική Ιακωβιανή** του fit χρόνου (`residual_jac`, `FIT_JAC = "analytic"`): παράγωγοι του
   Exp(φ(t))·p + s(t) − Exp(φ(t′))·q − s(t′) μέσω της αριστερής Ιακωβιανής του SO(3), για τα μοντέλα cv / car / ca.
   `FIT_JAC = "2-point"` = οι αριθμητικές παράγωγοι του scipy.
3. **Εικόνα σε χωριστή διεργασία** (`image_deskew.parallel`, προεπιλογή off): ο εκτιμητής ζει σε μία διεργασία-εργάτη
   (`spawn`)· το pipeline διαβάζει τη σάρωση k+1 και τη δίνει στον εργάτη πριν από τον ICP της k, οπότε τα δύο
   επικαλύπτονται. Ένας εργάτης, σαρώσεις με τη σειρά → ίδιες κινήσεις με τη σειριακή εκτέλεση. Νέο πεδίο
   `image_deskew.seed` ώστε ο σπόρος του RANSAC να φτάνει στον εργάτη. Διεργασία και όχι νήμα: ο ICP και τα κομμάτια
   Python του εκτιμητή δεν θα έτρεχαν ταυτόχρονα λόγω του GIL.

#### Αποτέλεσμα
Όλες οι μετρήσεις χρόνου με **10 άλλα runs στο ίδιο μηχάνημα** (φορτίο ~77 σε 48 πυρήνες): οι λόγοι ισχύουν, τα απόλυτα ms όχι.

| Αλλαγή | Έλεγχος ορθότητας | Ακρίβεια (01_short, σαρώσεις 4001–4100, SIFT) | Χρόνος |
|---|---|---|---|
| 1 | `kabsch_batch` = `kabsch` ως 3·10⁻¹⁵· `inliers_batch` = `inliers` (metric και bearing) | στροφή διάμεσος 0.722 → 0.728°, μετατόπιση 62.6 → 62.6 mm | 81 → 53 ms/ζεύγος |
| 2 | αναλυτική έναντι πεπερασμένων διαφορών 9·10⁻¹⁰ – 1.4·10⁻⁹ (cv/car/ca, και για γωνίες ~0) | κινήσεις ίδιες ως 1.5·10⁻⁹ | 53 → 45 ms/ζεύγος |
| 3 | 300 σαρώσεις: τροχιά παράλληλη = σειριακή ως 10⁻¹⁴· άλλος σπόρος → διαφορά 0.22 m (ο σπόρος φτάνει στον εργάτη) | ίδια τροχιά | 55 → 37 s (−32 %) |

Η αλλαγή 1 αλλάζει τις τυχαίες επιλογές του RANSAC, άρα ένα run διαφέρει από τον παλιό κώδικα μέσα στη διασπορά των σπόρων
(#037)· η ακρίβεια ανά ζεύγος δεν άλλαξε. Το `tests/test_image_deskew.py` (γ) συγκρίνει με npz του παλιού κώδικα και γι' αυτό
ορίζει `RANSAC_CONF = None`.

**Μέτρηση σε ήσυχο μηχάνημα (προσθήκη 23/9):** όλο το 01_short, `fast_test` (1 + 2 ενεργά), `--parallel`, **SURF κατώφλι 400**,
διαγνωστικά ICP off (KD-tree ανά σάρωση, δεν αλλάζει την τροχιά), ICP με όλους τους πυρήνες· 1 run, config
`data/runs/newer_college_01_short_speed/surf400_speed.yaml` στον δίσκο δεδομένων.

| | τιμή |
|---|---|
| Σαρώσεις / χρόνος βρόχου | 15 301 σε 1109 s → **13.8 σαρώσεις/s**, 72 ms ανά σάρωση μαζί με την ανάγνωση (1530 s δεδομένων → 1.37× ταχύτερα από τον αισθητήρα) |
| `process_scan` (pipeline) | 71 ms μέσος, 14 Hz |
| Παράθυρα 30 s κάτω από 10 σαρώσεις/s | 3 από 108 (χειρότερο 8.9/s)· ταχύτερο 17.7/s |
| CPU | 13 221 s user σε 1117 s → **~12 πυρήνες** κατά μέσο όρο (TBB του ICP + OpenCV) |
| Ακρίβεια (1 run) | RPE 1 s 8.23 cm / 0.804° · μήκος 1896 m · ATE 0.255 · KITTI 0.36 % · 38 αποτυχίες εικόνας |

Για σύγκριση (#041, κώδικας πριν από το `fast_test`): SURF 100 7.93 ± 0.01 cm / 0.777°, μήκος 1864· SIFT 8.42 ± 0.02 / 0.817, 1909.
Το SURF 400 είναι ανάμεσα — χειρότερο από το SURF 100 κατά ~4 % στο RPE· η διαφορά μπλέκει κατώφλι και τις αλλαγές του
`fast_test` (1 run, άλλες τυχαίες επιλογές RANSAC).

#### Συμπέρασμα ⏳
**Στο μηχάνημα `photogrammetry` η μέθοδος τρέχει ταχύτερα από τον αισθητήρα (13.8 έναντι 10 σαρώσεων/s)**, με μικρές πτώσεις
κάτω από 10/s που απορροφά ένα μικρό buffer. Προϋπόθεση ~12 πυρήνες: σε laptop δεν έχει κριθεί. Οι αλλαγές 1–2 κερδίζουν ~36 ms ανά σάρωση χωρίς αλλαγή της μεθόδου· η 3 κάνει το κόστος ανά σάρωση ≈ max(εικόνα, ICP) αντί
για το άθροισμα. **Αν η μέθοδος φτάνει τα 10 Hz δεν έχει κριθεί**: χρειάζεται μέτρηση σε ήσυχο μηχάνημα (το STATUS είχε
~60 ms/σάρωση για την εικόνα στο church_02). Επόμενες, που **αλλάζουν** τη μέθοδο και θέλουν έλεγχο με GT: SURF με κατώφλι
400 (#041), μικρότερη κατακόρυφη μεγέθυνση του πανοράματος (`UP` 8 → 4).

---

### 2026-09-23 — #041 Newer College 2020 (01_short, Ouster OS1-64): η εικόνα μειώνει το RPE 58–60 %· το SURF λίγο καλύτερο από το SIFT

**git commit:** κλάδος `fast_test`, `3c54ce0` · δεδομένα `/media/photogrammetry/A26C3DDF6C3DAF431/data/newer_college/`, runs `…/data/runs/newer_college_01_short/`

#### Στόχος
Η μέθοδος σε δεύτερο dataset και δεύτερο αισθητήρα, στις ακολουθίες του paper του KISS-SLAM (Πίνακας V: 2020 01_short,
02_long· 2021 cloister, math_easy, quad_easy, stairs, underground_easy). Σύγκριση τριών βραχιόνων έναντι GT:
(1) εικόνα + SIFT, (2) εικόνα + **SURF** (ζήτημα Μ.Τ.), (3) σκέτος KISS-SLAM.

#### Μέθοδος
- **Λήψη:** φάκελος Google Drive του dataset (φόρμα στη σελίδα του). 01_short: μόνο LiDAR (10 zip, 18.9 GB → 15 302 `.pcd`,
  28 GB)· GT για όλες τις ακολουθίες. 02_long υπάρχει μόνο ως πλήρη rosbags (170 GB) — παραλείφθηκε. Τα bags του 2021
  κατεβαίνουν από τον browser (το δημόσιο quota του Drive τα μπλόκαρε). `manifest.tsv` / `download.sh` στον φάκελο.
- **Reader** `kiss_slam/tools/ncd_pcd.py`: ο reader του kiss_icp κρατά μόνο x, y, z και φτιάχνει δικούς του χρόνους. Τα `.pcd`
  έχουν `intensity, t, reflectivity, ring` (64 × 1024). Επιστρέφει (xyz, χρόνος, intensity, ring)· σημεία χωρίς επιστροφή
  (range 0, 12–56 % ανά σάρωση) απορρίπτονται· GT ανά σάρωση (ακριβώς ίδιες χρονοσφραγίδες) στο σύστημα του LiDAR με το
  `T_CL` του kiss_icp.
- **Δύο προσαρμογές στο Ouster:**
  - **Χρόνος:** το `t` μετρά ns από την αρχή κάθε σάρωσης. Ο εκτιμητής βάζει δύο διαδοχικές σαρώσεις σε έναν άξονα χρόνου,
    άρα θέλει απόλυτους χρόνους: χρονοσφραγίδα σάρωσης (όνομα αρχείου) + `t`. Με το `t` μόνο, κάθε κίνηση υπολογιζόταν
    ~100° / 17 m ανά σάρωση και η τροχιά απέκλινε σε 30 s. Ο KISS κανονικοποιεί μόνος του τους χρόνους (έλεγχος: ίδιο deskew
    με απόλυτους, σχετικούς ή [0,1]).
  - **Κλίμακα intensity:** Ouster 0 – ~1100 (διάμεσος 150–450), ενώ το πανόραμα ψαλιδίζει στο 255 (φτιάχτηκε για τα 0–255 του
    Hesai). Ένας σταθερός συντελεστής **255/1024** για όλες τις σαρώσεις και τους δύο ανιχνευτές, χωρίς κανονικοποίηση ανά σάρωση.
- **SURF:** `image_deskew.detector: sift | surf` (+ `surf_hessian_threshold`, `surf_upright`), CLI `--image-detector`,
  precompute `--detector=surf` (κατάληξη `_surf`). Το SURF είναι patented: λείπει από όλα τα wheels του pip, ακόμη και το
  contrib. Χτίστηκε OpenCV 5.0.0.93 contrib με `OPENCV_ENABLE_NONFREE=ON` (οδηγός εκτέλεσης).
- **Runs:** config προεπιλογής του KISS-SLAM (η ρύθμιση του paper: voxel 1.0 m, χάρτες 100 m)· 4 νήματα ICP ανά run.
  KISS ×2, SIFT ×4 σπόροι, SURF ×4 σπόροι, παράλληλα. `scripts/run_ncd.py`, αξιολόγηση `scripts/evaluate_ncd.py`
  (ATE, RPE 1 s, μήκος διαδρομής, z RMSE στο σύστημα του GT όπου z = πάνω, σφάλμα KITTI όπως στο paper).

#### Αποτέλεσμα
**Πλήρη runs, όλο το 01_short** (15 301 σαρώσεις, GT 1609.5 m· config προεπιλογής, κατώφλι ενώσεων 5· μέσος ± σ, 4 σπόροι):

| Βραχίονας | ATE [m] | RPE 1 s [cm] | RPE 1 s [°] | Μήκος διαδρομής [m] | z RMSE [m] | KITTI [%] | αποτυχίες εικόνας | ενώσεις |
|---|---|---|---|---|---|---|---|---|
| KISS-SLAM (ντετερμινιστικό) | 0.391 | 20.01 | 1.430 | 2664.0 (+65 %) | 0.270 | 0.56 | — | 6 |
| εικόνα + SIFT | 0.267 ± 0.004 | 8.42 ± 0.02 | 0.817 ± 0.002 | 1909.4 ± 2.1 (+19 %) | 0.147 ± 0.004 | 0.36 | 56 ± 1 | 6 |
| εικόνα + SURF | 0.268 ± 0.011 | **7.93 ± 0.01** | **0.777 ± 0.002** | **1864.2 ± 1.6 (+16 %)** | 0.144 ± 0.021 | 0.35 | 6 ± 1 | 6 |

- **Εικόνα έναντι KISS:** RPE μετατόπισης −58 % (SIFT) / −60 % (SURF), στροφής −43 / −46 %· περίσσεια μήκους 65 → 19 / 16 %·
  z RMSE και ATE περίπου −45 % / −32 %· KITTI 0.56 → 0.36 / 0.35 %. Διαφορές πολλαπλάσιες της σ.
- **SURF έναντι SIFT:** RPE μετατόπισης 7.93 έναντι 8.42 cm (t ≈ 44), στροφής 0.777 έναντι 0.817° (t ≈ 28), μήκος 1864 έναντι
  1909 m (t ≈ 34): **σταθερά καλύτερο**, κατά ~5 %. ATE ίδιο (0.268 έναντι 0.267, σ 0.004–0.011)· z RMSE ίδιο. Αποτυχίες
  εικόνας (σάρωση χωρίς κίνηση → ταυτοτική) 6 έναντι 56.
- Η περίσσεια μήκους (+16–19 %) παραμένει μεγάλη — όπως το +5 % του church_02 (STATUS §2.1), αιτία άγνωστη.
- Χρόνος ανά σάρωση (279 / 352 / 394 ms) με 10 runs στο μηχάνημα: δεν είναι μέτρηση ταχύτητας.

**Μερικά αποτελέσματα πριν από τα runs:**
**Κίνηση ανά σάρωση έναντι GT** (σαρώσεις 1000–1200· κίνηση 0.97° / 110 mm):

| | inliers | σφάλμα στροφής | σφάλμα μετατόπισης | κλίμακα |
|---|---|---|---|---|
| SIFT | 101 | 0.59° (90ό εκατ. 1.19°) | 32 mm | 0.912 |
| SURF (κατώφλι 100) | 246 | 0.56° (90ό εκατ. 1.06°) | 29 mm | 0.912 |

**Χρόνος ανά σάρωση του βήματος εικόνας** (σαρώσεις 3000–3020, φορτωμένο μηχάνημα):

| | σημεία | inliers | πανόραμα | ανίχνευση | ταίριασμα | αναζήτηση + RANSAC + fit | σύνολο |
|---|---|---|---|---|---|---|---|
| SIFT | 491 | 42 | 16 | 87 | 1 | 43 | 147 ms |
| SURF 100 | 1795 | 89 | 13 | 68 | 6 | 57 | 145 ms |
| SURF 400 | 713 | 54 | 13 | 44 | 1 | 46 | 105 ms |
| SURF 800 | 339 | 29 | 14 | 39 | 1 | 42 | 96 ms |

Το SURF ανιχνεύει ταχύτερα, αλλά με το κατώφλι 100 του OpenCV βρίσκει 3.7× περισσότερα σημεία και το κέρδος χάνεται στο
ταίριασμα και στα κομμάτια Python ανά αντιστοίχιση.

**Ο σκέτος KISS-SLAM στην αρχή:** ακολουθεί το GT όσο ο αισθητήρας είναι ακίνητος (0–17 s) και μετά «τρέμει»: σαρώσεις
150–300, μήκος 28.4 m έναντι 2.1 m GT, βήμα έως 0.94 m. Ίδιο με τον reader του kiss_icp (28.9 m) → όχι θέμα του reader.

**Πλήρη runs (15 301 σαρώσεις): σε εξέλιξη** — τα αποτελέσματα προστίθενται εδώ. Σκέτος KISS-SLAM (2 runs, **ταυτόσημα**:
χωρίς την εικόνα ο KISS είναι ντετερμινιστικός εδώ): ATE 0.391 m · RPE 1 s 20.0 cm / 1.43° · **μήκος διαδρομής 2664 m έναντι
1610 m GT (+65 %)** · z RMSE 0.270 m · KITTI 0.56 % · 6 ενώσεις.

**Είναι λάθος οι ενώσεις με λίγα inliers;** (ερώτημα Μ.Τ.· έλεγχος του περιορισμού κάθε ένωσης έναντι του GT, όπως το
`check_closure_constraints.py`). Όχι: και οι 6 (9–27 inliers) απέχουν 0.25–2.84 m / 0.6–2.0° από το GT, όσο και οι ακμές
odometry ανάμεσα σε γειτονικούς χάρτες (0.36–2.99 m)· οι δύο με 9 inliers είναι οι **ακριβέστερες** (0.25, 0.34 m). Με κατώφλι
100 inliers (`configs/newer_college_lc100.yaml`) δεν περνά καμία — το run γίνεται odometry χωρίς ενώσεις. Δεν ξανατρέχτηκε
(απόφαση Μ.Τ. 23/9).

#### Συμπέρασμα ⏳
**Η μέθοδος δουλεύει και σε δεύτερο dataset / αισθητήρα (Ouster OS1-64), στη ρύθμιση του paper του KISS-SLAM: το σφάλμα ανά
δευτερόλεπτο πέφτει στο ~40 %, η περίσσεια μήκους από 65 % σε 16–19 %, το ATE κατά ~32 %.** Το **SURF** δίνει μικρό αλλά βέβαιο
κέρδος έναντι του SIFT στα σταθερά μέτρα (RPE, μήκος, ~5 %) και 9× λιγότερες αποτυχίες· στο ATE δεν ξεχωρίζουν. Επόμενα:
SURF με κατώφλι 400 (ταχύτερο, #041 χρόνοι) με GT· το ίδιο στις ακολουθίες του 2021.

Η μέθοδος μεταφέρεται στο Ouster με δύο προσαρμογές του reader και μετρά την κίνηση ανά σάρωση περίπου όσο καλά όσο στο
church_02 (~0.5°, #038)· η κλίμακα 0.91 δείχνει ότι η μεροληψία κοντινού πεδίου του #039 υπάρχει και εδώ. Ο συντελεστής
intensity 255/1024 είναι επιλογή **προς έγκριση Λ.Γ.** Η σύγκριση των βραχιόνων κρίνεται με RPE και μήκος διαδρομής
(4 σπόροι), όχι με το ATE ενός run.

---

### 2026-09-23 — #040 Επιθεώρηση κώδικα: τρία σφάλματα, ένα ανοιχτό ερώτημα

**git commit:** `66a0db4` (main, πρώτο commit στο GitHub) · μηχάνημα `photogrammetry`

#### Στόχος
Ανάγνωση του κώδικα της μεθόδου (`intensity_deskew.py`, `slam.py`) και σύγκριση με το upstream.

#### Αποτέλεσμα
**Σωστά:** τα αντίγραφα του `register_frame` ταυτίζονται με το upstream KISS-ICP (και εκείνο δίνει `kernel=sigma`)· τα
defaults του `image_deskew` αναπαράγουν τη μέθοδο του STATUS· η αντιστοίχιση intensity ↔ σημείου ακολουθεί τις αυστηρές
ανισότητες του KISS.

**Τρία σφάλματα, διορθωμένα:**
1. **Intensity των τοπικών χαρτών σε λάθος σύστημα αναφοράς** (`local_map_graph.finalize_local_map`, `slam._accumulate_intensity`).
   Αποθηκευόταν στο σύστημα του τοπικού χάρτη και αναζητούνταν στο παγκόσμιο → για κάθε χάρτη μετά τον πρώτο, intensity 0
   παντού, οπότε το ColoredICP των ενώσεων έτρεχε χωρίς φωτομετρικό όρο. Έλεγχος: σημεία που βρίσκουν intensity 0 % → 100 %.
   Αφορά μόνο το κλειστό «refine/replace»· **κάθε συμπέρασμα για ColoredICP στις ενώσεις δεν δοκίμασε ποτέ intensity.**
2. **`--motion-file` με `--jump`** (`slam._image_motion`): το αρχείο διαβαζόταν με τον μετρητή του run από το 0 → με
   `--jump 500` η σάρωση 500 έπαιρνε την κίνηση της 0. Τώρα `first_scan_index` (το ορίζει το pipeline) + έλεγχος ότι το αρχείο
   καλύπτει το run (αλλιώς σφάλμα, όχι σιωπηλά ταυτοτική). Έλεγχος με συνθετικό npz: jump 5 → γραμμές 5, 6, (7 αποτυχία), 8, 9.
3. **Διόρθωση κοντινού πεδίου «auto»** (`match_motion.last_ratio`): δεν μηδενιζόταν πριν από τις πρόωρες επιστροφές, οπότε
   σε αποτυχημένη σάρωση ο διάμεσος έπαιρνε ξανά τον λόγο της προηγούμενης. Μόνο με `trans_min_range` (ανενεργό από προεπιλογή).

**Ανοιχτό ερώτημα (ιδέα, όχι σφάλμα):** το `fit_time` μετρά υπόλοιπα σε μέτρα με κατώφλι 0.10 m. Ένα pixel (0.35°) είναι
~0.006·r m, άρα οι μακρινές αντιστοιχίσεις κόβονται ή βαραίνουν λιγότερο και η κίνηση στηρίζεται στις κοντινές — ακριβώς
αυτές που το #039 βρήκε κοντές (~74 % στα < 5 m). Το RANSAC έχει ήδη έλεγχο διεύθυνσης (`INLIER_TEST="bearing"`), το `fit_time`
όχι. Φθηνό πείραμα: υπόλοιπα διαιρεμένα με την απόσταση στο `fit_time`, κρίση με RPE και μήκος διαδρομής.

**Μικρότερα:** οι ρυθμίσεις κοντινού πεδίου γράφονται σε καθολικές μεταβλητές του `intensity_deskew` και μένουν ανάμεσα σε δύο
`KissSLAM` στην ίδια διεργασία· `TRANS_FACTOR` / διόρθωση μετατόπισης δεν περνούν στην καμπύλη (`last_params`) που χρησιμοποιεί
το `deskew_curve`· το `lookup` μπορεί να πάρει pixel έως 2 στήλες μακριά χωρίς διόρθωση· περίοδος σταθερή 0.1 s.

#### Συμπέρασμα ⏳
Η μέθοδος στην προεπιλογή της δεν επηρεάζεται. Το σφάλμα 1 ακυρώνει ό,τι είχε ειπωθεί για ColoredICP στις ενώσεις· το 2 αφορά
μόνο runs με `--jump` + `--motion-file`, που πρέπει να ξαναγίνουν.

---

### 2026-09-20 — #039 RoMa v2 αντί για SIFT: δοκιμή σε 60 ζεύγη — δεν αντικαθιστά, αλλά δείχνει κάτι για την κλίμακα

**git commit:** αυτή η εγγραφή · scratchpad `export_pairs.py`, `roma_match.py` (περιβάλλον `roma-poc`), `roma_eval.py` ·
δεδομένα `pairs60/` (60 ζεύγη απλωμένα στο church_02)

#### Στόχος
Πρόταση Λ.Γ.: πυκνή συνταύτιση **RoMa / RoMa v2** αντί για SIFT. Μετά το #038 (κορεσμός στις ~80–160 αντιστοιχίσεις)
το ερώτημα είναι συγκεκριμένο: **βελτιώνει την ουρά του σφάλματος και την κλίμακα;**

#### Μέθοδος
Τρία στάδια, ώστε να μην μπει torch στο `kissslam` (numpy/OpenBLAS): (1) εξαγωγή πανοραμάτων + P/T/valid ανά pixel για
60 ζεύγη απλωμένα σε όλη τη διαδρομή· (2) RoMa v2 στο περιβάλλον `roma-poc` (MPS), 5000 δείγματα ανά ζεύγος·
(3) πίσω στο `kissslam`: ίδιο `lookup` με subpixel, ίδιο φίλτρο δαπέδου, ίδιο RANSAC + `fit_time("car")`, σύγκριση με GT.
Το SIFT τρέχει στα **ίδια** ζεύγη και τις ίδιες εικόνες.

#### Αποτέλεσμα
**Ταχύτητα σε αυτό το Mac (MPS, εικόνα 512×1024):** turbo (320²) **224 ms**, fast (512²) 599, base (640²) 1122,
precise (800², αμφίδρομο) **66 000 ms**. Η δημοσίευση δίνει 30.9 ζεύγη/s σε H200 με δικό τους CUDA kernel (RoMa v1: 18.5).
Πραγματικά real-time εναλλακτικές: **EDM** (ICCV 2025) 16.7 ms, **EfficientLoFTR** 39 ms, LoFTR 71.8 ms σε RTX 3090 @640×480.

**Ακρίβεια, 60 ζεύγη, RoMa v2 turbo** (κίνηση GT διάμεσος 1.84°):

| παραλλαγή | ζεύγη πριν | inliers | στροφή διάμ. | 90ό | max | θέση | κλίμακα |
|---|---|---|---|---|---|---|---|
| **SIFT (σημερινό)** | 170 | 76 | **0.53°** | **1.15°** | **1.64°** | **17 mm** | 0.970 |
| RoMa όλες | 4496 | 2951 | 0.51° | 0.93° | 3.25° | 21 mm | 0.856 |
| RoMa μόνο εμπιστοσύνη (200) | 192 | 152 | 1.29° | — | 8.04° | 26 mm | 0.897 |
| RoMa > 5 m | 2126 | 1124 | 0.65° | 1.41° | 5.65° | 23 mm | **0.998** |
| RoMa απλωμένες (πλέγμα 16×32) | 491 | 312 | 0.52° | 0.93° | 3.12° | 20 mm | 0.849 |
| RoMa > 5 m + απλωμένες | 255 | 138 | 0.65° | 1.48° | 7.87° | 24 mm | 1.003 |

1. **Το RoMa μεταφέρεται** σε γκρίζα πανοράματα intensity: 2951 από 4496 ζεύγη περνούν το RANSAC (66 %). Δεν ήταν δεδομένο.
2. **Δεν βελτιώνει τον διάμεσο** (0.51° έναντι 0.53°) — **ακριβώς όπως προέβλεψε το #038**: 39× περισσότερες
   αντιστοιχίσεις πάνω από το σημείο κορεσμού δεν αποδίδουν.
3. **Χειροτερεύει την ουρά:** max 3.25° έναντι 1.64°, θέση 21 έναντι 17 mm.
4. **Η εμπιστοσύνη του δικτύου είναι κακός επιλογέας:** οι 200 με τη μεγαλύτερη εμπιστοσύνη δίνουν 1.29° και max 8.04°.
5. **Γιατί:** οι πυκνές αντιστοιχίσεις μαζεύονται κοντά. Διάμεση απόσταση **4.4 m** έναντι 6.2 m του SIFT· στη ζώνη
   κοντινού δαπέδου (< −10°, < 5 m) πέφτει **35 %** τους έναντι 21 % του SIFT (119 304 αντιστοιχίσεις σε 25 ζεύγη).
6. **Το εύρημα που αξίζει:** RoMa **> 5 m** → κλίμακα **0.998** (SIFT 0.970, RoMa όλες 0.856). Η συστηματική
   υποεκτίμηση της μετατόπισης ~3–4 %, που κουβαλάμε από το #026, **εξαφανίζεται**.
7. **Έλεγχος — δεν είναι γενικό φαινόμενο κοντινού πεδίου:** το ίδιο κατώφλι στο **SIFT** δεν διορθώνει την κλίμακα
   (0 m: 0.970 · 3 m: 0.970 · 5 m: 0.976 · 8 m: 0.939 · 12 m: 1.097, με θέση 17 → 104 mm). Άρα αφορά την **κατανομή**
   των πυκνών αντιστοιχίσεων, όχι απλώς το κόψιμο των κοντινών.

#### Προφίλ της μεροληψίας ανά απόσταση (προσθήκη, ίδια 60 ζεύγη)
Μετατόπιση με τη **στροφή δοσμένη από το GT**, εκτίμηση `median(p − R·q)` ανά ζώνη απόστασης (χωρίς RANSAC, άρα οι
απόλυτες τιμές δεν συγκρίνονται με το 0.97 της πλήρους διοχέτευσης — η **τάση** είναι το ζητούμενο):

| απόσταση της αντιστοίχισης | SIFT | RoMa |
|---|---|---|
| 1–3 m | 0.742 | 0.709 |
| 3–5 m | 0.741 | 0.766 |
| 5–8 m | 0.848 | 0.963 |
| 8–12 m | 0.896 | 0.933 |
| 12–20 m | 1.050 | 1.097 |
| 20–60 m | — | 0.879 |
| όλες | 0.828 | 0.771 |

**Μονότονη άνοδος με την απόσταση, και στους ΔΥΟ συνταυτιστές.** Οι αντιστοιχίσεις κάτω από 5 m αναφέρουν μόνο το
~74 % της πραγματικής μετατόπισης· πέρα από τα 12 m φτάνουν στο 100–105 %. Άρα η υποεκτίμηση **δεν** είναι ιδιότητα
του SIFT ούτε του RoMa: είναι φαινόμενο **κοντινού πεδίου** της συνταύτισης μοτίβων intensity — το μοτίβο στο κοντινό
δάπεδο εξαρτάται από απόσταση και γωνία πρόσπτωσης, άρα «σέρνεται» λιγότερο από όσο λέει η γεωμετρία (μηχανισμός #035).
Το RoMa φαίνεται χειρότερο συνολικά (0.771) **μόνο** επειδή δειγματοληπτεί περισσότερο κοντά (#039.5).

#### Οι τέσσερις υλοποιήσεις της διόρθωσης (προσθήκη· όλο το church_02, ray casting, έναντι GT)

Η στροφή είναι **αμετάβλητη σε όλες** (0.52°, 90ό 1.09°) — σωστά, δεν την αγγίζουμε. Βάση: κλίμακα 0.967, θέση 17 mm.

| υλοποίηση | τι κάνει | κλίμακα | θέση |
|---|---|---|---|
| — (βάση) | | 0.967 | **17 mm** |
| `vector`, > 5 m | αντικαθιστά το διάνυσμα με την εκτίμηση των μακρινών | 0.976 | 50 mm |
| `vector`, > 8 m | | **0.998** | 70 mm |
| `vector`, > 12 m | | 0.977 | 42 mm |
| `magnitude`, > 5 m | κρατά την ΚΑΤΕΥΘΥΝΣΗ από όλες, διορθώνει μόνο το μήκος | 0.969 | 28 mm |
| `magnitude`, > 8 m | | 0.985 | 31 mm |
| `magnitude`, > 12 m | | 0.972 | 26 mm |
| σταθερό ×1.034 | ένας αριθμός για όλη τη διαδρομή | **1.000** | **17 mm** |
| σταθερό ×1.060 | | 1.025 | 18 mm |
| **`auto`, > 8 m** | τρέχων διάμεσος του λόγου σε 100 προηγούμενες σαρώσεις | **1.004** | **18 mm** |

1. **Το ανταλλάγιο μεροληψίας–διασποράς είναι φυσικός περιορισμός:** όσο πιο μακριά κόβουμε, τόσο αμερόληπτη η κλίμακα
   αλλά τόσο μικρότερη η παράλλαξη, άρα τόσο θορυβωδέστερη η εκτίμηση. Το `vector` το πληρώνει τετραπλάσια (70 mm),
   το `magnitude` το υποδιπλασιάζει (26–31 mm).
2. **Ο σταθερός συντελεστής το εξαφανίζει** (1.000 / 17 mm): η μεροληψία είναι συστηματική, άρα δεν χρειάζεται να
   ξαναμετριέται ανά σάρωση. **Αλλά δεν γενικεύει:** ο ×1.034 του church_02 υπερδιορθώνει το christ-church-03 (0.986 → **1.020**),
   που χρειάζεται ×1.014. Το μέγεθος της μεροληψίας εξαρτάται από την κατανομή αποστάσεων της σκηνής.
3. **Το `auto` λύνει και τα δύο:** μετράει τον συντελεστή από τα δεδομένα, αιτιακά (μόνο προηγούμενες σαρώσεις, άρα
   δουλεύει και online), και βρίσκει **διαφορετική τιμή σε κάθε ακολουθία χωρίς να του πει κανείς**:

| ακολουθία | κλίμακα πριν | μετά | συντελεστής που βρήκε | θέση πριν → μετά |
|---|---|---|---|---|
| church_02 (2402 σαρ.) | 0.967 | **1.004** | 1.041 | 17 → 18 mm |
| christ-church-03 (3120) | 0.986 | **0.997** | 1.023 | 12 → 14 mm |
| keble-college-02 (2763) | 0.951 | **1.011** | 1.059 (διάμ.) | 18 → 19 mm |

⚠️ Στο keble ο τρέχων συντελεστής **παρασύρεται** στο τέλος (1.237 έναντι διαμέσου 1.059· περιορισμός στο 1.6): το
παράθυρο των 100 σαρώσεων είναι μικρό σε ανοιχτό χώρο. Να δοκιμαστεί διάμεσος όλου του παρελθόντος ή παράθυρο 300.
Παρατήρηση Λ.Γ.: αφού το loop closure τρέχει ούτως ή άλλως εκ των υστέρων, επιτρέπεται και **δεύτερο πέρασμα** —
που είναι ακριβώς ο σταθερός συντελεστής, ελαφρώς καλύτερος (1.000 / 17 mm) γιατί δεν έχει προθέρμανση ούτε παρέκκλιση.

#### End-to-end (προσθήκη, πρωτόκολλο #037: 4 σπόροι ανά βραχίονα, church_02)

| μέγεθος | σημερινή μ ± σ | με διόρθωση μ ± σ | t (6 β.ε.) |
|---|---|---|---|
| ATE RMSE (m) | 0.0936 ± 0.0414 | 0.1217 ± 0.0462 | 0.91 |
| z RMSE (m) | 0.0768 ± 0.0472 | 0.1074 ± 0.0525 | 0.87 |
| RPE μετατόπισης (m) | 0.0299 ± 0.0005 | 0.0304 ± 0.0004 | 1.41 |
| RPE στροφής (°) | 0.6183 ± 0.0033 | 0.6189 ± 0.0030 | 0.28 |
| μήκος διαδρομής (m) | 265.90 ± 0.10 | **266.22 ± 0.17** | **3.17** |

Η μόνη σημαντική διαφορά είναι το μήκος διαδρομής, και είναι **προς τα χειρότερα** (GT 253.39 m: ήδη υπερεκτιμούμε).

#### Συμπέρασμα ⏳
-1. **Η διόρθωση ΔΕΝ βελτιώνει το SLAM και δεν γίνεται προεπιλογή.** Η μεροληψία ζει στη μετρούμενη κίνηση, που
   χρησιμεύει μόνο για ξεστρέβλωση και αρχικό σημείο του ICP· την τελική θέση τη βγάζει ο ICP πάνω στον χάρτη, ο οποίος
   απορροφά το σφάλμα κλίμακας του αρχικού σημείου. **Παράπλευρο κέρδος: αποκλείστηκε η μεροληψία κλίμακας ως αιτία της
   υπερεκτίμησης του μήκους διαδρομής (+5 %)** — μένει ανοιχτό και είναι το μεγαλύτερο γνωστό συστηματικό μας σφάλμα.
0. **ΤΟ ΚΥΡΙΟ ΕΥΡΗΜΑ: η συστηματική υποεκτίμηση της μετατόπισης είναι συνάρτηση της ΑΠΟΣΤΑΣΗΣ της αντιστοίχισης**
   (0.74 στα < 5 m → ~1.00 στα > 12 m), κοινή και στους δύο συνταυτιστές. Δίνει άμεσο, φθηνό δρόμο διόρθωσης
   (βάρος ή κατώφλι απόστασης στο στάδιο της μετατόπισης) που **δεν χρειάζεται καθόλου RoMa**.
1. **Ως αντικαταστάτης του SIFT, το RoMa v2 δεν κερδίζει** σε αυτά τα δεδομένα: ίδιος διάμεσος, χειρότερη ουρά,
   χειρότερη θέση, 224 ms έναντι ~25 ms ανά ζεύγος. Επιβεβαιώνει το #038.
2. **Ως εργαλείο για την κλίμακα, ναι:** οι μακρινές πυκνές αντιστοιχίσεις δίνουν αμερόληπτη μετατόπιση (0.998).
   Επόμενο βήμα: **συνδυασμός** — στροφή από τις κοντινές/όλες, μετατόπιση από τις μακρινές, ή βάρος ανάλογο της
   απόστασης. Μπορεί να γίνει και με SIFT + περισσότερα μακρινά χαρακτηριστικά.
3. **Αν χρειαστεί πυκνή συνταύτιση σε πραγματικό χρόνο**, ο δρόμος δεν είναι το RoMa αλλά **EDM / EfficientLoFTR**
   σε CUDA. Σε αυτό το Mac καμία πυκνή μέθοδος δεν είναι real-time.
4. Επιφύλαξη: μόνο `turbo` (320×320 σε εικόνα 2:1, χωρίς περιτύλιξη αζιμουθίου) σε 60 ζεύγη μιας ακολουθίας. Το
   `precise` σε 11 ζεύγη έδωσε 0.70° έναντι 0.74° του SIFT — ομοίως χωρίς κέρδος.

---

### 2026-09-20 — #038 Πόσες αντιστοιχίσεις χρειαζόμαστε; (αφορμή: πρόταση Λ.Γ. για RoMa)

**git commit:** αυτή η εγγραφή · scratchpad `subsample2.py` (αιτιακό τεστ) · npz `i3_motion_car_sp_st0.05_floor{,_ray}{,_seedN}.npz`

#### Στόχος
Πρόταση Λ.Γ.: αντικατάσταση του SIFT με **RoMa / RoMa2** (πυκνή συνταύτιση, χιλιάδες αντιστοιχίσεις). Πριν από
οποιαδήποτε υλοποίηση: **είναι το πλήθος των αντιστοιχίσεων ο περιοριστικός παράγοντας;**

#### Μέθοδος
1. **Συσχέτιση** (2401 σαρώσεις, από τα υπάρχοντα npz): σφάλμα κίνησης έναντι GT ανά τεταρτημόριο πλήθους inliers.
2. **Αιτιακό τεστ** (το κρίσιμο): στις **ίδιες** 601 σαρώσεις κρατούνται τυχαία 15 / 25 / 40 / 80 / 160 / όλες οι
   αντιστοιχίσεις και ξανατρέχει RANSAC + `fit_time`. Ray casting, με φίλτρο δαπέδου, σύγκριση με GT.
3. **Ευστάθεια του RANSAC**: διαφωνία της κίνησης της ίδιας σάρωσης ανάμεσα στους 4 σπόρους του #037.

#### Αποτέλεσμα
1. **Συσχέτιση** (splat): inliers 11–47 → στροφή 0.81°, 47–72 → 0.74°, 72–135 → 0.62°, 135–402 → **0.44°**·
   θέση 28 → 13 mm. Συσχέτιση log–log −0.37 (στροφή), −0.51 (θέση). Στο χειρότερο 10 % των σαρώσεων τα inliers είναι
   50 έναντι 77 στις υπόλοιπες. **Συγχέεται όμως με τη σκηνή** (φτωχές σε υφή περιοχές δίνουν και λίγες αντιστοιχίσεις
   και κακή γεωμετρία) — γι' αυτό το αιτιακό τεστ.
2. **Αιτιακό τεστ, 601 ζεύγη, μόνο το πλήθος αλλάζει:**

   | όριο πλήθους | πραγμ. μέσο | στροφή διάμ. | 90ό | θέση διάμ. | κλίμακα | αποτυχίες |
   |---|---|---|---|---|---|---|
   | 15 | 15 | 0.97° | 1.88° | 51 mm | 0.886 | 47 |
   | 25 | 25 | 0.92° | 1.93° | 32 mm | 0.929 | 0 |
   | 40 | 40 | 0.71° | 1.62° | 23 mm | 0.961 | 0 |
   | 80 | 80 | 0.61° | 1.25° | 19 mm | 0.965 | 0 |
   | 160 | 141 | 0.54° | 1.19° | 17 mm | 0.966 | 0 |
   | όλες | 198 | **0.53°** | **1.19°** | **17 mm** | 0.964 | 0 |

   **Κορεσμός στις ~80–160.** Από 80 σε 198 το κέρδος είναι 13 % στη στροφή και 2 mm· από 160 σε 198, μηδέν.
   Προσαρμογή σφάλμα² ≈ a/N + b²: **b ≈ 0.50°**, a ≈ 10 — δηλαδή ακόμη και με **άπειρες** αντιστοιχίσεις της ίδιας
   ποιότητας το σφάλμα δεν πέφτει κάτω από ~0.50° (σήμερα 0.53°).
   Η **κλίμακα** επίσης κορεσμένει στο 0.965: η υποεκτίμηση 3.5 % είναι **συστηματική**, όχι θέμα πλήθους.
3. **Σήμερα:** διάμεσος 80 inliers (raycast), 10ο εκατοστημόριο 38. Δηλαδή **περίπου οι μισές σαρώσεις είναι κάτω από
   το σημείο κορεσμού** — εκεί υπάρχει περιθώριο (0.7–0.97° → ~0.55°), και είναι ακριβώς οι σαρώσεις που παράγουν την
   ουρά του σφάλματος (90ό εκατ. 1.19°).
4. **Το RANSAC είναι ευσταθές:** η κίνηση της ίδιας σάρωσης στους 4 σπόρους διαφωνεί κατά **0.000° διάμεσο**
   (99ό 0.88° splat / 0.56° raycast, > 1° μόνο στο 0.8 % / 0.1 % των σαρώσεων). Άρα η διασπορά του ATE (#037)
   **δεν** γεννιέται στον εκτιμητή: ελάχιστες διαφορές σε λίγες δύσκολες σαρώσεις μεγεθύνονται μέσα στο SLAM.

#### ⚠️ Παγίδα που κόστισε (να μην επαναληφθεί)
Ο reader του rosbag (`dataset_factory`) είναι **σειριακός**: `ds[k]` χωρίς να έχουν διαβαστεί τα προηγούμενα
επιστρέφει **άλλη σάρωση**. Η πρώτη εκδοχή του τεστ πηδούσε σαρώσεις και έδινε σφάλματα 2.5° αντί 0.53°.
Κάθε script που δειγματοληπτεί σαρώσεις πρέπει να **διαβάζει από το 0** και να επεξεργάζεται επιλεκτικά
(όπως κάνουν ήδη τα `make_panorama_video.py`, `figure_panorama_zoom.py`).

#### Συμπέρασμα ⏳
1. **Το πλήθος των αντιστοιχίσεων είναι πράγματι περιοριστικό — αλλά μόνο κάτω από ~80–160**, και το κέρδος από εκεί
   και πάνω είναι μηδενικό με τη σημερινή ποιότητα αντιστοιχίσεων.
2. **Η προοπτική του RoMa δεν είναι «καλύτερος διάμεσος» αλλά «κοντύτερη ουρά»:** οι μισές σαρώσεις που σήμερα έχουν
   < 80 αντιστοιχίσεις. Επιπλέον όφελος μόνο αν η **ακρίβεια θέσης ανά αντιστοίχιση** είναι καλύτερη (μικραίνει το a)
   ή αν η **κατανομή** των αντιστοιχίσεων αλλάζει (μπορεί να μικρύνει το συστηματικό b).
3. Το συστηματικό υπόλοιπο (b ≈ 0.50°, κλίμακα 0.965) **δεν** λύνεται με περισσότερες αντιστοιχίσεις· είναι η επόμενη
   ανεξάρτητη γραμμή έρευνας.

---

### 2026-09-19 — #037 Μέτρηση διασποράς: πόσο αξίζει το ATE ενός run;

**git commit:** αυτή η εγγραφή · `scripts/measure_variance.sh` · `precompute_i3_motion.py --seed=` ·
runs `var_{splat,raycast}_s{0..3}` · log `runs/variance_church02.log`

#### Στόχος
Σε όλες τις πρόσφατες εγγραφές (#031–#036) συγκρίναμε παραλλαγές με **ένα run η καθεμία** και σημειώναμε «⏳ χωρίς
εκτίμηση διασποράς». Ερώτημα: πόσο αλλάζει το αποτέλεσμα όταν **δεν αλλάζει τίποτα** στη μέθοδο;

#### Μέθοδος
church_02, 4 σπόροι του RANSAC (0–3) × 2 τρόποι κατασκευής της εικόνας (splat, ray casting) = 8 ανεξάρτητα runs. Για κάθε
ένα: κίνηση από την εικόνα (`precompute_i3_motion.py --model=car --subpixel --stuck=0.05 --floor-only --seed=N`) και μετά
SLAM με αυτήν (`--image-deskew --motion-file`). Τίποτε άλλο δεν αλλάζει: ίδιο bag, ίδιο config, ίδια μέθοδος. Σειριακά,
ένα run τη φορά, 8 νήματα (~1 ώρα). Αξιολόγηση έναντι GT με offset −10 ms.

#### Αποτέλεσμα

| μέγεθος | splat: μ ± σ | εύρος | raycast: μ ± σ | εύρος |
|---|---|---|---|---|
| **ATE RMSE (m)** | 0.1186 ± 0.0232 | 0.0929–0.1447 | 0.0936 ± 0.0414 | 0.0610–0.1542 |
| z RMSE (m) | 0.1084 ± 0.0240 | 0.0835–0.1351 | 0.0768 ± 0.0472 | 0.0372–0.1452 |
| **RPE μετατόπισης (m)** | 0.0333 ± 0.0004 | 0.0329–0.0337 | 0.0299 ± 0.0005 | 0.0295–0.0306 |
| **RPE στροφής (°)** | 0.7052 ± 0.0045 | 0.6984–0.7079 | 0.6183 ± 0.0033 | 0.6159–0.6228 |
| μήκος διαδρομής (m) | 268.71 ± 0.16 | 268.58–268.93 | 265.90 ± 0.10 | 265.79–266.04 |

1. **Το ATE ενός run είναι ουσιαστικά μη επαναλήψιμο:** σχετική διασπορά **20 %** (splat) και **44 %** (raycast). Το 95 %
   ενός μέσου 4 runs είναι ±0.027 m· μια μεμονωμένη διαφορά κάτω από ~0.05 m δεν κρίνεται.
2. **Ο θόρυβος ΔΕΝ είναι του εκτιμητή κίνησης:** σε κάθε σπόρο η κίνηση είναι πρακτικά ταυτόσημη (splat: κλίμακα
   0.961–0.962, στροφή διάμεσος 0.64°, θέση 18 mm· raycast: 0.967–0.968, 0.52°, 17 mm). Γεννιέται μέσα στο SLAM —
   ελάχιστα διαφορετική αρχική θέση → άλλη σύγκλιση του ICP και του γράφου. Τα loop closures ήταν 2 σε όλα τα runs.
3. **splat έναντι raycast στο ATE: ΜΗ σημαντική διαφορά** (0.119 έναντι 0.094, t = 1.05, 6 β.ε., όριο 2.45· τα εύρη
   επικαλύπτονται πλήρως).
4. **Στο RPE και στο μήκος: σαφέστατα σημαντική.** RPE μετατόπισης −10 % (t = 11), RPE στροφής −12 % (t = 31), **καμία
   επικάλυψη** μεταξύ των δύο ομάδων σε κανένα από τα δύο. Μήκος 268.7 → 265.9 m (GT 253.4): το raycast παράγει λιγότερη
   περίσσεια διαδρομής. Αυτά τα μέτρα έχουν σ 60–100 φορές μικρότερο από το ATE.

#### Συμπέρασμα ⏳
1. **Κανόνας για όλα τα επόμενα πειράματα: το ATE ενός run δεν είναι απόδειξη.** Διαφορά < 0.05 m στο church_02 δεν
   σημαίνει τίποτα. Είτε 4 runs και μέσος ± σ, είτε κρίση με **RPE / μήκος διαδρομής**, που είναι σταθερά.
2. **Ακυρώνεται το #034(3)** («το ATE πέφτει κατά τη μισή με ray casting στο church_02»): ήταν τύχη του σπόρου. Το ray
   casting **βελτιώνει πραγματικά** την τοπική ακρίβεια (RPE −10 / −12 %) — αυτό ήταν το σωστό εύρημα εξαρχής, και είναι
   συνεπές με τη βελτίωση της ίδιας της κίνησης (στροφή 0.64° → 0.52°).
3. **Επιβεβαιώνονται ως θόρυβος** τα #033 (καμπύλη, ±0.015) και #036(9) (`min_range`, 0.016).
4. **Επιβιώνουν** όλες οι μεγάλες διαφορές: KISS 0.319 έναντι 0.130 (8 σ), christ-church-03 0.122 έναντι 0.038,
   keble 2.133 έναντι 0.094 (50 σ). Το κύριο αποτέλεσμα της εργασίας δεν θίγεται.
5. **Για τη δημοσίευση:** κάθε νούμερο ATE χρειάζεται μέσο ± σ από ≥ 4 runs, αλλιώς αναφέρεται RPE. Να μετρηθεί η
   διασπορά και στις άλλες δύο ακολουθίες, και του ίδιου του KISS ως αναφοράς.

---

### 2026-09-19 — #036 Τι είναι καρφωμένο στον σαρωτή: σκιές της διάταξης, χειριστής — και γιατί καμία μάσκα δεν βοηθά

**git commit:** αυτή η εγγραφή · `scripts/analyze_rig_mask.py` · `kiss_slam/intensity_deskew.py::PIXEL_MASK` ·
`precompute_i3_motion.py --mask= --mask-key=` · `runs/rig_masks_church02.npz` · σχήματα `docs/figures/rig_temporal_median.png`,
`rig_edges.png`, `rig_annotated.png`, `rig_mask_vs_edges.png`

#### Στόχος
Συνέχεια του #035. Παρατηρήσεις Λ.Γ. στο βίντεο: οι κόκκινες (κολλημένες στο δάπεδο) μαζεύονται δίπλα στη μάσκα σκιάς·
το «μαύρο τρίγωνο» σημαδεύεται μόνο περιμετρικά· υπάρχει και ένα «τετράγωνο» δεξιά. Ερωτήματα: (α) τι είναι αυτά τα
σχήματα και σε ποια απόσταση· (β) αν διευρύνουμε τη ζώνη σκιάς (dilate ή buffer κατά μέγεθος σημείου SIFT) τι κερδίζουμε·
(γ) αν εξαιρέσουμε ολόκληρη τη ζώνη δαπέδου από το SIFT· (δ) αν βάλουμε τη μάσκα πάνω στο σημερινό φίλτρο.

#### Μέθοδος
481 σαρώσεις (κάθε 5η) του church_02, ray casting, όλα τα καρέ στη μνήμη. Ανά pixel: μέση, διάμεση, τυπική απόκλιση,
MAD, μέση απόσταση και διασπορά, ποσοστό χωρίς μέτρηση. Τρία κριτήρια για «καρφωμένο»: (1) σκουρότερο από τους ±20
γείτονες του δακτυλίου σταθερά στον χρόνο (η «στενή» μάσκα του #035, t > 20)· (2) σταθερή τιμή στον χρόνο (MAD)· (3)
**φίλτρο ακμών στη χρονικά διάμεση εικόνα** (ιδέα Λ.Γ.), παράγωγος κατά το αζιμούθιο του log(διάμεσης), κατώφλι 97ο
εκατοστημόριο (7 % ανά pixel). Για τα (β)–(δ): 13 610 αντιστοιχίσεις SIFT των σαρώσεων 400–520 (ποια κόκκινα πιάνει, πόσες
καλές θυσιάζει) και ο εκτιμητής κίνησης σε όλο το bag έναντι GT (`precompute_i3_motion.py`, car + subpixel).

#### Αποτέλεσμα
1. **Γιατί το τρίγωνο σημαδεύεται μόνο περιμετρικά:** το κριτήριο (1) συγκρίνει με ±20 στήλες· λεκές πλατύτερος από το
   παράθυρο γίνεται ο ίδιος του ο κανόνας. Παράθυρο ±20 → 2.0 % της εικόνας, ±60 → 7.2 %, ±150 → 20.5 %, ±300 → 24 %: το
   πλατύ παράθυρο πιάνει τη σκοτεινή σκηνή. Το κριτήριο (1) εγκαταλείπεται.
2. **Σταθερή τιμή στον χρόνο (MAD):** μόνο το τρίγωνο (τιμή 24, MAD 1 έναντι 11 στη σκηνή). Οι δύο γραμμές **δεν** έχουν
   σταθερή τιμή: είναι μερική σκίαση (70–80 % της τιμής του δαπέδου) και ακολουθούν το δάπεδο. Η τυπική απόκλιση (37) είναι
   φουσκωμένη από λίγες ακραίες σαρώσεις· η MAD (11) είναι το σωστό μέτρο σταθερότητας.
3. **Φίλτρο ακμών στη διάμεση εικόνα — βρίσκει και τα τρία σχήματα** (1425 px, 2.2 %). Μεγαλύτερες συνεκτικές ακμές:
   239 px δακτ. 6–40 στήλες 466–482 και 185 px δακτ. 4–35 στήλες 21–38 (τα δύο όρια της σιλουέτας του χειριστή)· 70+62+50 px
   δακτ. 39–54 στήλες 450–492 (οι δύο γραμμές)· 33 px δακτ. 57–63 στήλες 675–686 (πλευρά του τριγώνου).
4. **Τι είναι και σε ποια απόσταση** (`rig_annotated.png`): δύο γραμμές = μερική σκίαση του δαπέδου στα **3.3 m**· τρίγωνο
   = σχεδόν ολική σκίαση, δάπεδο στα **3.5 m**· «τετράγωνο δεξιά» = **σιλουέτα του χειριστή**, 19 % της εικόνας χωρίς
   **καμία** επιστροφή (δακτ. 0–43, στήλες 550–968 πάνω, στενότερο στον ορίζοντα), πάντα κενό κάτω δακτ. 56–63 στήλες
   683–777 (χέρι/λαβή)· σκούρα ζώνη δακτ. 40–52 δεξιά = δάπεδο στα ~10 m μερικώς σκιασμένο (αιτία όχι αποδεδειγμένη)·
   σημεία < 1 m: 0.28 % των σημείων (~150 ανά σάρωση), στα **0.48 m**, δακτ. 61–63 στήλες 500–1005 (η διάταξη)· τα κόβει
   το `MIN_RANGE = 1.0` της **εικόνας**, αλλά **όχι** ο ICP (`min_range: 0.0` στο `indoor_detail.yaml`).
   Ο οδηγός **δεν γράφει** σημεία χωρίς επιστροφή (54 079–54 712 ανά σάρωση αντί 76 800): το εμπόδιο είναι μέσα στη νεκρή
   ζώνη του αισθητήρα και δεν μετριέται πουθενά — μόνο η σκιά του πάνω στο δάπεδο, με **σωστό βάθος**.
5. **Οι κόκκινες κάθονται δίπλα στη μάσκα:** απόσταση από τη στενή μάσκα διάμεσος 1 px, 73 % σε ≤ 3 px (κανονικές: 30 px,
   15 %). Στο περίγραμμα του χειριστή (≤ 2 px) πέφτουν 184 αντιστοιχίσεις, 33 % κολλημένες (έναντι 7.5 % συνολικά).
6. **Dilate της ένωσης (στενή + ακμές):** ακτίνα 0 → 36 % των κόκκινων / 8 % των καλών· 1 → 59 / 15· 2 → 73 / 22· 3 → 78 / 26·
   6 → 85 / 39. **Buffer κατά μέγεθος σημείου** (εξαίρεση αν απόσταση ≤ f × ακτίνα του keypoint): f = 1 → 42 % / 8 %,
   f = 1.5 → 70 / 13, f = 2 → 78 / 16. Με f = 1 μόνο το 42 % των κόκκινων έχει τη σκιά μέσα στον περιγραφέα του — άρα
   **δεν είναι η σκιά που τις κάνει κολλημένες**, είναι η υφή του ακίνητου κοντινού δαπέδου γύρω της (#035).
7. **Εκτιμητής κίνησης σε όλο το bag έναντι GT** (κλίμακα / στροφή διάμεσος, 90ό / θέση / inliers / αποτυχίες):

   | | κλίμακα | στροφή | 90ό | θέση | inliers | αποτυχίες |
   |---|---|---|---|---|---|---|
   | σημερινό φίλτρο (κολλημένες στο δάπεδο) | 0.961 | 0.64° | 1.30° | 18 mm | 72 | 0 |
   | **όλη η ζώνη δαπέδου έξω** (< −10°, < 5 m) | 0.972 | 0.69° | 1.38° | 22 mm | 56 | 13 |
   | σημερινό + στενή μάσκα (758 px) | 0.965 | 0.63° | 1.29° | 18 mm | 71 | 1 |
   | σημερινό + ένωση (1949 px) | 0.966 | 0.63° | 1.27° | 18 mm | 69 | 1 |

9. **`min_range: 1.0` στον ICP** (ερώτηση Λ.Γ.: να εξαιρεθούν τα σημεία της διάταξης και από τον ICP;) — church_02, ίδια
   κίνηση από `i3_motion_car_sp_st0.05_floor.npz`, run `indoor_detail_i3_minrange1` έναντι `indoor_detail_i3_init_sigfixed`:
   ATE 0.130 → **0.146**, z RMSE 0.121 → 0.137, RPE μετατ. 0.034 → 0.035, closures 2 → 2, μήκος 268.6 → 269.0 m. Ένα run.
8. Το «−10°» του φίλτρου είναι στο **σύστημα του σαρωτή** (κάθε δακτύλιος έχει σταθερή γωνία → οριζόντια λωρίδα στην
   εικόνα, που γέρνει μαζί με το χέρι)· το φίλτρο το αντέχει επειδή έχει και το όριο 5 m και το κριτήριο «κολλημένη».

#### Συμπέρασμα ⏳
1. **Διόρθωση του #035(1): σκιές της διάταξης υπάρχουν** (δύο γραμμές, τρίγωνο) και ο χειριστής κρύβει το 19 % της
   εικόνας — απλώς **δεν είναι η αιτία** των κολλημένων. Η αιτία μένει το γεωμετρικά ακίνητο κοντινό δάπεδο· οι σκιές είναι
   η πιο έντονη υφή πάνω του.
2. **Κάθε κριτήριο θέσης στην εικόνα** (μάσκα, dilate, buffer, αποκλειστική ζώνη) κόβει καλές μαζί με κακές: για κάθε 3
   κολλημένες ~1 καλή. Το κριτήριο «μετακινήθηκε ως προς τον σαρωτή» τις ξεχωρίζει καθαρά (100 % / 0 %). Η αποκλειστική
   ζώνη δαπέδου είναι **χειρότερη**: το κοντινό δάπεδο δίνει τα μόνα κοντινά σημεία κάτω από τον σαρωτή (ύψος, κλίση).
3. **ΑΠΟΦΑΣΗ Λ.Γ.: το φίλτρο δαπέδου μένει ως έχει.** Η μάσκα pixel (`PIXEL_MASK`, `--mask=`) μένει προαιρετική και
   απενεργοποιημένη: κέρδος κλίμακας +0.005, θεωρητικά σωστή, ίσως χρήσιμη σε διάταξη που σκιάζει περισσότερο.
4. **Ο ICP (ερώτηση Λ.Γ.):** τα σημεία κάτω από τις σκιές έχουν σωστό βάθος (είναι το δάπεδο) και ο χειριστής δεν έχει
   σημεία — δεν χρειάζονται εξαίρεση. Τα ~150 σημεία της **διάταξης** στα 0.48 m όμως **μπαίνουν** στον ICP (config
   `min_range: 0.0`) και είναι καρφωμένα στον σαρωτή· μετά το voxel 1.0 m είναι 1–3 από ~900 σημεία. **Δοκιμάστηκε**
   (αποτέλεσμα 9): με `min_range: 1.0` το ATE ανεβαίνει 0.130 → 0.146, μέσα στο εύρος ±0.015 ενός run. Δεν υπάρχει κέρδος·
   το config μένει ως έχει. Πιθανή εξήγηση της μικρής χειροτέρευσης: τα σημεία στα 0.48 m δεν είναι μόνο η διάταξη —
   στις σκάλες και στις στροφές ο τοίχος πλησιάζει στο μισό μέτρο, και τα σημεία αυτά χάνονται (όχι αποδεδειγμένο).
5. Επιφύλαξη: η μάσκα υπολογίστηκε και δοκιμάστηκε στο ίδιο bag· εξαρτάται από τη διάταξη, όχι από τη σκηνή, αλλά ο
   καθαρός έλεγχος είναι church_02 → christ-church-03.

---

### 2026-09-19 — #035 Στατιστική ανά pixel: τι είναι πράγματι «ακίνητο» ως προς τον σαρωτή

**git commit:** αυτή η εγγραφή · `scripts/analyze_static_pixels.py`, `scripts/make_panorama_video.py` ·
σχήματα `docs/figures/pixel_statistics.png`, `near_field_rig.png`, `static_pixels.png` · βίντεο `runs/videos/`

#### Στόχος
Ιδέα Λ.Γ. μετά από παρατήρηση στο βίντεο των πανοραμάτων: οι σκούρες γραμμές και τα σχήματα στο κάτω μέρος μοιάζουν με
σκιές καλωδίων/εξαρτημάτων δίπλα στον σαρωτή. Πρόταση: στατιστική ανά pixel σε όλη τη διαδρομή, για να βρεθούν όσα
δείχνουν πάντα το ίδιο πράγμα.

#### Μέθοδος
481 σαρώσεις (κάθε 5η) του church_02. Ανά pixel: ποσοστό σαρώσεων χωρίς μέτρηση, μέση τιμή και διασπορά intensity και
**απόστασης**, και χωριστά τα pixel με επιστροφή < 1.5 m (χωρίς το όριο `MIN_RANGE`, ώστε να φανεί η ίδια η διάταξη).
Έλεγχος: πού πέφτουν οι «κολλημένες» αντιστοιχίσεις σε σχέση με αυτά.

#### Αποτέλεσμα
1. **Η διάταξη:** 52 pixel (0.08 %) με μόνιμη επιστροφή στα 0.67 m, μόνο στους δακτυλίους 62–63, σε αζιμούθια 12° και
   155–172°. Τα σημεία της (105 στα 0–0.5 m, 97 στα 0.5–1 m, **κανένα** στα 1–2 m) τα κόβει ήδη το `MIN_RANGE`.
2. **Δεν ρίχνει σκιά:** ποσοστό σαρώσεων χωρίς μέτρηση στις στήλες της 41 % έναντι 43 % αλλού (δακτύλιοι 40–62).
3. **Οι «κολλημένες» δεν είναι εκεί:** 9.2 % μέσα στη ζώνη της (±8 στήλες), ενώ η ζώνη είναι 9.3 % της εικόνας. Στον
   όροφο μάλιστα λιγότερες (4.0 % έναντι 5.4 % των υπολοίπων).
4. **Σταθερό είναι το δάπεδο:** διασπορά απόστασης ανά pixel σε όλη τη διαδρομή — δακτύλιοι 18–36: 1.1–1.4 m·
   42: 0.7· 48: 0.2· 54–60: **0.1 m**. Ο σαρωτής κρατιέται σε σταθερό ύψος, άρα το κοντινό δάπεδο είναι γεωμετρικά
   ακίνητο ως προς αυτόν.
5. Οι «κολλημένες» έχουν διασπορά απόστασης **0.00–0.10 m** στο pixel τους, οι υπόλοιπες 0.14–0.31 m. Ως φίλτρο όμως
   είναι ακριβό: κατώφλι 0.3 m κόβει 70–74 % των κολλημένων αλλά και 52–54 % όλων των αντιστοιχίσεων.

#### Συμπέρασμα ⏳
1. **Η υπόθεση «σκιές της διάταξης» δεν επιβεβαιώνεται.** Η διάταξη είναι μετρήσιμη αλλά αμελητέα σε έκταση και δεν
   σκιάζει. → **Διορθώνεται στο #036:** σκιές υπάρχουν (δύο γραμμές, τρίγωνο, σιλουέτα χειριστή 19 %), απλώς δεν είναι η
   αιτία των κολλημένων· το σημείο 2 παρακάτω παραμένει.
2. **Η βαθύτερη αιτία είναι γεωμετρική:** κάτω από φορητή μονάδα σταθερού ύψους, το κοντινό δάπεδο δεν αλλάζει ούτε
   απόσταση ούτε γωνία πρόσπτωσης — άρα και το μοτίβο φωτεινότητάς του είναι σχεδόν ακίνητο. Αυτό εξηγεί γιατί το
   φίλτρο «κοντινό δάπεδο» (#027) ήταν τόσο αποτελεσματικό, και είναι καλύτερη αιτιολόγηση από τα εμπειρικά κατώφλια.
3. Η διασπορά απόστασης ανά pixel είναι χρήσιμη ως **βάρος εμπιστοσύνης**, όχι ως απόρριψη.

---

### 2026-09-19 — #034 Ray casting (ιδέα Ι-5) στην κατασκευή του πανοράματος

**git commit:** αυτή η εγγραφή · `kiss_slam/intensity_deskew.py::raycast_grid` (`RENDER`) ·
`precompute_i3_motion.py --render=raycast` · runs `indoor_detail_i3_ray`, `christ-church-03_i3_ray` ·
σχήματα `docs/figures/panorama_zoom.png`, `sift_input.png`

#### Στόχος
Ιδέα Λ.Γ. (Ι-5): το πανόραμα να χτίζεται «από το pixel προς την επιφάνεια» (ακτίνα ανά pixel) αντί για «κάθε σημείο
στο πλησιέστερο pixel». Ερώτημα: εξαφανίζονται οι «κολλημένες» αντιστοιχίσεις χωρίς το φίλτρο δαπέδου;

#### Μέθοδος
`raycast_grid`: κάθε pixel = ακτίνα (ύψος του δακτυλίου του, αζιμούθιο του κέντρου του pixel)· απόσταση, intensity και
χρόνος με παρεμβολή ανάμεσα στα δύο δείγματα του δακτυλίου εκατέρωθεν· καμία παρεμβολή πάνω σε ακμή βάθους (παίρνεται
το πλησιέστερο σε γωνία) ή πάνω από κενό χωρίς επιστροφή· με διπλή επιστροφή κρατιέται η κοντινότερη. 71 % έγκυρα pixel
(από 42 %), ~20 ms/σάρωση επιπλέον.

#### Αποτέλεσμα
**Εκτός SLAM, χωρίς κανένα φίλτρο** (κλίμακα / στροφή / θέση):

| | splat | raycast |
|---|---|---|
| ισόγειο 300–400 | 0.815 / 0.77° / 45 mm | 0.853 / **0.60°** / **33 mm** |
| διάδρομος 400–500 | 0.793 / 0.88° / 35 mm | 0.798 / **0.60°** / **29 mm** |
| όροφος 1200–1300 | 0.896 / 0.50° / 13 mm | 0.905 / **0.34°** / 12 mm |

Με πανόραμα 600 στηλών (μία μέτρηση ανά στήλη): **χειρότερα** (κλίμακα 0.64–0.87, λιγότερες αντιστοιχίσεις).

**Μέσα στο SLAM** (με το φίλτρο δαπέδου, init, σ 2.0):

| | κίνηση: στροφή / θέση / κλίμακα | ATE (m) | z RMSE | RPE μετατ. | RPE στροφής |
|---|---|---|---|---|---|
| church_02 splat | 0.64° / 18 mm / 0.961 | 0.130 | 0.121 | 0.034 | 0.71° |
| **church_02 raycast** | **0.52° / 17 mm / 0.967** | **0.061** | **0.037** | **0.030** | **0.62°** |
| christ-church-03 splat | 0.56° / 14 mm / 0.985 | **0.038** | **0.029** | 0.021 | 0.66° |
| christ-church-03 raycast | **0.46° / 12 mm / 0.986** | 0.056 | 0.051 | **0.020** | **0.60°** |
| keble-college-02 splat | 0.094 (#032) | 0.094 | 0.038 | 0.051 | 0.86° |
| keble-college-02 raycast | — (η σύγκριση με GT έσκασε σε IndexError· το npz υπάρχει) | 0.099 | 0.041 | — | — |

(keble: μήκος διαδρομής 349.6 → 346.0 m, GT 293.9· τελικό z −0.03 → −0.06 m· αξιολογήθηκε 19/9 με `evaluate_gt.py … --offset-ms=-10`.)

#### Συμπέρασμα ⏳
1. **Η κατασκευή της εικόνας όντως πρόσθετε θόρυβο:** η ακρίβεια στροφής βελτιώνεται 18–32 % και η θέση 8–19 %, σε όλα
   τα παράθυρα και στις δύο ακολουθίες.
2. **Δεν λύνει όμως την υποεκτίμηση της μετατόπισης:** χωρίς φίλτρο η κλίμακα μένει 0.80–0.90. Άρα τα «κολλημένα»
   μοτίβα είναι φυσικά (εξάρτηση του intensity από απόσταση/γωνία στο σχεδόν ακίνητο δάπεδο, #035), όχι τεχνούργημα
   της σχεδίασης. Το φίλτρο δαπέδου παραμένει.
3. ~~**Το ATE δεν ακολουθεί μονότονα:** −53 % στο church_02, +47 % στο christ-church-03, +5 % στο keble.~~
   **ΑΚΥΡΟ (#037):** με 4 σπόρους ανά τρόπο, το ATE έχει διασπορά 20–44 % και οι δύο ομάδες δεν ξεχωρίζουν (0.119 ± 0.023
   έναντι 0.094 ± 0.041, t = 1.05). Το 0.061 ήταν η τυχερή τιμή. **Το σωστό εύρημα:** το ray casting βελτιώνει την τοπική
   ακρίβεια με βεβαιότητα (RPE μετατόπισης −10 %, στροφής −12 %, καμία επικάλυψη σε 4+4 runs) — συνεπές με το σημείο 1.

---

### 2026-09-19 — #033 Deskew με ολόκληρη την καμπύλη κίνησης, ανά σημείο

**git commit:** αυτή η εγγραφή · `kiss_slam/intensity_deskew.py::deskew_curve` · `precompute_i3_motion.py` (σώζει
`params`, `t_start`) · `run_i3_deskew.py … curve` · `scripts/figure_deskew_interpolation.py` ·
runs `indoor_detail_i3_curve`, `christ-church-03_i3_curve`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 19/9, βελτίωση (i) του §2.2: ο KISS κατανέμει το συνολικό `delta` **ομοιόμορφα** μέσα στη σάρωση
(`exp((s−1)·log delta)`), οπότε η γωνιακή επιτάχυνση που μετρά το μοντέλο «car» χρησιμοποιείται μόνο για σωστότερο
συνολικό `delta`. Εδώ κάθε σημείο ισιώνεται με τη θέση του σαρωτή **τη δική του στιγμή** πάνω στην καμπύλη.

#### Μέθοδος
`deskew_curve(xyz, ts, params, t_start)`: p' = pose(1)⁻¹ · pose(τ_i) · p_i, με pose(τ) = [R(τw + ½τ²α), τv] από την
προσαρμογή και τ_i = (t_i − t_start)/περίοδο· έξοδος στο πλαίσιο του τέλους της σάρωσης και μετά η ίδια περικοπή
απόστασης με τον KISS, ώστε να αντικαθιστά τον `Preprocessor.preprocess` ένα προς ένα.
**Έλεγχος:** με μοντέλο «cv» (χωρίς επιτάχυνση) ταυτίζεται με το deskew του KISS — max διαφορά **0.32 mm** σε 54.600
σημεία· με α = 2°/περίοδο² στο yaw η διαφορά φτάνει 0.96 m στα μακρινά σημεία, άρα ο όρος όντως εφαρμόζεται.
Τα υπόλοιπα (init, σ 2.0, φίλτρο δαπέδου) αμετάβλητα.

#### Αποτέλεσμα

| (−10 ms) | ATE (m) | z RMSE (m) | μήκος (m) | RPE μετατ. | RPE στροφής |
|---|---|---|---|---|---|
| church_02, ομοιόμορφα (#031) | **0.130** | 0.121 | 269 | 0.034 | 0.71° |
| church_02, καμπύλη | 0.145 | 0.123 | 269 | 0.035 | 0.77° |
| christ-church-03, ομοιόμορφα (#031) | 0.038 | 0.029 | 281 | 0.021 | 0.66° |
| christ-church-03, καμπύλη | **0.036** | **0.023** | 281 | 0.021 | 0.72° |

#### Συμπέρασμα ⏳
1. **Καμία ουσιαστική διαφορά:** −0.002 m στη μία ακολουθία, +0.015 m στην άλλη — μέσα στο αναμενόμενο ενός run
   (η διασπορά δεν έχει μετρηθεί, §3).
2. **Γιατί, σε νούμερα:** η διαφορά των δύο μοντέλων για ένα σημείο στη στιγμή τ είναι ½α(τ²−τ). Με τα δικά μας μεγέθη
   (στροφή ~1.7°, επιτάχυνση ~1.6° ανά σάρωση²) αυτό είναι ≤ 0.2°, δηλαδή ~3 cm στα 10 m — κάτω από τον θόρυβο των
   αντιστοιχίσεων του ICP. Θα μετρούσε σε πολύ πιο βίαιη κίνηση.
3. Ο κώδικας μένει ως επιλογή (`curve`), η προεπιλογή παραμένει η ομοιόμορφη κατανομή. Σχήμα επεξήγησης:
   `docs/figures/deskew_interpolation.png`.

---

### 2026-09-19 — #032 keble-college-02: ο KISS καταρρέει, η μέθοδος τον σώζει

**git commit:** αυτή η εγγραφή · `scripts/run_sequence.sh keble-college-02` · runs `keble-college-02_{kiss,nodeskew,i3}` ·
δεδομένα `data/keble-college-02.bag` (5.1 GB, 3007 σαρώσεις, 300 s), `gt/keble-college-02_gt-tum.txt`

#### Στόχος
Γενίκευση σε άλλο κτήριο, χωρίς καμία αλλαγή ρύθμισης (η μέθοδος του #031: εικόνα ως deskew + αρχική θέση, σ 2.0· φίλτρο
δαπέδου 5 cm / −10° / 5 m· `configs/indoor_detail.yaml`).

#### Αποτέλεσμα
Κίνηση από την εικόνα: → runs/keble-college-02_i3_motion_car_sp_st0.05_floor.npz: επιτυχία 99.6 % (11 αποτυχίες), inliers διάμεσος 64, ~95 ms/σάρωση (μαζί με την ανάγνωση)

| keble-college-02 (−10 ms· «χωρίς deskew» −55 ms) | ATE (m) | z RMSE (m) | z span (GT 4.7) | μήκος (GT 294) | RPE μετατ. | RPE στροφής | closures |
|---|---|---|---|---|---|---|---|
| KISS | **2.133** | 1.621 | 9.0 | 532 | 0.197 | 2.38° | 0 |
| χωρίς deskew | 0.183 | 0.085 | — | 387 | 0.090 | 0.51° | 0 |
| **μέθοδος** | **0.094** | **0.038** | — | 350 | **0.051** | 0.86° | 0 |

**Σύνοψη των τριών ακολουθιών (ATE, m):**

| | KISS | χωρίς deskew | deskew με GT | **μέθοδος** | βελτίωση έναντι KISS |
|---|---|---|---|---|---|
| church_02 (2402 σαρ.) | 0.319 | 0.287 | 0.101 | **0.130** | −59 % |
| christ-church-03 (3123) | 0.122 | 0.125 | 0.066 | **0.038** | −69 % |
| keble-college-02 (3007) | 2.133 | 0.183 | — | **0.094** | −96 % |

#### Συμπέρασμα ⏳
1. **Η μέθοδος γενικεύει:** σε τρεις ακολουθίες, δύο κτήρια, με μία ρύθμιση, είναι παντού η καλύτερη, και στο keble σώζει
   μια ακολουθία που ο KISS δεν λύνει (2.13 → 0.09 m).
2. Το «χωρίς deskew» είναι κι αυτό πολύ καλύτερο από τον KISS στο keble (0.18) — επιβεβαιώνει ότι το deskew σταθερής
   ταχύτητας είναι ο κύριος εχθρός σε φορητή μονάδα· η εικόνα προσθέτει το υπόλοιπο (0.18 → 0.09).
3. Επιφυλάξεις: ένα run ανά ακολουθία (η διασπορά του ATE δεν έχει μετρηθεί)· τα κατώφλια του φίλτρου δαπέδου ορίστηκαν
   στο church_02· η μέθοδος δεν είναι ενσωματωμένη στο `slam.py` (precompute + script)· βιβλιογραφία δεν έχει ελεγχθεί.

---

### 2026-09-18 — #031 Εικόνα ως deskew και αρχική θέση, πλήρη runs: cc03 0.035, church_02 0.218 (σε εξέλιξη)

**git commit:** αυτή η εγγραφή · `scripts/run_i3_deskew.py … 0 init [adaptive|kiss|fixed]` · runs `christ-church-03_i3_init`,
`indoor_detail_i3_init`, `indoor_detail_i3_init_sig{kiss,fixed}` (σε εξέλιξη)

#### Στόχος
Η σχεδίαση του #030 (η κίνηση της εικόνας αντικαθιστά το `last_delta`: deskew **και** αρχική θέση του ICP) σε πλήρη runs.

#### Αποτέλεσμα

| (−10 ms) | ATE (m) | z RMSE | μήκος (m) | RPE μετατ. | RPE στροφής | closures | ATE ισόγειο / όροφος |
|---|---|---|---|---|---|---|---|
| **christ-church-03** KISS | 0.122 | 0.092 | 408 (GT 266) | 0.110 | 2.21° | 3 | — |
| christ-church-03 deskew από GT | 0.066 | 0.048 | 278 | 0.031 | 0.59° | 5 | — |
| **christ-church-03 εικόνα deskew + init** | **0.035** | **0.023** | 281 | **0.021** | 0.66° | 7 | — |
| **church_02** KISS | 0.319 | 0.200 | 398 (GT 253) | 0.170 | 2.41° | 2 | — |
| church_02 deskew από GT | 0.101 | 0.076 | 264 | 0.035 | 0.46° | 2 | 0.149 / 0.073 |
| church_02 εικόνα deskew μόνο (#027) | **0.122** | 0.101 | 281 | 0.054 | 0.82° | 2 | 0.163 / 0.105 |
| church_02 εικόνα deskew + init | 0.218 | 0.204 | 270 | **0.036** | **0.73°** | 2 | 0.295 / 0.161 |

church_02 με init: πεταγμένες σαρώσεις (> 0.12 m στο ύψος) διάδρομος 21 → 6, ισόγειο 500–820 26 → 1· ακμές odometry
0.10 m / 0.9° (KISS 0.44 / 2.7°)· closures σωστά (0.09, 0.21 m). Αλλά **αργή απόκλιση ύψους**: +0.17 m στο 0–400, +0.44 m στο
500–820, −0.20 στο τέλος. **σ (διάμεσος ανά 400 σαρώσεις): deskew μόνο 2.35–2.56 · με init 0.59–0.72.**

#### Συμπέρασμα ⏳ (ανοιχτό)
1. Στο christ-church-03 η σχεδίαση δίνει το καλύτερο αποτέλεσμα όλων, καλύτερο κι από το deskew με GT: **−71 % από τον KISS**.
2. Στο church_02 βελτιώνει όλα τα τοπικά μέτρα αλλά χειροτερεύει το ATE, μέσω αργής απόκλισης στο ύψος του ισογείου.
3. Υπόθεση: με καλή αρχική θέση, το προσαρμοστικό σ του KISS (που μετρά την απόκλιση αρχικής θέσης–αποτελέσματος)
   πέφτει 4×· ο ICP με στενό πυρήνα διορθώνει λιγότερο, και η μικρή συστηματική απόκλιση της κίνησης της εικόνας (κλίση
   +0.069°/σάρωση στο church_02, +0.039 στο cc03, #029) περνά στην τροχιά ως αργό drift ύψους. Δοκιμές: σ από την πρόβλεψη
   του KISS (`kiss`) ή σταθερό 2.0 (`fixed`), με init.

**Δοκιμές σ (church_02, init):**

| σ | σ (διάμεσος) | ATE (m) | z RMSE | μήκος | RPE μετατ. | RPE στροφής |
|---|---|---|---|---|---|---|
| adaptive (KISS, από την απόκλιση init–αποτέλεσμα) | 0.59–0.72 | 0.218 | 0.204 | 270 | 0.036 | 0.73° |
| kiss (απόκλιση έναντι της πρόβλεψης σταθερής ταχύτητας) | 2.4–2.6 | 0.146 | 0.138 | 269 | 0.034 | 0.70° |
| **fixed 2.0** | 2.0 | **0.130** | 0.121 | 269 | 0.034 | 0.71° |
| *(deskew μόνο, #027, για σύγκριση)* | 2.4–2.6 | 0.122 | 0.101 | 281 | 0.054 | 0.82° |

4. **Η υπόθεση του σ επιβεβαιώνεται εν μέρει:** με σ ~2 το ATE επανέρχεται στο 0.13 — σχεδόν όσο το «deskew μόνο», με
   σαφώς λιγότερο τρέμουλο. Ό,τι απομένει (0.13 έναντι 0.12, και ύψος 0.12 έναντι 0.10) μένει ανοιχτό. Επόμενο: το
   ίδιο σ στο christ-church-03 — αν κρατά το ~0.035, η ρύθμιση «init + σ σταθερό 2.0» γίνεται η μέθοδος.

**christ-church-03, init:** σ adaptive 0.035 · **σ fixed 2.0: 0.038** (z 0.029, RPE 0.021 m / 0.66°, 5 closures) · σ kiss 0.038.

5. **Επιλογή (19/9): εικόνα ως deskew + αρχική θέση, σ σταθερό 2.0** — church_02 0.130, christ-church-03 0.038. Μία
   ρύθμιση, χωρίς προθέρμανση ή άλλο μπάλωμα. Ενσωματώθηκε στο `run_sequence.sh`· keble-college-02 σε εξέλιξη.

---

### 2026-09-18 — #030 Η εύθραυστη εκκίνηση: deskew και αρχική θέση του ICP πρέπει να συμφωνούν

**git commit:** αυτή η εγγραφή · `scripts/replay_start.py` · `scripts/run_i3_deskew.py` (+ `init`) · runs
`christ-church-03_i3_init`, `indoor_detail_i3_init` (σε εξέλιξη)

#### Στόχος
Ερώτημα Λ.Γ.: «δεν βγάζει νόημα» — η κίνηση της εικόνας είναι πιο ακριβής από τη μαντεψιά του KISS και όμως η εκκίνηση
ξεφεύγει (#028), ενώ η προθέρμανση σώζει τη μία ακολουθία και βλάπτει την άλλη (#029). Τι φταίει;

#### Μέθοδος
Επανάληψη των πρώτων 60 σαρώσεων (indoor_detail) σε έξι εκδοχές: `kiss`· `gt` (deskew από GT, σάρωση 0 χωρίς)· `gt0` (και η
σάρωση 0 με την αληθινή κίνησή της → υπόθεση «στραβός αρχικός χάρτης»)· `i3` (εικόνα ως deskew, αρχική θέση ICP του
KISS — όπως τα runs)· `i3_0` (και η σάρωση 0 με την κίνηση της 1)· `i3_init` (εικόνα ως deskew ΚΑΙ ως αρχική θέση του ICP,
`last_delta := M` → υπόθεση «ασυνέπεια deskew–αρχικής θέσης»). Μέτρο: σωρευτικό σφάλμα προσανατολισμού έναντι GT.

#### Αποτέλεσμα

| σωρευτικό σφάλμα προσανατολισμού, max στις 60 σαρώσεις | christ-church-03 | church_02 | σ (cc03) |
|---|---|---|---|
| kiss | 3.3° | 3.4° | 1.18 |
| gt (deskew από GT) | 25.9° | 4.1° | 1.35 |
| gt0 (και η σάρωση 0) | 25.5° | 2.6° | 1.29 |
| i3 (εικόνα ως deskew, όπως τα runs) | **58.0°** | 2.6° | 1.55 |
| i3_0 (και η σάρωση 0) | 16.0° | 2.6° | 1.46 |
| **i3_init (εικόνα ως deskew ΚΑΙ αρχική θέση)** | **1.9°** | 4.5° | **0.64** |

#### Συμπέρασμα ⏳
1. **Αιτία: η ασυνέπεια ανάμεσα στο deskew και στην αρχική θέση του ICP.** Ο KISS δίνει στον ICP αρχική θέση
   `last_pose·last_delta` — την ίδια κίνηση με την οποία ίσιωσε τη σάρωση. Όταν η σάρωση ισιώνεται με άλλη κίνηση
   (εικόνα ή GT) αλλά ο ICP ξεκινά από τη μαντεψιά του KISS, σε άδειο χάρτη με μεγάλο κατώφλι (3σ ≈ 6 m) ο ICP γλιστρά,
   το σ φουσκώνει, και ο κύκλος αυτοενισχύεται. Με την ίδια κίνηση και στα δύο: η πιο σταθερή εκκίνηση όλων, με το
   μικρότερο σ.
2. Η σάρωση 0 χωρίς deskew συμβάλλει (i3 → i3_0: 58° → 16°) αλλά δεν είναι η κύρια αιτία (gt0 ≈ gt).
3. Στο church_02 η εκκίνηση είναι εύκολη (όλα ≤ 4.5°) — γι' αυτό η ασυνέπεια δεν φάνηκε ως τώρα. Το `i3_init` είναι εκεί
   ελαφρώς χειρότερο στις πρώτες 60 (4.5°)· αν κρατά στο πλήρες run, μένει.
4. **Η σωστή σχεδίαση:** η κίνηση της εικόνας αντικαθιστά το `last_delta` του KISS — deskew **και** αρχική θέση. Είναι
   η βελτίωση (γ) του §2.2, που είχε σημειωθεί ως δευτερεύουσα. Η προθέρμανση καταργείται.

---

### 2026-09-18 — #029 christ-church-03: deskew από GT και «προθέρμανση» — η αποτυχία ήταν η εκκίνηση

**git commit:** αυτή η εγγραφή · `scripts/run_oracle_deskew.py` (+ bag, gt, run) · `scripts/run_i3_deskew.py` (+ warmup) ·
runs `christ-church-03_oracle`, `christ-church-03_i3_warm50`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: να φανεί αν η αποτυχία του #028 οφείλεται στη μέθοδο ή στον τρόπο που δίνεται εξωτερικό deskew στον KISS.

#### Μέθοδος
1. Πλήρες run με deskew από την αληθινή κίνηση (GT, τ = 0).
2. Μικρή συστηματική απόκλιση της κίνησης της εικόνας (μέση τιμή του σφάλματος στροφής ανά άξονα).
3. «Προθέρμανση»: οι πρώτες 50 σαρώσεις με το deskew του ίδιου του KISS (last_delta), οι υπόλοιπες από την εικόνα.

#### Αποτέλεσμα

| christ-church-03 (−10 ms) | ATE (m) | z RMSE (m) | μήκος (GT 266) | RPE μετατ. | RPE στροφής | closures |
|---|---|---|---|---|---|---|
| KISS | 0.122 | 0.092 | 408 | 0.110 | 2.21° | 3 |
| εικόνα (#028) | 0.803 | 0.753 | 295 | 0.063 | 1.18° | 6 |
| **deskew από GT** | 0.066 | 0.048 | 278 | 0.031 | 0.59° | 5 |
| **εικόνα + προθέρμανση 50** | **0.053** | **0.047** | 285 | **0.026** | 0.70° | 6 |

Σταθερή απόκλιση στροφής της εικόνας (°/σάρωση, x/y/z): church_02 −0.011 / **+0.069** / −0.011· christ-church-03 −0.011 /
**+0.039** / −0.009 — μικρή, και μεγαλύτερη στο church_02, όπου η μέθοδος δούλεψε.

#### Συμπέρασμα ⏳
1. **Ο μηχανισμός είναι σωστός:** με την αληθινή κίνηση το ATE πέφτει στο μισό του KISS (0.066).
2. **Η αποτυχία του #028 ήταν η εκκίνηση:** με τον χάρτη σχεδόν άδειο, ο ICP ξεφεύγει και η τροχιά μένει στραβή για πάντα.
   Με 50 σαρώσεις προθέρμανσης: **0.053 m, −57 % από τον KISS** — καλύτερα κι από το deskew με GT (ένα run· εύλογα
   μέσα στη διασπορά).
3. Γιατί η εκκίνηση είναι εύθραυστη μόνο με εξωτερικό deskew (και με GT υπήρξε αστάθεια 7–15° στις πρώτες 60, που
   επανήλθε): ανοιχτό.
4. **church_02 με την ίδια προθέρμανση (run `indoor_detail_i3_warm50`): ATE 0.122 → 0.154, z 0.101 → 0.143, RPE 0.054 →
   0.065 m.** Το μπάλωμα σώζει τη μία ακολουθία και βλάπτει την άλλη → δεν είναι λύση· χρειάζεται η αιτία (#030).

---

### 2026-09-18 — #028 Επιβεβαίωση σε christ-church-03: η μέθοδος αποτυγχάνει (σε εξέλιξη)

**git commit:** αυτή η εγγραφή · `scripts/download_sequence.sh`, `scripts/run_sequence.sh` · runs `christ-church-03_{kiss,
nodeskew,i3,i3_vc}` · δεδομένα `data/christ-church-03.bag` (5.5 GB, 3123 σαρώσεις, 312 s), `gt/christ-church-03_gt-tum.txt`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: επιβεβαίωση του #027 σε άλλη ακολουθία, με τις ίδιες ακριβώς ρυθμίσεις.

#### Αποτέλεσμα

| (−10 ms· «χωρίς deskew» −55 ms) | ATE (m) | z RMSE (m) | μήκος (GT 266) | RPE μετατ. | RPE στροφής | closures |
|---|---|---|---|---|---|---|
| KISS | **0.122** | 0.092 | 408 | 0.110 | 2.21° | 3 (σωστά) |
| χωρίς deskew | 0.125 | 0.082 | 294 | 0.060 | 0.59° | 3 |
| μέθοδος (#027) | **0.803** | 0.753 | 295 | 0.063 | 1.18° | 6 |
| μέθοδος + έλεγχος ύψους 1 m | 0.803 | 0.753 | 295 | 0.063 | 1.18° | 6 (καμία απόρριψη) |

Διαγνώσεις:
1. **Η κίνηση από την εικόνα είναι σωστή:** κλίμακα 0.985, στροφή 0.56° (p90 1.12°), θέση 14 mm, καμία σάρωση > 5°·
   στις πρώτες 25 σαρώσεις 0.15–0.8° έναντι 0.6–3.9° της μαντεψιάς του KISS.
2. **Οι ενώσεις συμφωνούν με την odometry** (διαφωνία ύψους ≤ 0.16 m), αν και δύο απέχουν από το GT 5.4 και 1.6 m στο
   ύψος → το σφάλμα βρίσκεται στην ίδια την τροχιά, όχι στις ενώσεις.
3. **Όχι οι χρονοσφραγίδες:** ίδια δομή με το church_02 (διάρκεια 99.5–100 ms, ts.min = header, ίδια φορά περιστροφής).
4. **Προσανατολισμός:** από την αρχή η τροχιά απέχει ~60° από το GT στις σαρώσεις 0–100 (σωρευτικά 40° ήδη στη σάρωση
   20)· η κίνηση της εικόνας μόνη της, συντιθέμενη, απέχει μόλις 2° εκεί. Όμως και χωρίς τις πρώτες 100–200 σαρώσεις το
   ATE μένει 0.80 m.
5. **Επανάληψη των πρώτων 60 σαρώσεων** (σωρευτικό σφάλμα προσανατολισμού): KISS ≤ 2.7° · χωρίς deskew έως 40° ·
   **deskew από GT 7–15°** · εικόνα έως 53°. Στο church_02 το deskew από GT ήταν το καλύτερο.
6. Το script ανάλυσης ακμών (`check_closure_constraints.py`) δίνει «node 0→1: 16 m, 62°» — ύποπτο· η αντιστοίχιση
   node → σάρωση διορθώθηκε (μονότονη), το εύρημα έμεινε, αλλά το σφάλμα ανά σάρωση εκεί είναι κανονικό (0.9°).

#### Συμπέρασμα ⏳ (ανοιχτό)
1. **Η μέθοδος δεν γενικεύει όπως είναι:** στο christ-church-03 καταστρέφει μια τροχιά που ο KISS ήδη λύνει καλά.
2. Δεν φταίει η κίνηση από την εικόνα ούτε οι ενώσεις. Ύποπτο: η αλληλεπίδραση ενός **εξωτερικού** deskew με τον ICP
   του KISS σε αυτή την ακολουθία — και το deskew από GT δείχνει αστάθεια στις πρώτες σαρώσεις.
3. **Αποφασιστικό επόμενο:** πλήρες run με deskew από GT στο christ-church-03. Αν κι αυτό αποτύχει → το πρόβλημα είναι
   στον τρόπο που δίνουμε το deskew στον KISS (ή στο GT/χρονισμό της ακολουθίας), όχι στη μέθοδο· αν πετύχει → η
   κίνηση της εικόνας έχει έναν συγκεκριμένο τρόπο αποτυχίας που δεν βλέπουμε στα μέσα σφάλματα.

---

### 2026-09-18 — #027 Φίλτρο «ίδιο σημείο» μόνο στο κοντινό δάπεδο: ATE 0.122 m

**git commit:** αυτή η εγγραφή · `kiss_slam/intensity_deskew.py` (`STUCK_FLOOR_ONLY`, −10°, 5 m) ·
`precompute_i3_motion.py --model=car --subpixel --stuck=0.05 --floor-only` → `runs/i3_motion_car_sp_st0.05_floor.npz` ·
run `indoor_detail_i3car_sp_stfloor_deskew`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: το φίλτρο του #026 να εφαρμόζεται μόνο εκεί όπου είναι τα μοτίβα — κοντινό δάπεδο — ώστε να μην
πετά σωστές αντιστοιχίσεις (παράθυρα του ορόφου).

#### Μέθοδος
Απόρριψη αντιστοίχισης μόνο αν |p − q| < 5 cm **και** το σημείο είναι κάτω από −10° (πλαίσιο αισθητήρα) **και** πιο
κοντά από 5 m. Κατά τα άλλα όπως το #026.

#### Αποτέλεσμα
Κίνηση: κλίμακα 0.961, στροφή 0.64° (p90 1.30°), θέση 18 mm, 0 αποτυχίες, inliers 72.

| deskew (−10 ms) | ATE (m) | z RMSE (m) | μήκος (GT 253) | RPE μετατ. | RPE στροφής | ATE ισόγειο / όροφος |
|---|---|---|---|---|---|---|
| KISS | 0.319 | 0.200 | 398 | 0.170 | 2.41° | — |
| κανένα | 0.287 | 0.234 | 291 | 0.084 | 1.43° | — |
| εικόνα car + παρεμβολή (#021) | 0.255 | 0.241 | 279 | 0.053 | 0.84° | 0.411 / 0.092 |
| + «ίδιο σημείο» παντού (#026) | 0.190 | 0.175 | 283 | 0.062 | 0.89° | 0.274 / 0.145 |
| **+ «ίδιο σημείο» μόνο στο δάπεδο** | **0.122** | **0.101** | 281 | **0.054** | **0.82°** | **0.163 / 0.105** |
| αληθινή κίνηση (#012) | 0.101 | 0.076 | 264 | 0.035 | 0.46° | 0.149 / 0.073 |

#### Συμπέρασμα ⏳
1. **ATE 0.122 m μόνο με LiDAR: −62 % από το KISS, και μόλις 0.02 m πάνω από το deskew με την αληθινή κίνηση.**
   Σφάλμα ύψους 0.101 m (KISS 0.200).
2. Το στοχευμένο φίλτρο κρατά τη βελτίωση του ισογείου (0.163) χωρίς το κόστος στον όροφο (0.105). Επιβεβαιώνει τον
   μηχανισμό του #024–#026: το πρόβλημα ήταν τα μοτίβα intensity του κοντινού δαπέδου.
3. Ένα run· τα κατώφλια (5 cm, −10°, 5 m) ορίστηκαν από την ανάλυση του ίδιου dataset — χρειάζεται επιβεβαίωση σε
   άλλη ακολουθία του Oxford Spires πριν από οποιονδήποτε ισχυρισμό.

---

### 2026-09-18 — #026 Deskew από την εικόνα με το φίλτρο «ίδιο σημείο» μέσα στο SLAM

**git commit:** αυτή η εγγραφή · `kiss_slam/intensity_deskew.py` (`STUCK_MIN`) ·
`precompute_i3_motion.py --model=car --subpixel --stuck=0.05` → `runs/i3_motion_car_sp_st0.05.npz` ·
`run_i3_deskew.py … runs/i3_motion_car_sp_st0.05.npz` · run `indoor_detail_i3car_sp_st_deskew` ·
σχήμα `scripts/figure_stuck_patterns.py` → `docs/figures/i3_stuck_patterns.png`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: το φίλτρο του #025 στο deskew του SLAM.

#### Αποτέλεσμα
Κίνηση σε όλο το bag: κλίμακα **0.919 → 0.980**, στροφή 0.65° (p90 1.31°), θέση 19 mm, 1 αποτυχία / 2402, inliers 68.

| deskew (−10 ms) | ATE (m) | z RMSE (m) | μήκος (GT 253) | RPE μετατ. | RPE στροφής | ATE ισόγειο / όροφος | πεταγμένες (400–500 / όλες) |
|---|---|---|---|---|---|---|---|
| κανένα | 0.287 | 0.234 | 291 | 0.084 | 1.43° | — | — |
| εικόνα cv (#019) | 0.258 | 0.179 | 295 | 0.069 | 1.07° | — | — |
| εικόνα car + παρεμβολή (#021) | 0.255 | 0.241 | 279 | **0.053** | **0.84°** | 0.411 / **0.092** | 27 / 35 |
| **+ φίλτρο «ίδιο σημείο»** | **0.190** | **0.175** | 283 | 0.062 | 0.89° | **0.274** / 0.145 | 33 / 71 |
| αληθινή κίνηση (#012) | 0.101 | 0.076 | 264 | 0.035 | 0.46° | 0.149 / 0.073 | 15 / 24 |

![«Κολλημένες» αντιστοιχίσεις στα πανοράματα](figures/i3_stuck_patterns.png)

Στο σχήμα: στο ισόγειο και στον διάδρομο οι «κολλημένες» (κόκκινο) είναι σχεδόν όλες στις κάτω γραμμές — το δάπεδο
1–3 m από τον σαρωτή· οι κοντινοί τοίχοι είναι κορεσμένοι (λευκοί, χωρίς σχέδιο). Στον όροφο υπάρχουν κόκκινα και πάνω
στα παράθυρα. Δεν φαίνονται κυριολεκτικοί «κύκλοι» — ο όρος ήταν υπόθεσή μου.

#### Συμπέρασμα ⏳
1. **ATE 0.190 m: η καλύτερη τροχιά μόνο με LiDAR** — −40 % από το KISS, −33 % από το «χωρίς deskew», σφάλμα ύψους
   0.175 m. Ένα run.
2. **Όμως δεν βελτιώνει παντού:** το ισόγειο βελτιώνεται πολύ (0.41 → 0.27), ο όροφος χειροτερεύει (0.09 → 0.15) και το
   τρέμουλο αυξάνεται λίγο. Εύλογη αιτία: όταν στροφή και μετατόπιση αλληλοαναιρούνται για κάποια σημεία, αυτά μένουν
   «ακίνητα» ως προς τον σαρωτή και το φίλτρο τα πετά (κόκκινα στα παράθυρα· κλίμακα ορόφου 1.03 στο #025).
3. **Επόμενο:** φίλτρο μόνο εκεί όπου βρίσκεται το πρόβλημα — κοντινό δάπεδο (κάτω γραμμές, μικρή απόσταση).

---

### 2026-09-18 — #025 Βαθμονόμηση του intensity και φίλτρο «ίδιο σημείο»

**git commit:** αυτή η εγγραφή · `scripts/calibrate_intensity.py` → `runs/intensity_calib.npz`,
`docs/figures/intensity_calibration.png` · `kiss_slam/intensity_deskew.py` (`grid`, `incidence_cos`, `calibrated`,
`CALIB`) · `scripts/test_i3_scale_bias.py --calib`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: να αφαιρεθεί η εξάρτηση του intensity από απόσταση και γωνία πρόσπτωσης, ώστε να σβήσουν τα μοτίβα
που ακολουθούν τον σαρωτή (#024) και η κλίμακα της μετατόπισης να φτάσει 1.00.

#### Μέθοδος
1. **Βαθμονόμηση από τα δεδομένα:** κάθε 20ή σάρωση (121 σαρώσεις, 3.4 εκατ. σημεία). Γωνία πρόσπτωσης από κάθετες PCA
   των 10 πλησιέστερων 3D γειτόνων (Open3D) — οι γείτονες του πλέγματος δεν δουλεύουν: κάποιοι δακτύλιοι έχουν μόνο ~250
   επιστροφές ανά περιστροφή, 42 % των pixel έγκυρα. Πίνακας διάμεσου intensity ανά 24 κελιά απόστασης (1–50 m, λογ.)
   × 10 κελιά |cos|· διορθωμένο = intensity / διάμεσος του κελιού.
2. **Φίλτρο «ίδιο σημείο»:** απόρριψη αντιστοιχίσεων με |p − q| (στο πλαίσιο του αισθητήρα) κάτω από 3 ή 5 cm — μια
   πραγματική αντιστοίχιση μετακινείται περίπου όσο ο αισθητήρας (~10 cm ανά σάρωση).

#### Αποτέλεσμα
Εξάρτηση του intensity (διάμεσος): απόσταση 1.1 m: 24 · 2.9 m: **229** · 5.5 m: 163 · 7.7 m: 67 · 10.6 m: 34 · 39 m: 23.
|cos πρόσπτωσης| (2–6 m): 0.15: **106** · 0.45: 199 · 0.95: **235**.

![Βαθμονόμηση intensity](figures/intensity_calibration.png)

| (κλίμακα / στροφή / θέση / επιτυχία) | ισόγειο 300–400 | διάδρομος 400–500 | όροφος 1200–1300 |
|---|---|---|---|
| σημερινό | 0.815 / 0.77° / 45 mm / 100 % | 0.793 / 0.88° / 35 mm / 100 % | 0.896 / 0.50° / 13 mm / 100 % |
| βαθμονομημένο intensity | 0.637 / 1.61° / 78 mm / 66 % | 0.638 / 1.10° / 60 mm / 80 % | 0.901 / 0.56° / 16 mm / 100 % |
| > 4 m (#024) | 0.965 / 0.83° / 47 mm / 96 % | 0.941 / 0.84° / 36 mm / 92 % | 0.983 / 0.60° / 15 mm / 100 % |
| όχι «ίδιο σημείο», 3 cm | 0.890 / 0.71° / 39 mm / 100 % | 0.920 / 0.84° / 27 mm / 100 % | 0.973 / 0.46° / 12 mm / 100 % |
| **όχι «ίδιο σημείο», 5 cm** | **0.940 / 0.72° / 34 mm / 98 %** | **0.923 / 0.71° / 28 mm / 100 %** | **1.027 / 0.48° / 13 mm / 100 %** |
| όχι «ίδιο σημείο» 5 cm, > 3 m | 0.982 / 0.71° / 38 mm / 98 % | 0.922 / 0.74° / 30 mm / 100 % | 1.047 / 0.49° / 15 mm / 100 % |

(Με βαθμονόμηση οι αντιστοιχίσεις ανά ζεύγος πέφτουν στο ισόγειο 93 → 44, στον διάδρομο 111 → 52.)

#### Συμπέρασμα ⏳
1. **Το intensity εξαρτάται πολύ από τη γεωμετρία** — ×10 με την απόσταση, ×2 με τη γωνία — επιβεβαιώνοντας τον
   μηχανισμό του #024.
2. **Η απλή βαθμονόμηση με πίνακα χειροτερεύει την αντιστοίχιση** στο ισόγειο. Πιθανές αιτίες (όχι αποδεδειγμένες):
   τα όρια των κελιών απόστασης δημιουργούν νέα ομόκεντρα «σκαλοπάτια»· θορυβώδεις κάθετες· αφαιρεί και πραγματικό
   σχέδιο. Μια ομαλή (παραμετρική) βαθμονόμηση θα απέφευγε το πρώτο.
3. **Το φίλτρο «ίδιο σημείο» (5 cm) είναι το καλύτερο μέχρι τώρα:** κλίμακα 0.92–1.03, μικρότερο σφάλμα στροφής και
   θέσης παντού, επιτυχία 98–100 %. Ο διάδρομος μένει 8 % κοντύτερος — υπάρχει και δεύτερη πηγή εκεί.

---

### 2026-09-18 — #024 Η αιτία της υποεκτίμησης: μοτίβα intensity που ακολουθούν τον σαρωτή

**git commit:** αυτή η εγγραφή · `scripts/test_i3_scale_bias.py` (+ `--bearing=`) · `kiss_slam/intensity_deskew.py`
(`INLIER_TEST = "metric" | "bearing"`, default αμετάβλητο)

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: να βρεθεί και να διορθωθεί η υποεκτίμηση της μετατόπισης της εικόνας (#023). Κριτήριο: κλίμακα ≈ 1.00.

#### Μέθοδος
Παράθυρα: ισόγειο 300–400, διάδρομος 400–500, όροφος 1200–1300, κάθε 2η σάρωση. Για κάθε αντιστοίχιση: απόσταση, γωνία
ύψους, μετακίνηση στην εικόνα (px) έναντι της αναμενόμενης από την αληθινή κίνηση, κλίμακα (p − R_gt q)·t_gt / |t_gt|².
Εκτίμηση με διάφορα φίλτρα (ίδιος αλγόριθμος: RANSAC + car + παρεμβολή)· και σύγκριση άκαμπτης λύσης / cv / car.

#### Αποτέλεσμα
1. **Όχι το χρονικό μοντέλο:** κλίμακα στον διάδρομο — άκαμπτη χωρίς χρόνο 0.647, cv 0.720, car 0.793 (όροφος 0.90 / 0.91 / 0.90).
2. **«Κολλημένες» αντιστοιχίσεις** (< 1 px μετακίνηση στην εικόνα):

| | κοντινές < 4 m: κολλημένες | κλίμακά τους | έπρεπε να μετακινηθούν | υπόλοιπες κοντινές | μακρινές (όλες) |
|---|---|---|---|---|---|
| διάδρομος | 26 % | **+0.13** | 4.1 px | 0.62 | ~0.85 |
| όροφος | 23 % | **+0.09** | 4.5 px | 0.75 | 0.84–1.08 |

3. **Φίλτρα** (κλίμακα / επιτυχία):

| φίλτρο | ισόγειο 300–400 | διάδρομος 400–500 | όροφος 1200–1300 |
|---|---|---|---|
| όλες (σημερινό) | 0.815 / 100 % | 0.793 / 100 % | 0.896 / 100 % |
| απόσταση > 3 m | 0.812 / 100 % | 0.810 / 100 % | 0.950 / 100 % |
| **απόσταση > 4 m** | **0.965 / 96 %** | **0.941 / 92 %** | **0.983 / 100 %** |
| απόσταση > 5 m | 0.939 / 74 % | 0.907 / 82 % | 0.977 / 100 % |
| όχι κάτω από −10° (δάπεδο) | 0.937 / 58 % | 0.899 / 88 % | 0.984 / 100 % |
| ακμές βάθους (5 %) εκτός | 0.839 / 14 % | 0.819 / 28 % | 0.969 / 100 % |
| έλεγχος κατά διεύθυνση 0.35° | 0.729 / 84 % | 0.633 / 94 % | 0.937 / 100 % |
| έλεγχος κατά διεύθυνση 0.7° | 0.842 / 98 % | 0.675 / 98 % | 0.951 / 100 % |

Στο ισόγειο 60–67 % των αντιστοιχίσεων κοιτούν κάτω από −10° (δάπεδο)· στον όροφο 31 %.

#### Συμπέρασμα ⏳
1. **Μηχανισμός:** το intensity εξαρτάται από την απόσταση και τη γωνία πρόσπτωσης. Σε κοντινές επιφάνειες — κυρίως στο
   δάπεδο γύρω από τον χειριστή — αυτό δημιουργεί μοτίβα που **κινούνται μαζί με τον σαρωτή**. Το SIFT τα αντιστοιχίζει
   στο ίδιο pixel, δείχνουν μηδενική κίνηση (κλίμακα ~0.1) και, επειδή η κίνηση ανά σάρωση (~11 cm) είναι της τάξης
   του κατωφλίου, περνούν ως σωστές.
2. **Ο έλεγχος κατά διεύθυνση χειροτερεύει:** οι κολλημένες αντιστοιχίσεις συμφωνούν μεταξύ τους στη «μηδενική κίνηση»·
   με αυστηρό κατώφλι αυτή η ομάδα κερδίζει συχνότερα.
3. **Κανένα απλό φίλτρο δεν δίνει 1.00 στον διάδρομο.** Το καλύτερο — μόνο σημεία > 4 m — δίνει 0.94–0.98. Για
   συνδυασμό με τον ICP στον διάδρομο (#023) ένα 6 % σε 10 m είναι ακόμη 0.6 m.
4. **Λύση στην πηγή:** βαθμονόμηση του intensity ως προς απόσταση και γωνία πρόσπτωσης, ώστε η εικόνα να δείχνει την
   ανακλαστικότητα του υλικού και όχι τη γεωμετρία. Είναι και θέμα της διατριβής (το intensity ως ιδιότητα του υλικού).

---

### 2026-09-18 — #023 Εκφύλιση του ICP + κίνηση της εικόνας: σωστή διάγνωση, κατάρρευση στο SLAM

**git commit:** αυτή η εγγραφή · `scripts/test_i4_degeneracy_fusion.py` · `scripts/run_i4_fusion.py` ·
`precompute_i3_motion.py --ransac= --fit=` · runs: `indoor_detail_i4_tau0.1`, `indoor_detail_i4_tau1.01`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: στις κατευθύνσεις όπου η γεωμετρία δεν δεσμεύει τον ICP, να παίρνεται η κίνηση από την εικόνα.

#### Μέθοδος
Πίνακας πληροφορίας point-to-plane της μετατόπισης H = mean(n nᵀ) από τις κάθετες της σάρωσης (voxel 0.25 m, PCA).
Η μετατόπιση ICP αναλύεται στις ιδιοκατευθύνσεις του H· όπου ιδιοτιμή < τ, η συνιστώσα αντικαθίσταται από την κίνηση
της εικόνας (car + παρεμβολή). Η στροφή μένει του ICP. Πρώτα εκτός SLAM (ICP = run `i3car_sp_deskew`), μετά στο SLAM.

#### Αποτέλεσμα
**Εκτός SLAM** — ελάχιστη ιδιοτιμή (διάμεσος): διάδρομος **0.057** (κατεύθυνση: κατά μήκος, |cos| 0.99)· αλλού
0.17–0.23 (ισόγειο: κατά μήκος· σκάλα, όροφος: κατακόρυφα). Σφάλμα μετατόπισης ανά σάρωση (RMS, κατά μήκος / πλάγια /
κατακόρυφα, mm):

| | ICP | τ = 0.10 | τ = 0.15 | εικόνα |
|---|---|---|---|---|
| ισόγειο 300–400 | 57 / 47 / 47 | 53 / 47 / 48 | 48 / 47 / 48 | 54 / 25 / 25 |
| **διάδρομος 400–500** | **116** / 61 / 120 | **49** / 62 / 119 | 42 / 63 / 119 | 41 / 19 / 18 |
| ισόγειο 500–600 | 77 / 57 / 58 | 77 / 57 / 58 | 60 / 57 / 58 | 39 / 15 / 16 |
| σκάλα πάνω | 16 / 12 / 21 | ίδιο | ίδιο | 14 / 13 / 18 |
| όροφος | 12 / 15 / 20 | ίδιο | ίδιο | 14 / 5 / 9 |

Το κατακόρυφο σφάλμα του ICP στον διάδρομο (120 mm) **δεν** εμφανίζεται ως γεωμετρική εκφύλιση.

**Μέσα στο SLAM:**

| | σαρώσεις διορθωμένες | closures | ATE (m) | μήκος (m, GT 253) |
|---|---|---|---|---|
| deskew εικόνας (#021) | — | 2 | 0.255 | 279 |
| + τ = 0.10 | 95 | 2 | **5.01** | 281 |
| + όλη η μετατόπιση από την εικόνα | 2401 | 1 | **12.14** | 228 |

**Αιτία — κλίμακα της μετατόπισης** (προβολή στην αληθινή κίνηση, διάμεσος): εικόνα **0.919** (διάδρομος 0.84,
όροφος 0.92)· ICP 1.003. Άθροισμα μήκους: GT 253 m, εικόνα 228 m, ICP 279 m. Ανά αντιστοίχιση, ανάλογα με την
απόσταση του σημείου:

| απόσταση | διάδρομος: κλίμακα (ποσοστό ≈ 0) | όροφος: κλίμακα (ποσοστό ≈ 0) |
|---|---|---|
| 2–4 m | 0.47 (22 %) | 0.58 (21 %) |
| 4–8 m | 0.83 (12 %) | 0.98 (11 %) |
| 8–16 m | 0.84 (10 %) | 1.06 (10 %) |

Αυστηρότερα κατώφλια απόρριψης (RANSAC / fit 0.10 / 0.04 και 0.05 / 0.03 m, από 0.30 / 0.10): κλίμακα 0.931 / 0.915,
επιτυχία 82 / 78 %, στροφή χειρότερη (0.76° / 0.87°) — δεν λύνουν το πρόβλημα.

#### Συμπέρασμα ⏳
1. **Η διάγνωση δουλεύει:** η ανάλυση εκφύλισης εντοπίζει τον διάδρομο και την «τυφλή» κατεύθυνσή του (κατά μήκος).
2. **Η μετατόπιση από την εικόνα έχει συστηματική υποεκτίμηση ~8 % (16 % στον διάδρομο).** Το RMS ανά σάρωση την
   έκρυβε· μέσα στο SLAM αθροίζεται και η τροχιά καταρρέει. Η στροφή της εικόνας δεν έχει τέτοιο πρόβλημα (#021).
3. Προέρχεται κυρίως από κοντινές αντιστοιχίσεις (< 4 m: κλίμακα ~0.5, 1 στις 5 χωρίς καμία κίνηση — πιθανώς
   σημεία σταθερά στην εικόνα, π.χ. ακμές της σκιάς του χειριστή, ή ακμές βάθους). Όχι απλώς θέμα κατωφλίων.
4. **Επόμενο:** εύρεση της αιτίας (κοντινά / σταθερά στην εικόνα / ακμές βάθους) και διόρθωση, πριν από κάθε
   συνδυασμό με τον ICP. Ως τότε η εικόνα χρησιμοποιείται μόνο για το deskew (#021), όπου η μετατόπιση μετρά λίγο.

---

### 2026-09-18 — #022 Πού ξεφεύγει το ύψος; Ένας διάδρομος, όχι η αλλαγή ορόφου

**git commit:** αυτή η εγγραφή · `scripts/analyze_height_error.py` · σχήμα `docs/figures/height_error_ground_floor.png`

#### Στόχος
Υπόθεση Λ.Γ.: το σφάλμα ύψους του #021 (z RMSE 0.241) οφείλεται στην αλλαγή ορόφου. Πού ακριβώς ξεφεύγει το ύψος;

#### Μέθοδος
Σφάλμα ύψους ανά σάρωση μετά από ευθυγράμμιση Umeyama (car + παρεμβολή, χωρίς deskew, deskew από GT)· μεταβολή ανά
τμήμα και ανά παράθυρο 25 σαρώσεων· «πεταγμένες» σαρώσεις (|Δz| > 0.12 m) ανά 100· έλεγχος κλίσης/κλίμακας (παλινδρόμηση
του σφάλματος στο x, y και στο ύψος GT)· έλεγχος ραφών στις αρχές των nodes· γεωμετρικά διαγνωστικά από το
`icp_metrics.csv`· ανάλυση του σφάλματος κίνησης ανά σάρωση κατά μήκος / πλάγια / κατακόρυφα της διαδρομής.

#### Αποτέλεσμα
1. **Όχι η σκάλα:** μεταβολή του σφάλματος ύψους μέσα σε κάθε σκάλα ≤ 0.10 m (όλα τα runs).
2. **Όχι κλίση ή κλίμακα:** εξηγούν ≈ 0 του σφάλματος (κλίση επιπέδου ≤ 0.05°, κλίμακα ύψους ≤ 2 %). **Όχι ραφές** χαρτών.
3. **Μεμονωμένες σαρώσεις που πετάγονται 0.2–0.6 m και επιστρέφουν**, συγκεντρωμένες στις σαρώσεις 400–500:

| > 0.12 m ανά 100 σαρώσεις | 0 | 100 | 200 | 300 | **400** | 500 | 600–2300 | σύνολο |
|---|---|---|---|---|---|---|---|---|
| KISS | 13 | 25 | 13 | 12 | 10 | 13 | 280 | 366 |
| χωρίς deskew | 4 | 0 | 1 | 1 | 9 | 7 | 11 | 33 |
| εικόνα car | 2 | 0 | 0 | 2 | **27** | 4 | 0 | 35 |
| deskew από GT | 2 | 0 | 0 | 0 | **15** | 5 | 2 | 24 |

4. **Γεωμετρία (δείκτες του σημερινού pipeline, διάμεσος ανά 100 σαρώσεις):** στις 400–500 γραμμικότητα **0.90** (η
   υψηλότερη), επιπεδότητα 0.06, σφαιρικότητα 0.04, δείκτης κατάστασης **23.8** (όροφος 3.6–3.8)· διαδρομή ευθεία,
   10.7 × 0.4 m → **διάδρομος**. Ταβάνι και έδαφος ορατά κανονικά — δεν είναι εξωτερικός χώρος.
5. **Σφάλμα κίνησης ανά σάρωση (RMS, mm), κατά μήκος / πλάγια / κατακόρυφα:**

| σαρώσεις | ICP με deskew από GT | εικόνα intensity (Ι-3, car + παρεμβολή) |
|---|---|---|
| 300–400 | 28 / 15 / 14 | 54 / 25 / 27 |
| **400–500 (διάδρομος)** | **93 / 33 / 83** | **41 / 19 / 18** |
| 500–600 | 46 / 30 / 62 | 39 / 16 / 17 |
| 1200–1400 (όροφος) | 9 / 7 / 10 | 14 / 5 / 10 |

![Σφάλμα ύψους και περιβάλλον στο ισόγειο](figures/height_error_ground_floor.png)

#### Συμπέρασμα ⏳
1. **Το σφάλμα ύψους του ισογείου προέρχεται από έναν διάδρομο (400–500), όχι από την αλλαγή ορόφου.** Η υπόθεση
   του Λ.Γ. ισχύει για τις ενώσεις χαρτών (#015), όχι για το ATE αυτού του dataset.
2. **Ο διάδρομος είναι γεωμετρικά degenerate για τον ICP ακόμη και με τέλειο deskew** (σφάλμα 7–10× του ορόφου,
   κατά μήκος και κατακόρυφα). Αφού διορθώθηκε το deskew, η εκφύλιση **φαίνεται** — όπως προέβλεπε το §2.5.
3. **Εκεί η εικόνα intensity μετρά την κίνηση 2–4× καλύτερα από τον ICP**, ενώ στον όροφο ο ICP είναι καλύτερος. Είναι
   το αρχικό concept της διατριβής — intensity εκεί όπου η γεωμετρία εκφυλίζεται — με τον σωστό μηχανισμό:
   όχι φιλτράρισμα σημείων, αλλά ανεξάρτητη μέτρηση της κίνησης από χαρακτηριστικά της εικόνας.
4. Επόμενο: ανίχνευση της εκφύλισης και συνδυασμός της κίνησης της εικόνας με τον ICP στις κατευθύνσεις που ο ICP
   δεν δεσμεύει.

**Έλεγχος της υπόθεσης με run:** deskew από την εικόνα (car + παρεμβολή) + κατάτμηση στο ύψος 1.5 m + top-5 + έλεγχος
ύψους 1 m (`runs/indoor_detail_lc_topk5_h1.5_vc1.0.yaml`, run `indoor_detail_i3car_sp_lc`): 15 closures, καμία απόρριψη,
ATE 0.258 m (μόνο deskew: 0.255), z RMSE 0.244 (0.241). Καμία αλλαγή — συνεπές με το συμπέρασμα 1.

---

### 2026-09-18 — #021 Ι-3: επιτάχυνση μόνο στη στροφή + παρεμβολή μέσα στο pixel

**git commit:** αυτή η εγγραφή · `kiss_slam/intensity_deskew.py` (μοντέλο `car`, `lookup(subpixel=True)`) ·
`precompute_i3_motion.py --model=car --subpixel` · run: `indoor_detail_i3car_sp_deskew`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: (1) να κρατηθεί η βελτίωση της στροφής του `ca` χωρίς το κόστος στη μετατόπιση· (2) να μειωθεί το
σφάλμα κβάντισης της αντιστοίχισης (το 3D σημείο του πλησιέστερου pixel: στήλη 0.35°, δακτύλιος 0.5–1°).

#### Μέθοδος
`car`: [R(t w + t²/2 α), t v] — 9 άγνωστοι. Παρεμβολή: διγραμμική, σημείου και χρόνου, από τα 4 γειτονικά pixel, μόνο
όταν είναι όλα έγκυρα και στην ίδια επιφάνεια (αποστάσεις εντός 5 %)· αλλιώς το πλησιέστερο pixel. SLAM όπως στο #019.

#### Αποτέλεσμα
Κίνηση έναντι GT, όλο το bag (διάμεσος στροφή / θέση): cv 0.92° / 28 mm · cv + παρεμβολή 0.90° / 27 mm · ca 0.70° /
34 mm · car 0.66° / 21 mm · **car + παρεμβολή 0.65° / 20 mm** (90ό εκατ. 1.32°). Επιτυχία 100 %, inliers 78.

| deskew | ATE (m) | z RMSE (m) | μήκος (m, GT 253) | RPE μετατ. (m) | RPE στροφής |
|---|---|---|---|---|---|
| κανένα (−55 ms) | 0.283 | 0.233 | 291 | 0.086 | 0.87° |
| εικόνα cv (#019) | 0.258 | **0.179** | 295 | 0.069 | 1.07° |
| εικόνα ca (#020) | 0.388 | 0.328 | 297 | 0.081 | 0.99° |
| **εικόνα car + παρεμβολή** | **0.255** | 0.241 | **279** | **0.053** | **0.84°** |
| αληθινή κίνηση (#012) | 0.101 | 0.076 | 264 | 0.035 | 0.46° |

(−10 ms εκτός από το «κανένα»· 2 closures σε όλα.)

#### Συμπέρασμα ⏳
1. **Η επιτάχυνση μόνο στη στροφή είναι το σωστό μοντέλο:** βελτιώνει στροφή **και** θέση (−29 %). Η παρεμβολή
   μέσα στο pixel βοηθά ελάχιστα — δεν ήταν το όριο.
2. **Τα τοπικά μέτρα ακολουθούν την ακρίβεια της κίνησης:** RPE μετατόπισης −23 %, RPE στροφής −22 % (για πρώτη
   φορά καλύτερο από το «χωρίς deskew»), τρέμουλο (μήκος) 295 → 279 m.
3. **Το ATE δεν ακολουθεί** (0.258 → 0.255) και το σφάλμα ύψους χειροτερεύει (0.179 → 0.241). Το ATE κυριαρχείται
   από σωρευτική μετατόπιση — κυρίως στο ύψος — που εξαρτάται από λίγα γεγονότα και δεν μειώνεται μονότονα με την
   ακρίβεια ανά σάρωση. Ένα run ανά εκδοχή: οι διαφορές ATE ±0.01–0.06 m ίσως είναι μέσα στη διασπορά.
4. Επόμενα: (i) συνδυασμός με το loop closure του #016 (κατάτμηση στο ύψος + έλεγχος), που στοχεύει τη σωρευτική
   μετατόπιση· (ii) εκτίμηση της διασποράς του ATE (π.χ. runs με μετατοπισμένη αρχή)· (iii) RoMa.

---

### 2026-09-18 — #020 Ι-3 με ταχύτητα που αλλάζει (σταθερή επιτάχυνση)

**git commit:** αυτή η εγγραφή · `kiss_slam/intensity_deskew.py` (μοντέλο `ca`) ·
`scripts/precompute_i3_motion.py --model=ca` · `scripts/run_i3_deskew.py … runs/i3_motion_ca.npz` · run: `indoor_detail_i3ca_deskew`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9 (§2.2α): η στροφή αλλάζει από σάρωση σε σάρωση σχεδόν όσο είναι η ίδια (#019)· να επιτραπεί στην
εκτίμηση της εικόνας ταχύτητα που αλλάζει μέσα στο ζεύγος των σαρώσεων.

#### Μέθοδος
Χρόνος σε περιόδους από την αρχή της τρέχουσας σάρωσης, pose(0) = I. `cv`: [R(t w), t v]· `ca`: [R(t w + t²/2 α),
t v + t²/2 a] (12 άγνωστοι, αρχική τιμή η λύση `cv`). Κίνηση της τρέχουσας σάρωσης = pose(1). Η αλλαγή αρχής του
χρόνου (από το πρώτο ζεύγος στην αρχή της σάρωσης) αφήνει το `cv` ίδιο: 0.92°, 28 mm, όπως στο #019.

#### Αποτέλεσμα

| μοντέλο | στροφή p50 / p90 / p99 | θέση p50 / p90 / p99 | inliers | ATE (m) | z RMSE (m) | RPE στροφής |
|---|---|---|---|---|---|---|
| cv (#019) | 0.92 / 1.73 / 2.80° | 28 / 62 / 109 mm | 73 | **0.258** | **0.179** | 1.07° |
| ca | **0.70 / 1.43 / 2.57°** | 34 / 82 / 153 mm | 79 | 0.388 | 0.328 | 0.99° |

(ATE στα −10 ms· 2 closures και στα δύο.)

#### Συμπέρασμα ⏳
1. Η ταχύτητα που αλλάζει βελτιώνει τη στροφή σε όλα τα εκατοστημόρια (−24 % στον διάμεσο), όπως αναμενόταν από
   το #019.
2. Όμως οι επιπλέον 6 άγνωστοι της μετατόπισης αυξάνουν το σφάλμα της (p99 +40 %) και η τροχιά χειροτερεύει πολύ
   (ATE +50 %, σφάλμα ύψους σχεδόν διπλάσιο). Εύλογο: το σφάλμα μετατόπισης στο deskew μετρά περισσότερο απ' όσο
   υπέθετα· δεν αποκλείεται όμως και ευαισθησία ενός μόνο run.
3. Επόμενο: επιτάχυνση **μόνο στη στροφή** (9 άγνωστοι) — κρατά τη μετατόπιση του `cv`.

---

### 2026-09-18 — #019 Ι-3 μέσα στο SLAM: deskew με την κίνηση από την εικόνα intensity

**git commit:** αυτή η εγγραφή · `kiss_slam/intensity_deskew.py` · `scripts/precompute_i3_motion.py` (~55 ms/σάρωση) ·
`scripts/run_i3_deskew.py` · run: `indoor_detail_i3_deskew`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: μετά το #018, πόσο βελτιώνει την τροχιά το deskew με την κίνηση από την εικόνα;

#### Μέθοδος
1. `ScanMotionEstimator` (ίδιος αλγόριθμος με το #018, τώρα module): για κάθε σάρωση k, κίνηση από το ζεύγος (k−1, k)
   — αιτιακά, χωρίς GT. Υπολογίστηκε για όλες τις σαρώσεις → `runs/i3_motion.npz`.
2. SLAM `indoor_detail`, όπως το deskew από GT του #012, με μία αλλαγή: το deskew της σάρωσης k παίρνει την κίνηση
   της εικόνας (ταυτοτική όπου αποτύχει — δεν χρειάστηκε). Όλα τα άλλα upstream.
3. Αξιολόγηση σε −70 / −55 / −10 ms· κάθε μέθοδος αναφέρεται στη δική της καλύτερη.

#### Αποτέλεσμα
Κίνηση: επιτυχία 2402/2402, inliers διάμεσος 72, σφάλμα στροφής έναντι GT 0.92° (90ό εκατ. 1.73°)· μέγεθος της
κίνησης 1.68°.

| deskew | μετατόπιση | ATE (m) | z RMSE (m) | μήκος (m, GT 253) | RPE μετατ. (m) | RPE στροφής |
|---|---|---|---|---|---|---|
| μαντεψιά KISS | −10 ms | 0.319 | 0.200 | 398 | 0.170 | 2.41° |
| κανένα | −55 ms | 0.283 | 0.233 | 291 | 0.086 | **0.87°** |
| **εικόνα intensity (Ι-3)** | −10 ms | **0.258** | **0.179** | 295 | **0.069** | 1.07° |
| αληθινή κίνηση (GT, #012) | −10 ms | 0.101 | 0.076 | 264 | 0.035 | 0.46° |

Εξομάλυνση της κίνησης (έλεγχος εκτός SLAM, σφάλμα έναντι της αληθινής κίνησης της σάρωσης, διάμεσος):

| κίνηση για το deskew | στροφή / θέση |
|---|---|
| εικόνα, ζεύγος (k−1, k) | **0.92° / 27 mm** |
| μέσος όρος 2 / 3 τελευταίων εκτιμήσεων της εικόνας | 1.38° / 26 mm · 1.54° / 26 mm |
| η ΑΛΗΘΙΝΗ κίνηση της προηγούμενης σάρωσης | 1.59° / 16 mm |
| κανένα | 1.68° / 106 mm |

#### Συμπέρασμα ⏳
1. **Η καλύτερη τροχιά μόνο με LiDAR μέχρι τώρα** (ATE 0.258 m, −19 % από το KISS, −9 % από το «χωρίς deskew») και
   το μικρότερο σφάλμα ύψους (0.179 m) — αλλά μακριά από το 0.101 του σωστού deskew. Το σφάλμα στροφής ανά σάρωση
   (RPE 1.07°) είναι ακόμη χειρότερο από το «χωρίς deskew».
2. **Η στροφή του χεριού αλλάζει από σάρωση σε σάρωση σχεδόν όσο είναι η ίδια:** ακόμη και η αληθινή κίνηση της
   προηγούμενης σάρωσης απέχει 1.59° από την τωρινή. Αυτό εξηγεί την αποτυχία της σταθερής ταχύτητας του KISS, και
   γιατί η εξομάλυνση χειροτερεύει.
3. Η εκτίμηση της εικόνας υποθέτει σταθερή ταχύτητα **σε δύο σαρώσεις** (0.2 s) — πιθανό όριο της ακρίβειάς της
   (0.92°). Επόμενο: ταχύτητα που μεταβάλλεται γραμμικά μέσα στο ζεύγος· και συνδυασμός με το loop closure του #016.

---

### 2026-09-18 — #018 Ι-3: κίνηση μέσα στη σάρωση από τις εικόνες intensity (εκτός SLAM)

**git commit:** αυτή η εγγραφή · `scripts/test_i3_intensity_deskew.py` (~30 s)

#### Στόχος
Ιδέα / ΑΠΟΦΑΣΗ Λ.Γ. 18/9: χωρίς IMU, μπορεί η εικόνα intensity να δώσει την κίνηση μέσα στη σάρωση — την ανεξάρτητη
πληροφορία που έλειπε από την επανάληψη του #017;

#### Μέθοδος
Για κάθε 2η σάρωση k σε τέσσερις περιοχές, ζεύγος (k, k+1) από **ακατέργαστα** σημεία: πανόραμα intensity 64×1024
με 3D σημείο και χρόνο ανά pixel· SIFT + ratio 0.75· RANSAC + Kabsch (κατώφλι 0.30 m) → «εικόνα χωρίς χρόνο»·
μετά σταθερή ταχύτητα ξ = (ω, v) ανά περίοδο με τον χρόνο κάθε σημείου, R(t_p ω)p + t_p v = R(t_q ω)q + t_q v,
ελάχιστα τετράγωνα (soft-L1, 3 επαναλήψεις επιλογής inliers < 0.10 m) → «εικόνα με χρόνο». Σύγκριση της κίνησης
μιας περιόδου με την αληθινή κίνηση της σάρωσης k+1 (inv(G_k)·G_{k+1}, τ = 0 — αυτή του deskew από GT, ATE 0.101).

#### Αποτέλεσμα
Σφάλμα έναντι της αληθινής κίνησης (διάμεσος, στροφή / θέση):

| πηγή της κίνησης για το deskew | ισόγειο, στροφές | σκάλα πάνω | όροφος | σκάλα κάτω |
|---|---|---|---|---|
| κανένα deskew (= το μέγεθος της κίνησης) | 1.33° / 109 mm | 2.00° / 87 mm | 1.31° / 95 mm | 2.30° / 74 mm |
| μαντεψιά KISS (προηγούμενη σάρωση) | 1.58° / 176 mm | 3.21° / 90 mm | 1.84° / 100 mm | 3.43° / 112 mm |
| ICP της ίδιας σάρωσης (#017) | 1.35° / 153 mm | 2.45° / 88 mm | 1.27° / 82 mm | 2.46° / 98 mm |
| εικόνα χωρίς χρόνο | 0.79° / 43 mm | 1.04° / 40 mm | 0.59° / 27 mm | 1.16° / 41 mm |
| **εικόνα με χρόνο (Ι-3)** | **0.78° / 42 mm** | **1.07° / 27 mm** | **0.66° / 19 mm** | **1.23° / 28 mm** |
| inliers (διάμεσος) | 47 | 66 | 174 | 70 |

Επιτυχία 100 % σε όλες τις περιοχές. Η εικόνα με χρόνο είναι πιο κοντά στην αλήθεια από τη μαντεψιά του KISS σε
**91 / 96 / 91 / 96 %** των σαρώσεων.

#### Συμπέρασμα ⏳
1. **Η εικόνα intensity μετρά την κίνηση μέσα στη σάρωση 2–3× καλύτερα από τη μαντεψιά του KISS**, και καλύτερα
   από το «κανένα deskew» και από τον ICP της ίδιας σάρωσης — **και στη σκάλα**, με μόνο LiDAR.
2. Επιβεβαιώνει την εξήγηση του #017: η μαντεψιά είναι χειρότερη από το μηδέν (1.6–3.4° έναντι 1.3–2.3°), και ο ICP
   της ίδιας σάρωσης μόλις που βελτιώνει το μηδέν· η εικόνα φέρνει ανεξάρτητη πληροφορία.
3. Ο χρόνος κάθε pixel βοηθά κυρίως τη μετατόπιση (40 → 27 mm)· τη στροφή τη δίνει ήδη η απλή αντιστοίχιση.
4. Ακόμη μακριά από το ιδανικό (deskew από GT: σφάλμα ανά σάρωση ~0.2°). Αναμενόμενο ATE κάπου ανάμεσα στο 0.283
   και στο 0.101 — πρέπει να μετρηθεί μέσα στο SLAM.
5. Επόμενο: deskew με αυτή την κίνηση μέσα στο SLAM (αιτιακά: τρέχουσα + προηγούμενη σάρωση).
6. **Για το διδακτορικό:** το intensity βρίσκει ρόλο εκεί που βρίσκεται το μεγαλύτερο σφάλμα (deskew), όχι στο
   φιλτράρισμα του ICP. Η πρωτοτυπία πρέπει να ελεγχθεί έναντι της βιβλιογραφίας.

---

### 2026-09-18 — #017 Deskew σε περισσότερα περάσματα, μόνο με LiDAR

**git commit:** αυτή η εγγραφή · `scripts/run_deskew_refine.sh 2 3` · `tests/test_deskew_refine.py` ·
runs: `indoor_detail_deskew_p2`, `indoor_detail_deskew_p3`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: να πλησιάσει το ATE του deskew με την αληθινή κίνηση (0.101 m, #012) χωρίς GT και χωρίς IMU:
κάθε σάρωση ισιώνεται ξανά με την κίνηση που μόλις υπολογίστηκε **γι' αυτήν**, όχι με της προηγούμενης.

#### Μέθοδος
`KissSLAM._register_frame_deskew_refined` (`deskew_refine.passes`): πέρασμα 1 = `KissICP.register_frame` του
kiss_icp 1.3.0· κάθε επόμενο: deskew της ακατέργαστης σάρωσης με inv(last_pose)·new_pose (η ποσότητα που το #012 πήρε
από το GT), ICP ξανά από new_pose. Κατώφλι σ, χάρτης και last_delta ενημερώνονται μία φορά, με το τελικό αποτέλεσμα.
Έλεγχος: με passes = 1 η τροχιά ταυτίζεται με του KISS (max|Δ| 7·10⁻¹⁵, 150 σαρώσεις). Βάση `indoor_detail`
(deskew true), κατά τα άλλα upstream. Αξιολόγηση με −10 ms (όπως το deskew από GT).

#### Αποτέλεσμα

| | ATE (m) | z RMSE (m) | μήκος (m, GT 253) | ανά σάρωση θέση / στροφή (διάμεσος) | χειρότερη ακμή odometry |
|---|---|---|---|---|---|
| KISS (1 πέρασμα) | 0.319 | 0.200 | 398 | 108 mm / 1.25° | 1.23 m, 5.8° |
| 2 περάσματα | 1.238 | 1.166 | 407 | 95 mm / 1.13° | **4.46 m, 17.8°** (σαρώσεις 1128–1283) |
| 3 περάσματα | 17.9 | 7.96 | 404 | 95 mm / 1.14° | **41.1 m, 121°** |
| deskew από GT (#012) | 0.101 | 0.076 | 264 | 10–30 mm / 0.2–0.3° | — |

Με 2 περάσματα οι υπόλοιπες ακμές odometry είναι συχνά καλύτερες από του KISS (0.13–0.14 m έναντι ~0.44 m), αλλά ο
χάρτης στον όροφο αμέσως μετά την ανάβαση στράβωσε και το ύψος ξέφυγε έως +7 m πριν το τραβήξει πίσω το closure 9↔13.
Με 3 περάσματα δεν έγινε κανένα closure.

#### Συμπέρασμα ⏳
1. **Η απλή επανάληψη είναι ασταθής:** περισσότερα περάσματα ⇒ χειρότερη κατάρρευση. Κλειστό ως μέθοδος.
2. **Ακόμη και όπου δεν καταρρέει, το κέρδος είναι μικρό (~12 % ανά σάρωση)** — πολύ μακριά από το 10–30 mm του
   deskew με την αληθινή κίνηση.
3. Εύλογη αιτία, όχι αποδεδειγμένη: η κίνηση υπολογίζεται από την ίδια στρεβλωμένη σάρωση· μια σάρωση ισιωμένη με
   λάθος κίνηση μπορεί να ταιριάζει καλά στον χάρτη, οπότε ο ICP δεν «βλέπει» το λάθος και η επανάληψη το ενισχύει.
   Λείπει **ανεξάρτητη** πληροφορία για την κίνηση μέσα στη σάρωση: ομαλότητα ανάμεσα σε σαρώσεις (CT-ICP) ή IMU.
4. Επόμενο, φθηνό: deskew με **μόνο τη στροφή** από το GT. Αν αρκεί, το γυροσκόπιο του bag είναι η φυσική λύση.

---

### 2026-09-18 — #016 Έλεγχος ύψους στις ενώσεις χαρτών

**git commit:** αυτή η εγγραφή · `scripts/run_lc_variants.sh 1.5 {nodeskew|deskew} 1.0` · runs:
`indoor_detail_nodeskew_lc_topk5_h1.5_vc1.0`, `indoor_detail_lc_topk5_h1.5_vc1.0`

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: να απορρίπτονται οι ενώσεις με λάθος ύψος του #015 (όροφος ↔ σκάλα, 2.3 m).

#### Μέθοδος
`LoopCloser.height_is_consistent`: μετά τον ICP και τον έλεγχο επικάλυψης, σύγκριση της διαφοράς ύψους των δύο
χαρτών κατά τον περιορισμό του closure (T_query←ref) και κατά την odometry (inv(K_query)·K_ref), πάνω στην κατακόρυφο
του χάρτη-ερωτήματος (τρίτη γραμμή του ground alignment του MapClosures). Απόρριψη αν διαφέρουν > 1.0 m. Βραχίονας:
top-5 + ύψος 1.5 m, με και χωρίς deskew.

#### Αποτέλεσμα

| | closures (απορρ.) | ATE (m) | z RMSE (m) | σφάλμα closures έναντι GT (διάμεσος / max) |
|---|---|---|---|---|
| χωρίς deskew, baseline | 2 | 0.283 | 0.233 | 0.30 / 0.31 m |
| χωρίς deskew, ύψος μόνο (#015) | 4 | 0.257 | 0.194 | 0.22 / 0.23 m |
| χωρίς deskew, top-5 + ύψος (#015) | 8 | 0.471 | 0.434 | 0.16 / 2.40 m |
| **χωρίς deskew, top-5 + ύψος + έλεγχος** | 6 (2) | **0.258** | **0.192** | 0.15 / 0.22 m |
| deskew KISS, top-5 + ύψος (#014) | 16 | 0.316 | 0.208 | 1.23 / 2.15 m |
| deskew KISS, top-5 + ύψος + έλεγχος | 16 (0) | 0.317 | 0.209 | 1.22 / 2.15 m |

Απορρίφθηκαν 11↔15 (closure −0.59 m, odometry +2.38 m) και 11↔16 (+1.06 / +4.03 m): διαφωνία 2.97 m. Οι σωστές
διαφωνούν 0.00–0.04 m (χωρίς deskew)· μετά την πρώτη δεκτή ένωση το pose graph έχει ήδη διορθωθεί, γι' αυτό οι
επόμενες συμφωνούν σχεδόν ακριβώς. Με το deskew του KISS οι διαφωνίες φτάνουν 0.80 m, καμία απόρριψη.

#### Συμπέρασμα ⏳
1. **Ο έλεγχος πιάνει ακριβώς τις ενώσεις όροφος ↔ σκάλα**, με μεγάλο περιθώριο (2.97 m έναντι ≤ 0.04 m), και
   επαναφέρει την τροχιά στο επίπεδο της κατάτμησης στο ύψος: ATE 0.258 m, −9 % από το baseline χωρίς deskew.
2. Το top-5 δεν προσθέτει κάτι μετρήσιμο στο ATE πέρα από την κατάτμηση στο ύψος — βρίσκει περισσότερες σωστές
   ενώσεις, αλλά αυτές συμφωνούν ήδη με το διορθωμένο graph.
3. Με το deskew του KISS ο έλεγχος δεν βοηθά: οι ενώσεις απέχουν 1.2 m από το GT αλλά **συμφωνούν με την odometry**,
   δηλαδή το σφάλμα βρίσκεται στους ίδιους τους (στρεβλωμένους) χάρτες. Ξανά: πρώτα σωστό deskew.

---

### 2026-09-18 — #015 Top-K και κατάτμηση στο ύψος χωρίς deskew

**git commit:** αυτή η εγγραφή · `scripts/run_lc_variants.sh 1.5 nodeskew` · runs: `indoor_detail_nodeskew` (baseline,
#012), `indoor_detail_nodeskew_lc_{topk5,h1.5,topk5_h1.5}` · αξιολόγηση με `--offset-ms=-55` (#012)

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: να επαναληφθεί το #014 σε baseline χωρίς το λάθος deskew, για να φανεί αν τα closures βοηθούν
όταν δεν τα σκεπάζει το τρέμουλο — και αν το λάθος ύψος των closures του #014 οφείλεται στο deskew.

#### Μέθοδος
Ίδια με το #014, με `odometry.preprocessing.deskew: false`. Τα scripts ανάλυσης δέχονται πλέον `--offset-ms`.

#### Αποτέλεσμα

| χωρίς deskew | χάρτες | closures | λάθος closures | ATE (m) | z RMSE (m) | τελικό z (GT +0.05) | RPE στροφής |
|---|---|---|---|---|---|---|---|
| baseline | 15 | 2 | 0 | 0.283 | 0.233 | +0.39 | 0.87° |
| top-5 | 15 | 4 (+7↔13, 9↔13) | 0 | 0.285 | 0.235 | +0.40 | 0.87° |
| **ύψος 1.5 m** | 19 | 4 | 0 | **0.257** | **0.194** | +0.44 | 0.86° |
| top-5 + ύψος | 19 | 8 | **2** | 0.471 | 0.434 | −0.73 | 0.88° |

Σφάλμα περιορισμού closure έναντι GT (διάμεσος): baseline 0.30 m · top-5 0.30 m · ύψος 0.22 m · συνδυασμός 0.16 m,
αλλά με δύο ακραία: **11↔15: 2.40 m (z −2.39)** και **11↔16: 2.29 m (z −2.28)**. Ο χάρτης 11 είναι στον όροφο, οι 15–16
στην κατάβαση. Είναι οι δύο με τη μικρότερη επικάλυψη (0.465 / 0.460) και λίγα inliers (7 / 10)· τα σωστά
closures έχουν επικάλυψη 0.67–0.78. Στο baseline χωρίς deskew οι πραγματικές επανεπισκέψεις (≥ 4 θέσεις) είναι 7,
βρέθηκαν 2· με top-5, 4.

#### Συμπέρασμα ⏳
1. **Το λάθος ύψος 1–2 m των closures του #014 ήταν κυρίως αποτέλεσμα του deskew:** χωρίς deskew τα σωστά closures
   απέχουν 0.15–0.30 m από το GT.
2. **Η κατάτμηση στο ύψος μόνη της βελτιώνει: ATE −9 %, z RMSE −17 %**, με 4 σωστά closures. Ένα run.
3. **Το top-5 σώζει τις χαμένες επανεπισκέψεις της βάσης της σκάλας (7↔13, 9↔13) αλλά δεν αλλάζει το ATE.**
4. **Ο συνδυασμός δέχτηκε δύο λάθος ενώσεις όροφος ↔ σκάλα, μετατοπισμένες 2.3 m, και το ATE σχεδόν διπλασιάστηκε.**
   Ο κίνδυνος που προέβλεψε το #013 (ίδια κάτοψη, άλλο ύψος) εμφανίστηκε μόλις εξετάστηκαν περισσότεροι
   υποψήφιοι. Ο ICP και ο έλεγχος επικάλυψης (0.46 > 0.4) δεν τον πιάνουν· και ένα λάθος closure με το ίδιο βάρος
   με την odometry αρκεί.
5. **Επόμενο: έλεγχος κατακόρυφης συνέπειας** — απόρριψη closure όταν η διαφορά ύψους που ορίζει διαφωνεί με την
   odometry πάνω από ένα όριο. Εδώ η odometry ξέρει το ύψος καλά (λάθη < 1 m), ενώ τα λάθος closures έχουν 2.3 m.
   Η αύξηση του κατωφλίου επικάλυψης σε 0.5 θα τα απέρριπτε κι αυτή, αλλά θα ήταν ρύθμιση πάνω στα ίδια δεδομένα.

---

### 2026-09-18 — #014 Βελτίωση της συνένωσης χαρτών: έλεγχος top-K και κατάτμηση στο ύψος

**git commit:** αυτή η εγγραφή · scripts: `run_lc_variants.sh`, `analyze_node_splits.py`,
`check_closure_constraints.py`, `analyze_loop_closures.py` (διορθωμένο) · runs: `indoor_detail_lc_{topk5,h1.5,topk5_h1.5}`
(πρώτη, λανθασμένη εκδοχή: `runs/lc_v1_buggy/`)

#### Στόχος
ΑΠΟΦΑΣΗ Λ.Γ. 18/9: να δοκιμαστούν μαζί οι βελτιώσεις (α) και (β) του #013 — επαλήθευση περισσότερων υποψηφίων
και ένας όροφος ανά τοπικό χάρτη.

#### Μέθοδος
1. **`loop_closer.top_k`:** `get_top_k_closures(k)`, ταξινόμηση κατά inliers, ICP + έλεγχος επικάλυψης σε **κάθε**
   υποψήφιο πάνω από το κατώφλι inliers, και προσθήκη **όλων** των δεκτών στο pose graph (μία βελτιστοποίηση στο
   τέλος). Με `top_k = 1` ο κώδικας είναι ο upstream.
2. **`local_mapper.splitting_height`:** νέος χάρτης όταν |up · t| > H, όπου t η θέση στο πλαίσιο του node και up η
   κατακόρυφος σε αυτό: η τρίτη γραμμή του `ground_alignment` που υπολογίζει το MapClosures για τον **προηγούμενο**
   χάρτη, στραμμένη με τη σχετική κίνηση. Για τον πρώτο χάρτη δεν υπάρχει κατακόρυφος → μόνο απόσταση. H = 1.5 m.
3. Βάση: `configs/indoor_detail.yaml` (deskew του KISS), ίδια με το baseline του #013. Ένα run ανά βραχίονα.
4. Αξιολόγηση: ATE/RPE (`evaluate_gt.py --offset-ms=-70`)· επανεπισκέψεις (`analyze_loop_closures.py`, τώρα μόνο
   ζεύγη ≥ 4 θέσεων, όσα επιτρέπεται να προταθούν)· και **ορθότητα κάθε closure**: ο περιορισμός της ακμής g2o
   έναντι inv(G_i)·G_j του GT (`check_closure_constraints.py`).

#### Δύο λάθη της πρώτης υλοποίησης (βρέθηκαν από τα αποτελέσματα, διορθώθηκαν πριν τα νούμερα παρακάτω)
- Το top-K σταματούσε στον **πρώτο δεκτό**: το 8↔13 γινόταν δεκτό, το 9↔13 δεν ελεγχόταν ποτέ → ίδιο αποτέλεσμα με
  το baseline. Ο χάρτης 13 επανεπισκέπτεται **πολλούς** παλιούς χάρτες· χρειάζονται όλοι.
- Το «ύψος» ήταν το z στο πλαίσιο του αισθητήρα. Η μονάδα κρατιέται με κλίση 5–26° (GT), άρα σε 15 m επίπεδου
  δαπέδου το z αλλάζει 1–3 m: 22 χάρτες αντί για 15, κατατμήσεις παντού στο ισόγειο, ενώ στη σκάλα ένας χάρτης
  κάλυψε +2.75 m χωρίς κατάτμηση.

#### Αποτέλεσμα

| βραχίονας | χάρτες | closures | ATE (m) | z RMSE (m) | τελικό z (m, GT +0.05) | μήκος (m, GT 253) | RPE στροφής |
|---|---|---|---|---|---|---|---|
| baseline (#013) | 15 | 2 | 0.339 | **0.200** | −0.23 | 398 | 2.27° |
| top-5 | 15 | 3 | 0.356 | 0.236 | −0.14 | 398 | 2.27° |
| ύψος 1.5 m | 20 | 5 | 0.357 | 0.248 | −0.07 | 389 | **2.17°** |
| **top-5 + ύψος** | 20 | **16** | **0.316** | 0.208 | **+0.01** | 389 | 2.18° |

**Κατακόρυφος (`analyze_node_splits.py`, run ύψους):** σφάλμα έναντι GT διάμεσος 4.1°, max 9.5° (στους χάρτες της
κατάβασης). Στο ισόγειο κατατμήσεις μόνο λόγω απόστασης· η ανάβαση κόπηκε σε χάρτες +1.56 / +1.30 / +0.59 m, η
κατάβαση σε −1.69 / −0.98 / −0.97 m.

**Ορθότητα των closures (`check_closure_constraints.py`)** — σφάλμα περιορισμού έναντι GT:

| run | ακμές odometry (διάμεσος θέση / στροφή) | closures (διάμεσος / max θέση, διάμεσος στροφή) |
|---|---|---|
| baseline | 0.44 m / 2.7° | 0.51 / 0.73 m, 5.0° (2) |
| top-5 | 0.44 m / 2.7° | 0.73 / 1.24 m, 3.9° (3) — το νέο 9↔13: 1.24 m, z −1.22 |
| ύψος | 0.27 m / 2.9° | 1.34 / 1.51 m, 5.0° (5) |
| top-5 + ύψος | 0.35 m / 3.0° | **1.23 / 2.15 m**, 5.1° (16) — σχεδόν όλο στο z |

Το 16↔12 έγινε δεκτό με επικάλυψη **0.894** και σφάλμα ύψους **2.10 m**. Κανένα closure δεν ένωσε ισόγειο με όροφο
στο επίπεδο· το λάθος είναι **κατακόρυφη μετατόπιση** μέσα στην ίδια περιοχή (κλιμακοστάσιο / όροφος).

#### Συμπέρασμα ⏳
1. **Ο μηχανισμός δουλεύει όπως σχεδιάστηκε:** το top-5 σώζει το 9↔13· η κατάτμηση στο ύψος φέρνει ενώσεις
   ορόφου ↔ σκάλας καθόδου, το είδος που χανόταν στο #013. Μαζί: 16 closures αντί για 2.
2. **Το ATE βελτιώνεται μόνο μαζί και λίγο (−7 %)**, με ένα run ανά βραχίονα — δεν είναι αποδεδειγμένη βελτίωση.
   Χωριστά, και τα δύο χειροτερεύουν ελαφρά το ATE και το z.
3. **Τα νέα closures μπαίνουν με λάθος ύψος 1–2 m, και ο έλεγχος (ICP + επικάλυψη) δεν το πιάνει.** Εύλογη αιτία,
   όχι αποδεδειγμένη: το κλιμακοστάσιο έχει κατακόρυφους τοίχους που επικαλύπτονται και μετά από κατακόρυφη
   μετατόπιση — **το z είναι degenerate στην επαλήθευση του closure**. Επιπλέον όλα τα closures μπαίνουν με το ίδιο
   βάρος με την odometry (μοναδιαίος πίνακας πληροφορίας).
4. Το κυρίαρχο σφάλμα παραμένει το τρέμουλο του deskew (μήκος 389–398 m έναντι 253)· τα closures δεν το αγγίζουν.
5. **Επόμενα:** (i) η ίδια σύγκριση πάνω σε baseline χωρίς το λάθος deskew· (ii) έλεγχος κατακόρυφης συνέπειας
   (§2.4γ): απόρριψη ή μικρότερο βάρος στο z όταν ο περιορισμός διαφωνεί με την odometry. Εδώ μπορεί να
   ξαναμπεί και το intensity: σε κατακόρυφους τοίχους το σχέδιο intensity (παράθυρα, κάγκελα) δεσμεύει το ύψος
   που η γεωμετρία αφήνει ελεύθερο.

---

### 2026-09-18 — #013 Δουλεύει η συνένωση χαρτών με κατόψεις σε κτήριο με ορόφους;

**git commit:** αυτή η εγγραφή · scripts: `analyze_loop_closures.py`, `run_closure_candidates.py` ·
runs: `indoor_detail_base_overlapfix`, `indoor_detail_closure_candidates`

#### Στόχος
Ερώτημα Λ.Γ.: σε dataset με ορόφους, το KISS — που συνενώνει χάρτες μέσω κατόψεων πυκνότητας — δουλεύει σωστά;
Μπορεί να βελτιωθεί;

#### Μέθοδος
1. **Τι έπρεπε να βρεθεί:** κάθε τοπικός χάρτης (PLY, τοποθετημένος με το GT: G(αρχή node)·keypose⁻¹·PLY)·
   για κάθε ζεύγος, 3D επικάλυψη σε voxels 0.5 m με την `local_maps_overlap` (ίδιος τύπος και κατώφλι 0.4 με τον
   loop closer), και επικάλυψη σε κάτοψη (κελιά 0.5 m).
2. **Τι πρότεινε ο ανιχνευτής:** run όπου το `get_best_closure` αντικαθίσταται από `get_top_k_closures(k=5)`,
   με την ίδια απόφαση (επαληθεύεται ο πρώτος) και καταγραφή όλων. Επαλήθευση ισοδυναμίας: ίδια overlap
   (0.660 / 0.681) και ίδια closures με το κανονικό run.
3. Ανάγνωση του `MapClosures.cpp` (v2.1.0) για τη λογική επιλογής υποψηφίων.

#### Αποτέλεσμα
Χάρτες και όροφοι: 0–6 ισόγειο · **7: ισόγειο → όροφος (ολόκληρη η ανάβαση)** · 8–11 όροφος · 12: όροφος → σκάλα ·
**13: σκάλα → ισόγειο (ολόκληρη η κατάβαση)**.

Πραγματικές επανεπισκέψεις (3D > 0.4, χάρτες ≥ 4 θέσεις μακριά — οι πλησιέστεροι αποκλείονται από τον ανιχνευτή
ούτως ή άλλως):

| ζεύγος | όροφοι | 3D | κάτοψη | ανιχνευτής (inliers) | αποτέλεσμα |
|---|---|---:|---:|---:|---|
| 8↔12 | όροφος | 0.42 | 0.86 | 9 (1ος) | ✔ βρέθηκε |
| 8↔13 | όροφος/σκάλα | 0.44 | 0.86 | 15 (1ος) | ✔ βρέθηκε |
| 9↔13 | όροφος/σκάλα | 0.44 | 0.92 | 14 (2ος) | ✘ δεν επαληθεύτηκε — μόνο ο 1ος ελέγχεται |
| 7↔13 | ισόγ./σκάλα | 0.57 | 0.70 | 3 | ✘ κάτω από το κατώφλι 5 |
| 6↔13 | ισόγ./σκάλα | 0.50 | 0.59 | — | ✘ δεν προτάθηκε |

Υποψήφιοι σε όλη τη διαδρομή: **μόνο 5**, σε 2 από τους 14 χάρτες (12: 8/9 inliers, 7/4· 13: 8/15, 9/14, 7/3).
**Κανένας υποψήφιος ισογείου ↔ ορόφου.**

Ζεύγη με ίδιο αποτύπωμα σε κάτοψη αλλά διαφορετικό ύψος: 23 με κάτοψη > 0.4 και 3D ≤ 0.4 — π.χ. 7↔9 (0.84 / 0.37),
6↔8 (0.67 / 0.33), 6↔9 (0.68 / 0.32). *(Η επικάλυψη σε κάτοψη είναι πάντα ≥ της 3D, οπότε ένα μέρος αυτών είναι
απλώς γειτονικοί χάρτες του ίδιου ορόφου· τα ζεύγη ισογείου–ορόφου είναι τα ενδιαφέροντα.)*

Από τον πηγαίο κώδικα: `no_of_local_maps_to_skip = 3` · `min_no_of_matches = 2` · `GetBestClosure` =
`GetTopKClosures(k=1)` · η 2D λύση (x, y, yaw) γίνεται 3D μέσω `ground_alignments_` του κάθε χάρτη.

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από τον Λ.Γ.**

1. **Σε αυτό το dataset η συνένωση με κατόψεις δεν μπερδεύει ορόφους** — δεν πρότεινε ποτέ ζεύγος ισογείου–ορόφου.
2. **Αλλά έχει χαμηλή ανάκληση:** 2 από τις 5 πραγματικές επανεπισκέψεις. Μία (9↔13) χάθηκε μόνο επειδή
   επαληθεύεται ένας υποψήφιος.
3. **Οι άλλες δύο χαμένες είναι στη βάση της σκάλας**, όπου οι χάρτες 7 και 13 περιέχουν δύο ορόφους. Εύλογη αιτία:
   κατόψεις που υπερθέτουν δύο επίπεδα και διφορούμενο «έδαφος» για τη μετάβαση 2D → 3D. Όχι αποδεδειγμένη.
4. **Για κτήρια με ίδιους επάλληλους ορόφους** η προστασία είναι οριακή (3D επικάλυψη ορόφων 0.30–0.37 έναντι
   κατωφλίου 0.4) — αυτό το dataset δεν το δοκιμάζει.

#### Επόμενα βήματα (βελτιώσεις, με σειρά κόστους)
- [ ] Επαλήθευση των K καλύτερων υποψηφίων, όχι μόνο του πρώτου.
- [ ] Κατάτμηση χαρτών όταν αλλάζει το ύψος → ένας όροφος ανά χάρτη.
- [ ] Έλεγχος κατακόρυφης συνέπειας των closures.
- [ ] Κάτοψη από ζώνη ύψους / κάτοψη intensity / όψεις (Ι-2) — θέλει dataset με επάλληλους ίδιους ορόφους.

---

### 2026-09-18 — #012 Η κατάρρευση εξηγήθηκε — το deskew του KISS είναι το κυρίαρχο σφάλμα

**git commit:** αυτή η εγγραφή · scripts: `run_oracle_deskew.py` (+ όρισμα μετατόπισης),
`analyze_deskew_seam.py`, `test_i1_panorama_rotation.py` (+ `--deskew-offset-ms`, `--eval-offset-ms`) ·
runs: `indoor_detail_oracle_deskew_t0`, `indoor_detail_nodeskew` (config `runs/indoor_detail_nodeskew.yaml`)

#### Στόχος
Βήμα 1 που ζήτησε ο Λ.Γ.: γιατί κατέρρευσε καθολικά το run με deskew από GT (#011, ATE 6.8 m), ενώ τοπικά
βελτιώθηκαν όλα;

#### Μέθοδος
1. **Προφίλ απόκλισης αγκυρωμένο στην πρώτη σάρωση** (όχι καθολική ευθυγράμμιση, που απλώνει το λάθος): κλίση και
   ύψος ανά σάρωση, συσσωρευμένο υπογεγραμμένο σφάλμα κλίσης, σ και διόρθωση ICP.
2. **Ζουμ στο παράθυρο της απόκλισης:** στροφή ανά σάρωση GT/baseline/oracle και συνολικό σφάλμα σε παράθυρα 50 σαρώσεων.
3. **Μέτρηση της «ραφής»** (αρχή έναντι τέλους της περιστροφής) για διάφορα deskew — με ελέγχους: αντίστροφη
   κίνηση, μισή κίνηση, σταθερή στροφή ±3°.
4. **Παρεμβάσεις:** (α) deskew από GT με τ = 0 αντί για −70 ms· (β) KISS με `deskew: false`.
5. **Δίκαιη αξιολόγηση:** κάθε run στη δική του βέλτιστη χρονική μετατόπιση (ελάχιστο σφάλμα στροφής/σάρωση).
6. Επανάληψη της σύγκρισης Ι-1 ↔ ICP με σωστά χρονισμένο deskew.

#### Αποτέλεσμα

**Τύπος της κατάρρευσης:** όχι σταδιακή απόκλιση — γεγονός στις σαρώσεις ~400–550, στις στροφές του ισογείου (GT:
19° σε 50 σαρώσεις). Σφάλμα ανά σάρωση 1–3°, αλλά **σταθερά προς την ίδια κατεύθυνση**: 34°, 30°, 58° ανά 50 σαρώσεις
(baseline 2–4°). Κλίση έναντι GT: 5.7° (400) → 39° (450) → 67° (550), και μένει εκεί.

**Ραφή — η μετρική απορρίφθηκε:**

| deskew (στροφές ισογείου, 26 σαρώσεις) | άνοιγμα ραφής (cm) |
|---|---:|
| χωρίς | 3.51 |
| GT ×1 / ×(−1) / ×0.5 / ×(−0.5) | 4.79 / 3.54 / 3.84 / 2.68 |
| **σταθερή στροφή −3° (άσχετη με την κίνηση)** | **0.48** |
| σταθερή στροφή +3° | 7.18 |

Η περιστροφή καλύπτει ~357° (όχι 360°)· η μετρική ανταμείβει ό,τι σπρώχνει τη λωρίδα της αρχής πάνω στη λωρίδα του
τέλους. **Δεν κρίνει την ποιότητα του deskew σε αυτόν τον αισθητήρα.**

**Παρεμβάσεις** (κάθε run στη δική του βέλτιστη μετατόπιση):

| | βέλτ. τ | ATE | κατακόρυφο | μήκος (GT 253) | RPE μετατ. | RPE στροφ. | στροφή/σάρωση |
|---|---:|---:|---:|---:|---:|---:|---:|
| KISS (deskew με προηγ. εκτίμηση) | −70 ms | 0.339 | 0.200 | 397.8 | 0.170 | 2.27° | 1.42° |
| deskew από GT, τ = −70 ms (#011) | −45 ms | 6.815 | 4.650 | 298.6 | 0.104 | 1.63° | 0.81° |
| **deskew από GT, τ = 0** | **−10 ms** | **0.101** | **0.076** | **263.9** | **0.035** | **0.46°** | **0.32°** |
| **χωρίς deskew** | −55 ms | 0.283 | 0.233 | 290.9 | 0.086 | 0.87° | 0.56° |

Σφάλμα στροφής/σάρωση ανά τμήμα (ισόγειο / όροφος / σκάλα↑ / σκάλα↓): KISS 1.29 / 1.36 / 1.89 / 1.91° ·
GT τ=0 **0.42 / 0.26 / 0.21 / 0.23°** · χωρίς deskew 0.65 / 0.53 / 0.38 / 0.41°.

Βέλτιστες μετατοπίσεις έναντι θεωρίας: οι σαρώσεις γράφονται με τον χρόνο εγγραφής, `header.stamp` + ~105 ms.
Σωστό deskew → πόζα στο τέλος της σάρωσης (`header` + 100 ms) → θεωρία **−5 ms**, μετρήθηκε **−10 ms**. Χωρίς deskew
→ πόζα στη μέση (`header` + 50 ms) → θεωρία **−55 ms**, μετρήθηκε **−55 ms**.

**Ι-1 ↔ ICP με σωστά χρονισμένο deskew** (τ = 0, σύγκριση στα −10 ms):

| τμήμα | Δ | ICP | εικόνα | ICP νικά |
|---|---:|---:|---:|---:|
| όροφος | 1 | 0.16° | 0.26° | 69 % |
| σκάλα πάνω | 1 | 0.19° | 0.33° | 82 % |
| σκάλα κάτω | 1 | 0.17° | 0.35° | 75 % |

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από τον Λ.Γ.**

1. **Η κατάρρευση ήταν δικό μου λάθος χρονισμού.** Χρησιμοποίησα για το deskew τα −70 ms της #010· με τ = 0 δεν
   υπάρχει κατάρρευση.
2. **Το deskew του KISS είναι το κυρίαρχο σφάλμα σε αυτά τα δεδομένα.** Σωστό deskew: ATE −70 %, RPE −80 %, το
   τρέμουλο σχεδόν εξαφανίζεται. Ακόμη και **χωρίς** deskew: ATE −17 %, RPE στροφής −62 %.
3. **Η «δυσκολία της σκάλας» ήταν σχεδόν ολόκληρη deskew.** Με σωστό deskew η σκάλα έχει **μικρότερο** σφάλμα από
   τους ορόφους (0.21–0.23° έναντι 0.26–0.42°). Δεν είναι degenerate για τον ICP.
4. **ΔΙΟΡΘΩΣΕΙΣ:** (α) #010 — δεν υπάρχει χρονική διαφορά στο GT· τα −70 ms ήταν τεχνούργημα του deskew του KISS.
   (β) #011 — δεν υπάρχει «όριο θορύβου ~1°» στο GT· ήταν σφάλμα deskew, κοινό σε εικόνα και ICP. (γ) #011 — η
   «δίκαιη» σύγκριση Ι-1 ↔ ICP είχε κι αυτή λάθος χρονισμό· με σωστό, ο ICP είναι ~2× καλύτερος.
5. **Η μέτρηση της ραφής απορρίφθηκε** — οι έλεγχοι έδειξαν ότι μετρά τη γεωμετρία σάρωσης, όχι το deskew.
6. **Τι σημαίνει για το έργο:** το αρχικό concept στόχευε στη γεωμετρική degeneracy· σε αυτά τα δεδομένα το
   κυρίαρχο πρόβλημα είναι η αντιστάθμιση της κίνησης. Ρεαλιστικοί δρόμοι: deskew με την κίνηση της **ίδιας** της
   σάρωσης (μόνο LiDAR, continuous-time ICP) ή από το IMU.

#### Επόμενα βήματα
- [ ] Απόφαση Λ.Γ.: `deskew: false` ως νέο σημείο αναφοράς;
- [ ] Deskew μόνο με LiDAR, με την κίνηση της ίδιας της σάρωσης (continuous-time).
- [ ] Deskew από το IMU — απόφαση Λ.Γ.

---

### 2026-09-18 — #011 Πρώτο τεστ της Ι-1 — και το deskew ως κοινός παρονομαστής

> ⚠️ **Διορθώνεται από την #012:** (α) η κατάρρευση οφειλόταν σε λάθος χρονισμό του deskew (−70 ms αντί 0)·
> (β) το «όριο θορύβου ~1° του GT» δεν υπάρχει· (γ) η σύγκριση Ι-1 ↔ ICP με σωστό χρονισμό δίνει ICP ~2× καλύτερο.

**git commit:** αυτή η εγγραφή · scripts: `test_i1_panorama_rotation.py`, `run_oracle_deskew.py` ·
runs: `indoor_detail_base_overlapfix`, `indoor_detail_oracle_deskew` · OpenCV 5.0.0 (`--no-deps`, το numpy μένει OpenBLAS)

#### Στόχος
Βήμα 2 της σειράς του Λ.Γ.: υπολογίζεται η στροφή στη σκάλα από τις εικόνες intensity (Ι-1) καλύτερα απ' ό,τι ο ICP;

#### Μέθοδος
1. **Ι-1:** για ζεύγη σαρώσεων (k, k+Δ), Δ ∈ {1, 5}, κάθε 2η σάρωση, σε σκάλα πάνω (820–990), κάτω
   (2014–2258) και όροφο (1200–1400, αναφορά): deskew όπως το KISS → πανόραμα intensity 64 × 1024 που κρατά το
   3D σημείο κάθε pixel (και τα μακρινά) → κατακόρυφη μεγέθυνση ×8 → SIFT, ratio 0.75 → 3D ζεύγη → RANSAC
   (κατώφλι 0.15 m) + Kabsch. Σφάλμα στροφής έναντι GT (−70 ms) και σε σύγκριση με τον ICP στα ίδια ζεύγη.
2. **Έλεγχοι για κοινό σφάλμα:** (α) διαφωνία εικόνας–ICP μεταξύ τους, χωρίς GT· (β) deskew με την αληθινή
   κίνηση από το GT αντί της εκτίμησης.
3. **Μηχανισμός:** αυτοσυσχέτιση του σφάλματος ICP και ανάλυση του σφάλματος deskew σε «αλλαγή αληθινής
   κίνησης» και «σφάλμα προηγούμενης εκτίμησης».
4. **Παρέμβαση:** πλήρες run KISS όπου αλλάζει **μόνο** η κίνηση του deskew (αληθινή, από GT)· πρόβλεψη, χάρτης,
   σ όπως πριν. Διερεύνηση της καθολικής κατάρρευσης: closures έναντι GT, συνέχεια του GT, στροφή ανάμεσα στα
   πλαίσια GT και KISS (Kabsch στα διανύσματα στροφής, 1.788 ζεύγη).
5. **Δίκαιη σύγκριση:** εικόνα **και** ICP με το ίδιο σωστό deskew.

#### Αποτέλεσμα

Ι-1 (διάμεσοι, °/ζεύγος, Δ = 1):

| τμήμα | deskew | ICP | εικόνα | εικόνα↔ICP | inliers |
|---|---|---:|---:|---:|---:|
| όροφος | KISS | 0.80 | 0.89 | 0.62 | 202 |
| σκάλα πάνω | KISS | 1.80 | 1.77 | 0.78 | 78 |
| σκάλα κάτω | KISS | 1.69 | 1.79 | 0.59 | 81 |
| σκάλα πάνω | εικόνα: GT · ICP: KISS | 1.80 | **1.18** | 2.24 | 93 |
| σκάλα κάτω | εικόνα: GT · ICP: KISS | 1.69 | **1.01** | 1.92 | 96 |
| σκάλα πάνω | **και τα δύο: GT** | **1.00** | 1.18 | 0.44 | 93 |
| σκάλα κάτω | **και τα δύο: GT** | **0.96** | 1.01 | 0.43 | 96 |

Επιτυχία RANSAC 100 % σε όλα τα ζεύγη.

![Αντιστοιχίσεις SIFT στη σκάλα (κατάβαση, σαρώσεις 2014–2015, inliers)](figures/i1_matches_stairs.png)

*Οι αντιστοιχίσεις πέφτουν στον θόλο με τις «βεντάλιες», στα παράθυρα, στα τόξα και στα κάγκελα.*

Μηχανισμός (baseline, −70 ms): σφάλμα deskew 2.16°/σάρωση· σχέση με το σφάλμα ICP: σφάλμα deskew ρ +0.29,
σφάλμα προηγούμενης εκτίμησης ρ +0.31, αλλαγή αληθινής κίνησης ρ +0.06. Αυτοσυσχέτιση σφάλματος ICP +0.40·
συνημίτονο διαδοχικών διανυσμάτων σφάλματος +0.49 (ίδια φορά).

Παρέμβαση — KISS με deskew από GT (−70 ms):

| | baseline | deskew από GT |
|---|---:|---:|
| σφάλμα στροφής/σάρωση: σκάλα πάνω / κάτω | 1.89 / 1.91° | **1.06 / 1.11°** |
| σφάλμα στροφής/σάρωση: όροφος / ισόγειο | 1.36 / 1.29° | 0.89 / 1.12° |
| σφάλμα κλίσης/σάρωση: σκάλα πάνω / κάτω | 1.44 / 1.67° | 0.80 / 0.88° |
| RPE μετατόπισης / στροφής @1 s | 0.170 m / 2.27° | **0.108 m** / 1.99° |
| μήκος τροχιάς (GT 253.36) | 397.78 m | **298.59 m** |
| **ATE RMSE** | **0.339 m** | **6.819 m** ❗ |
| σφάλμα ύψους ανά 200 σαρώσεις | ±0.3 m | −6.2, +0.5, **+11.2**, +5.0, … < 3 m |
| ταχύτητα | ~8 σαρώσεις/s | ~20 σαρώσεις/s |

Διερεύνηση της κατάρρευσης: closures σωστά (|t| ακμής/GT 2.99/3.10 m και 28.38/28.21 m)· GT συνεχές (διάμεσο
βήμα 50 ms, max κενό 137 ms)· στροφή ανάμεσα στα πλαίσια GT και KISS 0.65° (roll +0.59°), και η διόρθωσή της δεν
μειώνει το υπόλοιπο (2.54 → 2.54°) → τα πλαίσια συμφωνούν. Χειρότερο σφάλμα ανά σάρωση: 3.3° (baseline 6.5°).

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από τον Λ.Γ.**

1. **Η Ι-1, ως εκτιμητής στροφής, δεν ξεπερνά τον ICP με ίσους όρους.** Οι αντιστοιχίσεις δουλεύουν σταθερά,
   αλλά με το ίδιο deskew ο ICP είναι ίσος ή λίγο καλύτερος. Το φαινομενικό πλεονέκτημα εμφανίστηκε μόνο όταν η
   εικόνα είχε καλύτερο deskew — γι' αυτό έγινε και η δίκαιη σύγκριση.
2. **Το deskew είναι ο κοινός παρονομαστής.** Με σωστό deskew, το σφάλμα στροφής της σκάλας πέφτει σχεδόν στο
   μισό, το τρέμουλο υποχωρεί, και η σκάλα παύει να ξεχωρίζει από τους ορόφους. Το «περιβάλλον» της #010 περνούσε
   σε μεγάλο βαθμό μέσα από το deskew.
3. **Εύλογος μηχανισμός (όχι αποδεδειγμένος): φαύλος κύκλος.** Το KISS κάνει deskew με τη δική του εκτίμηση της
   προηγούμενης κίνησης· λάθος εκτίμηση → στρεβλωμένη σάρωση → νέο λάθος με την ίδια φορά.
4. ❗ **Η καθολική κατάρρευση του run με σωστό deskew δεν έχει εξηγηθεί.** Μέχρι να εξηγηθεί, το «καλύτερο deskew
   βελτιώνει» ισχύει **μόνο τοπικά**.
5. **Όριο μέτρησης:** ανά σάρωση, εικόνα και ICP διαφωνούν μεταξύ τους 0.4–0.5° αλλά ~1° με το GT → το GT έχει
   θόρυβο ~1° ανά σάρωση. Διαφορές μικρότερες από αυτό δεν είναι αξιόπιστες.

#### Επόμενα βήματα
- [ ] Εξήγηση της καθολικής κατάρρευσης (σαρώσεις 0–800).
- [ ] Deskew από το IMU του bag — ρεαλιστική εκδοχή· θέλει απόφαση Λ.Γ. (εκτός «μόνο LiDAR»).
- [ ] Ι-1 σε ρόλο ευρωστίας (αναγνώριση θέσης / loop closure), όχι ακρίβειας.

---

### 2026-09-18 — #010 Κίνηση ή περιβάλλον; — και μια χρονική διαφορά με το GT

> ⚠️ **Διορθώνεται από την #012:** δεν υπάρχει χρονική διαφορά στο GT — τα −70 ms ήταν τεχνούργημα του deskew
> του KISS. Και το «περιβάλλον» της σκάλας ήταν σχεδόν ολόκληρο deskew.

**git commit:** αυτή η εγγραφή · script: `scripts/analyze_motion_vs_error.py` · run: `runs/indoor_detail_base_overlapfix`

#### Στόχος
Βήμα 1 της σειράς που συμφωνήθηκε με τον Λ.Γ.: οφείλεται το σφάλμα στροφής της σκάλας στην **κίνηση** (απότομες
κινήσεις του χεριού → αστοχεί η πρόβλεψη σταθερής ταχύτητας και το deskew) ή στο **περιβάλλον**; Αν στην κίνηση,
κανένα intensity δεν θα βοηθήσει.

#### Μέθοδος
- Ανά σάρωση: σφάλμα στροφής = γωνία(Δ_GT⁻¹·Δ_εκτ.)· «αλλαγή στροφής» δ_k = γωνία(Δ_GT,k−1⁻¹·Δ_GT,k), δηλαδή
  αυτό που χάνει η υπόθεση σταθερής ταχύτητας.
- Σύγκριση σκάλας–ορόφων **στην ίδια κλάση δ** (πεμπτημόρια), και γραμμικό μοντέλο
  σφάλμα ~ a + b·δ + c·[σκάλα πάνω] + c′·[σκάλα κάτω], με 95 % διαστήματα από block bootstrap (μπλοκ 20
  σαρώσεων, 2.000 επαναλήψεις), γιατί οι διαδοχικές σαρώσεις δεν είναι ανεξάρτητες.
- **Έλεγχος χρονικής ευθυγράμμισης**, επειδή η κλίση b = 0.45 ήταν ύποπτα κοντά στο 0.5 (αυτό θα έδινε
  μετατόπιση μισής σάρωσης): σάρωση της μετατόπισης −100…+100 ms· ανάγνωση του χρόνου εγγραφής έναντι του
  `header.stamp` στο bag· ανάγνωση του deskew στον πηγαίο κώδικα του KISS-ICP 1.3.0.

#### Αποτέλεσμα

**Χρόνος.** Ο dataloader γράφει τον **χρόνο εγγραφής** στο bag: `header.stamp` + **105.3 ms** (102.5–113.6, 401
σαρώσεις). Το KISS κάνει deskew στο **τέλος** της σάρωσης (`Preprocessing.cpp`: `exp((stamp − 1.0) * omega)`),
δηλαδή `header.stamp` + ~100 ms. Άρα οι χρονοσφραγίδες μας είναι σωστές ±5 ms. Παρ' όλα αυτά:

| run | ελάχιστο σφάλμα στροφής/σάρωση | ελάχιστο σφάλμα θέσης/σάρωση |
|---|---:|---:|
| indoor_detail_base_overlapfix | −70 ms (1.424° έναντι 1.878° στα 0) | −75 ms |
| indoor_fast_base | −70 ms | −75 ms |
| indoor_detail_replace | −75 ms | −70 ms |

Το ATE αντίθετα ελαχιστοποιείται στα +80 ms (0.3078 έναντι 0.3169 m) — είναι όμως καθολικό μέγεθος και
ευαίσθητο στο drift.

**Κίνηση ή περιβάλλον** (baseline `indoor_detail`):

| | χωρίς διόρθωση χρόνου | με −70 ms |
|---|---:|---:|
| επίδραση ανά 1° αλλαγής κίνησης | +0.450° [0.398, 0.500] | **+0.020° [−0.036, +0.072]** |
| επιπλέον στη σκάλα πάνω | +0.739° [0.368, 1.155] | **+0.562° [0.226, 0.961]** |
| επιπλέον στη σκάλα κάτω | +0.678° [0.279, 1.055] | **+0.580° [0.218, 0.929]** |
| Spearman (σφάλμα, αλλαγή κίνησης) | +0.38 | +0.06 |

Σφάλμα στροφής ανά σάρωση με −70 ms: όροφοι 1.29–1.36°, σκάλα 1.89–1.91°. Σε **κάθε** κλάση αλλαγής κίνησης η
σκάλα σφάλλει περισσότερο (π.χ. χαμηλότερη κλάση: όροφοι 1.239°, σκάλα 1.935–1.944°).

![Σφάλμα στροφής έναντι αλλαγής κίνησης, με διόρθωση χρόνου](figures/motion_vs_error_offset-70ms.png)

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από τον Λ.Γ.**

1. **Η υπεροχή του σφάλματος στη σκάλα οφείλεται στο περιβάλλον, όχι στην κίνηση.** +0.56–0.58° ανά σάρωση, με
   την ίδια κίνηση. Η Ι-1 στοχεύει σωστά.
2. **Η κίνηση του χεριού δεν επηρεάζει το σφάλμα στροφής** μόλις διορθωθεί ο χρόνος. Άρα ούτε το τρέμουλο
   εξηγείται από απότομες κινήσεις — η πηγή του είναι αλλού.
3. **Υπάρχει χρονική διαφορά ~−70 ms με το GT**, συνεπής σε runs και μετρικές, με τις δικές μας χρονοσφραγίδες
   σωστές κατά τον πηγαίο κώδικα. Πιθανότατα σύμβαση χρόνου του GT (π.χ. αρχή σάρωσης)· **δεν έχει επιβεβαιωθεί**.
   Το ATE διαφωνεί στην κατεύθυνση, οπότε δεν υπάρχει ένας «σωστός» αριθμός για όλες τις μετρικές.
4. **Πρακτικά:** τα νούμερα του log μένουν με 0 ms για να αναπαράγονται. Όλες οι A/B συγκρίσεις ισχύουν, γιατί
   η διαφορά είναι κοινή σε όλους τους βραχίονες. Για αναλύσεις ανά σάρωση χρησιμοποιείται `--offset-ms=-70`.
5. **Επιφύλαξη:** αν το ίδιο το GT είναι πιο θορυβώδες στο κλιμακοστάσιο, μέρος της υπεροχής είναι δικό του.
6. **Παρεμπιπτόντως από τον πηγαίο κώδικα:** το KISS κόβει με `range < max && range > min`· η αντιστοίχιση του
   intensity ευθυγραμμίστηκε στις ίδιες αυστηρές ανισότητες (το τεστ περνά).

#### Επόμενα βήματα
- [ ] Τεστ Ι-1 στη σκάλα.
- [ ] Σύμβαση χρόνου του GT από την τεκμηρίωση του Oxford Spires.

---

### 2026-09-18 — #009 Διόρθωση του ελέγχου επικάλυψης των loop closures

**git commit:** αυτή η εγγραφή · run: `runs/indoor_detail_base_overlapfix`

#### Στόχος
Στο `indoor_detail` το «overlap» των loop closures έβγαινε 4.3–5.5 (λόγος > 1), οπότε το κατώφλι 0.4 δεν
απέρριπτε ποτέ. Απόφαση Λ.Γ.: να διορθωθεί.

#### Μέθοδος
- **Αιτία:** `loop_closer.py` μετρούσε το target ως `len(target_pts)`. Ο χάρτης που φτάνει εκεί είναι
  `PerVoxelPointAndNormal()`, **ένα σημείο (ο μέσος όρος) ανά voxel του `local_mapper.voxel_size`**. Αυτό
  ισούται με τα voxels στην ανάλυση της κάτοψης (`density_map_resolution` = 0.5) **μόνο** όταν οι δύο είναι
  ίσες. Στο `indoor_detail` (0.25) υπερμετρούσε ×3.9.
- **Διόρθωση:** νέα συνάρτηση `local_maps_overlap()` που μετρά source, target και ένωση ως voxels στην **ίδια**
  ανάλυση → λόγος στο [0, 1].
- **Τεστ** `tests/test_closure_overlap.py`: (a) ίδιος χάρτης → 1, μακρινός → 0· (b) ίδιες επιφάνειες σε 0.25 και
  0.5 m → 1.000 (ο παλιός τύπος: 49.3)· (c) οι πραγματικοί χάρτες των closures → 0.62–0.72, πάνω από το
  κατώφλι· (d) στο τοπικό πλαίσιο, με χάρτη 0.5 m: 1.296 σημεία = 1.296 voxels → **νέος == παλιός στο
  `indoor_fast`**· με χάρτη 0.25 m: ×3.9.
- **Από άκρη σε άκρη:** πλήρες run `indoor_detail` baseline με τη διόρθωση.
- Το `tests/test_replace_mode.py` καρφώθηκε στο commit `1409b64`: με «HEAD» έχανε το νόημά του μόλις γινόταν
  commit η ίδια η αλλαγή.

#### Αποτέλεσμα

| | πριν | μετά |
|---|---:|---:|
| overlap closure 1 / 2 | 4.609 / 4.701 | **0.660 / 0.681** |
| closures | 12↔8, 13↔8 | 12↔8, 13↔8 |
| ATE RMSE (m) | 0.3169 | 0.3169 |
| μέγιστη διαφορά τροχιάς | — | 1.3e-5 m |

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από τον Λ.Γ.**

1. Ο έλεγχος δίνει πλέον σωστό λόγο σε κάθε ρύθμιση. Στο `indoor_fast` η συμπεριφορά είναι ακριβώς ίδια.
2. Στο `indoor_detail` τα αποτελέσματα **δεν** άλλαξαν — τα 2 closures ήταν γνήσια και περνούν το κατώφλι και με
   τον σωστό τύπο. Όλα τα προηγούμενα runs του `indoor_detail` μένουν έγκυρα.
3. Η διαφορά 13 μm δεν είναι bitwise όπως στα τεστ 300–400 σαρώσεων (1e-14)· είναι αμελητέα, αλλά η αιτία της
   σε πλήρες run δεν έχει ελεγχθεί.

---

### 2026-09-18 — #008 Η σκάλα: πού σφάλλει ο ICP και τι intensity υπάρχει εκεί

**git commit:** `a1e60b6` (+ αυτή η εγγραφή) · script: `scripts/analyze_intensity_texture.py`

#### Στόχος
Ο Λ.Γ. θεωρεί ότι το σφάλμα του ICP βρίσκεται εκεί όπου η γεωμετρία είναι τυφλή — στη σκάλα που ανεβαίνει
όροφο — και ρώτησε αν η ένταση (intensity) εκεί έχει την πληροφορία που λείπει. Δύο ερωτήματα:
(α) πού και σε ποια κίνηση σφάλλει ο ICP; (β) έχουν οι επιφάνειες της σκάλας σχέδιο intensity;

#### Μέθοδος
- **Εντοπισμός σκάλας:** προφίλ ύψους του GT στις χρονοσφραγίδες των σαρώσεων· σκάλα = ρυθμός ανόδου
  > 0.15 m/s (εξομάλυνση 1 s).
- **Τοπικό σφάλμα (RPE σε 1 s)** του baseline `indoor_detail`, ανά τμήμα, αναλυμένο σε οριζόντιο /
  κατακόρυφο (στο πλαίσιο του GT), στροφή, και κλίση (στροφή του κατακόρυφου άξονα).
- **Σχέδιο intensity:** ακατέργαστο intensity 0–255 (όχι η κανονικοποίηση ανά σάρωση). Κάθε 10η σάρωση,
  4.000 δείγματα < 15 m, 24 γείτονες. Κρατιούνται μόνο γειτονιές **επίπεδες** (λ_min/Σλ < 0.01), που
  **καλύπτουν και τις δύο διευθύνσεις** του επιπέδου (λ₂/λ₃ > 0.05, όχι μία γραμμή σάρωσης) και **πυκνές**
  (10ος γείτονας < 0.6 m). Μετρώνται σ_I (αντίθεση) και |∇I|: κλίση έντασης **πάνω στο επίπεδο** με ελάχιστα
  τετράγωνα, όπως στο Colored ICP (Park et al., ICCV 2017).
- **Πανοραμικές εικόνες** δακτύλιος × αζιμούθιο (απόσταση και intensity) για σαρώσεις 940, 1500, 2050.

#### Αποτέλεσμα

Σκάλα από το GT: **ανάβαση σαρώσεις 820–990** (1.60 → 3.23 m, πλατύσκαλο, 3.25 → 5.37 m) και
**κατάβαση 2014–2258** (5.37 → 3.24 m, πλατύσκαλο, 3.22 → 1.65 m).

Τοπικό σφάλμα σε 1 s (baseline):

| τμήμα | σαρώσεις | οριζόντιο | κατακόρυφο | στροφή | κλίση |
|---|---|---:|---:|---:|---:|
| ισόγειο (αρχή) | 0–820 | 14.6 cm | 7.0 cm | 1.82° | 1.33° |
| **σκάλα πάνω** | 820–990 | **6.8 cm** | **5.8 cm** | 2.59° | 1.92° |
| επάνω όροφος | 990–2014 | 14.4 cm | 10.0 cm | 2.65° | 2.07° |
| **σκάλα κάτω** | 2014–2258 | **7.0 cm** | 11.5 cm | **3.78°** | **3.34°** |
| ισόγειο (τέλος) | 2258–2392 | 9.2 cm | 7.0 cm | 3.28° | 2.74° |

Σχέδιο intensity στις επίπεδες γειτονιές:

| τμήμα | σαρώσεις | επίπεδα | σ_I διάμ. | σ_I p90 | \|∇I\| διάμ. (/m) | \|∇I\| p90 | «με σχέδιο» (σ_I>10) |
|---|---:|---:|---:|---:|---:|---:|---:|
| ισόγειο | 40 | 32.8 % | 10.0 | 34.3 | 70.3 | 268.5 | 50.1 % |
| **σκάλα πάνω** | 17 | 45.1 % | **7.7** | 32.9 | **34.5** | 156.9 | **35.3 %** |
| επάνω όροφος | 60 | 31.4 % | 12.5 | 59.1 | 73.6 | 431.2 | 56.3 % |
| **σκάλα κάτω** | 25 | 39.4 % | **7.7** | 35.0 | **35.8** | 189.8 | **36.1 %** |

![Πανοραμικές εικόνες απόστασης και intensity](figures/intensity_panoramas.png)

*Πάνω σε κάθε ζεύγος: απόσταση· κάτω: intensity. Οι λωρίδες στις εικόνες απόστασης είναι τεχνούργημα της
δειγματοληψίας (1.024 στήλες έναντι ~850–1.200 σημείων ανά δακτύλιο), όχι γεωμετρία.*

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από τον Λ.Γ.**

1. **Ο ισχυρισμός του Λ.Γ. ισχύει, αλλά για τη στροφή.** Στη σκάλα ο ICP έχει το **μικρότερο** σφάλμα
   μετατόπισης της διαδρομής και το **μεγαλύτερο** σφάλμα στροφής/κλίσης — χειρότερο στην κατάβαση. Το μεγάλο
   συνολικό σφάλμα του επάνω ορόφου (έως 0.51 m) είναι συνέπεια: η κλίση που γεννιέται στη σκάλα γίνεται
   σφάλμα ύψους στη συνέχεια.
2. **Οι επίπεδες επιφάνειες της σκάλας έχουν το μισό σχέδιο intensity από τους ορόφους.** Οι μεγάλοι
   σοβατισμένοι τοίχοι και ο θόλος είναι κοντά στον κορεσμό (255). Όπου η γεωμετρία έχει τα περισσότερα επίπεδα
   (45 %), το intensity έχει το λιγότερο σχέδιο.
3. **Το σχέδιο υπάρχει, αλλά σε αραιά, έντονα χαρακτηριστικά** — παράθυρα (σκοτεινά), κάγκελα, τόξα, πόρτες.
   Η εικόνα intensity μοιάζει με φωτογραφία. Αυτό ευνοεί την **Ι-1** (αντιστοίχιση χαρακτηριστικών στην
   πανοραμική εικόνα) περισσότερο από μια κοινή λύση ICP + intensity **πάνω στα επίπεδα** (ColoredICP), που
   χρειάζεται ομαλή μεταβολή στις μεγάλες επιφάνειες.
4. **Το σφάλμα της σκάλας είναι στροφή, και η Ι-1 δίνει στροφή.** Οι δύο παρατηρήσεις συγκλίνουν.
5. **Όρια της μέτρησης:** ο θόρυβος του αισθητήρα στο intensity δεν έχει μετρηθεί, άρα το σ_I περιέχει και
   θόρυβο — η σύγκριση μεταξύ τμημάτων στέκει, η απόλυτη τιμή όχι. Ο κορεσμός στο 255 σημαίνει ότι οι πιο
   φωτεινές επιφάνειες δεν δίνουν καμία πληροφορία intensity.
6. **Εναλλακτική εξήγηση που δεν έχει ελεγχθεί:** η κλίση στην κατάβαση μπορεί να οφείλεται στην **κίνηση**
   (απότομες στροφές του χεριού → αποτυγχάνουν η πρόβλεψη σταθερής ταχύτητας και το deskew), όχι στη
   γεωμετρία. Αν ισχύει, κανένας όρος intensity δεν θα τη διορθώσει.

#### Επόμενα βήματα
- [ ] Τεστ Ι-1 στη σκάλα: στροφή από αντιστοιχίσεις χαρακτηριστικών σε διαδοχικά πανοράματα, έναντι GT και ICP.
- [ ] Έλεγχος της εναλλακτικής (σημείο 6): σφάλμα κλίσης ανά σάρωση έναντι του ρυθμού στροφής της μονάδας.

---

### 2026-09-18 — #007 Τρεις βραχίονες στο `indoor_detail` (intensity σωστά αντιστοιχισμένο)

**git commit:** `526fd97` (κώδικας) · runs: `runs/indoor_detail_{base,int,refine,replace}`

#### Στόχος
Απόφαση Λ.Γ.: να δοκιμαστεί η επιλογή με intensity **ανεξάρτητα**. Ερώτημα: βοηθά η επιλογή των φωτεινών σημείων
όταν (α) το intensity είναι σωστά αντιστοιχισμένο και (β) δεν στηρίζεται στην απάντηση του κανονικού ICP;

#### Μέθοδος
- Διόρθωση αντιστοίχισης intensity (μάσκα πάνω στο ίδιο deskew χωρίς όριο απόστασης) —
  `tests/test_intensity_alignment.py`: 100 % σωστά, με/χωρίς deskew.
- Νέος τρόπος `intensity.mode = replace`: πιστό αντίγραφο του `KissICP.register_frame` (kiss_icp 1.3.0) με μία
  αλλαγή — ο ICP βλέπει μόνο τα επιλεγμένα σημεία· ο χάρτης παίρνει όλα.
  `tests/test_replace_mode.py`: replace με όλα τα σημεία == baseline (1.4e-14 m)· baseline και refine
  αμετάβλητα πέρα από τη διόρθωση (1e-14 / 7e-15 m).
- `scripts/run_arms.sh configs/indoor_detail.yaml refine replace`. Το baseline **δεν** ξανατρέχει: αποδείχθηκε
  αμετάβλητο. Το `int` είναι το refine **πριν** τη διόρθωση (#006), για αναφορά.
- Αξιολόγηση: `scripts/evaluate_gt.py`· πρόσθετη ανάλυση σφάλματος σε οριζόντιο/κατακόρυφο/κλίση/yaw
  (κλίση = γωνία ανάμεσα στους κατακόρυφους άξονες εκτίμησης και GT, μετά από ευθυγράμμιση Umeyama).
- Διαγνωστικό επιλογής: σε 49 σαρώσεις (κάθε 50η) σύγκριση κρατημένων/πεταμένων σημείων σε γωνία ανύψωσης και
  απόσταση, με τις ίδιες συναρτήσεις του pipeline.

#### Αποτέλεσμα

| | base | int (πριν τη διόρθωση) | **refine** | **replace** |
|---|---:|---:|---:|---:|
| ATE RMSE (m) | **0.317** | 0.393 | 0.887 | 0.855 |
| — οριζόντιο (m) | 0.246 | 0.262 | 0.418 | **0.226** |
| — κατακόρυφο (m) | **0.200** | 0.293 | 0.783 | 0.824 |
| κλίση μ. / max (°) | **2.24 / 7.10** | 3.50 / 9.30 | 5.30 / 14.79 | 4.70 / 17.39 |
| yaw μ. (°) | 1.17 | 1.21 | 1.47 | 1.17 |
| RPE μετατόπισης @1s (m) | 0.170 | 0.184 | 0.170 | **0.152** |
| μήκος τροχιάς (m) · GT 253.39 | 397.78 | 406.69 | 396.45 | **368.37** |
| ms/σάρωση | 197 | 248 | 232 | 160 |

Ποια σημεία πετά η επιλογή (ίδιες συναρτήσεις με το pipeline):

| | κρατιούνται (70 %) | πετιούνται (30 %) |
|---|---:|---:|
| βλέπουν κάτω (< −15°) | 21.3 % | 14.1 % |
| σχεδόν οριζόντια (±5°) | 13.7 % | 16.0 % |
| βλέπουν πάνω (> 15°) | 38.9 % | 45.0 % |
| μ. γωνία ανύψωσης | 6.4° | 11.3° |
| μ. απόσταση | 10.2 m | 16.5 m |

Από όλα τα σημεία που βλέπουν κάτω πετιέται το **22.1 %** (τυχαία επιλογή: 30 %).

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από τον Λ.Γ.**

1. **Η επιλογή των 70 % φωτεινότερων σχεδόν τριπλασιάζει το ATE**, και με τις δύο εκδοχές. Η ανεξάρτητη εκδοχή
   δεν σώζει την ιδέα → φταίει η **επιλογή**, όχι ο δεύτερος ICP.
2. **Η ζημιά είναι στο ύψος, μέσω της κλίσης.** Κλίση ×2, κατακόρυφο ×4, ενώ yaw και οριζόντιο μένουν ίδια
   (το replace είναι μάλιστα οριζόντια λίγο καλύτερο). Μονότονη σχέση κλίσης–κατακόρυφου σε όλους τους βραχίονες.
3. **Η αρχική μου υπόθεση (πετιέται το πάτωμα) διαψεύστηκε.** Πετιούνται κυρίως μακρινά σημεία και όσα βλέπουν
   πάνω. Εύλογος μηχανισμός: τα μακρινά σημεία δίνουν μεγάλο μοχλοβραχίονα για την κλίση· χωρίς αυτά, μικρή
   κλίση επί δεκάδες μέτρα γίνεται σφάλμα ύψους. **Δεν** έχει ακόμη αποδειχθεί ελεγχόμενα.
4. **Το bug αντιστοίχισης έκρυβε τη ζημιά** (0.393 → 0.887 m μετά τη διόρθωση): με ανακατεμένο intensity η
   επιλογή στις εξωτερικές σαρώσεις ήταν ουσιαστικά τυχαία.
5. **Σύνδεση με την ιδέα Ι-1 του Λ.Γ.:** αν επιβεβαιωθεί ότι τα μακρινά σημεία κρατούν την κλίση, τότε η Ι-1
   (κρατάμε τις διευθύνσεις των μακρινών σημείων ως περιορισμούς στροφής) στοχεύει ακριβώς εκεί που χτυπά αυτό
   το πρόβλημα.

#### Επόμενα βήματα
- [ ] Ελεγχόμενα τεστ: **τυχαίο 70 %** (χωρίς intensity)· **πέτα μόνο τα μακρινά**· **πέτα μόνο όσα βλέπουν
      πάνω**. Ξεχωρίζει «ποια» από «πόσα» και εντοπίζει τι κρατά την κλίση.
- [ ] Απόφαση Λ.Γ.: κλείνει η επιλογή μόνο με φωτεινότητα (STATUS §5);

---

### 2026-09-18 — #006 🎯 **Πρώτη αξιολόγηση με Ground Truth**

**git commit:** `873e634` (+ αυτή η εγγραφή)

#### Στόχος
Να απαντηθεί επιτέλους η μόνη ερώτηση που μετράει: **βοηθάει η μέθοδος intensity έναντι
του Ground Truth;**

#### Μέθοδος
- Κατέβηκε το `gt-tum.txt` του Oxford Spires (δημόσιο, **χωρίς** auth, 2.24 MB,
  14.379 πόζες @25.6 Hz) → `gt/church_02_gt-tum.txt`.
- Νέο `scripts/evaluate_gt.py`: εφαρμογή του extrinsic **base→lidar**
  (`t=[0,0,0.124]`, `q_xyzw=[0,0,1,0]` — ⚠️ **στροφή 180° περί z, ΟΧΙ identity**),
  παρεμβολή GT στα 2402 timestamps των scans (SLERP + γραμμική), rigid ευθυγράμμιση
  Umeyama **χωρίς scale**, και υπολογισμός ATE / RPE / z-προφίλ.
- Νέο `scripts/run_ab.sh`· έξοδος πλέον στο **`runs/`** (μόνιμο) — ο προσωρινός φάκελος
  είχε καθαριστεί και χάθηκαν τα αρχεία της #005.

#### Αποτέλεσμα

| | BASELINE | INTENSITY |
|---|---:|---:|
| ATE RMSE (m) | 0.5708 | **0.5245** |
| ATE mean / median (m) | 0.5102 / 0.4436 | **0.4746 / 0.4170** |
| ATE max (m) | 1.2589 | **1.1183** |
| z σφάλμα RMSE (m) | **0.1765** | 0.2024 |
| z σφάλμα τελικό (m) | −0.0997 | **+0.0188** |
| z span εκτίμησης (m) | **4.967** | 5.443 |
| **z span GT (m)** | **4.418** | **4.418** |
| μήκος διαδρομής (m) | 426.76 | 421.12 |
| **μήκος διαδρομής GT (m)** | **253.39** | **253.39** |
| RPE μετατόπισης @1s (m) | 0.1786 | **0.1721** |
| RPE στροφής @1s (°) | **2.4481** | 2.4939 |

**Διάγνωση του μήκους διαδρομής** (baseline, ίδια δειγματοληψία 10 Hz και για τα δύο):

| | δικό μας | GT |
|---|---:|---:|
| μέσο βήμα/frame | 0.1777 m | 0.1055 m |
| p99 βήματος | 0.4376 m | 0.1612 m |
| max βήμα | 0.7098 m | **0.1772 m** |
| βήματα > 0.30 m | **195 / 2401** | **0** |

Μήκος μετά από εξομάλυνση της τροχιάς: 3 frames → 321.84 m· 5 → 265.93 m·
**9 → 256.10 m**, έναντι **GT 253.39 m**.

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από χρήστη**

1. **Η μέθοδος intensity δίνει μικρή, όχι αποφασιστική βελτίωση.** ATE RMSE −8 %
   (0.571 → 0.525 m), με συνεπή βελτίωση σε mean/median/max και οριακή σε RPE
   μετατόπισης. **Αλλά είναι χειρότερη κατακόρυφα**: z RMSE +15 % (0.177 → 0.202 m) και
   z span πιο μακριά από το GT (5.443 vs 4.967, GT 4.418). **Δεν υπάρχει καθαρή νίκη.**
2. ❌ **Η #005 ήταν λάθος.** Η «βελτίωση 77 % στο z-drift» μετρήθηκε στο πλαίσιο του
   ίδιου του SLAM, που ορίζεται από την πρώτη πόζα και **δεν είναι κατακόρυφο**. Με σωστή
   ευθυγράμμιση στο GT το φαινόμενο εξαφανίζεται. **Μάθημα: κανένα z μέγεθος δεν έχει
   νόημα πριν την ευθυγράμμιση με το GT.**
3. 🔴 **Το κυρίαρχο σφάλμα δεν είναι το drift — είναι ο υψηλόσυχνος θόρυβος.** Και οι δύο
   βραχίονες υπερεκτιμούν το μήκος διαδρομής κατά **~68 %** (426 / 421 m έναντι 253 m).
   Το GT **δεν έχει ούτε ένα** βήμα > 0.30 m· εμείς έχουμε **195**, με max 0.71 m έναντι
   0.177 m του GT. Εξομάλυνση 9 frames επαναφέρει το μήκος στα 256.10 m — δηλαδή η
   **υποκείμενη τροχιά είναι σωστή** και από πάνω κάθεται jitter ανά frame.
4. **Συνέπεια για την έρευνα:** το jitter (~68 % πλεόνασμα) είναι **τάξη μεγέθους
   μεγαλύτερο** από τη διαφορά των δύο βραχιόνων (~8 %). Οποιαδήποτε βελτίωση από το
   intensity πνίγεται μέσα του. **Προτεραιότητα: να βρεθεί η πηγή του jitter**
   (deskewing; adaptive threshold; πολύ λίγα σημεία ανά scan;) πριν από κάθε άλλη
   βελτιστοποίηση της μεθόδου.
5. Το ATE ~0.5 m σε διαδρομή 253 m είναι ~0.2 % — όχι κακό. Το πρόβλημα είναι τοπικό, όχι
   global drift.

#### Επόμενα βήματα
- [ ] **Πηγή του jitter.** Πρώτος ύποπτος: `indoor_fast` κρατά ~2.500 σημεία/scan. Να
      επαναληφθεί με `indoor_detail` (5.972 σημεία) και να μετρηθεί ξανά το max βήμα.
- [ ] Αφαίρεση του KD-tree των διαγνωστικών (δεν επηρεάζει τροχιές, ~110 από 126 ms/frame)
      ώστε το `indoor_detail` να γίνει πρακτικό.
- [ ] Αντικατάσταση του KD-tree του intensity με άμεσο υπολογισμό voxel key
      (`floor(p/voxel)`) — ταχύτερο **και** ακριβές. Απαιτεί δικό του A/B.
- [ ] Στοχευμένη αξιολόγηση **μόνο στο τμήμα της σκάλας**.

---

### 2026-09-18 — #005 Πρώτο **πλήρες** A/B (2402 frames, `indoor_fast`)

> ❌ **ΤΟ ΣΥΜΠΕΡΑΣΜΑ ΑΥΤΗΣ ΤΗΣ ΕΓΓΡΑΦΗΣ ΑΝΑΙΡΕΙΤΑΙ ΑΠΟ ΤΗΝ #006.** Τα μεγέθη z
> μετρήθηκαν στο **ίδιο το πλαίσιο του SLAM** (ορίζεται από την πρώτη πόζα), που **δεν
> είναι ευθυγραμμισμένο με τη βαρύτητα**. Μετά από σωστή ευθυγράμμιση με το GT τα νούμερα
> είναι εντελώς άλλα και η «βελτίωση 77 %» **δεν υφίσταται**. Διατηρείται ως ιστορικό.

**git commit:** `2715e72` (αρχικό commit· repo: `lazaros-pcvg/Kiss_SLAM`, private)

#### Στόχος
Πρώτη σύγκριση baseline vs intensity σε **ολόκληρη** τη σεκάνς, με ενεργό loop closer.

#### Μέθοδος
`configs/indoor_fast.yaml` (max_range 50, voxel 0.5, splitting_distance 15 m), 2402 frames,
ίδιο config και στους δύο βραχίονες, μόνη διαφορά η σημαία CLI.
Ξεκίνησε πρώτα run με `indoor_detail` (voxel 0.25) αλλά **διακόπηκε**: ο βραχίονας A
έτρεχε σε 1.15 frames/s (≈30 min) και ο B θα ξεπερνούσε τη μία ώρα.

#### Αποτέλεσμα

| | Baseline | Intensity |
|---|---:|---:|
| μ. RMS ICP | 0.1116 m | 0.1020 m |
| μ. inlier ratio | 1.0000 | 1.0000 |
| μ. `n_source` / `n_filtered` | 2550 / 2550 | 2550 / **1784** |
| Μήκος διαδρομής | 426.764 m | 421.116 m |
| Καθαρή μετατόπιση | 89.916 m | 89.943 m |
| **z span** | **7.342 m** | **4.483 m** |
| **z αρχή → τέλος** | **−3.559 m** | **−0.833 m** |
| Local map nodes | 14 | 14 |
| Loop closures | 2 | 2 |
| Χρόνος/frame | 126 ms | 158 ms |

Απόκλιση τροχιών: θέση mean **2.536 m**, max 5.558 m, τελικό 2.844 m·
στροφή mean 4.109°, max 11.832°.

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από χρήστη**

1. **Ενθαρρυντική ένδειξη:** το κατακόρυφο σφάλμα αρχής–τέλους πέφτει από **−3.559 m σε
   −0.833 m** (−77 %). Αυτό είναι ακριβώς το είδος σφάλματος που στοχεύει το concept
   (Z-drift σε σκάλα), και η καθαρή μετατόπιση παραμένει ουσιαστικά ίδια (89.92 vs 89.94 m),
   δηλαδή η οριζόντια λύση δεν χάλασε.

2. ⚠️ **Εναλλακτική εξήγηση που ΔΕΝ αποκλείεται:** το `z span` έπεσε επίσης, από 7.342 σε
   4.483 m. Αν το πραγματικό υψομετρικό εύρος της διαδρομής είναι ~7 m, τότε ο intensity
   βραχίονας **υπο-εκτιμά** την κατακόρυφη κίνηση και το μικρότερο τελικό σφάλμα είναι
   σύμπτωση συμπίεσης, όχι διόρθωση.

   **Μερικός έλεγχος:** αν επρόκειτο για καθαρή κατακόρυφη συμπίεση με συντελεστή *k*,
   τότε από `7.342·k = 4.483` προκύπτει `k = 0.611`, που θα έδινε τελικό σφάλμα
   `−3.559 · 0.611 = −2.17 m`. Το μετρημένο είναι **−0.833 m**, δηλαδή **2.6× μικρότερο
   από ό,τι προβλέπει η υπόθεση συμπίεσης**. Άρα η βελτίωση **δεν εξηγείται πλήρως** από
   scaling — κάτι δομικό όντως αλλάζει. Παραμένει όμως ένδειξη, **όχι απόδειξη**.

3. **Μόνο το GT κρίνει.** Το `gt-tum.txt` δίνει αμέσως το πραγματικό z-προφίλ και λύνει τη
   διχογνωμία του σημείου 2. **Καμία δημοσιεύσιμη δήλωση δεν στέκει πριν από αυτό.**

4. **Διόρθωση της εγγραφής #003:** είχε αναφερθεί κόστος intensity 2.5× (71→28 Hz). Στο
   σωστά ρυθμισμένο config η επιβάρυνση είναι μόλις **+25 %** (126→158 ms). Η προηγούμενη
   μέτρηση ήταν πολύ μικρός χάρτης, όπου το σταθερό κόστος του KDTree interpolation
   κυριαρχούσε.

5. **Τα διαγνωστικά είναι το bottleneck, όχι το SLAM.** Το `_compute_icp_metrics` χτίζει
   scipy KDTree πάνω σε ολόκληρο τον local map **σε κάθε frame**. Μετρήθηκε:
   build 29 / 110 / 264 ms για 150k / 500k / 1M σημεία, ενώ το query των ~6k σημείων
   κοστίζει μόλις ~2 ms. Με μ. `n_map = 523.123` σημεία, τα διαγνωστικά τρώνε ~110 ms από
   τα 126 ms/frame. **Οι συγκρίσεις χρόνου είναι μολυσμένες** μέχρι να διορθωθεί.

#### Επόμενα βήματα
- [ ] **GT** (`gt-tum.txt`) → z-προφίλ + ATE/RPE. Λύνει το #2· είναι το μόνο που μετράει τώρα.
- [ ] Επανάληψη με `indoor_detail` (voxel 0.25) — το sweep το έδειξε καλύτερα ρυθμισμένο.
- [ ] Φθηνά διαγνωστικά (rebuild KDTree ανά N frames ή query στον voxel map) ώστε οι
      χρόνοι να γίνουν συγκρίσιμοι.
- [ ] Στοχευμένο πείραμα **μόνο στο τμήμα της σκάλας** (`--jump` / `-n`) όπου το concept
      προβλέπει το μεγαλύτερο όφελος.

---

### 2026-09-17 — #004 Voxel sweep → tuned indoor configs

**git commit:** _δεν υπάρχει — εκκρεμεί `git init`._

#### Στόχος
Η εγγραφή #003 έδειξε ότι το default config κρατά **~904 από τα 54.600 σημεία/scan**.
Να βρεθεί, **με μέτρηση και όχι με εκτίμηση**, σημείο λειτουργίας κατάλληλο για εσωτερικό
χώρο, πριν τρέξει οποιοδήποτε συμπερασματικό A/B.

#### Μέθοδος
Σταθερά `max_range=50`, `deskew=true`, `splitting_distance=15 m`· σάρωση του
`odometry.mapping.voxel_size ∈ {1.0, 0.5, 0.25}` (και `local_mapper.voxel_size` ίδιο).
Baseline βραχίονας, 200 scans, ίδιο seed.

Υπενθύμιση μηχανικής: το `source` του ICP προκύπτει από
`voxel_down_sample(frame_downsample, voxel_size * 1.5)`, άρα το πραγματικό βήμα
δειγματοληψίας είναι **1.5 × voxel_size**.

#### Αποτέλεσμα

| voxel | σημεία/scan | σημεία χάρτη | RMS (m) | inlier | μήκος διαδρομής | nodes |
|---:|---:|---:|---:|---:|---:|---:|
| 1.0 | 891 | 41.640 | 0.1898 | 0.9996 | **48.28 m** | 2 |
| 0.5 | 2.498 | 158.302 | 0.1246 | 0.9998 | **44.83 m** | 2 |
| **0.25** | **5.972** | **500.702** | **0.0899** | **0.9999** | **39.38 m** | 2 |

Κόστος: 14 / 46 / 172 ms ανά frame αντίστοιχα.

Απόκλιση τροχιάς ως προς το πιο πυκνό (0.25): voxel 1.0 → mean 15.4 cm / max 30.4 cm·
voxel 0.5 → mean 21.1 cm / max 33.2 cm.

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από χρήστη**

1. **Το μήκος της διαδρομής συρρικνώνεται μονότονα** με την ανάλυση:
   48.28 → 44.83 → **39.38 m**, δηλαδή **−18 % από το πιο χονδρό στο πιο λεπτό**, στα ίδια
   200 frames (~20 s). Σε συνδυασμό με τη μονότονη πτώση του RMS (0.19 → 0.09 m), η πιο
   πιθανή ερμηνεία είναι ότι **τα χονδρά voxels προσθέτουν πλασματική κίνηση** (θόρυβος
   ανά frame που συσσωρεύεται ως μήκος). ⚠️ **Χωρίς GT αυτό παραμένει ερμηνεία, όχι
   απόδειξη** — αλλά είναι ισχυρή ένδειξη ότι το default config παρήγαγε drift.
2. Επιλέχθηκαν **δύο** σημεία λειτουργίας αντί για ένα, και γράφτηκαν στο repo:
   - `configs/indoor_fast.yaml` (voxel 0.5) — για γρήγορες επαναλήψεις.
   - `configs/indoor_detail.yaml` (voxel 0.25) — για συμπερασματικά runs.
3. **Παρενέργεια που τεκμηριώθηκε:** αν το YAML ορίζει `out_dir`, **υπερισχύει** του
   `KISS_SLAM_OUT_DIR` (τα init kwargs του pydantic-settings νικούν τα env vars). Στην
   πρώτη εκτέλεση του sweep αυτό έστειλε τα αποτελέσματα στο `./slam_output` του repo.
   Τα `configs/*.yaml` **σκόπιμα δεν ορίζουν** `out_dir`. Ο Οδηγός εκτέλεσης διορθώθηκε.
4. Το `splitting_distance=15 m` δίνει 2 nodes στα 200 frames — ο loop closer επιτέλους
   ασκείται (με το default 100 m δεν έσπαγε ποτέ, εξ ου και 0 closures παντού).

#### Επόμενα βήματα
- [ ] Πλήρες A/B 2402 frames με `indoor_detail` (σε εξέλιξη).
- [ ] `git init` + πρώτο commit.
- [ ] `gt-tum.txt` → ATE/RPE· μέχρι τότε κάθε σύγκριση βραχιόνων είναι μη συμπερασματική.

---

### 2026-09-17 — #003 macOS περιβάλλον + **πρώτο πραγματικό run** + πρώτο A/B (200 frames)

**git commit:** _δεν υπάρχει — εκκρεμεί `git init`._

#### Στόχος
Να τρέχει το pipeline **τοπικά στο macOS** (ώστε ο επιβλέπων να μπορεί να ξεμπλοκάρει τον
υποψήφιο διδάκτορα χωρίς εξάρτηση από το Linux μηχάνημα), και να γίνει το **πρώτο
πραγματικό A/B** baseline vs intensity.

#### Μέθοδος
Στήθηκε conda env `kissslam` (Py 3.11) και χτίστηκε το τροποποιημένο `kiss_slam` editable
από αυτό το source (πλήρης συνταγή στον Οδηγό εκτέλεσης). Δύο runs × 200 scans, ίδιο
(default) config, μόνη διαφορά η σημαία A/B, έξοδος σε scratch dir μέσω `KISS_SLAM_OUT_DIR`.

**Δύο εμπόδια που λύθηκαν στη διαδρομή:**

1. **`scikit-build-core 1.0.3` σπάει το build**: `ERROR: Use cmake.version instead of
   cmake.minimum-version with scikit-build-core >= 0.8`. Το `pyproject.toml` δηλώνει
   `cmake.minimum-version = "3.22"`, που η 1.0 κατάργησε.
   → Καθηλώθηκε σε **0.12.2** (ίδια με το Linux env), ώστε το build να είναι όσο πιο κοντά
   γίνεται στο γνωστό-καλό. *Εναλλακτικά*, μια γραμμή στο `pyproject.toml`
   (`cmake.version = ">=3.22"`) θα το έλυνε μόνιμα και είναι συμβατή και με την 0.12.2 —
   **δεν** έγινε, για να μην αποκλίνει το source από το Linux χωρίς έγκριση.

2. **Ψευδείς προειδοποιήσεις `matmul`**: κάθε frame τύπωνε
   `RuntimeWarning: overflow / invalid value / divide by zero encountered in matmul`
   στο `slam.py:39` (`transform_points`).
   → **Διερευνήθηκε, δεν αγνοήθηκε.** Με monkey-patch probe γύρω από το `transform_points`
   μετρήθηκε ότι σε **30/30** κλήσεις: `inputs_finite=True`, `output_finite=True`,
   `|pcd|max ≈ 58 m`, `|out|max ≈ 58 m`. Οι πόζες εξόδου: **όλες πεπερασμένες**,
   `max |R Rᵀ − I| = 8.9e-16`. Αιτία: το pip wheel του numpy στο macOS arm64 συνδέεται με
   **Accelerate**, που σηκώνει FP exception flags τα οποία το numpy αναφέρει ως warnings
   παρότι τα αποτελέσματα είναι σωστά. Στο Linux (OpenBLAS) δεν εμφανίζεται.
   → Λύση: numpy από conda-forge με OpenBLAS. **Οι προειδοποιήσεις εξαφανίστηκαν.**

#### Αποτέλεσμα

**Το pipeline τρέχει.** Πρώτο επιτυχές run στην ιστορία του project.

| | Baseline (A) | Intensity (B) |
|---|---|---|
| Ταχύτητα | **71 Hz** (14 ms/frame) | **28 Hz** (35 ms/frame) |
| μ. RMS σφάλμα ICP | 0.2152 m | 0.1745 m |
| μ. inlier ratio | 0.9997 | 0.9999 |
| μ. `n_source_pts` | 904 | 904 |
| μ. `n_filtered_pts` | 904 | **632** (= 70 %) |
| loop closures | 0 | 0 |
| μήκος διαδρομής | 48.153 m | 46.210 m |

**Διαφορά τροχιών A vs B** (200 frames): θέση `mean = 62.0 mm`, `max = 316.7 mm`,
`final = 35.8 mm`· στροφή `mean = 0.547°`, `max = 2.987°`.

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από χρήστη**

1. **Η πρόβλεψη της εγγραφής #002 ήταν λανθασμένη.** Είχα προβλέψει «σχεδόν ταυτόσημες
   τροχιές» λόγω του ευρήματος #2. Στην πράξη οι δύο βραχίονες **αποκλίνουν αισθητά**
   (έως 32 cm σε 200 frames). Το εύρημα #2 παραμένει έγκυρο ως *μεροληψία* (το 2ο ICP
   ευθυγραμμίζεται με χάρτη που ήδη περιέχει το frame), αλλά **δεν** μηδενίζει τη διόρθωση:
   το φιλτραρισμένο source (632 από 904 σημεία) δίνει αρκετά διαφορετικό optimum.
2. ⚠️ **Το χαμηλότερο RMS του B ΔΕΝ είναι απόδειξη βελτίωσης.** Υπολογίζεται πάνω στο
   *φιλτραρισμένο* source — δηλαδή στο 70 % των **φωτεινότερων** σημείων, έναντι χάρτη που
   περιέχει ήδη το frame. Μετράει **άλλο μέγεθος** από το RMS του A. Η σύγκριση είναι
   άκυρη ως έχει.
3. **Χωρίς GT δεν μπορεί να ειπωθεί ποιος βραχίονας είναι καλύτερος.** Οι τροχιές διαφέρουν·
   ποια πλησιάζει την αλήθεια είναι άγνωστο. Το `gt-tum.txt` παραμένει το κρίσιμο βήμα.
4. ⚠️ **Το default config είναι ακατάλληλο για indoor.** Με `max_range=100` ⇒ `voxel_size=1.0`
   ⇒ `source` voxel 1.5 m, τα **54.600 σημεία/scan συρρικνώνονται σε ~904**. Πετιέται
   ~98 % της πληροφορίας — και μαζί ακριβώς οι λεπτομέρειες (σκαλοπάτια, κάγκελα) που
   υποτίθεται ότι θα βοηθήσει το intensity. **Πριν από οποιοδήποτε συμπερασματικό run**
   χρειάζεται `max_range ≈ 50` και `local_mapper.splitting_distance ≈ 10–20 m`.
5. Το κόστος του intensity βραχίονα (2.5×) οφείλεται στο εύρημα #8 (Python loop ανά σημείο).
   Σε πλήρη σεκάνς 2402 frames: ~35 s vs ~85 s — ανεκτό προς το παρόν.

#### Επόμενα βήματα
- [ ] **Tuned config για indoor** (`max_range=50`, `splitting_distance=10–20`) και επανάληψη A/B.
- [ ] `git init` + πρώτο commit.
- [ ] Λήψη `gt-tum.txt` + χρονική ευθυγράμμιση & `base→lidar` extrinsic → ATE/RPE.
- [ ] Διόρθωση της σύγκρισης RMS ώστε να μετράται **ίδιο** μέγεθος και στους δύο βραχίονες.
- [ ] Πλήρες A/B 2402 frames.

---

### 2026-09-17 — #002 Άρση blockers + A/B harness (baseline vs intensity)

**git commit:** _δεν υπάρχει — εκκρεμεί `git init`._

#### Στόχος
Να γίνει το pipeline **εκτελέσιμο** και **μετρήσιμο**: άρση του ImportError blocker και
προσθήκη καθαρού διακόπτη A/B, ώστε baseline και intensity να τρέχουν με **ίδιο config**
και να ξεχωρίζουν στα αποτελέσματα. (Απόφαση χρήστη: μόνο blockers + flag — **όχι**
αναδιάρθρωση της μεθόδου σε αυτό το βήμα.)

#### Μέθοδος
Αλλαγές (μόνο Python, **δεν χρειάζεται rebuild**):

| Αρχείο | Αλλαγή |
|---|---|
| `kiss_slam/point_cloud2.py` → `kiss_slam/tools/point_cloud2.py` | **Μετακίνηση** ώστε να ταιριάζει με το ήδη υπάρχον import· ίδια θέση με το upstream `kiss_icp/tools/point_cloud2.py`. **Αίρει το blocker #1.** |
| `config/config.py` | Νέο `IntensityConfig` (`enabled=False`, `keep_ratio`, `min_intensity`, `lambda_geometric`) ως `KissSLAMConfig.intensity`. |
| `config/__init__.py` | Export `IntensityConfig`, `write_config`. |
| `slam.py` | Όλος ο intensity κώδικας (interpolation, filtering, 2ο ICP, accumulation) περνά πίσω από το `use_intensity`. Παράμετροι από config. **Seeded RNG (`default_rng(0)`)** ώστε ο intensity βραχίονας να είναι αναπαραγώγιμος. |
| `loop_closer.py` | `lambda_geometric` ως όρισμα constructor αντί για module constant. |
| `pipeline.py` | `use_intensity` override· ο reader μπαλώνεται **μόνο** στον intensity βραχίονα· τύπωμα βραχίονα στο stdout· νέο `_write_slam_cfg()` → `slam_config.yaml`. |
| `tools/cli.py` | `--use-intensity / --no-use-intensity`. |

Επαλήθευση (στο macOS, χωρίς `kiss_icp`): φορτώθηκε το `tools/point_cloud2.py` με
`importlib` και τροφοδοτήθηκε με **πραγματικά** `PointCloud2` μηνύματα, αποκωδικοποιημένα
απευθείας από το bag (duck-typed msg objects) → `scratchpad/test_reader.py`.

#### Αποτέλεσμα

`py_compile` OK σε όλα τα τροποποιημένα αρχεία. Ο reader σε 5 πραγματικά scans:

```
scan 0: N= 54629  intensity[min=0.000 p50=0.718 max=1.000]  ts_span=0.1002s  -> OK
scan 1: N= 54374  intensity[min=0.000 p50=0.731 max=1.000]  ts_span=0.1000s  -> OK
scan 2: N= 54573  intensity[min=0.000 p50=0.731 max=1.000]  ts_span=0.0994s  -> OK
scan 3: N= 54463  intensity[min=0.000 p50=0.731 max=1.000]  ts_span=0.0998s  -> OK
scan 4: N= 54197  intensity[min=0.000 p50=0.730 max=1.000]  ts_span=0.1001s  -> OK
RESULT: ALL CHECKS PASSED
```

Έλεγχοι που πέρασαν: σχήμα `(N,3)` float64, `len(timestamps)==len(points)`,
intensity όχι `None`, ίδιο μήκος, εντός `[0,1]`, κανένα NaN.

**Παράπλευρη μέτρηση** που επιβεβαιώνει το εύρημα #5: μόνο **5.9–7.2 %** των σημείων είναι
κάτω από `min_intensity=0.05` ⇒ ο κλάδος «keep-all-bright + random fill» **δεν εκτελείται
ποτέ** σε αυτά τα δεδομένα· τρέχει πάντα ο top-70%-κατά-intensity.

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από χρήστη**

1. Ο blocker ήρθε· ο intensity reader **επαληθεύτηκε σε πραγματικά δεδομένα** — αλλά το
   **pipeline συνολικά δεν έχει ακόμη τρέξει** (απαιτεί το Linux μηχάνημα).
2. Υπάρχει πλέον καθαρό A/B: ίδιο config, μία σημαία, και το `slam_config.yaml` καταγράφει
   τον βραχίονα σε κάθε output dir.
3. **Τα ευρήματα #2–#8 του #001 παραμένουν ανοιχτά** — ιδίως το #2: ο intensity βραχίονας
   αναμένεται να δώσει **σχεδόν ταυτόσημη** τροχιά με το baseline, επειδή το 2ο ICP
   ευθυγραμμίζεται με χάρτη που ήδη περιέχει το frame. **Αυτό είναι χρήσιμο ως πρόβλεψη:**
   αν το A/B δείξει ~μηδενική διαφορά, επιβεβαιώνει το #2 πειραματικά.

#### Επόμενα βήματα
- [ ] `git init` + πρώτο commit (baseline της δουλειάς).
- [ ] Στο Linux: smoke test `-n 200` και στους δύο βραχίονες.
- [ ] Λήψη `gt-tum.txt` (βλ. Οδηγό εκτέλεσης) + χρονική ευθυγράμμιση & `base→lidar` extrinsic.
- [ ] Πλήρες A/B 2402 frames → ATE/RPE.
- [ ] Μετά την επικύρωση: αναδιάρθρωση για το #2 (intensity στο *αρχικό* ICP).

---

### 2026-09-17 — #001 Αρχική καταγραφή dataset & audit της υπάρχουσας υλοποίησης

**git commit:** _δεν υπάρχει — το project **δεν είναι git repo** (`git init` εκκρεμεί)._

#### Στόχος
Πριν από οποιοδήποτε πείραμα: (α) να τεκμηριωθεί τι ακριβώς περιέχει το test bag και αν
το intensity είναι καν διαθέσιμο/χρήσιμο, και (β) να ελεγχθεί η υπάρχουσα intensity-aware
υλοποίηση ώστε να ξέρουμε αν τα μελλοντικά νούμερα σημαίνουν κάτι.

#### Μέθοδος
- Custom ROS1-bag parser (χωρίς `rosbags`, δεν είναι διαθέσιμο στο macOS) για ανάγνωση
  του index, των connections και αποκωδικοποίηση των πρώτων `PointCloud2` μηνυμάτων
  → `scratchpad/bagscan.py`, `bagfields2.py`, `bagcount.py`.
- Στατικό διάβασμα όλων των τροποποιημένων αρχείων και diff έναντι
  `kiss_slam/original_slam_files/`.
- Κατέβασμα του upstream `kiss-icp 1.3.0` sdist για επαλήθευση των συμβάσεων
  (`KissICP.register_frame`, `RosbagDataset.__getitem__`, `OdometryPipeline`).

#### Αποτέλεσμα

**Α. Περιεχόμενο του `church_02_cut.bag`** (9.4 GB, ROS1 V2.0, uncompressed chunks)

| Topic | Type | Msgs | Ρυθμός |
|---|---|---:|---:|
| `/hesai/pandar` | `sensor_msgs/PointCloud2` | **2402** | 10.01 Hz |
| `/alphasense_driver_ros/imu` | `sensor_msgs/Imu` | 95875 | 399.5 Hz |
| `…/cam0,1,2/debayered/image/compressed` | `CompressedImage` | ~4750 έκαστο | ~19.8 Hz |

**Διάρκεια: 240.0 s (4.00 min) ⇒ 2402 LiDAR frames.**

**Β. Δομή του PointCloud2** (`frame_id=pandar`, ~54.6k σημεία/scan, `point_step=48`)

| field | offset | dtype |
|---|---:|---|
| `x`, `y`, `z` | 0/4/8 | float32 |
| **`intensity`** | 16 | **float32** |
| `timestamp` | 24 | float64 (**απόλυτο** Unix time, span ≈ 0.1 s/scan) |
| `ring` | 32 | uint16 |

**✔ Το intensity υπάρχει και είναι αξιοποιήσιμο.** Στατιστικά (πρώτα 3 scans):
`min≈3, max=255, mean≈155, p1≈13, p50≈188, p99=255, ~117 unique τιμές, 0% μηδενικά`.
Δηλαδή **8-bit-like κατανομή με έντονο skew προς τις υψηλές τιμές** και κορεσμό στο 255.

**Γ. Ευρήματα ελέγχου κώδικα** — 8 ζητήματα, ταξινομημένα κατά σοβαρότητα:

| # | Σοβαρότητα | Αρχείο | Εύρημα |
|---|---|---|---|
| 1 | 🔴 **Blocker** | `pipeline.py:104` | `from kiss_slam.tools.point_cloud2 import …` αλλά το αρχείο είναι στο `kiss_slam/point_cloud2.py`. **ImportError στον constructor** (χωρίς try/except) ⇒ το pipeline **δεν έχει τρέξει ποτέ** σε αυτή τη μορφή. |
| 2 | 🔴 **Ακυρώνει το concept** | `slam.py:336-357` | Το δεύτερο ICP τρέχει **μετά** το `register_frame`, το οποίο στο upstream (`kiss_icp.py:70`) έχει ήδη κάνει `local_map.update(frame_downsample, new_pose)`. Άρα το intensity-filtered source ευθυγραμμίζεται με χάρτη **που ήδη περιέχει το ίδιο το frame** ⇒ η διόρθωση τείνει δομικά στο μηδέν. Το intensity filtering **δεν μπορεί να έχει επίδραση** έτσι όπως είναι. |
| 3 | 🟠 Σοβαρό | `slam.py:428-439` + `local_map_graph.py` | **Ασυμφωνία συστημάτων συντεταγμένων.** Ο `_intensity_accumulator` κλειδώνεται με `current_pose` (= `odometry.last_pose`, **τοπικό** πλαίσιο — μηδενίζεται σε κάθε `generate_new_node`), ενώ το `finalize_local_map` κάνει lookup αφού μετασχηματίσει τα σημεία με το `keypose` σε **παγκόσμιο** πλαίσιο ⇒ τα κλειδιά δεν ταιριάζουν, το lookup επιστρέφει ~μηδενικά χρώματα για όλους τους nodes πλην του πρώτου. Το ColoredICP παίρνει σκουπίδια. |
| 4 | 🟠 Σοβαρό | `point_cloud2.py:81` | Η κανονικοποίηση intensity γίνεται με **per-scan** percentiles (1–99). Το ίδιο φυσικό υλικό παίρνει **διαφορετική** τιμή σε κάθε scan ⇒ ασυνεπές για συσσώρευση σε χάρτη και για ColoredICP μεταξύ local maps. Χρειάζεται σταθερή/global κανονικοποίηση (π.χ. `/255`) ή range-compensation. |
| 5 | 🟠 Σοβαρό | `slam.py:68-129` | Το `_intensity_filter` **δεν κάνει αυτό που λέει το docstring**. **Μετρήθηκε στα πραγματικά δεδομένα:** μόνο **5.9–7.2 %** των σημείων πέφτουν κάτω από `min_intensity=0.05` (διάμεσος κανονικοποιημένης intensity ≈ **0.73**). Άρα ~93 % είναι «bright» > `keep_ratio=0.70` ⇒ εκτελείται **πάντα** ο κλάδος top-70%-κατά-intensity. Δηλαδή απλώς **πετάει το 30% των πιο σκοτεινών** επιστροφών (τυπικά μακρινές/λοξές), όχι «ακμές/γωνίες». Δεν υπάρχει καμία σύνδεση με γεωμετρική degeneracy. |
| 6 | 🟡 Μεσαίο | `slam.py:342-357` | Παρενέργειες του 2ου ICP: (α) το `odometry.last_delta` **δεν** ενημερώνεται μετά την αντικατάσταση του `last_pose` ⇒ χαλάει το constant-velocity μοντέλο του επόμενου frame· (β) το `adaptive_threshold.update_model_deviation` καλείται **δεύτερη φορά** ανά frame ⇒ μολύνεται το adaptive σ. |
| 7 | 🟡 Μεσαίο | `slam.py:219-238` | Το `_compute_geometric_degeneracy` υπολογίζει ιδιοτιμές της **συνδιακύμανσης των σημείων του scan** — αυτό μετράει το *σχήμα του όγκου σάρωσης*, όχι την **παρατηρησιμότητα του ICP**. Το σωστό μέτρο είναι η ιδιο-ανάλυση του **Hessian/information matrix του point-to-plane ICP** (Gelfand/Zhang degeneracy factor). Ο τρέχων «staircase detector» μετράει λάθος μέγεθος. |
| 8 | 🟡 Μεσαίο | `slam.py:418-426`, `428-439` | `_accumulate_intensity` / `_get_voxel_intensities`: **Python for-loop πάνω σε ~50k σημεία ανά frame**. Σε 2402 frames κυριαρχεί στον χρόνο εκτέλεσης. Θέλει διανυσματοποίηση (`np.floor_divide` + `np.unique`/`bincount`). |

**Δ. Κενό αξιολόγησης (το κρισιμότερο για το concept)**

Το `OdometryPipeline` του kiss-icp υπολογίζει ATE/RPE **μόνο** αν
`hasattr(dataset, "gt_poses")` (`pipeline.py:67`). Ο `RosbagDataset` **δεν έχει** `gt_poses`.
⇒ **Αυτή τη στιγμή δεν παράγεται κανένα σφάλμα ως προς Ground Truth.**
Το Oxford Spires διαθέτει GT τροχιές (TLS-registered), αλλά **δεν υπάρχουν στο `data/`**.

Επίσης **δεν υπάρχει διακόπτης baseline vs intensity** — όλα είναι hardcoded, άρα δεν
μπορεί να γίνει καθαρό A/B ablation με ίδιο config.

#### Συμπέρασμα

⏳ **ΕΚΚΡΕΜΕΙ επικύρωση από χρήστη**

1. Το **concept είναι υλοποιήσιμο**: το intensity υπάρχει στο bag, είναι float32 με καλή
   δυναμική περιοχή και μηδενικά dropouts. ✔
2. Η **τρέχουσα υλοποίηση δεν μπορεί να το αποδείξει**: ένα blocker (#1) εμποδίζει κάθε
   εκτέλεση, και ακόμη κι αν λυθεί, το #2 καθιστά το intensity filtering **δομικά αδρανές**
   (το ICP ευθυγραμμίζεται με χάρτη που ήδη περιέχει το frame).
3. **Δεν υπάρχει μετρική επιτυχίας**: χωρίς GT poses δεν μπορεί να μετρηθεί αν μειώθηκε
   το drift. Αυτό πρέπει να λυθεί **πρώτο**, αλλιώς κάθε πείραμα είναι μη αξιολογήσιμο.
4. Προτεινόμενη σειρά: **GT + A/B harness → baseline run → σωστό degeneracy metric →
   και μόνο τότε intensity μέθοδος.**

#### Επόμενα βήματα (προτεινόμενα, εκκρεμεί απόφαση)
- [ ] `git init` — χωρίς versioning δεν υπάρχει ιχνηλασιμότητα πειραμάτων.
- [ ] Απόκτηση Oxford Spires GT για `2024-03-18-christ-church-02` + alignment στο κομμένο
      χρονικό παράθυρο (1710754268.98 → +240 s) και υπολογισμός ATE/RPE.
- [ ] Διόρθωση #1 (import path) — smoke test με `-n 200`.
- [ ] Config flag `use_intensity: bool` για καθαρό A/B.
- [ ] Αναδιάρθρωση ώστε το intensity να επηρεάζει το **αρχικό** ICP (όχι post-hoc re-run).

