# Δεδομένα για δοκιμές — τι υπάρχει, πού, τι λείπει (25/9/2026)

Τα αρχεία μένουν όπου κατέβηκαν, στον δίσκο δεδομένων (NTFS, `/media/photogrammetry/A26C3DDF6C3DAF431/data/`, **μόνο ανάγνωση**, #046).
Η οργάνωση είναι **δέντρο συνδέσμων στο ext4**: `/home/photogrammetry/kiss_data/` — καμία εγγραφή στον NTFS, αναστρέψιμο, και ο reader
ανοίγει τους συνδέσμους κανονικά (π.χ. τα 16 bags του long experiment διαβάζονται ως μία ακολουθία, σε σειρά χρόνου, 26 560 σαρώσεις).
Οι παλιές ακολουθίες (Newer College 2020 01_short, 2021, Oxford Spires) μένουν στις διαδρομές του `scripts/results_table.py`.

## Νέες ακολουθίες

| Dataset | Ακολουθία | Φάκελος (`kiss_data/…`) | LiDAR / topic | Διάρκεια | GT | Κατάσταση |
|---|---|---|---|---|---|---|
| Newer College 2020 | 02_long_experiment | `newer_college/2020/02_long_experiment/rosbag/` (16 bags) | OS1-64, `/os1_cloud_node/points` | 2657 s, 26 560 σαρώσεις | `ground_truth/registered_poses.csv`, πλήρης τροχιά (= το ήδη υπάρχον αρχείο) | **έτοιμη** (`--frame=ncd2020`) |
| Newer College 2020 | dynamic_spinning | `newer_college/2020/dynamic_spinning/rosbag/` | OS1-64, `/os1_cloud_node/points` | 120 s, 1202 | `registered_poses.csv` (+ `time_offsets.csv`: κάμερες ↔ IMU, **όχι** LiDAR ↔ GT) | **έτοιμη** (χωρίς offset) |
| Hilti 2021 | LAB_Survey_2 | `hilti_2021/LAB_Survey_2/` | OS0-64 (2048 στήλες), `/os_cloud_node/points` | 136 s, 1357 | πυκνή τροχιά, **στο σύστημα του IMU** | **έτοιμη** (`evaluate_hilti.py`) |
| Hilti 2021 | UZH_Tracking_Area_Run_2 | `hilti_2021/UZH_Tracking_Area_Run_2/` | ίδιο | 89 s, 895 | πυκνή τροχιά, σύστημα IMU | **έτοιμη** |
| Hilti 2021 | Basement_1 | `hilti_2021/Basement_1/` | ίδιο | 113 s, 1130 | 5 σημεία ελέγχου (pole) | **έτοιμη** (`evaluate_hilti.py`, σημεία ελέγχου) |
| Hilti 2021 | IC_Office_1 | `hilti_2021/IC_Office_1/` | ίδιο | 200 s, 2004 | 13 σημεία ελέγχου | **έτοιμη** |
| Hilti 2021 | Office_Mitte_1 | `hilti_2021/Office_Mitte_1/` | ίδιο | 264 s, 2641 | 8 σημεία ελέγχου | **έτοιμη** |
| Hilti 2021 | Construction_Site_1 | `hilti_2021/Construction_Site_1/` | ίδιο | 200 s, 1995 | 10 θέσεις πρίσματος | **έτοιμη** |
| NTU VIRAL | eee_03 | `ntu_viral/eee_03/` (αποσυμπιεσμένο στο ext4) | δύο OS1-16: `/os1_cloud_node1/points` (οριζόντιος), `…node2…` (κατακόρυφος) | — | επίσημο GT `ntuviral_gt/eee_03/ground_truth.csv` (πρίσμα Leica), **σώμα → πρίσμα 0.40 m** | **έτοιμη** (`evaluate_ntu.py`) |
| NTU VIRAL | eee_01, eee_02 | `ntu_viral/eee_01/`, `ntu_viral/eee_02/` (ext4, 9.3 + 7.5 GB) | ίδιο | — | ίδιο | **έτοιμες** |

Έλεγχοι (25/9): κάθε bag ανοίγει (δείκτης ROS1 στο τέλος → όχι κομμένο)· τα 16 bags του long experiment είναι συνεχόμενα (επικάλυψη 0.04 s
μεταξύ τμημάτων) και οι σαρώσεις τους (26 560) = οι θέσεις του GT· πεδία σημείων όπως στα bags του 2021 (`t` σχετικός, 0–99.9 ms·
intensity 0–~1100 → κλίμακα ×255/1024, #041). Hilti: intensity έως ~4500 (p99 443).

## Solid-state: TIERS (2/10, #095) και Hard Point Cloud Localization (2/10, #104)

| Dataset | Ακολουθία | Φάκελος | LiDAR / topic | Διάρκεια | GT | Κατάσταση |
|---|---|---|---|---|---|---|
| TIERS multi-lidar ([github](https://github.com/TIERS/tiers-lidars-dataset), MIT, ακαδημαϊκή χρήση) | Indoor02 (`indoor02_sauna_normal_2022-02-21-19-05-17.bag`, 17 974 826 130 B) | `/media/photogrammetry/Extreme SSD/kiss_data_ssd/tiers/indoor02/` (εξωτερικός SSD, exFAT) | Livox Avia `/avia/livox/lidar`, Livox Horizon `/livox/lidar` (`livox_ros_driver/CustomMsg`: x, y, z, reflectivity, tag, line, offset_time ns· IMU `/avia/livox/imu`, `/livox/imu` 200 Hz)· Ouster OS0 `/os_cloud_node/points` (2048 στήλες, 128 rings), OS1 `/os_cloud_nodee/points`, VLP-16 `/velodyne_points` | 42.3 s, 423 σαρώσεις ανά LiDAR | mocap `indoor02_optitrack.csv` (100 Hz· t ns, x y z, 3 γωνίες, qx qy qz qw) και `/vrpn_client_node/UWBTest/pose` | κατέβηκε 2/10, έλεγχος εικόνας #095· όχι ακόμη στο pipeline |

| Hard Point Cloud Localization ([Zenodo 10122133](https://zenodo.org/records/10122133), CC BY 4.0) | outdoor_hard_01 (`outdoor_hard_01a.zip` 1 642 263 643 B + `outdoor_hard_01b.zip` 1 345 802 065 B, MD5 ελεγμένα) | `/media/photogrammetry/Extreme SSD/kiss_data_ssd/hard_pcl_loc/outdoor_hard_01/bags/` (δύο φάκελοι ROS2 sqlite, χωρίς ορισμούς τύπων) | Livox **Mid-360** (360°, μη επαναλαμβανόμενη)· `/livox/points` PointCloud2 (x y z, t uint32 ns, intensity, tag, line 0–3· ~20 000 σημεία), `/livox/lidar` (`livox_ros_driver2/CustomMsg`, χωρίς ορισμό), `/livox/imu` | 684 s, 999 m, βάδισμα 1.64 m/s, **γωνιακή ταχύτητα p95 73, p99 176, max 504 °/s** | `gt/gt/traj_lidar_outdoor_hard_01.txt` (TUM, **πλαίσιο LiDAR**, 10 Hz, ίδιο ρολόι με την επικεφαλίδα· βελτιστοποίηση LiDAR + IMU, όχι ανεξάρτητη μέτρηση) | κατέβηκε 2/10· reader `kiss_slam/tools/ros2bags.py` (#104) |

**Ρολόγια:** οι επικεφαλίδες των LiDAR και των IMU τους έχουν **χρόνο συσκευής** (δευτερόλεπτα από την εκκίνηση, π.χ. 754–766 s), όχι ROS· μόνο το mocap είναι σε ROS χρόνο.
Κάθε Livox και το IMU του μοιράζονται ρολόι. Μετατροπή σε ROS: + (χρόνος bag − επικεφαλίδα) του πρώτου μηνύματος, μετά αναζήτηση της υπολειπόμενης καθυστέρησης.

## Εντολές (όταν είναι έτοιμες)

```bash
python scripts/run_ncd.py surf /home/photogrammetry/kiss_data/newer_college/2020/02_long_experiment/rosbag <out> --topic=/os1_cloud_node/points
python scripts/evaluate_official.py /home/photogrammetry/kiss_data/newer_college/2020/02_long_experiment/ground_truth/registered_poses.csv <out> --frame=ncd2020
python scripts/run_ncd.py surf /home/photogrammetry/kiss_data/hilti_2021/LAB_Survey_2/rosbag <out> --topic=/os_cloud_node/points
```
Έξοδοι πάντα στο `/home/photogrammetry/kiss_runs/` (ext4).

## Hilti 2021: επίσημη αξιολόγηση (`scripts/evaluate_hilti.py`, 25/9)
Αναπαράγει το `evaluation-evo/evaluation.py` του `Hilti-Research/hilti-slam-challenge-2021`: τροχιά του **IMU**, × `T_imu_ref` ανάλογα με την
κατάληξη του αρχείου GT (`_pole` άκρη του κονταριού 1.67 m, `_prism` 0.27 m, `_imu` ταυτοτικός), αντιστοίχιση εντός 1 s, SE(3), APE θέσης.
Βαθμονόμηση: `kiss_data/hilti_2021/calibration.yaml` (Hugging Face `Hilti-Research/hilti-slam-challenge-2021`, 5.4 KB, «Calibration V2
26.08.2021»)· LiDAR (`os_sensor`) → IMU: περιστροφή ~180° γύρω από το x, 13 cm.
Έλεγχοι: ίδια νούμερα με το επίσημο script σε όλα τα παραδείγματά του (pole, prism, δύο πυκνά IMU — ως το τελευταίο ψηφίο)· η μετατροπή
LiDAR→IMU επιβεβαιώνεται από τη στροφή (με βαθμονόμηση 0.7° / 1.5° διάμεσο σφάλμα, χωρίς 179.7°).
Πρώτα runs (SURF δύο αρχές, 1 σπόρος): LAB_Survey_2 APE 0.035 m (παράδειγμα του dataset hdl_graph_slam 0.053)· UZH_Tracking_Area_Run_2 0.503 m
με αιχμή 9.8 m — η εικόνα αποτυγχάνει σε 453 / 895 σαρώσεις (να εξεταστεί).

```bash
python scripts/evaluate_hilti.py /home/photogrammetry/kiss_data/hilti_2021/<seq>/ground_truth/<seq>_<pole|prism|imu>.txt <run dir> --out=<dir>
```

## Hilti 2021 (όλες οι 12) και 2022 (όλες οι 16) — λήψη 8/10, αξιολόγηση (#238)

Λήψη (Μ.Τ. 8/10: όσες έχουν GT — όλες έχουν) από Hugging Face `Hilti-Research/hilti-slam-challenge-2021` / `-2022` (ανοιχτά, χωρίς σύνδεση) στον εξωτερικό SSD:
`Extreme SSD/hilti_2021/<seq>/{rosbag,ground_truth}/` (οι 6 που έλειπαν: Basement_3, Basement_4, Campus_1, Campus_2, Construction_Site_2 — prism· Parking_1 — pole· 138 GB) και
`Extreme SSD/hilti_2022/<exp>/{rosbag,ground_truth}/` (exp01–07, 09–11, 14–16, 18, 21, 23 σε 3 τμήματα· 336 GB) + `hilti_2022/calibration/lidar_calibration.yaml`, `README.md`.
Μονάδα `kiss-hiltidl` (`kiss_runs/hilti_dl/dl.sh`, συνέχιση / παράλειψη ολοκληρωμένων, έλεγχος μεγέθους) — **ολοκληρώθηκε 9/10 00:33, 24 / 24 bags, χωρίς αποτυχία**· έλεγχος: όλα ανοίγουν, το GT
πέφτει μέσα σε κάθε bag — εξαίρεση exp23: τα 3 τμήματα είναι συνεχόμενα (974 s) αλλά το τελευταίο σημείο ελέγχου είναι ~77 s μετά το τέλος (δεν αντιστοιχίζεται, για καμία μέθοδο)· διάρκειες
2021: Basement_3 331 s, Basement_4 350, Campus_1 430, Campus_2 375, Construction_Site_2 399, Parking_1 582· 2022: 74 (exp14) – 974 s (exp23), 5–22 σημεία ελέγχου ανά ακολουθία.
Λίστα για runs: `kiss_runs/hilti_new.tsv` (22: οι 6 νέες του 2021 + οι 16 του 2022, ίδια μορφή με `all_seqs.tsv`, χωριστά)· το `kiss_runs/q222/common.sh` τη διαβάζει· οι 6 παλιές του 2021
μένουν στο `all_seqs.tsv`. Runs όλων των μεθόδων και στις 28: #239–#241 (`kiss_runs/hilti_241.md`). οι 6 παλιές του 2021 μένουν στο `kiss_data/hilti_2021/`. Για χώρο σβήστηκαν
τα σημεία του KITTI (όλα ήδη διορθωμένα ως προς την κίνηση, #085 / #207: odometry velodyne 85 GB, raw 0027 sync / extract 16 GB)· κρατήθηκαν GT / poses / calib (25 MB).

**2022, αισθητήρας:** Hesai PandarXT-32, `/hesai/pandar`, πεδία x y z intensity (float) timestamp (float64, απόλυτος) ring (uint16), πλαίσιο `PandarXT-32` — η ίδια μορφή με το Spires
(`--topic=/hesai/pandar --intensity-scale=1.0`)· IMU `/alphasense/imu`, 5 κάμερες. LiDAR → IMU: `lidar_calibration.yaml` (quaternion **x, y, z, w** — ελέγχθηκε: το κάθετο του δαπέδου στο LiDAR,
στο πλαίσιο του IMU, απέχει 0.6° από τη βαρύτητα του IMU· η ανάγνωση w, x, y, z θα έδινε 91°).

**2022, επίσημη αξιολόγηση** (`scripts/evaluate_hilti2022.py`, αναπαράγει `evaluation-2022/evaluation.py` + `batch_evaluation.py` του github Hilti-Research/hilti-slam-challenge-2022, αντίγραφα
στο `kiss_data/hilti_2022_eval/`): τροχιά του IMU (TUM)· × T_imu_ref = ταυτοτικός για `*_imu.txt`, αλλιώς η άκρη μέτρησης (0.059, −0.00855, 0.1964) m — **και για τα `*_imu_3dof.txt`** (όπως το
επίσημο· ελέγχθηκε στο exp14: το πυκνό `_imu` με την άκρη πέφτει πάνω στα σημεία του `_imu_3dof`, άρα είναι θέσεις της άκρης)· αντιστοίχιση **2 s**· SE(3)· APE θέσης· **σκορ** ανά σημείο ελέγχου
10 / 6 / 3 / 1 / 0 για σφάλμα < 1 / 3 / 6 / 10 cm / περισσότερο, κανονικοποιημένο σε 0–100 ανά ακολουθία (exp04–06 εκτός επίσημου συνόλου)· πληρότητα. Έλεγχος: ίδια APE με το επίσημο script ως το
9ο δεκαδικό (exp14, `_imu` και `_imu_3dof`). GT: σποραδικά σημεία ελέγχου για όλες· πυκνή τροχιά IMU μόνο exp14 / 16 / 18.
**2021:** `scripts/evaluate_hilti.py` (ήδη, 25/9: αντιστοίχιση 1 s, pole / prism / imu).

```bash
python scripts/evaluate_hilti2022.py "/media/photogrammetry/Extreme SSD/hilti_2022/<exp>/ground_truth/<exp>.txt" <run dir> --out=<dir>
```

## NTU VIRAL: επίσημη αξιολόγηση (`scripts/evaluate_ntu.py`, 25/9)
Αναπαράγει το `ntuviral_evaluate.ipynb` του tutorial (ntu-aris.github.io/ntu_viral_dataset/evaluation_tutorial.html): τροχιά του **σώματος**
(= IMU) εντός του χρόνου του GT, + μετατόπιση **σώμα → πρίσμα (−0.294, −0.012, −0.273) m = 0.40 m** («πολλοί χρήστες ξεχνούν» τη, κατά τη
σελίδα του dataset), αντιστοίχιση εντός 0.05 s, SE(3), ATE = RMSE θέσης· πληρότητα < 90 % με ATE < 20 m → ∞. GT: `kiss_data/ntu_viral/ntuviral_gt/`
(github `ntu-aris/ntuviral_gt`, 4.9 MB, όλες οι 18 ακολουθίες). LiDAR: ο οριζόντιος OS1-16 (`/os1_cloud_node1/points`), `T_Body_Lidar` από
`lidar_horz.yaml`. Έλεγχος: ίδια νούμερα με το επίσημο notebook στα 18 δείγματα FAST-LIO2 του dataset (μέγιστη διαφορά 4·10⁻¹⁶ m).
Πρώτα runs, eee_03 (1 σπόρος): SURF δύο αρχές **ATE 0.548 m**, KISS 0.864 m (το δείγμα FAST-LIO2 του dataset, LiDAR + IMU: 0.102 m)· η εικόνα
των 16 γραμμών αποτυγχάνει στο 31 % των σαρώσεων (566 / 1814).

```bash
python scripts/evaluate_ntu.py eee_03 <run dir> --out=<dir>
```

## Τι χρειάζεται πριν από τα τεστ
1. ~~Hilti 2021: βαθμονόμηση + script~~ — έγινε 25/9 (πάνω). Το πανόραμα 64 × 2048 περνά σε 1024 στήλες (τα σημεία ανά στήλη ↓, λειτουργεί).
   Ανοιχτό: γιατί αποτυγχάνει η εικόνα στο UZH_Tracking_Area_Run_2.
2. ~~dynamic_spinning: time_offsets.csv~~ — **δεν αφορά την αξιολόγηση (25/9):** κατά το dataset είναι οι χρονικές μετατοπίσεις RealSense IMU /
   Ouster IMU ως προς τις κάμερες RealSense (continuous-time calibration)· το GT είναι ανά σάρωση LiDAR, με χρονοσφραγίδες ακριβώς πάνω στις
   σαρώσεις (0.000 ms, 1197 σαρώσεις). Αξιολόγηση χωρίς offset. Δοκιμή ±55 ms (Μ.Τ.): χειροτερεύει όλους τους βραχίονες και προς τις δύο κατευθύνσεις (#064). GT: 40 διπλές γραμμές — αφαιρούνται στο
   `evaluate_ncd.load_gt`.
3. ~~NTU VIRAL: script αξιολόγησης~~ — έγινε 25/9 (πάνω). Ανοιχτό: η εικόνα 16 γραμμών (31 % αποτυχίες)· eee_01/02 αποσυμπιεσμένες 25/9. Δίσκος συστήματος: 31 GB ελεύθερα. (Σελίδα του dataset: «πολλοί ξεχνούν τη μετατόπιση 0.4 m από το IMU στο πρίσμα, όπου
   μετράται το GT»· `leica_prism.yaml`: T_Body_Prism = (−0.294, −0.012, −0.273) m)· GT = `/leica/pose/relative` μέσα στο bag· δύο OS1-16, 16 ακτίνες
   = πανόραμα 16 γραμμών, πιθανό όριο της μεθόδου. eee_01/02: αποσυμπίεση στο ext4 μόλις τελειώσει η λήψη (47 GB ελεύθερα στο ext4).
4. Προσθήκη στο `scripts/results_table.py` (SEQUENCES) όταν υπάρχουν runs.


## NTU VIRAL — όλες οι ακολουθίες (4/10/2026)

Πέρα από τις eee_01–03: **nya_01–03, sbs_01–03, rtp_01–03, tnp_01–03, spms_01–03** (15, 354–584 s, ~60 000 σαρώσεις, OS1-16 οριζόντιος `/os1_cloud_node1/points`).
Αρχεία: DR-NTU (Data) `researchdata.ntu.edu.sg/api/access/datafile/<id>`, αποσυμπιεσμένα στον εξωτερικό SSD `kiss_data_ssd/ntu_viral/<seq>/` (zip διαγραμμένα, ΑΠΟΦΑΣΗ Μ.Τ.),
σύνδεσμοι στο `~/kiss_data/ntu_viral/<seq>`· GT στο `ntuviral_gt/<seq>/ground_truth.csv` (ήδη). Στο `kiss_runs/all_seqs.tsv` (42 γραμμές· οι 27 παλιές στο `all_seqs_27.tsv`).
**Προσοχή:** rtp / tnp / spms έχουν την παλιά μορφή βαθμονόμησης (`T_Body2Lidar`, το πρίσμα ως `T_Body2Imu`, ίδιες τιμές) — το `evaluate_ntu.py` δέχεται πλέον και τα δύο ονόματα.
spms: πληρότητα 99 % σε όλα τα runs (και του KISS).


## Oxford Spires — οι υπόλοιπες ακολουθίες με GT (8/10)

Λήψη από τον φάκελο Google Drive του dataset (Μ.Τ. 8/10), **μόνο όσες έχουν `gt-tum.txt`** (ΑΠΟΦΑΣΗ Μ.Τ. 8/10: όσες δεν έχουν GT δεν χρειάζονται)· στον εξωτερικό SSD
`Extreme SSD/oxford_spires/<ακολουθία>/{rosbag,ground_truth}/` (ROS1 bags + `gt-tum.txt`, και VILENS / HBA / COLMAP όπου υπάρχουν). Ελέγχθηκαν: ανοίγουν, `/hesai/pandar` (Hesai QT64), το GT καλύπτει όλη τη διάρκεια.

| ακολουθία | όνομα run | bags | διάρκεια s | σαρώσεις |
|---|---|---|---|---|
| 2024-03-12-keble-college-02 | `keble_02` | 1 | 300 | 3007 |
| 2024-03-12-keble-college-04 | `keble_04` | 2 | 681 | 6828 |
| 2024-03-12-keble-college-05 | `keble_05` | 2 | 581 | 5834 |
| 2024-03-13-observatory-quarter-02 | `observatory_02` | 1 | 275 | 2755 |
| 2024-03-14-blenheim-palace-01 | `blenheim_01` | 1 | 404 | 4052 |
| 2024-03-14-blenheim-palace-05 | `blenheim_05` | 1 | 339 | 3401 |
| 2024-03-20-christ-church-05 | `church_05` | 1 | 800 | 8007 |

Λίστα για runs: `kiss_runs/spires_new.tsv` (ίδια μορφή με `all_seqs.tsv`, **χωριστά** ώστε το «όλες οι 42» να μη αλλάξει)· βαθμολόγηση: `results_table.py` (frame `spires`, runs `oxford_spires_full/<όνομα>`).
Με τις 6 παλιές: 13 ακολουθίες Spires με GT. Χωρίς GT στο Drive (δεν κατέβηκαν): keble-college-01, blenheim-palace-03 / 04, bodleian-library-01, christ-church-01 / 04 / 06, new-college-01…04
(πλήρης λίστα αρχείων `kiss_runs/spires_dl/order_all.tsv`).

## Οχήματα (1–5/10)

- **KITTI odometry 07** (#084, #085): οι σαρώσεις του raw / sync είναι **ήδη διορθωμένες** ως προς την κίνηση — ακατάλληλο για δοκιμή deskew. Δεδομένα `Extreme SSD/kitti/`.
- **Boreas** `boreas-2021-01-26-11-22`, πρώτες 3000 σαρώσεις (311 s, 1.34 km), Velodyne Alpha Prime 128, raw με ring και χρόνο ανά σημείο (#085). Δεδομένα `Extreme SSD/boreas/`, GT
  `gt_boreas-2021-01-26-11-22_lidar_tum.txt` (ανά σάρωση)· runs `kiss_runs_ssd/boreas/2021-01-26-11-22_3000/`· `run_ncd.py ... --last=3000 --intensity-scale=1.0`· `oracle_motion.npz` (#127).
- **Boreas, 8 πλήρεις οδηγήσεις (7/10, #216, #225):** 2020-12-01-13-26, 2021-01-15-12-17, 2021-01-19-15-08, 2021-04-08-12-44, 2021-09-07-09-35, 2021-09-14-20-00, 2021-10-15-12-35, 2021-11-16-14-10
  (οι «δύσκολες»: χιόνι / βροχή / νύχτα / αυτοκινητόδρομος· ~8 km η καθεμία). Δεδομένα `Extreme SSD/boreas/boreas-<seq>/lidar/*.bin`, GT `gt_boreas-<seq>_lidar_tum.txt` (`scripts/boreas_gt.py`)·
  runs `kiss_runs_ssd/boreas/<seq>/`· `run_ncd.py ... --intensity-scale=1.0` (χωρίς `--last`). Μεροληψία ανύψωσης δεσμών ~+0.05–0.15° (κοινή σε όλες τις μεθόδους, #219 / #221, `--elev-offset`)·
  για το paper **χωρίς διόρθωση** (ΑΠΟΦΑΣΗ Μ.Τ. 7/10).
- **MulRan (7/10, #208–#217, #223):** KAIST01, DCC01 (αστικές), Riverside01, Sejong01 (αυτοκινητόδρομος), Ouster OS1-64 σε όχημα. Δεδομένα `Extreme SSD/mulran/<seq>/` (`Ouster/<ns>.bin` = x, y, z, intensity,
  πάντα 64 × 1024 στήλη-στήλη: δακτύλιος i % 64, χρόνος στήλη / 1024 × 0.1 s — άρα deskew δυνατό), GT `global_pose.csv` → `gt_lidar_tum.txt` (`scripts/mulran_gt.py`, βαθμονόμηση base → Ouster του KISS-ICP).
  Reader `kiss_slam/tools/mulran.py` (αναγνώριση από `run_ncd.py` / `run_baseline.py`)· runs `kiss_runs_ssd/mulran/<seq>/`. Η τελευταία σάρωση του KAIST01 είναι κομμένη (#208, χειρίζεται ο reader).
