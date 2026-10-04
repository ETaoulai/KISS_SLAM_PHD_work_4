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
