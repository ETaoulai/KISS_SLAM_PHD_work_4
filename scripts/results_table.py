#!/usr/bin/env python3
"""All results in one table: every sequence x arm, mean ± σ over its runs, against the ground truth.

    python scripts/results_table.py [<data root>] [--out=<prefix>] [--all-arms] [--extra=/home/photogrammetry/kiss_runs]
                                    [--runs=<dir>[,<dir>...]]
                                    [--legacy [--offset=best]]

Reads the run folders under <data root>/runs (default /media/photogrammetry/A26C3DDF6C3DAF431/data) and under <extra>,
and under every --runs dir (same layout as <extra>, e.g. the external SSD of #081; read only, nothing is written there),
and writes <prefix>.md and <prefix>.csv (on ext4: the NTFS data disk is read-only, #052).
Sequences: Newer College 2020 01_short (#041), 2021 (#043, #052), Oxford Spires (#044, #052): 16 in all.

Evaluation (DECISION M.T. 25/9): the official protocol ONLY — the Oxford Spires benchmark with evo, every pose at the
instant it stands for, no time offset (scripts/evaluate_official.py, #061); default <prefix> results_official, the
estimates / GT as TUM under <extra>/official_eval/<sequence>/ for the evo command line.
--legacy: the evaluation of #041-#060 (evaluate_ncd.py, GT interpolated at the scan stamps; --offset=best: each run at
its own best time shift, #045) — only to reproduce old tables, never for new comparisons.

Arms (DECISION M.T. 25/9): KISS-SLAM, KISS-SLAM no deskew, i3 + SIFT, i3 + SURF, i3 + SURF two starting points, and
two starting points with a switch margin (#062).  --all-arms: every arm with runs (ablations, indoor_detail, ...).
Arms are the run-folder names up to the first "_"; folders that do not exist are skipped.
"""
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from evaluate_gt import find_tum  # noqa: E402
from evaluate_ncd import evaluate, load_gt  # noqa: E402
from evaluate_hilti import evaluate_hilti  # noqa: E402
from evaluate_ntu import evaluate_ntu  # noqa: E402
from evaluate_official import evaluate_official  # noqa: E402

ROOT = Path(next((a for a in sys.argv[1:] if not a.startswith("--")), "/media/photogrammetry/A26C3DDF6C3DAF431/data"))
LEGACY = "--legacy" in sys.argv
OFFSET = "best" if LEGACY and "--offset=best" in sys.argv else 0.0
OFFICIAL = not LEGACY
NC, RUNS = ROOT / "newer_college", ROOT / "runs"
EXTRA = Path(next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--extra=")), "/home/photogrammetry/kiss_runs"))
# More run roots with the layout of EXTRA (#081: the external exFAT SSD), read only.
MORE_RUNS = [Path(d) for a in sys.argv[1:] if a.startswith("--runs=") for d in a.split("=", 1)[1].split(",") if d]
# Written to EXTRA (ext4), never to the NTFS data disk: ntfs3 kernel BUG on writes (#046, #052; decision M.T. 24/9).
OUT = Path(next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--out=")),
                EXTRA / ("results_official" if OFFICIAL else "results_all_best_offset" if OFFSET == "best" else "results_all")))

# (dataset, sequence, sensor, ground truth, frame, runs folder)
SEQUENCES = [("Newer College 2020", "01_short", "Ouster OS1-64", NC / "2020/01_short_experiment", "ncd2020",
              RUNS / "newer_college_01_short")]
for seq, gt in [("quad_easy", "collection 1 - newer college/ground_truth/tum_format/gt-nc-quad-easy.csv"),
                ("stairs", "collection 1 - newer college/ground_truth/tum_format/gt-nc-stairs.csv"),
                ("cloister", "collection 2 - newer college/ground_truth/tum_format/gt-nc-cloister.csv"),
                ("math_easy", "collection 3 - maths institute/ground_truth/tum_format/gt_math_easy.csv"),
                ("underground_easy", "collection 4 - underground mine/ground truth/tum_format/easy_gt_state_tum.csv")]:
    SEQUENCES.append(("Newer College 2021", seq, "Ouster OS0-128", NC / "2021" / gt, "ncd2021", RUNS / "newer_college_2021" / seq))
for n in (2, 3):
    SEQUENCES.append(("Oxford Spires", f"christ-church-0{n}", "Hesai QT64",
                      ROOT / f"oxford_spires/2024-03-18-christ-church-0{n}/ground_truth/gt-tum_church_{n}.txt", "spires",
                      RUNS / "oxford_spires_full" / f"church_0{n}"))
# #052: harder Newer College 2021 sequences and four more Oxford Spires sites
for seq, gt in [("quad_hard", "collection 1 - newer college/ground_truth/tum_format/gt-nc-quad-hard.csv"),
                ("math_medium", "collection 3 - maths institute/ground_truth/tum_format/gt_math_medium.csv"),
                ("underground_medium", "collection 4 - underground mine/ground truth/tum_format/medium_gt_state_tum.csv"),
                ("underground_hard", "collection 4 - underground mine/ground truth/tum_format/hard_gt_state_tum.csv")]:
    SEQUENCES.append(("Newer College 2021", seq, "Ouster OS0-128", NC / "2021" / gt, "ncd2021", RUNS / "newer_college_2021" / seq))
for seq, folder, gt in [("keble-college-03", "2024-03-12-keble-college-03", "gt-tum_keeble_3.txt"),
                        ("observatory-quarter-01", "2024-03-13-observatory-quarter-01", "gt-tum_observatory_quarter_1.txt"),
                        ("blenheim-palace-02", "2024-03-14-blenheim-palace-02", "gt-tum_blenheim_pallace.txt"),
                        ("bodleian-library-02", "2024-05-20-bodleian-library-02", "gt-tum_bodleian_library_2.txt")]:
    run = {"keble-college-03": "keble_03", "observatory-quarter-01": "observatory_01", "blenheim-palace-02": "blenheim_02",
           "bodleian-library-02": "bodleian_02"}[seq]
    SEQUENCES.append(("Oxford Spires", seq, "Hesai QT64", ROOT / f"oxford_spires/{folder}/ground_truth/{gt}", "spires",
                      RUNS / "oxford_spires_full" / run))
# 8/10: the other Oxford Spires sequences with a ground truth, on the external SSD (docs/datasets.md, kiss_runs/spires_new.tsv)
SSD_SPIRES = Path("/media/photogrammetry/Extreme SSD/oxford_spires")
for folder, run in [("2024-03-12-keble-college-02", "keble_02"), ("2024-03-12-keble-college-04", "keble_04"), ("2024-03-12-keble-college-05", "keble_05"),
                    ("2024-03-13-observatory-quarter-02", "observatory_02"), ("2024-03-14-blenheim-palace-01", "blenheim_01"),
                    ("2024-03-14-blenheim-palace-05", "blenheim_05"), ("2024-03-20-christ-church-05", "church_05")]:
    SEQUENCES.append(("Oxford Spires", folder[11:], "Hesai QT64", SSD_SPIRES / folder / "ground_truth" / "gt-tum.txt", "spires",
                      RUNS / "oxford_spires_full" / run))

# New datasets (25/9, docs/datasets.md): organised as links in KD (ext4); Hilti 2021 and NTU VIRAL scored with THEIR official
# protocols (scripts/evaluate_hilti.py, evaluate_ntu.py): one number per run, the challenge's APE / the dataset's ATE.
KD = Path("/home/photogrammetry/kiss_data")
for seq in ("02_long_experiment", "dynamic_spinning"):
    SEQUENCES.append(("Newer College 2020", seq, "Ouster OS1-64", KD / f"newer_college/2020/{seq}/ground_truth/registered_poses.csv",
                      "ncd2020", RUNS / f"newer_college_2020/{seq}"))
for seq, ref in [("Basement_1", "pole"), ("IC_Office_1", "pole"), ("Office_Mitte_1", "pole"), ("Construction_Site_1", "prism"),
                 ("LAB_Survey_2", "imu"), ("UZH_Tracking_Area_Run_2", "imu")]:
    SEQUENCES.append(("Hilti 2021", seq, "Ouster OS0-64", KD / f"hilti_2021/{seq}/ground_truth/{seq}_{ref}.txt", "hilti",
                      RUNS / f"hilti_2021/{seq}"))
for seq in ("eee_01", "eee_02", "eee_03"):
    SEQUENCES.append(("NTU VIRAL", seq, "Ouster OS1-16 (horizontal)", KD / f"ntu_viral/ntuviral_gt/{seq}/ground_truth.csv", "ntu",
                      RUNS / f"ntu_viral/{seq}"))

ARMS = {"kissncd": "KISS-SLAM, kiss_icp NCD loader", "kissnodeskew": "KISS-SLAM, no deskew", "kissdetail": "KISS-SLAM, indoor_detail", "kiss": "KISS-SLAM",
        "sift": "i3 + SIFT", "surf": "i3 + SURF", "surftrans": "i3 + SURF, translation only",
        "surfrot": "i3 + SURF, rotation only", "surfsmooth3": "i3 + SURF, rotation smoothed (3)",
        "surfh25": "i3 + SURF, threshold 25", "surfh10": "i3 + SURF, threshold 10", "surfgate": "i3 + SURF, gated", "surfgate25": "i3 + SURF, gated 25/20",
        "surfgate25id": "i3 + SURF, gated 25/20, identity fallback", "surftwo": "i3 + SURF, two starting points",
        "surfrangefb": "i3 + SURF, range when intensity fails", "surftworange": "i3 + SURF, three starts (+ range)",
        "surftwom2": "i3 + SURF, two starting points, margin 2 %", "surftwom4": "i3 + SURF, two starting points, margin 4 %",
        "surftwodetail": "i3 + SURF, two starting points, indoor_detail (one-off, stairs)",
        "kissdetailnodeskew": "KISS-SLAM, indoor_detail, no deskew (one-off, stairs)",
        "surftworangefb": "i3 + SURF, two starting points, range image when intensity fails",
        # #067 confirmation: pose-graph rotation weight x100 (only the sequences with loop closures)
        "kissrw": "KISS-SLAM, rotation weight 100", "kissnodeskewrw": "KISS-SLAM, no deskew, rotation weight 100",
        "siftrw": "i3 + SIFT, rotation weight 100", "surfrw": "i3 + SURF, rotation weight 100",
        "surftworw": "i3 + SURF, two starting points, rotation weight 100",
        "surftworangecand": "i3 + SURF, two starts + range image as third start",
        "surftwos1": "i3 + SURF, two starting points, intensity scale 1.0 (#075)",
        "surftworangefbs1": "i3 + SURF, two starts + range fallback, intensity scale 1.0 (#075)",
        "surftworangecands1": "i3 + SURF, two starts + range as third start, intensity scale 1.0 (#075)",
        "surftwogain": "i3 + SURF, two starting points, per-scan gain (#076)",
        "surftwogainw": "i3 + SURF, two starting points, per-scan gain, panorama 2048 (#076)",
        "surftworangefbfast": "i3 + SURF, two starting points, fast range fallback (#078)",
        "surftwofull": "i3 + SURF, two starts + gain + fast range fallback + rotation weight 100 (#079)",
        "surftwofbnostuck": "i3 + SURF, two starts + range fallback, no near-floor stuck-match filter (ablation #081)",
        "surftwofbadaptive": "i3 + SURF, two starts + range fallback, KISS adaptive sigma instead of fixed 2.0 (ablation #081)",
        "surftwofbcvstart": "i3 + SURF, two starts + range fallback, image motion for deskew only, ICP from constant velocity (ablation #081)",
        "surftwofbnodeskew": "i3 + SURF, two starts + range fallback, image motion as ICP start only, no deskew (#082)",
        "genz": "GenZ-ICP (odometry, own config, #083)", "mad": "MAD-ICP (odometry, own config, #083)",
        "trajlo": "Traj-LO (continuous-time odometry, own config, #083)",
        "surftworangefbodo": "i3 + SURF, two starts + range fallback, odometry only (no loop closures, replay_backend none, #083)",
        "cticp": "CT-ICP (continuous-time odometry, robust low-inertia profile, #083)",
        "cticpdriving": "CT-ICP (continuous-time odometry, driving profile, #083)",
        "coinlio": "COIN-LIO (LiDAR-inertial, intensity + IMU, reference, #083)",
        "fastlio": "FAST-LIO2 (LiDAR-inertial, reference, #083)",
        "fastlioblind1": "FAST-LIO2 blind 1 m (LiDAR-inertial, reference, stairs only, #083)",
        "dlo": "DLO (LiDAR-only odometry, authors' config with imu false, #085)",
        "rotmodelcv": "i3 + SURF, two starts + range fallback, motion model cv (rotation #086)",
        "rotdeskewcv": "i3 + SURF, two starts + range fallback, deskew rotation from constant velocity (rotation #086)",
        "rotredeskew": "i3 + SURF, two starts + range fallback, second deskew pass with ICP motion (rotation #086)",
        "rotsmooth3": "i3 + SURF, two starts + range fallback, image rotation smoothed over 3 scans (rotation #086)",
        "oracledeskew": "i3 + SURF, two starts + range fallback, DESKEW FROM GROUND TRUTH (diagnostic, #086)",
        "upright": "i3 + upright SURF, two starts + range fallback (#086)",
        "guided": "i3 + upright SURF + guided matching C++, two starts + range fallback (#087)",
        "surftwofbv2": "NEW DEFAULT: upright SURF + guided matching, two starts + range fallback (#088)",
        "surftwofbv2b": "NEW DEFAULT b: upright SURF + guided matching only when turning slowly (#088)",
        "surftwofbv3": "upright SURF + guided matching, window at the motion-predicted position (#089)",
        "surftwofbv4": "upright SURF + guided matching, HYBRID: shift when slow, motion prediction when fast (#089)",
        "base092": "default of after_091 (upright SURF + guided matching, shift rule), two starts + range fallback (#092)",
        "rotcv05": "default of after_091, image rotation blended with constant velocity w = 0.05 (#092)",
        "rotcv10": "default of after_091, image rotation blended with constant velocity w = 0.1 (#092)",
        "up2093": "default of after_091, panorama upscale 2 instead of 8 (square pixels, Ouster 128, #093)",
        "autowh093": "default of after_091, panorama from the sensor: own columns + square pixels, auto (#093)",
        "kissfloor108": "default of after_091, second start (and image-failure fallback) = KISS itself, CV deskew + start (#108)",
        "valid109": "KISS floor (#108) + per-scan range validation of the image motion (range validate, k = 3) (#109)",
        "always109": "KISS floor (#108) + every scan registered from all starts, best map fit (#109)",
        "autow111": "default of after_091, panorama columns of the sensor only (upscale 8 as before) (#111)",
        "rsmooth121": "default of after_091, intensity / smooth range curve of the scan (range_smooth, #118 / #121)",
        "rsq121": "default of after_091, intensity x r^2 (range2, Ouster sensors only, #118 / #121)",
        "blend131": "default of after_091, adaptive image / constant-velocity blend, deskew + ICP start (#131 / #133)",
        "multit131": "default of after_091, translation from the k-2 joint fit, rotation two-scan (#131 / #133)",
        "sec132": "default of after_091, sector weights (8) in the motion fit (#132 / #134)",
        "fr132": "default of after_091, range-panorama matches fused into the motion fit (#132 / #135)",
        "ds132": "default of after_091, drop pairs zero motion explains (#132 / #135)",
        "sr135": "default of after_091, sector weights for the rotation only (#135 / #139)",
        "bd137": "default of after_091, adaptive blend for the DESKEW only, ICP start = image (#137 / #138 / #140)",
        "cmb141": "default of after_091, COMBINATION: blend for the deskew + sector weights for the rotation (#141 / #142 / #144)",
        "fbk146": "default of after_091, fallback = KISS on scans where the image fails (#146 / #147)",
        "bf148": "default of after_091, PAIR: blend for the deskew + fallback = KISS (#148 / #149)",
        "bfc150": "default of after_091, OPTION C: blend for the deskew + fallback = KISS from the 4th consecutive failure (#150 / #151 / #164)",
        "blinl203": "locked method C + blend weighted also by the scan inliers (--cv-blend-inliers, #200 / #203)",
        "r512": "locked method C, panorama 512 rows (x4 on 128 beams, x32 on 16 beams; #204)",
        "w600m": "locked method C, Hesai native 600 columns x 512 rows (#205)",
        "r512c": "locked method C, 512 rows + C++ image + look-ahead + range cache + KISS-ICP without GIL (#206)",
        "w600c": "locked method C, Hesai 600 x 512 + C++ image + look-ahead + range cache + KISS-ICP without GIL (#206)",
        "car222": "locked method C + car configuration (ICP start from the blend + two-start translation trigger 0.2, #217 / #222)",
        "ral228": "locked method C (new Oxford Spires sequences, #228 / #229)",
        "kiss228": "KISS-SLAM (new Oxford Spires sequences, #228 / #229)",
        "klt230": "locked method C with 1D KLT flow on the native panorama (no upscaling) instead of SURF (#230)",
        "klt2d231": "locked method C with 2D pyramidal KLT flow (Shi-Tomasi corners) instead of SURF (#231)",
        "uorb231": "locked method C with upright ORB instead of SURF (#231)",
        "cub234": "locked method C with the cubic motion model (rotation and translation, #234)",
        "magsac236": "locked method C with MAGSAC++-style robust estimation (#235 / #236)",
        "gnc236": "locked method C with GNC-TLS instead of RANSAC (#236)",
        "multi236": "locked method C with two-model fitting against sensor-fixed patterns (#236)"}
# The arms compared from 25/9 on (decision M.T.): no indoor_detail, no ablations (rotation smoothed, translation only, ...).
MAIN_ARMS = ["kiss", "kissnodeskew", "sift", "surf", "surftwo", "surftwom2", "surftwom4"]
if "--all-arms" not in sys.argv:
    ARMS = {k: ARMS[k] for k in MAIN_ARMS}
METRICS = [("ate", "ATE [m]", "{:.3f}"), ("rpe_t", "RPE 1 s [cm]", "{:.2f}"), ("rpe_r", "RPE 1 s [°]", "{:.3f}"),
           ("path", "path [m]", "{:.1f}"), ("excess", "path vs GT [%]", "{:+.1f}"), ("z_rmse", "z RMSE [m]", "{:.3f}"),
           ("kitti", "KITTI [%]", "{:.2f}"), ("fail", "image fails", "{:.0f}"), ("offset", "time shift [s]", "{:+.3f}")]
if OFFICIAL:
    METRICS = [("ate", "APE [m]", "{:.3f}"), ("rpe_t", "RPE 1 m [cm]", "{:.2f}"), ("rpe_r", "RPE 1 m [°]", "{:.3f}"),
               ("rpe1s_t", "RPE 1 s [cm]", "{:.2f}"), ("rpe1s_r", "RPE 1 s [°]", "{:.3f}"), ("rte", "RTE [%]", "{:.2f}"),
               ("rre", "RRE [°/100 m]", "{:.3f}"), ("path", "path [m]", "{:.1f}"),
               ("excess", "path vs GT [%]", "{:+.1f}"), ("z_rmse", "z RMSE [m]", "{:.3f}"), ("fail", "image fails", "{:.0f}"),
               ("matched", "assoc.", "{:.2f}"), ("pose_minus_stamp", "pose − stamp [s]", "{:+.4f}"), ("gt_interp", "GT interp.", "{:.0f}")]


CACHE_VERSION = 1      # bump when evaluate_ncd.evaluate changes what it computes
OFFICIAL_VERSION = 6   # bump when evaluate_official.evaluate_official changes what it computes (5: RTE/RRE; 6: NTU association fix only when needed, #083)


def cached_evaluate(gt, frame, gt_t, gt_T, run):
    """evaluate_ncd.evaluate, cached in EXTRA/.eval_cache (ext4).  The key covers the run's trajectory file (path, size,
    mtime), its log (image-motion failures, runtime), the ground-truth file (path, size, mtime), the frame, the time-shift
    mode and CACHE_VERSION, so any change to one of them is scored again."""
    import hashlib
    import json

    tum, log, gtp = Path(find_tum(run)), run.parent / f"{run.name}.log", Path(gt)
    gtf = gtp / "ground_truth" / "registered_poses.csv" if gtp.is_dir() else gtp
    stat = lambda f: f"{f.resolve()}:{f.stat().st_size}:{f.stat().st_mtime_ns}" if f.exists() else f"{f}:-"
    parts = [stat(tum), stat(log), stat(gtf), frame, str(OFFSET), str(CACHE_VERSION)]
    if OFFICIAL:     # also what sets the pose times: the exact file, the configs, the two-start log
        extra = [f for pat in ("*_poses_posetime_tum.txt", "config.yml", "slam_config.yaml", "two_start.csv")
                 for f in sorted(run.glob(f"*/{pat}"))[-1:]]
        parts = parts + ["official", str(OFFICIAL_VERSION)] + [stat(f) for f in extra]
    key = hashlib.sha1("|".join(parts).encode()).hexdigest()
    path = EXTRA / ".eval_cache" / f"{key}.json"
    if path.exists():
        return json.loads(path.read_text())
    if frame in ("hilti", "ntu"):          # the dataset's own protocol: one score per run
        if frame == "hilti":
            h = evaluate_hilti(gt, run, out_dir=EXTRA / "official_eval" / run.parent.name)
            v = dict(ate=h["rmse"], matched=h["n_matched"] / h["n_ref"] if h["kind"] != "imu.txt" else float("nan"))
        else:
            h = evaluate_ntu(run.parent.name, run, out_dir=EXTRA / "official_eval" / run.parent.name)
            v = dict(ate=h["ate"], matched=h["completeness"] / 100)
        m = re.search(r"image motion: \d+/\d+ scans \((\d+) fell back", log.read_text(errors="replace")) if log.exists() else None
        v["fail"] = float(m[1]) if m else float("nan")
        v = {k: float(v.get(k, float("nan"))) for k in
             ("ate", "rpe_t", "rpe_r", "rpe1s_t", "rpe1s_r", "rte", "rre", "path", "gt_path", "z_rmse", "fail", "matched", "pose_minus_stamp", "gt_interp")}
    elif OFFICIAL:
        v = evaluate_official(gt_t, gt_T, run, frame, EXTRA / "official_eval" / run.parent.name)
        m = re.search(r"image motion: \d+/\d+ scans \((\d+) fell back", log.read_text(errors="replace")) if log.exists() else None
        v["fail"] = float(m[1]) if m else float("nan")
        v = {k: float(x) for k, x in v.items()}
    else:
        v = {k: float(x) for k, x in evaluate(gt_t, gt_T, run, OFFSET).items()}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(v))
    return v


def main():
    rows = []
    for dataset, seq, sensor, gt, frame, folder in SEQUENCES:
        # Also the same folder under EXTRA (runs written to ext4 after the ntfs3 kernel bug, #046); a run counts
        # only if its log ends with the "wall" line of /usr/bin/time, i.e. it finished (the crashed ones did not).
        found = {}
        for base in (folder, *(r / folder.relative_to(RUNS) for r in (EXTRA, *MORE_RUNS))):
            for p in (sorted(base.glob("*_*")) if base.exists() else []):
                log = p.parent / f"{p.name}.log"
                # ... and it wrote a trajectory: a crashed run also has the "wall" line (the exFAT symlink crash of #081)
                txt = log.read_text(errors="replace").replace("\r", "\n") if log.exists() else ""
                # #203: or the end-of-run summary of the pipeline (runs launched without /usr/bin/time)
                if (p.is_dir() and log.exists() and ("\nwall " in txt or "KissSLAM| image motion:" in txt)
                        and any(p.glob("*/*_poses_tum.txt"))):
                    found[p.name] = p
        runs = [found[k] for k in sorted(found)]
        if not runs:
            print(f"skip {seq}: no runs in {folder}")
            continue
        gt_t, gt_T, _ = load_gt(gt, frame) if frame not in ("hilti", "ntu") else (None, None, frame)
        res = {r.name: cached_evaluate(gt, frame, gt_t, gt_T, r) for r in runs}
        for v in res.values():
            v["excess"] = 100 * (v["path"] / v["gt_path"] - 1)
        gt_path = next(iter(res.values()))["gt_path"]
        for arm in ARMS:
            vs = [v for n, v in res.items() if n.split("_")[0] == arm]
            if not vs:
                continue
            row = dict(dataset=dataset, sequence=seq, sensor=sensor, gt_path=gt_path, arm=ARMS[arm], runs=len(vs))
            for c, _, _ in METRICS:
                x = np.array([v[c] for v in vs], float)
                row[c], row[c + "_sd"] = np.nanmean(x) if np.isfinite(x).any() else np.nan, (np.std(x, ddof=1) if len(vs) > 1 else np.nan)
            rows.append(row)
        print(f"{seq}: {len(runs)} runs")

    with open(OUT.with_suffix(".csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dataset", "sequence", "sensor", "gt_path_m", "arm", "runs"] + sum([[c, c + "_sd"] for c, _, _ in METRICS], []))
        for r in rows:
            w.writerow([r["dataset"], r["sequence"], r["sensor"], f"{r['gt_path']:.1f}", r["arm"], r["runs"]]
                       + sum([[f"{r[c]:.4f}", "" if np.isnan(r[c + "_sd"]) else f"{r[c + '_sd']:.4f}"] for c, _, _ in METRICS], []))

    def cell(r, c, f):
        if np.isnan(r[c]):
            return "—"
        s, sd = f.format(r[c]), r[c + "_sd"]
        return s + (" ± " + f.replace("+", "").format(sd) if not np.isnan(sd) and sd > 1e-9 else "")

    lines = ["# Results against the ground truth" + (" — official protocol (evo), poses at their own instant" if OFFICIAL else
                                                     " — each run at its best time shift" if OFFSET == "best" else ""), "",
             ("Oxford Spires benchmark protocol: `evo_ape tum gt_lidar.txt est.txt --align --t_max_diff 0.01` (APE = translation "
              "RMSE after a rigid SE(3) alignment, association within 10 ms, no time offset); RPE with evo over 1 m (all pairs, "
              "pairs from the reference) and over 1 s (same relative error, all pairs, pairs by time: evo has no time step).  Every pose is stamped with the instant it stands for "
              "(scripts/evaluate_official.py): scan stamp = first point of the sweep; deskewed scan = last point (+0.1 s); "
              "raw scan (no deskew) = mean point time (+0.05 s).  No search over time shifts (#045/#046 closed).  "
              "Poses without a GT sample within 10 ms are dropped, as evo does (Oxford Spires has GT at ~77 % of the scan stamps "
              "plus a 20 Hz grid).  Hilti 2021 and NTU VIRAL: the APE column is the dataset's own official score (Hilti SLAM Challenge "
              "2021: APE of the IMU / pole tip / prism, scripts/evaluate_hilti.py; NTU VIRAL: ATE of the prism, scripts/evaluate_ntu.py), "
              "assoc. = control points matched (Hilti, sparse GT) / completeness (NTU); the other columns stay empty.  GT interp. = 1: the GT is interpolated at the pose times (fewer than half associate: "
              "no-deskew poses fall between the GT samples).  " if OFFICIAL else
              "Every run is scored at the time shift (added to its scan stamps) that minimises its rotation RPE over 1 s, "
              "searched in -0.15..+0.25 s (last column).  Arms differ in which instant of the sweep a pose stands for, and "
              "the rotation RPE of a hand-held sensor doubles within 50 ms of shift (#045).  " if OFFSET == "best" else
              "Scan stamps as recorded (shift 0).  "),
             "KISS-SLAM default config (the setting of the KISS-SLAM paper), except the arms \"KISS-SLAM, indoor_detail\" "
             "(configs/indoor_detail.yaml: voxel 0.25 m, max range 50 m, local maps 15 m) and \"KISS-SLAM, no deskew\" "
             "(configs/kiss_paper_nodeskew.yaml: paper config, deskew off).  Mean ± σ over the runs of each arm "
             "(KISS-SLAM is deterministic: its runs are identical).  " + ("" if OFFICIAL else "ATE: RMSE after a rigid alignment.  RPE over 1 s.  ") +
             ("" if OFFICIAL else "KITTI: relative translation error over 100-800 m segments (undefined below 100 m).  ") +
             "*Best value per sequence in bold* (lower is better; path: closest to the GT).  "
             "Per CLAUDE.md, judge by RPE and path length: the ATE of a single run is not a measurement (#037).", ""]
    head = "| Dataset | Sequence (GT path) | Arm | runs | " + " | ".join(h for _, h, _ in METRICS) + " |"
    lines += [head, "|" + "---|" * (4 + len(METRICS))]
    for key in dict.fromkeys((r["dataset"], r["sequence"]) for r in rows):
        grp = [r for r in rows if (r["dataset"], r["sequence"]) == key]
        best = {c: min((r for r in grp if not np.isnan(r[c])), key=lambda r: abs(r[c]) if c == "excess" else r[c], default=None)
                for c, _, _ in METRICS if c not in ("path", "fail", "offset", "pose_minus_stamp", "gt_interp", "matched")}
        for i, r in enumerate(grp):
            cells = []
            for c, _, f in METRICS:
                s = cell(r, c, f)
                cells.append(f"**{s}**" if best.get(c) is r and len(grp) > 1 else s)
            gp = f" ({r['gt_path']:.0f} m)" if np.isfinite(r["gt_path"]) else ""
            first = f"{r['dataset']} | {r['sequence']}{gp}" if i == 0 else " | "
            lines.append(f"| {first} | {r['arm']} | {r['runs']} | " + " | ".join(cells) + " |")
    OUT.with_suffix(".md").write_text("\n".join(lines) + "\n")
    print(f"→ {OUT.with_suffix('.md')}, {OUT.with_suffix('.csv')}")


if __name__ == "__main__":
    main()
