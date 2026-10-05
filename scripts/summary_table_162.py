#!/usr/bin/env python3
"""One table of every full-run arm (#162): our variants, KISS-SLAM, KISS-SLAM without deskew and the other methods (#083 / #085), per
sequence (APE / ATE of each dataset's official protocol, mean over seeds), then per-dataset summaries.

    python scripts/summary_table_162.py <results csv (27 sequences)> <NTU-new dir with n149_<seq>.txt> <boreas_all.txt> <out.md>

Groups: NCD (2020 + 2021, without stairs / dynamic_spinning - special, shown apart), Oxford Spires, Hilti 2021, NTU VIRAL (eee_01-03 with all
methods; the 15 new NTU sequences with KISS and our arms only), car (Boreas, 3000 scans).  Summaries per group and arm, over the sequences
the arm has: median of APE / APE(KISS), geometric mean of the same ratio, wins among the LiDAR-only arms, failures (APE > 5 m), and the mean
rank among the arms that cover the whole group.
"""
import csv
import glob
import math
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

ARMS = [  # (csv arm label, short name, boreas folder prefix, NTU-new prefix, LiDAR-only?)
    ("KISS-SLAM", "KISS-SLAM", "kiss", "kiss", True),
    ("KISS-SLAM, no deskew", "KISS no deskew", "kissnodeskew", None, True),
    ("GenZ-ICP (odometry, own config, #083)", "GenZ-ICP", None, None, True),
    ("MAD-ICP (odometry, own config, #083)", "MAD-ICP", None, None, True),
    ("DLO (LiDAR-only odometry, authors' config with imu false, #085)", "DLO", None, None, True),
    ("CT-ICP (continuous-time odometry, robust low-inertia profile, #083)", "CT-ICP", None, None, True),
    ("Traj-LO (continuous-time odometry, own config, #083)", "Traj-LO", None, None, True),
    ("FAST-LIO2 (LiDAR-inertial, reference, #083)", "FAST-LIO2 (IMU)", None, None, False),
    ("COIN-LIO (LiDAR-inertial, intensity + IMU, reference, #083)", "COIN-LIO (IMU)", None, None, False),
    ("i3 + SURF, two starting points, range image when intensity fails", "Ours #081 (paper so far)", "surftworangefb", None, True),
    ("default of after_091 (upright SURF + guided matching, shift rule), two starts + range fallback (#092)", "Ours default", "base127", "base092", True),
    ("default of after_091, adaptive blend for the DESKEW only, ICP start = image (#137 / #138 / #140)", "+ blend (B)", "bd137", "bd137", True),
    ("default of after_091, PAIR: blend for the deskew + fallback = KISS (#148 / #149)", "+ blend + fallback (A)", "bf148", "bf148", True),
    ("default of after_091, COMBINATION: blend for the deskew + sector weights for the rotation (#141 / #142 / #144)", "+ blend + sectors", "cmb141", None, True),
]
GROUPS = {
    "NCD": ["01_short", "02_long_experiment", "quad_easy", "quad_hard", "cloister", "math_easy", "math_medium", "underground_easy",
            "underground_medium", "underground_hard"],
    "Oxford Spires": ["christ-church-02", "christ-church-03", "keble-college-03", "observatory-quarter-01", "blenheim-palace-02",
                      "bodleian-library-02"],
    "Hilti 2021": ["Construction_Site_1", "Office_Mitte_1", "IC_Office_1", "LAB_Survey_2", "Basement_1", "UZH_Tracking_Area_Run_2"],
    "NTU (eee)": ["eee_01", "eee_02", "eee_03"],
    "NTU (15 new)": ["nya_01", "nya_02", "nya_03", "sbs_01", "sbs_02", "sbs_03", "rtp_01", "rtp_02", "rtp_03", "tnp_01", "tnp_02",
                     "tnp_03", "spms_01", "spms_02", "spms_03"],
    "Car (Boreas)": ["Boreas"],
}
SPECIAL = ["stairs", "dynamic_spinning"]
FAIL = 5.0


def main():
    csv_path, ntu_dir, boreas_txt, out = sys.argv[1:5]
    ape = defaultdict(dict)                      # ape[seq][short] = mean APE
    rpe = defaultdict(dict)
    label = {a[0]: a[1] for a in ARMS}
    for r in csv.DictReader(open(csv_path)):
        if r["arm"] in label and r["ate"] not in ("", "nan"):
            ape[r["sequence"]][label[r["arm"]]] = float(r["ate"])
            if r["rpe_t"] not in ("", "nan"):
                rpe[r["sequence"]][label[r["arm"]]] = (float(r["rpe_t"]), float(r["rpe_r"]))
    ntu = {a[3]: a[1] for a in ARMS if a[3]}
    for f in glob.glob(str(Path(ntu_dir) / "n149_*.txt")):
        vals = defaultdict(list)
        for line in open(f):
            x = line.split()
            arm = x[1].rsplit("_s", 1)[0]
            if arm in ntu:
                vals[(x[0], ntu[arm])].append(float(x[2]))
        for (seq, short), v in vals.items():
            ape[seq][short] = st.mean(v)
    bor = {a[2]: a[1] for a in ARMS if a[2]}
    vals = defaultdict(list); rp = defaultdict(list)
    for line in open(boreas_txt):
        x = line.split()
        arm = x[0].rsplit("_s", 1)[0]
        if arm in bor:
            vals[bor[arm]].append(float(x[1])); rp[bor[arm]].append((float(x[2]), float(x[3])))
    for short, v in vals.items():
        ape["Boreas"][short] = st.mean(v)
        rpe["Boreas"][short] = (st.mean(a for a, _ in rp[short]), st.mean(b for _, b in rp[short]))
    names = [a[1] for a in ARMS]
    lidar_only = {a[1] for a in ARMS if a[4]}
    L = ["# Summary of every full-run arm (#162)", "",
         "APE / ATE in m (each dataset's official protocol), mean over the arm's seeds (4 for ours, 1 for KISS / no deskew / other methods). "
         "**Bold** = best LiDAR-only on the sequence; † = failure (APE > 5 m); – = not run. IMU methods (FAST-LIO2, COIN-LIO) are reference only.", ""]
    seqs = [s for g in GROUPS.values() for s in g] + SPECIAL
    L.append("| sequence | " + " | ".join(names) + " |")
    L.append("|---" * (len(names) + 1) + "|")
    for s in seqs:
        row = ape.get(s, {})
        lo = [row[n] for n in names if n in row and n in lidar_only]
        best = min(lo) if lo else None
        cells = []
        for n in names:
            if n not in row:
                cells.append("–"); continue
            v = row[n]; t = f"{v:.3f}" if v < 10 else f"{v:.1f}"
            if v > FAIL: t += " †"
            if best is not None and v == best and n in lidar_only: t = f"**{t}**"
            cells.append(t)
        L.append(f"| {s}{' (special)' if s in SPECIAL else ''} | " + " | ".join(cells) + " |")
    L += ["", "## Per dataset (special sequences excluded)", ""]
    summ = {}
    for g, gs in GROUPS.items():
        L += [f"### {g} ({len(gs)} sequences)", "", "| arm | sequences | median APE / KISS | geo-mean APE / KISS | LiDAR-only wins | failures (> 5 m) | mean rank* |",
              "|---|---|---|---|---|---|---|"]
        full = [n for n in names if n in lidar_only and all(n in ape.get(s, {}) for s in gs)]
        ranks = defaultdict(list)
        for s in gs:
            order = sorted(full, key=lambda n: ape[s][n])
            for i, n in enumerate(order):
                ranks[n].append(i + 1)
        wins = defaultdict(int)
        for s in gs:
            lo = {n: ape[s][n] for n in names if n in ape.get(s, {}) and n in lidar_only}
            if lo:
                wins[min(lo, key=lo.get)] += 1
        for n in names:
            have = [s for s in gs if n in ape.get(s, {})]
            if not have:
                continue
            rat = [ape[s][n] / ape[s]["KISS-SLAM"] for s in have if "KISS-SLAM" in ape[s]]
            gm = math.exp(st.mean(math.log(x) for x in rat)) if rat else float("nan")
            md = st.median(rat) if rat else float("nan")
            fails = sum(ape[s][n] > FAIL for s in have)
            mr = f"{st.mean(ranks[n]):.1f}" if n in ranks else "–"
            summ[(g, n)] = (md, gm, wins[n], fails, mr, len(have))
            L.append(f"| {n} | {len(have)} | {md:.2f} | {gm:.2f} | {wins[n] if n in lidar_only else '–'} | {fails} | {mr} |")
        L += ["", f"*mean rank among the {len(full)} LiDAR-only arms run on all {len(gs)} sequences: {', '.join(full)}", ""]
    L += ["## RPE 1 m (NCD + Oxford Spires + Boreas, translation cm / rotation °), median over sequences", "",
          "| arm | sequences | median RPE t | median RPE r | median RPE t / KISS | median RPE r / KISS |", "|---|---|---|---|---|---|"]
    rseq = GROUPS["NCD"] + GROUPS["Oxford Spires"] + ["Boreas"]
    for n in names:
        have = [s for s in rseq if n in rpe.get(s, {}) and "KISS-SLAM" in rpe[s]]
        if not have:
            continue
        t = [rpe[s][n][0] for s in have]; r = [rpe[s][n][1] for s in have]
        tk = [rpe[s][n][0] / rpe[s]["KISS-SLAM"][0] for s in have]; rk = [rpe[s][n][1] / rpe[s]["KISS-SLAM"][1] for s in have]
        L.append(f"| {n} | {len(have)} | {st.median(t):.2f} | {st.median(r):.3f} | {st.median(tk):.2f} | {st.median(rk):.2f} |")
    Path(out).write_text("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
