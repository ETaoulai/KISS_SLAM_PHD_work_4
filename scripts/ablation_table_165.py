#!/usr/bin/env python3
"""Ablation of the RA-L method (#165, branch ral_method): cumulative rows from KISS-SLAM to the locked method, and leave-one-out rows,
per dataset group, from existing runs (official protocols, mean over seeds).

    python scripts/ablation_table_165.py <results csv (27 sequences)> <boreas runs txt: "run APE RPEt RPEr"> <out.md> [<dir with n149_<seq>.txt>]

Rows (cumulative): KISS-SLAM · KISS-SLAM without deskew · + image motion (deskew + ICP start) · + two starts · + range-image fallback ·
+ upright SURF / guided matching (implementation, = default of after_091) · + blend for the deskew (B) · + KISS fallback after 4 failures (C =
the locked method).  Leave-one-out (from the #081 / #082 ablation, against the "+ range-image fallback" row): image motion for the deskew only /
for the ICP start only, no near-floor filter, KISS's adaptive sigma.  Per group: geometric mean of APE / APE(KISS), median RPE 1 m (t / r,
NCD + Spires), failures (APE > 5 m), and per step the sequences better / worse than the previous row by more than 5 %.
"""
import csv
import math
import statistics as st
import sys
from collections import defaultdict

ROWS = [
    ("KISS-SLAM", "KISS-SLAM", "kiss"),
    ("KISS-SLAM, no deskew", "KISS-SLAM without deskew", "kissnodeskew"),
    ("i3 + SURF", "+ image motion (deskew + ICP start)", "ablsurf167"),
    ("i3 + SURF, two starting points", "+ two starts", "ablsurftwo167"),
    ("i3 + SURF, two starting points, range image when intensity fails", "+ range-image fallback", "surftworangefb"),
    ("default of after_091 (upright SURF + guided matching, shift rule), two starts + range fallback (#092)",
     "+ upright SURF / guided matching (implementation)", "base127"),
    ("default of after_091, adaptive blend for the DESKEW only, ICP start = image (#137 / #138 / #140)", "+ blend for the deskew (B)", "bd137"),
    ("default of after_091, OPTION C: blend for the deskew + fallback = KISS from the 4th consecutive failure (#150 / #151 / #164)",
     "+ KISS fallback after 4 failures (C) = locked method", "bfc150"),
]
LOO = [
    ("i3 + SURF, two starts + range fallback, image motion for deskew only, ICP from constant velocity (ablation #081)", "− image as ICP start (deskew only)"),
    ("i3 + SURF, two starts + range fallback, image motion as ICP start only, no deskew (#082)", "− image deskew (ICP start only)"),
    ("i3 + SURF, two starts + range fallback, no near-floor stuck-match filter (ablation #081)", "− near-floor filter"),
    ("i3 + SURF, two starts + range fallback, KISS adaptive sigma instead of fixed 2.0 (ablation #081)", "− fixed σ (KISS adaptive)"),
]
GROUPS = {
    "NCD (10)": ["01_short", "02_long_experiment", "quad_easy", "quad_hard", "cloister", "math_easy", "math_medium", "underground_easy",
                 "underground_medium", "underground_hard"],
    "Oxford Spires (6)": ["christ-church-02", "christ-church-03", "keble-college-03", "observatory-quarter-01", "blenheim-palace-02",
                          "bodleian-library-02"],
    "Hilti (6)": ["Construction_Site_1", "Office_Mitte_1", "IC_Office_1", "LAB_Survey_2", "Basement_1", "UZH_Tracking_Area_Run_2"],
    "NTU eee (3)": ["eee_01", "eee_02", "eee_03"],
    "Car Boreas (1)": ["Boreas"],
    "NTU new (15)": ["nya_01", "nya_02", "nya_03", "sbs_01", "sbs_02", "sbs_03", "rtp_01", "rtp_02", "rtp_03", "tnp_01", "tnp_02", "tnp_03",
                     "spms_01", "spms_02", "spms_03"],
}

FAIL = 5.0


def main():
    csv_path, boreas_txt, out = sys.argv[1:4]
    ntu_dir = sys.argv[4] if len(sys.argv) > 4 else None
    ape, rpe = defaultdict(dict), defaultdict(dict)
    for r in csv.DictReader(open(csv_path)):
        if r["ate"] not in ("", "nan"):
            ape[r["sequence"]][r["arm"]] = float(r["ate"])
            if r["rpe_t"] not in ("", "nan"):
                rpe[r["sequence"]][r["arm"]] = (float(r["rpe_t"]), float(r["rpe_r"]))
    bor = defaultdict(list)
    for line in open(boreas_txt):
        x = line.split()
        bor[x[0].rsplit("_s", 1)[0]].append([float(v) for v in x[1:4]])
    for label, _, prefix in ROWS:
        if prefix and prefix in bor:
            v = bor[prefix]
            ape["Boreas"][label] = st.mean(a[0] for a in v)
            rpe["Boreas"][label] = (st.mean(a[1] for a in v), st.mean(a[2] for a in v))

    if ntu_dir:                                                       # the 15 new NTU sequences: KISS and the last three rows only
        import glob
        from pathlib import Path
        pref = {"kiss": "KISS-SLAM", "base092": ROWS[5][0], "bd137": ROWS[6][0], "bfc150": ROWS[7][0]}
        for f in glob.glob(str(Path(ntu_dir) / "n149_*.txt")):
            vals = defaultdict(list)
            for line in open(f):
                x = line.split()
                a = x[1].rsplit("_s", 1)[0]
                if a in pref:
                    vals[(x[0], pref[a])].append(float(x[2]))
            for (seq, lab), v in vals.items():
                ape[seq][lab] = st.mean(v)

    def block(rows, ref_label=None, cumulative=True):
        L = ["| row | " + " | ".join(f"{g}: APE / KISS" for g in GROUPS) + " | RPE 1 m t / r (NCD + Spires) | failures | vs previous: better / worse (> 5 %) |",
             "|---" * (len(GROUPS) + 4) + "|"]
        prev = None
        for label, name in rows:
            cells = []
            for g, seqs in GROUPS.items():
                rat = [ape[s][label] / ape[s]["KISS-SLAM"] for s in seqs if label in ape[s] and "KISS-SLAM" in ape[s]]
                cells.append(f"{math.exp(st.mean(math.log(x) for x in rat)):.2f}" if rat else "–")
            rs = [s for s in GROUPS["NCD (10)"] + GROUPS["Oxford Spires (6)"] if label in rpe[s]]
            rp = f"{st.median(rpe[s][label][0] for s in rs):.2f} / {st.median(rpe[s][label][1] for s in rs):.3f}" if rs else "–"
            allseq = [s for g2, seqs in GROUPS.items() if g2 != "NTU new (15)" for s in seqs if label in ape[s]]
            fails = sum(ape[s][label] > FAIL for s in allseq)
            cmp_to = prev if cumulative else ref_label
            if cmp_to:
                both = [s for s in allseq if cmp_to in ape[s]]
                b = sum(ape[s][label] < 0.95 * ape[s][cmp_to] for s in both); w = sum(ape[s][label] > 1.05 * ape[s][cmp_to] for s in both)
                vs = f"{b} / {w} (of {len(both)})"
            else:
                vs = "–"
            L.append(f"| {name} | " + " | ".join(cells) + f" | {rp} | {fails} | {vs} |")
            prev = label
        return L

    L = ["# Ablation of the RA-L method (#165)", "",
         "Geometric mean over each group of APE / APE(KISS-SLAM) (official protocols; ours 4 seeds, KISS / no deskew 1); RPE 1 m median over NCD + Spires;",
         "failures = APE > 5 m over all groups; last column: sequences better / worse than the previous row by > 5 % in APE. stairs / dynamic_spinning excluded.", "",
         "## Cumulative", ""]
    L += block([(a, b) for a, b, _ in ROWS])
    L += ["", "## Leave-one-out (each against “+ range-image fallback”, #081 / #082)", ""]
    L += block([(ROWS[4][0], ROWS[4][1])] + LOO, ref_label=ROWS[4][0], cumulative=False)
    open(out, "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
