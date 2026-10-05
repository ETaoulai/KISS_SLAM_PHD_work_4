# Ablation of the RA-L method (#165)

Geometric mean over each group of APE / APE(KISS-SLAM) (official protocols; ours 4 seeds, KISS / no deskew 1); RPE 1 m median over NCD + Spires;
failures = APE > 5 m over all groups; last column: sequences better / worse than the previous row by > 5 % in APE. stairs / dynamic_spinning excluded.

## Cumulative

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

## Leave-one-out (each against “+ range-image fallback”, #081 / #082)

| row | NCD (10): APE / KISS | Oxford Spires (6): APE / KISS | Hilti (6): APE / KISS | NTU eee (3): APE / KISS | Car Boreas (1): APE / KISS | NTU new (15): APE / KISS | RPE 1 m t / r (NCD + Spires) | failures | vs previous: better / worse (> 5 %) |
|---|---|---|---|---|---|---|---|---|---|
| + range-image fallback | 0.41 | 0.18 | 0.25 | 0.49 | 1.03 | – | 7.25 / 1.053 | 0 | 0 / 0 (of 26) |
| − image as ICP start (deskew only) | 0.76 | 0.55 | 0.70 | 0.40 | – | – | 8.77 / 1.078 | 2 | 5 / 15 (of 25) |
| − image deskew (ICP start only) | 0.51 | 0.32 | 0.28 | 0.56 | – | – | 9.65 / 0.791 | 0 | 4 / 18 (of 25) |
| − near-floor filter | 0.43 | 0.19 | 0.26 | 0.53 | – | – | 7.34 / 1.068 | 0 | 3 / 8 (of 25) |
| − fixed σ (KISS adaptive) | 0.41 | 0.17 | 0.26 | 0.69 | – | – | 7.41 / 1.047 | 0 | 5 / 6 (of 25) |
