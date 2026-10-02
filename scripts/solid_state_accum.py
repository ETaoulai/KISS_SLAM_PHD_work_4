#!/usr/bin/env python3
"""Motion-compensated accumulation for solid-state LiDARs (#096, open_tasks B.10a): TIERS Indoor02, Livox Avia / Horizon, scored on their gyros.

    python scripts/solid_state_accum.py <bag> <optitrack.csv> [--frames=N] [--kinds=avia,horizon] [--K=2,3,5] [--comp=estimate,gyro,icp]
                                  [--icp-avia=<KISS run dir>] [--icp-horizon=<KISS run dir>] [--out=<prefix>]

#095: one Livox scan gives too sparse an image (14-43 inliers), and accumulating raw scans blurs it (the sensor moves in the window).
Here the image of scan k is built from scans k-K+1 .. k, all brought into the sensor frame at the START of scan k:
  - an older scan j: deskewed to the end of its sweep with its own estimated motion (KISS convention, p' = exp((s - 1) log M_j) p), then
    carried to the start of k by (M_{j+1} ... M_{k-1})^-1;
  - the current scan k (motion still unknown): deskewed to its start with the constant-velocity prediction M_{k-1}, p' = exp(s log M) p.
The image only places features.  The 3D point and the time of a keypoint come from the NEWEST scan's raw points (nearest within 1.5 px,
its own time), so the time-aware fit sees what it sees today - raw points of scan k-1 against raw points of scan k - and the motion,
deskew and ICP are unchanged.  The shared content of the two images is placed by motions already estimated, so it does not pull the new
motion toward zero (#095: sliding raw windows would).  Risk: an estimate error moves the next image (error propagation).
Variant "icp" (#099): the compensation (older scans and the prediction of the current one) from a KISS run on the same sensor - what the
pipeline would use, causally (the ICP poses of scans before k exist when scan k arrives; rotation AND translation).
Variant "gyro": the older scans compensated with the gyro's rotation (rotation only, translation 0) - how much of the effect is density
and how much is the quality of the compensation.  Failed motions: constant velocity for the compensation.

Pixel = sqrt(FoV area / accumulated points); detector on the image x1 / x2 (SURF upright); brute-force matching; model "car".
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import kiss_slam.intensity_deskew as d  # noqa: E402
from solid_state_check import AVIA_TOPIC, HORIZON_TOPIC, read_bag, score_gyro, gyro_rotation  # noqa: E402


def twist(M):
    return np.real(logm(M))


def deskew(xyz, s, M, to_end):
    """KISS deskew with the whole-scan motion M (pose of the end in the start frame): to the END frame, exp((s - 1) L) p; to the START
    frame, exp(s L) p.  s in [0, 1] per point.  Points grouped in 50 time bins (the motion of one bin is far below a pixel)."""
    L = twist(M)
    out = np.empty_like(xyz)
    bins = np.minimum((s * 50).astype(int), 49)
    for b in np.unique(bins):
        m = bins == b
        T = expm(((b + 0.5) / 50 - (1.0 if to_end else 0.0)) * L)
        out[m] = xyz[m] @ T[:3, :3].T + T[:3, 3]
    return out


class Grid:
    """Fixed azimuth x elevation grid inside the FoV (bounds from the first scan)."""
    def __init__(self, xyz, step):
        az, el = self.angles(xyz)
        self.az1, self.el1 = np.percentile(az, 99.5) + 1, np.percentile(el, 99.5) + 1
        self.W = int(np.ceil((self.az1 - np.percentile(az, 0.5) + 1) / step)); self.H = int(np.ceil((self.el1 - np.percentile(el, 0.5) + 1) / step))
        self.step = step

    @staticmethod
    def angles(xyz):
        return np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])), np.degrees(np.arctan2(xyz[:, 2], np.linalg.norm(xyz[:, :2], axis=1)))

    def pix(self, xyz):
        az, el = self.angles(xyz)
        return (np.clip(((self.el1 - el) / self.step).astype(int), 0, self.H - 1), np.clip(((self.az1 - az) / self.step).astype(int), 0, self.W - 1))

    def image(self, pos, inten):
        r, c = self.pix(pos)
        acc = np.zeros((self.H, self.W)); cnt = np.zeros((self.H, self.W))
        np.add.at(acc, (r, c), inten); np.add.at(cnt, (r, c), 1)
        num = cv2.GaussianBlur(acc, (0, 0), 0.8); den = cv2.GaussianBlur(cnt, (0, 0), 0.8)
        return np.clip(np.where(den > 1e-3, num / np.maximum(den, 1e-9), 0.0), 0, 255).astype(np.uint8)

    def lift(self, pos, xyz_raw, ts):
        """P, T, valid from the newest scan: raw point and time at the pixel where its (compensated) position falls."""
        r, c = self.pix(pos)
        P = np.zeros((self.H, self.W, 3)); T = np.zeros((self.H, self.W)); have = np.zeros((self.H, self.W), bool)
        P[r, c] = xyz_raw; T[r, c] = ts; have[r, c] = True
        dist, (ri, ci) = ndimage.distance_transform_edt(~have, return_indices=True)
        return P[ri, ci], T[ri, ci], dist <= 1.5


def run(frames, kind, K, ds, comp, imu, clock_off, icp=None):
    fr = frames[kind]
    n_pts = np.median([len(f[1]) for f in fr])
    az, el = Grid.angles(fr[0][1])
    area = (np.percentile(az, 99.5) - np.percentile(az, 0.5)) * (np.percentile(el, 99.5) - np.percentile(el, 0.5))
    step = np.sqrt(area / (K * n_pts))
    g = Grid(fr[0][1], step)
    d.W, d.UP, d.DETECT_SCALE = g.W, 1, ds
    det = d.make_detector("surf", 100.0, True); bf = cv2.BFMatcher(cv2.NORM_L2); rng = np.random.default_rng(0)
    d.GUIDED_WINDOW = None
    hist = []                                            # the K-1 previous scans, oldest first: (xyz raw, s, inten, M used for it, t0)
    prev_feat, last_M = None, np.eye(4)
    times, motions, inl = [], [], []

    def gyro_M(t0):
        Rg = gyro_rotation(imu, t0 - clock_off, t0 - clock_off + 0.1)
        M = np.eye(4)
        if Rg is not None:
            M[:3, :3] = Rg.as_matrix()                   # rotation of the end in the start frame (body rates integrated), translation 0
        return M

    for i, (stamp, xyz, ts, inten, ring) in enumerate(fr):
        t0, t1 = float(ts.min()), float(ts.min()) + 0.1
        s = np.clip((ts - t0) / 0.1, 0, 1)
        if comp == "gyro":
            pred = gyro_M(t0)
        elif comp == "icp":
            pred = icp[i - 1] if i >= 1 else np.eye(4)            # constant velocity from the ICP (scan k-1's motion)
        else:
            pred = last_M
        cur_pos = deskew(xyz, s, pred, to_end=False)                 # current scan into its start frame
        pos, val = [cur_pos], [inten]
        carry = np.eye(4)                                            # C_j = M_{j+1} ... M_{k-1}: start of k in the frame at the end of scan j
        for hx, hs, hin, hM, ht0 in reversed(hist):                  # scan k-1 first
            p_end = deskew(hx, hs, hM, to_end=True)
            Ci = np.linalg.inv(carry)
            pos.append(p_end @ Ci[:3, :3].T + Ci[:3, 3]); val.append(hin)
            carry = hM @ carry
        img = g.image(np.concatenate(pos), np.concatenate(val))
        P, T, valid = g.lift(cur_pos, xyz, ts)
        kps, desc = d._detect(det, img)
        feat = (P, T, valid, kps, desc, t0, img)
        M = None
        if prev_feat is not None:
            M0, M1, n = d.match_motion(prev_feat, feat, 0.1, rng, bf, "car", True, 0.05, True, -10.0, 5.0)
            M = None if M1 is None else np.asarray(M1)
            times.append((t0, t1)); motions.append(M); inl.append(n)
        prev_feat = feat
        if M is not None:
            last_M = M
        used = gyro_M(t0) if comp == "gyro" else icp[i] if comp == "icp" else (M if M is not None else last_M)
        if K > 1:
            hist = (hist + [(xyz, s, inten, used, t0)])[-(K - 1):]
    return times, motions, inl, step, g.W


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    frames = read_bag(args[0], int(opts.get("frames", 423)))
    out = opts.get("out")
    for kind in opts.get("kinds", "avia,horizon").split(","):
        off = frames["clock"][AVIA_TOPIC if kind == "avia" else HORIZON_TOPIC]
        for K in [int(k) for k in opts.get("K", "2,3,5").split(",")]:
            icp = None
            if f"icp-{kind}" in opts:                    # a KISS run on this sensor: motion of scan j = P_{j-1}^-1 P_j (pose at the end of the scan)
                P = np.load(sorted(Path(opts[f"icp-{kind}"]).glob("*/*_poses.npy"))[-1])
                icp = [np.eye(4)] + [np.linalg.inv(P[j - 1]) @ P[j] for j in range(1, len(P))]
            for comp in opts.get("comp", "estimate,gyro" + (",icp" if icp is not None else "")).split(","):
                for ds in (1.0, 2.0):
                    times, motions, inl, step, W = run(frames, kind, K, ds, comp, frames["imu"][kind], off, icp)
                    score_gyro(f"{kind} K{K} {comp} det x{ds:g}", [(a - off, b - off) for a, b in times], motions, inl, frames["imu"][kind])
                    print(f"{'':28s} pixel {step:.3f} deg, image {W} columns")
                    if out:
                        np.savez(f"{out}_{kind}_K{K}_{comp}_ds{ds:g}.npz", times=np.array(times), inliers=np.array(inl),
                                 motion=np.array([m if m is not None else np.full((4, 4), np.nan) for m in motions]))


if __name__ == "__main__":
    main()
