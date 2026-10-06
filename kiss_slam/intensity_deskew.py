"""Ι-3: the motion of a LiDAR scan measured from its intensity image (#018).

The intensity panorama of a spinning LiDAR is a rolling-shutter image: each column was
measured at a different moment of the sweep.  Matching point-like intensity features
(window corners, railings) between two consecutive raw scans, each with its own time,
constrains the motion DURING the sweep independently of the geometry, which ICP cannot:
a scan deskewed with a wrong motion still fits a smooth map (#017).

Models, with time t in scan periods from the START of the later scan (pose(0) = I):
  "cv" constant velocity:      pose(t) = [R(t w), t v]                               (6 unknowns)
  "ca" constant acceleration:  pose(t) = [R(t w + t²/2 α), t v + t²/2 a]            (12 unknowns)
  "car" acceleration in rotation only: [R(t w + t²/2 α), t v]                        (9 unknowns)
       (#020: "ca" improves rotation but its 6 extra translation unknowns hurt translation)
A feature seen as p at t_p in one scan and as q at t_q in the next satisfies
pose(t_p)·p = pose(t_q)·q.  The motion of the later scan is pose(1).  The hand-held sensor's
rotation changes between consecutive scans almost as much as it is (#019), which a single
velocity over the two scans cannot follow; "ca" lets it change.

`ScanMotionEstimator.motion(scan)` returns the motion over one period, as the 4x4 pose of
the later sensor frame in the earlier one (the quantity KISS calls `delta` and deskews
with), estimated from the previous scan and this one; None when it cannot be estimated.
"""
import cv2

cv2.setNumThreads(8)   # όχι όλους τους πυρήνες (Λ.Γ. 18/9)
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

W, UP, MIN_RANGE = 1024, 8, 1.0          # panorama columns · vertical upscaling for SIFT · drop the operator
RATIO, RANSAC_THR, RANSAC_IT, MIN_INL, FIT_THR = 0.75, 0.30, 400, 10, 0.10
# #087 (branch rotation_bearing), both off by default:
# BEARING_MIN_RANGE (m): after the 3D fit, re-estimate the ROTATION parameters from the directions (bearings) of the matches farther than
#   this, translation fixed - the rotation then carries no range noise and no wrong depth at edges.  None = off.
# GUIDED_WINDOW (px, azimuth): match each keypoint only against the next panorama's keypoints within this many columns and GUIDED_ROWS
#   rings of its own position (wrap-around at 0/360 deg), ratio test inside the window.  None = brute force over the whole panorama.
BEARING_MIN_RANGE = None
# #132 (#130 offline): after the time fit, drop the pairs that zero motion explains at least as well as the fitted motion (only while the
# sensor clearly moves: fitted translation > 2 x the median residual) - matches that move with the sensor or barely move pull the translation towards zero - and refit.  False = off.
DROP_STATIONARY = False
BEARING_MIN_PAIRS = 20
GUIDED_WINDOW = None
GUIDED_ROWS = 4
GUIDED_K = 16
GUIDED_SHIFT = 0.0                     # #088: centre of the window, columns - the median column shift of the previous scan's matches
GUIDED_WINDOW_SAVED = None
GUIDED_PREDICTION = "shift"            # #089: window centre - "shift" (median column shift of the previous scan, #088) or "motion"
                                       #   (each keypoint's 3D point moved by the previous scan's motion and projected; per keypoint)
GUIDED_PRED_MOTION = None              # #089: the previous scan's motion (p = R q + t), set by the estimator before each match
GUIDED_MIN_MATCHES = 30
GUIDED_MAX_SHIFT = 20.0               # #088: previous shift above this many columns (~7 deg / scan) -> brute force (fast rotation)                # #088: fewer guided matches than this -> brute force for this scan (safety net, fast rotation)
ORB_FEATURES = 5000                    # #088: ORB keypoints per panorama (OpenCV default 500 - too few here)
# RANSAC stops once the best consensus so far makes a better one unlikely: after
# log(1 - RANSAC_CONF) / log(1 - w^3) hypotheses, w = best inlier fraction (≈ 50 at w = 0.5 instead of
# 400), never more than RANSAC_IT; hypotheses are drawn and scored RANSAC_BATCH at a time in numpy.
# None = the fixed RANSAC_IT one-by-one loop of every result before fast_test (same random draws).
RANSAC_CONF, RANSAC_BATCH = 0.999, 32
EDGE_REL = 0.05                          # sub-pixel: neighbours' ranges within 5 % → same surface
# Inlier test.  "metric": 3D distance < RANSAC_THR (m) — lets through intensity patterns that move WITH the sensor
# (range / incidence falloff on near floors and walls): they match at the same pixel, look motionless, and bias the
# translation toward zero (scale 0.92, corridor 0.84; #023-#024).  "bearing": the direction of M·q must agree with p
# within BEARING_THR (deg) and the range within RANGE_REL — the error metric of an image feature.
INLIER_TEST, BEARING_THR, RANGE_REL = "metric", 0.35, 0.05
# Drop matches that are the same point in the sensor frame (|p − q| < STUCK_MIN m): a real match moves about as much as
# the sensor (~0.1 m per scan); one that does not is a pattern travelling with the sensor (#025). None = keep all.
STUCK_MIN = None
# Apply it only to the near floor (below STUCK_ELEV deg in the sensor frame and closer than STUCK_RANGE m), where the
# patterns are; elsewhere a real match can look motionless when rotation and translation cancel (#026).
STUCK_FLOOR_ONLY, STUCK_ELEV, STUCK_RANGE = False, -10.0, 5.0
# Optional ring × azimuth boolean mask of pixels fixed to the sensor (shadows of the rig, #035); a match whose
# keypoint in the later scan falls on a masked pixel is dropped.  None = off.
PIXEL_MASK = None
# Near-field bias (#039): intensity-pattern matches closer than ~5 m report only ~74 % of the true translation, those
# beyond ~12 m report 100 %.  With TRANS_MIN_RANGE set, the translation is re-estimated with the rotation frozen, from
# the inliers farther than that (metres); the rotation, and the timed curve, are untouched.  None = off.
TRANS_MIN_RANGE = None
TRANS_MIN_PAIRS = 8
# "vector": replace the translation with the far-only estimate (removes the bias, adds variance, #039).
# "magnitude": keep the DIRECTION from all inliers (well estimated) and correct only its LENGTH by the scalar the far
# inliers imply, clipped to TRANS_CLIP — same bias removal, far less variance.
TRANS_MODE, TRANS_CLIP = "vector", (0.7, 1.6)
# Simplest form: the bias is systematic, so multiply every translation by one constant instead of re-estimating it per
# scan (which adds variance, #039).  1.0 = off.
TRANS_FACTOR = 1.0
# "auto" (#039): the correction is measured FROM THE DATA and needs no GT.  Per scan the ratio of the translation the
# far inliers imply to the one all inliers imply is noisy, but its running median over TRANS_AUTO_WINDOW past scans is
# stable — the far matches' lack of bias without their variance, and it follows the scene (a corridor biases more than
# a courtyard).  Causal: a scan is corrected with the median of the scans BEFORE it.
TRANS_AUTO_WINDOW, TRANS_AUTO_MIN = 100, 20


CALIB = None   # {"r_edges", "c_edges", "table"}: median intensity per (range, cos incidence) bin (#025)


def native_width(xyz, ts, ring):
    """The sensor's own columns per revolution (#093): 360 deg over the median azimuth step between consecutive firings of a ring
    (in time order; repeated azimuths of a dual return skipped).  Measured from one scan: Ouster 1024 / 2048, Hesai QT64 600."""
    ok = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > MIN_RANGE)
    rings = np.unique(ring[ok])
    steps = []
    for k in rings[:: max(1, len(rings) // 8)]:
        m = ok & (ring == k)
        a = np.degrees(np.unwrap(np.arctan2(xyz[m, 1], xyz[m, 0])[np.argsort(ts[m], kind="stable")]))
        d = np.abs(np.diff(a)); d = d[(d > 1e-4) & (d < 5)]
        if len(d):
            steps.append(np.median(d))
    return W if not steps else int(round(360.0 / float(np.median(steps))))


def square_upscale(xyz, ring, min_range=5.0):
    """Vertical upscaling that makes the panorama pixels square in angle (#093): the median elevation step between adjacent rings over
    the column width (360 / W deg), NOT rounded to an integer - an integer flips between scenes where the ratio is near x.5 (Hesai QT64 at
    600 columns: 1.5 deg / 0.6 deg = 2.5) - but to a multiple of 1 / rings, so the panorama has a whole number of rows (rings x UP).
    Ring elevations from points beyond min_range (near points see the beam origin's offset).  Measured from one scan, so it follows the
    sensor with no setting: Ouster 128 ~2, OS1-64 ~1.5, Hilti Ouster 64 at 2048 columns ~8, Hesai QT64 at 600 ~2.6, OS1-16 ~6."""
    rng_ = np.linalg.norm(xyz, axis=1)
    ok = np.isfinite(xyz).all(axis=1) & (rng_ > min_range)
    if len(np.unique(ring[ok])) < 2:
        ok = np.isfinite(xyz).all(axis=1) & (rng_ > MIN_RANGE)
    elev = np.degrees(np.arctan2(xyz[ok, 2], np.linalg.norm(xyz[ok, :2], axis=1)))
    r = ring[ok]
    e = np.sort([np.median(elev[r == k]) for k in np.unique(r)])
    if len(e) < 2:
        return UP
    n_rings = len(np.unique(ring))
    ratio = float(np.median(np.diff(e))) / (360.0 / W)
    return max(n_rings, int(round(ratio * n_rings))) / n_rings      # at least 1


def incidence_cos(xyz, knn=10):
    """|cos| of the incidence angle per point, from PCA normals of its 3D neighbours (Open3D).

    Grid neighbours do not work: some rings have only ~250 returns per turn, so adjacent
    pixels of the 1024-column panorama are almost never both measured.
    """
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn))
    n = np.asarray(pc.normals)
    return np.abs(np.sum(n * xyz, axis=1)) / np.maximum(np.linalg.norm(xyz, axis=1), 1e-9)


def calibrated(img, P, valid, cos):
    """Intensity divided by the median intensity of its (range, cos incidence) bin — reflectance-like, ~1 on average."""
    rng_ = np.linalg.norm(P, axis=2)
    ri = np.clip(np.searchsorted(CALIB["r_edges"], rng_) - 1, 0, len(CALIB["r_edges"]) - 2)
    ci = np.clip(np.searchsorted(CALIB["c_edges"], np.nan_to_num(cos, nan=-1)) - 1, -1, len(CALIB["c_edges"]) - 2)
    # unknown incidence: the range-only column (last)
    ci = np.where(np.isnan(cos), CALIB["table"].shape[1] - 1, ci)
    out = img / np.maximum(CALIB["table"][ri, ci], 1e-3)
    return np.where(valid, out, np.nan)


def ring_order(xyz, ring):
    """(rings present, rings sorted top to bottom by mean elevation).  #174: one pass with bincount instead of a mask per ring
    (Boreas, 128 rings x 220 000 points: 18 ms -> 1 ms); the same order."""
    x, y = xyz[:, 0], xyz[:, 1]
    elev = np.arctan2(xyz[:, 2], np.sqrt(x * x + y * y))
    cnt = np.bincount(ring)
    rings = np.flatnonzero(cnt)
    mean = np.bincount(ring, weights=elev)[rings] / cnt[rings]
    return rings, rings[np.argsort(-mean)]


def grid(xyz, ts, inten, ring, with_cos=False):
    """Ring × azimuth grid: (raw intensity, 3D point, time, valid mask[, |cos incidence|])."""
    rings, order = ring_order(xyz, ring)
    pos = np.empty(rings.max() + 1, dtype=np.int64); pos[order] = np.arange(len(order))
    row = pos[ring]
    col = ((np.arctan2(xyz[:, 1], xyz[:, 0]) + np.pi) / (2 * np.pi) * W).astype(int) % W
    H = len(rings)
    if _fit_cpp is not None and IMAGE_CPP != "off":            # #182: the same assignments in C++ (bit-identical)
        img, P, T, valid = _fit_cpp.grid_scatter(row, col, inten, xyz, ts, H, W)
    else:
        img = np.full((H, W), np.nan); P = np.zeros((H, W, 3)); T = np.zeros((H, W)); valid = np.zeros((H, W), bool)
        img[row, col] = inten; P[row, col] = xyz; T[row, col] = ts; valid[row, col] = True
    if not with_cos:
        return img, P, T, valid
    C = np.full((H, W), np.nan); C[row, col] = incidence_cos(xyz)
    return img, P, T, valid, C


# How the panorama is built (#029, idea Ι-5).
#   "splat":   each point is dropped into its nearest pixel; empty pixels of a row are filled by linear interpolation
#              of the image.  The sensor fires every 0.6 deg (dual return) while a column is 0.35 deg, so filled and
#              empty columns form a moiré that is the SAME in every turn — fixed with respect to the sensor.
#   "raycast": each pixel is a ray (its laser's elevation, the azimuth of the pixel centre) and takes range,
#              intensity and time by interpolation between the two samples of that laser on either side — the ray
#              meeting the surface through them.  No interpolation across a depth edge (the sample nearer in angle is
#              taken) or across a gap with no return (the pixel stays empty); with a dual return, the nearer surface.
RENDER, RAY_MAX_GAP, RAY_EDGE_REL, DUAL_EPS = "splat", 1.5, 0.05, 0.05     # deg, relative range, deg


def raycast_grid(xyz, ts, inten, ring):
    """Ring × azimuth grid by casting a ray per pixel: (intensity, 3D point, time, valid mask)."""
    el_all = np.arctan2(xyz[:, 2], np.linalg.norm(xyz[:, :2], axis=1))
    rings, order = ring_order(xyz, ring)
    H = len(order)
    img = np.full((H, W), np.nan); P = np.zeros((H, W, 3)); T = np.zeros((H, W)); valid = np.zeros((H, W), bool)
    azc = -180.0 + (np.arange(W) + 0.5) * 360.0 / W
    for row, rg in enumerate(order):
        m = ring == rg
        if m.sum() < 2:
            continue
        az = np.degrees(np.arctan2(xyz[m, 1], xyz[m, 0])); rng_ = np.linalg.norm(xyz[m], axis=1)
        el, it, t = el_all[m], inten[m], ts[m]
        o = np.lexsort((rng_, az)); az, rng_, el, it, t = az[o], rng_[o], el[o], it[o], t[o]
        keep = np.concatenate([[True], np.diff(az) > DUAL_EPS])      # dual return: first (nearer) surface only
        az, rng_, el, it, t = az[keep], rng_[keep], el[keep], it[keep], t[keep]
        n = len(az)
        # neighbours on either side of each pixel centre, with wrap-around
        j = np.searchsorted(az, azc)
        jl, jr = (j - 1) % n, j % n
        al = np.where(j == 0, az[jl] - 360.0, az[jl]); ar = np.where(j == n, az[jr] + 360.0, az[jr])
        gap = ar - al
        ok = gap <= RAY_MAX_GAP
        w = np.clip((azc - al) / np.maximum(gap, 1e-9), 0, 1)
        edge = np.abs(rng_[jr] - rng_[jl]) > RAY_EDGE_REL * np.minimum(rng_[jl], rng_[jr])
        w = np.where(edge, np.round(w), w)                             # depth edge: the sample nearer in angle
        r_ = (1 - w) * rng_[jl] + w * rng_[jr]
        e_ = (1 - w) * el[jl] + w * el[jr]
        a = np.radians(azc)
        P[row, ok] = (r_[:, None] * np.column_stack([np.cos(e_) * np.cos(a), np.cos(e_) * np.sin(a), np.sin(e_)]))[ok]
        img[row, ok] = ((1 - w) * it[jl] + w * it[jr])[ok]
        T[row, ok] = ((1 - w) * t[jl] + w * t[jr])[ok]
        valid[row, ok] = True
    return img, P, T, valid


def panorama(xyz, ts, inten, ring):
    """(uint8 image for the detector, 3D point per pixel, time per pixel, valid mask)."""
    if RENDER == "raycast":
        img, P, T, valid = raycast_grid(xyz, ts, inten, ring)
        img = np.nan_to_num(img, nan=0.0)                              # no return: black, not interpolated
    elif CALIB is None:
        img, P, T, valid = grid(xyz, ts, inten, ring)
    else:
        img, P, T, valid, C = grid(xyz, ts, inten, ring, with_cos=True)
        img = calibrated(img, P, valid, C)
        img = np.clip(img * 80.0, 0, 255)          # median reflectance → grey 80
    H = img.shape[0]
    for r in range(H if RENDER == "splat" else 0):
        v = valid[r]
        if v.sum() > 1:
            img[r, ~v] = np.interp(np.where(~v)[0], np.where(v)[0], img[r, v])
    img = np.nan_to_num(img, nan=0.0)
    big = cv2.resize(np.clip(img, 0, 255).astype(np.uint8), (W, int(round(H * UP))), interpolation=cv2.INTER_LINEAR)   # UP may be fractional (#093)
    return big, P, T, valid


def lookup(P, T, valid, kp, subpixel=False):
    """3D point and time at a keypoint.

    Nearest pixel by default.  One column is 0.35 deg and one ring 0.5-1 deg, so the nearest
    pixel alone carries an error of that order.  With `subpixel`, bilinear interpolation of
    point and time over the 4 surrounding pixels when all are valid and on one surface
    (ranges within EDGE_REL); otherwise the nearest pixel.
    """
    x, y = kp.pt
    rf = (y + 0.5) / UP - 0.5
    if subpixel:
        r0, c0 = int(np.floor(rf)), int(np.floor(x))
        if 0 <= r0 < P.shape[0] - 1:
            rr = [r0, r0, r0 + 1, r0 + 1]; cc = [c0 % W, (c0 + 1) % W, c0 % W, (c0 + 1) % W]
            if valid[rr, cc].all():
                rng = np.linalg.norm(P[rr, cc], axis=1)
                if rng.max() - rng.min() < EDGE_REL * rng.min():
                    a, b = rf - r0, x - c0
                    w = np.array([(1 - a) * (1 - b), (1 - a) * b, a * (1 - b), a * b])
                    return w @ P[rr, cc], w @ T[rr, cc]
    r = min(max(int(round(rf)), 0), P.shape[0] - 1); c = int(round(x)) % W
    for dc in (0, -1, 1, -2, 2):
        cc = (c + dc) % W
        if valid[r, cc]:
            return P[r, cc], T[r, cc]
    return None


def lookup_batch(P, T, valid, xy, subpixel=False):
    """`lookup` for N keypoints at once (#087, speed): pixel coordinates xy (N, 2) -> points (N, 3), times (N,), ok (N,).

    Exactly the rules of `lookup`: with `subpixel`, bilinear over the 4 surrounding pixels when all are valid and their ranges agree
    within EDGE_REL; otherwise the nearest pixel, then its row neighbours dc = 0, -1, 1, -2, 2 until a valid one.  Rounding as Python's
    round (half to even, as np.round).  Same results as the per-keypoint loop, without the Python loop."""
    n = len(xy)
    pts, ts, ok = np.full((n, 3), np.nan), np.full(n, np.nan), np.zeros(n, bool)
    if n == 0:
        return pts, ts, ok
    rows = P.shape[0]
    x, rf = xy[:, 0], (xy[:, 1] + 0.5) / UP - 0.5
    if subpixel:
        r0, c0 = np.floor(rf).astype(int), np.floor(x).astype(int)
        inr = (r0 >= 0) & (r0 < rows - 1)
        r0c = np.clip(r0, 0, rows - 2)
        rr = np.stack([r0c, r0c, r0c + 1, r0c + 1], 1); cc = np.stack([c0 % W, (c0 + 1) % W, c0 % W, (c0 + 1) % W], 1)
        vall = valid[rr, cc].all(1)
        rng = np.linalg.norm(P[rr, cc], axis=2)
        good = inr & vall & (rng.max(1) - rng.min(1) < EDGE_REL * rng.min(1))
        a, b = rf - r0, x - c0
        w = np.stack([(1 - a) * (1 - b), (1 - a) * b, a * (1 - b), a * b], 1)
        pts[good] = np.einsum("nk,nkd->nd", w[good], P[rr[good], cc[good]])
        ts[good] = np.einsum("nk,nk->n", w[good], T[rr[good], cc[good]])
        ok |= good
    rest = ~ok
    r = np.clip(np.round(rf).astype(int), 0, rows - 1); c = np.round(x).astype(int) % W
    for dc in (0, -1, 1, -2, 2):
        cc = (c + dc) % W
        hit = rest & valid[r, cc]
        pts[hit], ts[hit] = P[r[hit], cc[hit]], T[r[hit], cc[hit]]
        ok |= hit; rest &= ~hit
    return pts, ts, ok


def kabsch(A, B):
    """4x4 M with A ≈ R·B + t."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((B - cb).T @ (A - ca))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = ca - R @ cb
    return M


def inliers(A, B, M):
    Bm = B @ M[:3, :3].T + M[:3, 3]
    if INLIER_TEST == "metric":
        return np.linalg.norm(A - Bm, axis=1) < RANSAC_THR
    ra, rb = np.linalg.norm(A, axis=1), np.linalg.norm(Bm, axis=1)
    cos = np.sum(A * Bm, axis=1) / (ra * rb)
    return (np.degrees(np.arccos(np.clip(cos, -1, 1))) < BEARING_THR) & (np.abs(ra - rb) < RANGE_REL * ra)


def kabsch_batch(A, B):
    """(K, 4, 4) M with A[k] ≈ R_k·B[k] + t_k, for K point sets (K, n, 3) at once (as kabsch)."""
    ca, cb = A.mean(1), B.mean(1)
    U, _, Vt = np.linalg.svd(np.einsum("kni,knj->kij", B - cb[:, None], A - ca[:, None]))
    V, Ut = np.swapaxes(Vt, 1, 2), np.swapaxes(U, 1, 2)
    D = np.tile(np.eye(3), (len(A), 1, 1))
    D[:, 2, 2] = np.sign(np.linalg.det(V @ Ut))
    R = V @ D @ Ut
    M = np.tile(np.eye(4), (len(A), 1, 1))
    M[:, :3, :3] = R
    M[:, :3, 3] = ca - np.einsum("kij,kj->ki", R, cb)
    return M


def inliers_batch(A, B, M):
    """(K, n) inlier masks of K hypotheses M (K, 4, 4), the test of `inliers`."""
    Bm = np.einsum("kij,nj->kni", M[:, :3, :3], B) + M[:, None, :3, 3]
    if INLIER_TEST == "metric":
        return np.linalg.norm(A[None] - Bm, axis=2) < RANSAC_THR
    ra, rb = np.linalg.norm(A, axis=1)[None], np.linalg.norm(Bm, axis=2)
    cos = np.sum(A[None] * Bm, axis=2) / (ra * rb)
    return (np.degrees(np.arccos(np.clip(cos, -1, 1))) < BEARING_THR) & (np.abs(ra - rb) < RANGE_REL * ra)


def ransac(A, B, rng):
    best = None
    if RANSAC_CONF is None:
        for _ in range(RANSAC_IT):
            i = rng.choice(len(A), 3, replace=False)
            M = kabsch(A[i], B[i])
            inl = inliers(A, B, M)
            if best is None or inl.sum() > best.sum():
                best = inl
    else:
        done, needed = 0, RANSAC_IT
        while done < min(needed, RANSAC_IT):
            k = min(RANSAC_BATCH, RANSAC_IT - done)
            idx = np.argpartition(rng.random((k, len(A))), 3, axis=1)[:, :3]   # 3 distinct points per hypothesis
            if _fit_cpp is not None and IMAGE_CPP == "all" and INLIER_TEST == "metric":   # #182: the hypotheses in C++
                j, cnt, inl_j = _fit_cpp.ransac_batch(A, B, idx, RANSAC_THR)
                if best is None or cnt > best.sum():
                    best = inl_j
            else:
                inl = inliers_batch(A, B, kabsch_batch(A[idx], B[idx]))
                j = int(np.argmax(inl.sum(1)))                                    # first of the best, as the loop
                if best is None or inl[j].sum() > best.sum():
                    best = inl[j]
            done += k
            w = best.sum() / len(A)
            needed = 1 if w >= 1 else (np.inf if w == 0 else np.log(1 - RANSAC_CONF) / np.log(1 - w ** 3))
    if best is None or best.sum() < MIN_INL:
        return None, best
    return kabsch(A[best], B[best]), best


def pose_at(x, t):
    """Rotation vectors and translations of the sensor at times t (in periods) for parameters x."""
    w, v = x[:3], x[3:6]
    rot, tr = t[:, None] * w, t[:, None] * v
    if len(x) >= 9:                                        # "car" and "ca": angular acceleration
        rot = rot + 0.5 * t[:, None] ** 2 * x[6:9]
    if len(x) == 12:                                       # "ca": linear acceleration
        tr = tr + 0.5 * t[:, None] ** 2 * x[9:12]
    return rot, tr


def _rotmats(rv):
    """Rotation matrices of N rotation vectors (Rodrigues, series below 1e-4 rad) - the same as Rotation.from_rotvec(rv).as_matrix()
    up to rounding, without building scipy Rotation objects (#087, speed of the time fit)."""
    th = np.linalg.norm(rv, axis=1)
    small = th < 1e-4
    t = np.where(small, 1.0, th)
    a = np.where(small, 1 - th ** 2 / 6, np.sin(t) / t)
    b = np.where(small, 0.5 - th ** 2 / 24, (1 - np.cos(t)) / t ** 2)
    K = _skew(rv)
    return np.eye(3) + a[:, None, None] * K + b[:, None, None] * (K @ K)


def _rotate(rv, pts):
    """Exp(rv_i) . pts_i for N rotation vectors and points."""
    return np.einsum("nij,nj->ni", _rotmats(rv), pts)


def residual(x, p, tp, q, tq):
    rp, sp = pose_at(x, tp); rq, sq = pose_at(x, tq)
    return (_rotate(rp, p) + sp - _rotate(rq, q) - sq).ravel()


def _skew(v):
    """(N, 3, 3) cross-product matrices [v]x."""
    S = np.zeros(v.shape[:-1] + (3, 3))
    S[..., 0, 1], S[..., 0, 2], S[..., 1, 2] = -v[..., 2], v[..., 1], -v[..., 0]
    return S - np.swapaxes(S, -1, -2)


def _d_rotate(phi, y):
    """∂(Exp(phi)·x)/∂phi = −[y]x · J_l(phi), y = Exp(phi)·x, for N rotation vectors at once (N, 3, 3).

    J_l(phi) = I + (1 − cos θ)/θ² [phi]x + (θ − sin θ)/θ³ [phi]x², the left Jacobian of SO(3); the
    coefficients by their series below θ = 1e-4 (the motion of one scan is ~1 deg = 0.017 rad).
    """
    th = np.linalg.norm(phi, axis=1)
    small = th < 1e-4
    t = np.where(small, 1.0, th)
    a = np.where(small, 0.5 - th ** 2 / 24, (1 - np.cos(t)) / t ** 2)
    b = np.where(small, 1 / 6 - th ** 2 / 120, (t - np.sin(t)) / t ** 3)
    K = _skew(phi)
    Jl = np.eye(3) + a[:, None, None] * K + b[:, None, None] * K @ K
    return -_skew(y) @ Jl


def residual_jac(x, p, tp, q, tq):
    """Exact Jacobian of `residual` (3N × len(x)), for least_squares (FIT_JAC = "analytic")."""
    J = np.zeros((len(p), 3, len(x)))
    for pts, t, sign in ((p, tp, 1.0), (q, tq, -1.0)):
        rot, _ = pose_at(x, t)
        D = sign * _d_rotate(rot, _rotate(rot, pts))
        J[:, :, 0:3] += D * t[:, None, None]                       # w
        J[:, :, 3:6] += sign * t[:, None, None] * np.eye(3)        # v
        if len(x) >= 9:
            J[:, :, 6:9] += D * (0.5 * t ** 2)[:, None, None]      # alpha
        if len(x) == 12:
            J[:, :, 9:12] += sign * (0.5 * t ** 2)[:, None, None] * np.eye(3)   # a
    return J.reshape(-1, len(x))


# Jacobian of the time fit: "analytic" (residual_jac) or "2-point" (scipy's finite differences, every result
# before fast_test).  Same minimum; the analytic one needs no extra residual evaluations.
FIT_JAC = "analytic"
# #093, all off by default (= every result before):
# BUCKET_SECTORS: weight each match in the time fit so that every azimuth sector of the panorama (this many, equal) carries the same
#   total weight - features clustered in one direction (a textured facade) no longer dominate the rotation.  None = off.
# WHITEN = (sigma_r m, sigma_az rad, sigma_el rad): residual of the time fit in range / azimuth / elevation of the point, each divided by
#   its uncertainty (sigma_az, sigma_el times the range), one robust loss on the whole point (soft_l1 of |r|^2 with WHITEN_FSCALE sigma)
#   instead of one per x / y / z.  None = off.
# CROSS_CHECK: keep a match only when it is also the best one in the reverse direction (later -> earlier panorama, the same window).
BUCKET_SECTORS = None
# #135: "rotation" = the sector weights only for the ROTATION (#134: sectors improve the rotation, the translation needs the dense near
# pairs) - the translation from the same pairs fitted without sectors.  "both" = every result before.
SECTORS_PART = "both"
WHITEN, WHITEN_FSCALE = None, 1.5
CROSS_CHECK = False


def _fit_weights(q):
    """(N, 3, 3) per-point matrix A applied to the time-fit residual (A r, Jacobian A J): whitening and sector weights (#093); None = off."""
    if BUCKET_SECTORS is None and WHITEN is None:
        return None
    n = len(q)
    A = np.broadcast_to(np.eye(3), (n, 3, 3)).copy()
    if WHITEN is not None:
        sr, saz, sel = WHITEN
        rng_ = np.maximum(np.linalg.norm(q, axis=1), 1e-3)
        er = q / rng_[:, None]
        ea = np.cross(np.array([0.0, 0.0, 1.0]), er)
        ea /= np.maximum(np.linalg.norm(ea, axis=1), 1e-9)[:, None]
        ee = np.cross(er, ea)
        U = np.stack([er, ea, ee], axis=1)                                   # rows: range, azimuth, elevation directions
        A = np.stack([1.0 / sr * np.ones(n), 1.0 / (rng_ * saz), 1.0 / (rng_ * sel)], axis=1)[:, :, None] * U
    if BUCKET_SECTORS is not None:
        az = np.arctan2(q[:, 1], q[:, 0])
        sec = np.minimum(((az + np.pi) / (2 * np.pi) * BUCKET_SECTORS).astype(int), BUCKET_SECTORS - 1)
        cnt = np.bincount(sec, minlength=BUCKET_SECTORS).astype(float)
        occupied = (cnt > 0).sum()
        w = n / (occupied * cnt[sec])                                       # each occupied sector: total weight n / occupied
        A = A * np.sqrt(w)[:, None, None]                                   # squared residuals weighted by w
    return A


def _whitened(x, p, tp, q, tq, A):
    return np.einsum("nij,nj->ni", A, residual(x, p, tp, q, tq).reshape(-1, 3)).ravel()


def _whitened_jac(x, p, tp, q, tq, A):
    J = residual_jac(x, p, tp, q, tq).reshape(len(p), 3, -1)
    return np.einsum("nij,njk->nik", A, J).reshape(-1, len(x))


def _point_norm(x, p, tp, q, tq, A):
    """One residual per point, |A r| (#093 WHITEN: one robust loss on the whole point, rotation invariant)."""
    return np.linalg.norm(np.einsum("nij,nj->ni", A, residual(x, p, tp, q, tq).reshape(-1, 3)), axis=1)


def _point_norm_jac(x, p, tp, q, tq, A):
    r = np.einsum("nij,nj->ni", A, residual(x, p, tp, q, tq).reshape(-1, 3))
    nr = np.maximum(np.linalg.norm(r, axis=1), 1e-9)
    J = np.einsum("nij,njk->nik", A, residual_jac(x, p, tp, q, tq).reshape(len(p), 3, -1))
    return np.einsum("ni,nik->nk", r / nr[:, None], J)


def fit_time(p, tp, q, tq, M0, model="cv"):
    """Motion over the later scan, pose(1), using each point's time → (4x4, inliers)."""
    x = np.concatenate([Rotation.from_matrix(M0[:3, :3]).as_rotvec(), M0[:3, 3]])
    keep = np.ones(len(p), bool)
    extra = {"cv": 0, "car": 3, "ca": 6}[model]
    for stage in (["cv"] if model == "cv" else ["cv", model]):
        if stage != "cv":
            x = np.concatenate([x, np.zeros(extra)])    # start from the constant-velocity solution
        for _ in range(3):
            A = _fit_weights(q[keep])
            if A is None and _fit_cpp is not None and IMAGE_CPP == "all" and FIT_JAC == "analytic":   # #182: the same cost in C++
                x = _fit_cpp.fit_soft_l1(x, p[keep], tp[keep], q[keep], tq[keep], 0.05)
            elif A is None:
                x = least_squares(residual, x, args=(p[keep], tp[keep], q[keep], tq[keep]),
                                  jac=residual_jac if FIT_JAC == "analytic" else "2-point",
                                  loss="soft_l1", f_scale=0.05).x
            elif WHITEN is None:                                       # sector weights only: the same loss per coordinate (#093)
                x = least_squares(_whitened, x, args=(p[keep], tp[keep], q[keep], tq[keep], A), jac=_whitened_jac,
                                  loss="soft_l1", f_scale=0.05).x
            else:                                                      # whitened, one robust loss per point (#093): soft_l1 of
                for _ in range(2):                                     # |A r|^2 by reweighting (IRLS) - |A r| as the residual is
                    z = _point_norm(x, p[keep], tp[keep], q[keep], tq[keep], A) ** 2 / WHITEN_FSCALE ** 2   # not smooth at 0 (slow)
                    Aw = A * ((1.0 + z) ** -0.25)[:, None, None]       # sqrt of the soft_l1 weight 1 / sqrt(1 + z)
                    x = least_squares(_whitened, x, args=(p[keep], tp[keep], q[keep], tq[keep], Aw), jac=_whitened_jac).x
            r = np.linalg.norm(residual(x, p, tp, q, tq).reshape(-1, 3), axis=1)
            keep = r < FIT_THR
            if keep.sum() < MIN_INL:
                return None, keep
    rot, tr = pose_at(x, np.array([1.0]))
    M = np.eye(4); M[:3, :3] = Rotation.from_rotvec(rot[0]).as_matrix(); M[:3, 3] = tr[0]
    fit_time.last_params = x                              # the whole curve, for deskew_curve (#033)
    return M, keep


def make_detector(detector="sift", surf_hessian=100.0, surf_upright=False):
    """Feature detector + descriptor for the panorama: "sift" (default) or "surf".

    Both give float descriptors, so the same L2 matcher and ratio test serve both.  SURF is
    patented and lives in opencv-contrib's xfeatures2d, which only has it in a build with
    OPENCV_ENABLE_NONFREE=ON; the pip wheels (opencv-python, opencv-contrib-python) do not.
    `surf_hessian`: Hessian threshold, higher = fewer, stronger keypoints (OpenCV default 100).
    `surf_upright`: skip the orientation (U-SURF); the panorama is never rotated in-plane.
    """
    if detector == "sift":
        return cv2.SIFT_create()
    if detector == "surf":
        try:
            return cv2.xfeatures2d.SURF_create(hessianThreshold=surf_hessian, upright=surf_upright)
        except (AttributeError, cv2.error) as e:
            raise RuntimeError(
                "image_deskew.detector = 'surf' needs OpenCV with contrib and OPENCV_ENABLE_NONFREE=ON; "
                f"this cv2 ({cv2.__version__}) has no SURF.  Build it with:  ENABLE_CONTRIB=1 ENABLE_HEADLESS=1 "
                'CMAKE_ARGS="-DOPENCV_ENABLE_NONFREE=ON" pip install --no-binary opencv-contrib-python-headless '
                "opencv-contrib-python-headless  (after uninstalling opencv-python)"
            ) from e
    if detector == "orb":                                  # #088: binary descriptors, Hamming matching; many keypoints, small border / patch
        return cv2.ORB_create(nfeatures=ORB_FEATURES, scaleFactor=1.2, nlevels=8, edgeThreshold=15, patchSize=15, fastThreshold=10)
    if detector == "akaze":                                # #181: upright AKAZE (M-LDB binary descriptor without orientation), Hamming matching
        x = cv2.xfeatures2d if hasattr(cv2, "xfeatures2d") and hasattr(cv2.xfeatures2d, "AKAZE_create") else cv2   # OpenCV 5: in contrib
        return x.AKAZE_create(descriptor_type=x.AKAZE_DESCRIPTOR_MLDB_UPRIGHT, threshold=AKAZE_THRESHOLD)
    raise ValueError(f"unknown detector {detector!r}: 'sift', 'surf', 'orb' or 'akaze'")


AKAZE_THRESHOLD = 0.001                                    # #181: OpenCV default detector response threshold


# #093: scale of the panorama for the detector only (1 = every result before).  < 1: the image is shrunk before SURF / SIFT (cost ~ area)
# and the keypoints are mapped back to full-resolution pixels, so matching, lookup and the fit are unchanged.
DETECT_SCALE = 1.0


KP_GRID = None    # #199: (cell px, n) - keep at most the n strongest keypoints per cell x cell block of the panorama (even spread); None = all


def _grid_filter(kps, desc):
    cell, n = KP_GRID
    if not kps:
        return kps, desc
    key = np.array([(int(k.pt[1] // cell), int(k.pt[0] // cell)) for k in kps])
    resp = np.array([k.response for k in kps])
    order = np.lexsort((-resp, key[:, 1], key[:, 0]))                   # by cell, strongest first
    k_sorted = key[order]
    first = np.r_[True, (k_sorted[1:] != k_sorted[:-1]).any(1)]
    rank = np.arange(len(order)) - np.maximum.accumulate(np.where(first, np.arange(len(order)), 0))
    keep = np.sort(order[rank < n])                                     # original order kept
    return tuple(kps[i] for i in keep), desc[keep]


def _detect(detector, img):
    kps, desc = _detect_raw(detector, img)
    if KP_GRID is not None and desc is not None:
        kps, desc = _grid_filter(kps, desc)
    if desc is not None and desc.dtype == np.uint8 and desc.shape[1] % 8:   # #181: AKAZE M-LDB is 61 bytes; zero bytes added to a multiple
        desc = np.pad(desc, ((0, 0), (0, -desc.shape[1] % 8)))              # of 8 for the C++ Hamming matcher - the distances do not change
    return kps, desc


def _detect_raw(detector, img):
    if DETECT_SCALE == 1.0:
        return detector.detectAndCompute(img, None)
    h, w = img.shape[:2]
    small = cv2.resize(img, (max(1, round(w * DETECT_SCALE)), max(1, round(h * DETECT_SCALE))), interpolation=cv2.INTER_AREA)
    kps, desc = detector.detectAndCompute(small, None)
    sx, sy = w / small.shape[1], h / small.shape[0]
    kps = [cv2.KeyPoint((k.pt[0] + 0.5) * sx - 0.5, (k.pt[1] + 0.5) * sy - 0.5, k.size * sx, k.angle, k.response, k.octave, k.class_id)
           for k in kps]
    return kps, desc


def features(xyz, ts, inten, ring, detector, normalisation="none", _clahe=[]):
    """Panorama + keypoints/descriptors (SIFT or SURF, see make_detector) of one raw scan:
    (P, T, valid, keypoints, descriptors, t_start, panorama image).  match_motion uses the first six.

    normalisation (#075): "none" = the intensity as given (after the estimator's fixed intensity_scale); "gain" = this scan's
    intensity scaled so that its 99th percentile (points beyond MIN_RANGE) is 255; "gain_clahe" = gain, then local contrast
    equalisation of the image (CLAHE 3.0, tiles 4 x 16, as the range image, #058)."""
    ok = ~np.isnan(xyz).any(axis=1) & (np.linalg.norm(xyz, axis=1) > MIN_RANGE)
    if normalisation in ("gain", "gain_clahe") and ok.any():
        inten = inten * (255.0 / max(float(np.percentile(inten[ok], 99)), 1e-6))
    big, P, T, valid = panorama(xyz[ok], ts[ok], inten[ok], ring[ok])
    if normalisation == "gain_clahe":
        if not _clahe:
            _clahe.append(cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 16)))
        big = _clahe[0].apply(big)
    kps, desc = _detect(detector, big)
    return P, T, valid, kps, desc, ts[ok].min(), big


SATURATE_ROWS, SATURATE_FRAC = (128, 256, 512, 1024), 0.95


def saturate_upscale(xyz, ts, inten, ring, detector):
    """#198: the vertical upscaling where the detector's keypoints saturate - the smallest panorama height (of SATURATE_ROWS) with at least
    SATURATE_FRAC of the largest keypoint count, measured on this (first) scan.  #195: keypoints grow with the rows and saturate (~256 rows at
    16 beams, ~512 at 64); fewer than ~128 rows breaks the matching.  Sensor-independent: no constant per sensor."""
    global UP
    saved, nr = UP, len(np.unique(ring))
    counts = {}
    try:
        for rows in SATURATE_ROWS:
            UP = rows / nr
            kps, _ = _detect(detector, panorama(xyz, ts, inten, ring)[0])
            counts[rows] = len(kps)
    finally:
        UP = saved
    top = max(counts.values())
    rows = min(r for r, c in counts.items() if c >= SATURATE_FRAC * top)
    saturate_upscale.last = counts
    return rows / nr


def range_features(xyz, ts, ring, detector, _clahe=[]):
    """Like `features`, but the panorama is log range (1-60 m -> 0-255) with local contrast (CLAHE 4x16) instead of
    intensity (#057-#058): where intensity repeats (rows of identical windows) the depth structure does not."""
    if not _clahe:
        _clahe.append(cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 16)))
    r = np.linalg.norm(xyz, axis=1)
    ok = ~np.isnan(xyz).any(axis=1) & (r > MIN_RANGE)
    val = np.clip(255 * np.log(np.clip(r, 1, 60)) / np.log(60), 0, 255)
    big, P, T, valid = panorama(xyz[ok], ts[ok], val[ok], ring[ok])
    big = _clahe[0].apply(big)
    kps, desc = _detect(detector, big)
    return P, T, valid, kps, desc, ts[ok].min(), big


_DEFAULT = object()   # "use the module-level knob" (scripts set STUCK_* after import)


try:                                                              # #087: C++ guided matching, if built
    from kiss_slam import _guided_match as _guided_cpp
except ImportError:
    _guided_cpp = None

try:                                                              # #182: C++ RANSAC hypotheses, soft_l1 time fit, panorama scatter, if built
    from kiss_slam import _image_fit as _fit_cpp
except ImportError:
    _fit_cpp = None
# #182: use them (scripts/build_image_fit.sh).  "all" = the three; "exact" = only the bit-identical one (panorama scatter); "off" = Python.
IMAGE_CPP = __import__("os").environ.get("KISS_IMAGE_CPP", "off")   # #182: env, so a whole run (and its worker) can switch


def predict_pixels(f1, f2, M):
    """Where the keypoints of panorama f1 should appear in panorama f2 if the motion between them is M (p = R q + t, p in f1's scan):
    q = R^T (p - t), projected with the panorama's own mapping - column from azimuth, row from elevation interpolated over f2's per-ring
    median elevations.  Returns (N, 2) pixel positions and a mask of the keypoints with a 3D point (#089)."""
    P1, T1, v1, kp1 = f1[:4]
    P2, v2 = f2[0], f2[2]
    xy1 = np.array([k.pt for k in kp1]).reshape(-1, 2)
    p, _, ok = lookup_batch(P1, T1, v1, xy1, False)
    q = (p - M[:3, 3]) @ M[:3, :3]                                       # R^T (p - t), row-wise
    col = (np.arctan2(q[:, 1], q[:, 0]) + np.pi) / (2 * np.pi) * W - 0.5  # pixel coordinates of the keypoint image (centre convention)
    el2 = np.full(P2.shape[0], np.nan)
    for r in range(P2.shape[0]):
        if v2[r].any():
            pr = P2[r][v2[r]]
            el2[r] = np.median(np.arctan2(pr[:, 2], np.linalg.norm(pr[:, :2], axis=1)))
    rows = np.flatnonzero(np.isfinite(el2))
    el = np.arctan2(q[:, 2], np.linalg.norm(q[:, :2], axis=1))
    rf = np.interp(-el, -el2[rows], rows.astype(float))                 # elevation decreases with the row
    pred = np.column_stack([col % W, (rf + 0.5) * UP - 0.5])
    return pred, ok & np.isfinite(pred).all(1)


def _px(columns):
    """Columns of the 1024-column panorama -> columns of the current one (#093: the guided-matching windows and the fast-rotation
    threshold are angles, set as columns at W = 1024; exactly the same at 1024)."""
    return columns * W / 1024.0


def guided_matches(kp1, d1, kp2, d2, shift=0.0, window=None, centres=None):
    """Matches of kp1 among the kp2 within GUIDED_WINDOW columns / GUIDED_ROWS rings of the same pixel (#087), as cv2.DMatch.

    Consecutive scans are 0.1 s apart: a feature moves a few pixels (fast rotation ~30), so a look-alike elsewhere in the panorama
    (repeated facades) cannot win, and the cost is N x GUIDED_K descriptor distances instead of N x M.  Ratio test (RATIO) among the
    candidates of the window; one candidate only is accepted when its distance is below the median best distance."""
    if len(kp1) < 2 or len(kp2) < 2:
        return []
    a = np.array([k.pt for k in kp1]); b = np.array([k.pt for k in kp2])
    a = a.copy(); a[:, 0] = (a[:, 0] + shift) % W                  # #088: window centred on the predicted position
    if centres is not None:                                          # #089: per-keypoint prediction where available
        a[centres[1]] = centres[0][centres[1]]
    binary = d1.dtype == np.uint8                                 # ORB (#088): Hamming distance
    if _guided_cpp is not None:                                   # C++ (scripts/build_guided_match.sh): rectangular window, all candidates
        f = _guided_cpp.guided_match_hamming if binary else _guided_cpp.guided_match
        j, best, second = f(a.astype(np.float32), d1, b.astype(np.float32), d2, float(W), float(window or _px(GUIDED_WINDOW)), float(GUIDED_ROWS * UP))
        fin = np.isfinite(best)
        if not fin.any():
            return []
        ok = fin & ((best < RATIO * second) | (~np.isfinite(second) & (best < np.median(best[fin]))))
        return [cv2.DMatch(int(i), int(j[i]), float(best[i])) for i in np.flatnonzero(ok)]
    from scipy.spatial import cKDTree
    sy = _px(GUIDED_WINDOW) / (GUIDED_ROWS * UP)                  # anisotropic window as a circle of radius GUIDED_WINDOW
    bb = np.concatenate([b, b + [W, 0], b - [W, 0]]); src = np.tile(np.arange(len(b)), 3)
    tree = cKDTree(bb * [1.0, sy])
    dist, idx = tree.query(a * [1.0, sy], k=min(GUIDED_K, len(bb)), distance_upper_bound=_px(GUIDED_WINDOW))
    dist, idx = np.atleast_2d(dist), np.atleast_2d(idx)
    valid = np.isfinite(dist)
    cand = np.where(valid, src[np.minimum(idx, len(bb) - 1)], 0)
    if d1.dtype == np.uint8:                                      # ORB: Hamming
        dd = np.unpackbits(d1[:, None, :] ^ d2[cand], axis=2).sum(axis=2).astype(np.float32)
    else:
        dd = np.linalg.norm(d1[:, None, :].astype(np.float32) - d2[cand].astype(np.float32), axis=2)
    dd[~valid] = np.inf
    order = np.argsort(dd, axis=1)
    best, second = np.take_along_axis(dd, order[:, :1], 1)[:, 0], np.take_along_axis(dd, order[:, 1:2], 1)[:, 0] if dd.shape[1] > 1 else np.full(len(dd), np.inf)
    j = cand[np.arange(len(cand)), order[:, 0]]
    fin = np.isfinite(best)
    ok = fin & ((best < RATIO * second) | (~np.isfinite(second) & (best < np.median(best[fin]) if fin.any() else False)))
    return [cv2.DMatch(int(i), int(j[i]), float(best[i])) for i in np.flatnonzero(ok)]


def refine_rotation_bearings(x, p, tp, q, tq):
    """The rotation parameters of x re-estimated from bearings (#087): q, moved by the fitted motion into the sensor frame at the
    time of p, must point where p points.  Matches nearer than BEARING_MIN_RANGE are left out (their direction depends on the
    translation); translation parameters fixed.  Returns the new x, or None when too few far matches."""
    far = (np.linalg.norm(p, axis=1) > BEARING_MIN_RANGE) & (np.linalg.norm(q, axis=1) > BEARING_MIN_RANGE)
    if far.sum() < BEARING_MIN_PAIRS:
        return None
    p, tp, q, tq = p[far], tp[far], q[far], tq[far]
    up = p / np.linalg.norm(p, axis=1, keepdims=True)
    idx = [0, 1, 2] + ([6, 7, 8] if len(x) >= 9 else [])

    def res(r):
        xx = x.copy(); xx[idx] = r
        rp, sp = pose_at(xx, tp); rq, sq = pose_at(xx, tq)
        d = Rotation.from_rotvec(rp).inv().apply(Rotation.from_rotvec(rq).apply(q) + sq - sp)
        return (d / np.linalg.norm(d, axis=1, keepdims=True) - up).ravel()
    r = least_squares(res, x[idx], loss="soft_l1", f_scale=0.005).x
    xx = x.copy(); xx[idx] = r
    return xx


def _cross_checked(good, kp1, d1, kp2, d2, bf, guided):
    """The matches i -> j for which i is also the best match of j among kp1 (#093).  Guided: the reverse search in the same window
    (centred at minus the shift); brute force: over the whole panorama.  No ratio test in the reverse direction."""
    if guided and _guided_cpp is not None and d1.dtype != np.uint8:
        a = np.array([k.pt for k in kp2]); b = np.array([k.pt for k in kp1])
        a[:, 0] = (a[:, 0] - GUIDED_SHIFT) % W
        win = float(_px(GUIDED_WINDOW) + 0.5 * abs(GUIDED_SHIFT))
        rev, best, _ = _guided_cpp.guided_match(a.astype(np.float32), d2, b.astype(np.float32), d1, float(W), win, float(GUIDED_ROWS * UP))
        rev = np.where(np.isfinite(best), rev, -1)
    else:
        rev = np.full(len(kp2), -1)
        for m in bf.match(d2, d1):
            rev[m.queryIdx] = m.trainIdx
    return [m for m in good if rev[m.trainIdx] == m.queryIdx]


def match_motion(f1, f2, period, rng, bf, model="cv", subpixel=False,
                 stuck_min=_DEFAULT, floor_only=_DEFAULT, elev=_DEFAULT, range_=_DEFAULT):
    """Motion of the later of two consecutive scans → (rigid M0, timed M1, n inliers).

    The stuck-match filter (#025-#027) takes its knobs from the arguments; each one left at
    its default reads the module global (STUCK_MIN, STUCK_FLOOR_ONLY, STUCK_ELEV, STUCK_RANGE).
    """
    if stuck_min is _DEFAULT:
        stuck_min = STUCK_MIN
    if floor_only is _DEFAULT:
        floor_only = STUCK_FLOOR_ONLY
    if elev is _DEFAULT:
        elev = STUCK_ELEV
    if range_ is _DEFAULT:
        range_ = STUCK_RANGE
    # Reset before any early return: a failed scan must not leave the previous scan's ratio for
    # ScanMotionEstimator to append again (TRANS_MODE "auto").
    match_motion.last_ratio = None
    P1, T1, v1, kp1, d1, _ = f1[:6]; P2, T2, v2, kp2, d2, t_start = f2[:6]
    match_motion.last_good = []
    match_motion.last_pairs = None
    if d1 is None or d2 is None or len(kp1) < 2 or len(kp2) < 2:
        return None, None, 0
    global GUIDED_SHIFT
    good = None
    fast = abs(GUIDED_SHIFT) > _px(GUIDED_MAX_SHIFT)
    use_motion = GUIDED_PREDICTION == "motion" or (GUIDED_PREDICTION == "hybrid" and fast)        # #089: hybrid = motion only when fast
    if GUIDED_WINDOW is not None and use_motion and GUIDED_PRED_MOTION is not None:   # #089: per-keypoint prediction
        good = guided_matches(kp1, d1, kp2, d2, GUIDED_SHIFT, _px(GUIDED_WINDOW), predict_pixels(f1, f2, GUIDED_PRED_MOTION))
        if len(good) < GUIDED_MIN_MATCHES:
            good = None
    elif GUIDED_WINDOW is not None and abs(GUIDED_SHIFT) <= _px(GUIDED_MAX_SHIFT):   # #087; #088: only when turning slowly, window at the previous shift
        good = guided_matches(kp1, d1, kp2, d2, GUIDED_SHIFT, _px(GUIDED_WINDOW) + 0.5 * abs(GUIDED_SHIFT))   # wider when turning fast
        if len(good) < GUIDED_MIN_MATCHES:
            good = None
    guided_used = good is not None
    if good is None:
        good = [m for m, n in bf.knnMatch(d1, d2, k=2) if m.distance < RATIO * n.distance]
    if CROSS_CHECK and good:                                           # #093: mutual best match, the same window reversed
        good = _cross_checked(good, kp1, d1, kp2, d2, bf, guided_used)
    if GUIDED_WINDOW is not None and len(good) >= GUIDED_MIN_MATCHES:
        dx = np.array([kp2[m.trainIdx].pt[0] - kp1[m.queryIdx].pt[0] for m in good])
        GUIDED_SHIFT = float(np.median((dx + W / 2) % W - W / 2))      # wrapped column shift, for the next scan
    match_motion.last_good = good                      # for the images of rejected pairs (#054)
    if PIXEL_MASK is not None:
        def on_mask(kp):
            u, v = kp.pt
            r = min(max(int(round((v + 0.5) / UP - 0.5)), 0), PIXEL_MASK.shape[0] - 1)
            return PIXEL_MASK[r, int(round(u)) % W]
        good = [m for m in good if not on_mask(kp2[m.trainIdx])]
    xy1 = np.array([kp1[m.queryIdx].pt for m in good]).reshape(-1, 2)
    xy2 = np.array([kp2[m.trainIdx].pt for m in good]).reshape(-1, 2)
    p, tp, ok1 = lookup_batch(P1, T1, v1, xy1, subpixel)          # #087: vectorised, same results as the per-keypoint lookup
    q, tq, ok2 = lookup_batch(P2, T2, v2, xy2, subpixel)
    both = ok1 & ok2
    if both.sum() < MIN_INL:
        return None, None, 0
    p, q = p[both], q[both]
    tp = (tp[both] - t_start) / period                               # t = 0: start of the later scan
    tq = (tq[both] - t_start) / period
    if stuck_min is not None:
        moved = np.linalg.norm(p - q, axis=1) >= stuck_min
        if floor_only:
            q_elev = np.degrees(np.arctan2(q[:, 2], np.linalg.norm(q[:, :2], axis=1)))
            moved |= ~((q_elev < elev) & (np.linalg.norm(q, axis=1) < range_))
        p, q, tp, tq = p[moved], q[moved], tp[moved], tq[moved]
        if len(p) < MIN_INL:
            return None, None, 0
    M0, inl = ransac(p, q, rng)
    if M0 is None:
        return None, None, 0
    M1, keep = fit_time(p[inl], tp[inl], q[inl], tq[inl], M0, model)
    if SECTORS_PART == "rotation" and globals()["BUCKET_SECTORS"] is not None and M1 is not None:   # #135
        saved_sec = globals()["BUCKET_SECTORS"]
        globals()["BUCKET_SECTORS"] = None
        try:
            params_sec = fit_time.last_params
            Mn, _ = fit_time(p[inl], tp[inl], q[inl], tq[inl], M0, model)
        finally:
            globals()["BUCKET_SECTORS"] = saved_sec
        fit_time.last_params = params_sec
        if Mn is not None:
            M1 = M1.copy(); M1[:3, 3] = Mn[:3, 3]
    match_motion.last_pairs = (p[inl][keep], tp[inl][keep], q[inl][keep], tq[inl][keep]) if M1 is not None else None   # #090
    if DROP_STATIONARY and M1 is not None:                 # #132
        pk, tpk, qk, tqk = p[inl][keep], tp[inl][keep], q[inl][keep], tq[inl][keep]
        x = fit_time.last_params
        r_fit = np.linalg.norm(residual(x, pk, tpk, qk, tqk).reshape(-1, 3), axis=1)
        r_id = np.linalg.norm(residual(np.zeros_like(x), pk, tpk, qk, tqk).reshape(-1, 3), axis=1)
        mv = r_id > r_fit                                   # the #130 rule; only while clearly moving (|t| > 2 x the median residual):
        moving = np.linalg.norm(M1[:3, 3]) > 2.0 * np.median(r_fit)   # at rest zero motion fits every pair and the rule would drop half
        if moving and (~mv).any() and mv.sum() >= MIN_INL:
            Ms, keep_s = fit_time(pk[mv], tpk[mv], qk[mv], tqk[mv], M1, model)
            if Ms is not None:
                M1 = Ms
                match_motion.last_pairs = (pk[mv][keep_s], tpk[mv][keep_s], qk[mv][keep_s], tqk[mv][keep_s])
                keep = keep.copy(); idx = np.flatnonzero(keep); keep[idx[~mv]] = False; keep[idx[mv][~keep_s]] = False
    if BEARING_MIN_RANGE is not None and M1 is not None:   # #087: rotation from bearings, translation from 3D
        k = keep
        xb = refine_rotation_bearings(fit_time.last_params, p[inl][k], tp[inl][k], q[inl][k], tq[inl][k])
        if xb is not None:
            rot, tr = pose_at(xb, np.array([1.0]))
            M1 = np.eye(4); M1[:3, :3] = Rotation.from_rotvec(rot[0]).as_matrix(); M1[:3, 3] = tr[0]
            fit_time.last_params = xb
    if TRANS_FACTOR != 1.0:
        for M in (M0, M1):
            if M is not None:
                M[:3, 3] = TRANS_FACTOR * M[:3, 3]
    if TRANS_MIN_RANGE is not None:
        far = np.linalg.norm(q[inl], axis=1) > TRANS_MIN_RANGE
        if far.sum() >= TRANS_MIN_PAIRS:
            pf, qf = p[inl][far], q[inl][far]
            if TRANS_MODE == "auto":                            # only measure; the estimator applies the running median
                M = M1 if M1 is not None else M0
                t_all = M[:3, 3]; n2 = float(t_all @ t_all)
                if n2 > 1e-12:
                    t_far = np.median(pf - (M[:3, :3] @ qf.T).T, axis=0)
                    match_motion.last_ratio = float(t_far @ t_all) / n2
                match_motion.last_params = fit_time.last_params if M1 is not None else None
                match_motion.last_t_start = t_start
                return M0, M1, int(keep.sum()) if M1 is not None else 0
            for M in (M0, M1):                                  # p = R q + t  (same convention as the GT delta)
                if M is None:
                    continue
                t_far = np.median(pf - (M[:3, :3] @ qf.T).T, axis=0)
                if TRANS_MODE == "magnitude":
                    t_all = M[:3, 3]; n2 = float(t_all @ t_all)
                    if n2 > 1e-12:
                        M[:3, 3] = np.clip(float(t_far @ t_all) / n2, *TRANS_CLIP) * t_all
                else:
                    M[:3, 3] = t_far
    match_motion.last_params = fit_time.last_params if M1 is not None else None
    match_motion.last_t_start = t_start
    return M0, M1, int(keep.sum()) if M1 is not None else 0


def deskew_curve(xyz, ts, params, t_start, period=0.1, min_range=0.0, max_range=1e9):
    """Deskew every point with the sensor pose at ITS OWN time on the fitted curve (#033).

    KISS spreads the whole-scan motion `delta` uniformly over the sweep: p' = exp((s-1) log delta) p.
    Here p' = pose(1)^-1 · pose(tau_i) · p_i with pose(tau) = [R(tau w + tau^2/2 alpha), tau v] from
    the "car"/"ca" fit, tau_i = (t_i - t_start) / period, so the angular acceleration measured in
    the image is used per point, not only for a better total.  Output in the end-of-sweep frame,
    then the KISS range crop (strict inequalities, as Preprocessing.cpp) so the result replaces
    `Preprocessor.preprocess` one for one.  With a "cv" fit this equals the KISS deskew.
    """
    tau = (ts - t_start) / period
    rot, tr = pose_at(params, tau)
    r1, t1 = pose_at(params, np.array([1.0]))
    R1 = Rotation.from_rotvec(r1[0])
    world = Rotation.from_rotvec(rot).apply(xyz) + tr             # in the start-of-sweep frame
    out = R1.inv().apply(world - t1[0])                           # into the end-of-sweep frame
    rng_ = np.linalg.norm(out, axis=1)
    return out[(rng_ < max_range) & (rng_ > min_range)]


class ScanMotionEstimator:
    """Online: feed raw scans in order, get each scan's motion from it and the previous one."""

    def __init__(self, period=0.1, seed=0, model="cv", subpixel=False,
                 stuck_min=_DEFAULT, floor_only=_DEFAULT, elev=_DEFAULT, range_=_DEFAULT,
                 detector="sift", surf_hessian=100.0, surf_upright=False, intensity_scale=1.0,
                 gate_min_matches=None, gate_max_rotation_deg=None, gate_max_rotation_change_deg=None,
                 save_rejected_dir=None, range_motion=None, range_hessian=10.0,
                 intensity_normalisation="none", panorama_width=None, bearing_min_range=None, guided_window=None,
                 guided_prediction="shift", multi_baseline=False, fuse_range=False, panorama_up=None,
                 fit_sectors=None, whiten=None, cross_check=False, detect_scale=1.0, drop_stationary=False, sectors_part="both"):
        """`stuck_min`, `floor_only`, `elev`, `range_`: the stuck-match filter (#025-#027);
        left at their defaults they read the module globals STUCK_* at each call.
        `detector`, `surf_hessian`, `surf_upright`: the panorama features, see make_detector.
        `intensity_scale`: multiplies the raw intensity (1.0 Hesai, 255/1024 Ouster, #041).
        Plausibility gate (#054), each None = off: a motion is rejected (returned as None) when fewer than
        `gate_min_matches` matches support it, when its rotation over the scan exceeds `gate_max_rotation_deg`, or
        when its rotation vector differs from the last ACCEPTED one by more than `gate_max_rotation_change_deg`.
        `save_rejected_dir`: for every rejected or failed scan, both panoramas, their matches and a line in
        rejected.csv (scan, time, reason, matches, rotation), to inspect the pairs by eye.
        `range_motion` (#058): also measure the motion from the RANGE panorama (range_features, SURF threshold
        `range_hessian`, own RNG so the intensity estimate is unchanged): "fallback" returns it when the intensity motion
        failed or was rejected; "candidate" only keeps it in self.last_range_motion (a starting point for the ICP)."""
        self.period, self.model, self.subpixel = period, model, subpixel
        # #075: per-scan intensity normalisation (replaces the fixed intensity_scale when not "none") and the panorama
        # columns (module-wide W, so every panorama of this process; None = keep W, 1024).  2048 = the Hilti Ouster's own.
        self.intensity_normalisation = intensity_normalisation
        self.auto_width = panorama_width == "auto"   # #093: the sensor's own columns, measured from the first scan
        if panorama_width is not None and not self.auto_width:
            global W
            W = int(panorama_width)
        self.auto_up = panorama_up in ("auto", "saturate")   # #093: square pixels / #198: feature saturation, measured from the first scan
        self.up_mode = panorama_up
        self.panorama_up = None if self.auto_up else panorama_up
        if panorama_up is not None and not self.auto_up:   # #089: vertical upscaling of the panorama (8 = every result before; 4 for 128 beams)
            global UP
            UP = int(panorama_up)
        global BUCKET_SECTORS, WHITEN, CROSS_CHECK              # #093: module-wide, as W
        global DETECT_SCALE, DROP_STATIONARY
        DETECT_SCALE = float(detect_scale)
        DROP_STATIONARY = bool(drop_stationary)                 # #132
        BUCKET_SECTORS = None if fit_sectors is None else int(fit_sectors)
        global SECTORS_PART
        SECTORS_PART = sectors_part
        WHITEN = None if whiten is None else tuple(float(v) for v in whiten)
        CROSS_CHECK = bool(cross_check)
        global BEARING_MIN_RANGE, GUIDED_WINDOW               # #087: module-wide, as W
        global GUIDED_SHIFT, GUIDED_PREDICTION
        BEARING_MIN_RANGE, GUIDED_WINDOW = bearing_min_range, guided_window
        GUIDED_SHIFT = 0.0
        GUIDED_PREDICTION = guided_prediction
        self.intensity_scale = intensity_scale
        self.stuck_min, self.floor_only, self.elev, self.range_ = stuck_min, floor_only, elev, range_
        self.detector_name = detector
        self.detector = make_detector(detector, surf_hessian, surf_upright)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING if detector in ("orb", "akaze") else cv2.NORM_L2)   # #088 / #181: ORB, AKAZE binary
        self.range_bf = cv2.BFMatcher(cv2.NORM_L2)          # #181: the range panorama always uses SURF (float descriptors)
        self.rng = np.random.default_rng(seed)
        self.prev = None
        self.last_motion = None
        # #090: joint fit with more matches - multi_baseline: also scan k-2 <-> k (times -2..-1 periods, the same motion curve);
        # fuse_range: also the range-panorama matches of k-1 <-> k (computed on every scan, cached for the next).
        self.multi_baseline, self.fuse_range = multi_baseline, fuse_range
        self.prev2 = None; self.prev_range_feat = None
        self._inl_hist = []                                   # #152
        self.n_joint = 0; self.joint_pairs = []
        self.ratios = []; self.last_factor = 1.0
        self.gate = (gate_min_matches, gate_max_rotation_deg, gate_max_rotation_change_deg)
        self.save_rejected_dir = save_rejected_dir
        self.k = -1                                   # index of the current scan
        self.last_accepted_rotvec = None
        self.last_reason = None
        self.range_motion = range_motion
        if range_motion is not None:
            self.range_detector = make_detector("surf", range_hessian)
            self.range_rng = np.random.default_rng(seed + 1000)
            self.prev_range = None
            self.prev_raw = None                  # "fallback": the previous raw scan, for a lazy range motion
        self.last_range_motion = None
        self.n_range_used = 0

    def motion(self, xyz, ts, inten, ring):
        self.k += 1
        if self.k == 0 and self.auto_width:                   # #093: first scan - the columns, then the upscaling (it depends on them)
            global W
            W = native_width(xyz, ts, ring)
            print(f"ScanMotionEstimator| panorama columns auto (sensor's own): {W}", flush=True)
        if self.auto_up and self.panorama_up is None:          # #093: first scan - set the upscaling once, for the whole run
            global UP
            if self.up_mode == "saturate":
                inten_s = inten * self.intensity_scale if (self.intensity_scale != 1.0 and self.intensity_normalisation == "none") else inten
                UP = self.panorama_up = saturate_upscale(xyz, ts, inten_s, ring, self.detector)
                print(f"ScanMotionEstimator| panorama upscaling saturate: {UP:g} ({round(UP * len(np.unique(ring)))} rows; "
                      f"keypoints per height {saturate_upscale.last}", flush=True)
            else:
                UP = self.panorama_up = square_upscale(xyz, ring)
                print(f"ScanMotionEstimator| panorama upscaling auto (square pixels): {UP:.4f} ({round(UP * len(np.unique(ring)))} rows)", flush=True)
        self.last_reason = None
        if self.intensity_scale != 1.0 and self.intensity_normalisation == "none":
            inten = inten * self.intensity_scale
        cur = features(xyz, ts, inten, ring, self.detector, self.intensity_normalisation)
        prev, self.prev = self.prev, cur
        self.last_params, self.last_t_start = None, cur[5]
        if prev is None:
            if self.range_motion is not None:
                self._range(xyz, ts, ring, None, 0)
            return None, 0
        global GUIDED_PRED_MOTION
        GUIDED_PRED_MOTION = self.last_motion if GUIDED_PREDICTION in ("motion", "hybrid") else None   # #089: constant velocity
        M0, M1, n = match_motion(prev, cur, self.period, self.rng, self.bf, self.model, self.subpixel,
                                 self.stuck_min, self.floor_only, self.elev, self.range_)
        if M1 is None and GUIDED_WINDOW is not None:          # #088: guided matching failed (fast rotation) -> brute force for this scan
            global GUIDED_WINDOW_SAVED
            saved, GUIDED_WINDOW_SAVED = GUIDED_WINDOW, None
            globals()["GUIDED_WINDOW"] = None
            try:
                M0, M1, n = match_motion(prev, cur, self.period, self.rng, self.bf, self.model, self.subpixel,
                                         self.stuck_min, self.floor_only, self.elev, self.range_)
            finally:
                globals()["GUIDED_WINDOW"] = saved
            self.n_guided_retries = getattr(self, "n_guided_retries", 0) + 1
        if TRANS_MODE == "auto" and TRANS_MIN_RANGE is not None:
            f = (np.clip(np.median(self.ratios[-TRANS_AUTO_WINDOW:]), *TRANS_CLIP)
                 if len(self.ratios) >= TRANS_AUTO_MIN else 1.0)      # causal: only the scans before this one
            for M in (M0, M1):
                if M is not None:
                    M[:3, 3] = f * M[:3, 3]
            r = match_motion.last_ratio
            if r is not None and np.isfinite(r):
                self.ratios.append(r)
            self.last_factor = f
        self.last_params = match_motion.last_params if M1 is not None else None
        self.last_t_start = cur[5]
        self._fuse_now = True
        if self.fuse_range == "weak" and M1 is not None:                         # #152: fuse only when the intensity match count is low
            hist = self._inl_hist                                                 # for THIS sequence: < half the median of the last 100
            self._fuse_now = len(hist) >= 20 and n < 0.5 * float(np.median(hist[-100:]))
            hist.append(n)
            self.n_fuse_weak = getattr(self, "n_fuse_weak", 0) + int(self._fuse_now)
        if M1 is not None and (self.multi_baseline or self.fuse_range):          # #090
            M1, n = self._joint(M1, n, prev, cur, xyz, ts, ring)
        self.prev2 = prev
        M, n = self._gate(M1, n, prev, cur)
        self.last_motion = M if M is not None else None                   # #089: the next scan's prediction (None: shift mode)
        if self.range_motion is not None:
            M, n = self._range(xyz, ts, ring, M, n)
        return M, n

    def _joint(self, M1, n, prev, cur, xyz, ts, ring):
        """#090: refit the motion with the inlier pairs of k-1 <-> k plus those of k-2 <-> k (multi_baseline) and / or of the range
        panoramas k-1 <-> k (fuse_range), all on the same motion curve.  The extra matching must not disturb what the next scan
        inherits (the guided shift / prediction, the stored parameters), so those are saved and restored."""
        global GUIDED_SHIFT, GUIDED_PRED_MOTION, GUIDED_PREDICTION
        base = match_motion.last_pairs
        if base is None:
            return M1, n
        sets, saved = [base], (GUIDED_SHIFT, GUIDED_PRED_MOTION, GUIDED_PREDICTION)
        try:
            if self.multi_baseline and self.prev2 is not None:
                GUIDED_PREDICTION, GUIDED_PRED_MOTION = "motion", M1 @ M1          # predicted motion over two scans
                _, M2, _ = match_motion(self.prev2, cur, self.period, self.rng, self.bf, self.model, self.subpixel,
                                        self.stuck_min, self.floor_only, self.elev, self.range_)
                if M2 is not None and match_motion.last_pairs is not None:
                    sets.append(match_motion.last_pairs)
            if self.fuse_range:
                if not hasattr(self, "range_detector"):
                    self.range_detector = make_detector("surf", 10.0); self.range_rng = np.random.default_rng(1000)
                cur_r = range_features(xyz, ts, ring, self.range_detector)
                prev_r, self.prev_range_feat = self.prev_range_feat, cur_r
                if prev_r is not None and getattr(self, "_fuse_now", True):   # #152: "weak" = only on low-match scans (features cached every scan)
                    GUIDED_PREDICTION, GUIDED_PRED_MOTION = "motion", M1
                    _, Mr, _ = match_motion(prev_r, cur_r, self.period, self.range_rng, self.range_bf, self.model, self.subpixel,
                                            self.stuck_min, self.floor_only, self.elev, self.range_)
                    if Mr is not None and match_motion.last_pairs is not None:
                        sets.append(match_motion.last_pairs)
        finally:
            GUIDED_SHIFT, GUIDED_PRED_MOTION, GUIDED_PREDICTION = saved
        if len(sets) == 1:
            return M1, n
        p, tp, q, tq = (np.concatenate([st[i] for st in sets]) for i in range(4))
        Mj, keep = fit_time(p, tp, q, tq, M1, self.model)
        if Mj is None:
            return M1, n
        if self.multi_baseline == "translation":             # #131: rotation of the two-scan fit (better per scan), translation of the joint fit
            Mj = Mj.copy(); Mj[:3, :3] = M1[:3, :3]
        self.n_joint += 1
        self.joint_pairs.append([len(st[0]) for st in sets])
        self.last_params = fit_time.last_params
        return Mj, int(keep.sum())

    def _range(self, xyz, ts, ring, M, n):
        """The motion from the range panorama (#058), after the intensity estimate and its gate.

        "fallback" is lazy (branch fast_fallback): only the previous raw scan is kept, and both range panoramas, their
        matches and the fit are computed only when the intensity motion failed (M is None).  The eager version built and
        matched the range panorama on every scan and discarded it whenever the intensity motion succeeded: up to 1.9x the
        run time on sequences that never needed it.  The range RANSAC now draws only on failed scans, so its motions
        differ from the eager version as another seed would.  "candidate" needs the range motion on every scan: eager."""
        if self.range_motion == "fallback":
            prev_raw, self.prev_raw = self.prev_raw, (xyz, ts, ring)
            self.last_range_motion = None
            if M is not None or prev_raw is None:
                return M, n
            prev_r = range_features(*prev_raw, self.range_detector)
            cur_r = range_features(xyz, ts, ring, self.range_detector)
            _, Mr, nr = match_motion(prev_r, cur_r, self.period, self.range_rng, self.range_bf, self.model, self.subpixel,
                                     self.stuck_min, self.floor_only, self.elev, self.range_)
            self.last_range_motion = Mr
            if Mr is not None:
                self.n_range_used += 1
                return Mr, nr
            return M, n
        cur_r = range_features(xyz, ts, ring, self.range_detector)
        prev_r, self.prev_range = self.prev_range, cur_r
        self.last_range_motion = None
        if prev_r is not None:
            _, Mr, nr = match_motion(prev_r, cur_r, self.period, self.range_rng, self.range_bf, self.model, self.subpixel,
                                     self.stuck_min, self.floor_only, self.elev, self.range_)
            self.last_range_motion = Mr
            if self.range_motion in ("fallback", "validate") and M is None and Mr is not None:   # #109 validate: eager + fallback
                self.n_range_used += 1
                return Mr, nr
        return M, n

    def _gate(self, M, n, prev, cur):
        """The plausibility gate (#054): None + self.last_reason for a rejected motion; saves the pair if asked."""
        min_n, max_rot, max_change = self.gate
        reason, rot_deg = None, float("nan")
        if M is None:
            reason = "failed"
        else:
            rv = Rotation.from_matrix(M[:3, :3]).as_rotvec()
            rot_deg = float(np.degrees(np.linalg.norm(rv)))
            if min_n is not None and n < min_n:
                reason = f"matches {n} < {min_n}"
            elif max_rot is not None and rot_deg > max_rot:
                reason = f"rotation {rot_deg:.1f} > {max_rot:g} deg"
            elif (max_change is not None and self.last_accepted_rotvec is not None
                  and np.degrees(np.linalg.norm(rv - self.last_accepted_rotvec)) > max_change):
                reason = (f"rotation change {np.degrees(np.linalg.norm(rv - self.last_accepted_rotvec)):.1f} "
                          f"> {max_change:g} deg")
            else:
                self.last_accepted_rotvec = rv
        self.last_reason = reason
        if reason is not None and self.save_rejected_dir is not None:
            self._save_pair(prev, cur, n, rot_deg, reason)
        return (None, n) if reason is not None else (M, n)

    def _save_pair(self, prev, cur, n, rot_deg, reason):
        import csv
        import os
        os.makedirs(self.save_rejected_dir, exist_ok=True)
        stem = os.path.join(self.save_rejected_dir, f"scan{self.k:05d}")
        cv2.imwrite(stem + "_a_previous.png", prev[6])
        cv2.imwrite(stem + "_b_current.png", cur[6])
        good = sorted(match_motion.last_good, key=lambda m: m.distance)[:300]
        img = cv2.drawMatches(prev[6], prev[3], cur[6], cur[3], good, None,
                              flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imwrite(stem + "_c_matches.png", img)
        new = not os.path.exists(os.path.join(self.save_rejected_dir, "rejected.csv"))
        with open(os.path.join(self.save_rejected_dir, "rejected.csv"), "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["scan", "t_start", "reason", "matches_after_checks", "ratio_test_matches",
                            "keypoints_previous", "keypoints_current", "rotation_deg"])
            w.writerow([self.k, f"{cur[5]:.6f}", reason, n, len(match_motion.last_good), len(prev[3]), len(cur[3]),
                        f"{rot_deg:.3f}"])
