"""Deskew inside the registration (#115, open_tasks B.9; the LiDAR-only continuous-time idea of ECTLO / CT-ICP on KISS's map and kernel).

KISS deskews the scan ONCE with a motion guessed before the registration (constant velocity; ours: the image motion) and then aligns it
rigidly.  Here the scan is registered with TWO poses, the sensor at the start (T_b) and at the end (T_e) of the sweep, and every point is
placed at its own time between them:

    R(s) = slerp(R_b, R_e, s),  t(s) = (1 - s) t_b + s t_e,   x_i = R(s_i) p_i + t(s_i),   s_i in [0, 1] (time within the sweep)

so the deskew is corrected by the geometry at every Gauss-Newton iteration.  Cost: KISS's own - point-to-point to the nearest map point
within 3 sigma, Geman-McClure weight kernel^2 / (kernel + r^2)^2 with kernel = sigma - plus ECTLO's two soft constraints (lambda per point,
as ECTLO's mean registration cost): the start continues the previous end (location), and the motion over the sweep continues the previous
one (velocity).  Initial value: T_b = the previous end pose, T_e = T_b . M with M the image motion (or constant velocity).  Jacobians: the
first-order interpolation of CT-ICP, d x_i / d(dtheta_b, dt_b) = (1 - s_i) [-[R(s_i) p_i]x, I], d x_i / d(dtheta_e, dt_e) = s_i [...]
(left perturbations in the world frame).  Prototype: nearest neighbours by a KD-tree on the local map's points (KISS's C++ map exposes no
neighbour search to Python) - speed is not the point here.
"""
import os

import numpy as np
from scipy.spatial import cKDTree

try:                                                                   # #158: C++ inner loop (scripts/build_ct_accumulate.sh)
    from kiss_slam import _ct_accumulate as _CT
except ImportError:                                                    # pragma: no cover - the Python version below
    _CT = None
USE_CPP = _CT is not None and os.environ.get("KISS_CT_PYTHON") != "1"   # KISS_CT_PYTHON=1 forces the Python version (checks)
from scipy.spatial.transform import Rotation, Slerp


def voxel_down_sample(points, s, voxel):
    """First point of each voxel, keeping its time (as kiss_icp's voxel_down_sample keeps one point per voxel)."""
    keys = np.floor(points / voxel).astype(np.int64)
    keys -= keys.min(axis=0)                                   # #156: one int64 key per voxel - np.unique(axis=0) sorted 3 columns (slow)
    span = keys.max(axis=0) + 1
    flat = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]
    _, first = np.unique(flat, return_index=True)              # first occurrence of each voxel, as before
    first = np.sort(first)
    return points[first], s[first]


def _skew(v):
    S = np.zeros(v.shape[:-1] + (3, 3))
    S[..., 0, 1], S[..., 0, 2], S[..., 1, 2] = -v[..., 2], v[..., 1], -v[..., 0]
    return S - np.swapaxes(S, -1, -2)


def place(points, s, Tb, Te):
    """World positions of points at their times between Tb and Te."""
    # #156: slerp in closed form, R(s) = R_b exp(s log(R_b^T R_e)) (what scipy's Slerp computes), vectorised with Rodrigues - scipy Rotation
    # objects per call were ~20 % of the run time.  Rotating R_b^T-free: R(s) p = R_b (exp(s w) p).
    w = _rotvec(Tb[:3, :3].T @ Te[:3, :3])
    if USE_CPP:                                                        # #159
        return _CT.place(np.ascontiguousarray(points, dtype=np.float64), np.ascontiguousarray(s, dtype=np.float64),
                         np.ascontiguousarray(Tb[:3, :3]), w, Tb[:3, 3].copy(), Te[:3, 3].copy())
    s = np.clip(s, 0.0, 1.0)
    rp = _rodrigues_apply(s[:, None] * w, points) @ Tb[:3, :3].T
    t = (1.0 - s)[:, None] * Tb[:3, 3] + s[:, None] * Te[:3, 3]
    return rp + t


def _rodrigues_apply(rv, p):
    """exp(rv_i) p_i for N rotation vectors (Rodrigues; series below 1e-8 rad)."""
    th = np.linalg.norm(rv, axis=1)
    small = th < 1e-8
    t = np.where(small, 1.0, th)
    k = rv / t[:, None]
    c, sn = np.cos(th), np.sin(th)
    kxp = np.cross(k, p)
    kdp = np.einsum("ij,ij->i", k, p)
    out = p * c[:, None] + kxp * sn[:, None] + k * (kdp * (1.0 - c))[:, None]
    return np.where(small[:, None], p + np.cross(rv, p), out)


def _rotvec(Rm):
    """Rotation vector of a rotation matrix (#159: scipy's Rotation objects were ~7 % of the run time); antipodal-safe like scipy."""
    c = np.clip((np.trace(Rm) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    if th < 1e-6:
        return 0.5 * v
    if np.pi - th < 1e-4:                                              # near pi: fall back to scipy (rare)
        return Rotation.from_matrix(Rm).as_rotvec()
    return v * (th / (2.0 * np.sin(th)))


def _expm(rv):
    """Rotation matrix of a rotation vector (Rodrigues; series below 1e-8 rad)."""
    th = float(np.linalg.norm(rv))
    K = np.array([[0.0, -rv[2], rv[1]], [rv[2], 0.0, -rv[0]], [-rv[1], rv[0], 0.0]])
    if th < 1e-8:
        return np.eye(3) + K
    return np.eye(3) + (np.sin(th) / th) * K + ((1.0 - np.cos(th)) / th ** 2) * (K @ K)


def _left(T, d):
    """Exp(d) . T for d = (dtheta, dt): rotation applied on the left, translation added (world-frame perturbation)."""
    out = T.copy()
    out[:3, :3] = _expm(d[:3]) @ T[:3, :3]                             # #159: Rodrigues instead of a scipy Rotation object
    out[:3, 3] = T[:3, 3] + d[3:]
    return out


def motion(Tb, Te):
    """(rotation vector, translation) of the sweep in the start frame."""
    Rr = Tb[:3, :3].T @ Te[:3, :3]
    return Rotation.from_matrix(Rr).as_rotvec(), Tb[:3, :3].T @ (Te[:3, 3] - Tb[:3, 3])


def ct_register(source, s_src, map_points, T_prev_end, T_init_end, prev_motion, sigma, lam_loc=0.1, lam_vel=0.1,
                max_iter=30, tol=1e-4, tree=None, img=None, img_weight=1.0, img_kernel=None, voxel=None):
    """(T_b, T_e) of the sweep.  source: points in the sensor frame as measured (NOT deskewed), s_src their times in [0, 1].
    img (#115 joint): (world positions of the previous scan's matched points, placed by its own solved sweep - fixed; the current scan's
    matched points q, raw; their times in [0, 1]) - each match adds the residual x(q) - world(p), weighted img_weight x a geometric point,
    Geman-McClure with img_kernel (default sigma)."""
    Tb, Te = T_prev_end.copy(), T_init_end.copy()
    if len(map_points) == 0 or len(source) < 10:
        return Tb, Te
    bound = 3.0 * sigma if voxel is None else min(3.0 * sigma, 2.0 * np.sqrt(3.0) * voxel)
    if USE_CPP:                                                        # #158: grid with cell = bound -> the same nearest neighbour as the tree
        grid = _CT.Grid(np.ascontiguousarray(map_points, dtype=np.float64), float(0.5 * voxel if voxel else bound))   # #158b: small cells
        src_c = np.ascontiguousarray(source, dtype=np.float64); s_c = np.ascontiguousarray(s_src, dtype=np.float64)
        img_c = None
        if img is not None and len(img[0]) >= 3:
            img_c = tuple(np.ascontiguousarray(a, dtype=np.float64) for a in img)    # (targets, points, times)
    else:
        tree = tree if tree is not None else cKDTree(map_points)
    w_prev, v_prev = prev_motion
    for _ in range(max_iter):
        if USE_CPP:
            w_be = _rotvec(Tb[:3, :3].T @ Te[:3, :3])
            H, g, n = _CT.accumulate(grid, src_c, s_c, np.ascontiguousarray(Tb[:3, :3]), w_be, Tb[:3, 3].copy(), Te[:3, 3].copy(),
                                     float(sigma), float(bound))
            if n < 10:
                break
            H, g = np.array(H), np.array(g)
            if img_c is not None:                                       # image matches as residuals (joint), C++ (#159)
                _CT.accumulate_targets(H, g, img_c[1], img_c[2], img_c[0], np.ascontiguousarray(Tb[:3, :3]), w_be, Tb[:3, 3].copy(),
                                       Te[:3, 3].copy(), float(img_kernel if img_kernel is not None else sigma), float(img_weight))
            Tb, Te, done = _constrained_step(H, g, n, Tb, Te, T_prev_end, w_prev, v_prev, lam_loc, lam_vel, tol)
            if done:
                break
            continue
        x = place(source, s_src, Tb, Te)
        # KISS searches the closest neighbour only in the 27 voxels around the point (<= 2 sqrt(3) voxel sizes), THEN applies 3 sigma
        bound = 3.0 * sigma if voxel is None else min(3.0 * sigma, 2.0 * np.sqrt(3.0) * voxel)
        dist, idx = tree.query(x, distance_upper_bound=bound, workers=1)   # #156: workers=-1 started Python threads per call (~22 % of run time)
        ok = np.isfinite(dist)
        n = int(ok.sum())
        if n < 10:
            break
        r = x[ok] - map_points[idx[ok]]
        w = sigma ** 2 / (sigma + (r ** 2).sum(1)) ** 2
        s = s_src[ok]
        Rp = x[ok] - ((1.0 - s)[:, None] * Tb[:3, 3] + s[:, None] * Te[:3, 3])     # R(s) p, the rotated point
        Jr = -_skew(Rp)                                                              # d x / d dtheta (full weight)
        J = np.zeros((n, 3, 12))
        J[:, :, 0:3] = (1.0 - s)[:, None, None] * Jr
        J[:, :, 3:6] = (1.0 - s)[:, None, None] * np.eye(3)
        J[:, :, 6:9] = s[:, None, None] * Jr
        J[:, :, 9:12] = s[:, None, None] * np.eye(3)
        if img is not None and len(img[0]) >= 3:                    # image matches as residuals (joint)
            pw, q_img, s_img = img
            xq = place(q_img, s_img, Tb, Te)
            ri = xq - pw
            ki = img_kernel if img_kernel is not None else sigma
            wi = img_weight * ki ** 2 / (ki + (ri ** 2).sum(1)) ** 2
            Rq = xq - ((1.0 - s_img)[:, None] * Tb[:3, 3] + s_img[:, None] * Te[:3, 3])
            Ji = np.zeros((len(ri), 3, 12)); Jri = -_skew(Rq)
            Ji[:, :, 0:3] = (1.0 - s_img)[:, None, None] * Jri; Ji[:, :, 3:6] = (1.0 - s_img)[:, None, None] * np.eye(3)
            Ji[:, :, 6:9] = s_img[:, None, None] * Jri; Ji[:, :, 9:12] = s_img[:, None, None] * np.eye(3)
            r = np.concatenate([r, ri]); w = np.concatenate([w, wi]); J = np.concatenate([J, Ji])
        Jw = J * w[:, None, None]
        H = np.einsum("nki,nkj->ij", Jw, J)
        g = np.einsum("nki,nk->i", Jw, r)
        # soft constraints, weighted per point as ECTLO's mean registration cost: location (start = previous end), velocity
        lam_l, lam_v = lam_loc * n, lam_vel * n
        r_loc = np.concatenate([Rotation.from_matrix(Tb[:3, :3] @ T_prev_end[:3, :3].T).as_rotvec(), Tb[:3, 3] - T_prev_end[:3, 3]])
        H[0:6, 0:6] += lam_l * np.eye(6)
        g[0:6] += lam_l * r_loc
        wm, vm = motion(Tb, Te)
        r_vel = np.concatenate([wm - w_prev, vm - v_prev])
        Jv = np.zeros((6, 12))
        Rb_T = Tb[:3, :3].T
        Jv[0:3, 0:3], Jv[0:3, 6:9] = -Rb_T, Rb_T                 # first order: motion rotation ~ R_b^T (theta_e - theta_b)
        Jv[3:6, 3:6], Jv[3:6, 9:12] = -Rb_T, Rb_T
        H += lam_v * Jv.T @ Jv
        g += lam_v * Jv.T @ r_vel
        d = -np.linalg.solve(H + 1e-9 * np.eye(12), g)
        Tb, Te = _left(Tb, d[:6]), _left(Te, d[6:])
        if np.linalg.norm(d) < tol:
            break
    return Tb, Te


def _constrained_step(H, g, n, Tb, Te, T_prev_end, w_prev, v_prev, lam_loc, lam_vel, tol):
    """ECTLO's location / velocity constraints added to (H, g), one Gauss-Newton update - the same algebra as the Python loop (#158)."""
    lam_l, lam_v = lam_loc * n, lam_vel * n
    r_loc = np.concatenate([_rotvec(Tb[:3, :3] @ T_prev_end[:3, :3].T), Tb[:3, 3] - T_prev_end[:3, 3]])
    H[0:6, 0:6] += lam_l * np.eye(6)
    g[0:6] += lam_l * r_loc
    Rb_T = Tb[:3, :3].T
    wm, vm = _rotvec(Rb_T @ Te[:3, :3]), Rb_T @ (Te[:3, 3] - Tb[:3, 3])          # = motion(Tb, Te)
    r_vel = np.concatenate([wm - w_prev, vm - v_prev])
    Jv = np.zeros((6, 12))
    Jv[0:3, 0:3], Jv[0:3, 6:9] = -Rb_T, Rb_T
    Jv[3:6, 3:6], Jv[3:6, 9:12] = -Rb_T, Rb_T
    H += lam_v * Jv.T @ Jv
    g += lam_v * Jv.T @ r_vel
    d = -np.linalg.solve(H + 1e-9 * np.eye(12), g)
    return _left(Tb, d[:6]), _left(Te, d[6:]), bool(np.linalg.norm(d) < tol)


def deskew_to_end(points, s, Tb, Te):
    """Points at their times, expressed in the frame at the END of the sweep (KISS's deskewed-scan convention)."""
    x = place(points, s, Tb, Te)
    Ti = np.linalg.inv(Te)
    return x @ Ti[:3, :3].T + Ti[:3, 3]
