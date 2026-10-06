// C++ parts of the image motion (#182, speed of the image worker on 128-beam sensors):
//   ransac_batch  - Kabsch of K 3-point hypotheses + metric inlier test; returns the first best hypothesis (as numpy's argmax) and its mask.
//                   The random draws and the stopping rule stay in Python (same RNG stream).  Kabsch via Eigen's 3x3 SVD instead of LAPACK:
//                   the same rotations to ~1e-16, so an inlier can flip only exactly at the threshold.
//   fit_soft_l1   - the continuous-time fit of `fit_time` (residual / residual_jac, models cv / car / ca): minimises the same cost as
//                   scipy.optimize.least_squares(loss="soft_l1", f_scale) - 0.5 f^2 sum rho((r/f)^2), rho(z) = 2(sqrt(1+z) - 1), one residual
//                   per coordinate - by Levenberg-Marquardt on the IRLS normal equations, to a tighter tolerance than scipy's default (1e-8).
//   grid_scatter  - the assignments of `grid` (img / P / T / valid at [row, col], later points overwrite earlier ones, as numpy does).
//
// Build: scripts/build_image_fit.sh  ->  kiss_slam/_image_fit<EXT_SUFFIX>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

namespace py = pybind11;
using darray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using larray = py::array_t<int64_t, py::array::c_style | py::array::forcecast>;
using M3 = Eigen::Matrix3d;
using V3 = Eigen::Vector3d;

// ---------------------------------------------------------------------------------------------------------------- RANSAC
static void kabsch(const V3 *a, const V3 *b, int n, M3 &R, V3 &t) {
    V3 ca = V3::Zero(), cb = V3::Zero();
    for (int i = 0; i < n; ++i) { ca += a[i]; cb += b[i]; }
    ca /= n; cb /= n;
    M3 H = M3::Zero();
    for (int i = 0; i < n; ++i) H += (b[i] - cb) * (a[i] - ca).transpose();     // (B - cb)^T (A - ca), as numpy
    Eigen::JacobiSVD<M3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const M3 U = svd.matrixU(), V = svd.matrixV();
    M3 D = M3::Identity();
    D(2, 2) = (V * U.transpose()).determinant() >= 0 ? 1.0 : -1.0;
    R = V * D * U.transpose();
    t = ca - R * cb;
}

py::tuple ransac_batch(darray A, darray B, larray idx, double thr) {
    const ssize_t n = A.shape(0), K = idx.shape(0);
    const V3 *a = reinterpret_cast<const V3 *>(A.data()), *b = reinterpret_cast<const V3 *>(B.data());
    const int64_t *I = idx.data();
    std::vector<int> count(K);
    const double thr2 = thr * thr;
    for (ssize_t k = 0; k < K; ++k) {
        V3 pa[3], pb[3];
        for (int s = 0; s < 3; ++s) { pa[s] = a[I[3 * k + s]]; pb[s] = b[I[3 * k + s]]; }
        M3 R; V3 t; kabsch(pa, pb, 3, R, t);
        int c = 0;
        for (ssize_t i = 0; i < n; ++i) c += (a[i] - (R * b[i] + t)).squaredNorm() < thr2;
        count[k] = c;
    }
    ssize_t j = 0;
    for (ssize_t k = 1; k < K; ++k) if (count[k] > count[j]) j = k;           // first of the best
    V3 pa[3], pb[3];
    for (int s = 0; s < 3; ++s) { pa[s] = a[I[3 * j + s]]; pb[s] = b[I[3 * j + s]]; }
    M3 R; V3 t; kabsch(pa, pb, 3, R, t);
    py::array_t<bool> mask(n);
    bool *m = mask.mutable_data();
    for (ssize_t i = 0; i < n; ++i) m[i] = (a[i] - (R * b[i] + t)).squaredNorm() < thr2;
    return py::make_tuple(j, count[j], mask);
}

// ---------------------------------------------------------------------------------------------------------------- time fit
static M3 skew(const V3 &v) {
    M3 S; S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
}

static M3 expmap(const V3 &rv) {                       // _rotmats: Rodrigues, series below 1e-4 rad
    const double th = rv.norm();
    double a, b;
    if (th < 1e-4) { a = 1 - th * th / 6; b = 0.5 - th * th / 24; }
    else { a = std::sin(th) / th; b = (1 - std::cos(th)) / (th * th); }
    const M3 K = skew(rv);
    return M3::Identity() + a * K + b * K * K;
}

static M3 d_rotate(const V3 &phi, const V3 &y) {      // _d_rotate: -[y]x J_l(phi)
    const double th = phi.norm();
    double a, b;
    if (th < 1e-4) { a = 0.5 - th * th / 24; b = 1.0 / 6 - th * th / 120; }
    else { a = (1 - std::cos(th)) / (th * th); b = (th - std::sin(th)) / (th * th * th); }
    const M3 K = skew(phi);
    return -skew(y) * (M3::Identity() + a * K + b * K * K);
}

struct Fit {
    const V3 *p, *q; const double *tp, *tq; ssize_t n; int dim; double f;
    void pose(const Eigen::VectorXd &x, double t, V3 &rot, V3 &tr) const {
        rot = t * x.segment<3>(0); tr = t * x.segment<3>(3);
        if (dim >= 9) rot += 0.5 * t * t * x.segment<3>(6);
        if (dim == 12) tr += 0.5 * t * t * x.segment<3>(9);
    }
    // residual of point i (3) and, if J != nullptr, its 3 x dim Jacobian
    V3 res(const Eigen::VectorXd &x, ssize_t i, Eigen::MatrixXd *J) const {
        V3 r = V3::Zero();
        if (J) J->setZero(3, dim);
        for (int s = 0; s < 2; ++s) {
            const V3 &pt = s == 0 ? p[i] : q[i];
            const double t = s == 0 ? tp[i] : tq[i], sign = s == 0 ? 1.0 : -1.0;
            V3 rot, tr; pose(x, t, rot, tr);
            const V3 y = expmap(rot) * pt;
            r += sign * (y + tr);
            if (J) {
                const M3 D = sign * d_rotate(rot, y);
                J->block<3, 3>(0, 0) += D * t;
                J->block<3, 3>(0, 3) += sign * t * M3::Identity();
                if (dim >= 9) J->block<3, 3>(0, 6) += D * (0.5 * t * t);
                if (dim == 12) J->block<3, 3>(0, 9) += sign * (0.5 * t * t) * M3::Identity();
            }
        }
        return r;
    }
    double cost(const Eigen::VectorXd &x) const {      // 0.5 f^2 sum rho((r/f)^2), as scipy
        double c = 0;
        for (ssize_t i = 0; i < n; ++i) {
            const V3 r = res(x, i, nullptr);
            for (int k = 0; k < 3; ++k) c += 2.0 * (std::sqrt(1.0 + r(k) * r(k) / (f * f)) - 1.0);
        }
        return 0.5 * f * f * c;
    }
};

darray fit_soft_l1(darray x0, darray P, darray TP, darray Q, darray TQ, double f_scale) {
    Fit F{reinterpret_cast<const V3 *>(P.data()), reinterpret_cast<const V3 *>(Q.data()), TP.data(), TQ.data(), P.shape(0),
          static_cast<int>(x0.shape(0)), f_scale};
    Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(x0.data(), F.dim);
    double c = F.cost(x), lambda = 1e-3;
    Eigen::MatrixXd J(3, F.dim);
    for (int it = 0; it < 200; ++it) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(F.dim, F.dim);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(F.dim);
        for (ssize_t i = 0; i < F.n; ++i) {
            const V3 r = F.res(x, i, &J);
            for (int k = 0; k < 3; ++k) {
                const double w = 1.0 / std::sqrt(1.0 + r(k) * r(k) / (f_scale * f_scale));   // rho'(z)
                const auto Jk = J.row(k);
                H.noalias() += w * Jk.transpose() * Jk;
                g.noalias() += w * r(k) * Jk.transpose();
            }
        }
        if (g.lpNorm<Eigen::Infinity>() < 1e-14) break;
        bool accepted = false;
        for (int tries = 0; tries < 30 && !accepted; ++tries) {
            Eigen::MatrixXd A = H;
            A.diagonal() += lambda * H.diagonal().cwiseMax(1e-12);
            const Eigen::VectorXd dx = A.ldlt().solve(-g);
            const Eigen::VectorXd xn = x + dx;
            const double cn = F.cost(xn);
            if (cn <= c) {
                const double dc = c - cn, step = dx.norm();
                x = xn; c = cn; lambda = std::max(lambda / 3, 1e-12); accepted = true;
                if (dc <= 1e-15 * std::max(c, 1e-300) || step <= 1e-13 * (x.norm() + 1e-13)) it = 1 << 30;   // converged
            } else {
                lambda *= 4;
            }
        }
        if (!accepted) break;
    }
    darray out(F.dim);
    std::copy(x.data(), x.data() + F.dim, out.mutable_data());
    return out;
}

// ---------------------------------------------------------------------------------------------------------------- panorama
py::tuple grid_scatter(larray row, larray col, darray inten, darray xyz, darray ts, int64_t H, int64_t W) {
    const ssize_t n = row.shape(0);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    darray img({H, W}), P({H, W, int64_t(3)}), T({H, W});
    py::array_t<bool> valid({H, W});
    double *im = img.mutable_data(), *pp = P.mutable_data(), *tt = T.mutable_data();
    bool *v = valid.mutable_data();
    std::fill(im, im + H * W, nan); std::fill(pp, pp + H * W * 3, 0.0); std::fill(tt, tt + H * W, 0.0); std::fill(v, v + H * W, false);
    const int64_t *r = row.data(), *c = col.data();
    const double *in = inten.data(), *X = xyz.data(), *ts_ = ts.data();
    for (ssize_t i = 0; i < n; ++i) {
        const int64_t k = r[i] * W + c[i];
        im[k] = in[i]; tt[k] = ts_[i]; v[k] = true;
        pp[3 * k] = X[3 * i]; pp[3 * k + 1] = X[3 * i + 1]; pp[3 * k + 2] = X[3 * i + 2];
    }
    return py::make_tuple(img, P, T, valid);
}

PYBIND11_MODULE(_image_fit, m) {
    m.def("ransac_batch", &ransac_batch, "first best of K Kabsch-3 hypotheses: (index, count, inlier mask)");
    m.def("fit_soft_l1", &fit_soft_l1, "continuous-time fit with the soft_l1 loss (the cost of least_squares)");
    m.def("grid_scatter", &grid_scatter, "panorama grid assignments");
}
