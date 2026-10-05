// B.9 continuous-time registration (#158): the per-iteration inner loop of kiss_slam/ct_registration.ct_register in C++.
// For every source point at its time s in [0, 1]: place it between the start and end pose of the sweep (R(s) = R_b exp(s w),
// t(s) = (1 - s) t_b + s t_e), find the nearest map point within `bound` (a hash grid whose cell IS the bound, so the 3 x 3 x 3 cells
// around the point contain every map point closer than the bound - the same neighbour as the KD-tree query of the Python version),
// and add the Geman-McClure-weighted point-to-point residual to the 12 x 12 Gauss-Newton system (first-order interpolation Jacobians,
// left perturbations of both poses, as in the Python version).  Single-threaded: deterministic summation order.
//
// Build: scripts/build_ct_accumulate.sh  ->  kiss_slam/_ct_accumulate<EXT_SUFFIX>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace py = pybind11;
using darray = py::array_t<double, py::array::c_style | py::array::forcecast>;

struct Grid {
    double cell;
    std::vector<double> pts;                                        // x, y, z of the map points
    std::unordered_map<int64_t, std::vector<int32_t>> cells;

    static int64_t key(int64_t i, int64_t j, int64_t k) {           // 21 bits per axis, offset so negatives fit
        const int64_t o = 1 << 20;
        return ((i + o) << 42) | ((j + o) << 21) | (k + o);
    }
    Grid(darray points, double cell_) : cell(cell_) {
        const ssize_t n = points.shape(0);
        const double *P = points.data();
        pts.assign(P, P + 3 * n);
        cells.reserve(static_cast<size_t>(n));
        for (ssize_t i = 0; i < n; ++i) {
            auto c = key(static_cast<int64_t>(std::floor(P[3 * i] / cell)), static_cast<int64_t>(std::floor(P[3 * i + 1] / cell)),
                         static_cast<int64_t>(std::floor(P[3 * i + 2] / cell)));
            cells[c].push_back(static_cast<int32_t>(i));
        }
    }
    // Nearest map point strictly closer than `bound` (<= cell); -1 if none.
    int32_t nearest(const double *x, double bound, double &d2best) const {
        const int64_t ci = static_cast<int64_t>(std::floor(x[0] / cell)), cj = static_cast<int64_t>(std::floor(x[1] / cell)),
                      ck = static_cast<int64_t>(std::floor(x[2] / cell));
        int32_t best = -1;
        d2best = bound * bound;
        for (int64_t a = -1; a <= 1; ++a)
            for (int64_t b = -1; b <= 1; ++b)
                for (int64_t c = -1; c <= 1; ++c) {
                    auto it = cells.find(key(ci + a, cj + b, ck + c));
                    if (it == cells.end()) continue;
                    for (int32_t idx : it->second) {
                        const double *q = &pts[3 * static_cast<size_t>(idx)];
                        const double dx = x[0] - q[0], dy = x[1] - q[1], dz = x[2] - q[2];
                        const double d2 = dx * dx + dy * dy + dz * dz;
                        if (d2 < d2best) { d2best = d2; best = idx; }
                    }
                }
        return best;
    }
};

static inline void rodrigues_apply(const double *rv, const double *p, double *out) {
    const double th = std::sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
    if (th < 1e-8) {                                                 // p + rv x p, as the Python version
        out[0] = p[0] + rv[1] * p[2] - rv[2] * p[1];
        out[1] = p[1] + rv[2] * p[0] - rv[0] * p[2];
        out[2] = p[2] + rv[0] * p[1] - rv[1] * p[0];
        return;
    }
    const double k[3] = {rv[0] / th, rv[1] / th, rv[2] / th};
    const double c = std::cos(th), sn = std::sin(th);
    const double kxp[3] = {k[1] * p[2] - k[2] * p[1], k[2] * p[0] - k[0] * p[2], k[0] * p[1] - k[1] * p[0]};
    const double kdp = k[0] * p[0] + k[1] * p[1] + k[2] * p[2];
    for (int i = 0; i < 3; ++i) out[i] = p[i] * c + kxp[i] * sn + k[i] * kdp * (1.0 - c);
}

// (H 12x12, g 12, n used): Rb row-major 3x3, w = rotvec(Rb^T Re), tb, te.
py::tuple accumulate(const Grid &grid, darray points, darray s_arr, darray Rb_arr, darray w_arr, darray tb_arr, darray te_arr,
                     double sigma, double bound) {
    const ssize_t n = points.shape(0);
    const double *P = points.data(), *S = s_arr.data(), *Rb = Rb_arr.data(), *w = w_arr.data(), *tb = tb_arr.data(), *te = te_arr.data();
    py::array_t<double> H_out({12, 12}), g_out(12);
    double *H = H_out.mutable_data(), *g = g_out.mutable_data();
    std::fill(H, H + 144, 0.0);
    std::fill(g, g + 12, 0.0);
    int64_t used = 0;
    const double s2 = sigma * sigma;
    for (ssize_t i = 0; i < n; ++i) {
        const double s = std::min(std::max(S[i], 0.0), 1.0);
        const double rv[3] = {s * w[0], s * w[1], s * w[2]};
        double e[3];
        rodrigues_apply(rv, &P[3 * i], e);
        const double Rp[3] = {Rb[0] * e[0] + Rb[1] * e[1] + Rb[2] * e[2], Rb[3] * e[0] + Rb[4] * e[1] + Rb[5] * e[2],
                              Rb[6] * e[0] + Rb[7] * e[1] + Rb[8] * e[2]};    // R(s) p
        const double x[3] = {Rp[0] + (1.0 - s) * tb[0] + s * te[0], Rp[1] + (1.0 - s) * tb[1] + s * te[1], Rp[2] + (1.0 - s) * tb[2] + s * te[2]};
        double d2;
        const int32_t j = grid.nearest(x, bound, d2);
        if (j < 0) continue;
        ++used;
        const double *q = &grid.pts[3 * static_cast<size_t>(j)];
        const double r[3] = {x[0] - q[0], x[1] - q[1], x[2] - q[2]};
        const double den = sigma + d2;
        const double wt = s2 / (den * den);                                 // Geman-McClure, as kernel^2 / (kernel + r^2)^2
        // J (3 x 12): [(1-s) Jr, (1-s) I, s Jr, s I], Jr = -[Rp]x
        double J[3][12] = {};
        const double Jr[3][3] = {{0.0, Rp[2], -Rp[1]}, {-Rp[2], 0.0, Rp[0]}, {Rp[1], -Rp[0], 0.0}};
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) { J[a][b] = (1.0 - s) * Jr[a][b]; J[a][6 + b] = s * Jr[a][b]; }
            J[a][3 + a] = 1.0 - s;
            J[a][9 + a] = s;
        }
        for (int u = 0; u < 12; ++u) {
            const double ju0 = J[0][u], ju1 = J[1][u], ju2 = J[2][u];
            g[u] += wt * (ju0 * r[0] + ju1 * r[1] + ju2 * r[2]);
            for (int v = 0; v < 12; ++v) H[12 * u + v] += wt * (ju0 * J[0][v] + ju1 * J[1][v] + ju2 * J[2][v]);
        }
    }
    return py::make_tuple(H_out, g_out, used);
}

PYBIND11_MODULE(_ct_accumulate, m) {
    m.doc() = "B.9 continuous-time registration inner loop (#158)";
    py::class_<Grid>(m, "Grid").def(py::init<darray, double>(), py::arg("points"), py::arg("cell"));
    m.def("accumulate", &accumulate, py::arg("grid"), py::arg("points"), py::arg("s"), py::arg("Rb"), py::arg("w"), py::arg("tb"),
          py::arg("te"), py::arg("sigma"), py::arg("bound"));
}
