#include "cubic_spline_smoother.h"
#include "common/logger/logger.h"
namespace xju {
namespace pnc {

CubicSplineSmoother::CubicSplineSmoother() {
    _smooth = 0.;
}

void CubicSplineSmoother::cal_nodes_constraints(
    Eigen::MatrixXd& R,
    Eigen::MatrixXd& Q,
    Eigen::VectorXd& beta,
    Eigen::MatrixXd& H) {
    size_t n = _x.size() - 1;
    size_t dim_constraints = _constraints.size();
    Eigen::VectorXd dx = _x.bottomRows(n) - _x.topRows(n);
    Eigen::VectorXd reciprocal_dx = dx.cwiseInverse();
    R = Eigen::MatrixXd::Zero(n - 1 + dim_constraints, n + 1);
    for (size_t i = 0; i != n - 1; ++i) {
        R(i, i) = dx(i) / 6.0;
        R(i, i + 1) = (dx(i) + dx(i + 1)) / 3.0;
        R(i, i + 2) = dx(i + 1) / 6.0;
    }

    Q = Eigen::MatrixXd::Zero(n - 1 + dim_constraints, n + 1);
    for (size_t i = 0; i != n - 1; ++i) {
        Q(i, i) = reciprocal_dx(i);
        Q(i, i + 1) = -reciprocal_dx(i) - reciprocal_dx(i + 1);
        Q(i, i + 2) = reciprocal_dx(i + 1);
    }

    beta = Eigen::VectorXd::Zero(R.rows());

    // process terminal constraints
    size_t cur_row = n - 1;
    for (size_t i = 0; i < dim_constraints; ++i) {
        beta(cur_row) = _constraints[i];
        switch (i) {
        case 0:
            Q(cur_row, 0) = -1.0;
            break;
        case 1:
            R(cur_row, 0) = -dx(0) / 3.0;
            R(cur_row, 1) = -dx(0) / 6.0;
            Q(cur_row, 0) = reciprocal_dx(0);
            Q(cur_row, 1) = -reciprocal_dx(0);
            break;
        case 2:
            R(cur_row, 0) = 1.0;
            break;
        case 3:
            Q(cur_row, n) = -1.0;
            break;
        case 4:
            R(cur_row, n - 1) = dx(n - 1) / 6.0;
            R(cur_row, n) = dx(n - 1) / 3.0;
            Q(cur_row, n - 1) = reciprocal_dx(n - 1);
            Q(cur_row, n) = -reciprocal_dx(n - 1);
            break;
        case 5:
            R(cur_row, n) = 1.0;
            break;
        default:
            break;
        }
        ++cur_row;
    }

    H = Eigen::MatrixXd::Zero(n + 1, n + 1);
    for (size_t i = 0; i != n; ++i) {
        H(i, i + 1) = dx(i) / 6.0;
        H(i + 1, i) = dx(i) / 6.0;
        double a_ii = dx(i);
        if (i > 0) {
            a_ii += dx(i - 1);
        }
        H(i, i) = a_ii / 3.0;
    }
    H(n, n) = dx(n - 1) / 3.0;
}

void CubicSplineSmoother::init(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& y,
    double smooth,
    const Eigen::VectorXd& constraints,
    const Eigen::VectorXd& weight) {
    typedef Eigen::SparseMatrix<double> SpaMat;
    _x = x;
    _y = y;
    _smooth = smooth;
    _weight = weight;

    _constraints = constraints;
    if (_weight.size() != _x.size()) {
        _weight = Eigen::VectorXd::Ones(_x.size());
    }
    size_t n = _x.size() - 1;
    Eigen::VectorXd dx = _x.bottomRows(n) - _x.topRows(n);
    Eigen::VectorXd reciprocal_dx = dx.cwiseInverse();
    Eigen::MatrixXd R, Q, H;
    Eigen::VectorXd beta;
    cal_nodes_constraints(R, Q, beta, H);

    size_t dim_lmd = R.rows();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * n + 2 + dim_lmd, 2 * n + 2 + dim_lmd);
    A.block(0, 0, n + 1, n + 1) = 2.0 * _smooth * _weight.asDiagonal();
    A.block(0, 2 * n + 2, n + 1, dim_lmd) = -Q.transpose();
    A.block(n + 1, n + 1, n + 1, n + 1) = 2.0 * (1.0 - _smooth) * H;
    A.block(n + 1, 2 * n + 2, n + 1, dim_lmd) = R.transpose();
    A.block(2 * n + 2, 0, dim_lmd, n + 1) = -Q;
    A.block(2 * n + 2, n + 1, dim_lmd, n + 1) = R;
    Eigen::VectorXd b = Eigen::VectorXd::Zero(A.rows());
    b.topRows(n + 1) = 2.0 * _smooth * _weight.cwiseProduct(_y);
    b.bottomRows(beta.rows()) = beta;

    SpaMat spa_A = A.sparseView();
    Eigen::SparseLU<SpaMat> spa_solver;
    spa_solver.compute(spa_A);
    if (spa_solver.info() != 0) {
        AERROR << "map_mgr:cubic_spline:cubic_spline_decomposition failed";
        return;
    }
    Eigen::VectorXd params = spa_solver.solve(b);

    Eigen::VectorXd val_vec = params.topRows(n + 1);
    Eigen::VectorXd d2_vec = params.block(n + 1, 0, n + 1, 1);
    Eigen::VectorXd d_vec(n);
    d_vec = (val_vec.bottomRows(n) - val_vec.topRows(n)).cwiseProduct(reciprocal_dx) -
        (d2_vec.bottomRows(n) / 6.0 + d2_vec.topRows(n) / 3.0).cwiseProduct(dx);
    _coef = Eigen::MatrixXd::Zero(n, 4);
    _coef.col(3) = val_vec.topRows(n);
    _coef.col(2) = d_vec.topRows(n);
    _coef.col(1) = 0.5 * d2_vec.topRows(n);
    _coef.col(0) = (d2_vec.bottomRows(n) - d2_vec.topRows(n)).cwiseProduct(reciprocal_dx) / 6.0;
}

size_t CubicSplineSmoother::CalPieceIndex(double x) {
    return FindInterpSecIndex(_x, 0, _x.size(), x);
}

size_t CubicSplineSmoother::FindInterpSecIndex(Eigen::VectorXd& array, size_t idx_begin, size_t idx_end, double x) {
    size_t idx = 0;
    size_t N = idx_end - idx_begin;
    if (N <= 2 || x < array[idx_begin]) {
        idx = idx_begin;
    } else if (x >= array[idx_end - 1]) {
        idx = idx_end - 2;
    } else {
        size_t idx_middle = idx_begin + (N >> 1);  // floor(N/2);
        if (x >= array[idx_middle]) {
            idx = FindInterpSecIndex(array, idx_middle, idx_end, x);
        } else {
            idx = FindInterpSecIndex(array, idx_begin, idx_middle + 1, x);
        }
    }
    return idx;
}

Eigen::Vector3d CubicSplineSmoother::Evaluate(double x, size_t piece_idx) {
    double x_loc = x - _x(piece_idx);
    Eigen::VectorXd coef = _coef.row(piece_idx);
    double y = coef(0);
    for (int i = 1; i != coef.size(); ++i) {
        y = y * x_loc + coef(i);
    }
    double dy = 3.0 * coef(0) * x_loc * x_loc + 2.0 * coef(1) * x_loc + coef(2);
    double d2y = 6.0 * coef(0) * x_loc + 2.0 * coef(1);
    Eigen::Vector3d val_dif1_dif2{y, dy, d2y};
    return val_dif1_dif2;
}

double CubicSplineSmoother::Evaluate(double x) {
    auto piece_idx = CalPieceIndex(x);
    auto y_dy_d2y = Evaluate(x, piece_idx);
    return y_dy_d2y(0);
}

}  // namespace pnc
}  // namespace xju
