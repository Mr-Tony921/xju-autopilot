#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
namespace xju {
namespace pnc {

class CubicSplineSmoother {
public:
    CubicSplineSmoother();
    void init(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& y,
        double smooth,
        const Eigen::VectorXd& constraints = Eigen::VectorXd(),
        const Eigen::VectorXd& weight = Eigen::VectorXd());
    const Eigen::MatrixXd& get_coefs() const {
        return _coef;
    }

    Eigen::Vector3d Evaluate(double x, size_t piece_index);
    double Evaluate(double x);
    size_t CalPieceIndex(double x);

private:
    void cal_nodes_constraints(Eigen::MatrixXd& R, Eigen::MatrixXd& Q, Eigen::VectorXd& beta, Eigen::MatrixXd& H);
    size_t FindInterpSecIndex(Eigen::VectorXd& array, size_t idx_begin, size_t idx_end, double x);

    // polynomials' coefficients
    // i-th section polynomial: _coef(i, 0) * (x - x_i)^3 + _coef(i, 1) * (x - x_i)^2
    // + _coef(i, 2) * (x - x_i) + _coef(i, 3)
    Eigen::MatrixXd _coef;
    Eigen::VectorXd _x;            // interpolation knots
    Eigen::VectorXd _y;            // function value at interpolation knots
    Eigen::VectorXd _weight;       // weight at interpolation knots
    double _smooth;                // smooth parameter[0, 1], 0: smoothest, 1: most accuracy
    Eigen::VectorXd _constraints;  // terminal constraints
};

}  // namespace pnc
}  // namespace xju
