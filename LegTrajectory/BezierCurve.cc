#include "BezierCurve.hpp"
#include <cmath>
#include <stdexcept>

// Public interface forwarding to Impl
BezierCurve::BezierCurve()
    : pimpl_(std::make_unique<Impl>())   // Hip 영역에 Impl 객체를 만듬
{}

BezierCurve::~BezierCurve() = default;

// Impl definition
struct BezierCurve::Impl {

  /* Variable */
    Eigen::MatrixXd Derivative_points;
    double Bezier_Order = 6;

    Impl() = default; // Constructor  Impl() {/* */}

    Eigen::VectorXd compute(const Eigen::MatrixXd& points, double t) {
        int N = points.rows();  // number of Bezier points
        int m = points.cols();  // number of dimensions

        Eigen::VectorXd B(m);
        B.setZero();

        // check the range of t : t in [0, 1]
        if (t < 0 || t > 1) {
            throw std::runtime_error("The parameter t is out of range.");
        }

        for (int j = 0; j < N; ++j) {
            double binomial_coeff = std::tgamma(N) / (std::tgamma(j + 1) * std::tgamma(N - j));
            B += binomial_coeff * std::pow(t, j) * std::pow(1 - t, N - 1 - j) * points.row(j);
        }

        return B;
    }
};


Eigen::VectorXd BezierCurve::getBezierCurve(const Eigen::MatrixXd& points, double t) {
    return pimpl_->compute(points, t);
}
