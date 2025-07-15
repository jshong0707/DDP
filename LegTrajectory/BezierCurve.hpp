#pragma once

#include "globals.hpp"
#include <memory>
#include <Eigen/Dense>
#include <vector>

class BezierCurve {
public:
    BezierCurve();
    ~BezierCurve();

    Eigen::VectorXd getBezierCurve(const Eigen::MatrixXd& points, double t);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_; // 내부에서 destructor에 delete pimpl해줌
};