// filter.hpp
#pragma once

#include <memory>
#include "globals.hpp"   // Vector3d, Vector4d, Ts, MatrixXd, Matrix3d 등

class filter {
public:
    filter();
    ~filter();

    MatrixXd pinv(const MatrixXd &A);
    double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
    double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq);
    Vector3d quat2euler(double qw, double qx, double qy, double qz);
    Vector3d quat2rpy(const Vector4d &quat);
    Matrix3d skew(const Vector3d &v);
    Matrix3d xyz_to_zyx_matrix(const Matrix3d &R_xyz, Vector3d &out_zyx_angles);

private:
    struct Impl;                    // 전방 선언
    std::unique_ptr<Impl> pimpl_;   // 실제 구현은 Impl 내부에
};
