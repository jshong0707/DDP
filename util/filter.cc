// filter.cpp
#include <memory>
#include "filter.hpp"

struct filter::Impl {
    Vector3d euler;  // 내부 상태

    Impl() : euler(Vector3d::Zero()) {}

    MatrixXd pinv(const MatrixXd &A) {
        return A.completeOrthogonalDecomposition().pseudoInverse();
    }

    double tustin_derivative(double input, double input_old,
                             double output_old, double cutoff_freq) {
        double time_const = 1/(2*M_PI*cutoff_freq);
        // Ts 는 globals.hpp 에 extern 으로 선언·정의되어 있어야 합니다
        return (2*(input - input_old) - (Ts - 2*time_const)*output_old)
               / (Ts + 2*time_const);
    }

    double lowpassfilter(double input, double input_old,
                         double output_old, double cutoff_freq) {
        double time_const = 1/(2*M_PI*cutoff_freq);
        return (Ts*(input + input_old) - (Ts - 2*time_const)*output_old)
               / (Ts + 2*time_const);
    }

    Vector3d quat2euler(double qw, double qx, double qy, double qz) {
        double sinr_cosp = 2*(qw*qx + qy*qz);
        double cosr_cosp = 1 - 2*(qx*qx + qy*qy);
        euler[0] = M_PI/2 + std::atan2(sinr_cosp, cosr_cosp);

        double sinp = std::sqrt(1 + 2*(qw*qy - qx*qz));
        double cosp = std::sqrt(1 - 2*(qw*qy - qx*qz));
        euler[1] = 2*std::atan2(sinp, cosp) - M_PI/2;

        double siny_cosp = 2*(qw*qz + qx*qy);
        double cosy_cosp = 1 - 2*(qy*qy + qz*qz);
        euler[2] = std::atan2(siny_cosp, cosy_cosp);
        return euler;
    }

    Vector3d quat2rpy(const Vector4d &quat) {
        Vector3d rpy;
        rpy[0] = std::atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]),
                            1 - 2*(quat[1]*quat[1] + quat[2]*quat[2]));
        rpy[1] = std::asin(2*(quat[0]*quat[2] - quat[3]*quat[1]));
        rpy[2] = std::atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]),
                            1 - 2*(quat[2]*quat[2] + quat[3]*quat[3]));
        return rpy;
    }

    Matrix3d skew(const Vector3d &v) {
        Matrix3d S;
        S <<    0,   -v[2],  v[1],
             v[2],      0,  -v[0],
            -v[1],   v[0],     0;
        return S;
    }

    Matrix3d xyz_to_zyx_matrix(const Matrix3d &R_xyz, Vector3d &out_zyx_angles) {
        
        out_zyx_angles = R_xyz.eulerAngles(2,1,0);
        
        AngleAxisd Rz(out_zyx_angles[0], Vector3d::UnitZ());
        AngleAxisd Ry(out_zyx_angles[1], Vector3d::UnitY());
        AngleAxisd Rx(out_zyx_angles[2], Vector3d::UnitX());

        return (Rz * Ry * Rx).matrix();
    }
};

// Public API 위임
filter::filter() : pimpl_(std::make_unique<Impl>()) {}
filter::~filter() = default;

MatrixXd filter::pinv(const MatrixXd &A) {
    return pimpl_->pinv(A);
}

double filter::tustin_derivative(double input, double input_old,
                                 double output_old, double cutoff_freq) {
    return pimpl_->tustin_derivative(input, input_old, output_old, cutoff_freq);
}

double filter::lowpassfilter(double input, double input_old,
                             double output_old, double cutoff_freq) {
    return pimpl_->lowpassfilter(input, input_old, output_old, cutoff_freq);
}

Vector3d filter::quat2euler(double qw, double qx, double qy, double qz) {
    return pimpl_->quat2euler(qw, qx, qy, qz);
}

Vector3d filter::quat2rpy(const Vector4d &quat) {
    return pimpl_->quat2rpy(quat);
}

Matrix3d filter::skew(const Vector3d &v) {
    return pimpl_->skew(v);
}

Matrix3d filter::xyz_to_zyx_matrix(const Matrix3d &R_xyz, Vector3d &out_zyx_angles) {
    return pimpl_->xyz_to_zyx_matrix(R_xyz, out_zyx_angles);
}
