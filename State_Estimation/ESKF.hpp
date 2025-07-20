#pragma once

#include "filter.hpp"
#include "globals.hpp"
#include "mujoco/mujoco.h"

// Pinocchio
#include <pinocchio/fwd.hpp>                     // 필수: 전방 선언
#include <pinocchio/parsers/urdf.hpp>            // URDF 파서
#include <pinocchio/algorithm/kinematics.hpp>    // forwardKinematics 등
#include <pinocchio/algorithm/jacobian.hpp>      // getFrameJacobian 등
#include <pinocchio/algorithm/frames.hpp>        // updateFramePlacements 등
#include <pinocchio/algorithm/model.hpp>         // getFrameId, nq, nv 등 메타 정보

class ErrorStateKalmanFilter {
public:
    ErrorStateKalmanFilter();
    ~ErrorStateKalmanFilter();
    
    void sensor_measure(const mjModel* m, mjData* d);
    void prediction_step();
    void update_step();

    void setProcessAccelNoise(const Eigen::MatrixXd& Q);
    void setProcessAccelBiasNoise(const Eigen::MatrixXd& Q);
    void setProcessGyroNoise(const Eigen::MatrixXd& Q);
    void setProcessGyroBiasNoise(const Eigen::MatrixXd& Q);
    void setProcessFootNoise(const Eigen::MatrixXd& Q);
    void setMeasurementNoise(const Eigen::MatrixXd& R);
    Eigen::VectorXd get_state() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
