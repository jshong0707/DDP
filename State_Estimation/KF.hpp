#pragma once

#include "globals.hpp"
#include "filter.hpp"

#include "mujoco/mujoco.h"

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/math/rpy.hpp>        // for computeRpy
class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter();

    void sensor_measure(const mjModel* m, mjData* d);
    void prediction_step();
    void update_step();                             

    void setProcessNoise(const Eigen::MatrixXd& Q);
    void setMeasurementNoise(const Eigen::MatrixXd& R);
    Eigen::VectorXd get_state() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
