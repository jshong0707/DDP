// Trajectory.hpp
#pragma once

#include <memory>
#include <vector>
#include "globals.hpp"

class Body;
class robot_parameter;

class Trajectory {
public:

    Trajectory(robot_parameter &pino, Body &B_);
    ~Trajectory();

    VectorXd custom_leg_traj(double t);
    VectorXd Hold();
    VectorXd swing_traj(double t);
    VectorXd Traj(double t);
    Vector3d get_foot_pos(int Leg_num);
    std::vector<bool> FSM();

private:

    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};