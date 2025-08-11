#pragma once

#include <memory>
#include <vector>
#include <mujoco/mjmodel.h>
#include <mujoco/mjdata.h>
#include "globals.hpp"

class robot_parameter;
class Trajectory;
class Body;
class MPC;
class Controller;
class FSM;

class Integrate
{
public:
  Integrate(std::shared_ptr<robot_parameter> p,
            Trajectory      &Traj,
            Body            &B,
            // MPC             &M,
            Controller      &C,
            FSM             &FSM_);
  ~Integrate();

  void sensor_measure(const mjModel* m, mjData* d);
  void get_error(double t);
  void Leg_controller();
  void Data_log();

  Eigen::Vector3d get_FL_J_input();
  Eigen::Vector3d get_FR_J_input();
  Eigen::Vector3d get_RL_J_input();
  Eigen::Vector3d get_RR_J_input();

  Eigen::VectorXd get_leg_pos_ref();
  Eigen::VectorXd get_leg_pos();
  Eigen::VectorXd get_x0();
  Eigen::VectorXd get_x_ref();
  Eigen::VectorXd get_opt_GRF(int leg);

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};
