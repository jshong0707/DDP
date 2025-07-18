// Body.hpp
#pragma once

#include <memory>
#include <mujoco/mjmodel.h>
#include <mujoco/mjdata.h>
#include "globals.hpp"
#include "robot_parameter.hpp"

class filter;

class Body {
public:
  Body(robot_parameter &pino);
  ~Body();

  void sensor_measure(const mjModel* m, mjData* d);
  void foot_vector  (const mjModel* m, mjData* d);

  Eigen::VectorXd get_x_ref(double t);
  Eigen::VectorXd get_z_ref(double t);

  void init_x_ref_mpc(int horizon, double sampling_time);

  Eigen::VectorXd  get_x0()       const;
  Eigen::Matrix3d  get_R()        const;
  int              get_Task()     const;
  Eigen::Matrix3d  get_Body_I()   const;
  double           get_Body_M()   const;
  Eigen::Vector3d  get_r_W(int i) const;
  Eigen::Vector3d get_foot_pos_W(int i) const;

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};
