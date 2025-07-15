#pragma once

#include <memory>
#include <mujoco/mjmodel.h>   
#include <mujoco/mjdata.h>    
#include "globals.hpp"        

class filter;

class Actuator {
public:
  Actuator(int n);
  ~Actuator();

  Eigen::Vector3d Receive_data(const mjModel* m, mjData* d);

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};
