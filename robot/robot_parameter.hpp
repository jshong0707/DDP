#pragma once

#include "globals.hpp"
#include <memory>
#include <array>
#include <string>
// #include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/multibody/model.hpp>

#include <pinocchio/multibody/fwd.hpp>


class robot_parameter
{
public:
  robot_parameter();
  ~robot_parameter();

  void robot_param(const VectorXd &q,
                   const VectorXd &qd,
                   const VectorXd &qdd);

  shared_ptr<pinocchio::Model> getModel() const;
  shared_ptr<pinocchio::Data>  getData()  const;

  Vector3d get_leg_pos(int i) const;
  Matrix3d get_Jacb   (int i) const;
  Vector3d get_rpy    ()    const;
  Matrix3d get_R      ()    const;
  vector<string> get_foot_frame () const;
  VectorXd get_q() const;
  VectorXd get_qd() const;
  
private:
  struct Impl;                    // Pimpl 전방 선언
  unique_ptr<Impl> pimpl_;   // 실제 구현은 Impl 안에
  
};
