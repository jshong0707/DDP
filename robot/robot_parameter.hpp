#pragma once

#include "globals.hpp"
#include <memory>
#include <array>
#include <string>



class robot_parameter
{
public:
  robot_parameter();
  ~robot_parameter();

  void robot_param(const Eigen::VectorXd &q,
                   const Eigen::VectorXd &qd,
                   const Eigen::VectorXd &qdd);

  // 접근자들
  Eigen::Vector3d get_leg_pos(int i) const;
  Eigen::Matrix3d get_Jacb   (int i) const;
  Eigen::Vector3d get_rpy    ()    const;
  Eigen::Matrix3d get_R      ()    const;

private:
  struct Impl;                    // Pimpl 전방 선언
  std::unique_ptr<Impl> pimpl_;   // 실제 구현은 Impl 안에
};
