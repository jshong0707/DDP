#pragma once

#include "globals.hpp"

#include <pinocchio/fwd.hpp> // 반드시 모든 Pinocchio 헤더보다 먼저 포함
#include <pinocchio/parsers/urdf.hpp>          // URDF 파일 로드를 위해 필요
#include <pinocchio/algorithm/rnea.hpp>         // 역동역학 (Inverse Dynamics)
#include <pinocchio/algorithm/aba.hpp>          // 순동역학 (Forward Dynamics)
#include <pinocchio/algorithm/jacobian.hpp>     // Jacobian 계산
#include <pinocchio/algorithm/joint-configuration.hpp> // neutral, randomConfiguration 등
#include <pinocchio/algorithm/center-of-mass.hpp> // Center of Mass 계산
#include <pinocchio/algorithm/frames.hpp>       // 프레임 정보 업데이트 (Jacobian 등 전)
#include <pinocchio/algorithm/compute-all-terms.hpp> // 모든 동역학 항 계산 (RNEA, ABA 등에 유용)
#include <pinocchio/algorithm/crba.hpp>       

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/math/rpy.hpp>


using namespace pinocchio;

class robot_parameter
{
private:
    pinocchio::Model model_;
    pinocchio::Data data_;

    const vector<string> foot = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    const string Body = "imu_joint";

    Vector3d r_W[4]; // foot vector world frame
    std::vector<string> legs[4];
    Matrix3d R_BW; // Body to World
    Matrix3d J_B[4]; // Jacobian, Body frame
    
    pinocchio::SE3 T_W_HAA[4]; // Homogeneous Transform root to HAA
    pinocchio::SE3 T_W_foot[4]; // Homogeneous Transform root to foot
    pinocchio::SE3 T_B[4]; // Homogeneous Transform HAA to foot
    Vector3d leg_pos[4];

    Vector3d rpy = Vector3d::Zero();
    Matrix3d Leg_Inertia[4];

public:
    robot_parameter();
    ~robot_parameter();

    void robot_param(VectorXd q, VectorXd qd, VectorXd qdd);
    Vector3d get_leg_pos(int i){return leg_pos[i];}
    Matrix3d get_Jacb(int i){return J_B[i];}
    Vector3d get_rpy(){return rpy;}
    Matrix3d get_R(){return R_BW;}
    
    
};

