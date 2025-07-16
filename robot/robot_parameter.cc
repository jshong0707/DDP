#include "robot_parameter.hpp"

// Pinocchio 헤더는 CPP 파일에서만 포함
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/math/rpy.hpp>

#include <iostream>

// Impl 정의 및 구현
struct robot_parameter::Impl
{
  // Pinocchio 모델 & 데이터
  pinocchio::Model model_;
  pinocchio::Data  data_;

  // URDF 에 정의된 프레임 이름
  const std::string imu_frame = "imu_joint";

  // 각 다리별 HAA 지점부터 foot 지점까지의 링크 이름
  const std::array<std::array<std::string,4>,4> legs = {{
    { "FLHAA_point", "FLHIP",  "FLKNEE",  "FL_foot" },
    { "FRHAA_point", "FRHIP",  "FRKNEE",  "FR_foot" },
    { "RLHAA_point", "RLHIP",  "RLKNEE",  "RL_foot" },
    { "RRHAA_point", "RRHIP",  "RRKNEE",  "RR_foot" }
  }};

  // 계산된 결과 저장
  std::array<Eigen::Vector3d,4> leg_pos;    // HAA→foot 위치
  std::array<Eigen::Matrix3d,4> J_B;        // Body 프레임 자코비안
  Eigen::Matrix3d              R_BW;       // Body→World 회전
  Eigen::Vector3d              rpy;        // yaw‑pitch‑roll

  Impl()
  {
    constexpr char URDF[] = "../urdf/3D_Quad.urdf";
    
    pinocchio::urdf::buildModel(
      URDF,
      pinocchio::JointModelFreeFlyer(),
      model_);
    data_ = pinocchio::Data(model_);

    std::cout << "Pinocchio model loaded successfully!\n"
              << ", nq = "    << model_.nq    << "\n";
  }

  // q, qd, qdd 받아서 순·역·자코비안·RPY 계산
  void robot_param(const Eigen::VectorXd &q,
                   const Eigen::VectorXd &qd,
                   const Eigen::VectorXd &qdd)
  {
    using namespace pinocchio;

    // 순동역학/자코비안/프레임 업데이트
    forwardKinematics(model_, data_, q, qd);
    computeJointJacobians(model_, data_, q);
    updateFramePlacements(model_, data_);

    // IMU 프레임 정보
    const auto &oMf_imu = data_.oMf[model_.getFrameId(imu_frame)];
    R_BW = oMf_imu.rotation();

    // 각 다리별 HAA→foot 위치와 Body 프레임 자코비안 계산
    for(int i = 0; i < 4; ++i)
    {
      // HAA→foot 변환
      const auto &oMf_HAA  = data_.oMf[model_.getFrameId(legs[i][0])];
      const auto &oMf_foot = data_.oMf[model_.getFrameId(legs[i][3])];
      auto T_HAA2foot = oMf_HAA.inverse() * oMf_foot;
      leg_pos[i] = T_HAA2foot.translation();

      // World‑aligned 자코비안 → Body 프레임
      Eigen::MatrixXd J6(6, model_.nv);
      getFrameJacobian(
        model_, data_,
        model_.getFrameId(legs[i][3]),
        LOCAL_WORLD_ALIGNED,
        J6
      );
      // 위치 블록(0..2), 각 다리 조인트 블록(6+3*i ..)
      J_B[i] = R_BW.transpose()
             * J6.block(0, 6 + 3*i, 3, 3);
    }

    // RPY(Z‑Y‑X) → yaw,‑pitch,‑roll
    {
      auto ypr = rpy::matrixToRpy(R_BW);
      // Pinocchio 기본 [roll,pitch,yaw]를 [yaw,‑pitch,‑roll]로 조정
      rpy = { ypr[2], -ypr[1], -ypr[0] };
    }
  }

  // 접근자들
  Eigen::Vector3d get_leg_pos(int i) const { return leg_pos[i]; }
  Eigen::Matrix3d get_Jacb   (int i) const { return J_B[i];   }
  Eigen::Vector3d get_rpy    ()    const { return rpy;      }
  Eigen::Matrix3d get_R      ()    const { return R_BW;     }
};

// robot_parameter: public → Impl 위임

robot_parameter::robot_parameter()
 : pimpl_(std::make_unique<Impl>()) 
{}

robot_parameter::~robot_parameter() = default;

void robot_parameter::robot_param(const Eigen::VectorXd &q,
                                 const Eigen::VectorXd &qd,
                                 const Eigen::VectorXd &qdd)
{
  pimpl_->robot_param(q, qd, qdd);
}

Eigen::Vector3d robot_parameter::get_leg_pos(int i) const {
  return pimpl_->get_leg_pos(i);
}

Eigen::Matrix3d robot_parameter::get_Jacb(int i) const {
  return pimpl_->get_Jacb(i);
}

Eigen::Vector3d robot_parameter::get_rpy() const {
  return pimpl_->get_rpy();
}

Eigen::Matrix3d robot_parameter::get_R() const {
  return pimpl_->get_R();
}
