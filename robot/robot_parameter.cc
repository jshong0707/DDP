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

struct robot_parameter::Impl
{
    shared_ptr<pinocchio::Model> model_;
    shared_ptr<pinocchio::Data>  data_;

    const string imu_frame = "imu_link";

    const std::array<std::array<string,4>,4> legs = {{
      { "FLHAA_point", "FLHIP",  "FLKNEE",  "FL_foot" },
      { "FRHAA_point", "FRHIP",  "FRKNEE",  "FR_foot" },
      { "RLHAA_point", "RLHIP",  "RLKNEE",  "RL_foot" },
      { "RRHAA_point", "RRHIP",  "RRKNEE",  "RR_foot" }
    }};

    const vector<string> foot_frames = 
    {legs[0][3], legs[1][3], legs[2][3], legs[3][3]};

    std::array<Vector3d,4> leg_pos;    // HAA→foot 위치
    std::array<Matrix3d,4> J_B;        // Body 프레임 자코비안
    std::array<Vector3d,4> foot_vector;
    Matrix3d              R_BW;       // Body→World 회전
    Vector3d              rpy;        // yaw‑pitch‑roll
    Quaterniond quat;
    VectorXd q;
    VectorXd qd;
    

    Impl()
    {
      constexpr char URDF[] = "../urdf/3D_Quad.urdf";
      
      model_ = std::make_shared<pinocchio::Model>();
      pinocchio::urdf::buildModel(URDF, *model_);

      data_ = std::make_shared<pinocchio::Data>(*model_);

      std::cout << "Pinocchio model loaded successfully!\n"
                << ", nq = "    << model_->nq    << "\n";

    }

    
  // pinocchio::Model& getModel() { return model_; }
  // pinocchio::Data&  getData()  { return data_;  }
  
  void robot_param(const Eigen::VectorXd &pos,
                   const Eigen::VectorXd &vel,
                   const Eigen::VectorXd &acc)
  {
    using namespace pinocchio;

    q = pos;
    qd = vel;

    pinocchio::normalize(*model_, q);
    forwardKinematics(*model_, *data_, q, qd);
    computeJointJacobians(*model_, *data_, q);
    updateFramePlacements(*model_, *data_);

    const auto &oMf_imu = data_->oMf[model_->getFrameId(imu_frame)];
    R_BW = oMf_imu.rotation();
    
    
    // cout << "pino\n"<< R_BW << endl;
    // cout << "mujoco\n" << quat.toRotationMatrix() << endl;

    // 각 다리별 HAA→foot 위치와 Body 프레임 자코비안 계산  
    for(int i = 0; i < 4; ++i)
    {
      // HAA→foot 변환
      const auto &oMf_HAA  = data_->oMf[ model_->getFrameId(legs[i][0]) ];
      const auto &oMf_foot = data_->oMf[ model_->getFrameId(legs[i][3]) ];
      auto T_HAA2foot = oMf_HAA.inverse() * oMf_foot;
      auto T_imu2foot = oMf_imu.inverse() * oMf_foot;

      leg_pos[i] = T_HAA2foot.translation();
      
      foot_vector[i] = T_imu2foot.translation();

      // cout << i << ": " << leg_pos[i][0] << ": " << leg_pos[i][1] << ": " << leg_pos[i][2] << endl;

      // World‑aligned 자코비안 → Body 프레임
      Eigen::MatrixXd J6(6, model_->nv);
      getFrameJacobian(
        *model_, *data_,
        model_->getFrameId(legs[i][3]),
        LOCAL_WORLD_ALIGNED,
        J6
      );
      // 위치 블록(0..2), 각 다리 조인트 블록(6+3*i ..)
      J_B[i] = R_BW.transpose()
             * J6.block(0, 6 + 3*i, 3, 3);
    }
    
    // cout << "J_B\n" << J_B[0] << endl;
    // RPY(Z‑Y‑X) → yaw,‑pitch,‑roll
    {
      auto ypr = rpy::matrixToRpy(R_BW);
      // Pinocchio 기본 [roll,pitch,yaw]를 [yaw,‑pitch,‑roll]로 조정
      // rpy = { ypr[2], -ypr[1], -ypr[0] };
      rpy = { ypr[0], ypr[1], ypr[2] };
    }
    
    // pinocchio::rnea(*model_, *data_, q, Eigen::VectorXd::Zero(model_->nv), Eigen::VectorXd::Zero(model_->nv));
    // Eigen::VectorXd hg = data_->tau;  // pure gravity generalized forces
    // std::cout << "||hg|| = " << hg.norm() << "\n";  // e11이면 단위/관성 문제


  }

  

  Eigen::Vector3d get_leg_pos(int i) const { return leg_pos[i]; }
  Eigen::Matrix3d get_Jacb   (int i) const { return J_B[i];   }
  Eigen::Vector3d get_rpy    ()    const { return rpy;      }
  Eigen::Matrix3d get_R      ()    const { return R_BW;     }
  std::vector<std::string> get_foot_frame () const {return foot_frames;}

};

// robot_parameter: public → Impl 위임

robot_parameter::robot_parameter()
 : pimpl_(std::make_unique<Impl>()) 
{}

robot_parameter::~robot_parameter() = default;


void robot_parameter::robot_param(const Eigen::VectorXd &pos,
                                 const Eigen::VectorXd &vel,
                                 const Eigen::VectorXd &acc)
{
  pimpl_->robot_param(pos, vel, acc);
}

Eigen::Vector3d robot_parameter::get_leg_pos(int i) const {return pimpl_->get_leg_pos(i);}

Eigen::Matrix3d robot_parameter::get_Jacb(int i) const {return pimpl_->get_Jacb(i);}

Eigen::Vector3d robot_parameter::get_rpy() const {return pimpl_->get_rpy();}

Eigen::Matrix3d robot_parameter::get_R() const {return pimpl_->get_R();}

std::vector<std::string> robot_parameter::get_foot_frame() const {return pimpl_->get_foot_frame();}


std::shared_ptr<pinocchio::Model> robot_parameter::getModel() const {return pimpl_->model_; }

std::shared_ptr<pinocchio::Data> robot_parameter::getData() const {return pimpl_->data_; }

VectorXd robot_parameter::get_q() const {
  return pimpl_->q; }

VectorXd robot_parameter::get_qd() const {return pimpl_->qd;}
 
  