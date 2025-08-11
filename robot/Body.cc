// Body.cpp
#include "Body.hpp"
#include "filter.hpp"
#include <cmath>

struct Body::Impl {
  std::shared_ptr<robot_parameter> pino;
  Eigen::Matrix3d I;
  double          M;
  int             N_Task;
  filter*         F;

  Eigen::VectorXd x0;
  Eigen::Matrix3d R;

  Eigen::VectorXd x_ref;
  Eigen::VectorXd z_ref;

  Eigen::Vector3d omega_IMU;
  Eigen::Matrix3d omega_IMU_hat;
  Eigen::Vector3d u;
  Eigen::Matrix3d u_hat;
  double          theta;
  Eigen::Vector3d CoM_pos_W;
  Eigen::Vector3d r_W[4];
  Eigen::Quaterniond quat;
  
  Eigen::Matrix3d exp_omega_dt;
  int   horizon;
  int   nx;
  double MPC_dt;
  double dt;

  Impl(std::shared_ptr<robot_parameter> p)
    : pino(std::move(p)),
      I(Eigen::Matrix3d::Zero()),
      M(17.0),
      N_Task(1),
      F(nullptr),
      omega_IMU(Eigen::Vector3d::Zero()),
      omega_IMU_hat(Eigen::Matrix3d::Zero()),
      u(Eigen::Vector3d::Zero()),
      u_hat(Eigen::Matrix3d::Zero()),
      theta(0.0),
      CoM_pos_W(Eigen::Vector3d::Zero()),
      horizon(0),
      nx(12),
      MPC_dt(0.0),
      dt(0.001)
  {
    x0    = Eigen::VectorXd::Zero(12);
    x_ref = Eigen::VectorXd::Zero(12);
    z_ref = Eigen::VectorXd::Zero(12*2*horizon);

    // 관성 모멘트 초기화 (원본 Body 생성자 로직)
    I(0,0) = 0.289178;
    I(1,1) = 0.539984;
    I(2,2) = 0.829139;

    R = Eigen::Matrix3d::Identity();
  }

  void sensor_measure(const mjModel* m, mjData* d) {
    foot_vector(m, d);

    quat = Quaterniond(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
    // quat = Quaterniond(d->qpos[6], d->qpos[4], d->qpos[5], d->qpos[6]);

    Vector4d Quat;
    Quat << d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6];
    Eigen::Vector3d th = F->quat2rpy(Quat);
    
    omega_IMU << d->sensordata[3], d->sensordata[4], d->sensordata[5];

    x0 << th[0], th[1], th[2],
          d->qpos[0], d->qpos[1], d->qpos[2],
          omega_IMU[0], omega_IMU[1], omega_IMU[2],
          d->qvel[0], d->qvel[1], d->qvel[2];
    
  }

  
  void foot_vector(const mjModel* m, mjData* d) {
    CoM_pos_W << d->subtree_com[0], d->subtree_com[1], d->subtree_com[2];
    for(int i=0; i<4; ++i) {
      r_W[i] << d->site_xpos[3*i+3] - CoM_pos_W[0],
                d->site_xpos[3*i+4] - CoM_pos_W[1],
                d->site_xpos[3*i+5] - CoM_pos_W[2];
    }

  }

  Eigen::VectorXd get_x_ref(double t) {
    // 기본 trot reference
    x_ref << 0,        // roll
             0,        // pitch
             0.0*t,    // yaw
             0.2*t,       // x
             0,        // y
             0.3536,   // z
             0,        // roll dot
             0,        // pitch dot
             0.0,      // yaw dot
             0.2,       // x dot
             0,        // y dot
             0;        // z dot

    // 이하 여러 모션 시나리오 예시 (원본 주석 코드)
    /*
    if(t<5) {
      x_ref << 0,0,0.2*t,
               0.2*t,0.2*t,0.3536,
               0,0,0.2,
               0.2,0.2,0;
    } else {
      x_ref << 0,0,0.0,
               0.3*t,0,0.3536,
               0,0,0.0,
               0.3,0,0;
    }
    // Front jump
    if(t < 1) {
      x_ref[5]  = 0.3536 - 0.2536*t;
      x_ref[11] = -0.2536;
    } else if(t < 1.5) {
      x_ref[5]  = 0.1;
      x_ref[11] = 0;
    } else {
      x_ref[3]  = 1*t;
      x_ref[9]  = 1;
      x_ref[5]  = 0.1 + 2*t;
      x_ref[11] = 2;
    }
    // run
    if(t < 1) {
      x_ref[3] = 0.1*t;
      x_ref[9] = 0.1;
    } else if(t < 2) {
      x_ref[3] = 0.2*t;
      x_ref[9] = 0.2;
    } else {
      x_ref[3] = 0.25*t + 0.3;
      x_ref[9] = 0.25;
    }
    */

    return x_ref;
  }

  void init_x_ref_mpc(int h, double sampling_time) {
    horizon = h;
    MPC_dt   = sampling_time;
    z_ref.resize(nx * 2 * horizon);
    z_ref.setZero();
  }

  Eigen::VectorXd get_z_ref(double t) {
    // mpc horizon reference
    for(int k=0; k<horizon; ++k) {
      z_ref.segment(k*nx*2, nx) = get_x_ref(t + k*MPC_dt);
    }
    return z_ref;
  }

  Eigen::Matrix3d get_R() const {
    Eigen::Vector3d RPY  = x0.segment<3>(0);
    Eigen::Matrix3d R_BW;

    R_BW = quat.toRotationMatrix();
    // // Body→World 회전 (RPY)
    // R1 << cos(RPY[2])*cos(RPY[1]),
    //       cos(RPY[2])*sin(RPY[1])*sin(RPY[0]) - sin(RPY[2])*cos(RPY[0]),
    //       cos(RPY[2])*sin(RPY[1])*cos(RPY[0]) + sin(RPY[2])*sin(RPY[0]),
    //       sin(RPY[2])*cos(RPY[1]),
    //       sin(RPY[2])*sin(RPY[1])*sin(RPY[0]) + cos(RPY[2])*cos(RPY[0]),
    //       sin(RPY[2])*sin(RPY[1])*cos(RPY[0]) - cos(RPY[2])*sin(RPY[0]),
    //       -sin(RPY[1]),
    //       cos(RPY[1])*sin(RPY[0]),
    //       cos(RPY[1])*cos(RPY[0]);


          // cout << "Used R\n" << R_BW << endl;

    return R_BW;
  }
};

// Body public API 위임
Body::Body(std::shared_ptr<robot_parameter> pino)
  : pimpl_(std::make_unique<Impl>(std::move(pino))) {}

Body::~Body() = default;

void Body::sensor_measure(const mjModel* m, mjData* d) {
  pimpl_->sensor_measure(m, d);
}

void Body::foot_vector(const mjModel* m, mjData* d) {
  pimpl_->foot_vector(m, d);
}

Eigen::VectorXd Body::get_x_ref(double t) {
  return pimpl_->get_x_ref(t);
}

Eigen::VectorXd Body::get_z_ref(double t) {
  return pimpl_->get_z_ref(t);
}

void Body::init_x_ref_mpc(int h, double dt) {
  pimpl_->init_x_ref_mpc(h, dt);
}

Eigen::VectorXd Body::get_x0() const {
  return pimpl_->x0;
}

Eigen::Matrix3d Body::get_R() const {
  return pimpl_->get_R();
}

int Body::get_Task() const {
  return pimpl_->N_Task;
}

Eigen::Matrix3d Body::get_Body_I() const {
  return pimpl_->I;
}

double Body::get_Body_M() const {
  return pimpl_->M;
}

Eigen::Vector3d Body::get_r_W(int i) const {
  return pimpl_->r_W[i];
}
