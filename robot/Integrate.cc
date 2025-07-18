#include "Integrate.hpp"
#include "robot_parameter.hpp"
#include "Trajectory.hpp"
#include "Body.hpp"
#include "MPC.hpp"
#include "Controller.hpp"
#include "FSM.hpp"
#include "KF.hpp"
#include <array>

struct Integrate::Impl {
  robot_parameter &pino;
  Trajectory      &Traj;
  Body            &B;
  MPC             &M;
  Controller      &C;
  FSM             &FSM_;
  KalmanFilter    &KF;

  double t = 0;
  double optimization_t = 0;
  double mpc_dt = 0;

  Eigen::VectorXd q, qd, qdd;
  Eigen::Vector3d FB_input[4];
  Eigen::Vector3d F_Joint_input[2];
  Eigen::Vector3d B_Joint_input[2];
  Eigen::VectorXd opt_u;
  Eigen::Vector3d opt_GRF[4];
  bool Contact[4];
  bool Contact_traj[4];

  Eigen::VectorXd leg_pos;
  Eigen::VectorXd leg_pos_ref;
  Eigen::Vector3d pos_err[4], pos_err_old[4];
  std::vector<bool> is_contact;

  Impl(robot_parameter &p,
       Trajectory      &t_,
       Body            &b,
       MPC             &m,
       Controller      &c,
       FSM             &f,
       KalmanFilter    &kf)
    : pino(p),
      Traj(t_),
      B(b),
      M(m),
      C(c),
      FSM_(f),
      KF(kf),
      q(Eigen::VectorXd::Zero(19)),
      qd(Eigen::VectorXd::Zero(18)),
      qdd(Eigen::VectorXd::Zero(18)),
      opt_u(Eigen::VectorXd::Zero(12)),
      leg_pos(Eigen::VectorXd::Zero(12)),
      leg_pos_ref(Eigen::VectorXd::Zero(12)),
      is_contact(4, false)
  {
    mpc_dt = M.get_dt();
    // 초기화: FB_input, opt_GRF, pos_err, Contact 등
    for(int i=0;i<4;i++){
      FB_input[i].setZero();
      opt_GRF[i].setZero();
      pos_err[i].setZero();
      pos_err_old[i].setZero();
      Contact[i] = Contact_traj[i] = true;
    }
  }
};

Integrate::Integrate(robot_parameter &pino,
                     Trajectory      &Traj,
                     Body            &B,
                     MPC             &M,
                     Controller      &C,
                     FSM             &FSM_,
                     KalmanFilter    &KF)
  : pimpl_(std::make_unique<Impl>(pino, Traj, B, M, C, FSM_, KF))
{}

Integrate::~Integrate() = default;

void Integrate::sensor_measure(const mjModel* m, mjData* d) {
  auto &i = *pimpl_;
  i.t = d->time;
  for(int k=0;k<19;k++) i.q[k] = d->qpos[k];
  for(int k=0;k<18;k++) i.qd[k] = d->qvel[k];
  for(int k=0;k<18;k++) i.qdd[k] = d->qacc[k];

  i.pino.robot_param(i.q, i.qd, i.qdd);
  i.B.sensor_measure(m, d);
  i.M.foot_vector(m, d);

  // Q: 상태 예측 오차를 모델링 (가속 노이즈 등)
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(18, 18);
  Q.block<3,3>(0, 0) << 1e-2, 0, 0, // body x pos
                        0, 1e-2, 0, // body y pos
                        0, 0, 1e-2; // body z pos

  Q.block<3,3>(3, 3) << 1e-2, 0, 0, // body x vel
                        0, 1e-2, 0, // body y vel
                        0, 0, 1e-2; // body z vel

  Q.block<3,3>(6, 6) << 1e-2, 0, 0, // foot x
                        0, 1e-2, 0, // foot y
                        0, 0, 1e-2; // foot z
  Q.block<3,3>(9, 9) = Q.block<3,3>(6, 6);
  Q.block<3,3>(12, 12) = Q.block<3,3>(6, 6);

  // R: 측정 오차를 모델링 (상대 발 위치/속도 센서 노이즈)
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(24, 24);
  R.block<3,3>(0, 0) << 1e-2, 0, 0, // body to foot x pos
                        0, 1e-2, 0, // body to foot y pos
                        0, 0, 1e-2; // body to foot z pos
  R.block<3,3>(3, 3) = R.block<3,3>(0, 0);
  R.block<3,3>(6, 6) = R.block<3,3>(0, 0);
  R.block<3,3>(9, 9) = R.block<3,3>(0, 0);

  R.block<3,3>(12, 12) << 1e-12, 0, 0, // body to foot x pos
                          0, 1e-12, 0, // body to foot y pos
                          0, 0, 1e-12; // body to foot z pos
  R.block<3,3>(15, 15) = R.block<3,3>(12, 12);
  R.block<3,3>(18, 18) = R.block<3,3>(12, 12);
  R.block<3,3>(21, 21) = R.block<3,3>(12, 12);

  i.KF.setProcessNoise(Q);
  i.KF.setMeasurementNoise(R);

  i.KF.prediction_step();
  i.KF.sensor_measure(m, d);
  i.KF.update_step();
}

void Integrate::get_error(double /*unused*/) {
  auto &i = *pimpl_;
  i.leg_pos_ref = i.Traj.Traj(i.t);
  for(int k=0;k<4;k++) i.pos_err_old[k] = i.pos_err[k];
  for(int k=0;k<4;k++){
    i.pos_err[k] = i.leg_pos_ref.segment<3>(3*k)
                 - i.pino.get_leg_pos(k);
  }
}

void Integrate::Leg_controller() {
  auto &i = *pimpl_;

  i.is_contact = i.FSM_.contactschedule();
  for(int k=0;k<4;k++){
    if(!i.is_contact[k])
      i.FB_input[k] = i.C.FB_controller(i.pos_err[k], i.pos_err_old[k], k);
    else
      i.FB_input[k] = Eigen::Vector3d::Zero();
  }

  if(i.t >= i.optimization_t) {
    i.M.Dynamics();
    i.M.SolveQP();
    i.opt_u = i.M.get_opt_u();
    if(i.t < 1e-8) i.opt_u.setZero();
    for(int k=0;k<4;k++){
        i.opt_u[3*k] = -i.opt_u[3*k];
        i.opt_u[3*k + 1] = -i.opt_u[3*k + 1];

      i.opt_GRF[k] = i.B.get_R().transpose()
                     * (i.opt_u.segment<3>(3*k));
    }
    i.optimization_t += i.mpc_dt;
  }

  i.F_Joint_input[0] = i.pino.get_Jacb(0).transpose()
                      * (i.opt_GRF[0] + i.FB_input[0]);
  i.F_Joint_input[1] = i.pino.get_Jacb(1).transpose()
                      * (i.opt_GRF[1] + i.FB_input[1]);
  i.B_Joint_input[0] = i.pino.get_Jacb(2).transpose()
                      * (i.opt_GRF[2] + i.FB_input[2]);
  i.B_Joint_input[1] = i.pino.get_Jacb(3).transpose()
                      * (i.opt_GRF[3] + i.FB_input[3]);

}

void Integrate::Data_log() {
  auto &i = *pimpl_;
  for(int k=0;k<4;k++){
    i.leg_pos.segment<3>(3*k)
      = i.pino.get_leg_pos(k);
  }
}

Eigen::Vector3d Integrate::get_FL_J_input() { return pimpl_->F_Joint_input[0]; }
Eigen::Vector3d Integrate::get_FR_J_input() { return pimpl_->F_Joint_input[1]; }
Eigen::Vector3d Integrate::get_RL_J_input() { return pimpl_->B_Joint_input[0]; }
Eigen::Vector3d Integrate::get_RR_J_input() { return pimpl_->B_Joint_input[1]; }

Eigen::VectorXd Integrate::get_leg_pos_ref() { return pimpl_->leg_pos_ref; }
Eigen::VectorXd Integrate::get_leg_pos()     { return pimpl_->leg_pos; }
Eigen::VectorXd Integrate::get_x0()          { return pimpl_->B.get_x0(); }
Eigen::VectorXd Integrate::get_x0_est()      { return pimpl_->KF.get_state(); }
Eigen::VectorXd Integrate::get_W_foot_pos(int i)  { return pimpl_->B.get_foot_pos_W(i); }
Eigen::VectorXd Integrate::get_x_ref()       { return pimpl_->B.get_x_ref(pimpl_->t); }
Eigen::VectorXd Integrate::get_opt_GRF(int leg) { return pimpl_->opt_GRF[leg]; }