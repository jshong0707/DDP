// Actuator.cpp
#include "Actuator.hpp"
#include "filter.hpp"

struct Actuator::Impl {
  filter* F;
  int     ACT_num;
  double  d_cutoff;
  double  q, q_old;
  double  qd_tustin, qd_tustin_old;
  double  qdd_tustin, qdd_tustin_old;

  Impl(int n)
    : F(nullptr),
      ACT_num(n),
      d_cutoff(70.0),
      q(0.0), q_old(0.0),
      qd_tustin(0.0), qd_tustin_old(0.0),
      qdd_tustin(0.0), qdd_tustin_old(0.0)
  {}

  Eigen::Vector3d Receive_data(const mjModel* m, mjData* d) {
    q = d->qpos[ACT_num + 7];

    double vel  = F->tustin_derivative(q,       q_old,        qd_tustin_old, d_cutoff);
    double accel= F->tustin_derivative(vel,     qd_tustin_old, qdd_tustin_old, d_cutoff);

    q_old           = q;
    qd_tustin_old   = vel;
    qdd_tustin_old  = accel;

    return Eigen::Vector3d{ q, vel, accel };
  }
};

Actuator::Actuator(int n)
  : pimpl_(std::make_unique<Impl>(n))
{}

Actuator::~Actuator() = default;

Eigen::Vector3d Actuator::Receive_data(const mjModel* m, mjData* d) {
  return pimpl_->Receive_data(m, d);
}
