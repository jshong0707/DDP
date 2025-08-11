#include "Trajectory.hpp"
#include "Body.hpp"
#include "robot_parameter.hpp"
#include "F_Kinematics.hpp"
#include "B_Kinematics.hpp"
#include "BezierCurve.hpp"
#include <memory>
#include <cmath>

// Impl 정의 및 구현
struct Trajectory::Impl {

    std::unique_ptr<BezierCurve> Bezier;
    std::shared_ptr<robot_parameter> pino;
    Body &B_;

    Eigen::VectorXd pos_ref;
    int N_task;
    double t_norm;
    std::vector<double> gait_phase_delay;
    Eigen::MatrixXd Bz_points;
    double swing_ratio;
    double T_swing, T_stance, T_total;
    Eigen::Vector3d pdot_ref, pdot, thdot;
    double stride;
    std::vector<bool> is_contact, old_contact;
    Eigen::Vector3d stance_end_pos[4];
    Eigen::VectorXd x_ref, x0;
    Eigen::Vector3d CP;
    double w;

    // Impl 생성자에서 초기화
    Impl(std::shared_ptr<robot_parameter> p, Body &b)
      : Bezier(std::make_unique<BezierCurve>()), pino(std::move(p)), B_(b), t_norm(0),
        pdot_ref(Eigen::Vector3d::Zero()), pdot(Eigen::Vector3d::Zero()), thdot(Eigen::Vector3d::Zero()),
        is_contact{true,true,true,true}, old_contact{true,true,true,true} 
        {
            pos_ref = Eigen::VectorXd::Zero(12);
            T_swing = 0.3;
            T_stance = 0.3;
            T_total = T_swing + T_stance;
            gait_phase_delay = {0.0, 0.5, 0.5, 0.0};
            swing_ratio = 0.5;
            Bz_points.resize(5, 3);
            x_ref.resize(12); x_ref.setZero();
            N_task = B_.get_Task();
            w = std::sqrt(0.36/9.81);
        }

    // 내부 로직 메서드
    Eigen::VectorXd custom_leg_traj(double t) {
        // FL trajectory logic
        pos_ref << 0, 0, -0.3536,
                   0, 0, -0.3536,
                  -0.1, 0, -0.3536,
                  -0.1, 0, -0.3536;
        return pos_ref;
    }

    Eigen::VectorXd Hold() {
        is_contact = {true,true,true,true};
        for(int leg=0; leg<4; ++leg)
            pos_ref.segment<3>(leg*3) = get_foot_pos(leg);
        return pos_ref;
    }

    Eigen::VectorXd swing_traj(double t) {
        x_ref = B_.get_x_ref(t);
        x0 = B_.get_x0();

        pdot_ref << x_ref[9], x_ref[10], x_ref[11];
        pdot << x0[9], x0[10], x0[11];
        thdot << x0[6], x0[7], x0[8];
        stride = pdot_ref[0]*T_swing;

        for (int leg = 0; leg < 4; ++leg) {
            t_norm = t - T_total * gait_phase_delay[leg] + T_total;
            double phase = std::fmod(t_norm / T_total, 1.0);
            Eigen::Vector3d pos;

            if (phase < swing_ratio) {
                double s = phase / swing_ratio;
                is_contact[leg] = false;
                if (old_contact[leg]) {
                    stance_end_pos[leg] = get_foot_pos(leg);
                }

                Bz_points << stance_end_pos[leg][0], stance_end_pos[leg][1], stance_end_pos[leg][2],
                            -0.2*stride/2 + w*CP[0], w*CP[1], -0.2,
                            0.5*stride/2 + w*CP[0], w*CP[1], -0.25,
                            0.8*stride/2 + w*CP[0], w*CP[1], -0.34,
                            stride/2 + w*CP[0], w*CP[1], -0.36;

                CP = pdot - pdot_ref;
                CP[2] = 0;
                pos = Bezier->getBezierCurve(Bz_points, s);
            } else {
                double s = (phase - swing_ratio) / (1.0 - swing_ratio);
                is_contact[leg] = true;
                pos = get_foot_pos(leg);
            }
            old_contact[leg] = is_contact[leg];
            pos_ref.segment<3>(leg*3) = pos;
        }
        return pos_ref;
    }

    Eigen::VectorXd Traj(double t) {
        return (N_task == 1 ? swing_traj(t)
                            : (N_task == 0 ? Hold()
                                          : Eigen::VectorXd::Zero(12)));
    }

    Eigen::Vector3d get_foot_pos(int Leg_num) {
        return pino->get_leg_pos(Leg_num);
    }

    std::vector<bool> FSM() const {
        return is_contact;
    }
};

// Trajectory public 메서드 위임
Trajectory::Trajectory(std::shared_ptr<robot_parameter> p, Body &b)
  : pimpl_(std::make_unique<Impl>(std::move(p), b)) {}

Trajectory::~Trajectory() = default;

Eigen::VectorXd Trajectory::custom_leg_traj(double t) { return pimpl_->custom_leg_traj(t); }
Eigen::VectorXd Trajectory::Hold()             { return pimpl_->Hold(); }
Eigen::VectorXd Trajectory::swing_traj(double t) { return pimpl_->swing_traj(t); }
Eigen::VectorXd Trajectory::Traj(double t)      { return pimpl_->Traj(t); }
Eigen::Vector3d Trajectory::get_foot_pos(int Leg_num) { return pimpl_->get_foot_pos(Leg_num); }
std::vector<bool> Trajectory::FSM()            { return pimpl_->FSM(); }