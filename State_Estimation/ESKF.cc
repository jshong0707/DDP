#include "ESKF.hpp"
#include <iostream>

namespace {
inline double wrapToPi(double angle) {
    double wrapped = std::remainder(angle, 2.0 * M_PI);
    if (wrapped <= -M_PI)    
        wrapped += 2.0 * M_PI;
    return wrapped;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(), v.x(),     0;
         return m;
}

Eigen::Matrix3d Gamma0(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d Omega = skew(omega);
    Eigen::Matrix3d Omega2 = Omega * Omega;
    return Eigen::Matrix3d::Identity()
         - 0.5 * Omega
         + (1.0 / 6.0) * Omega2;
}

Eigen::Matrix3d Gamma1(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d Omega = skew(omega);
    Eigen::Matrix3d Omega2 = Omega * Omega;
    return Eigen::Matrix3d::Identity()
         - (1.0 / 3.0) * Omega
         + (1.0 / 12.0) * Omega2;
}

Eigen::Matrix3d Gamma2(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d Omega = skew(omega);
    Eigen::Matrix3d Omega2 = Omega * Omega;
    return 0.5 * Eigen::Matrix3d::Identity()
         - (1.0 / 8.0) * Omega
         + (1.0 / 40.0) * Omega2;
}

Eigen::Matrix3d Gamma3(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d Omega = skew(omega);
    Eigen::Matrix3d Omega2 = Omega * Omega;
    return (1.0 / 6.0) * Eigen::Matrix3d::Identity()
         - (1.0 / 20.0) * Omega
         + (1.0 / 120.0) * Omega2;
}

Eigen::Quaterniond zeta(const Eigen::Vector3d& dphi) {
    double theta = dphi.norm();
    if (theta < 1e-8)
        return Eigen::Quaterniond(1, 0, 0, 0);

    Eigen::Vector3d axis = dphi / theta;
    double half_theta = 0.5 * theta;
    return Eigen::Quaterniond(std::cos(half_theta),
                              axis.x() * std::sin(half_theta),
                              axis.y() * std::sin(half_theta),
                              axis.z() * std::sin(half_theta));
}
}  // namespace

struct ErrorStateKalmanFilter::Impl {
    // Constants
    const Eigen::Vector3d g = Eigen::Vector3d(0, 0, 9.81);
    double Ts = 0.001;

    // Pinocchio
    pinocchio::Model model;
    pinocchio::Data data;
    const std::string base_frame = "imu_joint";
    const std::array<std::string, 4> foot_frames = { "FL_foot", "FR_foot", "RL_foot", "RR_foot" };

    // Filter helper
    filter F;

    // Nominal state
    Eigen::Vector3d p_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_b = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_b = Eigen::Quaterniond::Identity();
    Eigen::Vector3d euler_raw = Eigen::Vector3d::Zero();
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_w = Eigen::Vector3d::Zero();
    std::array<Eigen::Vector3d, 4> p_i;
 
    // Error state
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(27, 27) * 1e-1;
    Eigen::Matrix3d Q_f  = Eigen::Matrix3d::Identity() * 1e-2;   // accel noise
    Eigen::Matrix3d Q_w  = Eigen::Matrix3d::Identity() * 1e-2;   // gyro noise
    Eigen::Matrix3d Q_bf = Eigen::Matrix3d::Identity() * 1e-4;   // accel bias noise
    Eigen::Matrix3d Q_bw = Eigen::Matrix3d::Identity() * 1e-4;   // gyro bias noise
    Eigen::MatrixXd Q_p  = Eigen::MatrixXd::Identity(12, 12) * 1e-2;   // foot noise

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(12, 12) * 1e-3;
    Eigen::MatrixXd H, K;
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(27);

    // Input
    Eigen::Vector3d f_tilde = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_tilde = Eigen::Vector3d::Zero();

    // Measurement
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd h_x = Eigen::VectorXd::Zero(12);

    // Joint state (for FK)
    Eigen::VectorXd q = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd q_prev = Eigen::VectorXd::Zero(12);

    // Intermediate foot rel pos
    std::array<Eigen::Vector3d, 4> foot_pos_rel_;

    Impl() {
        const std::string urdf_path = "../urdf/3D_Quad.urdf";
        pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model);
        data = pinocchio::Data(model);
        std::cout << "[ESKF] Pinocchio model loaded. nq = " << model.nq << ", nv = " << model.nv << "\n";
        p_b = Eigen::Vector3d(0, 0, 0.3536);
        p_i[0] = Eigen::Vector3d( 0.2,  0.15, 0.0);
        p_i[1] = Eigen::Vector3d( 0.2, -0.15, 0.0);
        p_i[2] = Eigen::Vector3d(-0.2,  0.15, 0.0);
        p_i[3] = Eigen::Vector3d(-0.2, -0.15, 0.0);
    }

    void sensor_measure(const mjModel* m, mjData* d) {
        Eigen::Matrix3d Rwb = q_b.toRotationMatrix().transpose();

        f_tilde << d->sensordata[0], d->sensordata[1], d->sensordata[2];
        omega_tilde << d->sensordata[3], d->sensordata[4], d->sensordata[5];

        // Joint states
        for (int i = 0; i < 12; ++i) {
            q[i] = d->qpos[7 + i];
            dq[i] = F.tustin_derivative(q[i], q_prev[i], dq[i], 100.0);
        }
        q_prev = q;

        // Full state for Pinocchio
        Eigen::VectorXd full_q = Eigen::VectorXd::Zero(model.nq);
        full_q[0] = 0.0; full_q[1] = 0.0; full_q[2] = 0.0;  // base pos
        full_q[3] = 1.0; full_q[4] = 0.0; full_q[5] = 0.0; full_q[6] = 0.0;  // base rot wxyz
        for (int i = 0; i < 12; ++i) full_q[7 + i] = q[i];

        Eigen::VectorXd full_dq = Eigen::VectorXd::Zero(model.nv);
        for (int i = 0; i < 12; ++i) full_dq[6 + i] = dq[i];

        pinocchio::forwardKinematics(model, data, full_q, full_dq);
        pinocchio::updateFramePlacements(model, data);

        Eigen::Matrix3d R_pino_BW = data.oMf[model.getFrameId(base_frame)].rotation().transpose();

        for (int i = 0; i < 4; ++i) {
            const auto& oMf = data.oMf[model.getFrameId(foot_frames[i])];
            Eigen::Vector3d pos = oMf.translation();
            Eigen::Vector3d rel_pos = R_pino_BW * (pos - full_q.head<3>());

            foot_pos_rel_[i] = rel_pos;
            z.segment<3>(i * 3) = rel_pos;
        }
    }

    void prediction_step() {
        Eigen::Matrix3d Rwb = q_b.toRotationMatrix().transpose();
        Eigen::Vector3d f_hat = f_tilde - b_f;
        Eigen::Vector3d omega_hat = omega_tilde - b_w;

        // Gamma functions
        Eigen::Matrix3d G0 = Gamma0(omega_hat);
        Eigen::Matrix3d G1 = Gamma1(omega_hat);
        Eigen::Matrix3d G2 = Gamma2(omega_hat);
        Eigen::Matrix3d G3 = Gamma3(omega_hat);
        Eigen::Matrix3d G0T = G0.transpose();
        Eigen::Matrix3d G1T = G1.transpose();
        Eigen::Matrix3d G2T = G2.transpose();
        Eigen::Matrix3d G3T = G3.transpose();

        // Nominal state update
        p_b += Ts * v_b + 0.5 * Ts * Ts * (Rwb.transpose() * f_hat + g);
        v_b += Ts * (Rwb.transpose() * f_hat + g);
        q_b = (zeta(Ts * omega_hat) * q_b).normalized();

        // Construct A
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(27, 27);
        A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        A.block<3, 3>(0, 3) = Ts * Eigen::Matrix3d::Identity();
        A.block<3, 3>(0, 6) = - 0.5 * Ts * Ts * Rwb.transpose() * skew(f_hat);
        A.block<3, 3>(0, 21) = - 0.5 * Ts * Ts * Rwb.transpose();

        A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        A.block<3, 3>(3, 6) = - Ts * Rwb.transpose() * skew(f_hat);
        A.block<3, 3>(3, 21) = - Ts * Rwb.transpose();

        A.block<3, 3>(6, 6) = G0T;
        A.block<3, 3>(6, 24) = -G1T;

        for (int i=0; i<5; i++) {
            A.block<3, 3>(9 + 3 * i, 9 + 3 * i) = Eigen::Matrix3d::Identity();
        }

        // Build Q according to discretization
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(27, 27);
        Q.block<3,3>(0,0) = (1.0/3.0)*Ts*Ts*Ts*Q_f + (1.0/20.0)*Ts*Ts*Ts*Ts*Ts*Q_bf;
        Q.block<3,3>(0,3) = (1.0/2.0)*Ts*Ts*Q_f + (1.0/8.0)*Ts*Ts*Ts*Ts*Q_bf;
        Q.block<3,3>(0,21) = -(1.0/6.0)*Ts*Ts*Ts * Rwb.transpose() * Q_bf;

        Q.block<3,3>(3,0) = (1.0/2.0)*Ts*Ts*Q_f + (1.0/8.0)*Ts*Ts*Ts*Ts*Q_bf;
        Q.block<3,3>(3,3) = Ts*Q_f + (1.0/3.0)*Ts*Ts*Ts*Q_bf;
        Q.block<3,3>(3,21) = -(1.0/2.0)*Ts*Ts * Rwb.transpose() * Q_bf;

        Q.block<3,3>(6,6) = Ts*Q_w + (G3 + G3T) * Q_bw;
        Q.block<3,3>(6,24) = -G2T * Q_bw;

        Eigen::MatrixXd Qp1 = Q_p.block<3,3>(0,0);
        Eigen::MatrixXd Qp2 = Q_p.block<3,3>(3,3);
        Eigen::MatrixXd Qp3 = Q_p.block<3,3>(6,6);
        Eigen::MatrixXd Qp4 = Q_p.block<3,3>(9,9);

        Q.block<3,3>(9,9) = Ts * Rwb.transpose() * Qp1 * Rwb;
        Q.block<3,3>(12,12) = Ts * Rwb.transpose() * Qp2 * Rwb;
        Q.block<3,3>(15,15) = Ts * Rwb.transpose() * Qp3 * Rwb;
        Q.block<3,3>(18,18) = Ts * Rwb.transpose() * Qp4 * Rwb;

        Q.block<3,3>(21,0) = - (1.0/6.0)*Ts*Ts*Ts*Q_bf*Rwb;
        Q.block<3,3>(21,3) = - (1.0/2.0)*Ts*Ts*Q_bf*Rwb;
        Q.block<3,3>(21,21) = Ts * Q_bf;

        Q.block<3,3>(24,6) = - Q_bw * G2;
        Q.block<3,3>(24,24) = Ts * Q_bw;

        // Covariance update
        P = A * P * A.transpose() + Q;
    }

    void update_step() {
        H = Eigen::MatrixXd::Zero(12, 27);
        h_x = Eigen::VectorXd::Zero(12);
        Eigen::Matrix3d Rwb = q_b.toRotationMatrix().transpose();
        
        // H matrix and Measurement Error hx Calculation
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d pi_rel = p_i[i] - p_b;
            h_x.segment<3>(i * 3) = Rwb * pi_rel;

            H.block<3, 3>(i * 3, 0) = -Rwb;
            H.block<3, 3>(i * 3, 6) = Rwb * skew(Rwb * pi_rel);
            H.block<3, 3>(i * 3, 9 + 3 * i) = Rwb;
        }

        // Error State Variance Correction
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        K = P * H.transpose() * S.inverse();
        P = (Eigen::MatrixXd::Identity(27, 27) - K * H) * P;

        // Nominal state correction
        dx = K * (z - h_x);
        p_b += dx.segment<3>(0);
        v_b += dx.segment<3>(3);
        q_b = (zeta(dx.segment<3>(6)) * q_b).normalized();
        for (int i = 0; i < 4; ++i)
            p_i[i] += dx.segment<3>(9 + 3 * i);
        b_f += dx.segment<3>(21);
        b_w += dx.segment<3>(24);

        // Euler Angle making
        Eigen::Vector4d Q_b = Eigen::Vector4d(q_b.w(), q_b.x(), q_b.y(), q_b.z());
        euler_raw = F.quat2rpy(Q_b).reverse();
    }

    void setProcessAccelNoise(const Eigen::MatrixXd& Q_in) {
        if (Q_in.rows() == 3 && Q_in.cols() == 3)
            Q_f = Q_in;
        else
            std::cout << "[ErrorStateKalmanFilter] Invalid Q_f size. Must be 3x3.\n" << std::endl;
    }

    void setProcessAccelBiasNoise(const Eigen::MatrixXd& Q_in) {
        if (Q_in.rows() == 3 && Q_in.cols() == 3)
            Q_bf = Q_in;
        else
            std::cout << "[ErrorStateKalmanFilter] Invalid Q_bf size. Must be 3x3.\n" << std::endl;
    }

    void setProcessGyroNoise(const Eigen::MatrixXd& Q_in) {
        if (Q_in.rows() == 3 && Q_in.cols() == 3)
            Q_w = Q_in;
        else
            std::cout << "[ErrorStateKalmanFilter] Invalid Q_w size. Must be 3x3.\n" << std::endl;
    }

    void setProcessGyroBiasNoise(const Eigen::MatrixXd& Q_in) {
        if (Q_in.rows() == 3 && Q_in.cols() == 3)
            Q_bw = Q_in;
        else
            std::cout << "[ErrorStateKalmanFilter] Invalid Q_wf size. Must be 3x3.\n" << std::endl;
    }

    void setProcessFootNoise(const Eigen::MatrixXd& Q_in) {
        if (Q_in.rows() == 12 && Q_in.cols() == 12)
            Q_p = Q_in;
        else
            std::cout << "[ErrorStateKalmanFilter] Invalid Q_p size. Must be 12x12.\n" << std::endl;
    }

    void setMeasurementNoise(const Eigen::MatrixXd& R_in) {
        if (R_in.rows() == 12 && R_in.cols() == 12)
            R = R_in;
        else
            std::cout << "[ErrorStateKalmanFilter] Invalid R size. Must be 12x12.\n" << std::endl;
    }

    Eigen::VectorXd get_state() const {
        Eigen::VectorXd state(21);
        state.segment<3>(0) = p_b;
        state.segment<3>(3) = v_b;
        state.segment<3>(6) = euler_raw;
        for (int i = 0; i < 4; ++i)
            state.segment<3>(9 + 3 * i) = p_i[i];
        return state;
    }
};

// Public API
ErrorStateKalmanFilter::ErrorStateKalmanFilter() : pimpl_(std::make_unique<Impl>()) {}
ErrorStateKalmanFilter::~ErrorStateKalmanFilter() = default;
void ErrorStateKalmanFilter::sensor_measure(const mjModel* m, mjData* d) { pimpl_->sensor_measure(m, d); }
void ErrorStateKalmanFilter::prediction_step() { pimpl_->prediction_step(); }
void ErrorStateKalmanFilter::update_step() { pimpl_->update_step(); }
void ErrorStateKalmanFilter::setProcessAccelNoise(const Eigen::MatrixXd& Q) {
    pimpl_->setProcessAccelNoise(Q);
}
void ErrorStateKalmanFilter::setProcessAccelBiasNoise(const Eigen::MatrixXd& Q) {
    pimpl_->setProcessAccelBiasNoise(Q);
}
void ErrorStateKalmanFilter::setProcessGyroNoise(const Eigen::MatrixXd& Q) {
    pimpl_->setProcessGyroNoise(Q);
}
void ErrorStateKalmanFilter::setProcessGyroBiasNoise(const Eigen::MatrixXd& Q) {
    pimpl_->setProcessGyroBiasNoise(Q);
}
void ErrorStateKalmanFilter::setProcessFootNoise(const Eigen::MatrixXd& Q) {
    pimpl_->setProcessFootNoise(Q);
}
void ErrorStateKalmanFilter::setMeasurementNoise(const Eigen::MatrixXd& R) {
    pimpl_->setMeasurementNoise(R);
}
Eigen::VectorXd ErrorStateKalmanFilter::get_state() const { return pimpl_->get_state(); }
