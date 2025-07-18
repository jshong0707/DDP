#include "KF.hpp"
#include <iostream>

struct KalmanFilter::Impl {
    // Filter & Constants
    filter F;
    const Eigen::Vector3d g = Eigen::Vector3d(0, 0, -9.81);

    // Pinocchio
    pinocchio::Model model;
    pinocchio::Data data;

    // Frames
    const std::string base_frame = "imu_joint";
    const std::array<std::string, 4> foot_frames = {
        "FL_foot", "FR_foot", "RL_foot", "RR_foot"
    };

    // States
    Eigen::VectorXd x = Eigen::VectorXd::Zero(18); // [pb, vb, p1, p2, p3, p4]
    Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(18);

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(18, 18); // State transition
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(18, 3);      // Control (accel)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(18, 18) * 1e-3;

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(18, 18); // Covariance

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(24, 18);     // Measurement matrix
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(24, 24) * 1e-2;

    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(18, 18);     // Kalman gain

    // Measurement
    Eigen::VectorXd z = Eigen::VectorXd::Zero(24);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(18);

    // Joint state
    Eigen::VectorXd q = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd q_prev = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(12);

    // IMU
    Eigen::Vector3d f_tilde = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();

    // Rotation
    Eigen::Matrix3d omega_hat = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d R_WB = Eigen::Matrix3d::Identity();

    // relative foot pos and vel
    std::array<Eigen::Vector3d, 4> foot_pos_rel_;
    std::array<Eigen::Vector3d, 4> foot_vel_rel_;

    Impl() {
        x[2] = 0.375;

        // Load URDF model
        const std::string urdf_path = "../urdf/3D_Quad.urdf";
        pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model);
        data = pinocchio::Data(model);

        std::cout << "[KalmanFilter] Pinocchio model loaded. nq = " << model.nq << ", nv = " << model.nv << "\n";
    }

    void sensor_measure(const mjModel* m, mjData* d) {
        // 0. 회전 행렬 업데이트 (IMU 각속도 사용)
        omega_hat = F.skew(omega);
        R_WB = R_WB * (omega_hat * Ts).exp();  // R_k = R_{k-1} * exp(Ts * [omega]_x)
        
        double roll, pitch, yaw;

        // Check for gimbal lock (pitch = ±90 deg)
        if (std::abs(R_WB(2,0)) < 1.0 - 1e-6) {
            pitch = std::asin(-R_WB(2,0));
            roll  = std::atan2(R_WB(2,1), R_WB(2,2));
            yaw   = std::atan2(R_WB(1,0), R_WB(0,0));
        } else {
            // Gimbal lock: pitch = ±90 deg
            pitch = (R_WB(2,0) <= -1.0) ? M_PI/2 : -M_PI/2;
            roll  = 0.0;
            yaw   = std::atan2(-R_WB(0,1), R_WB(1,1));
        }
        //cout << "roll: " << roll << "pitch: " << pitch << "yaw: " << yaw << endl;

        // 1. IMU: accelerometer (f_tilde) and gyroscope (omega)
        f_tilde << d->sensordata[0], d->sensordata[1], d->sensordata[2];
        omega    << d->sensordata[3], d->sensordata[4], d->sensordata[5];

        // 2. Joint positions and velocities
        for (int i = 0; i < 12; ++i) {
            q[i] = d->qpos[7 + i];
            // Tustin 미분 필터를 사용해 dq 계산
            dq[i] = F.tustin_derivative(q[i], q_prev[i], dq[i], 100.0);  // 예: 100 Hz cutoff
        }
        q_prev = q;

        /// 3. Full model update for Pinocchio
        // full_q 구성
        Eigen::VectorXd full_q = Eigen::VectorXd::Zero(model.nq);
        full_q[0] = 0.0;  // x
        full_q[1] = 0.0;  // y
        full_q[2] = 0.0;  // z
        full_q[3] = 1.0;  // w
        full_q[4] = 0.0;  // x
        full_q[5] = 0.0;  // y
        full_q[6] = 0.0;  // z
        for (int i = 0; i < 12; ++i) {
            full_q[7 + i] = q[i];
        }

        // full_dq 구성
        Eigen::VectorXd full_dq = Eigen::VectorXd::Zero(model.nv);
        // base 속도는 0이므로 0~5는 그대로 유지
        for (int i = 0; i < 12; ++i) {
            full_dq[6 + i] = dq[i];
        }

        pinocchio::forwardKinematics(model, data, full_q, full_dq);
        pinocchio::updateFramePlacements(model, data);

        Eigen::Matrix3d R_pino_BW = data.oMf[model.getFrameId(base_frame)].rotation().transpose();

        for (int i = 0; i < 4; ++i) {
            const auto& oMf = data.oMf[model.getFrameId(foot_frames[i])];
            Eigen::Vector3d pos = oMf.translation();
            Eigen::Vector3d vel = pinocchio::getFrameVelocity(
                model, data, model.getFrameId(foot_frames[i]),
                pinocchio::LOCAL_WORLD_ALIGNED
            ).linear();

            // 상대 위치 및 속도 계산 (World → Body)
            Eigen::Vector3d foot_rel_pos = R_pino_BW * (pos - full_q.head<3>());
            Eigen::Vector3d foot_rel_vel = R_pino_BW * (vel - full_dq.head<3>());

            // 내부 변수에 저장
            foot_pos_rel_[i] = foot_rel_pos;
            foot_vel_rel_[i] = foot_rel_vel;

            // Kalman Filter용 z 벡터 구성
            z.segment<3>(i * 3) = foot_rel_pos;
            z.segment<3>(i * 3 + 12) = foot_rel_vel;
        }
    }

    void prediction_step() {
        // System matrix A (18x18)
        A.setIdentity();  // initialize diagonal blocks to I_3

        // base_pos → base_vel 연결 (row 0~2, col 3~5)
        A.block<3, 3>(0, 3) = Ts * Eigen::Matrix3d::Identity();
        // 나머지는 이미 대각선에 I_3가 있음

        // Control matrix B (18x3)
        B.setZero();
        B.block<3,3>(0, 0) = 0.5 * Ts * Ts * Eigen::Matrix3d::Identity();  // base_pos 영향
        B.block<3,3>(3, 0) = Ts * Eigen::Matrix3d::Identity();             // base_vel 영향
        // 나머지 (foot pos/vel) 블록은 0

        // x_k+1 = A*x_k + B*(R * f_tilde + g)
        x = A * x + B * (R_WB * f_tilde + g);
        P = A * P * A.transpose() + Q;
    }

    void update_step() {
        // Measurement matrix
        for (int i = 0; i < 4; ++i) {
            // 위치 측정: -I_3 @ base pos (0), I_3 @ foot_i pos
            C.block<3, 3>(i * 3, 0) = -Eigen::Matrix3d::Identity();
            C.block<3, 3>(i * 3, 6 + i * 3) = Eigen::Matrix3d::Identity();
            // 속도 측정: -I_3 @ base vel (3), I_3 @ foot_i vel
            C.block<3, 3>(12 + i * 3, 3) = -Eigen::Matrix3d::Identity();
        }
        
        // y = z - Cx
        y = z - C * x;
        // K = P*C^T * (C*P*C^T + R)^-1
        Eigen::MatrixXd S = C * P * C.transpose() + R;
        K = P * C.transpose() * S.inverse();
        // cout << K << endl;

        // x = x + K * y
        x = x + K * y;
        P = (Eigen::MatrixXd::Identity(18,18) - K * C) * P;
    }

    void setProcessNoise(const Eigen::MatrixXd& Q_in) {
        if (Q_in.rows() == 18 && Q_in.cols() == 18)
            Q = Q_in;
        else
            std::cout << "[KalmanFilter] Invalid Q size. Must be 18x18.\n" << std::endl;
    }

    void setMeasurementNoise(const Eigen::MatrixXd& R_in) {
        if (R_in.rows() == 24 && R_in.cols() == 24)
            R = R_in;
        else
            std::cout << "[KalmanFilter] Invalid R size. Must be 24x24.\n" << std::endl;
    }

    Eigen::VectorXd get_state() const {
        return x;
    }
};

// Public API
KalmanFilter::KalmanFilter() : pimpl_(std::make_unique<Impl>()) {}
KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::sensor_measure(const mjModel* m, mjData* d) {
    pimpl_->sensor_measure(m, d);
}

void KalmanFilter::prediction_step() {
    pimpl_->prediction_step();
}

void KalmanFilter::update_step() {
    pimpl_->update_step();
}

void KalmanFilter::setProcessNoise(const Eigen::MatrixXd& Q) {
    pimpl_->setProcessNoise(Q);
}

void KalmanFilter::setMeasurementNoise(const Eigen::MatrixXd& R) {
    pimpl_->setMeasurementNoise(R);
}

Eigen::VectorXd KalmanFilter::get_state() const {
    return pimpl_->get_state();
}
