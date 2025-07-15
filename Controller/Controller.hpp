#pragma once

#include <memory>
#include "globals.hpp"    // Vector3d, Vector4d, Ts 등 프로젝트 공용 타입 정의

// 전방 선언: 실제 정의는 .cpp에서 인클루드
class filter;
class F_Kinematics;
class B_Kinematics;

class Controller {
public:
    Controller();
    ~Controller();

    // 공개 API
    Eigen::Vector3d FB_controller(const Eigen::Vector3d &error,
                                  const Eigen::Vector3d &error_old,
                                  int Leg_num);

    void Leg_controller(F_Kinematics &K_FL,
                        F_Kinematics &K_FR,
                        B_Kinematics &K_RL,
                        B_Kinematics &K_RR);

    bool get_FB_mode()      const;
    Eigen::Vector3d get_FL_M_input() const;
    Eigen::Vector3d get_FR_M_input() const;
    Eigen::Vector3d get_RL_M_input() const;
    Eigen::Vector3d get_RR_M_input() const;

private:
    struct Impl;                       // Pimpl 전방 선언
    std::unique_ptr<Impl> pimpl_;      // 모든 멤버 변수·로직은 Impl 내부에 숨김
};