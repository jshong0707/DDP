#include "Controller.hpp"
#include "F_Kinematics.hpp"
#include "B_Kinematics.hpp"
#include "filter.hpp"
#include <cmath>

struct Controller::Impl {
    // Original members
    filter* Filter;
    bool* Contact;
    double cutoff_freq;
    double tau;
    bool FB_mode;
    bool RWDOB_flag;
    bool Orientation_DOB_flag;

    Eigen::Vector3d P_term[4], P_term_old[4];
    Eigen::Vector3d I_term[4], I_term_old[4];
    Eigen::Vector3d D_term[4], D_term_old[4];
    Eigen::Vector3d PID_output[4];

    Eigen::Vector3d FL_ctrl_input, FR_ctrl_input;
    Eigen::Vector3d RL_ctrl_input, RR_ctrl_input;

    Eigen::Vector3d FL_Joint_input, FR_Joint_input;
    Eigen::Vector3d RL_Joint_input, RR_Joint_input;

    Eigen::Vector3d KP[4], KI[4], KD[4];
    Eigen::Vector4d pos_KP, pos_KD;

    Impl()
      : Filter(nullptr), Contact(nullptr),
        cutoff_freq(70),
        tau(1.0/(2.0*M_PI*70.0)),
        FB_mode(false),
        RWDOB_flag(true),
        Orientation_DOB_flag(true),
        
        FL_ctrl_input(Eigen::Vector3d::Zero()), FR_ctrl_input(Eigen::Vector3d::Zero()),
        RL_ctrl_input(Eigen::Vector3d::Zero()), RR_ctrl_input(Eigen::Vector3d::Zero()),
        
        FL_Joint_input(Eigen::Vector3d::Zero()), FR_Joint_input(Eigen::Vector3d::Zero()),
        RL_Joint_input(Eigen::Vector3d::Zero()), RR_Joint_input(Eigen::Vector3d::Zero()),

        pos_KP(1000,1000,1000,1000),
        pos_KD(20,20,20,20)

    {
        for(int i=0;i<4;++i){
            P_term[i].setZero(); I_term[i].setZero(); D_term[i].setZero();
            P_term_old[i].setZero(); I_term_old[i].setZero(); D_term_old[i].setZero();
            KP[i] << pos_KP[i], pos_KP[i], pos_KP[i];
            KI[i].setZero();
            KD[i] << pos_KD[i], pos_KD[i], pos_KD[i];
        }
    }

    Eigen::Vector3d FB_controller(const Eigen::Vector3d &error,
                                  const Eigen::Vector3d &error_old,
                                  int Leg_num)
    {
        tau = 1.0/(2.0*M_PI*cutoff_freq);
        P_term[Leg_num]  = KP[Leg_num].cwiseProduct(error);
     
        I_term[Leg_num]  = KI[Leg_num].cwiseProduct((Ts/2.0)*(error+error_old)) + I_term_old[Leg_num];
     
        D_term[Leg_num]  = 2.0*KD[Leg_num].cwiseProduct((1.0/(2.0*tau+Ts))*(error-error_old))
                          -((Ts-2.0*tau)/(2.0*tau+Ts))*D_term_old[Leg_num];
     
                          PID_output[Leg_num] = P_term[Leg_num] + I_term[Leg_num] + D_term[Leg_num];
        
        P_term_old[Leg_num] = P_term[Leg_num];
        I_term_old[Leg_num] = I_term[Leg_num];
        D_term_old[Leg_num] = D_term[Leg_num];
        
        return PID_output[Leg_num];
    }

    void Leg_controller(F_Kinematics &K_FL,
                        F_Kinematics &K_FR,
                        B_Kinematics &K_RL,
                        B_Kinematics &K_RR)
    {
        FL_Joint_input = K_FL.get_Jacb().transpose() * FL_ctrl_input;
        FR_Joint_input = K_FR.get_Jacb().transpose() * FR_ctrl_input;
        RL_Joint_input = K_RL.get_Jacb().transpose() * RL_ctrl_input;
        RR_Joint_input = K_RR.get_Jacb().transpose() * RR_ctrl_input;
    }
};

Controller::Controller()
  : pimpl_(std::make_unique<Impl>())
{
    // 필터 할당 등 필요 시 초기화
    pimpl_->Filter = new filter();
    // Contact 포인터 설정 등
}

Controller::~Controller()
{
    delete pimpl_->Filter;
}

Eigen::Vector3d Controller::FB_controller(const Eigen::Vector3d &e,
                                          const Eigen::Vector3d &eo,
                                          int leg)
{
    return pimpl_->FB_controller(e, eo, leg);
}

void Controller::Leg_controller(F_Kinematics &K_FL,
                                F_Kinematics &K_FR,
                                B_Kinematics &K_RL,
                                B_Kinematics &K_RR)
{
    pimpl_->Leg_controller(K_FL, K_FR, K_RL, K_RR);
}

bool Controller::get_FB_mode() const            { return pimpl_->FB_mode; }
Eigen::Vector3d Controller::get_FL_M_input() const { return pimpl_->FL_Joint_input; }
Eigen::Vector3d Controller::get_FR_M_input() const { return pimpl_->FR_Joint_input; }
Eigen::Vector3d Controller::get_RL_M_input() const { return pimpl_->RL_Joint_input; }
Eigen::Vector3d Controller::get_RR_M_input() const { return pimpl_->RR_Joint_input; }
