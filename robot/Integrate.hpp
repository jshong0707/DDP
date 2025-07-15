#pragma once

#include "globals.hpp"
#include "Actuator.hpp"
#include "F_Kinematics.hpp"
#include "B_Kinematics.hpp"
#include "Controller.hpp"
#include "Body.hpp"
#include "MPC.hpp"
#include "FSM.hpp"
#include "robot_parameter.hpp"

class Controller;
class F_Kinematics;
class B_Kinematics;
class Trajectory;
class Body;
class MPC;
class robot_parameter;

class Integrate
{
private:
    /* Class declaration */
        robot_parameter &pino;
        Trajectory &Traj;
        Body &B;
        Controller &C;
        MPC &M;
        FSM &FSM_;
        // QP qp;

        double t;

    /* Joint State */
        VectorXd q = VectorXd::Zero(19);
        VectorXd qd = VectorXd::Zero(18);
        VectorXd qdd = VectorXd::Zero(18);
        
    /* FeedBack input*/
        Vector3d FB_input[4];

    /* Joint input*/
        Vector3d F_Joint_input[2];
        Vector3d B_Joint_input[2];

    /* Optimized Value */
        double optimization_t = 0;
        VectorXd opt_u;  
        Vector3d opt_GRF[4];
        Vector3d opt_tau[4];
        bool Contact[4] = {1, 1, 1, 1};
        bool Contact_traj[4];
    
    /* MPC */
        double mpc_dt;
        
    /* Leg State */
        VectorXd leg_pos;
        
        /* Trajectory */
        VectorXd leg_pos_ref;

        
        Vector3d pos_err[4];
        Vector3d pos_err_old[4];

    /* Contact Schedule */
        vector<bool> is_contact = {false, false, false, false};

public:
    Integrate(robot_parameter &pino, Trajectory &Traj, Body &B, MPC &M, Controller &C, FSM &FSM_);
    ~Integrate();
    void sensor_measure(const mjModel* m, mjData* d);
    void get_error(double t);
    void Leg_controller();
    void Data_log();


    Vector3d get_FL_J_input() {return F_Joint_input[0];}
    Vector3d get_FR_J_input() {return F_Joint_input[1];}
    Vector3d get_RL_J_input() {return B_Joint_input[0];}
    Vector3d get_RR_J_input() {return B_Joint_input[1];}
    
    
    
    /* Data Logging */
    VectorXd get_leg_pos_ref() {return leg_pos_ref;}
    VectorXd get_leg_pos() {return leg_pos;}

    VectorXd get_x0() {return B.get_x0();}
    VectorXd get_x_ref() {return B.get_x_ref(t);}
    
    VectorXd get_opt_GRF(int leg) {return opt_GRF[leg];}
    // Vector3d get_leg_pos_ref(int Leg_num) {return leg_pos_ref[Leg_num];}
    // Vector3d get_leg_vel_ref(int Leg_num) {return leg_vel_ref[Leg_num];}
    // Vector3d get_leg_pos(int Leg_num) {return leg_pos[Leg_num];}
    // Vector3d get_leg_vel(int Leg_num) {return leg_vel[Leg_num];}
};
