// #pragma once

// #include "globals.hpp"
// // #include "qpOASES.hpp"

// #include "stdlib.h"
// #include "numeric"
// #include "osqp/osqp.h"
// #include <Eigen/SparseCore>
// #include "MPC_util.hpp"
// #include "filter.hpp"
// #include "FSM.hpp"
// #include "Body.hpp"

// using namespace std;

// class Body;
// class F_Kinematics;
// class MPC_util;
// class FSM;
// class filter;

// class MPC
// {
// private:
//     Body &B_;
//     FSM& FSM_;
//     MPC_util *util;
//     filter *F;
    
//     /* VectorXd state */
//         VectorXd x0;
//         VectorXd u0;
        
//         VectorXd x_ref; // Consider horizon
//         VectorXd z_ref; 

//     /* Special Matrix */ 
//         Matrix3d I_3 = Matrix3d::Identity();
//         Matrix3d One_3 = Matrix3d::Ones();
//         Matrix3d Zero_3 = Matrix3d::Zero();
//         MatrixXd I_12;

//     /* Weighting */
//         MatrixXd Q_x;
//         MatrixXd R_u;
//         MatrixXd Q_f;
        
//     /* Dimension */
//         int horizon, N, nx, nu, nz, n_x, n_u, n_var, n_con;
//         double dt;

//     /* Dynamics */
//         Vector3d r_W[4];  
//         Vector3d CoM_pos_W = Vector3d::Zero();
//         Vector3d Ctrl_point_W = Vector3d::Zero();
//         Vector3d RPY = Vector3d::Zero();
//         Matrix3d R;
//         Matrix3d I;
//         Matrix3d I_inv;
//         double M;
//         double t;

//         MatrixXd Ad;
//         MatrixXd Bd;
//         VectorXd gd;
    
    
//     /* Linear Term */
//         VectorXd q_vec;       // q = –2·P·z_ref
//         vector<OSQPFloat> q_arr;

//     /* Cost function & Constraint */
//         // lb < Az << ub
//         MatrixXd A_constraint;
//         MatrixXd P;
//         VectorXd lb_constraint;       
//         VectorXd ub_constraint;       
//         VectorXd lb_fric;
//         VectorXd ub_fric;
        
//         // The number of Constraint
//         int nC_dyn;       
//         int nC_fricandFSM;
//         vector<int> nC_Rows;  // A number of Rows of A in Az = b
        
//         // Contact Schedule
//         MatrixXd Contact_schedule;
//         vector<bool> is_contact = {true, true, true, true};

//     /* Friction */
//         double mu = 0.7;    
//         double Fz_lb;
//         double Fz_max;

//     /* OSQP */
//         OSQPCscMatrix *P_csc_, *A_csc_;
//         OSQPSettings  *settings_;
//         OSQPSolver     *solver_;

//         // sparsity (index) structures
//         SparseMatrix<OSQPFloat, Eigen::ColMajor, OSQPInt> A_sparse;
//         OSQPFloat*  A_x_ptr;
//         OSQPInt*  A_i_ptr;
//         OSQPInt*  A_p_ptr;
//         OSQPInt   nnzA;

//         SparseMatrix<OSQPFloat, Eigen::ColMajor, OSQPInt> P_sparse;
//         OSQPFloat*  P_x_ptr;
//         OSQPInt*  P_i_ptr;
//         OSQPInt*  P_p_ptr;
//         OSQPInt   nnzP;

//     /* Bound */
//         vector<OSQPFloat> lb_arr;
//         vector<OSQPFloat> ub_arr;
    
//     /* Solution */
//         const OSQPFloat* sol;
//         VectorXd opt_u = VectorXd::Zero(12);



// public:
//     MPC(Body &B, FSM& FSM_);
//     ~MPC();
//     void MPC_init();
//     void OSQP_init();

//     void foot_vector(const mjModel* m, mjData* d);
//     void Dynamics();
    
//     void nzz_pointer();
//     void A_default();
//     void A_update(); // A_Constraint
//     void b_default();
//     void b_update();
//     void P_default();
//     void q_update();


//     void OSQP_q_b_update();
//     void OSQP_A_update();
    
//     void A_Dynamics_Constraint();
//     void b_Dynamics_Constraint();

//     void A_Friction_Contact_Constraint();
//     void b_Friction_Contact_Constraint();
//     void b_Friction_Contact_default();
    
    
//     void SolveQP();
    
//     double get_dt() {return dt;}
//     VectorXd get_opt_u() {return opt_u;}
//     Matrix3d get_R() {return R;}
    

     
// };
