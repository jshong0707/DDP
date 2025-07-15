// #pragma once

// #include "globals.hpp"
// #include "qpOASES.hpp"

// using namespace qpOASES;

// class QP
// {
// private:
//     filter* F;

    
//     real_t xOpt[2];
//     double W = 0.25;
//     Vector3d y = Vector3d::Zero();
//     double x;
//     double M = 40; 
//     double g = 9.81;
//     double L = 0.25;

//     VectorXd P = VectorXd::Zero(6);
//     VectorXd D = VectorXd::Zero(6);

//     VectorXd opt_grf;


//     /* Optimal GRF */
//         Vector3d opt_GRF[4];
        
//     // error_pos
//     double total_mass;
    
    



// public:
//     QP();
//     ~QP();

//     void optCentroid2(const mjModel* m, mjData* d, bool* Contact_signal);

//     Vector3d get_opt_GRF(int Leg_num){return opt_GRF[Leg_num];}

// };

