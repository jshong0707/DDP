#include "B_Kinematics.hpp"

B_Kinematics::B_Kinematics(Actuator &SPINE, Actuator &HAA, Actuator &HIP, Actuator &KNEE)
:SPINE(SPINE), HAA(HAA), HIP(HIP), KNEE(KNEE)
{

    Jacb.resize(3,4);
    Jacb.setZero();
}

B_Kinematics::~B_Kinematics()
{
}



void B_Kinematics::Cal_Kinematics()
{
    


    // Foot position   x,y,z
    pos[0] = (-L * (cos(q[2]) + cos(q[2] + q[3])) - L_spine) * cos(q[0]);  
    pos[1] = L_hip * cos(q[1]) + L * (sin(q[2]) + sin(q[2] + q[3])) * sin(q[1]);
    pos[2] = (L * (sin(q[2]) + sin(q[2] + q[3])) * cos(q[1]) - L_hip * sin(q[1])) * cos(q[0]) + L_spine * sin(q[0]);

    double C2 = cos(q[2]), C23 = cos(q[2] + q[3]);
    double S2 = sin(q[2]), S23 = sin(q[2] + q[3]);

    double S = S2 + S23;
    double C = C2 + C23;

    double s0 = sin(q[0]), c0 = cos(q[0]);
    double s1 = sin(q[1]), c1 = cos(q[1]);

    // d/dq0
    Jacb(0,0) = (-L*(C) - L_spine)*c0;
    Jacb(1,0) = 0.0;
    Jacb(2,0) = -(L*(S)*c1 - L_hip*s1)*s0 + L_spine*c0;

    // d/dq1
    Jacb(0,1) = 0.0;
    Jacb(1,1) = -L_hip*s1 + L*(S)*c1;
    Jacb(2,1) = (-L*(S)*s1 - L_hip*c1)*c0;

    // d/dq2
    Jacb(0,2) = L*(S)*s0;
    Jacb(1,2) = L*(C)*s1;
    Jacb(2,2) = L*(C)*c1*c0;

    // d/dq3
    Jacb(0,3) = L*(S23)*s0;
    Jacb(1,3) = L*(C23)*s1;
    Jacb(2,3) = L*(C23)*c1*c0;

}

void B_Kinematics::sensor_measure(const mjModel* m, mjData* d)
{
    Vector3d SPINE_j;
    Vector3d HAA_j;
    Vector3d HIP_j;
    Vector3d KNEE_j;
    
    SPINE_j = SPINE.Receive_data(m, d);
    HAA_j = HAA.Receive_data(m, d);
    HIP_j = HIP.Receive_data(m, d);
    KNEE_j = KNEE.Receive_data(m, d);

    q << SPINE_j[0], HAA_j[0], HIP_j[0], KNEE_j[0];
    qd << SPINE_j[1], HAA_j[1], HIP_j[1], KNEE_j[1];
    qdd << SPINE_j[2], HAA_j[2], HIP_j[2], KNEE_j[2];

}


Vector3d B_Kinematics::get_error(Vector3d pos_ref)
{
    pos_err_old = pos_err;
    pos_err = pos_ref - pos;

    return pos_err;
}





