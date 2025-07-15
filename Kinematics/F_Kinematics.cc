#include "F_Kinematics.hpp"

F_Kinematics::F_Kinematics(Actuator &HAA, Actuator &HIP, Actuator &KNEE)
: HAA(HAA), HIP(HIP), KNEE(KNEE)
{
}

F_Kinematics::~F_Kinematics()
{
}


void F_Kinematics::Cal_Kinematics()
{
    

    // Foot position   x,y,z
        pos[0] = -L * (cos(q[1]) + cos(q[1] + q[2]));  
        pos[1] = L_hip * cos(q[0]) + L * (sin(q[1]) + sin(q[1] + q[2])) * sin(q[0]);
        pos[2] = L * (sin(q[1]) + sin(q[1] + q[2])) * cos(q[0]) - L_hip * sin(q[0]);
    

    // Jacobian
        Jacb << 
        0,
        L * (sin(q[1]) + sin(q[1] + q[2])),
        L * sin(q[1] + q[2]),

        -L_hip * sin(q[0]) + L * cos(q[0]) * (sin(q[1]) + sin(q[1] + q[2])),
        L * sin(q[0]) * (cos(q[1]) + cos(q[1] + q[2])),
        L * sin(q[0]) * cos(q[1] + q[2]),

        -L_hip * cos(q[0]) - L * sin(q[0]) * (sin(q[1]) + sin(q[1] + q[2])),
        L * cos(q[0]) * (cos(q[1]) + cos(q[1] + q[2])),
        L * cos(q[0]) * cos(q[1] + q[2]);
}

void F_Kinematics::sensor_measure(const mjModel* m, mjData* d)
{
    Vector3d HAA_j;
    Vector3d HIP_j;
    Vector3d KNEE_j;
    
    HAA_j = HAA.Receive_data(m, d);
    HIP_j = HIP.Receive_data(m, d);
    KNEE_j = KNEE.Receive_data(m, d);

    q << HAA_j[0], HIP_j[0], KNEE_j[0];
    qd << HAA_j[1], HIP_j[1], KNEE_j[1];
    qdd << HAA_j[2], HIP_j[2], KNEE_j[2];

}


Vector3d F_Kinematics::get_error(Vector3d pos_ref)
{
    pos_err_old = pos_err;
    pos_err = pos_ref - pos;

    return pos_err;
}


