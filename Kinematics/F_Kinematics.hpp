#pragma once

#include "globals.hpp"
#include "Actuator.hpp"

using namespace Eigen;

class Trajectory;
class Actuator;

class F_Kinematics
{
private:
    Actuator &HAA;
    Actuator &HIP;
    Actuator &KNEE;

    int Leg_num;
    double L = 0.25;
    double L_hip = 0;

    Vector3d pos = Vector3d::Zero();
    Vector3d pos_err = Vector3d::Zero();
    Vector3d pos_err_old = Vector3d::Zero();

    Vector3d q = Vector3d::Zero();
    Vector3d qd = Vector3d::Zero();
    Vector3d qdd = Vector3d::Zero();

    Matrix3d Jacb;

public:
    F_Kinematics(Actuator &HAA, Actuator &HIP, Actuator &KNEE);
    ~F_Kinematics();
    
    void sensor_measure(const mjModel* m, mjData* d);
    void Cal_Kinematics();
    
    Vector3d get_pos(){return pos;}
    Vector3d get_error(Vector3d pos_ref);
    Vector3d get_err_old(){return pos_err_old;}
    Matrix3d get_Jacb() {return Jacb;};



       
 
};

