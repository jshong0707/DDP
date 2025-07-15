#pragma once

#include "globals.hpp"
#include "Actuator.hpp"

using namespace Eigen;

class Controller;
class Trajectory;
class Actuator;

class B_Kinematics
{
private:
    Actuator &SPINE;
    Actuator &HAA;
    Actuator &HIP;
    Actuator &KNEE;

    double L = 0.25;
    double L_hip = 0; 
    double L_spine = 0.1;


    Vector3d pos = Vector3d::Zero();
    Vector3d pos_err = Vector3d::Zero();
    Vector3d pos_err_old = Vector3d::Zero();
    

    Vector4d q = Vector4d::Zero();
    Vector4d qd = Vector4d::Zero();
    Vector4d qdd = Vector4d::Zero();

    MatrixXd Jacb;
    



public:
    B_Kinematics(Actuator &SPINE, Actuator &HAA, Actuator &HIP, Actuator &KNEE);
    ~B_Kinematics();

    void sensor_measure(const mjModel* m, mjData* d);    
    void Cal_Kinematics();

    Vector3d get_pos(){return pos;}
    Vector3d get_error(Vector3d pos_ref);
    Vector3d get_err_old(){return pos_err_old;}
    MatrixXd get_Jacb() {return Jacb;};

    

    

       
 
};

