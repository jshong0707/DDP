#pragma once

#include <mujoco/mujoco.h>
// #include <Eigen/Core>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
// #include <eigen-master/unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <numeric>
    using namespace std;
    using namespace Eigen;

#include "filter.hpp"


#include "control_ui.hpp"
#include <iostream>



// #define NDOF_TRUNK 6 // #(DoF) of trunk
// #define NDOF_LEG 2   // #(DoF) of leg
// #define NUM_LEG 4

extern const double Ts; // sampling period
extern const double g;    // gravitational accel.
// #define PI 3.14159265358979323846264

