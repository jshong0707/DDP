#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <Eigen/Dense>
#include <Eigen/Core>



struct controlparams {
  bool RWDOB_flag = false;
  bool Orientation_DOB_flag = false;
  bool gravity_flag = false;
  bool Admittance_flag = false;
  double RWDOB_cutoff = 10;
  double Orientation_DOB_cutoff = 5;
  int FB_mode = 2;   //0: pos,
                     // 1: vel, 
                     // 2: r vel th pos
                     // 3: r vel만 feedback. 
  Eigen::Vector3d ref_trunk_vel = Eigen::Vector3d::Zero();

};

extern struct controlparams CP;