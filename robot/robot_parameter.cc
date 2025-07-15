#include "robot_parameter.hpp"

robot_parameter::robot_parameter()
{
    const std::string urdf_filename = "../urdf/3D_Quad.urdf";
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);

    cout << "Pinocchio model loaded successfully!" << endl;
    cout << "Number of joints: " << model_.njoints << endl;
    cout << model_.nq << endl;

    legs[0] = {"FLHAA_point",   "FLHIP",   "FLKNEE",   "FL_foot"};
    legs[1] = {"FRHAA_point",   "FRHIP",   "FRKNEE",   "FR_foot"};
    legs[2] = {"RLHAA_point",   "RLHIP",   "RLKNEE",   "RL_foot"},
    legs[3] = {"RRHAA_point",   "RRHIP",   "RRKNEE",   "RR_foot"};
    
}


robot_parameter::~robot_parameter()
{
}


void robot_parameter::robot_param(VectorXd q, VectorXd qd, VectorXd qdd)
{
    string A = "FL_foot";

    forwardKinematics(model_, data_, q, qd);
    computeJointJacobians(model_, data_, q);
    updateFramePlacements(model_, data_);



    pinocchio::SE3 T_imu = data_.oMf[model_.getFrameId(Body)]; 
    
    
    Eigen::MatrixXd J = MatrixXd::Zero(6,18);


    int i = 0;
    for (const auto& foot_name : foot) 
    {
        pinocchio::SE3 T_foot = data_.oMf[model_.getFrameId(foot_name)];
        pinocchio::SE3 imutofoot;
        imutofoot = T_imu.inverse() * T_foot;  // IMU to foot at IMU frame
        r_W[i] = imutofoot.translation();
        // cout << "Vector from IMU to " << foot_name << ": " << r_W[i].transpose() << endl;
        
        i++;
    }




    Eigen::MatrixXd J_FL(6, model_.nv);
    Eigen::MatrixXd J_FR(6, model_.nv);
    Eigen::MatrixXd J_RL(6, model_.nv);
    Eigen::MatrixXd J_RR(6, model_.nv);

    // Jacobian, World frame
    getFrameJacobian(model_, data_, model_.getFrameId(legs[0][3]), LOCAL_WORLD_ALIGNED, J_FL);
    getFrameJacobian(model_, data_, model_.getFrameId(legs[1][3]), LOCAL_WORLD_ALIGNED, J_FR);
    getFrameJacobian(model_, data_, model_.getFrameId(legs[2][3]), LOCAL_WORLD_ALIGNED, J_RL);
    getFrameJacobian(model_, data_, model_.getFrameId(legs[3][3]), LOCAL_WORLD_ALIGNED, J_RR);

    R_BW = data_.oMf[model_.getFrameId(Body)].rotation(); // Body to World


    J_B[0] = R_BW.transpose() * J_FL.block(0,6,3,3);
    J_B[1] = R_BW.transpose() * J_FR.block(0,9,3,3);
    J_B[2] = R_BW.transpose() * J_RL.block(0,12,3,3);
    J_B[3] = R_BW.transpose() * J_RR.block(0,15,3,3);
    
    // pinocchio::Motion::Vector3 rpy = pinocchio::rpy::matrixToRpy(R_BW);
    Matrix3d R_WB = R_BW.transpose();
    
    Vector3d ypr = pinocchio::rpy::matrixToRpy(R_BW);
    Eigen::Quaterniond q_WB(R_BW.transpose());

    rpy = {ypr[2], -ypr[1], -ypr[0]};
    // rpy = ypr;
    for(int i = 0; i < 4; i ++)
    {
        T_W_HAA[i] = data_.oMf[model_.getFrameId(legs[i][0])];
        T_W_foot[i] = data_.oMf[model_.getFrameId(legs[i][3])];

        T_B[i] = T_W_HAA[i].inverse() * T_W_foot[i]; // HAA to foot at HAA frame
        
        leg_pos[i] = T_B[i].translation();
    }    


    // cout << "Dynamics \n" << pinocchio::rnea(model_, data_, q, qd, qdd) << endl;  // Dynamics joint torque들
    // cout << pinocchio::computeGeneralizedGravity(model_, data_, q) << endl; // Gravity term만 계산
    // cout << pinocchio::computeCoriolisMatrix(model_, data_, q, qd) << endl;
    // cout <<"C+G\n"<< pinocchio::nonLinearEffects(model_, data_, q, qd) << endl;  // C + G Joint torque
    // pinocchio::crba(model_, data_, q);  // pinocchio::crba 함수를 호출합니다. 이 함수는 model과 q를 사용하여 관성 행렬을 계산하고, 그 결과를 data 객체의 M 멤버에 저장합니다.
    
    // for(int i = 0; i < 4; i++)
    //     Leg_Inertia[i] = data_.M.block(6 + 3*i,6 + 3*i,3,3);

    
        // cout << "Inertia\n" << pinocchio::crba(model_, data_, q) << endl;
        
}       

