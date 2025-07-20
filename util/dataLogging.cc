#include "dataLogging.hpp"

dataLogging::dataLogging(/* args */)
{
}

dataLogging::~dataLogging()
{
}

void dataLogging::cal_value(const mjModel* m, mjData* d, Integrate& I)
{
    I.Data_log();
    Leg_pos_ref = I.get_leg_pos_ref();
    Leg_pos = I.get_leg_pos();

    for(int leg = 0; leg < 4; leg++)
    {
        leg_pos_ref[leg] = Leg_pos_ref.segment(3*leg,3);
        leg_pos[leg] = Leg_pos.segment(3*leg,3);
        opt_GRF[leg] = I.get_opt_GRF(leg);
    }  
    x0 = I.get_x0();
    KF_x0_est = I.get_KF_x0_est();
    ESKF_x0_est = I.get_ESKF_x0_est();
    for (int i=0; i<4; i++) {
        W_foot_pos[i] = I.get_W_foot_pos(i);
    }
    x_ref = I.get_x_ref();
    
    
}

void dataLogging::initiate()
{    
    fid_FL = fopen(datapath_FL, "w");
    fid_FR = fopen(datapath_FR, "w");
    fid_RL = fopen(datapath_RL, "w");
    fid_RR = fopen(datapath_RR, "w");
    fid_Body = fopen(datapath_Body, "w");

    init_save_data_leg(fid_FL);    
    init_save_data_leg(fid_FR);
    init_save_data_leg(fid_RL);
    init_save_data_leg(fid_RR);
    init_save_data_Body(fid_Body);
}

void dataLogging::init_save_data_leg(FILE* fid)
{
    // This function is called once and is used to get the headers
    // Write name of the variable here (header)
    // comma(,) should be omitted in the last line.
    
    fprintf(fid, "t, ");
    //fprintf(fid, "x_pos_ref, y_pos_ref, z_pos_ref, x_pos, y_pos, z_pos, ");
    //fprintf(fid, "opt_ux, opt_uy, opt_uz");
    // fprintf(fid, "r_vel_ref, th_vel_ref, phi_vel_ref, r_vel_act, th_vel_act, phi_vel_act ");
    // Don't remove the newline
    fprintf(fid, "\n");
}

void dataLogging::save_data_leg(const mjModel* m, mjData* d, Integrate &I, FILE* fid, int leg)
{

    double t = d->time;
    fprintf(fid, "%f, ", d->time);
    
    //fprintf(fid, "%f, %f, %f, %f, %f, %f, ", leg_pos_ref[leg][0], leg_pos_ref[leg][1], leg_pos_ref[leg][2], leg_pos[leg][0], leg_pos[leg][1], leg_pos[leg][2]);
    //fprintf(fid, "%f, %f, %f ", opt_GRF[leg][0], opt_GRF[leg][1], opt_GRF[leg][2]);
    // fprintf(fid, "%f, %f, %f, %f, %f, %f, ", leg_pos_ref[leg], I.get_leg_vel_ref(leg)[0], I.get_leg_vel_ref(leg)[1], I.get_leg_vel(leg)[2], I.get_leg_vel(leg)[0], I.get_leg_vel(leg)[1]);

    // Don't remove the newline
    fprintf(fid, "\n");
}

void dataLogging::init_save_data_Body(FILE* fid)
{
    // This function is called once and is used to get the headers
    // Write name of the variable here (header)
    // comma(,) should be omitted in the last line.

    fprintf(fid, "t, ");
    //fprintf(fid, "Body_pos_roll, Body_pos_pitch, Body_pos_yaw, Body_vel_roll, Body_vel_pitch, Body_vel_yaw, ");    
    //fprintf(fid, "Body_pos_x, Body_pos_y, Body_pos_z, Body_vel_x, Body_vel_y, Body_vel_z,");
    //fprintf(fid, "Body_pos_roll_ref, Body_pos_pitch_ref, Body_pos_yaw_ref, Body_vel_roll_ref, Body_vel_pitch_ref, Body_vel_yaw_ref");
    //fprintf(fid, "Body_pos_x_ref, Body_pos_y_ref, Body_pos_z_ref, Body_vel_x_ref, Body_vel_y_ref, Body_vel_z_ref, ");

    fprintf(fid, "Body_pos_x, Body_pos_y, Body_pos_z, Body_vel_x, Body_vel_y, Body_vel_z, ");
    fprintf(fid, "Body_pos_x_KF_est, Body_pos_y_KF_est, Body_pos_z_KF_est, Body_vel_x_KF_est, Body_vel_y_KF_est, Body_vel_z_KF_est, ");
    fprintf(fid, "Body_pos_x_ESKF_est, Body_pos_y_ESKF_est, Body_pos_z_ESKF_est, Body_vel_x_ESKF_est, Body_vel_y_ESKF_est, Body_vel_z_ESKF_est, ");

    fprintf(fid, "Body_yaw, Body_pitch, Body_roll, ");
    fprintf(fid, "Body_yaw_KF_est, Body_pitch_KF_est, Body_roll_KF_est, ");
    fprintf(fid, "Body_yaw_ESKF_est, Body_pitch_ESKF_est, Body_roll_ESKF_est, ");
    
    fprintf(fid, "FL_foot_pos_x, FL_foot_pos_y, FL_foot_pos_z, ");
    fprintf(fid, "FL_foot_pos_x_KF_est, FL_foot_pos_y_KF_est, FL_foot_pos_z_KF_est, ");
    fprintf(fid, "FL_foot_pos_x_ESKF_est, FL_foot_pos_y_ESKF_est, FL_foot_pos_z_ESKF_est, ");

    fprintf(fid, "FR_foot_pos_x, FR_foot_pos_y, FR_foot_pos_z, ");
    fprintf(fid, "FR_foot_pos_x_KF_est, FR_foot_pos_y_KF_est, FR_foot_pos_z_KF_est, ");
    fprintf(fid, "FR_foot_pos_x_ESKF_est, FR_foot_pos_y_ESKF_est, FR_foot_pos_z_ESKF_est, ");

    fprintf(fid, "RL_foot_pos_x, RL_foot_pos_y, RL_foot_pos_z, ");
    fprintf(fid, "RL_foot_pos_x_KF_est, RL_foot_pos_y_KF_est, RL_foot_pos_z_KF_est, ");
    fprintf(fid, "RL_foot_pos_x_ESKF_est, RL_foot_pos_y_ESKF_est, RL_foot_pos_z_ESKF_est, ");

    fprintf(fid, "RR_foot_pos_x, RR_foot_pos_y, RR_foot_pos_z, ");
    fprintf(fid, "RR_foot_pos_x_KF_est, RR_foot_pos_y_KF_est, RR_foot_pos_z_KF_est, ");
    fprintf(fid, "RR_foot_pos_x_ESKF_est, RR_foot_pos_y_ESKF_est, RR_foot_pos_z_ESKF_est, ");
    
    // Don't remove the newline
    fprintf(fid, "\n");
}

void dataLogging::save_data_Body(const mjModel* m, mjData* d, Body &B, FILE* fid)
{
    // This function is called at a set frequency,put data here.
    // Data here should correspond to headers in init_save_data()
    // Seperate data by a space %f followed by space
    // comma(,) should be omitted in the last line.
    double t = d->time;

    fprintf(fid, "%f, ", d->time);
    //fprintf(fid, "%f, %f, %f, %f, %f, %f, ", x0[0], x0[1], x0[2], x0[3], x0[4], x0[5]);
    //fprintf(fid, "%f, %f, %f, %f, %f, %f, ", x0[6], x0[7], x0[8], x0[9], x0[10], x0[11]);
    //fprintf(fid, "%f, %f, %f, %f, %f, %f, ", x_ref[0], x_ref[1], x_ref[2], x_ref[3], x_ref[4], x_ref[5]);
    //fprintf(fid, "%f, %f, %f, %f, %f, %f ", x_ref[6], x_ref[7], x_ref[8], x_ref[9], x_ref[10], x_ref[11]);
    // fprintf(fid, "%f, %f, %f, ", B.get_vel_des(t)[0], B.get_vel_des(t)[1], B.get_vel_des(t)[2]);
    // fprintf(fid, "%f, %f, %f ", B.get_vel_des(t)[3], B.get_vel_des(t)[4], B.get_vel_des(t)[5]);
    
    fprintf(fid, "%f, %f, %f, %f, %f, %f, ", x0[3], x0[4], x0[5], x0[9], x0[10], x0[11]);
    fprintf(fid, "%f, %f, %f, %f, %f, %f, ", KF_x0_est[0], KF_x0_est[1], KF_x0_est[2], KF_x0_est[3], KF_x0_est[4], KF_x0_est[5]);
    fprintf(fid, "%f, %f, %f, %f, %f, %f, ", ESKF_x0_est[0], ESKF_x0_est[1], ESKF_x0_est[2], ESKF_x0_est[3], ESKF_x0_est[4], ESKF_x0_est[5]);

    fprintf(fid, "%f, %f, %f, ", x0[2], x0[1], x0[0]);
    fprintf(fid, "%f, %f, %f, ", KF_x0_est[6], KF_x0_est[7], KF_x0_est[8]);
    fprintf(fid, "%f, %f, %f, ", ESKF_x0_est[6], ESKF_x0_est[7], ESKF_x0_est[8]);
    
    fprintf(fid, "%f, %f, %f, ", W_foot_pos[0][0], W_foot_pos[0][1], W_foot_pos[0][2]);
    fprintf(fid, "%f, %f, %f, ", KF_x0_est[9], KF_x0_est[10], KF_x0_est[11]);
    fprintf(fid, "%f, %f, %f, ", ESKF_x0_est[9], ESKF_x0_est[10], ESKF_x0_est[11]);

    fprintf(fid, "%f, %f, %f, ", W_foot_pos[1][0], W_foot_pos[1][1], W_foot_pos[1][2]);
    fprintf(fid, "%f, %f, %f, ", KF_x0_est[12], KF_x0_est[13], KF_x0_est[14]);
    fprintf(fid, "%f, %f, %f, ", ESKF_x0_est[12], ESKF_x0_est[13], ESKF_x0_est[14]);

    fprintf(fid, "%f, %f, %f, ", W_foot_pos[2][0], W_foot_pos[2][1], W_foot_pos[2][2]);
    fprintf(fid, "%f, %f, %f, ", KF_x0_est[15], KF_x0_est[16], KF_x0_est[17]);
    fprintf(fid, "%f, %f, %f, ", ESKF_x0_est[15], ESKF_x0_est[16], ESKF_x0_est[17]);

    fprintf(fid, "%f, %f, %f, ", W_foot_pos[3][0], W_foot_pos[3][1], W_foot_pos[3][2]);
    fprintf(fid, "%f, %f, %f, ", KF_x0_est[18], KF_x0_est[19], KF_x0_est[20]);
    fprintf(fid, "%f, %f, %f, ", ESKF_x0_est[18], ESKF_x0_est[19], ESKF_x0_est[20]);

    // Don't remove the newline
    fprintf(fid, "\n");
}
