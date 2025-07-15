#ifndef DATALOGGING_H_
#define DATALOGGING_H_

#include <cstdio>             // for FILE*

#include "globals.hpp"
#include "Integrate.hpp"

class dataLogging {
private:
    /***************** Data Logging *****************/
    FILE* fid_FL;
    FILE* fid_FR;
    FILE* fid_RL;
    FILE* fid_RR;
    FILE* fid_Body;

    int loop_index = 0;
    const int data_frequency = 100; // frequency at which data is written

    const char* datapath_FL    = "../data/data_FL.csv";
    const char* datapath_FR    = "../data/data_FR.csv";
    const char* datapath_RL    = "../data/data_RL.csv";
    const char* datapath_RR    = "../data/data_RR.csv";
    const char* datapath_Body = "../data/data_Body.csv";
    
    double t = 0.0;

    VectorXd Leg_pos_ref;
    Vector3d leg_pos_ref[4];
    VectorXd Leg_pos;
    Vector3d leg_pos[4];

    
    VectorXd x0 = VectorXd::Zero(12);
    VectorXd x_ref = VectorXd::Zero(12);
    Vector3d opt_GRF[4];

public:
    dataLogging();
    ~dataLogging();

    void cal_value(const mjModel* m, mjData* d, Integrate& I);
    void init_save_data_leg(FILE* fid);
    void save_data_leg(const mjModel* m, mjData* d, Integrate &I,
                       FILE* fid, int leg);
    void init_save_data_Body(FILE* fid);
    void save_data_Body(const mjModel* m, mjData* d, Body &B,
                         FILE* fid);
    void initiate();
    FILE* getFidFL()    const { return fid_FL; }
    FILE* getFidFR()    const { return fid_FR; }
    FILE* getFidRL()    const { return fid_RL; }
    FILE* getFidRR()    const { return fid_RR; }
    FILE* getFidBody() const { return fid_Body; }
};

#endif // DATALOGGING_H_
