#pragma once

#include "globals.hpp"
#include "Trajectory.hpp"

class Trajectory;

class FSM
{
private:
    Trajectory &Traj;

    vector<bool> is_contact = {true, true, true, true};


public:
    FSM(Trajectory &Traj);
    ~FSM();

    vector<bool> contactschedule();    
};

