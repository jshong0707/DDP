#include "FSM.hpp"

FSM::FSM(Trajectory &Traj)
:Traj(Traj)
{

}

FSM::~FSM()
{
    
}


vector<bool> FSM::contactschedule()
{
    is_contact = Traj.FSM();

    return is_contact;
}