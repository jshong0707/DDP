#pragma once

#include "globals.hpp"
#include <memory>
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/fwd.hpp>

class robot_parameter;



class ModelBuilder
{
public:
    ModelBuilder(std::shared_ptr<robot_parameter> pino);
    ~ModelBuilder();
    
    shared_ptr<crocoddyl::StateMultibody> get_state();
    shared_ptr<crocoddyl::ActuationModelFloatingBase> get_act();
    shared_ptr<crocoddyl::ContactModelMultiple> get_contact_model();
    const double get_nu();
    VectorXd get_q() const;
    VectorXd get_qd() const;


private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    
};

