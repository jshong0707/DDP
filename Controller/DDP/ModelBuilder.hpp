#pragma once

#include "globals.hpp"
#include <memory>
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>

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
    const pinocchio::FrameIndex get_frame_id(const std::string frame_name) const;
    /**
     * @brief Update contact references based on current foot positions
     * @param foot_positions at LOCAL_WORLD_ALIGNED frame
     */
    void set_contact_ref();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    
};

