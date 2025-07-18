#include "DDP.hpp"
#include "robot_parameter.hpp"
#include <array>

struct DDP::Impl {

    std::vector<std::shared_ptr<crocoddyl::CostModelAbstract>> Cost;
    
    const double dt = 0.001;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(36);
    Eigen::VectorXd state = Eigen::VectorXd::Zero(36);
    MatrixXd A = MatrixXd::Zero(36,36);

    // std::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics>
    // auto runningModel = std::make_shared<crocoddyl::ActionModelLQR>(state, A, B, Q, R, dt);

    // std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> runningModels(T, runningModels);


    // auto state = std::make_shared<crocoddyl::StateVector>(nx);
    // std::make_shared<crocoddyl::SolverFDDP>
    Impl()
    {        
        // Cost.push_back(std::make_shared<crocoddyl::CostModelSum>(state,nu));   
    } // = default;

    // auto state = std::make_shared<crocoddyl::>
      
};



DDP::DDP(): pimpl_(std::make_unique<Impl>())
{}

DDP::~DDP() = default;