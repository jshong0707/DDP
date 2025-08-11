#include <DDPSolver.hpp>
#include "CostBuilder.hpp"

#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>


struct DDPSolver::Impl {

    std::shared_ptr<CostBuilder> CostBuilder_;


    Impl(std::shared_ptr<CostBuilder> CB)
        : CostBuilder_(CB)
    {
        // Initialize the DDP solver with the cost model
        auto costs = CostBuilder_->get_costfunction(); 
        
        // auto diff = std::make_shared<crocoddyl::DifferentialActionModel(costs);
    };


    // Implementation details for Solver
};


DDPSolver::DDPSolver(std::shared_ptr<CostBuilder> CB)
    : pimpl_(std::make_unique<Impl>(CB))
{}

DDPSolver::~DDPSolver() = default;
