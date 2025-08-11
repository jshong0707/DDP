#include "CostBuilder.hpp"

#include "ModelBuilder.hpp"
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>

// #include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>


struct CostBuilder::Impl{

    std::shared_ptr<ModelBuilder> ModelBuilder_;
    std::shared_ptr<crocoddyl::StateMultibody> state_;
    std::shared_ptr<crocoddyl::CostModelSum> costs;
    std::map<std::string, boost::shared_ptr<crocoddyl::CostItem>> costs_map;
    Eigen::VectorXd xref0;
    // Eigen::VectorXd xref;

    double nu;


    Impl(std::shared_ptr<ModelBuilder> MB)
        : ModelBuilder_(MB)
    {
        state_ = ModelBuilder_->get_state();
        nu = ModelBuilder_->get_nu();

        xref0 = state_->zero();
        costs = std::make_shared<crocoddyl::CostModelSum>(state_, nu);
        
    // State Weighting setup
        Eigen::VectorXd Wx = Eigen::VectorXd::Ones(state_->get_ndx());
        auto res_x = std::make_shared<crocoddyl::ResidualModelState>(state_, xref0, nu);
        auto W_x = std::make_shared<crocoddyl::ActivationModelWeightedQuad>(Wx);

        costs->addCost("State Error", std::make_shared<crocoddyl::CostModelResidual>(state_, W_x, res_x), 1.0);// xref(state_->get_nx());
    
    // Control Weighting setup
        Eigen::VectorXd Wu = Eigen::VectorXd::Ones(nu) * 1e-3;
        auto res_u = std::make_shared<crocoddyl::ResidualModelControl>(state_, nu);
        auto W_u = std::make_shared<crocoddyl::ActivationModelWeightedQuad>(Wu);
        costs->addCost("Input", std::make_shared<crocoddyl::CostModelResidual>(state_, W_u, res_u), 1.0);

        // auto act = std::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);
        // auto res_u = std::make_shared<crocoddyl::ResidualModelControl>(state_, );

    }

};




CostBuilder::CostBuilder(std::shared_ptr<ModelBuilder> ModelBuilder_)
    : pimpl_(std::make_unique<Impl>(ModelBuilder_))
{}

CostBuilder::~CostBuilder() = default;

std::shared_ptr<crocoddyl::CostModelSum> CostBuilder::get_costfunction()
{    
    auto costs_map = pimpl_->costs->get_costs();
    std::cout << " =============================== Cost function ===============================" << std::endl;
    std::cout << *(pimpl_->costs) << std::endl;
    return pimpl_->costs;
}
