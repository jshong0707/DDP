#include "CostBuilder.hpp"

#include "ModelBuilder.hpp"
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>

// #include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>


struct CostBuilder::Impl{

    shared_ptr<ModelBuilder> ModelBuilder_;
    shared_ptr<crocoddyl::StateMultibody> state_;
    shared_ptr<crocoddyl::CostModelSum> costs;
    map<string, boost::shared_ptr<crocoddyl::CostItem>> costs_map;
    Eigen::VectorXd xref0;
    // Eigen::VectorXd xref;
    shared_ptr<crocoddyl::ResidualModelState> res_x;
    double nu;
    double t;

    Impl(shared_ptr<ModelBuilder> MB)
        : ModelBuilder_(MB)
    {
        state_ = ModelBuilder_->get_state();
        nu = ModelBuilder_->get_nu();

        xref0 = state_->zero();
        xref0[2] = 0.36;

        costs = make_shared<crocoddyl::CostModelSum>(state_, nu);
        
    // State Weighting setup
        Eigen::VectorXd Wx = Eigen::VectorXd::Ones(state_->get_ndx());
        res_x = make_shared<crocoddyl::ResidualModelState>(state_, xref0, nu);
        auto W_x = make_shared<crocoddyl::ActivationModelWeightedQuad>(Wx);

        costs->addCost("State Error", make_shared<crocoddyl::CostModelResidual>(state_, W_x, res_x), 1.0);// xref(state_->get_nx());
    
    // Control Weighting setup
        Eigen::VectorXd Wu = Eigen::VectorXd::Ones(nu) * 1e-1; // 제어 입력 가중치 증가
        auto res_u = make_shared<crocoddyl::ResidualModelControl>(state_, nu);
        auto W_u = make_shared<crocoddyl::ActivationModelWeightedQuad>(Wu);

        costs->addCost("Input", make_shared<crocoddyl::CostModelResidual>(state_, W_u, res_u), 0.01);

        // auto act = make_shared<crocoddyl::ActuationModelFloatingBase>(state_);
        // auto res_u = make_shared<crocoddyl::ResidualModelControl>(state_, );

    }



};




CostBuilder::CostBuilder(shared_ptr<ModelBuilder> ModelBuilder_)
    : pimpl_(make_unique<Impl>(ModelBuilder_))
{}

CostBuilder::~CostBuilder() = default;

void CostBuilder::get_t(double t)
{
    pimpl_->t = t;

    pimpl_->xref0[2] = 0.1 * pimpl_->t;
    // cout << pimpl_->t << endl;
    VectorXd new_xref = pimpl_->xref0;
    // pimpl_->res_x->set_reference(new_xref);

}

shared_ptr<crocoddyl::CostModelSum> CostBuilder::get_costfunction()
{    
    cout << " =============================== Cost function ===============================" << endl;
    cout << *(pimpl_->costs) << endl;
    return pimpl_->costs;
}
