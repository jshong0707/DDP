#include "CostBuilder.hpp"

#include "ModelBuilder.hpp"
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/residuals/contact-force.hpp>




struct CostBuilder::Impl{

    shared_ptr<ModelBuilder> modelbuilder_;
    shared_ptr<crocoddyl::StateMultibody> state_;
    shared_ptr<crocoddyl::CostModelSum> costs;
    map<string, boost::shared_ptr<crocoddyl::CostItem>> costs_map;
    Eigen::VectorXd xref0;
    // Eigen::VectorXd xref;
    shared_ptr<crocoddyl::ResidualModelState> res_x;
    shared_ptr<crocoddyl::ResidualModelControl> res_u;
    shared_ptr<crocoddyl::ResidualModelContactForce> res_lambda;
    
    double nu;
    double t;

    double fz_star = 17 * 9.81 / 4.0;
    pinocchio::Force Fref;
    

    Impl(shared_ptr<ModelBuilder> MB)
        : modelbuilder_(MB)
    {
        state_ = modelbuilder_->get_state();
        nu = modelbuilder_->get_nu();

        xref0 = state_->zero();
        xref0[2] = 0.36;

        Fref = pinocchio::Force::Zero();
        Fref.linear()[2] = fz_star;
        costs = make_shared<crocoddyl::CostModelSum>(state_, nu);
        
    //* State Weighting setup
        res_x = make_shared<crocoddyl::ResidualModelState>(state_, xref0, nu);
        Eigen::VectorXd Wx = Eigen::VectorXd::Ones(state_->get_ndx()) * 1;
        auto W_x = make_shared<crocoddyl::ActivationModelWeightedQuad>(Wx);

        costs->addCost("1. State Error", make_shared<crocoddyl::CostModelResidual>(state_, W_x, res_x), 1.0);// xref(state_->get_nx());
    
    //* Control Weighting setup
        res_u = make_shared<crocoddyl::ResidualModelControl>(state_, nu);
        Eigen::VectorXd Wu = Eigen::VectorXd::Ones(nu) * 1e-1; // 제어 입력 가중치 증가
        auto W_u = make_shared<crocoddyl::ActivationModelWeightedQuad>(Wu);

        costs->addCost("2. Input", make_shared<crocoddyl::CostModelResidual>(state_, W_u, res_u), 0.01);

    // //* Contact Force Weighting setup
        std::size_t nc = 3; // 3D contact forced
        auto res_lambda_FL = std::make_shared<crocoddyl::ResidualModelContactForce>(state_, modelbuilder_->get_frame_id("FL_foot"), Fref, nc, nu);
        auto res_lambda_FR = std::make_shared<crocoddyl::ResidualModelContactForce>(state_, modelbuilder_->get_frame_id("FR_foot"), Fref, nc, nu);
        auto res_lambda_RL = std::make_shared<crocoddyl::ResidualModelContactForce>(state_, modelbuilder_->get_frame_id("RL_foot"), Fref, nc, nu);
        auto res_lambda_RR = std::make_shared<crocoddyl::ResidualModelContactForce>(state_, modelbuilder_->get_frame_id("RR_foot"), Fref, nc, nu);
        Eigen::VectorXd Wlambda = Eigen::VectorXd::Ones(nc) * 100; // 제어 입력 가중치 증가
        auto W_lambda = make_shared<crocoddyl::ActivationModelWeightedQuad>(Wlambda);
        
        const std::size_t nr_FL = res_lambda_FL->get_nr();
        

        costs->addCost("3. FL_Contact_Force", std::make_shared<crocoddyl::CostModelResidual>(state_, W_lambda , res_lambda_FL), 0.01);
        costs->addCost("4. FR_Contact_Force", std::make_shared<crocoddyl::CostModelResidual>(state_, W_lambda , res_lambda_FR), 0.01);
        costs->addCost("5. RL_Contact_Force", std::make_shared<crocoddyl::CostModelResidual>(state_, W_lambda , res_lambda_RL), 0.01);
        costs->addCost("6. RR_Contact_Force", std::make_shared<crocoddyl::CostModelResidual>(state_, W_lambda , res_lambda_RR), 0.01);

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
    pimpl_->res_x->set_reference(new_xref);
    pimpl_->res_u->set_reference(Eigen::VectorXd::Ones(pimpl_->nu) * 1);
    
}

shared_ptr<crocoddyl::CostModelSum> CostBuilder::get_costfunction()
{    
    cout << " =============================== Cost function ===============================" << endl;
    cout << *(pimpl_->costs) << endl;
    return pimpl_->costs;
}
