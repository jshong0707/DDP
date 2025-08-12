#include <DDPSolver.hpp>

#include "ModelBuilder.hpp"
#include "CostBuilder.hpp"

#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp> 
#include <crocoddyl/multibody/actuations/floating-base.hpp>
// #include <crocoddyl/multibody/residuals/state.hpp>

struct DDPSolver::Impl {

    shared_ptr<ModelBuilder> ModelBuilder_;
    shared_ptr<CostBuilder> CostBuilder_;
    double dt = 0.01;
    double Horizon = 10;    
    vector<shared_ptr<crocoddyl::ActionModelAbstract>> models;
    shared_ptr<crocoddyl::ShootingProblem> problem;
    unique_ptr<crocoddyl::SolverDDP> solver;
    // shared_ptr<crocoddyl::ShootingProblem> problem_;
    // crocoddyl::SolverDDP solver_;

    Impl(shared_ptr<ModelBuilder> MB, shared_ptr<CostBuilder> CB)
        : ModelBuilder_(MB), 
          CostBuilder_(CB),
          models(Horizon)
    {
        // Initialize the DDP solver with the cost model
        
        auto state = ModelBuilder_->get_state();
        auto actuation = ModelBuilder_->get_act();
        auto contact_model = ModelBuilder_->get_contact_model();
        
        VectorXd x0 = state->zero();
        x0[2] = 0.36; 

        
        auto costs = CostBuilder_->get_costfunction();
         
        auto diff = make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state, actuation, contact_model, costs);
        auto running = make_shared<crocoddyl::IntegratedActionModelEuler>(diff, dt);
        auto terminal = make_shared<crocoddyl::IntegratedActionModelEuler>(diff, dt);
        
        for(int i = 0; i < Horizon - 1; ++i)
        {   
            models[i] = running;
        }
        models[Horizon - 1] = terminal;

        problem = make_shared<crocoddyl::ShootingProblem>(x0, models, terminal);
        solver = std::make_unique<crocoddyl::SolverDDP>(problem); // 여기서 초기화


        // auto problem = make_shared<crocoddyl::ShootingProblem>(state, models, terminal);
        // auto solver = make_shared<crocoddyl::SolverDDP>(problem);   

    };

    void solve()
    {
        solver->solve();
        cout << solver->get_us()[0] << endl;
    }


    // Implementation details for Solver
};


DDPSolver::DDPSolver(shared_ptr<ModelBuilder> MB, shared_ptr<CostBuilder> CB)
    : pimpl_(make_unique<Impl>(MB, CB))
{}

void DDPSolver::solve()
{
    
    pimpl_->solve();
}

DDPSolver::~DDPSolver() = default;
