#include <DDPSolver.hpp>

#include "ModelBuilder.hpp"
#include "CostBuilder.hpp"

#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/multibody/data/contacts.hpp>


struct DDPSolver::Impl {

  // === members ===
  std::shared_ptr<ModelBuilder> modelbuilder_;
  std::shared_ptr<CostBuilder>  costbuilder_;

  double dt      = 0.01;
  int    Horizon = 10;

  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> models;
  std::shared_ptr<crocoddyl::ShootingProblem>                  problem;
  std::unique_ptr<crocoddyl::SolverDDP>                        solver;

  // 참고: contact_model은 ModelBuilder가 소유/관리 (사전 등록 + 토글 전제)
  std::shared_ptr<crocoddyl::ContactModelMultiple> contact_model;

  Eigen::VectorXd              x0;
  std::vector<Eigen::VectorXd> opt_u;  // size T
  std::vector<Eigen::VectorXd> opt_x;  // size T+1

  // === ctor ===
  Impl(std::shared_ptr<ModelBuilder> MB, std::shared_ptr<CostBuilder> CB)
  : modelbuilder_(MB),
    costbuilder_(CB),
    models(Horizon)
  {
    // 1) state / actuation / contacts (사전 등록이 되어 있다고 가정)
    auto state     = modelbuilder_->get_state();
    auto actuation = modelbuilder_->get_act();
    contact_model  = modelbuilder_->get_contact_model();


    // 2) costs
    auto costs = costbuilder_->get_costfunction();


    // 3) x0 (초기화)
    x0 = state->zero();
    x0[2] = 0.36; // 예시: 베이스 z

    // 4) Differential + Integrated 모델 구성
    //    KKT 안정화를 위한 damping, force 출력 활성화를 위한 enable_force
    const double damping      = 1e-12;  // 1e-10 ~ 1e-8 로 점진 튜닝 가능
    const bool   enable_force = true;

    auto diff    = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
                      state, actuation, contact_model, costs, damping, enable_force);
    auto running = std::make_shared<crocoddyl::IntegratedActionModelEuler>(diff, dt);
    auto terminal= std::make_shared<crocoddyl::IntegratedActionModelEuler>(diff, dt);

    for (int i = 0; i < Horizon - 1; ++i) models[i] = running;
    models[Horizon - 1] = terminal;

    // 5) Problem & Solver
    problem = std::make_shared<crocoddyl::ShootingProblem>(x0, models, terminal);
    solver  = std::make_unique<crocoddyl::SolverDDP>(problem);
     
    //! Callback Verbose
    // solver->setCallbacks({std::make_shared<crocoddyl::CallbackVerbose>()});


  }

    void getLambdaAt(std::size_t k) const {

    auto data_euler = std::dynamic_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
        problem->get_runningDatas()[k]);

    auto data_contact = std::dynamic_pointer_cast<
        crocoddyl::DifferentialActionDataContactFwdDynamics>(data_euler->differential);
    
    cout << data_contact->pinocchio.lambda_c << endl;

    }

  



  void solve() {
    // 최신 상태로 x0 갱신
    x0 << modelbuilder_->get_q(), modelbuilder_->get_qd();
    problem->set_x0(x0);
    
    //TODO 1. z방향 force의 부호가 서있는 상태를 유지하려면 모든 다리가 같아야하는데, 다름.
    //TODO 2. 최적화된 GRF가 너무 크게 나옴.
    //TODO 3. set_x0로 현재 state를 갱신해주고 있는데도 불구하고, 로봇을 마우스로 당겨도 cost가 변하지 않음. 
    
     //! Debugging 
        problem->set_x0(x0);

        solver->solve();

    opt_x = solver->get_xs();
    opt_u = solver->get_us();

    
    std::cout << "[DDP::solve] done (iter=" << solver->get_iter()
              << ", cost=" << solver->get_cost() << ")\n";
  }
};


DDPSolver::DDPSolver(std::shared_ptr<ModelBuilder> MB, std::shared_ptr<CostBuilder> CB)
: pimpl_(std::make_unique<Impl>(MB, CB)) {}

void DDPSolver::solve() { pimpl_->solve(); }
Eigen::VectorXd DDPSolver::get_x(int){ return pimpl_->opt_x[0]; }
Eigen::VectorXd DDPSolver::get_u(int){ return pimpl_->opt_u[0]; }
DDPSolver::~DDPSolver() = default;
