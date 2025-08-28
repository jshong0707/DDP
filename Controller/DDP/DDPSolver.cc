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

  int nu;
  int nx;
  
  std::shared_ptr<crocoddyl::ContactModelMultiple> contact_model;

  Eigen::VectorXd              x0;
  std::vector<Eigen::VectorXd> opt_u;  // size T
  std::vector<Eigen::VectorXd> opt_x;  // size T+1

  std::vector<Eigen::VectorXd> prev_xs_;  
  std::vector<Eigen::VectorXd> prev_us_;  

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
    nu = actuation->get_nu();
    nx = state->get_nx();
    x0 = state->zero();

    x0[2] = 0.36; // 예시: 베이스 z

    prev_xs_.assign(Horizon + 1, x0);
    prev_us_.resize(Horizon,   Eigen::VectorXd::Zero(nu));

    // 4) Differential + Integrated 모델 구성
    const double damping      = 1e-12;  // Mass Matrix의 역행렬 풀 때 regularization term
    const bool   enable_force = true;

    auto diff    = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
                      state, actuation, contact_model, costs, damping, enable_force);

    std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running_models(Horizon-1);
    auto running  = std::make_shared<crocoddyl::IntegratedActionModelEuler>(diff, dt);
    auto terminal = std::make_shared<crocoddyl::IntegratedActionModelEuler>(diff, dt);
    for (int i=0;i<Horizon-1;++i) running_models[i] = running;

    // 5) Problem & Solver
    problem = std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal);
    solver  = std::make_unique<crocoddyl::SolverDDP>(problem);
     
    //! Callback Verbose
    // solver->setCallbacks({std::make_shared<crocoddyl::CallbackVerbose>()});


  }

    void getLambdaAt(std::size_t k) const {

    auto data_euler = std::dynamic_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
        problem->get_runningDatas()[k]);

    auto data_contact = std::dynamic_pointer_cast<
        crocoddyl::DifferentialActionDataContactFwdDynamics>(data_euler->differential);
    
    cout << "GRF\n" << data_contact->pinocchio.lambda_c << endl;
    // cout << "Acc\n" << data_contact->pinocchio.ddq << endl;
    
  }

  



  void solve() {
    // 최신 상태로 x0 갱신
    x0 << modelbuilder_->get_q(), modelbuilder_->get_qd();

     //! Debugging 

    //TODO 1. z방향 force의 부호가 서있는 상태를 유지하려면 모든 다리가 같아야하는데, 다름.
    //TODO 2. 최적화된 GRF가 너무 크게 나옴.
    //TODO 3. set_x0로 현재 state를 갱신해주고 있는데도 불구하고, 로봇을 마우스로 당겨도 cost가 변하지 않음. 
    //TODO 4. 결국 u와 lambda가 trade off 관계인거 같음. 근데 lambda가 cost에 안들어 가 있고, u만 들어가 있으니, u를 0으로 뱉고 lambda를 매우크게 뱉는듯
    //? contact 발의 위치를 계속 갱신해주면 lambda가 변함. 갱신 안하면 고정 값으로 매우크게 나옴
    const int T  = problem->get_T();  

    std::vector<Eigen::VectorXd> xs_init(T+1, Eigen::VectorXd::Zero(nx));
    std::vector<Eigen::VectorXd> us_init(T,   Eigen::VectorXd::Zero(nu));
    
    for (int t = 0; t < T-1; ++t) {
      us_init[t]   = prev_us_[t+1];    // u_{t}   <- u_{t+1}^{prev}
      xs_init[t+1] = prev_xs_[t+1];    // x_{t+1} <- x_{t+1}^{prev} (대충 seed)
    }
    us_init[T-1] = prev_us_.back();    // 마지막은 이전의 마지막으로 채움
    xs_init[T]   = prev_xs_.back();    // terminal도 이전의 마지막으로
    
    
    xs_init[0] = x0;

    modelbuilder_->set_contact_ref();

    problem->set_x0(x0);

    // solver->solve();
    solver->solve(xs_init, us_init, /*maxiter=*/50);
    getLambdaAt(0);

    opt_x = solver->get_xs();
    opt_u = solver->get_us();

    prev_xs_ = opt_x;
    prev_us_ = opt_u;
  
    //! Debugging 
     
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
