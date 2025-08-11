#include "ModelBuilder.hpp"
#include "robot_parameter.hpp"


// #include <crocoddyl/core/solvers/ddp.hpp>
// #include <crocoddyl/core/costs/cost-sum.hpp>
// #include <crocoddyl/core/costs/residual.hpp>
// #include <crocoddyl/core/integrator/euler.hpp>
// #include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
// #include <crocoddyl/multibody/actions/contact-fwddyn.hpp>    // 접촉동역학 쓸 때

// #include <crocoddyl/multibody/actuations/full.hpp>
// #include <crocoddyl/multibody/residuals/state.hpp>
// #include <crocoddyl/multibody/actions/free-fwddyn.hpp>
// #include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
// #include <crocoddyl/multibody/contacts/contact-3d.hpp>


struct ModelBuilder::Impl {
    
    std::shared_ptr<robot_parameter> pino;
    std::shared_ptr<pinocchio::Model> model_;
    std::shared_ptr<pinocchio::Data>  data_;
    // std::vector<std::string> foot_frame;
    const double nu;
    std::shared_ptr<crocoddyl::StateMultibody> state_;
    std::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;

    // std::shared_ptr<crocoddyl::ContactModelMultiple> contact_model_;
    // std::vector<std::shared_ptr<crocoddyl::CostModelAbstract>> allCosts;
    
    Impl(std::shared_ptr<robot_parameter> p)
      : pino(p),
        model_(p->getModel()),
        data_(p->getData()),
        nu(model_->nv)
    {
        // foot_frame = pino->get_foot_frame();

        state_ = std::make_shared<crocoddyl::StateMultibody>(model_);
        actuation_ = std::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);    
        
        // contact_model_ = std::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
        
        // Eigen::Vector2d gains; gains << 0., 0.;
        // for (auto const &frame_name : foot_frame) {
        //     const auto fid = model_->getFrameId(frame_name);

        //     // 1) Eigen → pinocchio::SE3 변환
        //     Eigen::Isometry3d jMf = Eigen::Isometry3d::Identity();
        //     pinocchio::SE3Tpl<double> pref(jMf.rotation(), jMf.translation());

        //     // 2) ContactModel6D 생성 (gains 오버로드)
        //     auto contact3d = std::make_shared<crocoddyl::ContactModel3D>(
        //         state_, fid, Vector3d::Zero(),
        //         pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, // ← 프레임 명시
        //         actuation_->get_nu(),                                             // ← 제어 차원 명시
        //         gains);

        //     // 3) Multiple-Contacts 에 추가
        //     contact_model_->addContact(frame_name, contact3d);
        // }
        
    }   
    

    void test() {        
            Eigen::VectorXd x0 = Eigen::VectorXd::Zero(state_->get_nx());
            
        }

};

ModelBuilder::ModelBuilder(std::shared_ptr<robot_parameter> p)
  : pimpl_(std::make_unique<Impl>(p))
{}

ModelBuilder::~ModelBuilder() = default;

void ModelBuilder::test() { pimpl_->test(); }
shared_ptr<crocoddyl::StateMultibody> ModelBuilder::get_state() {return pimpl_->state_;}
shared_ptr<crocoddyl::ActuationModelFloatingBase> ModelBuilder::get_act() {return pimpl_->actuation_;}
const double ModelBuilder::get_nu() {return pimpl_->nu;}