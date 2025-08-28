#include "ModelBuilder.hpp"
#include "robot_parameter.hpp"


#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/fwd.hpp> // pinocchio::LOCAL을 위해 추가됨
#include <crocoddyl/multibody/contacts/contact-3d.hpp>
// #include <crocoddyl/multibody/contacts/contact-6d.hpp>


struct ModelBuilder::Impl {
    
    shared_ptr<robot_parameter> pino;
    shared_ptr<pinocchio::Model> model_;
    shared_ptr<pinocchio::Data>  data_;
    shared_ptr<crocoddyl::StateMultibody> state_;
    shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;
    shared_ptr<crocoddyl::ContactModelMultiple> contact_model_;
    double nu;
    
    shared_ptr<crocoddyl::ContactModel3D> c_FL, c_FR, c_RL, c_RR;

    std::array<std::string,4> foot_frame_names = {
      "FL_foot", "FR_foot", "RL_foot", "RR_foot"
    };

    Impl(shared_ptr<robot_parameter> p)
      : pino(p),
        model_(p->getModel()),
        data_(p->getData())        
    {
      // foot_frame = pino->get_foot_frame();
      
      state_ = make_shared<crocoddyl::StateMultibody>(model_);
      actuation_ = make_shared<crocoddyl::ActuationModelFloatingBase>(state_);    
      nu = actuation_->get_nu();
      
      contact_model_ = make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());

      const auto rf = pinocchio::LOCAL_WORLD_ALIGNED;
      const Eigen::Vector2d gains(2.0, 1.0);    // 예시 게인
      const Eigen::Vector3d p0 = Eigen::Vector3d::Zero();

      // 프레임 ID 체크 (존재 안 하면 -1 주의)
      const pinocchio::FrameIndex fid_FL = model_->getFrameId(foot_frame_names[0]);
      const pinocchio::FrameIndex fid_FR = model_->getFrameId(foot_frame_names[1]);
      const pinocchio::FrameIndex fid_RL = model_->getFrameId(foot_frame_names[2]);
      const pinocchio::FrameIndex fid_RR = model_->getFrameId(foot_frame_names[3]);


      c_FL = make_shared<crocoddyl::ContactModel3D>(state_, fid_FL, p0, rf, (std::size_t)nu, gains);
      c_FR = make_shared<crocoddyl::ContactModel3D>(state_, fid_FR, p0, rf, (std::size_t)nu, gains);
      c_RL = make_shared<crocoddyl::ContactModel3D>(state_, fid_RL, p0, rf, (std::size_t)nu, gains);
      c_RR = make_shared<crocoddyl::ContactModel3D>(state_, fid_RR, p0, rf, (std::size_t)nu, gains);

      contact_model_->addContact("FL_contact", c_FL, /*active=*/true);
      contact_model_->addContact("FR_contact", c_FR, /*active=*/true);
      contact_model_->addContact("RL_contact", c_RL, /*active=*/true);
      contact_model_->addContact("RR_contact", c_RR, /*active=*/true);
      
      
      
    }   
    
    void set_contact_ref()
      {
        c_FL->set_reference(pino->get_leg_pos(0));
        c_FR->set_reference(pino->get_leg_pos(1));
        c_RL->set_reference(pino->get_leg_pos(2));
        c_RR->set_reference(pino->get_leg_pos(3));   
      }

};

ModelBuilder::ModelBuilder(shared_ptr<robot_parameter> p)
  : pimpl_(make_unique<Impl>(p))
{}

ModelBuilder::~ModelBuilder() = default;

shared_ptr<crocoddyl::StateMultibody> ModelBuilder::get_state() {return pimpl_->state_;}
shared_ptr<crocoddyl::ActuationModelFloatingBase> ModelBuilder::get_act() {return pimpl_->actuation_;}
shared_ptr<crocoddyl::ContactModelMultiple> ModelBuilder::get_contact_model() {return pimpl_->contact_model_;}
const double ModelBuilder::get_nu() {return pimpl_->nu;}
VectorXd ModelBuilder::get_q() const {return pimpl_->pino->get_q();}
VectorXd ModelBuilder::get_qd() const {return pimpl_->pino->get_qd();}
void ModelBuilder::set_contact_ref(){pimpl_->set_contact_ref();}
const pinocchio::FrameIndex ModelBuilder::get_frame_id(const std::string frame_name) const
{
  return pimpl_->model_->getFrameId(frame_name);
}