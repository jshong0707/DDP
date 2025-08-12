#include "ModelBuilder.hpp"
#include "robot_parameter.hpp"


#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>



struct ModelBuilder::Impl {
    
    shared_ptr<robot_parameter> pino;
    shared_ptr<pinocchio::Model> model_;
    shared_ptr<pinocchio::Data>  data_;
    shared_ptr<crocoddyl::StateMultibody> state_;
    shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;
    shared_ptr<crocoddyl::ContactModelMultiple> contact_model_;
    double nu;
    
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
    }   
    
    
    void test() {        
            VectorXd x0 = Eigen::VectorXd::Zero(state_->get_nx());
            
        }

};

ModelBuilder::ModelBuilder(shared_ptr<robot_parameter> p)
  : pimpl_(make_unique<Impl>(p))
{}

ModelBuilder::~ModelBuilder() = default;

void ModelBuilder::test() { pimpl_->test(); }
shared_ptr<crocoddyl::StateMultibody> ModelBuilder::get_state() {return pimpl_->state_;}
shared_ptr<crocoddyl::ActuationModelFloatingBase> ModelBuilder::get_act() {return pimpl_->actuation_;}
shared_ptr<crocoddyl::ContactModelMultiple> ModelBuilder::get_contact_model() {return pimpl_->contact_model_;}
const double ModelBuilder::get_nu() {return pimpl_->nu;}
