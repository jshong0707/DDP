#pragma once

#include "globals.hpp"
#include <memory>
#include <array>
#include <Eigen/Dense>

#include <crocoddyl/core/fwd.hpp>

class ModelBuilder;

class CostBuilder
{

public:
    CostBuilder(shared_ptr<ModelBuilder> ModelBuilder_);
    ~CostBuilder();
    
    shared_ptr<crocoddyl::CostModelSum> get_costfunction();
    void get_t(double t);

private:
    struct Impl;
    unique_ptr<Impl> pimpl_;
};