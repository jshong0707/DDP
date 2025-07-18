#pragma once

#include "globals.hpp"
#include <memory>
#include <Eigen/Dense>
#include <vector>
#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/actions/lqr.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/states/euclidean.hpp>
#include <crocoddyl/core/utils/timer.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>



class DDP
{
public:
    DDP();
    ~DDP();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

