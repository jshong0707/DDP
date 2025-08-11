#pragma once

#include "globals.hpp"
#include <memory>
#include <array>
#include <Eigen/Dense>

class CostBuilder;

class DDPSolver
{

public:
    DDPSolver(std::shared_ptr<CostBuilder> CostBuilder);
    ~DDPSolver();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
