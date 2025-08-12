#pragma once

#include "globals.hpp"
#include <memory>
#include <array>
#include <Eigen/Dense>

class ModelBuilder;
class CostBuilder;

class DDPSolver
{

public:
    DDPSolver(shared_ptr<ModelBuilder> ModelBuilder, shared_ptr<CostBuilder> CostBuilder);
    ~DDPSolver();
    void solve();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
