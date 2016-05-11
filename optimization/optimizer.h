#pragma once

#include <nlopt.hpp>

#include "../configuration.hpp"

class Optimizer
{
public:
    Optimizer();

    void optimize(Configuration &config);
};
