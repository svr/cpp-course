#pragma once

#include "ballistic_solver.hpp"
#include "config_loader.hpp"
#include "target_provider.hpp"

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

class DroneSimulationFactory {
    public:
    IBallisticSolver* createSolver(SolverType type);
    ITargetProvider*  createProvider(ProviderType type);
    IConfigLoader*    createLoader(LoaderType type);
};
