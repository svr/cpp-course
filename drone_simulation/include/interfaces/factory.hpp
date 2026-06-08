#pragma once
#include <memory>

#include "ballistic_solver.hpp"
#include "config_loader.hpp"
#include "target_provider.hpp"

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

class DroneSimulationFactory {
    public:
    std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
    std::unique_ptr<ITargetProvider> createProvider(ProviderType type);
    std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
};
