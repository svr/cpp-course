#pragma once
#include <memory>

#include "ballistic_solver.hpp"
#include "config_loader.hpp"
#include "target_provider.hpp"

enum class SolverType   { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

class DroneSimulationFactory {
    public:
    std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const std::string& datafile);
    std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& targetsfile);
    std::unique_ptr<IConfigLoader> createLoader(LoaderType type, const std::string& configfile, const std::string& ammofile);
};
