#include "factory.hpp"
#include "analytical_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"

class JsonTargetProvider;
class FileConfigLoader;

std::unique_ptr<IBallisticSolver> DroneSimulationFactory::createSolver(SolverType type) {
    switch(type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
    }
}

std::unique_ptr<ITargetProvider> DroneSimulationFactory::createProvider(ProviderType type) {
    switch(type) {
        case ProviderType::JSON:
            return std::make_unique<JsonTargetProvider>();
    }
}

std::unique_ptr<IConfigLoader> DroneSimulationFactory::createLoader(LoaderType type) {
    switch(type) {
        case LoaderType::FILE:
            return std::make_unique<FileConfigLoader>();
    }
}
