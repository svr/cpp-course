#include "factory.hpp"
#include "analytical_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"

class JsonTargetProvider;
class FileConfigLoader;

IBallisticSolver* DroneSimulationFactory::createSolver(SolverType type) {
    switch(type) {
        case SolverType::ANALYTICAL:
            return new AnalyticalSolver();
    }
}

ITargetProvider* DroneSimulationFactory::createProvider(ProviderType type) {
    switch(type) {
        case ProviderType::JSON:
            return new JsonTargetProvider();
    }
}

IConfigLoader* DroneSimulationFactory::createLoader(LoaderType type) {
    switch(type) {
        case LoaderType::FILE:
            return new FileConfigLoader();
    }
}
