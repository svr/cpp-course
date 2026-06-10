#include <stdexcept>

#include "factory.hpp"
#include "analytical_solver.hpp"
#include "table_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"

class JsonTargetProvider;
class FileConfigLoader;

std::unique_ptr<IBallisticSolver> DroneSimulationFactory::createSolver(SolverType type, const std::string& datafile) {
    switch(type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
        case SolverType::TABLE:
            return std::make_unique<TableSolver, const std::string&>(datafile);
        default:
            throw std::invalid_argument("Unsupported SolverType");
    }
}

std::unique_ptr<ITargetProvider> DroneSimulationFactory::createProvider(ProviderType type, const std::string& targetsfile) {
    switch(type) {
        case ProviderType::JSON:
            return std::make_unique<JsonTargetProvider, const std::string&>(targetsfile);
        default:
            throw std::invalid_argument("Unsupported ProviderType");
    }
}

std::unique_ptr<IConfigLoader> DroneSimulationFactory::createLoader(LoaderType type, const std::string& configfile, const std::string& ammofile) {
    switch(type) {
        case LoaderType::FILE:
            return std::make_unique<FileConfigLoader, const std::string&, const std::string&>(configfile, ammofile);
        default:
            throw std::invalid_argument("Unsupported LoaderType");
    }
}
