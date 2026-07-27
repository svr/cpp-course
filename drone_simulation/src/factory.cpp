#include <stdexcept>
#include <memory>
#include <variant>
#include "factory.hpp"
#include "analytical_solver.hpp"
#include "table_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"
#include "thread_safe_target_provider.hpp"
#include "uart_target_provider.hpp"
#include "uart_config_loader.hpp"

class JsonTargetProvider;
class FileConfigLoader;
class ThreadSafeTargetProvider;
class UartTargetProvider;

std::unique_ptr<IBallisticSolver> DroneSimulationFactory::createSolver(SolverType type, const std::string& datafile) {
    switch(type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
        case SolverType::TABLE:
            return std::make_unique<TableSolver>(datafile);
        default:
            throw std::invalid_argument("Unsupported SolverType");
    }
}

std::unique_ptr<ITargetProvider> DroneSimulationFactory::createProvider(ProviderType type, const ProviderParams& params) {
    switch(type) {
        case ProviderType::JSON:
        {
            JsonProviderParams jsonParams = std::get<JsonProviderParams>(params);
            return std::make_unique<JsonTargetProvider>(jsonParams.targetsfile);
        }
        case ProviderType::THREAD_SAFE:
        {
            JsonProviderParams tsParams = std::get<JsonProviderParams>(params);
            return std::make_unique<ThreadSafeTargetProvider>(tsParams.targetsfile);
        }
        case ProviderType::UART:
        {
            UartProviderParams uartParams = std::get<UartProviderParams>(params);
            return std::make_unique<UartTargetProvider>(uartParams.uartDataProvider, uartParams.targetcount, uartParams.timeStep);
        }
        default:
            throw std::invalid_argument("Unsupported ProviderType");
    }
}

std::unique_ptr<IConfigLoader> DroneSimulationFactory::createLoader(LoaderType type, const LoaderParams& params) {
    switch(type) {
        case LoaderType::FILE:
        {
            FileLoaderParams fileParams = std::get<FileLoaderParams>(params);
            return std::make_unique<FileConfigLoader>(fileParams.configfile, fileParams.ammofile);
        }
        case LoaderType::UART:
        {
            UartLoaderParams uartParams = std::get<UartLoaderParams>(params);
            return std::make_unique<UartConfigLoader>(uartParams.uartDataProvider);
        }
        default:
            throw std::invalid_argument("Unsupported LoaderType");
    }
}
