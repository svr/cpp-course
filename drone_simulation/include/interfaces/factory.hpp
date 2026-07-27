#pragma once
#include <memory>
#include <string>
#include <variant>
#include <memory>

class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;
class UART;
class UartDataProvider;

enum class SolverType   { ANALYTICAL, TABLE };
enum class ProviderType { JSON, THREAD_SAFE, UART };
enum class LoaderType   { FILE, UART };


struct FileLoaderParams {
    std::string configfile;
    std::string ammofile;
};

struct UartLoaderParams {
    std::shared_ptr<UartDataProvider> uartDataProvider;
};

struct JsonProviderParams {
    std::string targetsfile;
};

struct UartProviderParams {
    int targetcount;
    float timeStep;
    std::shared_ptr<UartDataProvider> uartDataProvider;
};

using LoaderParams = std::variant<FileLoaderParams, UartLoaderParams>;
using ProviderParams = std::variant<JsonProviderParams, UartProviderParams>;

class DroneSimulationFactory {
    public:
    std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const std::string& datafile);
    std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const ProviderParams& params);
    std::unique_ptr<IConfigLoader> createLoader(LoaderType type, const LoaderParams& params);
};
