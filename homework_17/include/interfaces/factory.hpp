#pragma once
#include <memory>

class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;

enum class SolverType   { ANALYTICAL, TABLE };
enum class ProviderType { JSON, THREAD_SAFE };
enum class LoaderType   { FILE };

class DroneSimulationFactory {
    public:
    std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const std::string& datafile);
    std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& targetsfile);
    std::unique_ptr<IConfigLoader> createLoader(LoaderType type, const std::string& configfile, const std::string& ammofile);
};
