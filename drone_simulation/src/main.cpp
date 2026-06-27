#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "factory.hpp"
#include "mission_processor.hpp"
#include "thread_safe_target_provider.hpp"
#include "drone_physics.hpp"

int main() {
    DroneSimulationFactory factory;
    auto solver = factory.createSolver(SolverType::ANALYTICAL, "");
    auto loader = factory.createLoader(LoaderType::FILE, "config.json", "ammo.json");

    loader->load();
    DroneConfig config = loader->getConfig();

    float physicsTimeStep = config.physicsTimeStep;
    float timeScale = config.timeScale;
    float simTimeStep = config.simTimeStep;

    std::shared_ptr<ITargetProvider> baseProvider = factory.createProvider(ProviderType::THREAD_SAFE, "targets.json");

    auto providerShared = std::dynamic_pointer_cast<ThreadSafeTargetProvider>(baseProvider);
    if (!providerShared) {
        std::cerr << "Critical Error: Provider is not ThreadSafeTargetProvider!\n";
        return 1;
    }

    ThreadSafeTargetProvider* provider = providerShared.get();

    provider->init(config);
    provider->load();

    auto physics = std::make_shared<DronePhysics>();
    physics->init(config, physicsTimeStep, timeScale, std::move(solver), providerShared, std::move(loader));

    auto mission = std::make_unique<MissionProcessor>(physics);
    mission->init(config);

    std::thread providerThread(&ThreadSafeTargetProvider::run, providerShared);
    std::thread physicsThread(&DronePhysics::run, physics);
    std::thread missionThread(&MissionProcessor::run, mission.get());

    while (!providerShared->isThreadReady() || !physics->isThreadReady() || !mission->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    providerShared->start();
    physics->start();
    mission->start();

    missionThread.join();

    physics->stop();
    providerShared->stop();

    physicsThread.join();
    providerThread.join();

    json simulationResult = mission->getSimulationLog();

    std::ofstream simfstream("simulation.json");
    if (!simfstream) {
        std::cerr << "Error accessing simulation.json\n";
        return 1;
    }

    simfstream << simulationResult.dump(4);
    simfstream.close();

    return 0;
}