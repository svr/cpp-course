#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "factory.hpp"
#include "mission_processor.hpp"
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

    std::shared_ptr<ITargetProvider> provider = factory.createProvider(ProviderType::THREAD_SAFE, "targets.json");
    provider->init(config);
    provider->load();

    auto physics = std::make_shared<DronePhysics>();
    physics->init(config, physicsTimeStep, timeScale, std::move(solver), provider, std::move(loader));

    auto mission = std::make_unique<MissionProcessor>(physics);
    mission->init(config);

    std::thread providerThread([provider]() { provider->run(); });
    std::thread missionThread(&MissionProcessor::run, mission.get());

    while (!provider->isThreadReady() || !mission->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    provider->start();
    mission->start();

    missionThread.join();

    provider->stop();

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