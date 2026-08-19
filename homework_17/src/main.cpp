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
#include "mavlink_transport_udp.hpp"
#include "mavlink_communication.hpp"

namespace {
	constexpr const char* mavlink_host = "127.0.0.1";
	constexpr uint16_t mavlink_port = 14550;
} 


int main() {
    auto transport = std::make_unique<MavlinkTransportUDP>(mavlink_host, mavlink_port);

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


    auto mavlink = std::make_shared<MavlinkCommunication>(
        std::move(transport)
    );

    auto mission = std::make_unique<MissionProcessor>(physics, mavlink);
    mission->init(config);

    std::thread mavlinkHeartbeatThread([mavlink]() { mavlink->run(); });
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

    mavlink->stop();
    mavlinkHeartbeatThread.join();

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
