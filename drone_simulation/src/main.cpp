#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "factory.hpp"
#include "mission_processor.hpp"
#include "drone_uart.hpp"

#include "factory.hpp"
#include "ballistic_solver.hpp"
#include "config_loader.hpp"
#include "gpio.hpp"
#include "parse_args.hpp"
#include "uart.hpp"
#include "uart_data_provider.hpp"

int main(int argc, char* argv[]) {
    const cli::CommandArgs args = cli::parse_args(argc, argv);

    auto uartDataProvider = std::make_shared<UartDataProvider>(args.uart);
    if(!uartDataProvider->isThreadReady()) {
        std::cerr << "Failed to create UART data provider\n";
        return 1;
    }
    std::thread uartDataProviderThread([uartDataProvider]() { uartDataProvider->run(); });
    while (!uartDataProvider->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    uartDataProvider->start();

    auto gpio = std::make_shared<GPIO>(args.gpiochip, static_cast<unsigned int>(args.start_line), static_cast<unsigned int>(args.drop_line));
    if (!gpio->isReady()) {
        std::cerr << "Failed to initialize GPIO chip/lines\n";
        return 1;
    }

    if (gpio->start() != 0) {
        std::cerr << "Failed to set START line active\n";
        return 1;
    }

    DroneSimulationFactory factory;
    auto solver = factory.createSolver(SolverType::TABLE, "ballistic_table.txt");
    solver->load();

    auto loader = factory.createLoader(LoaderType::UART, UartLoaderParams{ uartDataProvider });
    loader->load();

    DroneConfig config = loader->getConfig();
    AmmoParams ammo = loader->getAmmoParams();

    std::shared_ptr<ITargetProvider> provider = factory.createProvider(ProviderType::UART, UartProviderParams{ config.nTargets, config.simTimeStep / config.timeScale, uartDataProvider });
    provider->init(config);
    provider->load();

    auto physics = std::make_shared<DroneUart>();
    physics->init(std::move(solver), provider, std::move(loader), uartDataProvider);

    auto mission = std::make_unique<MissionProcessor>(physics, gpio);
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

    uartDataProvider->stop();
    uartDataProviderThread.join();

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