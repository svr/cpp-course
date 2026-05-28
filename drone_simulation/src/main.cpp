#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "factory.hpp"
#include "mission_processor.hpp"

#define ENABLE_LOG    1
#define ENABLE_DEBUG  1


int main() {
    DroneSimulationFactory factory;

    MissionProcessor mission(
        factory.createSolver(SolverType::ANALYTICAL),
        factory.createProvider(ProviderType::JSON),
        factory.createLoader(LoaderType::FILE)
    );

    try {
        mission.init("config.json", "ammo.json", "targets.json");
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    json simulationResult;
    simulationResult["steps"] = json::array();


    while (mission.hasNext()) {
        SimStep simStep = mission.step();

        json step;
        step["position"]        = {{"x", simStep.pos.x}, {"y", simStep.pos.y}};
        step["direction"]       = simStep.direction;
        step["state"]           = simStep.state;
        step["targetIndex"]     = simStep.targetIdx;
        step["dropPoint"]       = {{"x", simStep.dropPoint.x},       {"y", simStep.dropPoint.y}};
        step["aimPoint"]        = {{"x", simStep.aimPoint.x},        {"y", simStep.aimPoint.y}};
        step["predictedTarget"] = {{"x", simStep.predictedTarget.x}, {"y", simStep.predictedTarget.y}};

        simulationResult["steps"].push_back(step);
    }

    std::ofstream simfstream("simulation.json");
    if (!simfstream) {
        std::cerr << "Error accessing simulation.json\n";
        std::exit(1);
    }

    simulationResult["totalSteps"] = mission.getCurrentStep();
    simfstream << simulationResult;
    simfstream.close();
}