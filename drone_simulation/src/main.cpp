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
        factory.createSolver(SolverType::TABLE, "ballistic_table.txt"),
        factory.createProvider(ProviderType::JSON, "targets.json"),
        factory.createLoader(LoaderType::FILE, "config.json", "ammo.json")
    );

    try {
        mission.init();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    json simulationResult;
    simulationResult["steps"] = json::array();


    while (mission.hasNext()) {
        DroneContext ctx = mission.step();

        json step;
        step["position"]        = {{"x", ctx.pos.x}, {"y", ctx.pos.y}};
        step["direction"]       = ctx.direction;
        step["state"]           = mission.getStateId();
        step["targetIndex"]     = ctx.targetIdx;
        step["dropPoint"]       = {{"x", ctx.dropPoint.x},       {"y", ctx.dropPoint.y}};
        step["aimPoint"]        = {{"x", ctx.aimPoint.x},        {"y", ctx.aimPoint.y}};
        step["predictedTarget"] = {{"x", ctx.predictedTarget.x}, {"y", ctx.predictedTarget.y}};

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