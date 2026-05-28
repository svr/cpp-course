#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "common.hpp"
#include "json_target_provider.hpp"

JsonTargetProvider::~JsonTargetProvider() {
    for (int i = 0; i < targetCount; i++) {
        delete[] targets[i];
    }
    delete[] targets;
}

void JsonTargetProvider::load(const char* file) {
    std::ifstream targetsfs(file);
    if (!targetsfs) {
        throw std::runtime_error("Failed to open targets file");
    }

    json targetsJson;
    targetsfs >> targetsJson;
    targetsfs.close();

    targetCount = targetsJson["targetCount"];
    timeSteps = targetsJson["timeSteps"];

    DEBUG(targetCount);
    DEBUG(timeSteps);

    targets = new Coord* [targetCount];
    for (int i = 0; i < targetCount; i++) {
        targets[i] = new Coord[timeSteps];
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = targetsJson["targets"][i]["positions"][j]["x"];
            targets[i][j].y = targetsJson["targets"][i]["positions"][j]["y"];
        }
    }
}

int JsonTargetProvider::getTargetCount() const {
    return targetCount;
}

int JsonTargetProvider::getTargetTimeSteps() const {
    return timeSteps;
}

Coord JsonTargetProvider::getTarget(int num, int timeIndex) const {
    if (num > targetCount) {
        throw std::out_of_range("num exceeds target count");
    }
    if (timeIndex > timeSteps) {
        throw std::out_of_range("index exceeds target time steps");
    }
    return targets[num][timeIndex];
}
