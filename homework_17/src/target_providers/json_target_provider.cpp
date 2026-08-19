#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "common.hpp"
#include "json_target_provider.hpp"

void JsonTargetProvider::load() {
    std::ifstream targetsfs(targetsfile);
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

    targets = std::vector(targetCount, std::vector<Coord>(timeSteps));
    for (int i = 0; i < targetCount; i++) {
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = targetsJson["targets"][i]["positions"][j]["x"];
            targets[i][j].y = targetsJson["targets"][i]["positions"][j]["y"];
        }
    }
}

int JsonTargetProvider::getTargetCount() const {
    return targetCount;
}

Target JsonTargetProvider::getTarget(int targetNum) const {
    if (targetNum < 0 || targetNum >= targetCount) {
        throw std::out_of_range("num exceeds target count");
    }

    Target target;
    if (timeSteps > 0) {
        target.pos = targets[targetNum][0];
    }
    target.velocity = {0.0f, 0.0f};
    return target;
}

int JsonTargetProvider::getTargetTimeSteps() const {
    return timeSteps;
}

Coord JsonTargetProvider::getTarget(int num, int timeIndex) const {
    if (num >= targetCount) {
        throw std::out_of_range("num exceeds target count");
    }
    if (timeIndex >= timeSteps) {
        throw std::out_of_range("index exceeds target time steps");
    }
    return targets[num][timeIndex];
}
