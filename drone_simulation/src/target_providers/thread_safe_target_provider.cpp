#include <fstream>
#include <thread>
#include <stdexcept>
#include <nlohmann/json.hpp>

#include "thread_safe_target_provider.hpp"
using json = nlohmann::json;

void ThreadSafeTargetProvider::load() {
    std::ifstream targetsfs(targetsfile);
    if (!targetsfs) {
        throw std::runtime_error("Failed to open targets file: " + targetsfile);
    }

    json targetsJson;
    targetsfs >> targetsJson;
    targetsfs.close();

    targetCount = targetsJson["targetCount"];
    timeSteps = targetsJson["timeSteps"];

    targets = std::vector(targetCount, std::vector<Coord>(timeSteps));

    {
        std::lock_guard<std::mutex> lock(providerMutex);
        currentTargets.resize(targetCount);

        for (int i = 0; i < targetCount; i++) {
            for (int j = 0; j < timeSteps; j++) {
                targets[i][j].x = targetsJson["targets"][i]["positions"][j]["x"];
                targets[i][j].y = targetsJson["targets"][i]["positions"][j]["y"];
            }

            if (timeSteps > 0) {
                currentTargets[i].pos = targets[i][0];
                currentTargets[i].velocity = {0.0f, 0.0f};
            }
        }
    }

    isReady = true;
}

void ThreadSafeTargetProvider::run() {
    while (!isStarted && !shouldStop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    while (!shouldStop && currentIdx < timeSteps) {
        {
            std::lock_guard<std::mutex> lock(providerMutex);
            for (int i = 0; i < targetCount; ++i) {
                if (currentIdx < (int)targets[i].size()) {
                    Coord nextPos = targets[i][currentIdx];

                    if (currentIdx > 0) {
                        currentTargets[i].velocity.x = (nextPos.x - currentTargets[i].pos.x) / config.arrayTimeStep;
                        currentTargets[i].velocity.y = (nextPos.y - currentTargets[i].pos.y) / config.arrayTimeStep;
                    }
                    currentTargets[i].pos = nextPos;
                }
            }
        }

        currentIdx++;

        float sleepDuration = config.arrayTimeStep / config.timeScale;
        std::this_thread::sleep_for(std::chrono::duration<float>(sleepDuration));
    }
}