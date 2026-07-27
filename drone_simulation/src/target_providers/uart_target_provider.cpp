#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <iostream>

#include <nlohmann/json.hpp>
#include <unordered_set>
using std::unordered_set;
using json = nlohmann::json;

#include "common.hpp"
#include "uart_target_provider.hpp"

void UartTargetProvider::load() {
    std::unordered_set<int> receivedTargetIds;
    while(receivedTargetIds.size() < static_cast<size_t>(getTargetCount())) {
        std::optional<dlink::TargetPos> packet = uartDataProvider->readTargetPacket();
        if (!packet.has_value()) {
            continue;
        }

        const dlink::TargetPos target_pos = *packet;
        {
            std::lock_guard<std::mutex> lock(providerMutex);
            targets[target_pos.id] = {{target_pos.x, target_pos.y}, {0.0f, 0.0f}};
        }
        receivedTargetIds.insert(target_pos.id);
    }
    isReady = true;
}

Target UartTargetProvider::getTarget(int targetNum) const {
    if (targetNum < 0 || targetNum >= getTargetCount()) {
        throw std::out_of_range("num exceeds target count");
    }

    Target target;
    {
        std::lock_guard<std::mutex> lock(providerMutex);
        target = targets[targetNum];
    }
    return target;
}

Coord UartTargetProvider::getTarget(int num, int timeIndex) const {
    if (num >= getTargetCount()) {
        throw std::out_of_range("num exceeds target count");
    }
    Coord target;
    {
        std::lock_guard<std::mutex> lock(providerMutex);
        target = targets[num].pos;
    }
    return target;
}

void UartTargetProvider::run() {
    while (!isStarted && !shouldStop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while (!shouldStop) {
        std::optional<dlink::TargetPos> packet = uartDataProvider->readTargetPacket();
        if (!packet.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const dlink::TargetPos target_pos = *packet;
        auto id = static_cast<size_t>(target_pos.id);
        {
            std::lock_guard<std::mutex> lock(providerMutex);
            auto prev_target = targets[id];
            Coord velocity = {
                (target_pos.x - prev_target.pos.x) / timeStep,
                (target_pos.y - prev_target.pos.y) / timeStep
            };
            targets[id] = {{target_pos.x, target_pos.y}, velocity};
        }
    }
}
