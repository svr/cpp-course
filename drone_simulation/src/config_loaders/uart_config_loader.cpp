#include <iostream>
#include <optional>
#include <variant>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "uart_config_loader.hpp"

void UartConfigLoader::load() {
    loadAmmoParams();
    loadConfig();
}

void UartConfigLoader::loadConfig() {
    while (true) {
        std::optional<dlink::DroneCfg> packet = uartDataProvider->readConfigPacket();
        if (!packet.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        const dlink::DroneCfg drone_cfg = *packet;
        config.attackSpeed = drone_cfg.attackSpeed;
        config.accelPath = drone_cfg.accelerationPath;
        config.angularSpeed = drone_cfg.angularSpeed;
        config.turnThreshold = drone_cfg.turnThreshold;
        config.simTimeStep = drone_cfg.timeStep;
        config.timeScale = drone_cfg.timeScale;
        break;
    }
}

void UartConfigLoader::loadAmmoParams() {
    while (true) {
        std::optional<dlink::AmmoCfg> packet = uartDataProvider->readAmmoPacket();
        if (!packet.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        const dlink::AmmoCfg ammo_cfg = *packet;
        ammo.name = std::string(ammo_cfg.name);
        ammo.mass = ammo_cfg.mass;
        ammo.drag = ammo_cfg.drag;
        ammo.lift = ammo_cfg.lift;

        config.nTargets = static_cast<int>(ammo_cfg.nTargets);
        config.hitRadius = ammo_cfg.hitRadius;
        break;
    }
}
