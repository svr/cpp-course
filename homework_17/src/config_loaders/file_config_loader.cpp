#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "file_config_loader.hpp"

DroneConfig FileConfigLoader::loadConfig(const std::string& file) const {
    std::ifstream configStream(file);
    if (!configStream) {
        throw std::runtime_error("Failed to open config file");
    }
    json configJson;
    configStream >> configJson;
    configStream.close();

    DroneConfig config;
    config.startPos.x    = configJson["drone"]["position"]["x"];
    config.startPos.y    = configJson["drone"]["position"]["y"];
    config.altitude      = configJson["drone"]["altitude"];
    config.initialDir    = configJson["drone"]["initialDirection"];
    config.attackSpeed   = configJson["drone"]["attackSpeed"];
    config.accelPath     = configJson["drone"]["accelerationPath"];
    config.angularSpeed  = configJson["drone"]["angularSpeed"];
    config.turnThreshold = configJson["drone"]["turnThreshold"];
    config.simTimeStep   = configJson["simulation"]["timeStep"];
    config.hitRadius     = configJson["simulation"]["hitRadius"];
    config.arrayTimeStep = configJson["targetArrayTimeStep"];
    config.targetTimeStep  = configJson["simulation"]["targetTimeStep"];
    config.physicsTimeStep = configJson["simulation"]["physicsTimeStep"];
    config.timeScale       = configJson["simulation"]["timeScale"];
    config.ammoName = configJson["ammo"].get<std::string>();

    return config;
}

AmmoParams FileConfigLoader::loadAmmoParams(const std::string& name, const std::string& file) const {
    std::ifstream ammoStream(file);
    if (!ammoStream) {
        throw std::runtime_error("Failed to open ammo file");
    }

    json ammoList;
    ammoStream >> ammoList;
    ammoStream.close();

    int ammoCount = ammoList.size();

    for (int i = 0; i < ammoCount; ++i) {
        std::string ammoName = ammoList[i]["name"].get<std::string>();
        if (name == ammoName) {
            AmmoParams ammoParams;
            ammoParams.name = name;
            ammoParams.mass = ammoList[i]["mass"];
            ammoParams.drag = ammoList[i]["drag"];
            ammoParams.lift = ammoList[i]["lift"];

            return ammoParams;
        }
    }
    throw std::runtime_error("Error: Unknown ammo name ");
};

void FileConfigLoader::load() {
    config = loadConfig(configfile);
    ammo = loadAmmoParams(config.ammoName, ammofile);
}

DroneConfig FileConfigLoader::getConfig() const {
    return config;
}

AmmoParams FileConfigLoader::getAmmoParams() const {
    return ammo;
};