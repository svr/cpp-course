#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "file_config_loader.hpp"

DroneConfig FileConfigLoader::loadConfig(const char* file) const {
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

    std::string ammoNameStr = configJson["ammo"].get<std::string>();
    std::strncpy(config.ammoName, ammoNameStr.c_str(), MAX_NAME_LENGTH - 1);
    config.ammoName[MAX_NAME_LENGTH - 1] = '\0';

    return config;
}

AmmoParams FileConfigLoader::loadAmmoParams(const char* name, const char* file) const {
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
        if (std::strncmp(name, ammoName.c_str(), MAX_NAME_LENGTH) == 0) {
            AmmoParams ammoParams;
            std::strncpy(ammoParams.name, ammoName.c_str(), MAX_NAME_LENGTH - 1);
            ammoParams.name[MAX_NAME_LENGTH - 1] = '\0';
            ammoParams.mass = ammoList[i]["mass"];
            ammoParams.drag = ammoList[i]["drag"];
            ammoParams.lift = ammoList[i]["lift"];

            return ammoParams;
        }
    }
    throw std::runtime_error("Error: Unknown ammo name ");
};

void FileConfigLoader::load(const char* configfile = "config.json" , const char* ammofile = "ammo.json") {
    config = loadConfig(configfile);
    ammo = loadAmmoParams(config.ammoName, ammofile);
}

DroneConfig FileConfigLoader::getConfig() const {
    return config;
}

AmmoParams FileConfigLoader::getAmmoParams() const {
    return ammo;
};