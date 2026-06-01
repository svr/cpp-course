#pragma once
#include <string>

#include "drone_config.hpp"
#include "ammo_params.hpp"
#include "config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
    DroneConfig config;
    AmmoParams ammo;

    DroneConfig loadConfig(const std::string& file) const;
    AmmoParams loadAmmoParams(const std::string& name, const std::string& file) const;

    public:
    ~FileConfigLoader() override = default;
    void load(const std::string& configfile, const std::string& ammofile) override;
    DroneConfig getConfig() const override;
    AmmoParams getAmmoParams() const override;
};