#pragma once
#include <string>

#include "drone_config.hpp"
#include "ammo_params.hpp"

class IConfigLoader {
public:
    virtual void load() = 0;
    virtual DroneConfig getConfig() const = 0;
    virtual AmmoParams getAmmoParams() const = 0;
    virtual ~IConfigLoader() = default;
};
