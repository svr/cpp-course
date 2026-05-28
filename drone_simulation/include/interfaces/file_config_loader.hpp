
#include "drone_config.hpp"
#include "ammo_params.hpp"
#include "config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
    DroneConfig config;
    AmmoParams ammo;

    DroneConfig loadConfig(const char* file) const;
    AmmoParams loadAmmoParams(const char* name, const char* file) const;

    public:
    ~FileConfigLoader() override = default;
    void load(const char* configfile, const char* ammofile) override;
    DroneConfig getConfig() const override;
    AmmoParams getAmmoParams() const override;
};