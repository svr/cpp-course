#pragma once
#include <string>
#include <memory>
#include "drone_config.hpp"
#include "ammo_params.hpp"
#include "config_loader.hpp"
#include "uart_data_provider.hpp"

class UartConfigLoader : public IConfigLoader {
    DroneConfig config;
    AmmoParams ammo;
    std::shared_ptr<UartDataProvider> uartDataProvider;
    void loadConfig();
    void loadAmmoParams();

    public:
    UartConfigLoader(std::shared_ptr<UartDataProvider> uartDataProvider)
        : uartDataProvider(std::move(uartDataProvider)) {};
    ~UartConfigLoader() override = default;
    void load() override;
    DroneConfig getConfig() const override {
        return config;
    }
    AmmoParams getAmmoParams() const override {
        return ammo;
    }
};