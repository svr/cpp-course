#include <gpiod.h>
#include <cstdio>
#include <unistd.h>
#include <string>
#include <iostream>

#include "gpio.hpp"

GPIO::GPIO(const char* gpio_chip, unsigned int start_line, unsigned int drop_line)
    : GPIO_CHIP(gpio_chip),
      START_LINE(start_line),
      DROP_LINE(drop_line),
      chip(nullptr),
      settings(nullptr),
      line_cfg(nullptr),
      req_cfg(nullptr),
      request(nullptr)
{
    std::string chip_path = std::string("/dev/") + GPIO_CHIP;
    chip = gpiod_chip_open(chip_path.c_str());
    if (!chip) {
        perror("gpiod_chip_open");
        return;
    }

    settings = gpiod_line_settings_new();
    if (!settings) {
        perror("gpiod_line_settings_new");
        return;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_DOWN);

    line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        perror("gpiod_line_config_new");
        return;
    }

    const unsigned int offsets[] = {START_LINE, DROP_LINE};

    int ret = gpiod_line_config_add_line_settings(
        line_cfg,
        offsets,
        2,
        settings
    );

    if (ret < 0) {
        perror("gpiod_line_config_add_line_settings");
        return;
    }

    req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        perror("gpiod_request_config_new");
        return;
    }

    gpiod_request_config_set_consumer(req_cfg, "drone_gpio");

    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!request) {
        perror("gpiod_chip_request_lines");
        return;
    }
}

GPIO::~GPIO() {
    if (request)
        gpiod_line_request_release(request);
    if (req_cfg)
        gpiod_request_config_free(req_cfg);
    if (line_cfg)
        gpiod_line_config_free(line_cfg);
    if (settings)
        gpiod_line_settings_free(settings);
    if (chip)
        gpiod_chip_close(chip);
}

bool GPIO::isReady() const {
    return request != nullptr;
}

int GPIO::start() {
    if (!request)
        return -1;

    return gpiod_line_request_set_value(
        request,
        START_LINE,
        GPIOD_LINE_VALUE_ACTIVE
    );
}

int GPIO::drop() {
    if (!request)
        return -1;

    if (gpiod_line_request_set_value(request, DROP_LINE, GPIOD_LINE_VALUE_ACTIVE) < 0) {
        perror("gpiod drop: failed to set ACTIVE");
        return -1;
    }

    usleep(100000);

    gpiod_line_request_set_value(request, DROP_LINE, GPIOD_LINE_VALUE_INACTIVE);

    return 0;
}