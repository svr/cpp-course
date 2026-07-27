#pragma once

#include <gpiod.h>

class GPIO {
    const char* GPIO_CHIP;
    const unsigned int START_LINE;
    const unsigned int DROP_LINE;
    struct gpiod_chip* chip;

    struct gpiod_line_settings* settings;
    struct gpiod_line_config* line_cfg;
    struct gpiod_request_config* req_cfg;
    struct gpiod_line_request* request;

   public:
    GPIO(const char* gpio_chip, unsigned int start_line, unsigned int drop_line);
    ~GPIO();

    bool isReady() const;
    int start();
    int drop();
};


