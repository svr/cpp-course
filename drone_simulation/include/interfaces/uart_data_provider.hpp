#pragma once

#include <queue>
#include <memory>
#include <mutex>
#include <optional>
#include <chrono>
#include <iostream>

#include "runnable_thread.hpp"
#include "uart.hpp"

class UartDataProvider : public RunnableThread {
    std::shared_ptr<UART> uart;
    mutable std::mutex queueMutex;

    static constexpr std::size_t QUEUE_MAX_CAPACITY = 64;
    static constexpr auto IDLE_SLEEP = std::chrono::milliseconds(1);
    std::queue<dlink::TargetPos> targetQueue;

    std::optional<dlink::Telemetry> telemetryLatest;
    std::optional<dlink::AmmoCfg> ammoLatest;
    std::optional<dlink::Result> resultLatest;
    std::optional<dlink::Control> controlLatest;
    std::optional<dlink::DroneCfg> configLatest;

    void dispatchPacket(const UART::Packet& packet);

    public:
    explicit UartDataProvider(const char* uart_path): uart(std::make_shared<UART>(uart_path)) {
        if (uart->isOpen()) {
            isReady = true;
        }
    }
    ~UartDataProvider() override = default;

    void run() override;

    std::optional<dlink::Telemetry> readTelemetryPacket();
    std::optional<dlink::TargetPos> readTargetPacket();
    std::optional<dlink::AmmoCfg> readAmmoPacket();
    std::optional<dlink::Result> readResultPacket();
    std::optional<dlink::Control> readControlPacket();
    std::optional<dlink::DroneCfg> readConfigPacket();
    int sendControlPacket(float accel, float turnRate);
};
