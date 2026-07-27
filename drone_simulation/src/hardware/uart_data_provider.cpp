#include <chrono>
#include <thread>
#include <utility>
#include <variant>

#include "uart_data_provider.hpp"

void UartDataProvider::dispatchPacket(const UART::Packet& packet) {
    if (const auto* p = std::get_if<dlink::Telemetry>(&packet)) {
        telemetryLatest = *p;
        return;
    }
    if (const auto* p = std::get_if<dlink::TargetPos>(&packet)) {
        targetQueue.push(*p);
        return;
    }
    if (const auto* p = std::get_if<dlink::AmmoCfg>(&packet)) {
        ammoLatest = *p;
        return;
    }
    if (const auto* p = std::get_if<dlink::Result>(&packet)) {
        resultLatest = *p;
        return;
    }
    if (const auto* p = std::get_if<dlink::Control>(&packet)) {
        controlLatest = *p;
        return;
    }
    if (const auto* p = std::get_if<dlink::DroneCfg>(&packet)) {
        configLatest = *p;
    }
}

void UartDataProvider::run() {
    while (!isStarted && !shouldStop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    while (!shouldStop) {
        std::optional<UART::Packet> packet = uart->readPacket();
        if (packet.has_value()) {
            std::lock_guard<std::mutex> lock(queueMutex);
            dispatchPacket(*packet);
            continue;
        }
    }
}

std::optional<dlink::Telemetry> UartDataProvider::readTelemetryPacket() {
    std::lock_guard<std::mutex> lock(queueMutex);
    return telemetryLatest;
}

std::optional<dlink::TargetPos> UartDataProvider::readTargetPacket() {
    std::lock_guard<std::mutex> lock(queueMutex);
    if(targetQueue.empty()) {
        return std::nullopt;
    }
    auto val = std::make_optional(targetQueue.front());
    targetQueue.pop();
    return val;
}

std::optional<dlink::AmmoCfg> UartDataProvider::readAmmoPacket() {
    std::lock_guard<std::mutex> lock(queueMutex);
    return ammoLatest;
}

std::optional<dlink::Result> UartDataProvider::readResultPacket() {
    std::lock_guard<std::mutex> lock(queueMutex);
    return resultLatest;
}

std::optional<dlink::Control> UartDataProvider::readControlPacket() {
    std::lock_guard<std::mutex> lock(queueMutex);
    return controlLatest;
}

std::optional<dlink::DroneCfg> UartDataProvider::readConfigPacket() {
    std::lock_guard<std::mutex> lock(queueMutex);
    return configLatest;
}

int UartDataProvider::sendControlPacket(float accel, float turnRate) {
    return uart->sendControl(accel, turnRate);
}
