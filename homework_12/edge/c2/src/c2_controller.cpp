#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <string>

static constexpr uint16_t STUB_PORT = 14560;

std::ostream& operator<<(std::ostream& os, C2State state) {
    switch (state) {
        case C2State::DISARMED:     return os << "DISARMED";
        case C2State::ARMED_HOLD:   return os << "ARMED_HOLD";
        case C2State::ARMED_GUIDED: return os << "ARMED_GUIDED";
        case C2State::ARMED_MANUAL: return os << "ARMED_MANUAL";
        default:                    return os << "UNKNOWN";
    }
}

struct C2Controller::Impl {
    C2State state = C2State::DISARMED;
    FcLink fc_link;
    UdpSocket udp_socket;
    std::ofstream log_file;
    bool connected = false;

    Impl(uint16_t fc_port)
        : fc_link(fc_port), udp_socket(STUB_PORT) {}

    void init() {
        std::ofstream("/tmp/c2_healthy").close();
        connected = true;
    }

    void transition(C2State next) {
        if(next != state) {
            std::cout << "[C2] state: " << state << " -> " << next << std::endl;
            log_file << "[C2] state: " << state << " -> " << next << "\n";
            state = next;
        }
    }
    void forward_next_point() {
        // Отримати точку маршруту у форматi JSON вiд auto_stub.
        std::string json_str;
        char buf[1024];
        ssize_t n = udp_socket.recv(buf, sizeof(buf));
        if (n > 0) {
            json_str.assign(buf, n);
            try {
                auto json = nlohmann::json::parse(json_str);
                float north = json.at("north_m").get<float>();
                float east  = json.at("east_m").get<float>();
                fc_link.go_to_ned(north, east);
                log_file << "[C2] fwd: north=" << north << " east=" << east << "\n";
            } catch (const nlohmann::json::exception& e) {
                std::cerr << "[C2] error: invalid JSON from auto_stub: " << e.what() << "\n";
            }
        }

    }

    void hold() {
        fc_link.hold();
        log_blocked();
    }

    void log_blocked() {
        log_file << "[C2] blocked: waypoint in " << state << "\n";
    }
};

C2Controller::C2Controller(uint16_t fc_port): impl_(std::make_unique<Impl>(fc_port)) {
    impl_->log_file.open("/var/log/c2/c2.log", std::ios::app);
}

C2Controller::~C2Controller() = default;

void C2Controller::tick() {
    if(!impl_->fc_link.is_connected()) {
        return;
    }

    if(!impl_->connected) {
        impl_->init();
    }
    if(!impl_->fc_link.is_armed()) {
        impl_->transition(C2State::DISARMED);
    } else {
        switch(impl_->fc_link.flight_mode()) {
            case FcLink::FlightMode::Guided:
                impl_->transition(C2State::ARMED_GUIDED);
                impl_->forward_next_point();
                break;
            case FcLink::FlightMode::Manual:
                impl_->transition(C2State::ARMED_MANUAL);
                impl_->log_blocked();
                break;
            case FcLink::FlightMode::Hold:
            default:
            {
                const bool entering_hold = (impl_->state != C2State::ARMED_HOLD);
                impl_->transition(C2State::ARMED_HOLD);
                if (entering_hold) {
                    impl_->hold();
                }
                break;
            }
        }
    }
}

C2State C2Controller::current_state() const {
    return impl_->state;
}
