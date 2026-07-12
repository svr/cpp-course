#pragma once

#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/turret_status.hpp"

namespace antidrone_turret {
    struct State {
        std::uint8_t target_state{antidrone_turret::msg::TurretStatus::TARGET_NONE};
        std::uint8_t action_state{antidrone_turret::msg::TurretStatus::ACTION_IDLE};
        std::uint8_t trigger_state{antidrone_turret::msg::TurretStatus::TRIGGER_SKIP};
    };
    class TurretController {
    public:
        TurretController() = delete;
        TurretController(float confidence_threshold, float max_distance_m)
            : confidence_threshold_(confidence_threshold), max_distance_m_(max_distance_m) {}

        void set_actuator_state(std::uint8_t state) {
            actuator_state_ = state;
        }

        State compute_state(const antidrone_turret::msg::Target& target) {
            State state;
            if(!target.visible) {
                state.target_state = antidrone_turret::msg::TurretStatus::TARGET_NONE;
                state.action_state = antidrone_turret::msg::TurretStatus::ACTION_IDLE;
                state.trigger_state = antidrone_turret::msg::TurretStatus::TRIGGER_SKIP;
            } else if(target.confidence < confidence_threshold_) {
                state.target_state = antidrone_turret::msg::TurretStatus::TARGET_LOW_CONFIDENCE;
                state.action_state = antidrone_turret::msg::TurretStatus::ACTION_IDLE;
                state.trigger_state = antidrone_turret::msg::TurretStatus::TRIGGER_SKIP;
            } else if(target.distance_m <= max_distance_m_) {
                state.target_state = antidrone_turret::msg::TurretStatus::TARGET_LOCKED;
                state.action_state = antidrone_turret::msg::TurretStatus::ACTION_TRACK;
                if(actuator_state_ == antidrone_turret::msg::ActuatorStatus::READY) {
                    state.trigger_state = antidrone_turret::msg::TurretStatus::TRIGGER_REQUESTED;
                } else {
                    state.trigger_state = antidrone_turret::msg::TurretStatus::TRIGGER_RELOADING;
                }
            } else if( target.distance_m > max_distance_m_) {
                state.target_state = antidrone_turret::msg::TurretStatus::TARGET_LOCKED;
                state.action_state = antidrone_turret::msg::TurretStatus::ACTION_TRACK;
                state.trigger_state = antidrone_turret::msg::TurretStatus::TRIGGER_SKIP;
            }

            return state;
        }

        antidrone_turret::msg::ServoCommand compute_servo_command(float target_x) {
            std::int8_t direction;

            if (target_x < 320.0f) {
                direction = antidrone_turret::msg::ServoCommand::LEFT;
            } else if (target_x > 320.0f) {
                direction = antidrone_turret::msg::ServoCommand::RIGHT;
            } else {
                direction = antidrone_turret::msg::ServoCommand::CENTER;
            }

            antidrone_turret::msg::ServoCommand command;
            command.direction = direction;
            command.target_x = target_x;
            command.error_x = 320.0f - target_x;

            return command;
        }

        antidrone_turret::msg::GimbalCommand compute_gimbal_command(float target_y) {
            std::int8_t direction;

            if(target_y < 240.0f) {
                direction = antidrone_turret::msg::GimbalCommand::UP;
            } else if(target_y > 240.0f) {
                direction = antidrone_turret::msg::GimbalCommand::DOWN;
            } else {
                direction = antidrone_turret::msg::GimbalCommand::CENTER;
            }

            antidrone_turret::msg::GimbalCommand command;
            command.direction = direction;
            command.target_y = target_y;
            command.error_y = 240.0f - target_y;

            return command;
        }
    private:
        std::uint8_t actuator_state_{antidrone_turret::msg::ActuatorStatus::RELOADING};
        float confidence_threshold_;
        float max_distance_m_ ;
    };

}  // namespace antidrone_turret
