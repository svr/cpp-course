#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <vector>

#include "antidrone_turret/turret_controller.hpp"


namespace {

TEST(TurretControllerTest, NonVisibleTarget) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;

  antidrone_turret::msg::Target target;
  target.visible = false;
  target.x = 320.0F;
  target.y = 240.0F;
  target.distance_m = 70.0F;
  target.confidence = 0.70F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);
  auto state = controller.compute_state(target);

  ASSERT_EQ(state.target_state, antidrone_turret::msg::TurretStatus::TARGET_NONE);
  ASSERT_EQ(state.action_state, antidrone_turret::msg::TurretStatus::ACTION_IDLE);
  ASSERT_EQ(state.trigger_state, antidrone_turret::msg::TurretStatus::TRIGGER_SKIP);
}

TEST(TurretControllerTest, LowConfidenceTarget) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  antidrone_turret::msg::Target target;
  target.visible = true;
  target.x = 320.0F;
  target.y = 240.0F;
  target.distance_m = 70.0F;
  target.confidence = 0.60F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);
  auto state = controller.compute_state(target);

  ASSERT_EQ(state.target_state, antidrone_turret::msg::TurretStatus::TARGET_LOW_CONFIDENCE);
  ASSERT_EQ(state.action_state, antidrone_turret::msg::TurretStatus::ACTION_IDLE);
  ASSERT_EQ(state.trigger_state, antidrone_turret::msg::TurretStatus::TRIGGER_SKIP);
}

TEST(TurretControllerTest, ReloadingActuatorAndTargetWithinDistance) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  antidrone_turret::msg::Target target;
  target.visible = true;
  target.x = 320.0F;
  target.y = 240.0F;
  target.distance_m = 50.0F;
  target.confidence = 0.80F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);
  controller.set_actuator_state(antidrone_turret::msg::ActuatorStatus::RELOADING);

  auto state = controller.compute_state(target);

  ASSERT_EQ(state.target_state, antidrone_turret::msg::TurretStatus::TARGET_LOCKED);
  ASSERT_EQ(state.action_state, antidrone_turret::msg::TurretStatus::ACTION_TRACK);
  ASSERT_EQ(state.trigger_state, antidrone_turret::msg::TurretStatus::TRIGGER_RELOADING);
}

TEST(TurretControllerTest, ReadyActuatorAndTargetWithinDistance) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  antidrone_turret::msg::Target target;
  target.visible = true;
  target.x = 320.0F;
  target.y = 240.0F;
  target.distance_m = 50.0F;
  target.confidence = 0.80F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);
  controller.set_actuator_state(antidrone_turret::msg::ActuatorStatus::READY);

  auto state = controller.compute_state(target);

  ASSERT_EQ(state.target_state, antidrone_turret::msg::TurretStatus::TARGET_LOCKED);
  ASSERT_EQ(state.action_state, antidrone_turret::msg::TurretStatus::ACTION_TRACK);
  ASSERT_EQ(state.trigger_state, antidrone_turret::msg::TurretStatus::TRIGGER_REQUESTED);
}

TEST(TurretControllerTest, ComputeServoCommandLeft) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  const float target_x = 100.0F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);

  auto servo_command = controller.compute_servo_command(target_x);

  ASSERT_EQ(servo_command.direction, antidrone_turret::msg::ServoCommand::LEFT);
  ASSERT_EQ(servo_command.target_x, target_x);
  ASSERT_EQ(servo_command.error_x, target_x - 320.0f);
}

TEST(TurretControllerTest, ComputeServoCommandCenter) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  const float target_x = 320.0F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);

  auto servo_command = controller.compute_servo_command(target_x);

  ASSERT_EQ(servo_command.direction, antidrone_turret::msg::ServoCommand::CENTER);
  ASSERT_EQ(servo_command.target_x, target_x);
  ASSERT_EQ(servo_command.error_x, target_x - 320.0f);
}

TEST(TurretControllerTest, ComputeServoCommandRight) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  const float target_x = 540.0F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);

  auto servo_command = controller.compute_servo_command(target_x);

  ASSERT_EQ(servo_command.direction, antidrone_turret::msg::ServoCommand::RIGHT);
  ASSERT_EQ(servo_command.target_x, target_x);
  ASSERT_EQ(servo_command.error_x, target_x - 320.0f);
}

TEST(TurretControllerTest, ComputeGimbalCommandUp) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  const float target_y = 100.0F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);

  auto gimbal_command = controller.compute_gimbal_command(target_y);

  ASSERT_EQ(gimbal_command.direction, antidrone_turret::msg::GimbalCommand::UP);
  ASSERT_EQ(gimbal_command.target_y, target_y);
  ASSERT_EQ(gimbal_command.error_y, 240.0f - target_y);
}

TEST(TurretControllerTest, ComputeGimbalCommandCenter) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  const float target_y = 240.0F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);

  auto gimbal_command = controller.compute_gimbal_command(target_y);

  ASSERT_EQ(gimbal_command.direction, antidrone_turret::msg::GimbalCommand::CENTER);
  ASSERT_EQ(gimbal_command.target_y, target_y);
  ASSERT_EQ(gimbal_command.error_y, 240.0f - target_y);
}

TEST(TurretControllerTest, ComputeGimbalCommandDown) {
  const float confidence_threshold = 0.7F;
  const float max_distance_m = 70.0F;
  const float target_y = 380.0F;

  antidrone_turret::TurretController controller(confidence_threshold, max_distance_m);

  auto gimbal_command = controller.compute_gimbal_command(target_y);

  ASSERT_EQ(gimbal_command.direction, antidrone_turret::msg::GimbalCommand::DOWN);
  ASSERT_EQ(gimbal_command.target_y, target_y);
  ASSERT_EQ(gimbal_command.error_y, 240.0f - target_y);
}

}  // namespace
