#include "sensor_fusion.hpp"

#include <cmath>
#include <algorithm>

namespace {

constexpr float MIN_DT = 0.0001f;
constexpr float MAX_DT = 0.1f;

constexpr float MIN_ACCEL_NORM = 0.5f;
constexpr float MAX_ACCEL_NORM = 1.5f;

constexpr float GRAVITY = 9.80665f;
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;

} // namespace

SensorFusion::SensorFusion(float tau) : tau_(tau) {}

void SensorFusion::reset() {
  roll_ = {};
  pitch_ = {};
  yaw_ = 0.0f;
}

float SensorFusion::normalize_angle(float angle) {
    return std::remainder(angle, TWO_PI);
}

float SensorFusion::kalman_update(KalmanAxis &state, float gyro_rate, float accel_angle, float dt) {
  const float rate = gyro_rate - state.bias;
  state.angle += dt * rate;
  state.p00 += dt * (dt * state.p11 - state.p01 - state.p10 + q_angle_);
  state.p01 -= dt * state.p11;
  state.p10 -= dt * state.p11;
  state.p11 += q_bias_ * dt;

  const float innovation = accel_angle - state.angle;
  const float s = state.p00 + r_measure_;
  const float k0 = state.p00 / s;
  const float k1 = state.p10 / s;

  state.angle += k0 * innovation;
  state.bias += k1 * innovation;

  const float p00_temp = state.p00;
  const float p01_temp = state.p01;

  state.p00 = state.p00 - k0 * p00_temp;
  state.p01 = state.p01 - k0 * p01_temp;
  state.p10 = state.p10 - k1 * p00_temp;
  state.p11 = state.p11 - k1 * p01_temp;
  state.angle = normalize_angle(state.angle);

  return state.angle;
}

AttitudeState SensorFusion::update(const ImuSample &imu, float dt) {
  dt = std::clamp(dt, MIN_DT, MAX_DT);

  const float accel_norm = std::sqrt(imu.accel_x * imu.accel_x + imu.accel_y * imu.accel_y +
                imu.accel_z * imu.accel_z);

  const bool accel_valid = accel_norm > MIN_ACCEL_NORM * GRAVITY &&
                           accel_norm < MAX_ACCEL_NORM * GRAVITY;

  const float roll_accel = std::atan2(-imu.accel_y, imu.accel_z);
  const float pitch_accel = std::atan2(-imu.accel_x, std::sqrt(imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z));

  const float roll_gyro = -imu.gyro_x;
  const float pitch_gyro = imu.gyro_y;
  const float yaw_gyro = imu.gyro_z;

  if (accel_valid) {
    kalman_update(roll_, roll_gyro, roll_accel, dt);
    kalman_update(pitch_, pitch_gyro, pitch_accel, dt);
  } else {
    roll_.angle += (roll_gyro - roll_.bias) * dt;
    pitch_.angle += (pitch_gyro - pitch_.bias) * dt;
    roll_.angle = normalize_angle(roll_.angle);
    pitch_.angle = normalize_angle(pitch_.angle);
  }

  yaw_ += yaw_gyro * dt;
  yaw_ = normalize_angle(yaw_);

  AttitudeState state{};

  state.roll = roll_.angle;
  state.pitch = pitch_.angle;
  state.yaw = yaw_;

  state.rollspeed = roll_gyro;
  state.pitchspeed = pitch_gyro;
  state.yawspeed = yaw_gyro;

  return state;
}