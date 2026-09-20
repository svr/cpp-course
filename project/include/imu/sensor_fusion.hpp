#pragma once

#include "attitude_state.hpp"
#include "imu_sample.hpp"

class SensorFusion {
public:
  explicit SensorFusion(float tau = 0.5f);
  AttitudeState update(const ImuSample &imu, float dt);
  void reset();

private:
  struct KalmanAxis {
    float angle{0.0f};
    float bias{0.0f};

    float p00{1.0f};
    float p01{0.0f};
    float p10{0.0f};
    float p11{1.0f};
  };

  float kalman_update(KalmanAxis &state, float gyro_rate, float accel_angle, float dt);
  static float normalize_angle(float angle);

private:
  float tau_;

  float q_angle_{0.001f};
  float q_bias_{0.003f};
  float r_measure_{0.03f};

  float yaw_{0.0f};

  KalmanAxis roll_;
  KalmanAxis pitch_;
};