#include "ballistics.hpp"
#include <gtest/gtest.h>

TEST(Ballistics, VOG_17_NearTarget) {

  const BallisticsInput input{
      .drone_x = 100.0,
      .drone_y = 100.0,
      .drone_z = 100.0,
      .target_x = 200.0,
      .target_y = 200.0,
      .attack_speed = 10.0,
      .acceleration_path = 10.0,
      .ammo_name = "VOG-17",
  };

  const DropSolution solution = compute_drop_solution(input);

  EXPECT_NEAR(solution.fire_x, 173.759, 0.01);
  EXPECT_NEAR(solution.fire_y, 173.759, 0.01);
}

TEST(Ballistics, GLIDING_VOG_FarTarget) {

  const BallisticsInput input{
      .drone_x = 0,
      .drone_y = 0,
      .drone_z = 100.0,
      .target_x = 300.0,
      .target_y = 300.0,
      .attack_speed = 20.0,
      .acceleration_path = 50.0,
      .ammo_name = "GLIDING-VOG",
  };

  const DropSolution solution = compute_drop_solution(input);

  EXPECT_EQ(solution.status_code, StatusCode::Ok);
  EXPECT_NEAR(solution.fire_x, 242.711, 0.01);
  EXPECT_NEAR(solution.fire_y, 242.711, 0.01);
}

TEST(Ballistics, GLIDING_RKG_FarTarget) {

  const BallisticsInput input{
      .drone_x = 543,
      .drone_y = 232,
      .drone_z = 120.0,
      .target_x = 1034.0,
      .target_y = 432.0,
      .attack_speed = 13.0,
      .acceleration_path = 12.0,
      .ammo_name = "GLIDING-RKG",
  };

  const DropSolution solution = compute_drop_solution(input);

  EXPECT_EQ(solution.status_code, StatusCode::Ok);
  EXPECT_NEAR(solution.fire_x, 966.534, 0.01);
  EXPECT_NEAR(solution.fire_y, 404.519, 0.01);
}

TEST(Ballistics, RKG_3_IntermediatePoint) {

  const BallisticsInput input{
      .drone_x = 543.0,
      .drone_y = 232.0,
      .drone_z = 120.0,
      .target_x = 553.0,
      .target_y = 242.0,
      .attack_speed = 13.0,
      .acceleration_path = 12.0,
      .ammo_name = "RKG-3",
  };

  const DropSolution solution = compute_drop_solution(input);

  EXPECT_EQ(solution.status_code, StatusCode::Ok);
  EXPECT_NEAR(solution.drone_x, 504.6, 0.01);
  EXPECT_NEAR(solution.drone_y, 193.6, 0.01);

  EXPECT_NEAR(solution.fire_x, 513.085, 0.01);
  EXPECT_NEAR(solution.fire_y, 202.085, 0.01);
}

TEST(Ballistics, M67_StartAboveTarget) {

  const BallisticsInput input{
      .drone_x = 543.0,
      .drone_y = 232.0,
      .drone_z = 120.0,
      .target_x = 543.0,
      .target_y = 232.0,
      .attack_speed = 13.0,
      .acceleration_path = 12.0,
      .ammo_name = "M67",
  };

  const DropSolution solution = compute_drop_solution(input);

  EXPECT_EQ(solution.status_code, StatusCode::Ok);
  EXPECT_NEAR(solution.drone_x, 478.496, 0.01);
  EXPECT_NEAR(solution.drone_y, 232.0, 0.01);

  EXPECT_NEAR(solution.fire_x, 490.496, 0.01);
  EXPECT_NEAR(solution.fire_y, 232.0, 0.01);
}

TEST(Ballistics, UknownAmmo) {

  const BallisticsInput input{
      .drone_x = 543.0,
      .drone_y = 232.0,
      .drone_z = 120.0,
      .target_x = 543.0,
      .target_y = 232.0,
      .attack_speed = 13.0,
      .acceleration_path = 12.0,
      .ammo_name = "Unknown",
  };

  const DropSolution solution = compute_drop_solution(input);

  EXPECT_EQ(solution.status_code, StatusCode::UknownAmmo);
}

