#include <fstream>
#include <iostream>

#include "ballistics.hpp"

int main()
{
  BallisticsInput input{};

  std::ifstream inputfs("input.txt");
  if (!inputfs) {
    std::cerr << "Error opening input.txt\n";
    return 1;
  }

  inputfs >> input.drone_x >> input.drone_y >> input.drone_z;
  inputfs >> input.target_x >> input.target_y;
  inputfs >> input.attack_speed >> input.acceleration_path;
  inputfs >> input.ammo_name;
  inputfs.close();

  const DropSolution solution = compute_drop_solution(input);

  std::ofstream outputfs("output.txt");
  if (!outputfs) {
    std::cerr << "Error opening output.txt for writing";
    return 1;
  }

  if (std::abs(solution.drone_x - input.drone_x) > EPS || std::abs(solution.drone_y - input.drone_y) > EPS) {
    outputfs << solution.drone_x << " " << solution.drone_y << " ";
    if (!outputfs) {
      std::cerr << "Error writing xd:" << solution.drone_x << " yd:" << solution.drone_y << " to output.txt";
      return 1;
    }
  }

  outputfs << solution.fire_x << " " << solution.fire_y;
  if (!outputfs) {
    std::cerr << "Error writing fireX:" << solution.fire_x << " fireY:" << solution.fire_y << " to output.txt";
    return 1;
  }
}