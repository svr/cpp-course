#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>

#include "ballistics.hpp"


DropSolution compute_drop_solution(const BallisticsInput& input) {
    float m, d, l;
    float drone_x = input.drone_x;
    float drone_y = input.drone_y;

    DropSolution solution;

    if (std::strncmp(input.ammo_name, "VOG-17", MAX_AMMO_NAME_LENGTH) == 0) {
        m = 0.35;
        d = 0.07;
        l = 0.0;
    } else if (std::strncmp(input.ammo_name, "M67", MAX_AMMO_NAME_LENGTH) == 0) {
        m = 0.6;
        d = 0.1;
        l = 0.0;
    } else if (std::strncmp(input.ammo_name, "RKG-3", MAX_AMMO_NAME_LENGTH) == 0) {
        m = 1.2;
        d = 0.1;
        l = 0.0;
    } else if (std::strncmp(input.ammo_name, "GLIDING-VOG", MAX_AMMO_NAME_LENGTH) == 0) {
        m = 0.45;
        d = 0.1;
        l = 1.0;
    } else if (std::strncmp(input.ammo_name, "GLIDING-RKG", MAX_AMMO_NAME_LENGTH) == 0) {
        m = 1.4;
        d = 0.1;
        l = 1.0;
    } else {
        solution.status_code = 1;
        return solution;
    }

    constexpr float g = 9.81;
    float a = d * g * m - 2 * d * d * l * input.attack_speed;
    float b = -3 * g * m * m + 3 * d * l * m * input.attack_speed;
    float c = 6 * m * m * input.drone_z;

    float p = -(b * b) / (3 * a * a);
    float q = 2 * std::pow(b, 3) / (27 * std::pow(a, 3)) + c / a;

    float acos_arg = 3 * q / (2 * p) * std::sqrt(-3 / p);

    if (acos_arg < -1 || acos_arg > 1) {
        solution.status_code = 2;
        return solution;
    }

    float phi = std::acos(acos_arg);
    float t = 2 * std::sqrt(-p / 3.0) * std::cos((phi + 4 * M_PI) / 3.0) - b / (3.0 * a);

    float h = input.attack_speed * t -
        t * t * d * input.attack_speed / (2 * m) + std::pow(t, 3) * (6 * d * g * l * m - 6 * d * d * (l * l - 1) * input.attack_speed) / (36 * m * m) +
        std::pow(t, 4) * (-6 * d * d * g * l * (1 + l * l + std::pow(l, 4)) * m + 3 * std::pow(d, 3) * l * l * (1 + l * l) * input.attack_speed + 6 * std::pow(d, 3) * std::pow(l, 4) * (1 + l * l) * input.attack_speed) / (36 * std::pow((1 + l * l), 2) * std::pow(m, 3)) +
        std::pow(t, 5) * (3 * std::pow(d, 3) * g * std::pow(l, 3) * m - 3 * pow(d, 4) * l * l * (1 + l * l) * input.attack_speed) / (36 * (1 + l * l) * std::pow(m, 4));

    float D = std::sqrt(
        std::pow(input.target_x - drone_x, 2) +
        std::pow(input.target_y - drone_y, 2)
    );

    if (h + input.acceleration_path > D) {
        const float EPS = 1e-6f;
        if (D < EPS) {
         drone_x = drone_x - h + input.acceleration_path;
            D = std::sqrt(
                std::pow(input.target_x - drone_x, 2) +
                std::pow(input.target_y - drone_y, 2)
            );
        }

        drone_x = input.target_x - (input.target_x - drone_x) * (h + input.acceleration_path) / D;
        drone_y = input.target_y - (input.target_y - drone_y) * (h + input.acceleration_path) / D;

        D = std::sqrt(
            std::pow(input.target_x - drone_x, 2) +
            std::pow(input.target_y - drone_y, 2)
        );
    }

    float ratio = (D - h) / D;
    float fire_x = drone_x + (input.target_x - drone_x) * ratio;
    float fire_y = drone_y + (input.target_y - drone_y) * ratio;

    solution.status_code = 0;
    solution.drone_x = drone_x;
    solution.drone_y = drone_y;
    solution.fire_x = fire_x;
    solution.fire_y = fire_y;

    return solution;
}