

constexpr int MAX_AMMO_NAME_LENGTH = 16;
constexpr float EPS = 1e-6F;

enum class StatusCode {
    Ok = 0,
    UknownAmmo = 1,
    CalcEror = 3
};

struct BallisticsInput {
    float drone_x;
    float drone_y;
    float drone_z;
    float target_x ;
    float target_y;
    float attack_speed;
    float acceleration_path;
    char ammo_name[MAX_AMMO_NAME_LENGTH];
};

struct DropSolution {
    float fire_x;
    float fire_y;
    float drone_x;
    float drone_y;
    StatusCode status_code;
};

DropSolution compute_drop_solution(const BallisticsInput& input);
