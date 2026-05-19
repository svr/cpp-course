

constexpr int MAX_AMMO_NAME_LENGTH = 16;
constexpr float EPS = 1e-6f;


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
    short status_code;
    const char* status_message;
};

DropSolution compute_drop_solution(const BallisticsInput&);
