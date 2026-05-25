constexpr int MAX_STEPS = 10000;
constexpr int MAX_NAME_LENGTH = 32;

constexpr float EPSILON = 1e-6f;
constexpr float GRAVITY = 9.81f;

struct AmmoParams {
    char name[MAX_NAME_LENGTH];
    float mass;
    float drag;
    float lift;
};

enum DroneState {
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING
};

struct Coord {
    float x;
    float y;

    Coord& operator+=(const Coord& other);
    Coord& operator-=(const Coord& other);
    Coord& operator*=(float s);
    Coord& operator/=(float s);
    Coord  operator+(const Coord& other) const;
    Coord  operator-(const Coord& other) const;
    Coord  operator*(float s) const;
    Coord  operator/(float s) const;
    bool   operator==(const Coord& other) const;
};

struct DroneConfig {
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    char  ammoName[MAX_NAME_LENGTH];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimStep {
    Coord pos;
    float direction;
    int   state;
    int   targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};

float length(const Coord& coord);
float distance(const Coord& coord1, const Coord& coord2);
Coord newPosition(const Coord& coord, float dir, float dist);
DroneConfig readConfig();
AmmoParams readAmmoParams(const char* name);
float calcHDistance(const float m, const float d, const float l, const float zd, const float attackSpeed);
float calcDirFromXAxis(const Coord& pos);
Coord calcDropPosition(const Coord& targetPos, const Coord& dronePos, float hDist);
float calcDroneFlightTime(float D, float attackSpeed, float accelerationPath);


#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif