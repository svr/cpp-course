#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>

constexpr int ticks_per_revolution   = 1024;
constexpr double wheel_radius_m      = 0.3;
constexpr double wheelbase_m         = 1.0;
constexpr double distance_per_tick   = 2.0 * M_PI * wheel_radius_m / ticks_per_revolution;

void calcPosition(double d_left, double d_right, double& outX, double& outY, double& outTheta) {
    double dL = d_left  * distance_per_tick;
    double dR = d_right * distance_per_tick;
    double d      = (dL + dR) / 2;
    double dtheta = (dR - dL) / wheelbase_m;

    outX     += d * cos(outTheta + dtheta / 2);
    outY     += d * sin(outTheta + dtheta / 2);
    outTheta += dtheta;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <input_path>\n";
        return 1;
    }

    std::ifstream inputfs(argv[1]);
    if (!inputfs) {
        std::cerr << "could not open file " << argv[1] << " for reading\n";
        return 1;
    }

    long timestamp_ms;

    long prev_fl_ticks;
    long prev_fr_ticks;
    long prev_bl_ticks;
    long prev_br_ticks;

    long fl_ticks;
    long fr_ticks;
    long bl_ticks;
    long br_ticks;

    double x{};
    double y{};
    double theta{};

    inputfs >> timestamp_ms >> prev_fl_ticks >> prev_fr_ticks >> prev_bl_ticks >> prev_br_ticks;
    if (!inputfs) {
        std::cerr << "Invalid or empty input file " << argv[1] << " \n";
        return 1;
    }

    while (inputfs >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks) {
        long d_fl = fl_ticks - prev_fl_ticks;
        long d_fr = fr_ticks - prev_fr_ticks;
        long d_bl = bl_ticks - prev_bl_ticks;
        long d_br = br_ticks - prev_br_ticks;

        double d_left  = (d_fl + d_bl) / 2.0;
        double d_right = (d_fr + d_br) / 2.0;

        calcPosition(d_left, d_right, x, y, theta);

        std::cout << timestamp_ms << std::fixed << std::setprecision(5) << " " << x << " " << y << " " << theta << "\n";

        prev_fl_ticks = fl_ticks;
        prev_fr_ticks = fr_ticks;
        prev_bl_ticks = bl_ticks;
        prev_br_ticks = br_ticks;
    }

    return 0;
}
