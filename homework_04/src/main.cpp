#include <iostream>
#include <fstream>
#include <cmath>

constexpr int ticks_per_revolution   = 1024;
constexpr float wheel_radius_m       = 0.3;
constexpr float wheelbase_m          = 1.0;
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

    double x;
    double y;
    double theta;

    inputfs >> timestamp_ms >> prev_fl_ticks >> prev_fr_ticks >> prev_bl_ticks >> prev_br_ticks;

    while(inputfs) {
        inputfs >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks;

        long d_fl = fl_ticks - prev_fl_ticks;
        long d_fr = fr_ticks - prev_fr_ticks;
        long d_bl = bl_ticks - prev_bl_ticks;
        long d_br = br_ticks - prev_br_ticks;

        long d_left  = (d_fl + d_bl) / 2;
        long d_right = (d_fr + d_br) / 2;

        calcPosition(d_left, d_right, x, y, theta);

        prev_fl_ticks = fl_ticks;
        prev_fr_ticks = fr_ticks;
        prev_bl_ticks = bl_ticks;
        prev_br_ticks = br_ticks;
    }

    // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
    // Output: same tabular format on stdout, starting from the second sample:
    //         timestamp_ms x y theta

    return 0;
}
