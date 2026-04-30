#include <iostream>
#include <fstream>

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


    inputfs >> timestamp_ms >> prev_fl_ticks >> prev_fr_ticks >> prev_bl_ticks >> prev_br_ticks;

    while(inputfs) {
        inputfs >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks;

        // TODO: calculations

        prev_fl_ticks = fl_ticks;
        prev_fr_ticks = fr_ticks;
        prev_bl_ticks = bl_ticks;
        prev_br_ticks = br_ticks;
    }

    // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
    //
    // Parameters:
    //   ticks_per_revolution = 1024
    //   wheel_radius_m       = 0.3
    //   wheelbase_m          = 1.0
    //
    // Input:  text file with 5 whitespace-separated numbers per line:
    //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
    // Output: same tabular format on stdout, starting from the second sample:
    //         timestamp_ms x y theta

    return 0;
}
