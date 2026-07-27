#include <getopt.h>
#include <iostream>

#include "parse_args.hpp"

namespace cli {
    void print_usage(const char* filename) {
        std::cerr <<
            "Usage:\n"
            "  " << filename << " --uart <path> --gpiochip <chip> --start-line <num> --drop-line <num>\n";
    }


    CommandArgs parse_args(int argc, char* argv[]) {
        CommandArgs cfg;

        static struct option long_options[] = {
            {"uart",       required_argument, 0, 'u'},
            {"gpiochip",   required_argument, 0, 'g'},
            {"start-line", required_argument, 0, 's'},
            {"drop-line",  required_argument, 0, 'd'},
            {"help",       no_argument,       0, 'h'},
            {0, 0, 0, 0}
        };

        int opt;
        int option_index = 0;

        while ((opt = getopt_long(argc, argv, "u:g:s:d:h",
                                long_options, &option_index)) != -1) {
            switch (opt) {
                case 'u':
                    cfg.uart = optarg;
                    break;

                case 'g':
                    cfg.gpiochip = optarg;
                    break;

                case 's':
                    cfg.start_line = std::atoi(optarg);
                    break;

                case 'd':
                    cfg.drop_line = std::atoi(optarg);
                    break;

                case 'h':
                    print_usage(argv[0]);
                    std::exit(0);

                case '?':
                default:
                    print_usage(argv[0]);
                    std::exit(1);
            }
        }

        if (!cfg.uart) {
            std::cerr << "Missing --uart\n";
            std::exit(1);
        }

        if (!cfg.gpiochip) {
            std::cerr << "Missing --gpiochip\n";
            std::exit(1);
        }

        if (cfg.start_line < 0) {
            std::cerr << "Invalid or missing --start-line\n";
            std::exit(1);
        }

        if (cfg.drop_line < 0) {
            std::cerr << "Invalid or missing --drop-line\n";
            std::exit(1);
        }

        return cfg;
    }
}