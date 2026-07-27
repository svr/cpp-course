#pragma once

namespace cli {
    struct CommandArgs {
        const char* uart = nullptr;
        const char* gpiochip = nullptr;
        int start_line = -1;
        int drop_line = -1;
    };

    CommandArgs parse_args(int argc, char* argv[]);
};
