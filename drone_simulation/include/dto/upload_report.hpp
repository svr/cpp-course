#pragma once
#include <string>

struct UploadReport {
    std::string test_id;
    int status;
    int attempts;
};