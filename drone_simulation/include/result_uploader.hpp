#pragma once
#include <chrono>

#include "httplib.h"
#include "upload_report.hpp"

class ResultUploader {

public:

private:
    httplib::Client http;
    const std::string api_key;
    const std::string student_id;
    static constexpr int MAX_ATTEMPTS = 5;
    std::chrono::seconds retry_delay = std::chrono::seconds(1);

    static bool is_success_status(int status) {
        return status >= 200 && status < 300;
    }

    static bool is_retryable_status(int status) {
        return status == 503;
    }

    bool verify_result(const std::string& test_id);

public:
    ResultUploader(std::string host, const std::string& api_key, const std::string& student_id);
    UploadReport upload(const std::string& test_id, const std::string& result_json);
};
