#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

#include "result_uploader.hpp"


ResultUploader::ResultUploader(std::string host, const std::string& api_key, const std::string& student_id) : http(host), api_key(api_key), student_id(student_id) {
    http.set_connection_timeout(2, 0);
    http.set_read_timeout(2, 0);
}

bool ResultUploader::verify_result(const std::string& test_id) {
    httplib::Headers headers = {
        {"x-api-key", api_key}
    };

    const auto url = "/api/dz12/results/" + test_id + "/" + student_id;
    const auto res = http.Get(url, headers);

    if (!res) {
        return false;
    }

    if (!is_success_status(res->status)) {
        return false;
    }

    try {
        const auto body = nlohmann::json::parse(res->body);
        if (body.value("found", false)) {
            return true;
        }

        return false;
    } catch (const std::exception& ex) {
        return false;
    }
}

UploadReport ResultUploader::upload(const std::string& test_id, const std::string& result_json) {
    httplib::Headers headers = {
        {"Content-Type", "application/json"},
        {"x-api-key", api_key}
    };

    UploadReport report{test_id, 200, 1};

    nlohmann::json payload;
    try {
        payload = {
            {"testId", test_id},
            {"studentId", student_id},
            {"simulation", nlohmann::json::parse(result_json)}
        };
    } catch (const std::exception& ex) {
        std::cerr << "Failed to parse result JSON:" << ex.what() << std::endl;
        report.status = 400;
        return report;
    }

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
        const auto res = http.Post("/api/dz12/results", headers, payload.dump(), "application/json");

        if (!res ||is_retryable_status(res->status)) {
            report.status = 503;
            if (attempt < MAX_ATTEMPTS) {
                std::this_thread::sleep_for(retry_delay);
                ++report.attempts;
                retry_delay *= 2;
                continue;
            }
            return report;
        } else if (!is_success_status(res->status)) {
            report.status = res->status;
            return report;
        }

        const bool verified = verify_result(test_id);
        if (verified) {
            report.status = 200;
            return report;
        }

        if (attempt < MAX_ATTEMPTS) {
            ++report.attempts;
            std::this_thread::sleep_for(retry_delay);
            retry_delay *= 2;
            continue;
        }

        report.status = 0;
        return report;
    }

    return report;
}