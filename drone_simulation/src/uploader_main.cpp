#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "result_uploader.hpp"

static const std::string file = "uploader_config.json";

int main() {
    std::ifstream configStream(file);
    if (!configStream) {
        throw std::runtime_error("Failed to open config file");
    }
    json configJson;
    configStream >> configJson;
    configStream.close();

    ResultUploader uploader(configJson["host"], configJson["api_key"], configJson["student_id"]);

    for (int i = 1; i <= 10; ++i) {
        std::string prefix = i < 10 ? "0" : "";
        std::string test_id = "T" + prefix + std::to_string(i);
        std::string result_file = "results/simulation_" + test_id + ".json";

        std::ifstream resultStream(result_file);
        if (!resultStream) {
            continue;
        }

        std::string result_json((std::istreambuf_iterator<char>(resultStream)), std::istreambuf_iterator<char>());
        resultStream.close();

        UploadReport report = uploader.upload(test_id, result_json);

        std::cout << "Test: " << test_id
                  << " Status: " << report.status
                  << " Attempts: " << report.attempts
                  << "\n";
    }

    return 0;
}