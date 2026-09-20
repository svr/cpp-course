#include "imu_manager.hpp"
#include "mavlink_communication.hpp"
#include "mavlink_transport_udp.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <getopt.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_running{true};

constexpr char DEFAULT_MAVLINK_HOST[] = "127.0.0.1";
constexpr std::uint16_t DEFAULT_MAVLINK_PORT = 14550;
constexpr double DEFAULT_RATE_HZ = 50.0;
constexpr std::size_t MAX_IMUS = 4;

enum OptionIndex {
  OPT_DEV0 = 1000,
  OPT_ADDR0,
  OPT_DEV1,
  OPT_ADDR1,
  OPT_DEV2,
  OPT_ADDR2,
  OPT_DEV3,
  OPT_ADDR3,
  OPT_MAVLINK_HOST,
  OPT_MAVLINK_PORT
};

void signal_handler(int sig) {
  if (sig == SIGINT || sig == SIGTERM) {
    g_running = false;
  }
}

void print_usage(const char *program) {
  std::cout << "Usage: " << program << " [options]\n\n"

            << "IMU options:\n"
            << "  --dev0 PATH              IMU 0 I2C device\n"
            << "  --addr0 HEX              IMU 0 I2C address\n"
            << "  --dev1 PATH              IMU 1 I2C device\n"
            << "  --addr1 HEX              IMU 1 I2C address\n"
            << "  --dev2 PATH              IMU 2 I2C device\n"
            << "  --addr2 HEX              IMU 2 I2C address\n"
            << "  --dev3 PATH              IMU 3 I2C device\n"
            << "  --addr3 HEX              IMU 3 I2C address\n"

            << "\nMAVLink options:\n"
            << "  --mavlink-host HOST      MAVLink UDP destination "
            << "default: " << DEFAULT_MAVLINK_HOST << "\n"
            << "  --mavlink-port PORT      MAVLink UDP destination port "
            << "default: " << DEFAULT_MAVLINK_PORT << "\n"

            << "\nOther:\n"
            << "  -h, --help               Show this help\n";
}

std::uint8_t parse_address(const char *value) {
  char *end = nullptr;

  const unsigned long address = std::strtoul(value, &end, 16);

  if (end == value || *end != '\0' || address > 0x7F) {
    throw std::invalid_argument("Invalid I2C address: " + std::string(value));
  }

  return static_cast<std::uint8_t>(address);
}

std::uint16_t parse_port(const char *value) {
  char *end = nullptr;

  const unsigned long port = std::strtoul(value, &end, 10);

  if (end == value || *end != '\0' || port == 0 ||
      port > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("Invalid UDP port: " + std::string(value));
  }

  return static_cast<std::uint16_t>(port);
}

void print_imu_config(std::size_t index, const ImuConfig &config) {
  std::cout << "[INFO] IMU " << index << ": " << config.device << " @ 0x"
            << std::hex << static_cast<unsigned>(config.address) << std::dec
            << '\n';
}

} // namespace

int main(int argc, char *argv[]) {
  std::vector<ImuConfig> configs = {ImuConfig{0x68, "/dev/i2c-1"},
                                    ImuConfig{0x68, "/dev/i2c-3"}};

  std::string mavlink_host = DEFAULT_MAVLINK_HOST;
  std::uint16_t mavlink_port = DEFAULT_MAVLINK_PORT;

  static const struct option long_options[] = {
      {"dev0", required_argument, nullptr, OPT_DEV0},
      {"addr0", required_argument, nullptr, OPT_ADDR0},

      {"dev1", required_argument, nullptr, OPT_DEV1},
      {"addr1", required_argument, nullptr, OPT_ADDR1},

      {"dev2", required_argument, nullptr, OPT_DEV2},
      {"addr2", required_argument, nullptr, OPT_ADDR2},

      {"dev3", required_argument, nullptr, OPT_DEV3},
      {"addr3", required_argument, nullptr, OPT_ADDR3},

      {"mavlink-host", required_argument, nullptr, OPT_MAVLINK_HOST},
      {"mavlink-port", required_argument, nullptr, OPT_MAVLINK_PORT},

      {"help", no_argument, nullptr, 'h'},

      {nullptr, 0, nullptr, 0}};

  try {
    int opt;

    while ((opt = getopt_long(argc, argv, "h", long_options, nullptr)) != -1) {

      switch (opt) {
      case OPT_DEV0:
        configs[0].device = optarg;
        break;

      case OPT_ADDR0:
        configs[0].address = parse_address(optarg);
        break;

      case OPT_DEV1:
        configs[1].device = optarg;
        break;

      case OPT_ADDR1:
        configs[1].address = parse_address(optarg);
        break;

      case OPT_DEV2:
        if (configs.size() < 3) {
          configs.resize(3);
        }

        if (configs[2].device.empty()) {
          configs[2].device = "/dev/i2c-6";
        }

        configs[2].address = parse_address(optarg);
        break;

      case OPT_DEV3:
        if (configs.size() < 4) {
          configs.resize(4);
        }

        if (configs[3].device.empty()) {
          configs[3].device = "/dev/i2c-7";
        }

        configs[3].address = parse_address(optarg);
        break;

      case OPT_ADDR2:
        if (configs.size() < 3) {
          configs.resize(3);
        }

        if (configs[2].device.empty()) {
          configs[2].device = "/dev/i2c-6";
        }

        configs[2].address = parse_address(optarg);
        break;

      case OPT_ADDR3:
        if (configs.size() < 4) {
          configs.resize(4);
        }

        if (configs[3].device.empty()) {
          configs[3].device = "/dev/i2c-7";
        }

        configs[3].address = parse_address(optarg);
        break;

      case OPT_MAVLINK_HOST:
        mavlink_host = optarg;
        break;

      case OPT_MAVLINK_PORT:
        mavlink_port = parse_port(optarg);
        break;

      case 'h':
        print_usage(argv[0]);
        return EXIT_SUCCESS;

      default:
        print_usage(argv[0]);
        return EXIT_FAILURE;
      }
    }

    if (configs.size() > MAX_IMUS) {
      throw std::invalid_argument("A maximum of 4 IMUs is supported");
    }

  } catch (const std::exception &e) {
    std::cerr << "[ERROR] " << e.what() << "\n\n";
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::cout << "[INFO] IMUs configured: " << configs.size() << '\n';

  for (std::size_t i = 0; i < configs.size(); ++i) {
    print_imu_config(i, configs[i]);
  }

  std::cout << "[INFO] MAVLink destination: " << mavlink_host << ":" << mavlink_port << '\n';

  auto transport =std::make_unique<MavlinkTransportUDP>(mavlink_host.c_str(), mavlink_port);
  MavlinkCommunication mavlink_comm(std::move(transport));

  mavlink_comm.start();
  mavlink_comm.send_info("IMU system initializing...");

  try {
    ImuManager imu_manager(configs);

    const auto loop_period = std::chrono::duration<double>(1.0 / DEFAULT_RATE_HZ);
    const auto start_time = std::chrono::steady_clock::now();

    auto previous_time = start_time;

    while (g_running) {
      const auto now = std::chrono::steady_clock::now();

      const float dt = std::chrono::duration<float>(now - previous_time).count();
      previous_time = now;
      const std::uint32_t time_boot_ms = static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count()
      );
      imu_manager.update(dt, time_boot_ms, mavlink_comm);

      std::this_thread::sleep_for(loop_period);
    }

  } catch (const std::exception &e) {
    std::cerr << "[ERROR] " << e.what() << '\n';
    mavlink_comm.send_error(e.what());
    mavlink_comm.stop();

    return EXIT_FAILURE;
  }

  mavlink_comm.send_info("IMU system stopped.");
  mavlink_comm.stop();

  return EXIT_SUCCESS;
}
