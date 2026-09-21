#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "mavlink_transport_udp.hpp"

MavlinkTransportUDP::MavlinkTransportUDP(const char *ip, uint16_t port) {
    if (ip == nullptr) {
        throw std::invalid_argument("IP address string cannot be null");
    }

    sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0) {
        throw std::runtime_error("socket() failed: " + std::string(std::strerror(errno)));
    }

    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    if (::setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        ::close(sockfd);
        sockfd = -1;
        throw std::runtime_error("setsockopt(SO_RCVTIMEO) failed: " + std::string(std::strerror(errno)));
    }

    timeval snd_timeout{};
    snd_timeout.tv_sec = 0;
    snd_timeout.tv_usec = 100'000;

    if (::setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout)) < 0) {
        ::close(sockfd);
        sockfd = -1;
        throw std::runtime_error("setsockopt(SO_SNDTIMEO) failed: " + std::string(std::strerror(errno)));
    }

    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (::inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) {
        ::close(sockfd);
        sockfd = -1;
        throw std::runtime_error("Invalid IP address provided: " + std::string(ip));
    }
}

MavlinkTransportUDP::~MavlinkTransportUDP() {
    if (sockfd >= 0) {
        ::close(sockfd);
    }
}

MavlinkTransportUDP::MavlinkTransportUDP(MavlinkTransportUDP &&other) noexcept
    : sockfd(std::exchange(other.sockfd, -1)), addr(other.addr) {}

MavlinkTransportUDP &
MavlinkTransportUDP::operator=(MavlinkTransportUDP &&other) noexcept {
    if (this == &other) {
        return *this;
    }

    if (sockfd >= 0) {
        ::close(sockfd);
    }

    sockfd = std::exchange(other.sockfd, -1);
    addr = other.addr;

    return *this;
}

int MavlinkTransportUDP::send(const uint8_t *buf, size_t len) {
    if (sockfd < 0 || buf == nullptr || len == 0) {
        return -1;
    }

    const ssize_t bytes_sent = ::sendto(sockfd, buf, len, 0, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr));

    if (bytes_sent < 0) {
        return -1;
    }

    return static_cast<int>(bytes_sent);
}

int MavlinkTransportUDP::receive(uint8_t *buf, size_t len) {
    if (sockfd < 0 || buf == nullptr || len == 0) {
        return -1;
    }

    sockaddr_in src_addr{};
    socklen_t src_addr_len = sizeof(src_addr);

    const ssize_t bytes_received = ::recvfrom(sockfd, buf, len, 0, reinterpret_cast<sockaddr *>(&src_addr), &src_addr_len);

    if (bytes_received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }

    return static_cast<int>(bytes_received);
}