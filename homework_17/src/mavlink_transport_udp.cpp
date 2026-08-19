#include "mavlink_transport_udp.hpp"
#include <cstring>
#include <stdexcept>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

MavlinkTransportUDP::MavlinkTransportUDP(const std::string& ip, uint16_t port) {
    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0) {
        throw std::runtime_error("socket() failed");
    }

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        close(sockfd);
        sockfd = -1;
        throw std::runtime_error("Invalid IP address: " + ip);
    }
}

int MavlinkTransportUDP::send(const uint8_t* buf, size_t len) {
    return sendto(
        sockfd,
        buf,
        len,
        0,
        reinterpret_cast<const sockaddr*>(&addr),
        sizeof(addr)
    );
}

int MavlinkTransportUDP::receive(uint8_t* buf, size_t len) {
    sockaddr_in src_addr {};
    socklen_t src_addr_len = sizeof(src_addr);

    return recvfrom(
        sockfd,
        buf,
        len,
        0,
        reinterpret_cast<sockaddr*>(&src_addr),
        &src_addr_len
    );
}