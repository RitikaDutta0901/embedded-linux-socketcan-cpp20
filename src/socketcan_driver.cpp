// SPDX-License-Identifier: MIT

#include "socketcan_driver.hpp"

#include <fcntl.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace vcansim {
namespace {

volatile std::sig_atomic_t g_shutdown_requested = 0;

void handle_shutdown_signal(int /*signal*/) noexcept {
    g_shutdown_requested = 1;
}

[[nodiscard]] int clamp_timeout_ms(std::chrono::milliseconds timeout) noexcept {
    const auto count = timeout.count();
    if (count <= 0) {
        return 0;
    }
    const auto max_ms = static_cast<long long>(
        std::numeric_limits<int>::max());
    return static_cast<int>(std::min<long long>(count, max_ms));
}

void throw_system_error(const char* operation, int error) {
    throw std::system_error{error, std::generic_category(), operation};
}

} // namespace

void install_shutdown_handlers() noexcept {
    struct sigaction action {};
    action.sa_handler = &handle_shutdown_signal;
    sigemptyset(&action.sa_mask);
    // Deliberately leave SA_RESTART clear so a blocking poll() is interrupted
    // promptly and the main loop can observe the shutdown flag.
    action.sa_flags = 0;
    (void)::sigaction(SIGINT, &action, nullptr);
    (void)::sigaction(SIGTERM, &action, nullptr);
}

bool shutdown_requested() noexcept {
    return g_shutdown_requested != 0;
}

void reset_shutdown_flag() noexcept {
    g_shutdown_requested = 0;
}

SocketCanDriver::SocketCanDriver(
    std::string_view interface_name,
    std::span<const can_filter> filters,
    std::chrono::milliseconds receive_timeout) {
    open(interface_name, filters, receive_timeout);
}

SocketCanDriver::~SocketCanDriver() {
    close();
}

SocketCanDriver::SocketCanDriver(SocketCanDriver&& other) noexcept
    : fd_{other.fd_},
      interface_name_{std::move(other.interface_name_)},
      receive_timeout_{other.receive_timeout_} {
    other.fd_ = -1;
}

SocketCanDriver&
SocketCanDriver::operator=(SocketCanDriver&& other) noexcept {
    if (this != &other) {
        close();
        fd_ = other.fd_;
        interface_name_ = std::move(other.interface_name_);
        receive_timeout_ = other.receive_timeout_;
        other.fd_ = -1;
    }
    return *this;
}

void SocketCanDriver::open(std::string_view interface_name,
                           std::span<const can_filter> filters,
                           std::chrono::milliseconds receive_timeout) {
    close();

    if (interface_name.empty()) {
        throw std::invalid_argument{"SocketCAN interface name is empty"};
    }
    if (interface_name.size() >= IFNAMSIZ) {
        throw std::invalid_argument{"SocketCAN interface name is too long"};
    }

    const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) {
        throw_system_error("socket(PF_CAN)", errno);
    }

    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        const int error = errno;
        ::close(fd);
        throw_system_error("fcntl(O_NONBLOCK)", error);
    }

    struct ifreq interface_request {};
    std::snprintf(interface_request.ifr_name,
                  sizeof(interface_request.ifr_name),
                  "%.*s",
                  static_cast<int>(interface_name.size()),
                  interface_name.data());

    if (::ioctl(fd, SIOCGIFINDEX, &interface_request) < 0) {
        const int error = errno;
        ::close(fd);
        throw_system_error("ioctl(SIOCGIFINDEX)", error);
    }

    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_request.ifr_ifindex;

    if (::bind(fd, reinterpret_cast<const struct sockaddr*>(&address),
               sizeof(address)) < 0) {
        const int error = errno;
        ::close(fd);
        throw_system_error("bind(AF_CAN)", error);
    }

    if (!filters.empty()) {
        const auto filter_bytes = static_cast<socklen_t>(
            filters.size() * sizeof(can_filter));
        if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                         filter_bytes) < 0) {
            const int error = errno;
            ::close(fd);
            throw_system_error("setsockopt(CAN_RAW_FILTER)", error);
        }
    }

    fd_ = fd;
    interface_name_.assign(interface_name);
    receive_timeout_ = receive_timeout;
}

void SocketCanDriver::close() noexcept {
    if (fd_ >= 0) {
        (void)::close(fd_);
        fd_ = -1;
    }
    interface_name_.clear();
}

bool SocketCanDriver::send(const can_frame& frame) noexcept {
    if (fd_ < 0) {
        return false;
    }

    constexpr ssize_t frame_size = static_cast<ssize_t>(sizeof(can_frame));
    constexpr int send_flags = MSG_DONTWAIT | MSG_NOSIGNAL;

    ssize_t written = ::send(fd_, &frame, sizeof(frame), send_flags);
    if (written == frame_size) {
        return true;
    }

    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
                        errno == ENOBUFS)) {
        // A CAN controller may briefly report a full TX queue.  Wait a short,
        // bounded amount of time before retrying once; this keeps the cyclic
        // sender from silently losing a frame without ever blocking forever.
        struct pollfd descriptor {};
        descriptor.fd = fd_;
        descriptor.events = POLLOUT;

        int poll_result = 0;
        do {
            poll_result = ::poll(&descriptor, 1, 5);
        } while (poll_result < 0 && errno == EINTR &&
                 !shutdown_requested());

        if (poll_result > 0) {
            written = ::send(fd_, &frame, sizeof(frame), send_flags);
            return written == frame_size;
        }
    }

    return false;
}

std::optional<can_frame>
SocketCanDriver::receive(std::chrono::milliseconds timeout) noexcept {
    if (fd_ < 0) {
        return std::nullopt;
    }

    const int timeout_ms = clamp_timeout_ms(timeout);
    struct pollfd descriptor {};
    descriptor.fd = fd_;
    descriptor.events = POLLIN;

    int poll_result = 0;
    do {
        poll_result = ::poll(&descriptor, 1, timeout_ms);
    } while (poll_result < 0 && errno == EINTR && !shutdown_requested());

    if (poll_result <= 0) {
        return std::nullopt;
    }
    if ((descriptor.revents & POLLIN) == 0) {
        return std::nullopt;
    }

    can_frame frame{};
    const ssize_t received =
        ::recv(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
    if (received != static_cast<ssize_t>(sizeof(can_frame))) {
        return std::nullopt;
    }
    return frame;
}

std::optional<can_frame> SocketCanDriver::receive() noexcept {
    return receive(receive_timeout_);
}

} // namespace vcansim
