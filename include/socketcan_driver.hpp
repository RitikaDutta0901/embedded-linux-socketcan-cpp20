// SPDX-License-Identifier: MIT
//
// VCanSim - RAII wrapper around a Linux SocketCAN raw socket.

#ifndef VCANSIM_SOCKETCAN_DRIVER_HPP
#define VCANSIM_SOCKETCAN_DRIVER_HPP

#include <linux/can.h>

#include <chrono>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace vcansim {

/// Install SIGINT/SIGTERM handlers that flip the internal shutdown flag.
/// The handlers are async-signal-safe and only store a sig_atomic_t.
void install_shutdown_handlers() noexcept;

/// True once SIGINT or SIGTERM has been observed.
[[nodiscard]] bool shutdown_requested() noexcept;

/// Clear the shutdown flag (useful for tests or repeated runs).
void reset_shutdown_flag() noexcept;

/// Build an exact-match standard-ID kernel filter.
[[nodiscard]] constexpr can_filter
exact_can_filter(canid_t id) noexcept {
    return can_filter{id, CAN_SFF_MASK};
}

/// Owns a CAN_RAW socket and its interface binding.  Setup failures throw
/// std::system_error; the real-time send/receive paths are noexcept and never
/// allocate memory.
class SocketCanDriver {
public:
    SocketCanDriver() noexcept = default;

    explicit SocketCanDriver(
        std::string_view interface_name,
        std::span<const can_filter> filters = {},
        std::chrono::milliseconds receive_timeout =
            std::chrono::milliseconds{100});

    ~SocketCanDriver();

    SocketCanDriver(const SocketCanDriver&) = delete;
    SocketCanDriver& operator=(const SocketCanDriver&) = delete;

    SocketCanDriver(SocketCanDriver&& other) noexcept;
    SocketCanDriver& operator=(SocketCanDriver&& other) noexcept;

    /// Open and bind the interface, apply optional kernel-level filters, and
    /// switch the socket to non-blocking mode.
    void open(std::string_view interface_name,
              std::span<const can_filter> filters = {},
              std::chrono::milliseconds receive_timeout =
                  std::chrono::milliseconds{100});

    void close() noexcept;

    [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }
    [[nodiscard]] int native_handle() const noexcept { return fd_; }
    [[nodiscard]] const std::string& interface_name() const noexcept {
        return interface_name_;
    }

    /// Send one classic CAN frame.  Returns false on EAGAIN/ENOBUFS or any
    /// other socket error; never throws.
    [[nodiscard]] bool send(const can_frame& frame) noexcept;

    /// Wait up to `timeout` for a frame.  A zero timeout performs a
    /// non-blocking poll.  Returns std::nullopt on timeout, EINTR, or error.
    [[nodiscard]] std::optional<can_frame>
    receive(std::chrono::milliseconds timeout) noexcept;

    /// Receive using the timeout supplied at open() time.
    [[nodiscard]] std::optional<can_frame> receive() noexcept;

private:
    int fd_{-1};
    std::string interface_name_{};
    std::chrono::milliseconds receive_timeout_{100};
};

} // namespace vcansim

#endif // VCANSIM_SOCKETCAN_DRIVER_HPP
