// SPDX-License-Identifier: MIT
//
// VCanSim - real-time terminal dashboard.
//
// Receives only 0x100 and 0x200 using kernel-level CAN_RAW filters, validates
// the diagnostic alive counter and CRC-8, and renders an ANSI dashboard at
// approximately 10 Hz.  The receive path performs no heap allocations.

#include "can_protocol.hpp"
#include "socketcan_driver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

constexpr const char* kReset = "\x1b[0m";
constexpr const char* kBoldCyan = "\x1b[1;36m";
constexpr const char* kCyan = "\x1b[36m";
constexpr const char* kGreen = "\x1b[1;32m";
constexpr const char* kRed = "\x1b[1;31m";
constexpr const char* kYellow = "\x1b[1;33m";
constexpr const char* kDim = "\x1b[2m";

constexpr double kMaxRpmDisplay = 7000.0;
constexpr double kMaxCoolantDisplay = 130.0;
constexpr double kMaxThrottleDisplay = 100.0;
constexpr double kMaxSpeedDisplay = 220.0;
constexpr double kMaxBatteryDisplayMv = 16000.0;

struct Config {
    std::string interface_name{"vcan0"};
};

struct DashboardState {
    vcansim::EngineTelemetry engine{};
    vcansim::DiagnosticTelemetry diagnostics{};

    Clock::time_point started{Clock::now()};
    Clock::time_point last_engine{};
    Clock::time_point last_diagnostic{};

    bool have_engine{false};
    bool have_diagnostic{false};
    bool have_last_alive{false};
    bool last_crc_ok{true};

    std::uint8_t last_alive{0};
    std::uint64_t engine_frames{0};
    std::uint64_t diagnostic_frames{0};
    std::uint64_t dropped_diagnostic{0};
    std::uint64_t crc_errors{0};
    std::uint64_t malformed_frames{0};
};

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [options]\n"
              << "\n"
              << "Options:\n"
              << "  --iface <name>   SocketCAN interface (default: vcan0)\n"
              << "  -h, --help       Show this help text\n";
}

[[nodiscard]] std::optional<Config> parse_arguments(int argc, char** argv) {
    Config config;
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument{argv[index]};
        if (argument == "-h" || argument == "--help") {
            print_usage(argv[0]);
            return std::nullopt;
        }
        if (argument == "--iface") {
            if (++index >= argc) {
                throw std::invalid_argument{"--iface requires an argument"};
            }
            config.interface_name = argv[index];
            continue;
        }
        throw std::invalid_argument{"unknown argument: " +
                                    std::string{argument}};
    }
    return config;
}

class TextBuffer {
public:
    /// Append a literal string without printf-style interpretation.
    void append(const char* text) noexcept {
        if (text == nullptr) {
            return;
        }
        while (*text != '\0' && size_ + 1U < data_.size()) {
            data_[size_++] = *text++;
        }
    }

    template <typename... Args>
    void append(const char* format, Args... args) noexcept {
        if (size_ + 1U >= data_.size()) {
            return;
        }
        const std::size_t remaining = data_.size() - size_;
        const int written = std::snprintf(data_.data() + size_, remaining,
                                          format, args...);
        if (written > 0) {
            size_ += std::min<std::size_t>(
                static_cast<std::size_t>(written), remaining - 1U);
        }
    }

    [[nodiscard]] std::span<const char> view() const noexcept {
        return {data_.data(), size_};
    }

private:
    std::array<char, 8192> data_{};
    std::size_t size_{0};
};

class CursorGuard {
public:
    CursorGuard() { std::cout << "\x1b[?25l" << std::flush; }
    ~CursorGuard() { std::cout << "\x1b[?25h" << kReset << std::flush; }

    CursorGuard(const CursorGuard&) = delete;
    CursorGuard& operator=(const CursorGuard&) = delete;
};

[[nodiscard]] std::array<char, 32>
make_bar(double value, double maximum, int width = 20) noexcept {
    std::array<char, 32> bar{};
    width = std::clamp(width, 0, 28);
    const double ratio =
        maximum > 0.0 ? std::clamp(value / maximum, 0.0, 1.0) : 0.0;
    const int filled = static_cast<int>(
        std::lround(ratio * static_cast<double>(width)));

    std::size_t position = 0;
    bar[position++] = '[';
    for (int column = 0; column < width; ++column) {
        bar[position++] = column < filled ? '#' : '-';
    }
    bar[position++] = ']';
    bar[position] = '\0';
    return bar;
}

void append_rule(TextBuffer& out, int width = 74) noexcept {
    for (int column = 0; column < width; ++column) {
        out.append("-");
    }
    out.append("\x1b[K\n");
}

struct AgeText {
    std::array<char, 24> text{};
};

[[nodiscard]] AgeText format_age(bool have_value,
                                 Clock::time_point last,
                                 Clock::time_point now) noexcept {
    AgeText age{};
    if (!have_value) {
        std::snprintf(age.text.data(), age.text.size(), "   ---");
        return age;
    }
    auto age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last)
            .count();
    if (age_ms < 0) {
        age_ms = 0;
    }
    std::snprintf(age.text.data(), age.text.size(), "%5lld ms",
                  static_cast<long long>(age_ms));
    return age;
}

void process_frame(const can_frame& frame,
                   DashboardState& state,
                   Clock::time_point now) noexcept {
    if (static_cast<std::size_t>(frame.can_dlc) != vcansim::kCanPayloadSize) {
        ++state.malformed_frames;
        return;
    }

    const std::span<const std::uint8_t, vcansim::kCanPayloadSize> payload{
        frame.data, vcansim::kCanPayloadSize};

    if (frame.can_id == vcansim::kEngineTelemetryId) {
        state.engine = vcansim::unpack_engine_frame(payload);
        state.last_engine = now;
        state.have_engine = true;
        ++state.engine_frames;
        return;
    }

    if (frame.can_id == vcansim::kDiagnosticId) {
        ++state.diagnostic_frames;
        const auto diagnostics = vcansim::unpack_diagnostic_frame(payload);
        if (!diagnostics.has_value()) {
            ++state.crc_errors;
            state.last_crc_ok = false;
            return;
        }

        state.last_crc_ok = true;

        if (state.have_last_alive) {
            const std::uint8_t expected = static_cast<std::uint8_t>(
                (state.last_alive + 1U) & vcansim::kAliveCounterMask);
            const std::uint8_t missed = static_cast<std::uint8_t>(
                (diagnostics->alive_counter - expected) &
                vcansim::kAliveCounterMask);
            state.dropped_diagnostic += missed;
        }

        state.last_alive = diagnostics->alive_counter;
        state.have_last_alive = true;
        state.diagnostics = *diagnostics;
        state.last_diagnostic = now;
        state.have_diagnostic = true;
    }
}

void render_dashboard(const DashboardState& state,
                      Clock::time_point now,
                      std::string_view interface_name) {
    const bool engine_fresh =
        state.have_engine && (now - state.last_engine) < 500ms;
    const bool diagnostic_fresh =
        state.have_diagnostic && (now - state.last_diagnostic) < 1500ms;
    const bool link_ok = engine_fresh && diagnostic_fresh;
    const char* link_text =
        link_ok ? "ACTIVE" : ((state.have_engine || state.have_diagnostic)
                                  ? "STALE"
                                  : "NO SIGNAL");
    const char* link_color = link_ok ? kGreen : kRed;
    const double uptime_s =
        std::chrono::duration<double>(now - state.started).count();

    const AgeText engine_age =
        format_age(state.have_engine, state.last_engine, now);
    const AgeText diagnostic_age =
        format_age(state.have_diagnostic, state.last_diagnostic, now);

    TextBuffer out;
    out.append("\x1b[H");
    out.append("%s VCanSim :: Virtual SocketCAN ECU & Telemetry Simulator%s"
               "\x1b[K\n",
               kBoldCyan, kReset);
    append_rule(out);

    out.append(" Interface : %.*s", static_cast<int>(interface_name.size()),
               interface_name.data());
    out.append("        Uptime: %8.1f s        Link: %s%s%s\x1b[K\n", uptime_s,
               link_color, link_text, kReset);
    append_rule(out);

    out.append(" ENGINE TELEMETRY  (0x100 @ 100 Hz)\x1b[K\n");
    if (state.have_engine) {
        const auto rpm_bar =
            make_bar(state.engine.rpm, kMaxRpmDisplay);
        const auto coolant_bar =
            make_bar(state.engine.coolant_c, kMaxCoolantDisplay);
        const auto throttle_bar =
            make_bar(state.engine.throttle_pct, kMaxThrottleDisplay);
        const auto speed_bar =
            make_bar(state.engine.speed_kph, kMaxSpeedDisplay);
        const bool overheat = state.engine.coolant_c > 110.0;

        out.append("   %-9s %s%s%s %6.0f rpm\x1b[K\n", "RPM", kCyan,
                   rpm_bar.data(), kReset, state.engine.rpm);
        out.append("   %-9s %s%s%s %6.1f C%s\x1b[K\n", "Coolant",
                   overheat ? kRed : kCyan, coolant_bar.data(), kReset,
                   state.engine.coolant_c, overheat ? "  (OVERHEAT)" : "");
        out.append("   %-9s %s%s%s %6.1f %%\x1b[K\n", "Throttle", kCyan,
                   throttle_bar.data(), kReset, state.engine.throttle_pct);
        out.append("   %-9s %s%s%s %6.1f km/h\x1b[K\n", "Speed", kCyan,
                   speed_bar.data(), kReset, state.engine.speed_kph);
    } else {
        out.append("   %sWaiting for engine telemetry...%s\x1b[K\n", kDim,
                   kReset);
    }
    append_rule(out);

    out.append(" DIAGNOSTICS & FAULTS  (0x200 @ 10 Hz)\x1b[K\n");
    if (state.have_diagnostic) {
        const auto battery_bar = make_bar(
            static_cast<double>(state.diagnostics.battery_mv),
            kMaxBatteryDisplayMv);
        const char* crc_text = state.last_crc_ok ? "OK" : "ERR";
        const char* crc_color = state.last_crc_ok ? kGreen : kRed;

        out.append("   %-9s %s%s%s %6.2f V\x1b[K\n", "Battery", kCyan,
                   battery_bar.data(), kReset,
                   static_cast<double>(state.diagnostics.battery_mv) / 1000.0);
        out.append("   MIL: %s%-3s%s        Overheat: %s%-3s%s"
                   "        Alive: %2u        CRC: %s%-3s%s\x1b[K\n",
                   state.diagnostics.mil ? kRed : kGreen,
                   state.diagnostics.mil ? "ON" : "OFF", kReset,
                   state.diagnostics.overheat ? kRed : kGreen,
                   state.diagnostics.overheat ? "ON" : "OFF", kReset,
                   static_cast<unsigned>(state.diagnostics.alive_counter),
                   crc_color, crc_text, kReset);

        if (state.diagnostics.overheat) {
            out.append("   %s!! OVERHEAT WARNING - COOLANT ABOVE 110 C !!%s"
                       "\x1b[K\n",
                       kRed, kReset);
        }
        if (state.diagnostics.mil) {
            out.append("   %s!! CHECK ENGINE MIL ACTIVE !!%s\x1b[K\n", kYellow,
                       kReset);
        }
    } else {
        out.append("   %sWaiting for diagnostic frames...%s\x1b[K\n", kDim,
                   kReset);
    }
    append_rule(out);

    out.append(" LINK STATISTICS\x1b[K\n");
    out.append("   Frames    engine=%llu  diag=%llu  malformed=%llu"
               "\x1b[K\n",
               static_cast<unsigned long long>(state.engine_frames),
               static_cast<unsigned long long>(state.diagnostic_frames),
               static_cast<unsigned long long>(state.malformed_frames));
    out.append("   Errors    dropped_diag=%llu  crc_errors=%s%llu%s"
               "\x1b[K\n",
               static_cast<unsigned long long>(state.dropped_diagnostic),
               state.crc_errors > 0 ? kRed : kGreen,
               static_cast<unsigned long long>(state.crc_errors), kReset);
    out.append("   Age       engine=%s  diag=%s\x1b[K\n",
               engine_age.text.data(), diagnostic_age.text.data());
    append_rule(out);
    out.append(" %sPress Ctrl+C to exit%s\x1b[J", kDim, kReset);

    const auto view = out.view();
    std::cout.write(view.data(),
                    static_cast<std::streamsize>(view.size()));
    std::cout.flush();
}

int run(const Config& config) {
    vcansim::install_shutdown_handlers();

    const std::array<can_filter, 2> filters{
        vcansim::exact_can_filter(vcansim::kEngineTelemetryId),
        vcansim::exact_can_filter(vcansim::kDiagnosticId)};

    vcansim::SocketCanDriver driver{config.interface_name, filters, 50ms};
    CursorGuard cursor_guard;

    DashboardState state{};
    state.started = Clock::now();
    auto last_render = state.started;
    bool first_render = true;

    while (!vcansim::shutdown_requested()) {
        if (auto frame = driver.receive(50ms)) {
            do {
                process_frame(*frame, state, Clock::now());
            } while ((frame = driver.receive(0ms)));
        }

        const auto now = Clock::now();
        if (first_render || (now - last_render) >= 100ms) {
            render_dashboard(state, now, config.interface_name);
            last_render = now;
            first_render = false;
        }
    }

    std::cout << "\n" << kDim << "dashboard_rx: shutdown requested, exiting"
              << kReset << "\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::ios::sync_with_stdio(false);
    std::cout.tie(nullptr);

    try {
        const auto config = parse_arguments(argc, argv);
        if (!config.has_value()) {
            return 0;
        }
        return run(*config);
    } catch (const std::exception& error) {
        std::cerr << "dashboard_rx: " << error.what() << '\n';
        return 1;
    }
}
