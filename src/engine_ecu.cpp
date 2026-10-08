// SPDX-License-Identifier: MIT
//
// VCanSim - simulated engine ECU.
//
// The ECU advances a lightweight, deterministic vehicle model at 10 ms
// intervals and broadcasts:
//   * 0x100 Engine telemetry    every 10 ms
//   * 0x200 Diagnostics/faults  every 100 ms
//
// No heap allocation occurs in the cyclic loop.

#include "can_protocol.hpp"
#include "socketcan_driver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

constexpr auto kEnginePeriod = 10ms;
constexpr auto kDiagnosticPeriod = 100ms;
constexpr int kDiagnosticEveryNthTick =
    static_cast<int>(kDiagnosticPeriod / kEnginePeriod);

constexpr double kPi = 3.14159265358979323846;
constexpr double kIdleRpm = 800.0;
constexpr double kMaxRpm = 6800.0;
constexpr double kFinalDrive = 3.70;
constexpr double kWheelCircumferenceM = 2.0;
constexpr std::array<double, 6> kGearRatios{
    3.55, 2.10, 1.40, 1.00, 0.80, 0.65};

struct SimulationConfig {
    std::string interface_name{"vcan0"};
    double cooling_fault_after_s{0.0}; // <= 0 disables fault injection
};

struct EngineState {
    double elapsed_s{0.0};
    double rpm{kIdleRpm};
    double coolant_c{22.0};
    double throttle_pct{0.0};
    double speed_kph{0.0};
    int gear{1};
    std::uint8_t alive_counter{0};
    bool cooling_fault{false};
    bool overheat{false};
    bool mil{false};
};

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "\n"
        << "Options:\n"
        << "  --iface <name>         SocketCAN interface (default: vcan0)\n"
        << "  --fault-after <sec>    Inject a cooling failure after N seconds\n"
        << "                         (0 = never, default: 0)\n"
        << "  -h, --help             Show this help text\n";
}

[[nodiscard]] std::optional<SimulationConfig>
parse_arguments(int argc, char** argv) {
    SimulationConfig config;

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
        if (argument == "--fault-after") {
            if (++index >= argc) {
                throw std::invalid_argument{
                    "--fault-after requires a number of seconds"};
            }
            const std::string value{argv[index]};
            char* end = nullptr;
            const double seconds = std::strtod(value.c_str(), &end);
            if (end == value.c_str() || *end != '\0' || !std::isfinite(seconds) ||
                seconds < 0.0) {
                throw std::invalid_argument{
                    "invalid value for --fault-after (expected >= 0 seconds)"};
            }
            config.cooling_fault_after_s = seconds;
            continue;
        }

        throw std::invalid_argument{"unknown argument: " +
                                    std::string{argument}};
    }

    return config;
}

/// Deterministic 32-second urban drive cycle.
[[nodiscard]] double drive_cycle_throttle(double elapsed_s) noexcept {
    const double phase = std::fmod(elapsed_s, 32.0);
    if (phase < 2.0) {
        return 0.0;
    }
    if (phase < 10.0) {
        return 45.0;
    }
    if (phase < 13.0) {
        return 20.0;
    }
    if (phase < 21.0) {
        return 75.0;
    }
    if (phase < 26.0) {
        return 55.0;
    }
    return 0.0;
}

/// Engine speed implied by road speed, selected gear, and driveline ratios.
[[nodiscard]] double drivetrain_rpm(double speed_kph, int gear) noexcept {
    const double speed_mps = speed_kph / 3.6;
    const double wheel_rps = speed_mps / kWheelCircumferenceM;
    return wheel_rps * kFinalDrive * kGearRatios[gear - 1] * 60.0;
}

void advance_physics(EngineState& state,
                     double dt,
                     double cooling_fault_after_s) noexcept {
    state.elapsed_s += dt;

    if (cooling_fault_after_s > 0.0 && !state.cooling_fault &&
        state.elapsed_s >= cooling_fault_after_s) {
        state.cooling_fault = true;
    }

    // Throttle actuator follows the drive-cycle demand with a slew limit.
    const double requested_throttle = drive_cycle_throttle(state.elapsed_s);
    constexpr double kThrottleSlewPctPerSecond = 60.0;
    const double throttle_delta =
        std::clamp(requested_throttle - state.throttle_pct,
                   -kThrottleSlewPctPerSecond * dt,
                   kThrottleSlewPctPerSecond * dt);
    state.throttle_pct += throttle_delta;

    // Longitudinal vehicle dynamics.
    if (state.speed_kph < 0.5 && state.throttle_pct < 1.0) {
        state.speed_kph = 0.0;
    } else {
        const double gear_factor = 1.0 / (0.55 + 0.30 * state.gear);
        const double traction_mps2 =
            (state.throttle_pct / 100.0) * 4.5 * gear_factor;
        const double aerodynamic_mps2 =
            0.00012 * state.speed_kph * state.speed_kph;
        const double rolling_mps2 = state.speed_kph > 0.1 ? 0.15 : 0.0;
        const double acceleration_mps2 =
            traction_mps2 - aerodynamic_mps2 - rolling_mps2;
        state.speed_kph = std::clamp(
            state.speed_kph + acceleration_mps2 * 3.6 * dt, 0.0, 220.0);
    }

    // Automatic gearbox: upshift near 2900 rpm, downshift when coasting.
    const double wheel_rpm = drivetrain_rpm(state.speed_kph, state.gear);
    if (state.gear < 6 && wheel_rpm > 2900.0 && state.throttle_pct > 10.0) {
        ++state.gear;
    } else if (state.gear > 1 && wheel_rpm < 1150.0 &&
               state.throttle_pct < 15.0) {
        --state.gear;
    }
    if (state.speed_kph < 2.0) {
        state.gear = 1;
    }

    // Engine speed: idle governor plus driveline coupling, first-order lag.
    const double target_rpm =
        std::max(kIdleRpm + state.throttle_pct * 8.0,
                 drivetrain_rpm(state.speed_kph, state.gear));
    constexpr double kRpmTimeConstantS = 0.18;
    state.rpm += (target_rpm - state.rpm) *
                 std::min(1.0, dt / kRpmTimeConstantS);
    state.rpm = std::clamp(state.rpm, 750.0, kMaxRpm);

    // Coolant thermal model.  A failed thermostat raises the equilibrium
    // temperature and slowly pushes the engine past the 110 degC threshold.
    const double load = state.throttle_pct / 100.0;
    double equilibrium_c = 0.0;
    double time_constant_s = 0.0;
    if (state.cooling_fault) {
        equilibrium_c = 140.0 + load * 10.0;
        time_constant_s = 30.0;
    } else {
        equilibrium_c = 88.0 + load * 10.0;
        time_constant_s = 35.0;
    }
    state.coolant_c += (equilibrium_c - state.coolant_c) *
                       std::min(1.0, dt / time_constant_s);

    state.overheat = state.coolant_c > 110.0;
    state.mil = state.overheat;
}

[[nodiscard]] std::uint16_t battery_millivolts(
    const EngineState& state) noexcept {
    const double base_mv = 13650.0 + 500.0 * (state.rpm / kMaxRpm);
    const double ripple_mv =
        40.0 * std::sin(state.elapsed_s * 2.0 * kPi * 3.0);
    const double sag_mv = state.overheat ? 750.0 : 0.0;
    const double value_mv =
        std::clamp(base_mv + ripple_mv - sag_mv, 0.0, 65535.0);
    return static_cast<std::uint16_t>(std::lround(value_mv));
}

[[nodiscard]] can_frame make_frame(std::uint32_t identifier) noexcept {
    can_frame frame{};
    frame.can_id = identifier;
    frame.can_dlc = static_cast<__u8>(vcansim::kCanPayloadSize);
    return frame;
}

int run(const SimulationConfig& config) {
    vcansim::install_shutdown_handlers();
    vcansim::SocketCanDriver driver{config.interface_name};

    std::cerr << "engine_ecu: broadcasting on " << config.interface_name
              << " (0x100 every 10 ms, 0x200 every 100 ms)";
    if (config.cooling_fault_after_s > 0.0) {
        std::cerr << ", cooling fault after "
                  << config.cooling_fault_after_s << " s";
    }
    std::cerr << '\n';

    EngineState state{};
    can_frame engine_frame = make_frame(vcansim::kEngineTelemetryId);
    can_frame diagnostic_frame = make_frame(vcansim::kDiagnosticId);
    int diagnostic_divider = 0;
    bool send_failed = false;

    auto next_engine_tick = Clock::now();

    while (!vcansim::shutdown_requested()) {
        const auto now = Clock::now();
        if (now < next_engine_tick) {
            std::this_thread::sleep_until(next_engine_tick);
            continue;
        }

        advance_physics(state, 0.01, config.cooling_fault_after_s);

        vcansim::pack_engine_frame(
            vcansim::EngineTelemetry{state.rpm, state.coolant_c,
                                     state.throttle_pct, state.speed_kph},
            std::span<std::uint8_t, vcansim::kCanPayloadSize>{
                engine_frame.data, vcansim::kCanPayloadSize});

        send_failed = !driver.send(engine_frame) || send_failed;

        if (++diagnostic_divider >= kDiagnosticEveryNthTick) {
            diagnostic_divider = 0;

            vcansim::DiagnosticTelemetry diagnostics{};
            diagnostics.mil = state.mil;
            diagnostics.overheat = state.overheat;
            diagnostics.battery_mv = battery_millivolts(state);
            diagnostics.alive_counter = state.alive_counter;

            vcansim::pack_diagnostic_frame(
                diagnostics,
                std::span<std::uint8_t, vcansim::kCanPayloadSize>{
                    diagnostic_frame.data, vcansim::kCanPayloadSize});

            send_failed = !driver.send(diagnostic_frame) || send_failed;

            state.alive_counter = static_cast<std::uint8_t>(
                (state.alive_counter + 1U) & vcansim::kAliveCounterMask);
        }

        next_engine_tick += kEnginePeriod;
        if (next_engine_tick <= now) {
            // Recover from a scheduling pause without emitting a burst of
            // backlog frames.
            next_engine_tick = now + kEnginePeriod;
        }
    }

    if (send_failed) {
        std::cerr << "engine_ecu: warning: one or more CAN frames could not "
                     "be sent\n";
    }
    std::cerr << "engine_ecu: shutdown requested, exiting\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto config = parse_arguments(argc, argv);
        if (!config.has_value()) {
            return 0;
        }
        return run(*config);
    } catch (const std::exception& error) {
        std::cerr << "engine_ecu: " << error.what() << '\n';
        return 1;
    }
}
