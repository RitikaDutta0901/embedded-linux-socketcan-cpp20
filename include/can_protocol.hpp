// SPDX-License-Identifier: MIT
//
// VCanSim - CAN application protocol (header-only)
//
// Classic CAN payloads are always treated as big-endian ("network byte
// order"), regardless of the host CPU.  All helpers are allocation-free and
// suitable for use from hard real-time contexts.
//
// Frame 0x100 - Engine telemetry (10 ms cycle)
//   Byte 0..1 : Engine speed              uint16 BE, 0.25 rpm/bit
//   Byte 2    : Coolant temperature       uint8,   1 degC/bit, offset -40 degC
//   Byte 3    : Throttle position         uint8,   0.5 %/bit
//   Byte 4..5 : Vehicle speed             uint16 BE, 0.01 km/h/bit
//   Byte 6..7 : Reserved (transmit as 0x00)
//
// Frame 0x200 - Diagnostics & faults (100 ms cycle)
//   Byte 0    : Flag byte
//                 bit 0 : Check-engine MIL
//                 bit 1 : Overheat warning (coolant > 110 degC)
//                 bit 2..7 : Reserved (0)
//   Byte 1..2 : Battery voltage           uint16 BE, 1 mV/bit
//   Byte 3..5 : Reserved (transmit as 0x00)
//   Byte 6    : Rolling alive counter     4-bit, low nibble (high nibble = 0)
//   Byte 7    : CRC-8/SAE-J1850 over bytes 0..6
//
// The alive counter is a 4-bit rolling counter directly preceding the CRC-8
// byte.  Placing the two fields in the final two bytes keeps the counter and
// checksum adjacent while leaving a full 8 bits for the CRC.

#ifndef VCANSIM_CAN_PROTOCOL_HPP
#define VCANSIM_CAN_PROTOCOL_HPP

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace vcansim {

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

inline constexpr std::uint32_t kEngineTelemetryId = 0x100U;
inline constexpr std::uint32_t kDiagnosticId = 0x200U;

inline constexpr std::size_t kCanPayloadSize = 8U;

// Engine telemetry scaling
inline constexpr double kRpmPerBit = 0.25;
inline constexpr double kCoolantOffsetC = -40.0;
inline constexpr double kThrottlePercentPerBit = 0.5;
inline constexpr double kSpeedKphPerBit = 0.01;

// Diagnostic frame layout
inline constexpr std::uint8_t kFlagMil = 0x01U;
inline constexpr std::uint8_t kFlagOverheat = 0x02U;
inline constexpr std::uint8_t kAliveCounterMask = 0x0FU;

static_assert(sizeof(std::uint16_t) == 2U,
              "the protocol requires 16-bit uint16_t");
static_assert(std::endian::native == std::endian::little ||
                  std::endian::native == std::endian::big,
              "mixed-endian hosts are not supported");

// ---------------------------------------------------------------------------
// Deterministic big-endian helpers
// ---------------------------------------------------------------------------

/// Serialise a 16-bit value into network byte order.
[[nodiscard]] constexpr std::array<std::uint8_t, 2>
to_be16(std::uint16_t value) noexcept {
    if constexpr (std::endian::native == std::endian::little) {
        const auto host = std::bit_cast<std::array<std::uint8_t, 2>>(value);
        return {host[1], host[0]};
    } else {
        return std::bit_cast<std::array<std::uint8_t, 2>>(value);
    }
}

/// Deserialise a network-byte-order 16-bit value.
[[nodiscard]] constexpr std::uint16_t
from_be16(std::array<std::uint8_t, 2> bytes) noexcept {
    if constexpr (std::endian::native == std::endian::little) {
        bytes = {bytes[1], bytes[0]};
    }
    return std::bit_cast<std::uint16_t>(bytes);
}

/// Quantise a physical value to an unsigned 16-bit raw value, saturating at
/// the representable range.  Non-finite input is mapped to zero.
[[nodiscard]] inline std::uint16_t quantize_u16(double value) noexcept {
    if (!std::isfinite(value)) {
        return 0U;
    }
    const double clamped = std::clamp(value, 0.0, 65535.0);
    return static_cast<std::uint16_t>(std::lround(clamped));
}

/// Quantise a physical value to an unsigned 8-bit raw value.
[[nodiscard]] inline std::uint8_t quantize_u8(double value) noexcept {
    if (!std::isfinite(value)) {
        return 0U;
    }
    const double clamped = std::clamp(value, 0.0, 255.0);
    return static_cast<std::uint8_t>(std::lround(clamped));
}

// ---------------------------------------------------------------------------
// CRC-8/SAE-J1850
//   polynomial : 0x1D (x^8 + x^4 + x^3 + x^2 + 1)
//   init       : 0xFF
//   xor-out    : 0xFF
//   reflected  : no
// ---------------------------------------------------------------------------

[[nodiscard]] constexpr std::uint8_t
crc8_sae_j1850(std::span<const std::uint8_t> data) noexcept {
    std::uint8_t crc = 0xFFU;
    for (const std::uint8_t byte : data) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = static_cast<std::uint8_t>((crc << 1U) ^ 0x1DU);
            } else {
                crc = static_cast<std::uint8_t>(crc << 1U);
            }
        }
    }
    return static_cast<std::uint8_t>(crc ^ 0xFFU);
}

// ---------------------------------------------------------------------------
// Engineering-unit data structures
// ---------------------------------------------------------------------------

struct EngineTelemetry {
    double rpm{0.0};              ///< Engine speed in revolutions per minute
    double coolant_c{-40.0};      ///< Coolant temperature in degrees Celsius
    double throttle_pct{0.0};     ///< Throttle position, 0..127.5 %
    double speed_kph{0.0};        ///< Vehicle speed, 0..655.35 km/h
};

struct DiagnosticTelemetry {
    bool mil{false};              ///< Check-engine malfunction indicator lamp
    bool overheat{false};         ///< Coolant over-temperature warning
    std::uint16_t battery_mv{0};  ///< Battery voltage in millivolts
    std::uint8_t alive_counter{0};///< Rolling 4-bit counter (0..15)
};

// ---------------------------------------------------------------------------
// Engine telemetry frame 0x100
// ---------------------------------------------------------------------------

void pack_engine_frame(
    const EngineTelemetry& telemetry,
    std::span<std::uint8_t, kCanPayloadSize> out) noexcept;

[[nodiscard]] EngineTelemetry unpack_engine_frame(
    std::span<const std::uint8_t, kCanPayloadSize> in) noexcept;

// ---------------------------------------------------------------------------
// Diagnostic frame 0x200
// ---------------------------------------------------------------------------

void pack_diagnostic_frame(
    const DiagnosticTelemetry& telemetry,
    std::span<std::uint8_t, kCanPayloadSize> out) noexcept;

/// Verify the CRC-8 carried in byte 7 against bytes 0..6.
[[nodiscard]] bool verify_diagnostic_crc(
    std::span<const std::uint8_t, kCanPayloadSize> in) noexcept;

/// Decode a diagnostic frame.  Returns std::nullopt when the CRC is invalid.
[[nodiscard]] std::optional<DiagnosticTelemetry> unpack_diagnostic_frame(
    std::span<const std::uint8_t, kCanPayloadSize> in) noexcept;

// ---------------------------------------------------------------------------
// Implementations
// ---------------------------------------------------------------------------

inline void pack_engine_frame(
    const EngineTelemetry& telemetry,
    std::span<std::uint8_t, kCanPayloadSize> out) noexcept {
    const std::uint16_t rpm_raw =
        quantize_u16(telemetry.rpm / kRpmPerBit);
    const std::uint8_t coolant_raw =
        quantize_u8(telemetry.coolant_c - kCoolantOffsetC);
    const std::uint8_t throttle_raw =
        quantize_u8(telemetry.throttle_pct / kThrottlePercentPerBit);
    const std::uint16_t speed_raw =
        quantize_u16(telemetry.speed_kph / kSpeedKphPerBit);

    const auto rpm_bytes = to_be16(rpm_raw);
    const auto speed_bytes = to_be16(speed_raw);

    out[0] = rpm_bytes[0];
    out[1] = rpm_bytes[1];
    out[2] = coolant_raw;
    out[3] = throttle_raw;
    out[4] = speed_bytes[0];
    out[5] = speed_bytes[1];
    out[6] = 0x00U;
    out[7] = 0x00U;
}

[[nodiscard]] inline EngineTelemetry unpack_engine_frame(
    std::span<const std::uint8_t, kCanPayloadSize> in) noexcept {
    EngineTelemetry telemetry{};
    telemetry.rpm =
        static_cast<double>(from_be16({in[0], in[1]})) * kRpmPerBit;
    telemetry.coolant_c =
        static_cast<double>(in[2]) + kCoolantOffsetC;
    telemetry.throttle_pct =
        static_cast<double>(in[3]) * kThrottlePercentPerBit;
    telemetry.speed_kph =
        static_cast<double>(from_be16({in[4], in[5]})) * kSpeedKphPerBit;
    return telemetry;
}

inline void pack_diagnostic_frame(
    const DiagnosticTelemetry& telemetry,
    std::span<std::uint8_t, kCanPayloadSize> out) noexcept {
    std::uint8_t flags = 0U;
    if (telemetry.mil) {
        flags |= kFlagMil;
    }
    if (telemetry.overheat) {
        flags |= kFlagOverheat;
    }

    const auto battery_bytes = to_be16(telemetry.battery_mv);

    out[0] = flags;
    out[1] = battery_bytes[0];
    out[2] = battery_bytes[1];
    out[3] = 0x00U;
    out[4] = 0x00U;
    out[5] = 0x00U;
    out[6] = static_cast<std::uint8_t>(telemetry.alive_counter &
                                       kAliveCounterMask);
    out[7] = crc8_sae_j1850(std::span<const std::uint8_t>{out.data(), 7U});
}

[[nodiscard]] inline bool verify_diagnostic_crc(
    std::span<const std::uint8_t, kCanPayloadSize> in) noexcept {
    const std::uint8_t expected =
        crc8_sae_j1850(std::span<const std::uint8_t>{in.data(), 7U});
    return expected == in[7];
}

[[nodiscard]] inline std::optional<DiagnosticTelemetry>
unpack_diagnostic_frame(
    std::span<const std::uint8_t, kCanPayloadSize> in) noexcept {
    if (!verify_diagnostic_crc(in)) {
        return std::nullopt;
    }

    DiagnosticTelemetry telemetry{};
    telemetry.mil = (in[0] & kFlagMil) != 0U;
    telemetry.overheat = (in[0] & kFlagOverheat) != 0U;
    telemetry.battery_mv = from_be16({in[1], in[2]});
    telemetry.alive_counter =
        static_cast<std::uint8_t>(in[6] & kAliveCounterMask);
    return telemetry;
}

} // namespace vcansim

#endif // VCANSIM_CAN_PROTOCOL_HPP
