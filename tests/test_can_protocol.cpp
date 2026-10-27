// SPDX-License-Identifier: MIT
//
// Unit tests for the VCanSim application protocol.  These tests are entirely
// self-contained and do not require a vcan interface or any elevated rights.

#include "can_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace {

using Payload = std::array<std::uint8_t, vcansim::kCanPayloadSize>;

[[nodiscard]] Payload pack_engine(const vcansim::EngineTelemetry& telemetry) {
    Payload payload{};
    vcansim::pack_engine_frame(
        telemetry, std::span<std::uint8_t, vcansim::kCanPayloadSize>{
                       payload.data(), payload.size()});
    return payload;
}

[[nodiscard]] Payload
pack_diagnostic(const vcansim::DiagnosticTelemetry& telemetry) {
    Payload payload{};
    vcansim::pack_diagnostic_frame(
        telemetry, std::span<std::uint8_t, vcansim::kCanPayloadSize>{
                       payload.data(), payload.size()});
    return payload;
}

[[nodiscard]] vcansim::EngineTelemetry
unpack_engine(const Payload& payload) {
    return vcansim::unpack_engine_frame(
        std::span<const std::uint8_t, vcansim::kCanPayloadSize>{
            payload.data(), payload.size()});
}

[[nodiscard]] std::optional<vcansim::DiagnosticTelemetry>
unpack_diagnostic(const Payload& payload) {
    return vcansim::unpack_diagnostic_frame(
        std::span<const std::uint8_t, vcansim::kCanPayloadSize>{
            payload.data(), payload.size()});
}

// ---------------------------------------------------------------------------
// Big-endian helpers
// ---------------------------------------------------------------------------

TEST(Endianness, SixteenBitRoundTrip) {
    EXPECT_EQ(vcansim::to_be16(0x1234U),
              (std::array<std::uint8_t, 2>{0x12U, 0x34U}));
    EXPECT_EQ(vcansim::from_be16({0x12U, 0x34U}), 0x1234U);
    EXPECT_EQ(vcansim::from_be16(vcansim::to_be16(0xABCDU)), 0xABCDU);
    EXPECT_EQ(vcansim::to_be16(0x0000U),
              (std::array<std::uint8_t, 2>{0x00U, 0x00U}));
    EXPECT_EQ(vcansim::to_be16(0xFFFFU),
              (std::array<std::uint8_t, 2>{0xFFU, 0xFFU}));
}

TEST(Endianness, EnginePayloadIsNetworkByteOrder) {
    // 1165 rpm => 4660 raw = 0x1234; 439.81 km/h => 43981 raw = 0xABCD.
    const auto payload =
        pack_engine(vcansim::EngineTelemetry{1165.0, 90.0, 50.0, 439.81});

    EXPECT_EQ(payload[0], 0x12U);
    EXPECT_EQ(payload[1], 0x34U);
    EXPECT_EQ(payload[4], 0xABU);
    EXPECT_EQ(payload[5], 0xCDU);
}

// ---------------------------------------------------------------------------
// Engine telemetry frame 0x100
// ---------------------------------------------------------------------------

TEST(EngineFrame, NominalLayout) {
    const vcansim::EngineTelemetry telemetry{3000.0, 90.0, 40.0, 100.5};
    const auto payload = pack_engine(telemetry);

    // 3000 / 0.25 = 12000 = 0x2EE0
    EXPECT_EQ(payload[0], 0x2EU);
    EXPECT_EQ(payload[1], 0xE0U);
    // 90 C with a -40 C offset = 130 = 0x82
    EXPECT_EQ(payload[2], 0x82U);
    // 40 % / 0.5 = 80 = 0x50
    EXPECT_EQ(payload[3], 0x50U);
    // 100.5 / 0.01 = 10050 = 0x2742
    EXPECT_EQ(payload[4], 0x27U);
    EXPECT_EQ(payload[5], 0x42U);
    EXPECT_EQ(payload[6], 0x00U);
    EXPECT_EQ(payload[7], 0x00U);
}

TEST(EngineFrame, NominalRoundTrip) {
    const vcansim::EngineTelemetry telemetry{1234.25, 87.0, 33.5, 88.88};
    const auto decoded = unpack_engine(pack_engine(telemetry));

    EXPECT_DOUBLE_EQ(decoded.rpm, telemetry.rpm);
    EXPECT_DOUBLE_EQ(decoded.coolant_c, telemetry.coolant_c);
    EXPECT_DOUBLE_EQ(decoded.throttle_pct, telemetry.throttle_pct);
    EXPECT_NEAR(decoded.speed_kph, telemetry.speed_kph, 1e-9);
}

TEST(EngineFrame, MinimumValues) {
    const vcansim::EngineTelemetry telemetry{0.0, -40.0, 0.0, 0.0};
    const auto payload = pack_engine(telemetry);

    for (std::size_t index = 0; index < vcansim::kCanPayloadSize; ++index) {
        EXPECT_EQ(payload[index], 0x00U) << "byte " << index;
    }

    const auto decoded = unpack_engine(payload);
    EXPECT_DOUBLE_EQ(decoded.rpm, 0.0);
    EXPECT_DOUBLE_EQ(decoded.coolant_c, -40.0);
    EXPECT_DOUBLE_EQ(decoded.throttle_pct, 0.0);
    EXPECT_DOUBLE_EQ(decoded.speed_kph, 0.0);
}

TEST(EngineFrame, MaximumValues) {
    const vcansim::EngineTelemetry telemetry{
        16383.75, 215.0, 127.5, 655.35};
    const auto payload = pack_engine(telemetry);

    for (std::size_t index = 0; index < 6U; ++index) {
        EXPECT_EQ(payload[index], 0xFFU) << "byte " << index;
    }
    EXPECT_EQ(payload[6], 0x00U);
    EXPECT_EQ(payload[7], 0x00U);

    const auto decoded = unpack_engine(payload);
    EXPECT_DOUBLE_EQ(decoded.rpm, 16383.75);
    EXPECT_DOUBLE_EQ(decoded.coolant_c, 215.0);
    EXPECT_DOUBLE_EQ(decoded.throttle_pct, 127.5);
    EXPECT_NEAR(decoded.speed_kph, 655.35, 1e-9);
}

TEST(EngineFrame, SaturatesOutOfRangeValues) {
    const auto high =
        pack_engine(vcansim::EngineTelemetry{99999.0, 300.0, 200.0, 9999.0});
    for (std::size_t index = 0; index < 6U; ++index) {
        EXPECT_EQ(high[index], 0xFFU) << "byte " << index;
    }

    const auto high_decoded = unpack_engine(high);
    EXPECT_DOUBLE_EQ(high_decoded.rpm, 16383.75);
    EXPECT_DOUBLE_EQ(high_decoded.coolant_c, 215.0);
    EXPECT_DOUBLE_EQ(high_decoded.throttle_pct, 127.5);
    EXPECT_NEAR(high_decoded.speed_kph, 655.35, 1e-9);

    const auto low =
        pack_engine(vcansim::EngineTelemetry{-1.0, -100.0, -5.0, -10.0});
    for (std::size_t index = 0; index < 6U; ++index) {
        EXPECT_EQ(low[index], 0x00U) << "byte " << index;
    }
}

TEST(EngineFrame, QuantisesToNearestBit) {
    // Values below half a bit quantise down, values at/above half a bit
    // quantise up.  RPM has a 0.25 rpm resolution.
    EXPECT_EQ(pack_engine(vcansim::EngineTelemetry{0.1, 0.0, 0.0, 0.0})[1],
              0x00U);
    EXPECT_EQ(pack_engine(vcansim::EngineTelemetry{0.2, 0.0, 0.0, 0.0})[1],
              0x01U);

    // Coolant: 0.4 C above the -40 C offset is below half a count.
    EXPECT_EQ(pack_engine(vcansim::EngineTelemetry{0.0, -39.6, 0.0, 0.0})[2],
              0x00U);
    EXPECT_EQ(pack_engine(vcansim::EngineTelemetry{0.0, -39.4, 0.0, 0.0})[2],
              0x01U);
}

// ---------------------------------------------------------------------------
// CRC-8
// ---------------------------------------------------------------------------

TEST(Crc8, StandardCheckVector) {
    constexpr std::array<std::uint8_t, 9> check{'1', '2', '3', '4', '5',
                                                '6', '7', '8', '9'};
    EXPECT_EQ(vcansim::crc8_sae_j1850(std::span<const std::uint8_t>{
                  check.data(), check.size()}),
              0x4BU);
}

TEST(Crc8, EmptyInputUsesInitAndXorOut) {
    EXPECT_EQ(vcansim::crc8_sae_j1850(
                  std::span<const std::uint8_t>{}),
              0x00U);
}

// ---------------------------------------------------------------------------
// Diagnostic frame 0x200
// ---------------------------------------------------------------------------

TEST(DiagnosticFrame, LayoutAndCrc) {
    const vcansim::DiagnosticTelemetry telemetry{true, true, 13800U, 5U};
    const auto payload = pack_diagnostic(telemetry);

    EXPECT_EQ(payload[0], 0x03U);       // MIL | overheat
    EXPECT_EQ(payload[1], 0x35U);       // 13800 = 0x35E8, big-endian
    EXPECT_EQ(payload[2], 0xE8U);
    EXPECT_EQ(payload[3], 0x00U);
    EXPECT_EQ(payload[4], 0x00U);
    EXPECT_EQ(payload[5], 0x00U);
    EXPECT_EQ(payload[6], 0x05U);       // alive counter, low nibble
    EXPECT_EQ(payload[7], 0xF1U);       // CRC-8/SAE-J1850 over bytes 0..6

    EXPECT_TRUE(vcansim::verify_diagnostic_crc(
        std::span<const std::uint8_t, vcansim::kCanPayloadSize>{
            payload.data(), payload.size()}));
}

TEST(DiagnosticFrame, RoundTrip) {
    const vcansim::DiagnosticTelemetry telemetry{false, true, 12450U, 9U};
    const auto decoded = unpack_diagnostic(pack_diagnostic(telemetry));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->mil, telemetry.mil);
    EXPECT_EQ(decoded->overheat, telemetry.overheat);
    EXPECT_EQ(decoded->battery_mv, telemetry.battery_mv);
    EXPECT_EQ(decoded->alive_counter, telemetry.alive_counter);
}

TEST(DiagnosticFrame, FlagsAreIndependent) {
    const auto mil_only =
        unpack_diagnostic(pack_diagnostic({true, false, 13000U, 0U}));
    ASSERT_TRUE(mil_only.has_value());
    EXPECT_TRUE(mil_only->mil);
    EXPECT_FALSE(mil_only->overheat);

    const auto overheat_only =
        unpack_diagnostic(pack_diagnostic({false, true, 13000U, 0U}));
    ASSERT_TRUE(overheat_only.has_value());
    EXPECT_FALSE(overheat_only->mil);
    EXPECT_TRUE(overheat_only->overheat);

    const auto neither =
        unpack_diagnostic(pack_diagnostic({false, false, 13000U, 0U}));
    ASSERT_TRUE(neither.has_value());
    EXPECT_FALSE(neither->mil);
    EXPECT_FALSE(neither->overheat);
}

TEST(DiagnosticFrame, AliveCounterWrapsAndIsMasked) {
    for (std::uint8_t counter = 0; counter < 16U; ++counter) {
        const auto payload = pack_diagnostic({false, false, 12000U, counter});
        EXPECT_EQ(payload[6], counter);
        const auto decoded = unpack_diagnostic(payload);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(decoded->alive_counter, counter);
    }

    // Only the low nibble is part of the wire format.
    const auto masked = pack_diagnostic({false, false, 12000U, 0x1FU});
    EXPECT_EQ(masked[6], 0x0FU);
    const auto masked_decoded = unpack_diagnostic(masked);
    ASSERT_TRUE(masked_decoded.has_value());
    EXPECT_EQ(masked_decoded->alive_counter, 0x0FU);
}

TEST(DiagnosticFrame, DetectsEveryCorruptedDataByte) {
    Payload payload = pack_diagnostic({false, false, 13800U, 7U});

    for (std::size_t index = 0; index < 7U; ++index) {
        Payload corrupted = payload;
        corrupted[index] ^= 0xFFU;
        EXPECT_FALSE(vcansim::verify_diagnostic_crc(
            std::span<const std::uint8_t, vcansim::kCanPayloadSize>{
                corrupted.data(), corrupted.size()}))
            << "corruption in byte " << index << " was not detected";
        EXPECT_FALSE(unpack_diagnostic(corrupted).has_value())
            << "corruption in byte " << index << " was not rejected";
    }
}

TEST(DiagnosticFrame, DetectsCorruptedCrcByte) {
    Payload payload = pack_diagnostic({true, false, 13800U, 3U});
    payload[7] ^= 0x01U;
    EXPECT_FALSE(unpack_diagnostic(payload).has_value());
}

TEST(DiagnosticFrame, IgnoresReservedBits) {
    Payload payload = pack_diagnostic({false, false, 12000U, 2U});
    payload[0] |= 0xFCU; // reserved flag bits
    payload[6] |= 0xF0U; // reserved counter nibble
    payload[7] = vcansim::crc8_sae_j1850(
        std::span<const std::uint8_t>{payload.data(), 7U});

    const auto decoded = unpack_diagnostic(payload);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_FALSE(decoded->mil);
    EXPECT_FALSE(decoded->overheat);
    EXPECT_EQ(decoded->alive_counter, 2U);
}

TEST(DiagnosticFrame, BatteryBoundaries) {
    const auto zero = unpack_diagnostic(pack_diagnostic({false, false, 0U, 0U}));
    ASSERT_TRUE(zero.has_value());
    EXPECT_EQ(zero->battery_mv, 0U);

    const auto maximum = unpack_diagnostic(
        pack_diagnostic({false, false, 65535U, 0U}));
    ASSERT_TRUE(maximum.has_value());
    EXPECT_EQ(maximum->battery_mv, 65535U);
}

} // namespace
