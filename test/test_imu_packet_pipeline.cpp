// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/imu_packet_pipeline.hpp"

#include <ouster/packet.h>
#include <ouster/types.h>

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#ifndef OUSTER_SIM_CORE_TEST_DATA_DIR
#error "OUSTER_SIM_CORE_TEST_DATA_DIR must identify core metadata fixtures"
#endif

namespace {

using namespace std::chrono_literals;
using ouster_sim_core::OusterImuPacketPipeline;
using ouster_sim_core::OusterImuState;
using ouster_sim_core::OusterMetadata;

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

std::string metadataPath()
{
    return std::string(OUSTER_SIM_CORE_TEST_DATA_DIR) +
        "/os1_64_rev7.json";
}

std::string readFixture()
{
    std::ifstream stream(metadataPath(), std::ios::binary);
    return {std::istreambuf_iterator<char>(stream),
            std::istreambuf_iterator<char>()};
}

void replaceAll(
    std::string & value,
    const std::string & from,
    const std::string & to)
{
    std::size_t position = 0;
    while ((position = value.find(from, position)) != std::string::npos) {
        value.replace(position, from.size(), to);
        position += to.size();
    }
}

std::string modernMetadataJson()
{
    auto json = readFixture();
    replaceAll(
        json,
        "\"udp_profile_imu\": \"LEGACY\"",
        "\"udp_profile_imu\": \"ACCEL32_GYRO32_NMEA\"");
    const auto end = json.rfind('}');
    if (end == std::string::npos) {
        throw std::runtime_error("test metadata is not a JSON object");
    }
    json.insert(
        end,
        ",\n  \"imu_data_format\": {"
        "\"imu_measurements_per_packet\": 8, "
        "\"imu_packets_per_frame\": 8} \n");
    return json;
}

ouster::sdk::core::ImuPacket decode(
    const OusterMetadata & metadata,
    const ouster_sim_core::EncodedOusterImuPacket & encoded)
{
    const auto information = std::make_shared<ouster::sdk::core::SensorInfo>(
        metadata.publishedJson());
    auto packet = ouster::sdk::core::ImuPacket(
        static_cast<int>(encoded.bytes.size()));
    packet.buf = encoded.bytes;
    packet.host_timestamp = encoded.first_sample_timestamp_ns;
    packet.format = std::make_shared<ouster::sdk::core::PacketFormat>(
        *information);
    EXPECT_EQ(
        packet.validate(*information),
        ouster::sdk::core::PacketValidationFailure::NONE);
    return packet;
}

TEST(OusterImuPacketPipeline, LegacyPacketRoundTripsThroughSdkInSiUnits)
{
    const auto metadata = OusterMetadata::fromFile(metadataPath());
    OusterImuPacketPipeline pipeline(metadata, 100ms, 41, 7);
    const auto & contract = pipeline.contract();
    EXPECT_TRUE(contract.legacy);
    EXPECT_EQ(contract.packet_bytes, 48u);
    EXPECT_EQ(contract.measurements_per_packet, 1u);
    EXPECT_EQ(contract.packets_per_frame, 10u);
    EXPECT_EQ(contract.sample_period, 10ms);

    OusterImuState state;
    state.timestamp_ns = 1'000'000'000;
    state.linear_acceleration_mps2 = {kGravity, -2.0 * kGravity, 0.5};
    state.angular_velocity_rad_s = {kPi, -0.5 * kPi, 0.25};
    const auto encoded = pipeline.ingest(state);
    ASSERT_EQ(encoded.size(), 1u);
    EXPECT_EQ(encoded[0].sensor_stream_id, 41u);
    EXPECT_EQ(encoded[0].epoch, 7u);
    EXPECT_EQ(encoded[0].packet_sequence, 0u);
    EXPECT_EQ(encoded[0].first_sample_timestamp_ns, 1'000'000'000u);

    const auto packet = decode(metadata, encoded[0]);
    EXPECT_EQ(packet.sys_ts(), 1'000'000'000u);
    EXPECT_EQ(packet.accel_ts(), 1'000'000'000u);
    EXPECT_EQ(packet.gyro_ts(), 1'000'000'000u);
    const auto acceleration = packet.accel();
    const auto angular_velocity = packet.gyro();
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR(
            acceleration(0, axis), state.linear_acceleration_mps2[axis],
            1.0e-5);
        EXPECT_NEAR(
            angular_velocity(0, axis), state.angular_velocity_rad_s[axis],
            1.0e-5);
    }
}

TEST(OusterImuPacketPipeline, ModernPacketCarriesNativeHeaderSamplesAndCrc)
{
    const auto metadata = OusterMetadata::fromJson(modernMetadataJson());
    OusterImuPacketPipeline pipeline(metadata, 100ms, 52, 9);
    const auto & contract = pipeline.contract();
    ASSERT_FALSE(contract.legacy);
    ASSERT_EQ(contract.packet_bytes, 452u);
    ASSERT_EQ(contract.measurements_per_packet, 8u);
    ASSERT_EQ(contract.packets_per_frame, 8u);
    ASSERT_EQ(contract.sample_period, 1'562'500ns);

    constexpr std::int64_t start = 2'000'000'000;
    std::vector<ouster_sim_core::EncodedOusterImuPacket> encoded;
    for (std::uint64_t sample = 0; sample < 8; ++sample) {
        OusterImuState state;
        state.timestamp_ns = start +
            static_cast<std::int64_t>(sample) * contract.sample_period.count();
        state.linear_acceleration_mps2 = {
            1.0 + sample, 2.0 + sample, 3.0 + sample};
        state.angular_velocity_rad_s = {
            -1.0 - sample, -2.0 - sample, -3.0 - sample};
        const auto ready = pipeline.ingest(state);
        encoded.insert(encoded.end(), ready.begin(), ready.end());
    }

    ASSERT_EQ(encoded.size(), 1u);
    EXPECT_EQ(encoded[0].sensor_stream_id, 52u);
    EXPECT_EQ(encoded[0].epoch, 9u);
    EXPECT_EQ(encoded[0].packet_sequence, 0u);
    EXPECT_EQ(encoded[0].first_sample_timestamp_ns, start);
    EXPECT_EQ(encoded[0].sample_count, 8u);

    const auto packet = decode(metadata, encoded[0]);
    EXPECT_EQ(packet.packet_type(), 0x2u);
    EXPECT_EQ(packet.frame_id(), metadata.packetFrameId(0));
    EXPECT_EQ(packet.init_id(), metadata.initializationId());
    EXPECT_EQ(packet.prod_sn(), metadata.sensorSerial());
    EXPECT_EQ(packet.nmea_ts(), static_cast<std::uint64_t>(start));
    ASSERT_TRUE(packet.crc().has_value());
    EXPECT_EQ(packet.crc().value(), packet.calculate_crc());

    const auto timestamps = packet.timestamp();
    const auto measurement_ids = packet.measurement_id();
    const auto statuses = packet.status();
    const auto acceleration = packet.accel();
    const auto angular_velocity = packet.gyro();
    for (std::uint64_t sample = 0; sample < 8; ++sample) {
        EXPECT_EQ(
            timestamps(sample),
            static_cast<std::uint64_t>(start) +
                sample * contract.sample_period.count());
        EXPECT_EQ(measurement_ids(sample), sample * 16u);
        EXPECT_EQ(statuses(sample), 1u);
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_NEAR(
                acceleration(sample, axis),
                static_cast<double>(axis + 1) + sample,
                1.0e-6);
            EXPECT_NEAR(
                angular_velocity(sample, axis),
                -static_cast<double>(axis + 1) - sample,
                1.0e-6);
        }
    }
}

TEST(OusterImuPacketPipeline, ResetRestartsPacketIdentityAndTimeDomain)
{
    const auto metadata = OusterMetadata::fromFile(metadataPath());
    OusterImuPacketPipeline pipeline(metadata, 100ms, 63, 3);
    ASSERT_EQ(pipeline.ingest(OusterImuState{100}).size(), 1u);
    EXPECT_THROW(pipeline.ingest(OusterImuState{99}), std::invalid_argument);
    EXPECT_THROW(pipeline.reset(3), std::invalid_argument);

    pipeline.reset(4);
    const auto encoded = pipeline.ingest(OusterImuState{50});
    ASSERT_EQ(encoded.size(), 1u);
    EXPECT_EQ(encoded[0].epoch, 4u);
    EXPECT_EQ(encoded[0].packet_sequence, 0u);
    EXPECT_EQ(encoded[0].first_sample_timestamp_ns, 50u);
}

}  // namespace
