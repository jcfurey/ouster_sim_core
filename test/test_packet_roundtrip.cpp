// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/metadata.hpp"
#include "ouster_sim_core/packet_encoder.hpp"
#include "ouster_sim_core/revolution_assembler.hpp"

#include <ouster/lidar_scan.h>
#include <ouster/packet.h>
#include <ouster/types.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#ifndef OUSTER_SIM_CORE_TEST_DATA_DIR
#error "OUSTER_SIM_CORE_TEST_DATA_DIR must identify the core metadata fixtures"
#endif

namespace {

using ouster_sim_core::OusterFiringTable;
using ouster_sim_core::OusterMetadata;
using ouster_sim_core::OusterPacketEncoder;
using ouster_sim_core::OusterReturnSample;
using ouster_sim_core::OusterRevolutionAssembler;
using ouster_sim_core::ScheduledColumnBatch;

std::string metadataPath()
{
    return std::string(OUSTER_SIM_CORE_TEST_DATA_DIR) +
        "/os1_64_rev7.json";
}

std::vector<OusterReturnSample> deterministicReturns(
    const OusterFiringTable & table,
    std::uint64_t revolution)
{
    const ScheduledColumnBatch batch{
        revolution * table.columnsPerFrame(),
        table.columnsPerFrame()};
    std::vector<OusterReturnSample> returns;
    returns.reserve(table.sampleCount());
    for (std::size_t offset = 0; offset < table.sampleCount(); ++offset) {
        OusterReturnSample sample;
        sample.identity = table.identity(batch, offset);
        sample.is_hit = (offset % 11u) != 0u;
        if (sample.is_hit) {
            const auto millimeters = static_cast<std::uint32_t>(
                1'000u + (offset % 60'000u));
            sample.range_m = static_cast<double>(millimeters) * 0.001;
            sample.signal = static_cast<std::uint16_t>(
                (offset * 37u) & 0xffffu);
            sample.reflectivity = static_cast<std::uint8_t>(
                (offset * 13u) & 0xffu);
            sample.near_ir = static_cast<std::uint16_t>(
                (offset * 17u) & 0xffffu);
        }
        returns.push_back(sample);
    }
    return returns;
}

TEST(OusterMetadata, LoadsProductionCalibrationThroughTheSdk)
{
    const auto metadata = OusterMetadata::fromFile(metadataPath());
    EXPECT_EQ(metadata.columnsPerFrame(), 1024u);
    EXPECT_EQ(metadata.pixelsPerColumn(), 64u);
    EXPECT_EQ(metadata.columnsPerPacket(), 16u);
    EXPECT_GT(metadata.lidarPacketSize(), 0u);
    EXPECT_EQ(metadata.productLine(), "OS1-64");
    EXPECT_EQ(metadata.beamAltitudeDeg().size(), 64u);
    EXPECT_EQ(metadata.beamAzimuthDeg().size(), 64u);
    EXPECT_NEAR(metadata.beamOriginM(), 0.015806, 1.0e-12);
    EXPECT_FALSE(metadata.firmwareAdvertisementAdjusted());
    EXPECT_EQ(metadata.publishedJson(), metadata.sourceJson());

    EXPECT_THROW(
        OusterMetadata::fromFile(metadataPath(), 100),
        std::length_error);
    EXPECT_THROW(OusterMetadata::fromJson(""), std::invalid_argument);
}

TEST(OusterPacketEncoder, FullRevolutionRoundTripsThroughOusterSdk)
{
    constexpr std::uint64_t kRevolution = 7;
    constexpr std::int64_t kEpochNs = 1'000'000'000;

    const auto metadata = OusterMetadata::fromFile(metadataPath());
    const auto table = std::make_shared<const OusterFiringTable>(
        metadata.firingTableConfig(10.0));
    const auto returns = deterministicReturns(*table, kRevolution);
    OusterRevolutionAssembler assembler(table, kEpochNs, kRevolution);
    const auto frames = assembler.ingest(returns);
    ASSERT_EQ(frames.size(), 1u);
    const auto & frame = frames.front();

    OusterPacketEncoder encoder(metadata);
    const auto packets = encoder.encode(frame);
    ASSERT_EQ(
        packets.size(),
        metadata.columnsPerFrame() / metadata.columnsPerPacket());

    ouster::sdk::core::SensorInfo sensor_info(metadata.publishedJson());
    ouster::sdk::core::PacketFormat packet_format(sensor_info);
    ouster::sdk::core::ScanBatcher batcher(sensor_info);
    ouster::sdk::core::LidarScan decoded(sensor_info);
    const auto shared_format =
        std::make_shared<ouster::sdk::core::PacketFormat>(packet_format);

    bool scan_complete = false;
    for (std::size_t packet_index = 0;
         packet_index < packets.size(); ++packet_index) {
        const auto & encoded = packets[packet_index];
        ASSERT_EQ(encoded.bytes.size(), packet_format.lidar_packet_size);
        EXPECT_EQ(packet_format.frame_id(encoded.bytes.data()), kRevolution);
        EXPECT_EQ(packet_format.init_id(encoded.bytes.data()), 1u);
        EXPECT_EQ(packet_format.packet_type(encoded.bytes.data()), 0x1u);

        const auto stored_crc = packet_format.crc(
            encoded.bytes.data(), encoded.bytes.size());
        ASSERT_TRUE(stored_crc.has_value());
        EXPECT_EQ(
            stored_crc.value(),
            packet_format.calculate_crc(
                encoded.bytes.data(), encoded.bytes.size()));

        for (std::uint16_t local_column = 0;
             local_column < metadata.columnsPerPacket(); ++local_column) {
            const auto measurement = static_cast<std::uint16_t>(
                packet_index * metadata.columnsPerPacket() + local_column);
            const auto * column = packet_format.nth_col(
                local_column, encoded.bytes.data());
            EXPECT_EQ(packet_format.col_measurement_id(column), measurement);
            EXPECT_EQ(
                packet_format.col_timestamp(column),
                frame.column_timestamp_ns[measurement]);
            EXPECT_EQ(packet_format.col_status(column) & 0x01u, 0x01u);

            std::vector<std::uint32_t> range(metadata.pixelsPerColumn());
            packet_format.col_field<std::uint32_t>(
                column, ouster::sdk::core::ChanField::RANGE, range.data());
            for (std::uint16_t ring = 0;
                 ring < metadata.pixelsPerColumn(); ++ring) {
                EXPECT_EQ(
                    range[ring],
                    frame.range_mm[frame.sdkImageIndex(measurement, ring)]);
            }
        }

        ouster::sdk::core::LidarPacket lidar_packet(
            static_cast<int>(encoded.bytes.size()));
        lidar_packet.buf = encoded.bytes;
        lidar_packet.host_timestamp = encoded.first_column_timestamp_ns;
        lidar_packet.format = shared_format;
        scan_complete = batcher(lidar_packet, decoded) || scan_complete;
    }
    ASSERT_TRUE(scan_complete);
    EXPECT_EQ(decoded.frame_id, static_cast<std::int64_t>(kRevolution));

    const auto decoded_range = decoded.field<std::uint32_t>(
        ouster::sdk::core::ChanField::RANGE);
    const auto decoded_signal = decoded.field<std::uint16_t>(
        ouster::sdk::core::ChanField::SIGNAL);
    const auto decoded_reflectivity = decoded.field<std::uint8_t>(
        ouster::sdk::core::ChanField::REFLECTIVITY);
    const auto decoded_near_ir = decoded.field<std::uint16_t>(
        ouster::sdk::core::ChanField::NEAR_IR);

    for (std::uint32_t measurement = 0;
         measurement < metadata.columnsPerFrame(); ++measurement) {
        EXPECT_EQ(
            decoded.timestamp()[measurement],
            frame.column_timestamp_ns[measurement]);
        EXPECT_EQ(decoded.measurement_id()[measurement], measurement);
        for (std::uint16_t ring = 0;
             ring < metadata.pixelsPerColumn(); ++ring) {
            const auto index = frame.sdkImageIndex(measurement, ring);
            EXPECT_EQ(decoded_range(ring, measurement), frame.range_mm[index]);
            EXPECT_EQ(decoded_signal(ring, measurement), frame.signal[index]);
            EXPECT_EQ(
                decoded_reflectivity(ring, measurement),
                frame.reflectivity[index]);
            EXPECT_EQ(decoded_near_ir(ring, measurement), frame.near_ir[index]);
        }
    }
}

}  // namespace
