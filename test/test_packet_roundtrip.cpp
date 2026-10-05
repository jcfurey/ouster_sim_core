// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/metadata.hpp"
#include "support/conformance_v1.hpp"
#include "ouster_sim_core/packet_encoder.hpp"
#include "ouster_sim_core/revolution_assembler.hpp"

#include <ouster/lidar_scan.h>
#include <ouster/packet.h>
#include <ouster/types.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
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
using ouster_sim_core::OusterScanFrame;
using ouster_sim_core::ScheduledColumnBatch;

std::string metadataPath()
{
    return std::string(OUSTER_SIM_CORE_TEST_DATA_DIR) +
        "/os1_64_rev7.json";
}

std::string metadataJsonWithProfile(const std::string & profile)
{
    std::ifstream stream(metadataPath(), std::ios::binary);
    std::string json{std::istreambuf_iterator<char>(stream),
                     std::istreambuf_iterator<char>()};
    const std::string source_profile = "RNG19_RFL8_SIG16_NIR16";
    std::size_t position = 0;
    while ((position = json.find(source_profile, position)) !=
           std::string::npos) {
        json.replace(position, source_profile.size(), profile);
        position += profile.size();
    }
    return json;
}

OusterScanFrame blankFrame(const OusterMetadata & metadata)
{
    OusterScanFrame frame;
    frame.revolution = 1;
    frame.frame_start_timestamp_ns = 1'000'000'000;
    frame.columns_per_frame = metadata.columnsPerFrame();
    frame.pixels_per_column = metadata.pixelsPerColumn();
    frame.column_timestamp_ns.resize(frame.columns_per_frame);
    for (std::uint32_t measurement = 0;
         measurement < frame.columns_per_frame; ++measurement) {
        frame.column_timestamp_ns[measurement] =
            static_cast<std::uint64_t>(frame.frame_start_timestamp_ns) +
            measurement + 1u;
    }
    frame.range_mm.assign(frame.sampleCount(), 0u);
    frame.signal.assign(frame.sampleCount(), 0u);
    frame.reflectivity.assign(frame.sampleCount(), 0u);
    frame.near_ir.assign(frame.sampleCount(), 0u);
    return frame;
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
            sample.range_mm = millimeters;
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

TEST(OusterPacketEncoder, LowDataRangeBoundaryRoundTripsWithoutMasking)
{
    const auto metadata = OusterMetadata::fromJson(
        metadataJsonWithProfile("RNG15_RFL8_NIR8"));
    ASSERT_EQ(metadata.maximumEncodableRangeMm(), 262136u);
    OusterPacketEncoder encoder(metadata);
    auto frame = blankFrame(metadata);
    frame.range_mm[frame.sdkImageIndex(0, 0)] = 262136u;

    const auto packets = encoder.encode(frame);
    ASSERT_FALSE(packets.empty());

    ouster::sdk::core::SensorInfo sensor_info(metadata.publishedJson());
    ouster::sdk::core::PacketFormat packet_format(sensor_info);
    const auto * first_column = packet_format.nth_col(
        0, packets.front().bytes.data());
    std::vector<std::uint32_t> decoded(metadata.pixelsPerColumn());
    packet_format.col_field<std::uint32_t>(
        first_column, ouster::sdk::core::ChanField::RANGE, decoded.data());
    EXPECT_EQ(decoded.front(), 262136u);

    frame.range_mm[frame.sdkImageIndex(0, 0)] = 262144u;
    EXPECT_THROW(encoder.encode(frame), std::invalid_argument);

    frame.range_mm[frame.sdkImageIndex(0, 0)] = 262135u;
    EXPECT_THROW(encoder.encode(frame), std::invalid_argument);
}

TEST(OusterPacketEncoder, NarrowChannelFieldsSaturateInsteadOfWrapping)
{
    // RNG15_RFL8_NIR8 stores NEAR_IR >> 4 in eight bits. The SDK writer masks,
    // so 5000 would alias to 896 (dimmer than 4080) if not saturated.
    const auto metadata = OusterMetadata::fromJson(
        metadataJsonWithProfile("RNG15_RFL8_NIR8"));
    OusterPacketEncoder encoder(metadata);
    auto frame = blankFrame(metadata);
    frame.near_ir[frame.sdkImageIndex(0, 0)] = 5000u;
    frame.near_ir[frame.sdkImageIndex(0, 1)] = 4080u;
    frame.near_ir[frame.sdkImageIndex(0, 2)] = 65535u;
    frame.near_ir[frame.sdkImageIndex(0, 3)] = 1000u;

    const auto packets = encoder.encode(frame);
    ouster::sdk::core::PacketFormat format(
        ouster::sdk::core::SensorInfo(metadata.publishedJson()));
    std::vector<std::uint16_t> near_ir(metadata.pixelsPerColumn());
    format.col_field<std::uint16_t>(
        format.nth_col(0, packets.front().bytes.data()),
        ouster::sdk::core::ChanField::NEAR_IR, near_ir.data());
    EXPECT_EQ(near_ir[0], 4080u);
    EXPECT_EQ(near_ir[1], 4080u);
    EXPECT_EQ(near_ir[2], 4080u);
    // In-range values keep the profile's documented 16-count quantization.
    EXPECT_EQ(near_ir[3], 992u);
    // Saturation uses a private copy; the caller's frame is untouched.
    EXPECT_EQ(frame.near_ir[frame.sdkImageIndex(0, 0)], 5000u);
}

TEST(OusterPacketEncoder, ColumnWindowMatchesSensorPacketTopology)
{
    // A window wrapping through measurement 0 selects columns 1000..1023 and
    // 0..23, i.e. packets 62, 63, 0 and 1. The sensor sends only those, in
    // measurement order, with out-of-window columns invalid and zeroed.
    auto json = metadataJsonWithProfile("RNG19_RFL8_SIG16_NIR16");
    const std::string full = "\"column_window\": [\n      0,\n      1023\n    ]";
    const auto position = json.find(full);
    ASSERT_NE(position, std::string::npos);
    json.replace(position, full.size(), "\"column_window\": [1000, 23]");
    const auto metadata = OusterMetadata::fromJson(json);
    ASSERT_FALSE(metadata.fullColumnWindow());
    ASSERT_EQ(metadata.columnWindowFirst(), 1000u);
    ASSERT_EQ(metadata.columnWindowLast(), 23u);
    EXPECT_TRUE(metadata.isColumnInWindow(1023));
    EXPECT_TRUE(metadata.isColumnInWindow(0));
    EXPECT_FALSE(metadata.isColumnInWindow(24));
    EXPECT_FALSE(metadata.isColumnInWindow(999));
    ASSERT_EQ(metadata.lidarPacketsPerFrame(), 4u);

    ouster::sdk::core::SensorInfo info(metadata.publishedJson());
    ASSERT_EQ(static_cast<int>(metadata.lidarPacketsPerFrame()),
              info.format.lidar_packets_per_frame());

    OusterPacketEncoder encoder(metadata);
    auto frame = blankFrame(metadata);
    std::fill(frame.range_mm.begin(), frame.range_mm.end(), 4242u);
    std::fill(frame.signal.begin(), frame.signal.end(), 7u);
    const auto packets = encoder.encode(frame);
    ASSERT_EQ(packets.size(), 4u);
    const std::vector<std::uint16_t> first_ids{0, 16, 992, 1008};
    ouster::sdk::core::PacketFormat format(info);
    for (std::size_t index = 0; index < packets.size(); ++index) {
        EXPECT_EQ(packets[index].first_measurement_id, first_ids[index]);
        for (int local = 0; local < format.columns_per_packet; ++local) {
            const auto * column =
                format.nth_col(local, packets[index].bytes.data());
            const auto measurement = format.col_measurement_id(column);
            const bool in_window = metadata.isColumnInWindow(measurement);
            EXPECT_EQ(format.col_status(column) & 1u, in_window ? 1u : 0u);
            EXPECT_EQ(format.col_timestamp(column),
                      frame.column_timestamp_ns[measurement]);
            std::vector<std::uint32_t> range(metadata.pixelsPerColumn());
            format.col_field<std::uint32_t>(
                column, ouster::sdk::core::ChanField::RANGE, range.data());
            EXPECT_EQ(range.front(), in_window ? 4242u : 0u) << measurement;
        }
    }

    // The SDK validator discards an out-of-bounds window and keeps the full
    // default, which the core then reports consistently.
    auto out_of_bounds = json;
    out_of_bounds.replace(out_of_bounds.find("[1000, 23]"), 10, "[0, 1024]");
    const auto fallback = OusterMetadata::fromJson(out_of_bounds);
    EXPECT_TRUE(fallback.fullColumnWindow());
    EXPECT_EQ(fallback.lidarPacketsPerFrame(), 64u);
}

TEST(OusterPacketEncoder, LegacyColumnsCarrySensorHeaderFields)
{
    for (const char * mode : {"512x10", "1024x10", "2048x10"}) {
        SCOPED_TRACE(mode);
        auto json = metadataJsonWithProfile("LEGACY");
        const std::uint32_t width = static_cast<std::uint32_t>(
            std::stoul(std::string(mode).substr(0, std::string(mode).find('x'))));
        const auto replace_all = [&](const std::string & before,
                                     const std::string & after) {
            for (auto at = json.find(before); at != std::string::npos;
                 at = json.find(before, at + after.size())) {
                json.replace(at, before.size(), after);
            }
        };
        replace_all("\"1024x10\"", "\"" + std::string(mode) + "\"");
        replace_all("\"columns_per_frame\": 1024",
                    "\"columns_per_frame\": " + std::to_string(width));
        replace_all("1023\n    ]", std::to_string(width - 1) + "\n    ]");
        const auto metadata = OusterMetadata::fromJson(json);
        ASSERT_EQ(metadata.columnsPerFrame(), width);

        auto frame = blankFrame(metadata);
        frame.revolution = 70000;  // wraps the 16-bit legacy frame ID
        const auto packets = OusterPacketEncoder(metadata).encode(frame);
        ASSERT_EQ(packets.size(), width / 16u);
        ouster::sdk::core::PacketFormat format(
            ouster::sdk::core::SensorInfo(metadata.publishedJson()));
        for (const auto & packet : packets) {
            for (int local = 0; local < format.columns_per_packet; ++local) {
                const auto * column = format.nth_col(local, packet.bytes.data());
                std::uint16_t frame_id = 0;
                std::uint32_t encoder_count = 0;
                std::uint32_t status = 0;
                std::memcpy(&frame_id, column + 10, sizeof(frame_id));
                std::memcpy(&encoder_count, column + 12, sizeof(encoder_count));
                std::memcpy(&status, column + format.col_size - 4, sizeof(status));
                const auto measurement = format.col_measurement_id(column);
                EXPECT_EQ(frame_id, 70000u % 65536u);
                EXPECT_EQ(encoder_count, measurement * (90112u / width));
                EXPECT_EQ(status, 0xffffffffu);
            }
        }
    }
}

TEST(OusterPacketEncoder, RejectsDualReturnProfileBeforeEncoding)
{
    const auto metadata = OusterMetadata::fromJson(
        metadataJsonWithProfile("RNG19_RFL8_SIG16_NIR16_DUAL"));
    ASSERT_EQ(metadata.activeReturnCount(), 2u);
    EXPECT_THROW(OusterPacketEncoder{metadata}, std::invalid_argument);
}

TEST(OusterPacketEncoder, SharedFixturesAndBorrowedBuffersRetainWireContract)
{
    namespace fixture = ouster_sim_core::conformance_v1;
    for (const auto * profile : fixture::primaryProfiles) {
        SCOPED_TRACE(profile);
        const auto metadata = OusterMetadata::fromJson(
            fixture::metadataJson(metadataPath(), profile));
        const OusterPacketEncoder encoder(metadata);
        auto frame = fixture::frame(metadata, 65536);
        const auto view = ouster_sim_core::OusterScanFrameView::fromFrame(frame);
        const auto owned = encoder.encode(frame);
        std::vector<ouster_sim_core::EncodedLidarPacket> borrowed;
        encoder.encode(view, borrowed);
        ASSERT_EQ(owned.size(), 64u);
        ASSERT_EQ(borrowed.size(), owned.size());
        ouster::sdk::core::PacketFormat format(
            ouster::sdk::core::SensorInfo(metadata.publishedJson()));
        for (std::size_t i = 0; i < owned.size(); ++i) {
            EXPECT_EQ(borrowed[i].bytes, owned[i].bytes);
            EXPECT_EQ(format.frame_id(borrowed[i].bytes.data()), 0u);
            if (std::string(profile) != "LEGACY") {
                const auto & bytes = borrowed[i].bytes;
                EXPECT_EQ(format.packet_type(bytes.data()), 1u);
                EXPECT_EQ(format.init_id(bytes.data()), 1234u);
                EXPECT_EQ(format.prod_sn(bytes.data()), 123456789012ULL);
                EXPECT_EQ(format.crc(bytes.data(), bytes.size()),
                          format.calculate_crc(bytes.data(), bytes.size()));
            }
        }
        // A bad borrowed span must not replace any part of the previous batch.
        auto invalid = view;
        invalid.range_mm = invalid.range_mm.first(1);
        EXPECT_THROW(encoder.encode(invalid, borrowed), std::invalid_argument);
        EXPECT_EQ(borrowed.front().bytes, owned.front().bytes);
        frame.column_timestamp_ns[1] = frame.column_timestamp_ns[0];
        EXPECT_THROW(encoder.encode(view, borrowed), std::invalid_argument);
        EXPECT_EQ(borrowed.back().bytes, owned.back().bytes);
    }
}

}  // namespace
