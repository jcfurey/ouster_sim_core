// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

// Wire conformance against packets captured from physical Ouster sensors.
//
// The Ouster SDK repository ships captures and their metadata from several
// products, firmware releases and UDP profiles. Each case below decodes one
// complete captured frame, feeds the decoded fields and column timestamps back
// through the simulator encoder, and requires the simulated packets to be
// indistinguishable from the captured ones wherever the simulator claims to
// model the sensor: packet topology and sizes, packet and column headers, the
// column window, and every decoded channel value. Captured bits the simulator
// intentionally does not model (range flags, firmware footer debug words) are
// excluded and reported.

#include "ouster_sim_core/firing_table.hpp"
#include "ouster_sim_core/imu_packet_pipeline.hpp"
#include "ouster_sim_core/metadata.hpp"
#include "ouster_sim_core/packet_encoder.hpp"
#include "support/pcap_reader.hpp"

#include <ouster/impl/packet_writer.h>
#include <ouster/lidar_scan.h>
#include <ouster/packet.h>
#include <ouster/types.h>
#include <ouster/xyzlut.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#ifndef OUSTER_SIM_CORE_SDK_TEST_DATA_DIR
#error "OUSTER_SIM_CORE_SDK_TEST_DATA_DIR must name the Ouster SDK tests directory"
#endif

namespace {

namespace sdk = ouster::sdk::core;
using ouster_sim_core::EncodedLidarPacket;
using ouster_sim_core::OusterMetadata;
using ouster_sim_core::OusterPacketEncoder;
using ouster_sim_core::OusterScanFrame;
using ouster_sim_core::test_support::readUdpDatagrams;
using ouster_sim_core::test_support::UdpDatagram;

std::string sdkData(const std::string & relative)
{
    return std::string(OUSTER_SIM_CORE_SDK_TEST_DATA_DIR) + "/" + relative;
}

struct CaptureCase {
    const char * name;
    const char * pcap;
    const char * metadata;
    const char * profile;
    std::uint16_t pixels_per_column;
    std::uint32_t columns_per_frame;
    // Independent of the SDK: sizes from the Ouster sensor packet formats.
    // LEGACY: 16 x (16 + H x 12 + 4); RNG19_RFL8_SIG16_NIR16:
    // 32 + 16 x (12 + H x 12) + 32; RNG15_RFL8_NIR8: 32 + 16 x (12 + H x 4)
    // + 32.
    std::size_t documented_packet_bytes;
    std::uint32_t packets_per_frame;
};

const CaptureCase kCaptures[] = {
    {"os2_32_legacy_fw2_0", "pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap",
     "pcaps/OS-2-32-U0_v2.0.0_1024x10.json", "LEGACY", 32, 1024, 6464, 64},
    {"os1_32_legacy_fw2_1", "pcaps/OS-1-32-G_v2.1.1_1024x10.pcap",
     "pcaps/OS-1-32-G_v2.1.1_1024x10.json", "LEGACY", 32, 1024, 6464, 64},
    {"os2_128_rng19_fw2_3", "pcaps/OS-2-128-U1_v2.3.0_1024x10.pcap",
     "pcaps/OS-2-128-U1_v2.3.0_1024x10.json", "RNG19_RFL8_SIG16_NIR16", 128,
     1024, 24832, 64},
    {"os0_128_low_data_fw2_3", "pcaps/OS-0-128-U1_v2.3.0_1024x10.pcap",
     "pcaps/OS-0-128-U1_v2.3.0_1024x10.json", "RNG15_RFL8_NIR8", 128, 1024,
     8448, 64},
    {"os0_128_low_data_column_window", "pcaps/windowed_frame1.pcap",
     "pcaps/windowed_frame1_0.json", "RNG15_RFL8_NIR8", 128, 512, 8448, 17},
};

const std::array<std::string, 4> kModelledFields{
    sdk::ChanField::RANGE, sdk::ChanField::SIGNAL, sdk::ChanField::REFLECTIVITY,
    sdk::ChanField::NEAR_IR};

void PrintTo(const CaptureCase & capture, std::ostream * output)
{
    *output << capture.name;
}

struct DecodedFrame {
    std::uint32_t frame_id = 0;
    std::vector<std::vector<std::uint8_t>> packets;
    OusterScanFrame frame;
    std::vector<std::uint32_t> status;
};

/// Return the first frame whose window-intersecting packets were all captured.
std::vector<std::vector<std::uint8_t>> firstCompleteFrame(
    const std::vector<UdpDatagram> & datagrams,
    const OusterMetadata & metadata,
    const sdk::PacketFormat & format)
{
    std::map<std::uint32_t, std::map<std::uint16_t, std::vector<std::uint8_t>>>
        frames;
    std::vector<std::uint32_t> order;
    for (const auto & datagram : datagrams) {
        if (datagram.payload.size() != metadata.lidarPacketSize()) {
            continue;
        }
        const auto frame_id = format.frame_id(datagram.payload.data());
        const auto first = format.col_measurement_id(
            format.nth_col(0, datagram.payload.data()));
        if (!frames.count(frame_id)) {
            order.push_back(frame_id);
        }
        frames[frame_id].emplace(first, datagram.payload);
    }
    for (const auto frame_id : order) {
        const auto & packets = frames[frame_id];
        if (packets.size() != metadata.lidarPacketsPerFrame()) {
            continue;
        }
        std::vector<std::vector<std::uint8_t>> result;
        for (const auto & [first, bytes] : packets) {
            static_cast<void>(first);
            result.push_back(bytes);
        }
        return result;
    }
    return {};
}

template <typename T>
void decodeField(const sdk::PacketFormat & format, const std::uint8_t * column,
                 const std::string & field, std::uint16_t height,
                 std::uint32_t measurement, std::uint32_t width,
                 std::vector<T> & image)
{
    if (format.field_type(field) == sdk::ChanFieldType::VOID) {
        return;
    }
    std::vector<std::uint64_t> values(height);
    format.col_field<std::uint64_t>(column, field, values.data());
    for (std::uint16_t ring = 0; ring < height; ++ring) {
        image[static_cast<std::size_t>(ring) * width + measurement] =
            static_cast<T>(values[ring]);
    }
}

DecodedFrame decodeCapture(const CaptureCase & capture,
                           const OusterMetadata & metadata)
{
    const sdk::SensorInfo info(metadata.publishedJson());
    const sdk::PacketFormat format(info);
    DecodedFrame decoded;
    decoded.packets = firstCompleteFrame(
        readUdpDatagrams(sdkData(capture.pcap)), metadata, format);
    if (decoded.packets.empty()) {
        return decoded;
    }
    decoded.frame_id = format.frame_id(decoded.packets.front().data());

    const auto width = metadata.columnsPerFrame();
    const auto height = metadata.pixelsPerColumn();
    auto & frame = decoded.frame;
    frame.revolution = decoded.frame_id;
    frame.columns_per_frame = width;
    frame.pixels_per_column = height;
    frame.column_timestamp_ns.assign(width, 0u);
    frame.range_mm.assign(frame.sampleCount(), 0u);
    frame.signal.assign(frame.sampleCount(), 0u);
    frame.reflectivity.assign(frame.sampleCount(), 0u);
    frame.near_ir.assign(frame.sampleCount(), 0u);
    decoded.status.assign(width, 0u);

    for (const auto & packet : decoded.packets) {
        for (int local = 0; local < format.columns_per_packet; ++local) {
            const auto * column = format.nth_col(local, packet.data());
            const auto measurement = format.col_measurement_id(column);
            frame.column_timestamp_ns[measurement] = format.col_timestamp(column);
            decoded.status[measurement] = format.col_status(column);
            decodeField(format, column, sdk::ChanField::RANGE, height,
                        measurement, width, frame.range_mm);
            decodeField(format, column, sdk::ChanField::SIGNAL, height,
                        measurement, width, frame.signal);
            decodeField(format, column, sdk::ChanField::REFLECTIVITY, height,
                        measurement, width, frame.reflectivity);
            decodeField(format, column, sdk::ChanField::NEAR_IR, height,
                        measurement, width, frame.near_ir);
        }
    }

    // Columns of packets a windowed sensor did not send still need a strictly
    // increasing simulated time base; extend the captured cadence.
    std::int64_t period = 0;
    for (std::uint32_t m = 1; m < width && period == 0; ++m) {
        if (frame.column_timestamp_ns[m] != 0 &&
            frame.column_timestamp_ns[m - 1] != 0) {
            period = static_cast<std::int64_t>(
                frame.column_timestamp_ns[m] - frame.column_timestamp_ns[m - 1]);
        }
    }
    for (std::uint32_t m = 1; m < width; ++m) {
        if (frame.column_timestamp_ns[m] == 0) {
            frame.column_timestamp_ns[m] =
                frame.column_timestamp_ns[m - 1] + static_cast<std::uint64_t>(period);
        }
    }
    if (frame.column_timestamp_ns[0] == 0) {
        frame.column_timestamp_ns[0] =
            frame.column_timestamp_ns[1] - static_cast<std::uint64_t>(period);
    }
    frame.frame_start_timestamp_ns = static_cast<std::int64_t>(
        frame.column_timestamp_ns[0]) - period;
    return decoded;
}

class RealCaptureConformance : public ::testing::TestWithParam<CaptureCase> {};

TEST_P(RealCaptureConformance, SimulatedPacketsMatchCapturedSensorPackets)
{
    const auto & capture = GetParam();
    const auto metadata = OusterMetadata::fromFile(sdkData(capture.metadata));
    ASSERT_EQ(metadata.activeLidarUdpProfile(), capture.profile);
    ASSERT_EQ(metadata.pixelsPerColumn(), capture.pixels_per_column);
    ASSERT_EQ(metadata.columnsPerFrame(), capture.columns_per_frame);
    ASSERT_EQ(metadata.lidarPacketSize(), capture.documented_packet_bytes);
    ASSERT_EQ(metadata.lidarPacketsPerFrame(), capture.packets_per_frame);

    const auto decoded = decodeCapture(capture, metadata);
    ASSERT_EQ(decoded.packets.size(), capture.packets_per_frame)
        << "capture holds no complete frame";

    const OusterPacketEncoder encoder(metadata);
    const auto simulated = encoder.encode(decoded.frame);
    ASSERT_EQ(simulated.size(), decoded.packets.size());

    const sdk::SensorInfo info(metadata.publishedJson());
    const sdk::PacketFormat format(info);
    const bool legacy = format.udp_profile_lidar == sdk::UDPProfileLidar::LEGACY;
    std::size_t captured_flag_pixels = 0;

    for (std::size_t index = 0; index < simulated.size(); ++index) {
        SCOPED_TRACE("packet " + std::to_string(index));
        const auto & real = decoded.packets[index];
        const auto & sim = simulated[index].bytes;
        ASSERT_EQ(sim.size(), real.size());

        // Packet header: type, frame ID, initialization ID, serial number and
        // the zero alert/countdown words of a healthy sensor.
        EXPECT_TRUE(std::equal(real.begin(),
                               real.begin() + format.packet_header_size,
                               sim.begin()));
        EXPECT_EQ(format.frame_id(sim.data()), format.frame_id(real.data()));
        EXPECT_EQ(simulated[index].first_measurement_id,
                  format.col_measurement_id(format.nth_col(0, real.data())));

        for (int local = 0; local < format.columns_per_packet; ++local) {
            SCOPED_TRACE("column " + std::to_string(local));
            const auto * real_column = format.nth_col(local, real.data());
            const auto * sim_column = format.nth_col(local, sim.data());
            // Column header bytes: timestamp, measurement ID, status and, for
            // legacy packets, the repeated frame ID and angle encoder count.
            EXPECT_TRUE(std::equal(real_column,
                                   real_column + format.col_header_size,
                                   sim_column));
            EXPECT_TRUE(std::equal(
                real_column + format.col_size - format.col_footer_size,
                real_column + format.col_size,
                sim_column + format.col_size - format.col_footer_size));

            const bool valid = (format.col_status(real_column) & 0x01u) != 0;
            for (const auto & field : kModelledFields) {
                if (format.field_type(field) == sdk::ChanFieldType::VOID) {
                    continue;
                }
                std::vector<std::uint64_t> expected(format.pixels_per_column);
                std::vector<std::uint64_t> actual(format.pixels_per_column);
                format.col_field<std::uint64_t>(real_column, field,
                                                expected.data());
                format.col_field<std::uint64_t>(sim_column, field,
                                                actual.data());
                if (!valid) {
                    // A sensor zeroes data for out-of-window columns.
                    EXPECT_TRUE(std::all_of(actual.begin(), actual.end(),
                                            [](auto v) { return v == 0; }));
                }
                EXPECT_EQ(actual, expected) << field;
            }
            if (format.field_type(sdk::ChanField::FLAGS) !=
                sdk::ChanFieldType::VOID) {
                std::vector<std::uint64_t> flags(format.pixels_per_column);
                format.col_field<std::uint64_t>(real_column,
                                                sdk::ChanField::FLAGS,
                                                flags.data());
                captured_flag_pixels += static_cast<std::size_t>(std::count_if(
                    flags.begin(), flags.end(), [](auto v) { return v != 0; }));
            }
        }

        if (!legacy) {
            const auto crc = format.crc(sim.data(), sim.size());
            ASSERT_TRUE(crc.has_value());
            EXPECT_EQ(*crc, format.calculate_crc(sim.data(), sim.size()));
        }
    }
    RecordProperty("captured_pixels_with_unmodelled_flags",
                   static_cast<int>(captured_flag_pixels));
}

TEST_P(RealCaptureConformance, SdkScanBatcherDecodesIdenticalScans)
{
    const auto & capture = GetParam();
    const auto metadata = OusterMetadata::fromFile(sdkData(capture.metadata));
    const auto decoded = decodeCapture(capture, metadata);
    ASSERT_FALSE(decoded.packets.empty());
    const auto simulated = OusterPacketEncoder(metadata).encode(decoded.frame);

    const auto info = std::make_shared<sdk::SensorInfo>(metadata.publishedJson());
    const auto format = std::make_shared<sdk::PacketFormat>(*info);
    const auto batch = [&](const auto & packets, auto bytes_of) {
        sdk::ScanBatcher batcher(info);
        sdk::LidarScan scan(*info);
        bool complete = false;
        for (const auto & packet : packets) {
            sdk::LidarPacket lidar(static_cast<int>(bytes_of(packet).size()));
            lidar.buf = bytes_of(packet);
            lidar.format = format;
            complete = batcher(lidar, scan) || complete;
        }
        // A windowed frame finishes when the next frame begins.
        if (!complete) {
            sdk::LidarPacket next(static_cast<int>(bytes_of(packets.front()).size()));
            next.buf = bytes_of(packets.front());
            next.format = format;
            auto * header = next.buf.data();
            sdk::impl::PacketWriter writer(*format);
            writer.set_frame_id(header, (format->frame_id(header) + 1u) %
                                            (format->max_frame_id + 1u));
            complete = batcher(next, scan);
        }
        EXPECT_TRUE(complete);
        return scan;
    };
    const auto real_scan = batch(
        decoded.packets, [](const auto & bytes) { return bytes; });
    const auto sim_scan = batch(
        simulated, [](const EncodedLidarPacket & p) { return p.bytes; });

    EXPECT_EQ(sim_scan.frame_id, real_scan.frame_id);
    EXPECT_TRUE((sim_scan.timestamp() == real_scan.timestamp()).all());
    EXPECT_TRUE((sim_scan.measurement_id() == real_scan.measurement_id()).all());
    const auto valid = [](std::uint32_t status) { return status & 1u; };
    EXPECT_TRUE((sim_scan.status().unaryExpr(valid) ==
                 real_scan.status().unaryExpr(valid)).all());
    EXPECT_EQ(sim_scan.complete(), real_scan.complete());
    for (const auto & [name, field] : real_scan.fields()) {
        if (name == sdk::ChanField::FLAGS) {
            continue;
        }
        ASSERT_TRUE(sim_scan.has_field(name)) << name;
        EXPECT_TRUE(sim_scan.field(name) == field) << name;
    }
}

INSTANTIATE_TEST_SUITE_P(
    OusterSdkCaptures, RealCaptureConformance, ::testing::ValuesIn(kCaptures),
    [](const auto & info) { return std::string(info.param.name); });

TEST(RealCaptureConformance, FiringGeometryMatchesSdkXyzLutForRealCalibrations)
{
    // The simulator casts each ray from the translated beam origin and reports
    // the lidar-origin-equivalent range. The decoder must then place the hit
    // exactly where the simulator found it.
    for (const auto & capture : kCaptures) {
        SCOPED_TRACE(capture.name);
        const auto metadata = OusterMetadata::fromFile(sdkData(capture.metadata));
        const sdk::SensorInfo info(metadata.publishedJson());
        const auto lut = sdk::make_xyz_lut(
            info.format.columns_per_frame, info.format.pixels_per_column, 0.001,
            info.beam_to_lidar_transform, sdk::mat4d::Identity(),
            info.beam_azimuth_angles, info.beam_altitude_angles);
        const ouster_sim_core::OusterFiringTable table(
            metadata.firingTableConfig(10.0));

        double worst_m = 0.0;
        for (std::uint32_t m = 0; m < table.columnsPerFrame(); m += 7) {
            for (std::uint16_t ring = 0; ring < table.channelCount(); ++ring) {
                const auto & firing = table.at(m, ring);
                for (const double path_m : {0.5, 7.25, 63.0, 180.5}) {
                    const double hit[3] = {
                        firing.origin_m.x + path_m * firing.direction.x,
                        firing.origin_m.y + path_m * firing.direction.y,
                        firing.origin_m.z + path_m * firing.direction.z};
                    const auto range_mm = static_cast<std::uint32_t>(
                        std::llround((path_m + metadata.beamOriginM()) * 1000.0));
                    const auto row = static_cast<Eigen::Index>(ring) *
                        table.columnsPerFrame() + m;
                    double error2 = 0.0;
                    for (int axis = 0; axis < 3; ++axis) {
                        const double decoded = lut.direction(row, axis) *
                            static_cast<double>(range_mm) + lut.offset(row, axis);
                        error2 += (decoded - hit[axis]) * (decoded - hit[axis]);
                    }
                    worst_m = std::max(worst_m, std::sqrt(error2));
                }
            }
        }
        // Integer-millimetre range quantization bounds the residual.
        EXPECT_LT(worst_m, 0.0005 + 1e-9);
        RecordProperty(std::string(capture.name) + "_worst_xyz_error_um",
                       static_cast<int>(std::lround(worst_m * 1e6)));
    }
}

TEST(RealCaptureConformance, LegacyImuUnitsMatchCapturedSensor)
{
    // A stationary sensor's legacy IMU packets carry gravity in g and rates in
    // deg/s. The simulator encodes SI state in the same convention, so the
    // SDK decodes both to the same SI magnitudes.
    const CaptureCase & capture = kCaptures[2];
    const auto metadata = OusterMetadata::fromFile(sdkData(capture.metadata));
    ASSERT_TRUE(metadata.legacyImuProfile());
    const auto format = std::make_shared<sdk::PacketFormat>(
        sdk::SensorInfo(metadata.publishedJson()));

    std::size_t packets = 0;
    for (const auto & datagram : readUdpDatagrams(sdkData(capture.pcap))) {
        if (datagram.payload.size() != metadata.imuPacketSize()) {
            continue;
        }
        sdk::ImuPacket real(static_cast<int>(datagram.payload.size()));
        real.buf = datagram.payload;
        real.format = format;
        const Eigen::ArrayX3f accel = real.accel();
        const Eigen::ArrayX3f gyro = real.gyro();
        const double gravity = std::sqrt(accel.row(0).square().sum());
        EXPECT_NEAR(gravity, 9.80665, 0.5);
        EXPECT_LT(std::sqrt(gyro.row(0).square().sum()), 0.1);

        ouster_sim_core::OusterImuPacketPipeline pipeline(
            metadata, std::chrono::milliseconds(100), 1, 1);
        ouster_sim_core::OusterImuState state;
        state.timestamp_ns = 1'000'000'000;
        for (int axis = 0; axis < 3; ++axis) {
            state.linear_acceleration_mps2[axis] = accel(0, axis);
            state.angular_velocity_rad_s[axis] = gyro(0, axis);
        }
        const auto encoded = pipeline.ingest(state);
        ASSERT_EQ(encoded.size(), 1u);
        ASSERT_EQ(encoded.front().bytes.size(), datagram.payload.size());
        sdk::ImuPacket sim(static_cast<int>(encoded.front().bytes.size()));
        sim.buf = encoded.front().bytes;
        sim.format = format;
        EXPECT_TRUE(sim.accel().isApprox(accel, 1e-6f));
        EXPECT_TRUE(sim.gyro().isApprox(gyro, 1e-5f) ||
                    (sim.gyro() - gyro).abs().maxCoeff() < 1e-6f);
        ++packets;
    }
    EXPECT_GE(packets, 10u);
}

TEST(RealCaptureConformance, DualReturnCaptureIsRejectedNotMisencoded)
{
    const auto metadata = OusterMetadata::fromFile(
        sdkData("pcaps/OS-0-32-U1_v2.2.0_1024x10.json"));
    EXPECT_EQ(metadata.activeReturnCount(), 2u);
    EXPECT_THROW(OusterPacketEncoder{metadata}, std::invalid_argument);
}

}  // namespace
