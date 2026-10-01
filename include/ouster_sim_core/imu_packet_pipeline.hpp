// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/metadata.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace ouster_sim_core {

/// One simulator-provided IMU state in the physical IMU coordinate frame.
struct OusterImuState {
    std::int64_t timestamp_ns = 0;
    std::array<double, 3> linear_acceleration_mps2{};
    std::array<double, 3> angular_velocity_rad_s{};
};

/// One complete native Ouster IMU UDP payload.
struct EncodedOusterImuPacket {
    std::uint64_t sensor_stream_id = 0;
    std::uint64_t epoch = 0;
    std::uint64_t packet_sequence = 0;
    std::uint64_t first_sample_timestamp_ns = 0;
    std::uint16_t sample_count = 0;
    std::vector<std::uint8_t> bytes;
};

struct OusterImuPacketContract {
    std::uint32_t packet_bytes = 0;
    std::uint16_t measurements_per_packet = 0;
    std::uint16_t packets_per_frame = 0;
    std::chrono::nanoseconds sample_period{};
    bool legacy = false;
};

/// Resample simulator kinematics and encode native Ouster IMU UDP packets.
class OusterImuPacketPipeline {
public:
    OusterImuPacketPipeline(
        OusterMetadata metadata,
        std::chrono::nanoseconds nominal_lidar_frame_period,
        std::uint64_t sensor_stream_id,
        std::uint64_t initial_epoch,
        std::size_t maximum_catch_up_samples = 128);
    ~OusterImuPacketPipeline();

    OusterImuPacketPipeline(OusterImuPacketPipeline &&) noexcept;
    OusterImuPacketPipeline & operator=(OusterImuPacketPipeline &&) noexcept;
    OusterImuPacketPipeline(const OusterImuPacketPipeline &) = delete;
    OusterImuPacketPipeline & operator=(const OusterImuPacketPipeline &) = delete;

    std::vector<EncodedOusterImuPacket> ingest(const OusterImuState & state);
    void reset(std::uint64_t new_epoch);

    const OusterImuPacketContract & contract() const noexcept;
    std::uint64_t epoch() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ouster_sim_core
