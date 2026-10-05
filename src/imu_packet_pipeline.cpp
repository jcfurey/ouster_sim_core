// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/imu_packet_pipeline.hpp"

#include <ouster/impl/packet_writer.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace ouster_sim_core {
namespace {

constexpr std::int64_t kLegacySamplePeriodNs = 10'000'000;
constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

void requireFinite(const OusterImuState & state)
{
    if (state.timestamp_ns < 0) {
        throw std::invalid_argument("IMU state timestamp must not be negative");
    }
    for (const double value : state.linear_acceleration_mps2) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("IMU acceleration must be finite");
        }
    }
    for (const double value : state.angular_velocity_rad_s) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("IMU angular velocity must be finite");
        }
    }
}

void writeLittleEndianU64(std::uint8_t * output, std::uint64_t value)
{
    for (unsigned index = 0; index < 8; ++index) {
        output[index] = static_cast<std::uint8_t>(value >> (index * 8u));
    }
}

float checkedFloat(double value, const char * field)
{
    if (value < -std::numeric_limits<float>::max() ||
        value > std::numeric_limits<float>::max()) {
        throw std::overflow_error(std::string(field) + " exceeds float");
    }
    return static_cast<float>(value);
}

std::uint16_t checkedU16(std::uint64_t value, const char * field)
{
    if (value > std::numeric_limits<std::uint16_t>::max()) {
        throw std::invalid_argument(std::string(field) + " exceeds uint16");
    }
    return static_cast<std::uint16_t>(value);
}

struct Sample {
    std::uint64_t timestamp_ns = 0;
    std::array<double, 3> acceleration{};
    std::array<double, 3> angular_velocity{};
    std::uint64_t index = 0;
};

}  // namespace

struct OusterImuPacketPipeline::Impl {
    Impl(
        OusterMetadata metadata_value,
        std::chrono::nanoseconds frame_period,
        std::uint64_t stream,
        std::uint64_t initial_epoch,
        std::size_t maximum_catch_up)
        : metadata(std::move(metadata_value)), writer(metadata.packetWriter()),
          sensor_stream_id(stream), epoch_value(initial_epoch),
          maximum_catch_up_samples(maximum_catch_up)
    {
        if (frame_period <= std::chrono::nanoseconds::zero()) {
            throw std::invalid_argument(
                "nominal lidar frame period must be positive");
        }
        if (maximum_catch_up_samples == 0) {
            throw std::invalid_argument(
                "maximum IMU catch-up sample count must be positive");
        }
        if (metadata.imuPacketSize() == 0 ||
            metadata.imuPacketSize() >
                std::numeric_limits<std::uint32_t>::max()) {
            throw std::invalid_argument("metadata has no valid IMU packet layout");
        }

        contract_value.packet_bytes = static_cast<std::uint32_t>(
            metadata.imuPacketSize());
        contract_value.legacy = metadata.legacyImuProfile();
        if (contract_value.legacy) {
            contract_value.measurements_per_packet = 1;
            contract_value.sample_period =
                std::chrono::nanoseconds(kLegacySamplePeriodNs);
            const auto packets = frame_period.count() / kLegacySamplePeriodNs;
            contract_value.packets_per_frame = checkedU16(
                std::max<std::int64_t>(packets, 1),
                "legacy IMU packets per frame");
        } else {
            contract_value.measurements_per_packet = checkedU16(
                metadata.imuMeasurementsPerPacket(),
                "IMU measurements per packet");
            contract_value.packets_per_frame = checkedU16(
                metadata.imuPacketsPerFrame(), "IMU packets per frame");
            const std::uint64_t samples_per_frame =
                static_cast<std::uint64_t>(
                    contract_value.measurements_per_packet) *
                contract_value.packets_per_frame;
            if (samples_per_frame == 0 ||
                frame_period.count() %
                    static_cast<std::int64_t>(samples_per_frame) != 0) {
                throw std::invalid_argument(
                    "modern IMU cadence does not divide the lidar frame period");
            }
            contract_value.sample_period = std::chrono::nanoseconds(
                frame_period.count() /
                static_cast<std::int64_t>(samples_per_frame));
            if (metadata.columnsPerFrame() % samples_per_frame != 0) {
                throw std::invalid_argument(
                    "IMU cadence does not divide lidar measurement IDs");
            }
        }
        if (contract_value.measurements_per_packet == 0 ||
            contract_value.packets_per_frame == 0 ||
            contract_value.sample_period <= std::chrono::nanoseconds::zero()) {
            throw std::invalid_argument("invalid Ouster IMU packet contract");
        }
        pending_samples.reserve(contract_value.measurements_per_packet);
    }

    EncodedOusterImuPacket encodePending()
    {
        if (pending_samples.size() != contract_value.measurements_per_packet) {
            throw std::logic_error("cannot encode an incomplete IMU packet");
        }
        EncodedOusterImuPacket output;
        output.sensor_stream_id = sensor_stream_id;
        output.epoch = epoch_value;
        output.packet_sequence = next_packet_sequence++;
        output.first_sample_timestamp_ns = pending_samples.front().timestamp_ns;
        output.sample_count = contract_value.measurements_per_packet;
        output.bytes.assign(contract_value.packet_bytes, 0);
        auto * packet = output.bytes.data();

        if (contract_value.legacy) {
            const auto & sample = pending_samples.front();
            writeLittleEndianU64(packet + 0, sample.timestamp_ns);
            writeLittleEndianU64(packet + 8, sample.timestamp_ns);
            writeLittleEndianU64(packet + 16, sample.timestamp_ns);
            writer.set_imu_la_x(packet, checkedFloat(
                sample.acceleration[0] / kStandardGravityMps2,
                "legacy IMU acceleration"));
            writer.set_imu_la_y(packet, checkedFloat(
                sample.acceleration[1] / kStandardGravityMps2,
                "legacy IMU acceleration"));
            writer.set_imu_la_z(packet, checkedFloat(
                sample.acceleration[2] / kStandardGravityMps2,
                "legacy IMU acceleration"));
            writer.set_imu_av_x(packet, checkedFloat(
                sample.angular_velocity[0] * kRadiansToDegrees,
                "legacy IMU angular velocity"));
            writer.set_imu_av_y(packet, checkedFloat(
                sample.angular_velocity[1] * kRadiansToDegrees,
                "legacy IMU angular velocity"));
            writer.set_imu_av_z(packet, checkedFloat(
                sample.angular_velocity[2] * kRadiansToDegrees,
                "legacy IMU angular velocity"));
            return output;
        }

        const std::uint64_t samples_per_frame =
            static_cast<std::uint64_t>(contract_value.measurements_per_packet) *
            contract_value.packets_per_frame;
        const std::uint64_t frame_index =
            pending_samples.front().index / samples_per_frame;
        writer.set_packet_type(packet, 0x2);
        writer.set_frame_id(packet, metadata.packetFrameId(frame_index));
        writer.set_init_id(packet, metadata.initializationId());
        writer.set_prod_sn(packet, metadata.sensorSerial());
        writer.set_imu_nmea_ts(packet, output.first_sample_timestamp_ns);

        const std::uint64_t measurement_stride =
            metadata.columnsPerFrame() / samples_per_frame;
        for (std::size_t index = 0; index < pending_samples.size(); ++index) {
            const auto & sample = pending_samples[index];
            auto * measurement = writer.imu_nth_measurement(
                static_cast<int>(index), packet);
            const std::uint64_t frame_sample = sample.index % samples_per_frame;
            writer.set_col_timestamp(measurement, sample.timestamp_ns);
            writer.set_col_measurement_id(measurement, checkedU16(
                frame_sample * measurement_stride, "IMU measurement ID"));
            writer.set_col_status(measurement, 1u);
            writer.set_imu_la_x(measurement, checkedFloat(
                sample.acceleration[0], "IMU acceleration"));
            writer.set_imu_la_y(measurement, checkedFloat(
                sample.acceleration[1], "IMU acceleration"));
            writer.set_imu_la_z(measurement, checkedFloat(
                sample.acceleration[2], "IMU acceleration"));
            writer.set_imu_av_x(measurement, checkedFloat(
                sample.angular_velocity[0], "IMU angular velocity"));
            writer.set_imu_av_y(measurement, checkedFloat(
                sample.angular_velocity[1], "IMU angular velocity"));
            writer.set_imu_av_z(measurement, checkedFloat(
                sample.angular_velocity[2], "IMU angular velocity"));
        }
        const std::uint64_t crc = writer.calculate_crc(
            output.bytes.data(), output.bytes.size());
        writeLittleEndianU64(output.bytes.data() + output.bytes.size() - 8, crc);
        return output;
    }

    Sample interpolate(const OusterImuState & state, std::int64_t deadline)
    {
        const std::int64_t span = state.timestamp_ns - previous->timestamp_ns;
        const double alpha = span > 0
            ? std::clamp(
                  static_cast<double>(deadline - previous->timestamp_ns) /
                      static_cast<double>(span),
                  0.0, 1.0)
            : 1.0;
        Sample sample;
        sample.timestamp_ns = static_cast<std::uint64_t>(deadline);
        sample.index = next_sample_index++;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            sample.acceleration[axis] =
                previous->linear_acceleration_mps2[axis] +
                (state.linear_acceleration_mps2[axis] -
                 previous->linear_acceleration_mps2[axis]) * alpha;
            sample.angular_velocity[axis] =
                previous->angular_velocity_rad_s[axis] +
                (state.angular_velocity_rad_s[axis] -
                 previous->angular_velocity_rad_s[axis]) * alpha;
        }
        return sample;
    }

    OusterMetadata metadata;
    const ouster::sdk::core::impl::PacketWriter & writer;
    OusterImuPacketContract contract_value;
    std::uint64_t sensor_stream_id = 0;
    std::uint64_t epoch_value = 0;
    std::size_t maximum_catch_up_samples = 0;
    std::optional<OusterImuState> previous;
    std::optional<std::int64_t> next_sample_timestamp_ns;
    std::uint64_t next_sample_index = 0;
    std::uint64_t next_packet_sequence = 0;
    std::vector<Sample> pending_samples;
};

OusterImuPacketPipeline::OusterImuPacketPipeline(
    OusterMetadata metadata,
    std::chrono::nanoseconds nominal_lidar_frame_period,
    std::uint64_t sensor_stream_id,
    std::uint64_t initial_epoch,
    std::size_t maximum_catch_up_samples)
    : impl_(std::make_unique<Impl>(
          std::move(metadata), nominal_lidar_frame_period, sensor_stream_id,
          initial_epoch, maximum_catch_up_samples))
{}

OusterImuPacketPipeline::~OusterImuPacketPipeline() = default;
OusterImuPacketPipeline::OusterImuPacketPipeline(
    OusterImuPacketPipeline &&) noexcept = default;
OusterImuPacketPipeline & OusterImuPacketPipeline::operator=(
    OusterImuPacketPipeline &&) noexcept = default;

std::vector<EncodedOusterImuPacket> OusterImuPacketPipeline::ingest(
    const OusterImuState & state)
{
    requireFinite(state);
    if (impl_->previous && state.timestamp_ns < impl_->previous->timestamp_ns) {
        throw std::invalid_argument(
            "IMU state timestamp moved backwards without an epoch reset");
    }
    if (!impl_->previous) {
        impl_->previous = state;
        impl_->next_sample_timestamp_ns = state.timestamp_ns;
    }

    const auto period = impl_->contract_value.sample_period.count();
    if (*impl_->next_sample_timestamp_ns <= state.timestamp_ns) {
        const std::uint64_t due = static_cast<std::uint64_t>(
            (state.timestamp_ns - *impl_->next_sample_timestamp_ns) / period) +
            1u;
        if (due > impl_->maximum_catch_up_samples) {
            const std::uint64_t skipped = due - impl_->maximum_catch_up_samples;
            if (skipped >
                std::numeric_limits<std::uint64_t>::max() -
                    impl_->next_sample_index ||
                skipped > static_cast<std::uint64_t>(
                    std::numeric_limits<std::int64_t>::max() / period)) {
                throw std::overflow_error("IMU sample sequence would wrap");
            }
            impl_->next_sample_index += skipped;
            *impl_->next_sample_timestamp_ns +=
                static_cast<std::int64_t>(skipped) * period;
            impl_->pending_samples.clear();
        }
    }

    std::vector<EncodedOusterImuPacket> output;
    while (*impl_->next_sample_timestamp_ns <= state.timestamp_ns) {
        impl_->pending_samples.push_back(impl_->interpolate(
            state, *impl_->next_sample_timestamp_ns));
        if (*impl_->next_sample_timestamp_ns >
            std::numeric_limits<std::int64_t>::max() - period) {
            throw std::overflow_error("IMU sample timestamp would wrap");
        }
        *impl_->next_sample_timestamp_ns += period;
        if (impl_->pending_samples.size() ==
            impl_->contract_value.measurements_per_packet) {
            // Drop the samples of a packet that cannot be encoded; keeping
            // them would leave the buffer permanently over-full and stop all
            // further packets.
            try {
                output.push_back(impl_->encodePending());
            } catch (...) {
                impl_->pending_samples.clear();
                impl_->previous = state;
                throw;
            }
            impl_->pending_samples.clear();
        }
    }
    impl_->previous = state;
    return output;
}

void OusterImuPacketPipeline::reset(std::uint64_t new_epoch)
{
    if (new_epoch <= impl_->epoch_value) {
        throw std::invalid_argument("IMU pipeline epoch must increase");
    }
    impl_->epoch_value = new_epoch;
    impl_->previous.reset();
    impl_->next_sample_timestamp_ns.reset();
    impl_->next_sample_index = 0;
    impl_->next_packet_sequence = 0;
    impl_->pending_samples.clear();
}

const OusterImuPacketContract & OusterImuPacketPipeline::contract() const noexcept
{
    return impl_->contract_value;
}

std::uint64_t OusterImuPacketPipeline::epoch() const noexcept
{
    return impl_->epoch_value;
}

}  // namespace ouster_sim_core
