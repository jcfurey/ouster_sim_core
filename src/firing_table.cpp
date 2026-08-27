// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/firing_table.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace ouster_sim_core {
namespace {

constexpr std::size_t kMaxChannels = 256;
constexpr std::uint32_t kMaxColumns = 4096;
constexpr double kNanosecondsPerSecond = 1.0e9;

void requireFinite(const char * name, double value)
{
    if (!std::isfinite(value)) {
        throw std::invalid_argument(std::string(name) + " must be finite");
    }
}

std::int64_t firingTimeOffsetNs(
    std::int64_t scan_period_ns,
    std::uint32_t measurement_id,
    std::uint32_t columns_per_frame)
{
    const std::int64_t columns = static_cast<std::int64_t>(columns_per_frame);
    const std::int64_t ordinal =
        static_cast<std::int64_t>(measurement_id) + 1;
    return (scan_period_ns / columns) * ordinal +
           ((scan_period_ns % columns) * ordinal) / columns;
}

}  // namespace

std::size_t ScheduledColumnBatch::rayCount(
    std::size_t channel_count) const
{
    if (channel_count == 0) {
        throw std::invalid_argument("channel_count must be greater than zero");
    }
    if (column_count >
        std::numeric_limits<std::size_t>::max() / channel_count) {
        throw std::overflow_error("scheduled ray count exceeds size_t");
    }
    return static_cast<std::size_t>(column_count) * channel_count;
}

OusterFiringTable::OusterFiringTable(OusterFiringTableConfig config)
    : channel_count_(config.beam_altitude_deg.size()),
      columns_per_frame_(config.columns_per_frame),
      lidar_hz_(config.lidar_hz),
      beam_origin_m_(config.lidar_origin_to_beam_origin_m)
{
    if (channel_count_ == 0 || channel_count_ > kMaxChannels) {
        throw std::invalid_argument(
            "beam_altitude_deg must contain between 1 and 256 channels");
    }
    if (columns_per_frame_ == 0 || columns_per_frame_ > kMaxColumns) {
        throw std::invalid_argument(
            "columns_per_frame must be between 1 and 4096");
    }
    requireFinite("lidar_hz", lidar_hz_);
    if (lidar_hz_ <= 0.0) {
        throw std::invalid_argument("lidar_hz must be greater than zero");
    }
    requireFinite(
        "lidar_origin_to_beam_origin_m",
        beam_origin_m_);
    if (beam_origin_m_ < 0.0) {
        throw std::invalid_argument(
            "lidar_origin_to_beam_origin_m must not be negative");
    }

    if (config.beam_azimuth_deg.empty()) {
        config.beam_azimuth_deg.resize(channel_count_, 0.0);
    } else if (config.beam_azimuth_deg.size() != channel_count_) {
        throw std::invalid_argument(
            "beam_azimuth_deg size must be zero or match beam_altitude_deg");
    }

    for (std::size_t ring = 0; ring < channel_count_; ++ring) {
        requireFinite("beam altitude", config.beam_altitude_deg[ring]);
        requireFinite("beam azimuth", config.beam_azimuth_deg[ring]);
        if (config.beam_altitude_deg[ring] < -90.0 ||
            config.beam_altitude_deg[ring] > 90.0) {
            throw std::invalid_argument(
                "beam altitude must be in the closed interval [-90, 90]");
        }
    }

    const long double period_ns =
        static_cast<long double>(kNanosecondsPerSecond) /
        static_cast<long double>(lidar_hz_);
    if (period_ns < 1.0L ||
        period_ns >
            static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
        throw std::invalid_argument("lidar_hz produces an invalid scan period");
    }
    scan_period_ns_ = static_cast<std::int64_t>(std::llround(period_ns));

    if (channel_count_ >
        std::numeric_limits<std::size_t>::max() / columns_per_frame_) {
        throw std::overflow_error("Ouster firing table dimensions overflow");
    }
    firings_.reserve(channel_count_ * columns_per_frame_);

    constexpr double kPi = 3.141592653589793238462643383279502884;
    constexpr double kDegreesToRadians = kPi / 180.0;
    const double radians_per_column =
        2.0 * kPi / static_cast<double>(columns_per_frame_);

    // Ouster measurement IDs increase clockwise when viewed from above.
    // Store column-major so every complete vertical rack is contiguous.
    for (std::uint32_t measurement = 0;
         measurement < columns_per_frame_; ++measurement) {
        const double encoder_azimuth =
            -static_cast<double>(measurement) * radians_per_column;
        const Vector3d origin{
            beam_origin_m_ * std::cos(encoder_azimuth),
            beam_origin_m_ * std::sin(encoder_azimuth),
            0.0};

        for (std::size_t ring = 0; ring < channel_count_; ++ring) {
            const double elevation =
                config.beam_altitude_deg[ring] * kDegreesToRadians;
            // Match the Ouster SDK XYZ LUT: calibrated beam azimuth is
            // subtracted from the clockwise encoder azimuth.
            const double ray_azimuth =
                encoder_azimuth -
                config.beam_azimuth_deg[ring] * kDegreesToRadians;
            const double cos_elevation = std::cos(elevation);

            OusterFiring firing;
            firing.measurement_id = measurement;
            firing.ring_id = static_cast<std::uint16_t>(ring);
            firing.linear_index = static_cast<std::uint32_t>(
                static_cast<std::size_t>(measurement) * channel_count_ + ring);
            firing.time_offset_ns = firingTimeOffsetNs(
                scan_period_ns_, measurement, columns_per_frame_);
            firing.origin_m = origin;
            firing.direction = {
                cos_elevation * std::cos(ray_azimuth),
                cos_elevation * std::sin(ray_azimuth),
                std::sin(elevation)};
            firings_.push_back(firing);
        }
    }
}

const OusterFiring & OusterFiringTable::at(
    std::uint32_t measurement_id, std::uint16_t ring_id) const
{
    if (measurement_id >= columns_per_frame_) {
        throw std::out_of_range("measurement_id exceeds columns_per_frame");
    }
    if (static_cast<std::size_t>(ring_id) >= channel_count_) {
        throw std::out_of_range("ring_id exceeds channel_count");
    }
    return firings_[
        static_cast<std::size_t>(measurement_id) * channel_count_ + ring_id];
}

OusterFiringIdentity OusterFiringTable::identity(
    const ScheduledColumnBatch & batch, std::size_t ray_offset) const
{
    const std::size_t ray_count = batch.rayCount(channel_count_);
    if (ray_offset >= ray_count) {
        throw std::out_of_range("ray_offset exceeds scheduled batch");
    }

    const std::uint64_t column_offset = ray_offset / channel_count_;
    if (batch.first_global_column >
        std::numeric_limits<std::uint64_t>::max() - column_offset) {
        throw std::overflow_error("global column identity overflow");
    }
    const std::uint64_t global_column =
        batch.first_global_column + column_offset;
    const auto measurement = static_cast<std::uint32_t>(
        global_column % columns_per_frame_);
    const auto ring = static_cast<std::uint16_t>(
        ray_offset % channel_count_);
    const OusterFiring & firing = at(measurement, ring);

    return {
        global_column / columns_per_frame_,
        measurement,
        ring,
        firing.linear_index,
        firing.time_offset_ns,
        0};
}

OusterColumnScheduler::OusterColumnScheduler(
    const OusterFiringTable & table)
    : columns_per_second_(
          static_cast<long double>(table.columnsPerFrame()) *
          static_cast<long double>(table.lidarHz()))
{
}

ScheduledColumnBatch OusterColumnScheduler::advance(double dt_seconds)
{
    if (!std::isfinite(dt_seconds) || dt_seconds < 0.0) {
        throw std::invalid_argument(
            "scheduler dt_seconds must be finite and non-negative");
    }

    long double available = fractional_column_credit_ +
        static_cast<long double>(dt_seconds) * columns_per_second_;
    const long double tolerance =
        64.0L * std::numeric_limits<long double>::epsilon() *
        std::max(1.0L, std::fabs(available));
    available += tolerance;

    if (available >
        static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
        throw std::overflow_error("scheduled column count exceeds uint64_t");
    }
    const auto column_count =
        static_cast<std::uint64_t>(std::floor(available));
    fractional_column_credit_ = available -
        static_cast<long double>(column_count) - tolerance;
    if (fractional_column_credit_ < 0.0L &&
        fractional_column_credit_ > -tolerance) {
        fractional_column_credit_ = 0.0L;
    }

    if (next_global_column_ >
        std::numeric_limits<std::uint64_t>::max() - column_count) {
        throw std::overflow_error("global column scheduler overflow");
    }
    ScheduledColumnBatch batch{next_global_column_, column_count};
    next_global_column_ += column_count;
    return batch;
}

void OusterColumnScheduler::reset(
    std::uint64_t first_global_column) noexcept
{
    next_global_column_ = first_global_column;
    fractional_column_credit_ = 0.0L;
}

}  // namespace ouster_sim_core
