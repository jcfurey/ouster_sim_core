// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/revolution_assembler.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace ouster_sim_core {
namespace {

std::string identityDescription(const OusterFiringIdentity & identity)
{
    return "revolution=" + std::to_string(identity.revolution) +
           " measurement_id=" + std::to_string(identity.measurement_id) +
           " ring_id=" + std::to_string(identity.ring_id) +
           " linear_index=" + std::to_string(identity.linear_index);
}

}  // namespace

std::size_t OusterScanFrame::sampleCount() const noexcept
{
    return static_cast<std::size_t>(columns_per_frame) * pixels_per_column;
}

std::size_t OusterScanFrame::sdkImageIndex(
    std::uint32_t measurement_id, std::uint16_t ring_id) const
{
    if (measurement_id >= columns_per_frame) {
        throw std::out_of_range("measurement_id exceeds frame width");
    }
    if (ring_id >= pixels_per_column) {
        throw std::out_of_range("ring_id exceeds frame height");
    }
    return static_cast<std::size_t>(ring_id) * columns_per_frame +
           measurement_id;
}

OusterRevolutionAssembler::OusterRevolutionAssembler(
    std::shared_ptr<const OusterFiringTable> firing_table,
    std::int64_t revolution_zero_timestamp_ns,
    std::uint64_t first_revolution)
    : firing_table_(std::move(firing_table))
{
    if (!firing_table_) {
        throw std::invalid_argument("firing_table must not be null");
    }
    reset(revolution_zero_timestamp_ns, first_revolution);
}

std::int64_t OusterRevolutionAssembler::frameStartTimestamp(
    std::uint64_t revolution) const
{
    if (revolution_zero_timestamp_ns_ < 0) {
        throw std::invalid_argument(
            "revolution zero timestamp must be non-negative");
    }
    const auto period = firing_table_->scanPeriodNs();
    const auto maximum = std::numeric_limits<std::int64_t>::max();
    if (revolution >
        static_cast<std::uint64_t>(
            (maximum - revolution_zero_timestamp_ns_) / period)) {
        throw std::overflow_error("revolution timestamp exceeds int64_t");
    }
    return revolution_zero_timestamp_ns_ +
        static_cast<std::int64_t>(revolution) * period;
}

OusterScanFrame OusterRevolutionAssembler::makeFrame(
    std::uint64_t revolution) const
{
    OusterScanFrame frame;
    frame.revolution = revolution;
    frame.frame_start_timestamp_ns = frameStartTimestamp(revolution);
    frame.columns_per_frame = firing_table_->columnsPerFrame();
    frame.pixels_per_column =
        static_cast<std::uint16_t>(firing_table_->channelCount());

    frame.column_timestamp_ns.resize(frame.columns_per_frame);
    for (std::uint32_t measurement = 0;
         measurement < frame.columns_per_frame; ++measurement) {
        const auto offset = firing_table_->at(measurement, 0).time_offset_ns;
        if (frame.frame_start_timestamp_ns >
            std::numeric_limits<std::int64_t>::max() - offset) {
            throw std::overflow_error("column timestamp exceeds int64_t");
        }
        frame.column_timestamp_ns[measurement] = static_cast<std::uint64_t>(
            frame.frame_start_timestamp_ns + offset);
    }

    const std::size_t count = firing_table_->sampleCount();
    frame.range_mm.assign(count, 0);
    frame.signal.assign(count, 0);
    frame.reflectivity.assign(count, 0);
    frame.near_ir.assign(count, 0);
    return frame;
}

std::vector<OusterScanFrame> OusterRevolutionAssembler::ingest(
    std::span<const OusterReturnSample> samples)
{
    // Validate the complete batch before mutating assembly state. This makes
    // malformed batches transactional and prevents a partially consumed
    // identity stream from being mistaken for a later valid batch.
    std::uint64_t validation_revolution = expected_revolution_;
    std::uint32_t validation_linear = expected_linear_index_;
    const auto height = firing_table_->channelCount();
    const auto count = firing_table_->sampleCount();

    for (const auto & sample : samples) {
        if (validation_linear == 0) {
            static_cast<void>(frameStartTimestamp(validation_revolution));
        }
        const auto expected_measurement = static_cast<std::uint32_t>(
            validation_linear / height);
        const auto expected_ring = static_cast<std::uint16_t>(
            validation_linear % height);
        const auto & firing =
            firing_table_->at(expected_measurement, expected_ring);
        const auto & identity = sample.identity;

        if (identity.revolution != validation_revolution ||
            identity.measurement_id != expected_measurement ||
            identity.ring_id != expected_ring ||
            identity.linear_index != validation_linear ||
            identity.time_offset_ns != firing.time_offset_ns ||
            identity.return_index != 0) {
            throw std::invalid_argument(
                "source identity discontinuity at " +
                identityDescription(identity) + "; expected revolution=" +
                std::to_string(validation_revolution) +
                " linear_index=" + std::to_string(validation_linear));
        }

        if (sample.is_hit && sample.range_mm == 0) {
            throw std::invalid_argument(
                "a source hit must have nonzero integer range_mm");
        }
        if (!sample.is_hit &&
            (sample.range_mm != 0 || sample.signal != 0 ||
             sample.reflectivity != 0 || sample.near_ir != 0)) {
            throw std::invalid_argument(
                "a source miss must retain zero range and zero channels");
        }

        ++validation_linear;
        if (validation_linear == count) {
            validation_linear = 0;
            if (validation_revolution ==
                std::numeric_limits<std::uint64_t>::max()) {
                throw std::overflow_error("revolution identity overflow");
            }
            ++validation_revolution;
        }
    }

    std::vector<OusterScanFrame> completed;
    for (const auto & sample : samples) {
        if (!pending_frame_) {
            pending_frame_.emplace(makeFrame(expected_revolution_));
        }

        const std::size_t index = pending_frame_->sdkImageIndex(
            sample.identity.measurement_id, sample.identity.ring_id);
        if (sample.is_hit) {
            pending_frame_->range_mm[index] = sample.range_mm;
            pending_frame_->signal[index] = sample.signal;
            pending_frame_->reflectivity[index] = sample.reflectivity;
            pending_frame_->near_ir[index] = sample.near_ir;
        }

        ++expected_linear_index_;
        if (expected_linear_index_ == count) {
            completed.push_back(std::move(*pending_frame_));
            pending_frame_.reset();
            expected_linear_index_ = 0;
            ++expected_revolution_;
        }
    }
    return completed;
}

void OusterRevolutionAssembler::reset(
    std::int64_t revolution_zero_timestamp_ns,
    std::uint64_t first_revolution)
{
    if (revolution_zero_timestamp_ns < 0) {
        throw std::invalid_argument(
            "revolution zero timestamp must be non-negative");
    }

    // Validate the complete first-frame timestamp domain before committing
    // any state. In particular, a failed reset must not discard a partial
    // pre-reset revolution or leave the new origin paired with the old
    // expected identity.
    const auto period = firing_table_->scanPeriodNs();
    const auto maximum = std::numeric_limits<std::int64_t>::max();
    if (first_revolution >
        static_cast<std::uint64_t>(
            (maximum - revolution_zero_timestamp_ns) / period)) {
        throw std::overflow_error("revolution timestamp exceeds int64_t");
    }
    const std::int64_t first_frame_timestamp =
        revolution_zero_timestamp_ns +
        static_cast<std::int64_t>(first_revolution) * period;
    std::int64_t maximum_offset = 0;
    for (std::uint32_t measurement = 0;
         measurement < firing_table_->columnsPerFrame(); ++measurement) {
        maximum_offset = std::max(
            maximum_offset,
            firing_table_->at(measurement, 0).time_offset_ns);
    }
    if (first_frame_timestamp > maximum - maximum_offset) {
        throw std::overflow_error("column timestamp exceeds int64_t");
    }

    revolution_zero_timestamp_ns_ = revolution_zero_timestamp_ns;
    expected_revolution_ = first_revolution;
    expected_linear_index_ = 0;
    pending_frame_.reset();
}

}  // namespace ouster_sim_core
