// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/packet_pacing.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace ouster_sim_core {
namespace {

constexpr auto kMinimumProducerPeriod = std::chrono::microseconds(100);

PacketPacingTimePoint checkedAdd(
    PacketPacingTimePoint base,
    std::chrono::nanoseconds offset)
{
    const auto base_ticks = base.time_since_epoch().count();
    const auto offset_ticks = offset.count();
    constexpr auto kMaximum = std::numeric_limits<std::int64_t>::max();
    constexpr auto kMinimum = std::numeric_limits<std::int64_t>::min();

    if ((offset_ticks > 0 && base_ticks > kMaximum - offset_ticks) ||
        (offset_ticks < 0 && base_ticks < kMinimum - offset_ticks)) {
        throw std::overflow_error("packet pacing deadline exceeds int64_t");
    }
    return PacketPacingTimePoint{
        std::chrono::nanoseconds(base_ticks + offset_ticks)};
}

std::optional<std::chrono::nanoseconds> positiveDifference(
    PacketPacingTimePoint newer,
    PacketPacingTimePoint older) noexcept
{
    const auto newer_ticks = newer.time_since_epoch().count();
    const auto older_ticks = older.time_since_epoch().count();
    if (newer_ticks <= older_ticks) return std::nullopt;

    constexpr auto kMaximum = std::numeric_limits<std::int64_t>::max();
    if (older_ticks < 0 && newer_ticks > kMaximum + older_ticks) {
        return std::chrono::nanoseconds::max();
    }
    return std::chrono::nanoseconds(newer_ticks - older_ticks);
}

}  // namespace

std::chrono::nanoseconds packetBatchDrainSpan(
    std::chrono::nanoseconds nominal_period,
    std::optional<std::chrono::nanoseconds> observed_period)
{
    if (nominal_period <= std::chrono::nanoseconds::zero()) {
        throw std::invalid_argument(
            "nominal_frame_period must be greater than zero");
    }

    auto source = observed_period.has_value() &&
                          *observed_period > std::chrono::nanoseconds::zero()
        ? *observed_period
        : nominal_period;
    source = std::max(
        source,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            kMinimumProducerPeriod));

    // Divide before multiplying so nanoseconds::max() remains representable.
    const auto ticks = source.count();
    const auto scaled = (ticks / 5) * 4 + ((ticks % 5) * 4) / 5;
    return std::chrono::nanoseconds(std::max<std::int64_t>(1, scaled));
}

PacketPacingPolicy::PacketPacingPolicy(
    std::chrono::nanoseconds nominal_frame_period,
    PacketDeliveryMode delivery_mode)
    : nominal_frame_period_(nominal_frame_period),
      delivery_mode_(delivery_mode)
{
    // Reuse the public calculation so its validation remains the single
    // definition of a valid nominal period.
    static_cast<void>(packetBatchDrainSpan(nominal_frame_period_));
}

PacketPacingFramePlan PacketPacingPolicy::beginFrame(
    PacketPacingTimePoint produced_at,
    PacketPacingTimePoint drain_started_at,
    std::size_t packet_count)
{
    if (paused_) {
        throw std::logic_error("cannot begin a packet frame while paused");
    }
    if (active_) {
        throw std::logic_error(
            "cannot begin a packet frame before completing the active frame");
    }
    if (packet_count == 0) {
        throw std::invalid_argument("packet_count must be greater than zero");
    }
    if (drain_started_at < produced_at) {
        throw std::invalid_argument(
            "drain_started_at must not precede produced_at");
    }

    std::optional<std::chrono::nanoseconds> observed_period;
    if (previous_produced_at_.has_value()) {
        observed_period = positiveDifference(
            produced_at, *previous_produced_at_);
    }
    const bool uses_observation = observed_period.has_value();
    const auto producer_period = uses_observation
        ? *observed_period
        : nominal_frame_period_;
    const auto drain_span = delivery_mode_ == PacketDeliveryMode::kBurst
        ? std::chrono::nanoseconds::zero()
        : packetBatchDrainSpan(nominal_frame_period_, observed_period);

    if (delivery_mode_ == PacketDeliveryMode::kPaced &&
        packet_count > static_cast<std::size_t>(drain_span.count())) {
        throw std::invalid_argument(
            "packet_count exceeds the drain span's nanosecond resolution");
    }
    const auto packet_spacing = std::chrono::nanoseconds(
        delivery_mode_ == PacketDeliveryMode::kBurst
            ? 0
            : drain_span.count() /
                  static_cast<std::int64_t>(packet_count));

    // Validate the complete absolute schedule before mutating policy state.
    const auto last_offset = std::chrono::nanoseconds(
        packet_spacing.count() *
        static_cast<std::int64_t>(packet_count - 1));
    const auto last_deadline = checkedAdd(drain_started_at, last_offset);

    previous_produced_at_ = produced_at;
    active_ = true;
    packet_count_ = packet_count;
    next_packet_index_ = 0;
    packet_spacing_ = packet_spacing;
    next_deadline_ = drain_started_at;

    return PacketPacingFramePlan{
        packet_count,
        uses_observation,
        producer_period,
        drain_span,
        packet_spacing,
        drain_started_at,
        last_deadline};
}

std::optional<PacketPacingTimePoint>
PacketPacingPolicy::nextDeadline() const noexcept
{
    if (!active_ || paused_) return std::nullopt;
    return next_deadline_;
}

void PacketPacingPolicy::markPacketPublished()
{
    if (!active_) {
        throw std::logic_error("no active packet frame");
    }
    if (paused_) {
        throw std::logic_error("cannot publish a packet while paused");
    }

    if (next_packet_index_ + 1 == packet_count_) {
        active_ = false;
        packet_count_ = 0;
        next_packet_index_ = 0;
        packet_spacing_ = std::chrono::nanoseconds::zero();
        next_deadline_ = PacketPacingTimePoint{};
        return;
    }

    // Compute before mutating the index so deadline overflow has strong
    // exception safety.
    const auto following_deadline = checkedAdd(
        next_deadline_, packet_spacing_);
    ++next_packet_index_;
    next_deadline_ = following_deadline;
}

void PacketPacingPolicy::pause() noexcept
{
    if (paused_) return;
    paused_ = true;
    previous_produced_at_.reset();
}

void PacketPacingPolicy::resume(PacketPacingTimePoint resumed_at)
{
    if (!paused_) return;

    std::optional<PacketPacingTimePoint> shifted_deadline;
    if (active_) {
        shifted_deadline = checkedAdd(resumed_at, packet_spacing_);
    }

    if (shifted_deadline.has_value()) {
        next_deadline_ = *shifted_deadline;
    }
    paused_ = false;
}

void PacketPacingPolicy::reset() noexcept
{
    previous_produced_at_.reset();
    active_ = false;
    packet_count_ = 0;
    next_packet_index_ = 0;
    packet_spacing_ = std::chrono::nanoseconds::zero();
    next_deadline_ = PacketPacingTimePoint{};
}

}  // namespace ouster_sim_core
