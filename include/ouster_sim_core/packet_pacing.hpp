// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <cstddef>
#include <optional>

namespace ouster_sim_core {

using PacketPacingTimePoint = std::chrono::time_point<
    std::chrono::steady_clock, std::chrono::nanoseconds>;

/// Select hardware-like packet spacing or immediate whole-frame delivery.
enum class PacketDeliveryMode {
    kPaced,
    kBurst,
};

/// Spread a frame over 80% of its observed producer interval.
///
/// A missing or non-positive observation falls back to `nominal_period`. The
/// selected source interval is clamped to 100 microseconds before scaling, as
/// in the established Gazebo pacing policy. This affects delivery deadlines
/// only; it must never be used to derive packet acquisition timestamps.
std::chrono::nanoseconds packetBatchDrainSpan(
    std::chrono::nanoseconds nominal_period,
    std::optional<std::chrono::nanoseconds> observed_period = std::nullopt);

/// Immutable description of the active frame's absolute delivery schedule.
struct PacketPacingFramePlan {
    std::size_t packet_count = 0;
    bool uses_observed_period = false;
    std::chrono::nanoseconds producer_period{};
    std::chrono::nanoseconds drain_span{};
    std::chrono::nanoseconds packet_spacing{};
    PacketPacingTimePoint first_deadline{};
    PacketPacingTimePoint last_deadline{};
};

/// Pure, clock-injected packet pacing state machine.
///
/// The caller owns clocks, waits, threads, queues, and publication. Producer
/// arrival times select the frame drain span, while `drain_started_at` anchors
/// absolute deadlines so individual late wakeups do not accumulate drift.
/// Call `markPacketPublished()` after each successful publication.
///
/// Burst delivery assigns every packet the `drain_started_at` deadline. A
/// pause invalidates the producer-period observation. On resume, paced
/// delivery shifts the next packet by one normal spacing; burst delivery is
/// immediately due. `reset()` cancels the active frame and clears cadence
/// history but intentionally preserves the paused state.
class PacketPacingPolicy {
public:
    explicit PacketPacingPolicy(
        std::chrono::nanoseconds nominal_frame_period,
        PacketDeliveryMode delivery_mode = PacketDeliveryMode::kPaced);

    const std::chrono::nanoseconds & nominalFramePeriod() const noexcept {
        return nominal_frame_period_;
    }

    PacketDeliveryMode deliveryMode() const noexcept {
        return delivery_mode_;
    }

    PacketPacingFramePlan beginFrame(
        PacketPacingTimePoint produced_at,
        PacketPacingTimePoint drain_started_at,
        std::size_t packet_count);

    bool hasActiveFrame() const noexcept { return active_; }
    bool paused() const noexcept { return paused_; }
    std::size_t nextPacketIndex() const noexcept { return next_packet_index_; }
    std::size_t packetCount() const noexcept { return packet_count_; }

    /// The next absolute deadline, or no value while paused or inactive.
    std::optional<PacketPacingTimePoint> nextDeadline() const noexcept;

    /// Advance after publishing the packet selected by `nextPacketIndex()`.
    void markPacketPublished();

    /// Idempotently enter the paused state and invalidate cadence history.
    void pause() noexcept;

    /// Idempotently resume. The active frame's next packet becomes due one
    /// packet spacing after `resumed_at` (immediately in burst mode).
    void resume(PacketPacingTimePoint resumed_at);

    /// Cancel any active frame and make the next frame use nominal cadence.
    /// The paused state is preserved so reset and lifecycle state stay
    /// orthogonal.
    void reset() noexcept;

private:
    std::chrono::nanoseconds nominal_frame_period_{};
    PacketDeliveryMode delivery_mode_ = PacketDeliveryMode::kPaced;
    std::optional<PacketPacingTimePoint> previous_produced_at_;

    bool active_ = false;
    bool paused_ = false;
    std::size_t packet_count_ = 0;
    std::size_t next_packet_index_ = 0;
    std::chrono::nanoseconds packet_spacing_{};
    PacketPacingTimePoint next_deadline_{};
};

}  // namespace ouster_sim_core
