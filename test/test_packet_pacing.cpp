// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/packet_pacing.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using namespace std::chrono_literals;
using ouster_sim_core::PacketPacingPolicy;
using ouster_sim_core::PacketPacingTimePoint;
using ouster_sim_core::packetBatchDrainSpan;

PacketPacingTimePoint at(std::chrono::nanoseconds elapsed)
{
    return PacketPacingTimePoint{elapsed};
}

void finishFrame(PacketPacingPolicy & policy)
{
    while (policy.hasActiveFrame()) {
        ASSERT_TRUE(policy.nextDeadline().has_value());
        policy.markPacketPublished();
    }
}

TEST(PacketPacing, DrainSpanMatchesGazeboPolicy)
{
    EXPECT_EQ(packetBatchDrainSpan(100ms), 80ms);
    EXPECT_EQ(packetBatchDrainSpan(100ms, 50ms), 40ms);
    EXPECT_EQ(packetBatchDrainSpan(100ms, 200ms), 160ms);

    // Invalid observations retain the nominal fallback behavior.
    EXPECT_EQ(packetBatchDrainSpan(100ms, 0ns), 80ms);
    EXPECT_EQ(packetBatchDrainSpan(100ms, -1ms), 80ms);

    // The source interval is clamped before the 80% drain fraction.
    EXPECT_EQ(packetBatchDrainSpan(1ns), 80us);
    EXPECT_EQ(
        packetBatchDrainSpan(std::chrono::nanoseconds::max()),
        std::chrono::nanoseconds(
            (std::chrono::nanoseconds::max().count() / 5) * 4 +
            ((std::chrono::nanoseconds::max().count() % 5) * 4) / 5));
}

TEST(PacketPacing, TracksQuarterThroughQuadrupleRateProducerCadence)
{
    PacketPacingPolicy policy(100ms);
    auto produced = at(1s);

    const auto first = policy.beginFrame(produced, produced, 4);
    EXPECT_FALSE(first.uses_observed_period);
    EXPECT_EQ(first.producer_period, 100ms);
    EXPECT_EQ(first.drain_span, 80ms);
    finishFrame(policy);

    const std::vector<std::chrono::nanoseconds> intervals{
        400ms, 200ms, 100ms, 50ms, 25ms};  // 0.25x through 4x.
    const std::vector<std::chrono::nanoseconds> expected_spans{
        320ms, 160ms, 80ms, 40ms, 20ms};

    for (std::size_t i = 0; i < intervals.size(); ++i) {
        produced += intervals[i];
        const auto plan = policy.beginFrame(produced, produced + 1ms, 4);
        EXPECT_TRUE(plan.uses_observed_period);
        EXPECT_EQ(plan.producer_period, intervals[i]);
        EXPECT_EQ(plan.drain_span, expected_spans[i]);
        finishFrame(policy);
    }
}

TEST(PacketPacing, ExposesDriftFreeAbsoluteDeadlines)
{
    PacketPacingPolicy policy(100ms);
    const auto start = at(5s);
    const auto plan = policy.beginFrame(start - 2ms, start, 4);

    EXPECT_EQ(plan.packet_spacing, 20ms);
    EXPECT_EQ(plan.first_deadline, start);
    EXPECT_EQ(plan.last_deadline, start + 60ms);

    for (std::size_t i = 0; i < 4; ++i) {
        ASSERT_EQ(policy.nextPacketIndex(), i);
        ASSERT_TRUE(policy.nextDeadline().has_value());
        EXPECT_EQ(*policy.nextDeadline(), start + 20ms * i);
        policy.markPacketPublished();
    }
    EXPECT_FALSE(policy.hasActiveFrame());
    EXPECT_FALSE(policy.nextDeadline().has_value());
}

TEST(PacketPacing, OnePacketIsDueImmediatelyAndCompletesAtomically)
{
    PacketPacingPolicy policy(100ms);
    const auto start = at(2s);
    const auto plan = policy.beginFrame(start, start, 1);

    EXPECT_EQ(plan.packet_spacing, 80ms);
    EXPECT_EQ(plan.first_deadline, start);
    EXPECT_EQ(plan.last_deadline, start);
    ASSERT_EQ(policy.nextDeadline(), start);

    policy.markPacketPublished();
    EXPECT_FALSE(policy.hasActiveFrame());
    EXPECT_EQ(policy.packetCount(), 0u);
}

TEST(PacketPacing, PauseResumeShiftsScheduleWithoutCatchUpBurst)
{
    PacketPacingPolicy policy(100ms);
    const auto produced = at(1s);
    const auto start = at(1100ms);
    policy.beginFrame(produced, start, 4);

    ASSERT_EQ(policy.nextDeadline(), start);
    policy.markPacketPublished();
    ASSERT_EQ(policy.nextDeadline(), start + 20ms);

    policy.pause();
    EXPECT_TRUE(policy.paused());
    EXPECT_FALSE(policy.nextDeadline().has_value());
    EXPECT_THROW(policy.markPacketPublished(), std::logic_error);

    const auto resumed_at = at(10s);
    policy.resume(resumed_at);
    EXPECT_FALSE(policy.paused());
    ASSERT_EQ(policy.nextPacketIndex(), 1u);
    ASSERT_EQ(policy.nextDeadline(), resumed_at + 20ms);

    // Every remaining packet retains one normal spacing; none are immediately
    // overdue at the resume time.
    policy.markPacketPublished();
    ASSERT_EQ(policy.nextDeadline(), resumed_at + 40ms);
    policy.markPacketPublished();
    ASSERT_EQ(policy.nextDeadline(), resumed_at + 60ms);
    policy.markPacketPublished();
    EXPECT_FALSE(policy.hasActiveFrame());

    // A pause transition also invalidates the producer observation, making
    // the following frame use the nominal period instead of the long pause.
    const auto after_pause = policy.beginFrame(
        at(10050ms), at(10051ms), 4);
    EXPECT_FALSE(after_pause.uses_observed_period);
    EXPECT_EQ(after_pause.drain_span, 80ms);
}

TEST(PacketPacing, ResetCancelsFrameAndRestoresNominalFallback)
{
    PacketPacingPolicy policy(100ms);
    policy.beginFrame(at(1s), at(1s), 4);
    finishFrame(policy);
    auto observed = policy.beginFrame(at(1050ms), at(1051ms), 4);
    EXPECT_TRUE(observed.uses_observed_period);
    EXPECT_EQ(observed.drain_span, 40ms);

    policy.markPacketPublished();
    policy.reset();
    EXPECT_FALSE(policy.hasActiveFrame());
    EXPECT_FALSE(policy.nextDeadline().has_value());

    const auto after_reset = policy.beginFrame(at(2s), at(2s), 4);
    EXPECT_FALSE(after_reset.uses_observed_period);
    EXPECT_EQ(after_reset.drain_span, 80ms);
}

TEST(PacketPacing, ResetPreservesIndependentPauseState)
{
    PacketPacingPolicy policy(100ms);
    policy.beginFrame(at(1s), at(1s), 4);
    policy.pause();
    policy.reset();

    EXPECT_TRUE(policy.paused());
    EXPECT_FALSE(policy.hasActiveFrame());
    EXPECT_THROW(
        policy.beginFrame(at(2s), at(2s), 4),
        std::logic_error);

    policy.resume(at(3s));
    const auto plan = policy.beginFrame(at(3s), at(3s), 4);
    EXPECT_FALSE(plan.uses_observed_period);
}

TEST(PacketPacing, RejectsInvalidConfigurationAndStateTransitions)
{
    EXPECT_THROW(PacketPacingPolicy(0ns), std::invalid_argument);
    EXPECT_THROW(PacketPacingPolicy(-1ns), std::invalid_argument);
    EXPECT_THROW(packetBatchDrainSpan(0ns), std::invalid_argument);

    PacketPacingPolicy policy(100us);  // 80 us drain span.
    EXPECT_THROW(
        policy.beginFrame(at(1s), at(1s), 0),
        std::invalid_argument);
    EXPECT_THROW(
        policy.beginFrame(at(1s), at(1s - 1ns), 1),
        std::invalid_argument);
    EXPECT_THROW(
        policy.beginFrame(at(1s), at(1s), 80'001),
        std::invalid_argument);

    policy.beginFrame(at(1s), at(1s), 1);
    EXPECT_THROW(
        policy.beginFrame(at(2s), at(2s), 1),
        std::logic_error);
    policy.markPacketPublished();
    EXPECT_THROW(policy.markPacketPublished(), std::logic_error);
}

TEST(PacketPacing, RejectsAnAbsoluteScheduleThatWouldOverflow)
{
    PacketPacingPolicy policy(100ms);
    const auto near_max = PacketPacingTimePoint{
        std::chrono::nanoseconds::max() - 10ms};

    EXPECT_THROW(
        policy.beginFrame(near_max, near_max, 4),
        std::overflow_error);
    EXPECT_FALSE(policy.hasActiveFrame());
}

TEST(PacketPacing, NonIncreasingProducerTimeFallsBackToNominal)
{
    PacketPacingPolicy policy(100ms);
    policy.beginFrame(at(2s), at(2s), 1);
    policy.markPacketPublished();

    const auto plan = policy.beginFrame(at(2s), at(2s), 1);
    EXPECT_FALSE(plan.uses_observed_period);
    EXPECT_EQ(plan.producer_period, 100ms);
    EXPECT_EQ(plan.drain_span, 80ms);
}

}  // namespace
