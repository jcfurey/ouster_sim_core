// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/revolution_assembler.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace {

using ouster_sim_core::OusterFiringTable;
using ouster_sim_core::OusterFiringTableConfig;
using ouster_sim_core::OusterReturnSample;
using ouster_sim_core::OusterRevolutionAssembler;
using ouster_sim_core::ScheduledColumnBatch;

std::shared_ptr<const OusterFiringTable> makeTable()
{
    OusterFiringTableConfig config;
    config.columns_per_frame = 4;
    config.lidar_hz = 10.0;
    config.beam_altitude_deg = {1.0, -1.0};
    config.beam_azimuth_deg = {0.25, -0.25};
    config.lidar_origin_to_beam_origin_m = 0.015;
    return std::make_shared<const OusterFiringTable>(std::move(config));
}

std::vector<OusterReturnSample> makeReturns(
    const OusterFiringTable & table,
    std::uint64_t first_global_column,
    std::uint64_t column_count)
{
    const ScheduledColumnBatch batch{first_global_column, column_count};
    std::vector<OusterReturnSample> returns;
    returns.reserve(batch.rayCount(table.channelCount()));
    for (std::size_t offset = 0;
         offset < batch.rayCount(table.channelCount()); ++offset) {
        OusterReturnSample sample;
        sample.identity = table.identity(batch, offset);
        sample.is_hit = (sample.identity.linear_index % 3u) != 0u;
        if (sample.is_hit) {
            sample.range_mm = 1'000u + sample.identity.linear_index;
            sample.signal = static_cast<std::uint16_t>(100 + offset);
            sample.reflectivity = static_cast<std::uint8_t>(10 + offset);
            sample.near_ir = static_cast<std::uint16_t>(200 + offset);
        }
        returns.push_back(sample);
    }
    return returns;
}

TEST(OusterRevolutionAssembler, BatchSegmentationDoesNotChangeFrame)
{
    const auto table = makeTable();
    auto returns = makeReturns(*table, 0, table->columnsPerFrame());
    OusterRevolutionAssembler assembler(table, 1'000'000'000);

    const auto first = assembler.ingest(
        std::span<const OusterReturnSample>(returns).first(3));
    EXPECT_TRUE(first.empty());
    EXPECT_EQ(assembler.pendingSampleCount(), 3u);

    const auto completed = assembler.ingest(
        std::span<const OusterReturnSample>(returns).subspan(3));
    ASSERT_EQ(completed.size(), 1u);
    const auto & frame = completed.front();
    EXPECT_EQ(frame.revolution, 0u);
    EXPECT_EQ(frame.columns_per_frame, 4u);
    EXPECT_EQ(frame.pixels_per_column, 2u);
    EXPECT_EQ(frame.column_timestamp_ns.front(), 1'025'000'000u);
    EXPECT_EQ(frame.column_timestamp_ns.back(), 1'100'000'000u);

    // Source index 1 is measurement 0/ring 1, but packet image index 4.
    const std::size_t sdk_index = frame.sdkImageIndex(0, 1);
    EXPECT_EQ(sdk_index, 4u);
    EXPECT_EQ(frame.range_mm[sdk_index], 1001u);
    EXPECT_EQ(frame.signal[sdk_index], 101u);
    EXPECT_EQ(frame.reflectivity[sdk_index], 11u);
    EXPECT_EQ(frame.near_ir[sdk_index], 201u);

    // Source index 0 was a true miss and remains a zero-range pixel.
    EXPECT_EQ(frame.range_mm[frame.sdkImageIndex(0, 0)], 0u);
    EXPECT_EQ(assembler.expectedRevolution(), 1u);
    EXPECT_EQ(assembler.expectedLinearIndex(), 0u);
}

TEST(OusterRevolutionAssembler, OneBatchMaySpanMultipleRevolutions)
{
    const auto table = makeTable();
    const auto returns = makeReturns(*table, 0, 8);
    OusterRevolutionAssembler assembler(table, 5'000'000'000);
    const auto completed = assembler.ingest(returns);
    ASSERT_EQ(completed.size(), 2u);
    EXPECT_EQ(completed[0].revolution, 0u);
    EXPECT_EQ(completed[1].revolution, 1u);
    EXPECT_EQ(
        completed[1].frame_start_timestamp_ns -
            completed[0].frame_start_timestamp_ns,
        table->scanPeriodNs());
}

TEST(OusterRevolutionAssembler, RejectsIdentityRepairInputsTransactionally)
{
    const auto table = makeTable();
    auto returns = makeReturns(*table, 0, table->columnsPerFrame());
    OusterRevolutionAssembler assembler(table, 1'000'000'000);

    returns[2].identity.ring_id = 1;
    EXPECT_THROW(assembler.ingest(returns), std::invalid_argument);
    EXPECT_EQ(assembler.expectedRevolution(), 0u);
    EXPECT_EQ(assembler.expectedLinearIndex(), 0u);

    returns = makeReturns(*table, 0, table->columnsPerFrame());
    returns[0].range_mm = 1'000u;
    EXPECT_THROW(assembler.ingest(returns), std::invalid_argument);
    EXPECT_EQ(assembler.expectedLinearIndex(), 0u);

    returns = makeReturns(*table, 0, table->columnsPerFrame());
    returns[0].identity.return_index = 1;
    EXPECT_THROW(assembler.ingest(returns), std::invalid_argument);
    EXPECT_EQ(assembler.expectedLinearIndex(), 0u);
}

}  // namespace
