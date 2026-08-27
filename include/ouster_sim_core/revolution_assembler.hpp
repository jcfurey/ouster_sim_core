// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/firing_table.hpp"
#include "ouster_sim_core/return_sample.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace ouster_sim_core {

/// One complete source-identified revolution in Ouster SDK image layout.
///
/// Channel vectors are row-major H x W: ring * W + measurement_id. This is
/// deliberately distinct from OusterFiringIdentity::linear_index, whose
/// column-major order reflects ray dispatch: measurement_id * H + ring.
struct OusterScanFrame {
    std::uint64_t revolution = 0;
    std::int64_t frame_start_timestamp_ns = 0;
    std::uint32_t columns_per_frame = 0;
    std::uint16_t pixels_per_column = 0;

    std::vector<std::uint64_t> column_timestamp_ns;
    std::vector<std::uint32_t> range_mm;
    std::vector<std::uint16_t> signal;
    std::vector<std::uint8_t> reflectivity;
    std::vector<std::uint16_t> near_ir;

    std::size_t sampleCount() const noexcept;
    std::size_t sdkImageIndex(
        std::uint32_t measurement_id, std::uint16_t ring_id) const;
};

/// Strictly assembles immutable source returns into complete revolutions.
///
/// Batch boundaries may occur anywhere, including across revolutions, but
/// records must retain the exact source order and identity assigned before ray
/// dispatch. The assembler rejects gaps, duplicates, reordering, fabricated
/// indices, nonzero miss ranges, and unsupported secondary returns.
class OusterRevolutionAssembler {
public:
    OusterRevolutionAssembler(
        std::shared_ptr<const OusterFiringTable> firing_table,
        std::int64_t revolution_zero_timestamp_ns,
        std::uint64_t first_revolution = 0);

    std::vector<OusterScanFrame> ingest(
        std::span<const OusterReturnSample> samples);

    /// Discard any partial frame and restart at an explicit time/identity.
    /// The complete first-frame timestamp domain is validated before state is
    /// changed; on failure the previous partial frame and identity remain.
    void reset(
        std::int64_t revolution_zero_timestamp_ns,
        std::uint64_t first_revolution = 0);

    std::uint64_t expectedRevolution() const noexcept {
        return expected_revolution_;
    }
    std::uint32_t expectedLinearIndex() const noexcept {
        return expected_linear_index_;
    }
    std::size_t pendingSampleCount() const noexcept {
        return expected_linear_index_;
    }

private:
    OusterScanFrame makeFrame(std::uint64_t revolution) const;
    std::int64_t frameStartTimestamp(std::uint64_t revolution) const;

    std::shared_ptr<const OusterFiringTable> firing_table_;
    std::int64_t revolution_zero_timestamp_ns_ = 0;
    std::uint64_t expected_revolution_ = 0;
    std::uint32_t expected_linear_index_ = 0;
    std::optional<OusterScanFrame> pending_frame_;
};

}  // namespace ouster_sim_core
