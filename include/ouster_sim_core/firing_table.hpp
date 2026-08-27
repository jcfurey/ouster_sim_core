// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ouster_sim_core {

struct Vector3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/// Static geometry and within-frame timing of one Ouster laser firing.
struct OusterFiring {
    std::uint32_t measurement_id = 0;
    std::uint16_t ring_id = 0;
    std::uint32_t linear_index = 0;
    std::int64_t time_offset_ns = 0;
    Vector3d origin_m;
    Vector3d direction;
};

/// Source-owned identity assigned before a ray is dispatched to a simulator.
struct OusterFiringIdentity {
    std::uint64_t revolution = 0;
    std::uint32_t measurement_id = 0;
    std::uint16_t ring_id = 0;
    std::uint32_t linear_index = 0;
    std::int64_t time_offset_ns = 0;
    std::uint8_t return_index = 0;
};

struct OusterFiringTableConfig {
    std::uint32_t columns_per_frame = 0;
    double lidar_hz = 0.0;
    std::vector<double> beam_altitude_deg;
    std::vector<double> beam_azimuth_deg;
    double lidar_origin_to_beam_origin_m = 0.0;
};

/// A contiguous source batch containing complete Ouster measurement columns.
struct ScheduledColumnBatch {
    std::uint64_t first_global_column = 0;
    std::uint64_t column_count = 0;

    std::size_t rayCount(std::size_t channel_count) const;
    bool empty() const noexcept { return column_count == 0; }
};

/// Exact half-open H x W calibrated firing geometry for one revolution.
class OusterFiringTable {
public:
    explicit OusterFiringTable(OusterFiringTableConfig config);

    std::size_t channelCount() const noexcept { return channel_count_; }
    std::uint32_t columnsPerFrame() const noexcept {
        return columns_per_frame_;
    }
    std::size_t sampleCount() const noexcept { return firings_.size(); }
    double lidarHz() const noexcept { return lidar_hz_; }
    std::int64_t scanPeriodNs() const noexcept { return scan_period_ns_; }
    double beamOriginM() const noexcept { return beam_origin_m_; }

    const OusterFiring & at(
        std::uint32_t measurement_id, std::uint16_t ring_id) const;

    OusterFiringIdentity identity(
        const ScheduledColumnBatch & batch, std::size_t ray_offset) const;

private:
    std::size_t channel_count_ = 0;
    std::uint32_t columns_per_frame_ = 0;
    double lidar_hz_ = 0.0;
    double beam_origin_m_ = 0.0;
    std::int64_t scan_period_ns_ = 0;
    std::vector<OusterFiring> firings_;
};

/// Physics-step-independent scheduler. It emits whole columns and preserves a
/// fractional column credit between calls.
class OusterColumnScheduler {
public:
    explicit OusterColumnScheduler(const OusterFiringTable & table);

    ScheduledColumnBatch advance(double dt_seconds);
    void reset(std::uint64_t first_global_column = 0) noexcept;

    std::uint64_t nextGlobalColumn() const noexcept {
        return next_global_column_;
    }
    long double fractionalColumnCredit() const noexcept {
        return fractional_column_credit_;
    }

private:
    long double columns_per_second_ = 0.0L;
    long double fractional_column_credit_ = 0.0L;
    std::uint64_t next_global_column_ = 0;
};

}  // namespace ouster_sim_core
