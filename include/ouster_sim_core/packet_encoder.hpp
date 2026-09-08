// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/metadata.hpp"
#include "ouster_sim_core/revolution_assembler.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace ouster_sim_core {

/// Non-owning, row-major frame contract for producers with existing buffers.
/// All spans must remain valid until synchronous encoding returns.
struct OusterScanFrameView {
    std::uint64_t revolution = 0;
    std::int64_t frame_start_timestamp_ns = 0;
    std::uint32_t columns_per_frame = 0;
    std::uint16_t pixels_per_column = 0;
    std::span<const std::uint64_t> column_timestamp_ns;
    std::span<const std::uint32_t> range_mm;
    std::span<const std::uint16_t> signal;
    std::span<const std::uint8_t> reflectivity;
    std::span<const std::uint16_t> near_ir;

    static OusterScanFrameView fromFrame(const OusterScanFrame & frame);
};

struct EncodedLidarPacket {
    std::uint64_t revolution = 0;
    std::uint32_t frame_id = 0;
    std::uint16_t first_measurement_id = 0;
    std::uint64_t first_column_timestamp_ns = 0;
    std::vector<std::uint8_t> bytes;
};

/// Deterministic synchronous Ouster lidar packet encoder.
///
/// Publication and pacing belong to the outer package. This class owns only
/// the hardware byte contract, including modern packet headers and CRC.
class OusterPacketEncoder {
public:
    explicit OusterPacketEncoder(OusterMetadata metadata);

    const OusterMetadata & metadata() const noexcept { return metadata_; }
    std::vector<EncodedLidarPacket> encode(
        const OusterScanFrame & frame) const;
    std::vector<EncodedLidarPacket> encode(
        const OusterScanFrameView & frame) const;

    /// Reuse packet storage. The complete input is validated before output
    /// mutation; invalid frames cannot partially replace a pending batch.
    void encode(const OusterScanFrameView & frame,
                std::vector<EncodedLidarPacket> & packets) const;

private:
    OusterMetadata metadata_;
};

}  // namespace ouster_sim_core
