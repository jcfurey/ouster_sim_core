// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/metadata.hpp"
#include "ouster_sim_core/revolution_assembler.hpp"

#include <cstdint>
#include <vector>

namespace ouster_sim_core {

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

private:
    OusterMetadata metadata_;
};

}  // namespace ouster_sim_core
