// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/firing_table.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace ouster::sdk::core::impl {
class PacketWriter;
}

namespace ouster_sim_core {

class OusterPacketEncoder;

/// Validated Ouster metadata and its packet-layout contract.
///
/// SDK types stay behind the implementation pointer so simulator-facing public
/// headers remain free of Ouster SDK details. Instances are immutable and
/// cheap to copy.
class OusterMetadata {
public:
    static constexpr std::uintmax_t kDefaultMaximumFileBytes =
        10u * 1024u * 1024u;

    static OusterMetadata fromFile(
        const std::filesystem::path & path,
        std::uintmax_t maximum_file_bytes = kDefaultMaximumFileBytes);
    static OusterMetadata fromJson(std::string json);

    std::uint32_t columnsPerFrame() const noexcept;
    std::uint16_t pixelsPerColumn() const noexcept;
    std::uint16_t columnsPerPacket() const noexcept;
    std::size_t lidarPacketSize() const noexcept;
    std::uint64_t sensorSerial() const noexcept;
    std::uint32_t initializationId() const noexcept;

    const std::string & productLine() const noexcept;
    const std::string & sourceJson() const noexcept;
    const std::string & publishedJson() const noexcept;
    bool firmwareAdvertisementAdjusted() const noexcept;

    const std::vector<double> & beamAltitudeDeg() const noexcept;
    const std::vector<double> & beamAzimuthDeg() const noexcept;
    double beamOriginM() const noexcept;

    OusterFiringTableConfig firingTableConfig(double lidar_hz) const;

private:
    struct Impl;
    explicit OusterMetadata(std::shared_ptr<Impl> impl);
    const ouster::sdk::core::impl::PacketWriter & packetWriter() const noexcept;

    std::shared_ptr<Impl> impl_;
    friend class OusterPacketEncoder;
};

}  // namespace ouster_sim_core
