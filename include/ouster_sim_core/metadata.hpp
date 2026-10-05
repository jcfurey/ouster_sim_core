// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/firing_table.hpp"
#include "ouster_sim_core/product_profile.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ouster::sdk::core::impl {
class PacketWriter;
}

namespace ouster_sim_core {

class OusterPacketEncoder;
class OusterImuPacketPipeline;

/// SDK-neutral semantic firmware version parsed from source metadata.
struct OusterFirmwareVersion {
    std::uint16_t major = 0;
    std::uint16_t minor = 0;
    std::uint16_t patch = 0;
    std::string version_string;
};

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
    std::size_t imuPacketSize() const noexcept;
    std::uint32_t imuMeasurementsPerPacket() const noexcept;
    std::uint32_t imuPacketsPerFrame() const noexcept;
    bool legacyImuProfile() const noexcept;
    const std::string & activeImuUdpProfile() const noexcept;
    std::uint64_t sensorSerial() const noexcept;
    std::uint32_t initializationId() const noexcept;

    /// Packet-header frame identity for a monotonically increasing revolution.
    ///
    /// Standard and legacy packet layouts wrap at 16 bits; FUSA packet
    /// layouts wrap at 32 bits. This semantic conversion keeps simulator
    /// adapters from duplicating SDK-specific header-width rules.
    std::uint32_t packetFrameId(std::uint64_t revolution) const noexcept;

    /// Inclusive measurement-ID window over which the sensor reports data.
    ///
    /// `first > last` denotes a window that wraps through measurement 0. A
    /// sensor emits only packets containing at least one in-window column and
    /// reports out-of-window columns in those packets as invalid.
    std::uint32_t columnWindowFirst() const noexcept;
    std::uint32_t columnWindowLast() const noexcept;
    bool isColumnInWindow(std::uint32_t measurement_id) const noexcept;
    bool fullColumnWindow() const noexcept;

    /// Lidar packets a sensor emits per frame with the column window applied.
    std::uint32_t lidarPacketsPerFrame() const noexcept;
    bool isPacketInWindow(std::uint32_t packet_index) const noexcept;

    /// Active source UDP lidar profile as an SDK-neutral stable name.
    const std::string & activeLidarUdpProfile() const noexcept;

    /// Number of packet return slots selected by the active UDP profile.
    std::uint8_t activeReturnCount() const noexcept;

    /// Bit mask of millimetre values exactly representable by RANGE.
    ///
    /// The mask also expresses alignment. For example RNG15 profiles return
    /// 0x3fff8: range values must be no greater than 262136 and divisible by
    /// eight.
    std::uint32_t encodableRangeMaskMm() const noexcept;
    std::uint32_t maximumEncodableRangeMm() const noexcept;
    bool isRangeEncodable(std::uint32_t range_mm) const noexcept;

    /// Reject profiles whose packet contract requires secondary returns.
    ///
    /// The current shared channel and frame contract is intentionally
    /// primary-return-only. Callers can inspect the profile and return count
    /// before invoking this gate.
    void requirePrimaryReturnProfile() const;

    const std::string & productLine() const noexcept;
    const std::string & sourceProductPartNumber() const noexcept;
    const OusterFirmwareVersion & sourceFirmwareVersion() const noexcept;
    const std::string & sourceJson() const noexcept;
    const std::string & publishedJson() const noexcept;
    bool firmwareAdvertisementAdjusted() const noexcept;

    const std::vector<double> & beamAltitudeDeg() const noexcept;
    const std::vector<double> & beamAzimuthDeg() const noexcept;
    double beamOriginM() const noexcept;

    OusterFiringTableConfig firingTableConfig(double lidar_hz) const;

    /// Build a physical-product request from the unmodified source metadata.
    ///
    /// A missing hardware override preserves automatic revision inference. A
    /// missing low-data override derives the value from the source UDP packet
    /// profile. Publication-only firmware adjustments never affect this
    /// request.
    OusterProductProfileRequest productProfileRequest(
        std::optional<std::string> hardware_revision = std::nullopt,
        std::optional<bool> low_data_profile = std::nullopt) const;

    OusterProductProfile resolvedProductProfile(
        std::optional<std::string> hardware_revision = std::nullopt,
        std::optional<bool> low_data_profile = std::nullopt) const;

private:
    struct Impl;
    explicit OusterMetadata(std::shared_ptr<Impl> impl);
    const ouster::sdk::core::impl::PacketWriter & packetWriter() const noexcept;

    std::shared_ptr<Impl> impl_;
    friend class OusterPacketEncoder;
    friend class OusterImuPacketPipeline;
};

}  // namespace ouster_sim_core
