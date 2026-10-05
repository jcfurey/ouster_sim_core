// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/packet_encoder.hpp"

#include <ouster/impl/packet_writer.h>
#include <ouster/types.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ouster_sim_core {
namespace {

void validateFrame(
    const OusterScanFrameView & frame,
    const OusterMetadata & metadata)
{
    if (frame.columns_per_frame != metadata.columnsPerFrame() ||
        frame.pixels_per_column != metadata.pixelsPerColumn()) {
        throw std::invalid_argument(
            "scan frame dimensions do not match Ouster metadata");
    }
    if (frame.frame_start_timestamp_ns < 0) {
        throw std::invalid_argument(
            "scan frame start timestamp must be non-negative");
    }

    const std::size_t count = static_cast<std::size_t>(frame.columns_per_frame) *
        frame.pixels_per_column;
    if (frame.column_timestamp_ns.size() != frame.columns_per_frame ||
        frame.range_mm.size() != count || frame.signal.size() != count ||
        frame.reflectivity.size() != count || frame.near_ir.size() != count) {
        throw std::invalid_argument(
            "scan frame channel or timestamp vector has the wrong size");
    }
    for (std::size_t measurement = 0;
         measurement < frame.column_timestamp_ns.size(); ++measurement) {
        if (frame.column_timestamp_ns[measurement] == 0) {
            throw std::invalid_argument(
                "Ouster column timestamps must be nonzero");
        }
        if (measurement != 0 &&
            frame.column_timestamp_ns[measurement] <=
                frame.column_timestamp_ns[measurement - 1]) {
            throw std::invalid_argument(
                "Ouster column timestamps must increase strictly");
        }
    }
    for (const std::uint32_t range_mm : frame.range_mm) {
        if (!metadata.isRangeEncodable(range_mm)) {
            throw std::invalid_argument(
                "scan frame RANGE value " + std::to_string(range_mm) +
                " mm is not exactly encodable by Ouster UDP lidar profile '" +
                metadata.activeLidarUdpProfile() + "' (value mask " +
                std::to_string(metadata.encodableRangeMaskMm()) + ")");
        }
    }
}

// Legacy packets carry a 90112-count angle encoder per column.
constexpr std::uint32_t kLegacyEncoderTicksPerRevolution = 90112;
constexpr std::uint32_t kLegacyValidColumnStatus = 0xffffffffu;
constexpr std::size_t kLegacyColumnFrameIdOffset = 10;
constexpr std::size_t kLegacyColumnEncoderOffset = 12;

void writeLittleEndian(std::uint8_t * output, std::uint64_t value,
                       std::size_t bytes)
{
    for (std::size_t index = 0; index < bytes; ++index) {
        output[index] = static_cast<std::uint8_t>(value >> (index * 8u));
    }
}

template <typename T>
bool fieldNeedsSaturation(
    const ouster::sdk::core::impl::PacketWriter & writer,
    const std::string & field)
{
    if (writer.field_type(field) == ouster::sdk::core::ChanFieldType::VOID) {
        return false;
    }
    return writer.field_value_mask(field) <
        static_cast<std::uint64_t>(std::numeric_limits<T>::max());
}

/// Return a block source that the SDK writer cannot wrap modulo its width.
///
/// Narrow packet fields such as the low-data NIR8 slot store a shifted subset
/// of the source bits. The SDK writer masks, so an out-of-range value would
/// alias to an unrelated dim value; a physical channel saturates instead.
template <typename T>
const T * saturatedBlock(
    const ouster::sdk::core::impl::PacketWriter & writer,
    const std::string & field,
    std::span<const T> source,
    std::vector<T> & scratch)
{
    if (!fieldNeedsSaturation<T>(writer, field)) {
        return source.data();
    }
    const auto maximum = static_cast<T>(writer.field_value_mask(field));
    scratch.resize(source.size());
    std::transform(source.begin(), source.end(), scratch.begin(),
                   [maximum](T value) { return std::min(value, maximum); });
    return scratch.data();
}

}  // namespace

OusterPacketEncoder::OusterPacketEncoder(OusterMetadata metadata)
    : metadata_(std::move(metadata))
{
    metadata_.requirePrimaryReturnProfile();
}

OusterScanFrameView OusterScanFrameView::fromFrame(const OusterScanFrame & frame)
{
    return {frame.revolution, frame.frame_start_timestamp_ns,
            frame.columns_per_frame, frame.pixels_per_column,
            frame.column_timestamp_ns, frame.range_mm, frame.signal,
            frame.reflectivity, frame.near_ir};
}

std::vector<EncodedLidarPacket> OusterPacketEncoder::encode(
    const OusterScanFrame & frame) const
{
    return encode(OusterScanFrameView::fromFrame(frame));
}

std::vector<EncodedLidarPacket> OusterPacketEncoder::encode(
    const OusterScanFrameView & frame) const
{
    std::vector<EncodedLidarPacket> packets;
    encode(frame, packets);
    return packets;
}

void OusterPacketEncoder::encode(
    const OusterScanFrameView & frame,
    std::vector<EncodedLidarPacket> & packets) const
{
    validateFrame(frame, metadata_);

    const auto & writer = metadata_.packetWriter();
    const auto width = metadata_.columnsPerFrame();
    const auto columns_per_packet = metadata_.columnsPerPacket();
    const auto frame_packet_count = width / columns_per_packet;
    const std::uint32_t packet_frame_id =
        metadata_.packetFrameId(frame.revolution);
    const bool legacy =
        writer.udp_profile_lidar == ouster::sdk::core::UDPProfileLidar::LEGACY;
    const std::uint32_t legacy_encoder_ticks_per_column =
        kLegacyEncoderTicksPerRevolution / width;
    const std::uint32_t valid_column_status =
        legacy ? kLegacyValidColumnStatus : 0x01u;

    std::vector<std::uint16_t> signal_scratch;
    std::vector<std::uint8_t> reflectivity_scratch;
    std::vector<std::uint16_t> near_ir_scratch;
    const auto * signal = saturatedBlock(
        writer, ouster::sdk::core::ChanField::SIGNAL, frame.signal,
        signal_scratch);
    const auto * reflectivity = saturatedBlock(
        writer, ouster::sdk::core::ChanField::REFLECTIVITY, frame.reflectivity,
        reflectivity_scratch);
    const auto * near_ir = saturatedBlock(
        writer, ouster::sdk::core::ChanField::NEAR_IR, frame.near_ir,
        near_ir_scratch);

    // A sensor omits packets wholly outside its column window, so the output
    // holds only metadata.lidarPacketsPerFrame() packets.
    packets.resize(metadata_.lidarPacketsPerFrame());
    std::size_t output_index = 0;
    for (std::uint32_t packet_index = 0;
         packet_index < frame_packet_count; ++packet_index) {
        if (!metadata_.isPacketInWindow(packet_index)) {
            continue;
        }
        auto & packet = packets[output_index++];
        packet.revolution = frame.revolution;
        packet.frame_id = packet_frame_id;
        packet.first_measurement_id = static_cast<std::uint16_t>(
            packet_index * columns_per_packet);
        packet.first_column_timestamp_ns =
            frame.column_timestamp_ns[packet.first_measurement_id];
        packet.bytes.assign(writer.lidar_packet_size, 0);
        auto * data = packet.bytes.data();

        writer.set_frame_id(data, packet_frame_id);
        writer.set_init_id(data, metadata_.initializationId());
        writer.set_prod_sn(data, metadata_.sensorSerial());
        writer.set_packet_type(data, 0x1u);
        writer.set_alert_flags(data, 0u);
        writer.set_shutdown(data, 0u);
        writer.set_shot_limiting(data, 0u);
        writer.set_shutdown_countdown(data, 0u);
        writer.set_shot_limiting_countdown(data, 0u);

        for (std::uint16_t local_column = 0;
             local_column < columns_per_packet; ++local_column) {
            const auto measurement = static_cast<std::uint16_t>(
                packet.first_measurement_id + local_column);
            auto * column = writer.nth_col(local_column, data);
            writer.set_col_timestamp(
                column, frame.column_timestamp_ns[measurement]);
            writer.set_col_measurement_id(column, measurement);
            // Out-of-window columns keep their identity and timestamp but are
            // invalid; the SDK writer leaves their channel data zeroed.
            writer.set_col_status(
                column,
                metadata_.isColumnInWindow(measurement)
                    ? valid_column_status
                    : 0u);
            if (legacy) {
                // Legacy sensors repeat the frame ID in every column header
                // and report the angle encoder; the SDK writer sets only the
                // first column's frame ID.
                writeLittleEndian(
                    column + kLegacyColumnFrameIdOffset, packet_frame_id, 2);
                writeLittleEndian(
                    column + kLegacyColumnEncoderOffset,
                    static_cast<std::uint64_t>(measurement) *
                        legacy_encoder_ticks_per_column,
                    4);
            }
        }

        writer.set_block<std::uint32_t>(
            frame.range_mm.data(), static_cast<int>(width),
            ouster::sdk::core::ChanField::RANGE, data);
        if (writer.field_type(ouster::sdk::core::ChanField::SIGNAL) !=
            ouster::sdk::core::ChanFieldType::VOID) {
            writer.set_block<std::uint16_t>(
                signal, static_cast<int>(width),
                ouster::sdk::core::ChanField::SIGNAL, data);
        }
        if (writer.field_type(ouster::sdk::core::ChanField::REFLECTIVITY) !=
            ouster::sdk::core::ChanFieldType::VOID) {
            writer.set_block<std::uint8_t>(
                reflectivity, static_cast<int>(width),
                ouster::sdk::core::ChanField::REFLECTIVITY, data);
        }
        if (writer.field_type(ouster::sdk::core::ChanField::NEAR_IR) !=
            ouster::sdk::core::ChanFieldType::VOID) {
            writer.set_block<std::uint16_t>(
                near_ir, static_cast<int>(width),
                ouster::sdk::core::ChanField::NEAR_IR, data);
        }

        if (!legacy &&
            writer.header_type == ouster::sdk::core::HeaderType::STANDARD) {
            const std::uint64_t crc =
                writer.calculate_crc(data, packet.bytes.size());
            std::memcpy(
                data + packet.bytes.size() - sizeof(crc),
                &crc,
                sizeof(crc));
        }
    }
}

}  // namespace ouster_sim_core
