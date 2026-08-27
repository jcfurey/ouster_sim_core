// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/packet_encoder.hpp"

#include <ouster/impl/packet_writer.h>
#include <ouster/types.h>

#include <cstring>
#include <stdexcept>
#include <utility>

namespace ouster_sim_core {
namespace {

void validateFrame(
    const OusterScanFrame & frame,
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

    const std::size_t count = frame.sampleCount();
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

}  // namespace

OusterPacketEncoder::OusterPacketEncoder(OusterMetadata metadata)
    : metadata_(std::move(metadata))
{
    metadata_.requirePrimaryReturnProfile();
}

std::vector<EncodedLidarPacket> OusterPacketEncoder::encode(
    const OusterScanFrame & frame) const
{
    validateFrame(frame, metadata_);

    const auto & writer = metadata_.packetWriter();
    const auto width = metadata_.columnsPerFrame();
    const auto columns_per_packet = metadata_.columnsPerPacket();
    const auto packet_count = width / columns_per_packet;
    const std::uint32_t packet_frame_id =
        metadata_.packetFrameId(frame.revolution);

    std::vector<EncodedLidarPacket> packets;
    packets.reserve(packet_count);
    for (std::uint32_t packet_index = 0;
         packet_index < packet_count; ++packet_index) {
        EncodedLidarPacket packet;
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
            writer.set_col_status(column, 0x01u);
        }

        writer.set_block<std::uint32_t>(
            frame.range_mm.data(), static_cast<int>(width),
            ouster::sdk::core::ChanField::RANGE, data);
        if (writer.field_type(ouster::sdk::core::ChanField::SIGNAL) !=
            ouster::sdk::core::ChanFieldType::VOID) {
            writer.set_block<std::uint16_t>(
                frame.signal.data(), static_cast<int>(width),
                ouster::sdk::core::ChanField::SIGNAL, data);
        }
        if (writer.field_type(ouster::sdk::core::ChanField::REFLECTIVITY) !=
            ouster::sdk::core::ChanFieldType::VOID) {
            writer.set_block<std::uint8_t>(
                frame.reflectivity.data(), static_cast<int>(width),
                ouster::sdk::core::ChanField::REFLECTIVITY, data);
        }
        if (writer.field_type(ouster::sdk::core::ChanField::NEAR_IR) !=
            ouster::sdk::core::ChanFieldType::VOID) {
            writer.set_block<std::uint16_t>(
                frame.near_ir.data(), static_cast<int>(width),
                ouster::sdk::core::ChanField::NEAR_IR, data);
        }

        if (writer.udp_profile_lidar !=
                ouster::sdk::core::UDPProfileLidar::LEGACY &&
            writer.header_type == ouster::sdk::core::HeaderType::STANDARD) {
            const std::uint64_t crc =
                writer.calculate_crc(data, packet.bytes.size());
            std::memcpy(
                data + packet.bytes.size() - sizeof(crc),
                &crc,
                sizeof(crc));
        }
        packets.push_back(std::move(packet));
    }
    return packets;
}

}  // namespace ouster_sim_core
