// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/metadata.hpp"

#include <ouster/impl/packet_writer.h>
#include <ouster/lidar_scan.h>
#include <ouster/metadata.h>
#include <ouster/types.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ouster_sim_core {
namespace {

constexpr std::uint32_t kMaximumColumns = 4096;
constexpr std::uint16_t kMaximumChannels = 256;

bool isLowDataProfile(ouster::sdk::core::UDPProfileLidar profile)
{
    using Profile = ouster::sdk::core::UDPProfileLidar;
    return profile == Profile::RNG15_RFL8_NIR8 ||
           profile == Profile::FUSA_RNG15_RFL8_NIR8_DUAL ||
           profile == Profile::RNG15_RFL8_NIR8_DUAL ||
           profile == Profile::RNG15_RFL8_NIR8_ZONE16 ||
           profile == Profile::RNG15_RFL8_WIN8;
}

std::string readMetadataFile(
    const std::filesystem::path & path,
    std::uintmax_t maximum_file_bytes)
{
    if (maximum_file_bytes == 0) {
        throw std::invalid_argument("maximum metadata file size must be nonzero");
    }

    std::error_code error;
    const std::uintmax_t size = std::filesystem::file_size(path, error);
    if (error) {
        throw std::runtime_error(
            "cannot stat Ouster metadata '" + path.string() + "': " +
            error.message());
    }
    if (size > maximum_file_bytes) {
        throw std::length_error(
            "Ouster metadata '" + path.string() + "' is " +
            std::to_string(size) + " bytes; configured limit is " +
            std::to_string(maximum_file_bytes));
    }

    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw std::runtime_error(
            "cannot open Ouster metadata '" + path.string() + "'");
    }
    std::string json{
        std::istreambuf_iterator<char>(stream),
        std::istreambuf_iterator<char>()};
    if (!stream.good() && !stream.eof()) {
        throw std::runtime_error(
            "failed while reading Ouster metadata '" + path.string() + "'");
    }
    return json;
}

void validateFiniteAngles(
    const std::vector<double> & angles,
    const char * name,
    bool validate_altitude_range)
{
    for (const double angle : angles) {
        if (!std::isfinite(angle)) {
            throw std::invalid_argument(std::string(name) + " must be finite");
        }
        if (validate_altitude_range && (angle < -90.0 || angle > 90.0)) {
            throw std::invalid_argument(
                "beam altitude must be in the closed interval [-90, 90]");
        }
    }
}

}  // namespace

struct OusterMetadata::Impl {
    explicit Impl(std::string json)
        : source_json(std::move(json)),
          sensor_info(
              std::make_shared<ouster::sdk::core::SensorInfo>(source_json)),
          packet_format(*sensor_info),
          packet_writer(packet_format)
    {
        const auto height = packet_writer.pixels_per_column;
        const auto width = sensor_info->format.columns_per_frame;
        const auto columns_per_packet = packet_writer.columns_per_packet;

        // Preserve the physical-product inputs before mutating the SDK object
        // for publication compatibility below. In particular, an advertised
        // firmware floor must not silently upgrade simulated sensor physics.
        source_product_line = sensor_info->prod_line;
        source_product_part_number = sensor_info->prod_pn;
        const auto source_version = sensor_info->get_version();
        source_firmware_version.major = source_version.major;
        source_firmware_version.minor = source_version.minor;
        source_firmware_version.patch = source_version.patch;
        source_firmware_version.version_string =
            source_version.simple_version_string();
        active_lidar_udp_profile = ouster::sdk::core::to_string(
            sensor_info->format.udp_profile_lidar);
        source_low_data_profile =
            isLowDataProfile(sensor_info->format.udp_profile_lidar);

        if (height <= 0 || height > kMaximumChannels) {
            throw std::invalid_argument(
                "Ouster metadata pixels_per_column must be between 1 and 256");
        }
        if (width == 0 || width > kMaximumColumns) {
            throw std::invalid_argument(
                "Ouster metadata columns_per_frame must be between 1 and 4096");
        }
        if (columns_per_packet <= 0 ||
            static_cast<std::uint32_t>(columns_per_packet) > width ||
            width % static_cast<std::uint32_t>(columns_per_packet) != 0) {
            throw std::invalid_argument(
                "columns_per_packet must be nonzero and divide "
                "columns_per_frame exactly");
        }
        if (packet_writer.lidar_packet_size == 0) {
            throw std::invalid_argument(
                "Ouster metadata selects a lidar profile with no packets");
        }
        if (packet_writer.max_frame_id >
            std::numeric_limits<std::uint32_t>::max()) {
            throw std::invalid_argument(
                "Ouster packet frame ID cannot be represented in uint32");
        }
        if (packet_writer.field_type(ouster::sdk::core::ChanField::RANGE) ==
            ouster::sdk::core::ChanFieldType::VOID) {
            throw std::invalid_argument(
                "Ouster lidar packet profile does not contain RANGE");
        }

        active_return_count =
            packet_writer.field_type(ouster::sdk::core::ChanField::RANGE2) ==
                    ouster::sdk::core::ChanFieldType::VOID
                ? 1u
                : 2u;
        const std::uint64_t range_mask = packet_writer.field_value_mask(
            ouster::sdk::core::ChanField::RANGE);
        if (range_mask == 0 ||
            range_mask > std::numeric_limits<std::uint32_t>::max()) {
            throw std::invalid_argument(
                "Ouster lidar RANGE mask cannot be represented in uint32");
        }
        encodable_range_mask_mm = static_cast<std::uint32_t>(range_mask);

        beam_altitude_deg = sensor_info->beam_altitude_angles;
        beam_azimuth_deg = sensor_info->beam_azimuth_angles;
        if (beam_altitude_deg.size() != static_cast<std::size_t>(height)) {
            throw std::invalid_argument(
                "beam_altitude_angles size does not match pixels_per_column");
        }
        if (beam_azimuth_deg.empty()) {
            beam_azimuth_deg.resize(static_cast<std::size_t>(height), 0.0);
        } else if (beam_azimuth_deg.size() != static_cast<std::size_t>(height)) {
            throw std::invalid_argument(
                "beam_azimuth_angles size does not match pixels_per_column");
        }
        validateFiniteAngles(
            beam_altitude_deg, "beam_altitude_angles", true);
        validateFiniteAngles(
            beam_azimuth_deg, "beam_azimuth_angles", false);

        const double beam_origin_mm =
            sensor_info->lidar_origin_to_beam_origin_mm;
        if (!std::isfinite(beam_origin_mm) || beam_origin_mm < 0.0) {
            throw std::invalid_argument(
                "lidar_origin_to_beam_origin_mm must be finite and non-negative");
        }
        beam_origin_m = beam_origin_mm * 1.0e-3;

        // ouster_ros 0.16 removes WINDOW from scans advertised as firmware
        // older than 3.2, even when the selected modern packet layout carries
        // it. Publish a layout-consistent firmware advertisement while
        // retaining source_json verbatim for auditability.
        const ouster::sdk::core::Version kWindowMinimum{3, 2, 0};
        const auto modern_fields = ouster::sdk::core::get_field_types(
            sensor_info->format, kWindowMinimum);
        const bool profile_has_window = std::any_of(
            modern_fields.begin(), modern_fields.end(),
            [](const ouster::sdk::core::FieldType & field) {
                return field.name == ouster::sdk::core::ChanField::WINDOW;
            });
        if (profile_has_window && sensor_info->get_version() < kWindowMinimum) {
            sensor_info->image_rev = "ousteros-image-prod-aries-v3.2.0";
            sensor_info->fw_rev = "v3.2.0";
            firmware_adjusted = true;
        }
        published_json = firmware_adjusted
            ? sensor_info->to_json_string()
            : source_json;
    }

    std::string source_json;
    std::string published_json;
    std::string source_product_line;
    std::string source_product_part_number;
    std::string active_lidar_udp_profile;
    OusterFirmwareVersion source_firmware_version;
    std::shared_ptr<ouster::sdk::core::SensorInfo> sensor_info;
    ouster::sdk::core::PacketFormat packet_format;
    ouster::sdk::core::impl::PacketWriter packet_writer;
    std::vector<double> beam_altitude_deg;
    std::vector<double> beam_azimuth_deg;
    double beam_origin_m = 0.0;
    std::uint32_t encodable_range_mask_mm = 0;
    std::uint8_t active_return_count = 0;
    bool source_low_data_profile = false;
    bool firmware_adjusted = false;
};

OusterMetadata::OusterMetadata(std::shared_ptr<Impl> impl)
    : impl_(std::move(impl))
{
}

OusterMetadata OusterMetadata::fromFile(
    const std::filesystem::path & path,
    std::uintmax_t maximum_file_bytes)
{
    return fromJson(readMetadataFile(path, maximum_file_bytes));
}

OusterMetadata OusterMetadata::fromJson(std::string json)
{
    if (json.empty()) {
        throw std::invalid_argument("Ouster metadata JSON must not be empty");
    }
    return OusterMetadata(std::make_shared<Impl>(std::move(json)));
}

std::uint32_t OusterMetadata::columnsPerFrame() const noexcept
{
    return impl_->sensor_info->format.columns_per_frame;
}

std::uint16_t OusterMetadata::pixelsPerColumn() const noexcept
{
    return static_cast<std::uint16_t>(impl_->packet_writer.pixels_per_column);
}

std::uint16_t OusterMetadata::columnsPerPacket() const noexcept
{
    return static_cast<std::uint16_t>(impl_->packet_writer.columns_per_packet);
}

std::size_t OusterMetadata::lidarPacketSize() const noexcept
{
    return impl_->packet_writer.lidar_packet_size;
}

std::uint64_t OusterMetadata::sensorSerial() const noexcept
{
    return impl_->sensor_info->sn;
}

std::uint32_t OusterMetadata::initializationId() const noexcept
{
    return impl_->sensor_info->init_id;
}

std::uint32_t OusterMetadata::packetFrameId(
    std::uint64_t revolution) const noexcept
{
    const std::uint64_t modulus =
        static_cast<std::uint64_t>(impl_->packet_writer.max_frame_id) +
        std::uint64_t{1};
    return static_cast<std::uint32_t>(revolution % modulus);
}

const std::string & OusterMetadata::activeLidarUdpProfile() const noexcept
{
    return impl_->active_lidar_udp_profile;
}

std::uint8_t OusterMetadata::activeReturnCount() const noexcept
{
    return impl_->active_return_count;
}

std::uint32_t OusterMetadata::encodableRangeMaskMm() const noexcept
{
    return impl_->encodable_range_mask_mm;
}

std::uint32_t OusterMetadata::maximumEncodableRangeMm() const noexcept
{
    return impl_->encodable_range_mask_mm;
}

bool OusterMetadata::isRangeEncodable(std::uint32_t range_mm) const noexcept
{
    return (range_mm & ~impl_->encodable_range_mask_mm) == 0;
}

void OusterMetadata::requirePrimaryReturnProfile() const
{
    if (activeReturnCount() != 1u) {
        throw std::invalid_argument(
            "Ouster UDP lidar profile '" + activeLidarUdpProfile() +
            "' carries " + std::to_string(activeReturnCount()) +
            " returns; the current simulation contract supports exactly one");
    }
}

const std::string & OusterMetadata::productLine() const noexcept
{
    return impl_->source_product_line;
}

const std::string & OusterMetadata::sourceProductPartNumber() const noexcept
{
    return impl_->source_product_part_number;
}

const OusterFirmwareVersion &
OusterMetadata::sourceFirmwareVersion() const noexcept
{
    return impl_->source_firmware_version;
}

const std::string & OusterMetadata::sourceJson() const noexcept
{
    return impl_->source_json;
}

const std::string & OusterMetadata::publishedJson() const noexcept
{
    return impl_->published_json;
}

bool OusterMetadata::firmwareAdvertisementAdjusted() const noexcept
{
    return impl_->firmware_adjusted;
}

const std::vector<double> & OusterMetadata::beamAltitudeDeg() const noexcept
{
    return impl_->beam_altitude_deg;
}

const std::vector<double> & OusterMetadata::beamAzimuthDeg() const noexcept
{
    return impl_->beam_azimuth_deg;
}

double OusterMetadata::beamOriginM() const noexcept
{
    return impl_->beam_origin_m;
}

OusterFiringTableConfig OusterMetadata::firingTableConfig(
    double lidar_hz) const
{
    OusterFiringTableConfig config;
    config.columns_per_frame = columnsPerFrame();
    config.lidar_hz = lidar_hz;
    config.beam_altitude_deg = beamAltitudeDeg();
    config.beam_azimuth_deg = beamAzimuthDeg();
    config.lidar_origin_to_beam_origin_m = beamOriginM();
    return config;
}

OusterProductProfileRequest OusterMetadata::productProfileRequest(
    std::optional<std::string> hardware_revision,
    std::optional<bool> low_data_profile) const
{
    OusterProductProfileRequest request;
    request.product_line = productLine();
    request.product_part_number = sourceProductPartNumber();
    request.hardware_revision = hardware_revision.value_or(std::string{"auto"});
    request.firmware_major = sourceFirmwareVersion().major;
    request.firmware_minor = sourceFirmwareVersion().minor;
    request.beam_count = pixelsPerColumn();
    request.low_data_profile =
        low_data_profile.value_or(impl_->source_low_data_profile);
    return request;
}

OusterProductProfile OusterMetadata::resolvedProductProfile(
    std::optional<std::string> hardware_revision,
    std::optional<bool> low_data_profile) const
{
    return resolveOusterProductProfile(
        productProfileRequest(std::move(hardware_revision), low_data_profile));
}

const ouster::sdk::core::impl::PacketWriter &
OusterMetadata::packetWriter() const noexcept
{
    return impl_->packet_writer;
}

}  // namespace ouster_sim_core
