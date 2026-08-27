// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/metadata.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#ifndef OUSTER_SIM_CORE_TEST_DATA_DIR
#error "OUSTER_SIM_CORE_TEST_DATA_DIR must identify the core metadata fixtures"
#endif

namespace ouster_sim_core {
namespace {

std::filesystem::path metadataPath()
{
    return std::filesystem::path{OUSTER_SIM_CORE_TEST_DATA_DIR} /
           "os1_64_rev7.json";
}

std::string readFixture()
{
    std::ifstream stream(metadataPath(), std::ios::binary);
    return {std::istreambuf_iterator<char>(stream),
            std::istreambuf_iterator<char>()};
}

void replaceAll(std::string & value, const std::string & from,
                const std::string & to)
{
    std::size_t position = 0;
    while ((position = value.find(from, position)) != std::string::npos) {
        value.replace(position, from.size(), to);
        position += to.size();
    }
}

TEST(OusterMetadataProductProfile, PreservesSourceProductAndFirmwareFields)
{
    const auto metadata = OusterMetadata::fromFile(metadataPath());

    EXPECT_EQ(metadata.productLine(), "OS1-64");
    EXPECT_EQ(metadata.sourceProductPartNumber(), "860-os164");
    EXPECT_EQ(metadata.sourceFirmwareVersion().major, 3u);
    EXPECT_EQ(metadata.sourceFirmwareVersion().minor, 2u);
    EXPECT_EQ(metadata.sourceFirmwareVersion().patch, 0u);
    EXPECT_EQ(metadata.sourceFirmwareVersion().version_string, "3.2.0");

    const auto request = metadata.productProfileRequest();
    EXPECT_EQ(request.product_line, "OS1-64");
    EXPECT_EQ(request.product_part_number, "860-os164");
    EXPECT_EQ(request.hardware_revision, "auto");
    EXPECT_EQ(request.firmware_major, 3);
    EXPECT_EQ(request.firmware_minor, 2);
    EXPECT_EQ(request.beam_count, 64);
    EXPECT_FALSE(request.low_data_profile);

    const auto profile = metadata.resolvedProductProfile();
    EXPECT_EQ(profile.model, OusterModel::OS1);
    EXPECT_EQ(profile.revision, OusterRevision::Rev07);
    EXPECT_TRUE(profile.revision_inferred);
    EXPECT_FALSE(profile.fallback_revision);
    EXPECT_TRUE(profile.supported);
}

TEST(OusterMetadataProductProfile, AcceptsExplicitPhysicsOverrides)
{
    const auto metadata = OusterMetadata::fromFile(metadataPath());
    const auto request = metadata.productProfileRequest("rev06", true);

    EXPECT_EQ(request.hardware_revision, "rev06");
    EXPECT_TRUE(request.low_data_profile);

    const auto profile = metadata.resolvedProductProfile("rev06", true);
    EXPECT_EQ(profile.revision, OusterRevision::Rev06);
    EXPECT_FALSE(profile.revision_inferred);
    EXPECT_FALSE(profile.fallback_revision);
    EXPECT_TRUE(profile.supported);
    EXPECT_DOUBLE_EQ(profile.range_resolution_m, 0.008);
}

TEST(OusterMetadataProductProfile, ResolvesPhysicsFromSourceFirmware)
{
    auto json = readFixture();
    ASSERT_FALSE(json.empty());
    replaceAll(json, "v3.2.0", "v2.3.0");

    const auto metadata = OusterMetadata::fromJson(std::move(json));
    ASSERT_TRUE(metadata.firmwareAdvertisementAdjusted());
    EXPECT_EQ(metadata.sourceFirmwareVersion().version_string, "2.3.0");
    EXPECT_NE(metadata.publishedJson(), metadata.sourceJson());

    const auto request = metadata.productProfileRequest();
    EXPECT_EQ(request.firmware_major, 2);
    EXPECT_EQ(request.firmware_minor, 3);

    // The publication copy advertises 3.2 for WINDOW compatibility, but the
    // physical profile remains the conservative revision implied by source
    // firmware 2.3 and the otherwise ambiguous fixture part number.
    const auto profile = metadata.resolvedProductProfile();
    EXPECT_EQ(profile.revision, OusterRevision::Rev06);
    EXPECT_TRUE(profile.fallback_revision);
}

TEST(OusterMetadataProductProfile, ExposesActiveWireContractWithoutSdkTypes)
{
    const auto standard = OusterMetadata::fromFile(metadataPath());
    EXPECT_EQ(
        standard.activeLidarUdpProfile(),
        "RNG19_RFL8_SIG16_NIR16");
    EXPECT_EQ(standard.activeReturnCount(), 1u);
    EXPECT_EQ(standard.encodableRangeMaskMm(), 524287u);
    EXPECT_EQ(standard.maximumEncodableRangeMm(), 524287u);
    EXPECT_TRUE(standard.isRangeEncodable(524287u));
    EXPECT_FALSE(standard.isRangeEncodable(524288u));
    EXPECT_NO_THROW(standard.requirePrimaryReturnProfile());

    auto low_data_json = readFixture();
    replaceAll(
        low_data_json,
        "RNG19_RFL8_SIG16_NIR16",
        "RNG15_RFL8_NIR8");
    const auto low_data = OusterMetadata::fromJson(std::move(low_data_json));
    EXPECT_EQ(low_data.activeLidarUdpProfile(), "RNG15_RFL8_NIR8");
    EXPECT_EQ(low_data.activeReturnCount(), 1u);
    EXPECT_EQ(low_data.encodableRangeMaskMm(), 262136u);
    EXPECT_EQ(low_data.maximumEncodableRangeMm(), 262136u);
    EXPECT_TRUE(low_data.isRangeEncodable(262136u));
    EXPECT_FALSE(low_data.isRangeEncodable(262135u));
    EXPECT_FALSE(low_data.isRangeEncodable(262144u));
    EXPECT_TRUE(low_data.productProfileRequest().low_data_profile);

    auto dual_json = readFixture();
    replaceAll(
        dual_json,
        "RNG19_RFL8_SIG16_NIR16",
        "RNG19_RFL8_SIG16_NIR16_DUAL");
    const auto dual = OusterMetadata::fromJson(std::move(dual_json));
    EXPECT_EQ(
        dual.activeLidarUdpProfile(),
        "RNG19_RFL8_SIG16_NIR16_DUAL");
    EXPECT_EQ(dual.activeReturnCount(), 2u);
    EXPECT_THROW(dual.requirePrimaryReturnProfile(), std::invalid_argument);
}

TEST(OusterMetadataProductProfile, DerivesPacketFrameIdAtLayoutWidth)
{
    const auto standard = OusterMetadata::fromFile(metadataPath());
    EXPECT_EQ(standard.packetFrameId(0), 0u);
    EXPECT_EQ(standard.packetFrameId(65'535), 65'535u);
    EXPECT_EQ(standard.packetFrameId(65'536), 0u);
    EXPECT_EQ(standard.packetFrameId(65'537), 1u);

    auto fusa_json = readFixture();
    replaceAll(
        fusa_json,
        "RNG19_RFL8_SIG16_NIR16",
        "FUSA_RNG15_RFL8_NIR8_DUAL");
    const auto fusa = OusterMetadata::fromJson(std::move(fusa_json));
    const auto maximum = std::numeric_limits<std::uint32_t>::max();
    EXPECT_EQ(fusa.packetFrameId(65'536), 65'536u);
    EXPECT_EQ(fusa.packetFrameId(maximum), maximum);
    EXPECT_EQ(
        fusa.packetFrameId(static_cast<std::uint64_t>(maximum) + 1u),
        0u);
}

}  // namespace
}  // namespace ouster_sim_core
