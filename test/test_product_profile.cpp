// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>

#include "ouster_sim_core/product_profile.hpp"

namespace ouster_sim_core {
namespace {

OusterProductProfile profile(
    const char * product, const char * revision,
    int firmware_major = 3, int firmware_minor = 1,
    int beams = 128, bool low_data = false)
{
    OusterProductProfileRequest request;
    request.product_line = product;
    request.hardware_revision = revision;
    request.firmware_major = firmware_major;
    request.firmware_minor = firmware_minor;
    request.beam_count = beams;
    request.low_data_profile = low_data;
    return resolveOusterProductProfile(request);
}

TEST(OusterProductProfile, CoversPublishedProductGenerations)
{
    const auto gen1 = profile("OS1-64", "gen1", 2, 2, 64);
    EXPECT_EQ(gen1.generation, OusterGeneration::Gen1);
    EXPECT_DOUBLE_EQ(gen1.detection_range_10_d90_m, 50.0);
    EXPECT_DOUBLE_EQ(gen1.detection_range_80_d90_m, 110.0);
    EXPECT_DOUBLE_EQ(gen1.minimum_range_m, 0.8);
    EXPECT_EQ(gen1.max_returns, 1);

    for (const char * revision : {"revC", "revD", "rev05"}) {
        const auto os1 = profile("OS1-64", revision, 2, 5, 64);
        EXPECT_EQ(os1.generation, OusterGeneration::Gen2);
        EXPECT_DOUBLE_EQ(os1.detection_range_10_d90_m, 45.0);
        EXPECT_DOUBLE_EQ(os1.detection_range_80_d90_m, 100.0);
        EXPECT_DOUBLE_EQ(os1.representable_range_m, 270.0);
        EXPECT_EQ(os1.max_returns, 1);
    }

    for (const char * revision : {"rev06", "rev06.2"}) {
        const auto os1 = profile("OS1-128", revision, 2, 5);
        EXPECT_EQ(os1.generation, OusterGeneration::Gen2);
        EXPECT_EQ(os1.max_returns, 2);
    }

    EXPECT_DOUBLE_EQ(
        profile("OS2-128", "rev06", 2, 4).representable_range_m,
        465.0);

    const auto rev7 = profile("OS1-128", "rev07");
    EXPECT_EQ(rev7.generation, OusterGeneration::Gen3);
    EXPECT_DOUBLE_EQ(rev7.detection_range_10_d90_m, 90.0);
    EXPECT_DOUBLE_EQ(rev7.detection_range_80_d90_m, 170.0);

    const auto rev8 = profile("OS1MAX-256", "rev08", 4, 0, 256);
    EXPECT_EQ(rev8.generation, OusterGeneration::Gen4);
    EXPECT_EQ(rev8.model, OusterModel::OS1Max);
    EXPECT_DOUBLE_EQ(rev8.detection_range_10_d90_m, 200.0);
    EXPECT_DOUBLE_EQ(rev8.detection_range_80_d90_m, 350.0);
    EXPECT_DOUBLE_EQ(rev8.representable_range_m, 500.0);
}

TEST(OusterProductProfile, CoversEveryRev8Model)
{
    const auto os0 = profile("OS0-128", "rev08", 4, 0);
    const auto os1 = profile("OS1-128", "rev08", 4, 0);
    const auto dome = profile("OSDome-128", "rev08", 4, 0);
    const auto max = profile("OS1MAX-256", "rev08", 4, 0, 256);

    for (const auto * value : {&os0, &os1, &dome, &max}) {
        EXPECT_TRUE(value->supported);
        EXPECT_EQ(value->generation, OusterGeneration::Gen4);
        EXPECT_DOUBLE_EQ(value->representable_range_m, 500.0);
    }
    EXPECT_DOUBLE_EQ(os0.detection_range_10_d90_m, 35.0);
    EXPECT_DOUBLE_EQ(os1.detection_range_10_d90_m, 90.0);
    EXPECT_DOUBLE_EQ(dome.detection_range_10_d90_m, 20.0);
    EXPECT_DOUBLE_EQ(dome.precision_min_std_m, 0.005);
    EXPECT_DOUBLE_EQ(dome.precision_max_std_m, 0.050);
    EXPECT_DOUBLE_EQ(max.detection_range_10_d90_m, 200.0);
}

TEST(OusterProductProfile, CoversEveryRev7Model)
{
    const auto os0 = profile("OS0-128", "rev07");
    const auto os1 = profile("OS1-64", "rev07", 3, 1, 64);
    const auto os2 = profile("OS2-128", "rev07", 2, 5);
    const auto dome = profile("OSDome-128", "rev07");

    EXPECT_DOUBLE_EQ(os0.detection_range_10_d90_m, 35.0);
    EXPECT_DOUBLE_EQ(os0.detection_range_80_d90_m, 75.0);
    EXPECT_DOUBLE_EQ(os1.detection_range_10_d90_m, 90.0);
    EXPECT_DOUBLE_EQ(os1.detection_range_80_d90_m, 170.0);
    EXPECT_DOUBLE_EQ(os2.detection_range_10_d90_m, 200.0);
    EXPECT_DOUBLE_EQ(os2.detection_range_80_d90_m, 350.0);
    EXPECT_DOUBLE_EQ(os2.minimum_range_m, 0.8);
    EXPECT_DOUBLE_EQ(dome.detection_range_10_d90_m, 20.0);
    EXPECT_DOUBLE_EQ(dome.detection_range_80_d90_m, 45.0);
    EXPECT_DOUBLE_EQ(dome.representable_range_m, 233.0);
    EXPECT_TRUE(os0.supported);
    EXPECT_TRUE(os1.supported);
    EXPECT_TRUE(os2.supported);
    EXPECT_TRUE(dome.supported);
}

TEST(OusterProductProfile, RejectsImpossibleProductRevisionPairs)
{
    EXPECT_FALSE(profile("OS2-128", "rev08", 4, 0).supported);
    EXPECT_FALSE(profile("OSDome-128", "rev06", 2, 5).supported);
    EXPECT_FALSE(profile("OS0-128", "gen1", 1, 13).supported);
    EXPECT_FALSE(profile("OS2-128", "rev07.1", 3, 1).supported);
}

TEST(OusterProductProfile, InfersRevisionFromPartNumbersAndFirmware)
{
    OusterProductProfileRequest old_part_number;
    old_part_number.product_line = "OS1-128";
    old_part_number.product_part_number = "860-105010-07";
    const auto rev7 = resolveOusterProductProfile(old_part_number);
    EXPECT_EQ(rev7.revision, OusterRevision::Rev07);
    EXPECT_TRUE(rev7.revision_inferred);
    EXPECT_FALSE(rev7.fallback_revision);

    auto revc_part_number = old_part_number;
    revc_part_number.product_part_number = "840105010C";
    const auto revc = resolveOusterProductProfile(revc_part_number);
    EXPECT_EQ(revc.revision, OusterRevision::RevC);
    EXPECT_TRUE(revc.revision_inferred);

    auto rev8_part_number = old_part_number;
    rev8_part_number.product_line = "OS1MAX-256";
    rev8_part_number.product_part_number = "OS1MAX-080-256-U-002-XX";
    const auto rev8 = resolveOusterProductProfile(rev8_part_number);
    EXPECT_EQ(rev8.revision, OusterRevision::Rev08);
    EXPECT_TRUE(rev8.revision_inferred);

    auto firmware_four = old_part_number;
    firmware_four.product_line = "OSDome-128";
    firmware_four.product_part_number = "synthetic";
    firmware_four.firmware_major = 4;
    const auto rev8_firmware =
        resolveOusterProductProfile(firmware_four);
    EXPECT_EQ(rev8_firmware.revision, OusterRevision::Rev08);
    EXPECT_TRUE(rev8_firmware.revision_inferred);
}

TEST(OusterProductProfile, MarksAmbiguousAndInvalidRevisionsAsFallback)
{
    OusterProductProfileRequest request;
    request.product_line = "OS2-128";
    request.product_part_number = "860-os2128";
    request.firmware_major = 2;
    request.firmware_minor = 5;
    const auto ambiguous = resolveOusterProductProfile(request);
    EXPECT_EQ(ambiguous.revision, OusterRevision::Rev06);
    EXPECT_TRUE(ambiguous.fallback_revision);

    request.hardware_revision = "definitely-not-a-revision";
    const auto invalid = resolveOusterProductProfile(request);
    EXPECT_EQ(invalid.revision, OusterRevision::Rev06);
    EXPECT_TRUE(invalid.fallback_revision);
}

TEST(OusterProductProfile, FirmwareAndPacketProfileAffectResolution)
{
    EXPECT_DOUBLE_EQ(
        profile("OS1-64", "rev05", 2, 2, 64).range_resolution_m,
        0.003);
    EXPECT_DOUBLE_EQ(
        profile("OS1-64", "rev05", 2, 5, 64).range_resolution_m,
        0.001);
    const auto rev7_low_data =
        profile("OS1-64", "rev07", 3, 1, 64, true);
    EXPECT_DOUBLE_EQ(rev7_low_data.range_resolution_m, 0.008);
    EXPECT_DOUBLE_EQ(rev7_low_data.representable_range_m, 233.0);

    // The physical range of newer hardware can exceed RNG15's packet
    // capacity. Packet selection must cap the reported channel range so the
    // encoder can never wrap it through the 15-bit wire field.
    const auto rev8_low_data =
        profile("OS1MAX-256", "rev08", 4, 0, 256, true);
    EXPECT_DOUBLE_EQ(rev8_low_data.range_resolution_m, 0.008);
    EXPECT_DOUBLE_EQ(
        rev8_low_data.representable_range_m,
        kRng15MaximumRepresentableRangeM);
}

TEST(OusterProductProfile, ModeScalesUseDocumentedGatheringRate)
{
    const auto value = profile("OS1-64", "rev07", 3, 1, 64);
    EXPECT_NEAR(ousterModeRangeScale(value, 1024, 10.0), 1.0, 1e-12);
    EXPECT_NEAR(ousterModeRangeScale(value, 512, 10.0), 1.19, 1e-12);
    EXPECT_NEAR(ousterModeRangeScale(value, 1024, 20.0),
                1.0 / 1.19, 1e-12);
    EXPECT_NEAR(ousterModeRangeScale(value, 1024, 40.0),
                1.0 / (1.19 * 1.19), 1e-12);

    EXPECT_NEAR(ousterModePrecisionScale(value, 1024, 10.0),
                1.0, 1e-12);
    EXPECT_NEAR(ousterModePrecisionScale(value, 512, 10.0),
                1.0 / std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(ousterModePrecisionScale(value, 1024, 20.0),
                std::sqrt(2.0), 1e-12);

    const auto os2 = profile("OS2-128", "rev07", 2, 5);
    EXPECT_NEAR(ousterModeRangeScale(os2, 2048, 10.0), 1.0, 1e-12);
}

TEST(OusterProductProfile, ModeScalesAreClampedAndValidateInputs)
{
    const auto value = profile("OS1-64", "rev07", 3, 1, 64);
    EXPECT_NEAR(ousterModeRangeScale(value, 1, 1.0),
                1.19 * 1.19, 1e-12);
    EXPECT_NEAR(ousterModePrecisionScale(value, 4096, 40.0),
                2.0, 1e-12);
    EXPECT_DOUBLE_EQ(ousterModeRangeScale(value, 0, 10.0), 1.0);
    EXPECT_DOUBLE_EQ(ousterModeRangeScale(value, 1024, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(ousterModePrecisionScale(value, -1, 10.0), 1.0);
}

}  // namespace
}  // namespace ouster_sim_core
