// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/optical_channel_model.hpp"
#include "ouster_sim_core/product_profile.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using ouster_sim_core::NormalizedOpticalReturn;
using ouster_sim_core::OpticalChannelContext;
using ouster_sim_core::OpticalChannelModel;
using ouster_sim_core::OpticalChannelModelConfig;
using ouster_sim_core::OpticalDetectionParameters;
using ouster_sim_core::OpticalRandomEffect;
using ouster_sim_core::OpticalRandomKey;
using ouster_sim_core::OpticalReturnKind;
using ouster_sim_core::QuantizedOpticalReturn;

NormalizedOpticalReturn surface(
    std::uint32_t measurement_id,
    std::uint16_t ring_id,
    double range_m)
{
    NormalizedOpticalReturn input;
    input.identity.revolution = 7;
    input.identity.measurement_id = measurement_id;
    input.identity.ring_id = ring_id;
    input.identity.linear_index = measurement_id * 64u + ring_id;
    input.identity.return_index = 0;
    input.return_kind = OpticalReturnKind::kSurface;
    input.geometric_hit = true;
    input.path_length_m = range_m;
    input.reported_range_m = range_m;
    return input;
}

void expectSameChannels(
    const QuantizedOpticalReturn & actual,
    const QuantizedOpticalReturn & expected)
{
    EXPECT_EQ(actual.identity.revolution, expected.identity.revolution);
    EXPECT_EQ(actual.identity.measurement_id, expected.identity.measurement_id);
    EXPECT_EQ(actual.identity.ring_id, expected.identity.ring_id);
    EXPECT_EQ(actual.return_kind, expected.return_kind);
    EXPECT_EQ(actual.geometric_hit, expected.geometric_hit);
    EXPECT_EQ(actual.range_mm, expected.range_mm);
    EXPECT_EQ(actual.signal, expected.signal);
    EXPECT_EQ(actual.reflectivity, expected.reflectivity);
    EXPECT_EQ(actual.near_ir, expected.near_ir);
}

TEST(OpticalChannelModel, DocumentedNoiselessGoldenVectors)
{
    OpticalChannelModelConfig config;
    config.base_signal = 800.0;
    const OpticalChannelModel model(config);
    const OpticalChannelContext context{};

    const auto miss_output = model.process(NormalizedOpticalReturn{}, context);
    EXPECT_EQ(miss_output.range_mm, 0u);
    EXPECT_EQ(miss_output.signal, 0u);
    EXPECT_EQ(miss_output.reflectivity, 0u);
    EXPECT_EQ(miss_output.near_ir, 0u);

    auto diffuse = surface(0, 0, 10.0);
    diffuse.apparent_reflectance = 0.5;
    diffuse.ambient_near_ir_factor = 0.25;
    const auto diffuse_output = model.process(diffuse, context);
    // range=10m; signal=800*.5/10^2; reflectivity=.5*100; NIR=.25*256.
    EXPECT_EQ(diffuse_output.range_mm, 10'000u);
    EXPECT_EQ(diffuse_output.signal, 4u);
    EXPECT_EQ(diffuse_output.reflectivity, 50u);
    EXPECT_EQ(diffuse_output.near_ir, 64u);

    auto unit = surface(1, 0, 2.0);
    unit.apparent_reflectance = 1.0;
    unit.ambient_near_ir_factor = 0.0;
    const auto unit_output = model.process(unit, context);
    EXPECT_EQ(unit_output.range_mm, 2'000u);
    EXPECT_EQ(unit_output.signal, 200u);
    EXPECT_EQ(unit_output.reflectivity, 100u);
    EXPECT_EQ(unit_output.near_ir, 0u);

    auto retro = surface(2, 0, 4.0);
    retro.apparent_reflectance = 2.0;
    retro.ambient_near_ir_factor = 0.0;
    const auto retro_output = model.process(retro, context);
    // 100 + log2(2)*22 is the first retroreflective-band anchor.
    EXPECT_EQ(retro_output.range_mm, 4'000u);
    EXPECT_EQ(retro_output.signal, 100u);
    EXPECT_EQ(retro_output.reflectivity, 122u);
    EXPECT_EQ(retro_output.near_ir, 0u);

    auto saturated = surface(3, 0, 0.5);
    saturated.apparent_reflectance = 1024.0;
    saturated.ambient_near_ir_factor = 300.0;
    const auto saturated_output = model.process(saturated, context);
    EXPECT_EQ(saturated_output.range_mm, 500u);
    EXPECT_EQ(
        saturated_output.signal,
        std::numeric_limits<std::uint16_t>::max());
    EXPECT_EQ(
        saturated_output.reflectivity,
        std::numeric_limits<std::uint8_t>::max());
    EXPECT_EQ(
        saturated_output.near_ir,
        std::numeric_limits<std::uint16_t>::max());
}

TEST(OpticalChannelModel, BlackIsDifferentFromMissingReflectance)
{
    OpticalChannelModelConfig config;
    config.base_signal = 800.0;
    config.missing_reflectivity_byte = 50;
    const OpticalChannelModel model(config);

    auto black = surface(0, 0, 10.0);
    black.apparent_reflectance = 0.0;
    const auto black_output = model.process(black, {});
    ASSERT_TRUE(black_output.hasReturn());
    EXPECT_EQ(black_output.signal, 0u);
    EXPECT_EQ(black_output.reflectivity, 0u);
    EXPECT_EQ(black_output.near_ir, 0u);

    const auto missing_output = model.process(surface(0, 0, 10.0), {});
    ASSERT_TRUE(missing_output.hasReturn());
    EXPECT_EQ(missing_output.signal, 8u);
    EXPECT_EQ(missing_output.reflectivity, 50u);
    EXPECT_EQ(missing_output.near_ir, 0u);
}

TEST(OpticalChannelModel, PathLengthAndReportedRangeHaveDistinctRoles)
{
    OpticalChannelModelConfig config;
    config.base_signal = 1'000.0;
    config.range_resolution_m = 0.0;
    const OpticalChannelModel model(config);

    auto input = surface(0, 0, 5.0);
    input.path_length_m = 10.0;
    input.apparent_reflectance = 1.0;
    const auto output = model.process(input, {});

    EXPECT_EQ(output.range_mm, 5'000u);  // Reported Ouster path.
    EXPECT_EQ(output.signal, 10u);       // 1000 / physical 10m path squared.
}

TEST(OpticalChannelModel, RangeQuantizationIsExactInIntegerMillimeters)
{
    struct Case {
        double resolution_m;
        std::uint32_t expected_range_mm;
    };
    constexpr std::array<Case, 3> cases{{
        {0.001, 4'007u},
        {0.003, 4'008u},
        {0.008, 4'008u},
    }};

    // 4.007 cannot be represented exactly as binary64. Quantizing in metres
    // and then truncating metres*1000 historically produced 4006 for the 1 mm
    // profile. All supported wire resolutions are now quantized directly in
    // the integer-millimetre packet domain.
    for (const auto & test_case : cases) {
        OpticalChannelModelConfig config;
        config.range_resolution_m = test_case.resolution_m;
        const OpticalChannelModel model(config);
        EXPECT_EQ(
            model.process(surface(0, 0, 4.007), {}).range_mm,
            test_case.expected_range_mm)
            << "resolution_m=" << test_case.resolution_m;
    }
}

TEST(OpticalChannelModel, ExplicitZeroNearIrSuppressesLegacyFallback)
{
    const OpticalChannelModel model(OpticalChannelModelConfig{});
    auto input = surface(0, 0, 10.0);
    input.apparent_reflectance = 0.5;

    EXPECT_EQ(model.process(input, {}).near_ir, 128u);
    input.ambient_near_ir_factor = 0.0;
    EXPECT_EQ(model.process(input, {}).near_ir, 0u);
}

TEST(OpticalChannelModel, TrueMissIsAllZeroAndLosslesslyAssemblerReady)
{
    const OpticalChannelModel model(OpticalChannelModelConfig{});
    NormalizedOpticalReturn input;
    input.identity.revolution = 9;
    input.identity.measurement_id = 11;
    input.identity.ring_id = 4;
    input.apparent_reflectance = 3.0;
    input.ambient_near_ir_factor = 2.0;

    const auto output = model.process(input, {});
    EXPECT_FALSE(output.hasReturn());
    EXPECT_EQ(output.range_mm, 0u);
    EXPECT_EQ(output.signal, 0u);
    EXPECT_EQ(output.reflectivity, 0u);
    EXPECT_EQ(output.near_ir, 0u);

    const auto assembler_sample = output.assemblerSample();
    EXPECT_FALSE(assembler_sample.is_hit);
    EXPECT_EQ(assembler_sample.range_mm, 0u);
    EXPECT_EQ(assembler_sample.signal, 0u);
    EXPECT_EQ(assembler_sample.reflectivity, 0u);
    EXPECT_EQ(assembler_sample.near_ir, 0u);
}

TEST(OpticalChannelModel, EveryIntegerChannelSaturates)
{
    OpticalChannelModelConfig config;
    config.base_signal = 1.0e20;
    config.near_ir_scale = 1.0e20;
    config.minimum_range_m = 0.001;
    config.maximum_range_m = 10'000'000.0;
    config.range_resolution_m = 0.0;
    config.range_noise_reference_range_m = config.maximum_range_m;
    const OpticalChannelModel model(config);

    auto input = surface(0, 0, 5'000'000.0);
    input.apparent_reflectance = 1.0e20;
    input.ambient_near_ir_factor = 1.0e20;
    const auto output = model.process(input, {});
    EXPECT_EQ(output.range_mm, std::numeric_limits<std::uint32_t>::max());
    EXPECT_EQ(output.signal, std::numeric_limits<std::uint16_t>::max());
    EXPECT_EQ(output.reflectivity, std::numeric_limits<std::uint8_t>::max());
    EXPECT_EQ(output.near_ir, std::numeric_limits<std::uint16_t>::max());

    const auto assembler_sample = output.assemblerSample();
    EXPECT_EQ(assembler_sample.range_mm, output.range_mm);
    EXPECT_EQ(assembler_sample.signal, output.signal);
    EXPECT_EQ(assembler_sample.reflectivity, output.reflectivity);
    EXPECT_EQ(assembler_sample.near_ir, output.near_ir);
}

TEST(OpticalChannelModel, ExtremeFiniteInputsNeverCreateUndefinedCasts)
{
    OpticalChannelModelConfig config;
    config.base_signal = std::numeric_limits<double>::max();
    config.near_ir_scale = std::numeric_limits<double>::max();
    config.near_ir_noise_scale = 1.0;
    const OpticalChannelModel model(config);

    auto input = surface(0, 0, 1.0);
    input.path_length_m = std::numeric_limits<double>::max();
    input.apparent_reflectance = std::numeric_limits<double>::max();
    input.ambient_near_ir_factor = std::numeric_limits<double>::max();
    const auto output = model.process(input, {9, 8, 7});

    // The signal expression is mathematically one even though each naive
    // product and square overflows binary64. Passive NIR is mathematically
    // above its wire range and must saturate, including with noise enabled.
    EXPECT_EQ(output.signal, 1u);
    EXPECT_EQ(output.near_ir, std::numeric_limits<std::uint16_t>::max());
    EXPECT_EQ(output.reflectivity, std::numeric_limits<std::uint8_t>::max());
}

TEST(OpticalChannelModel, IncidentAngleIsValidatedButNotAppliedTwice)
{
    const OpticalChannelModel model(OpticalChannelModelConfig{});
    auto input = surface(0, 0, 10.0);
    input.apparent_reflectance = 0.5;
    const auto without_angle = model.process(input, {});

    input.incident_angle_rad = 0.7;
    expectSameChannels(model.process(input, {}), without_angle);

    input.incident_angle_rad = -0.01;
    EXPECT_THROW(model.process(input, {}), std::invalid_argument);
    input.incident_angle_rad = std::acos(-1.0);
    EXPECT_THROW(model.process(input, {}), std::invalid_argument);
    input.incident_angle_rad = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(model.process(input, {}), std::invalid_argument);
}

TEST(OpticalChannelModel, DetectionCalibrationPreservesPublishedAnchors)
{
    OpticalDetectionParameters parameters;
    parameters.range_10_d90_m = 20.0;
    parameters.range_80_d90_m = 80.0;
    parameters.range_10_d50_m = 30.0;
    parameters.range_80_d50_m = 100.0;

    EXPECT_DOUBLE_EQ(
        ouster_sim_core::calibratedDetectionRangeM(0.1, 20.0, 80.0),
        20.0);
    EXPECT_DOUBLE_EQ(
        ouster_sim_core::calibratedDetectionRangeM(0.8, 20.0, 80.0),
        80.0);
    EXPECT_NEAR(
        ouster_sim_core::calibratedDetectionProbability(
            20.0, 0.1, parameters, 120.0),
        0.9,
        1.0e-14);
    EXPECT_NEAR(
        ouster_sim_core::calibratedDetectionProbability(
            30.0, 0.1, parameters, 120.0),
        0.5,
        1.0e-14);
    EXPECT_EQ(
        ouster_sim_core::calibratedDetectionProbability(
            20.0, 0.0, parameters, 120.0),
        0.0);
}

TEST(OpticalChannelModel, ProductProfileFactoryAppliesActiveModeScaling)
{
    ouster_sim_core::OusterProductProfile profile;
    profile.supported = true;
    profile.minimum_range_m = 0.5;
    profile.representable_range_m = 233.0;
    profile.range_resolution_m = 0.001;
    profile.detection_range_10_d90_m = 90.0;
    profile.detection_range_80_d90_m = 170.0;
    profile.detection_range_10_d50_m = 100.0;
    profile.detection_range_80_d50_m = 190.0;
    profile.precision_min_std_m = 0.005;
    profile.precision_max_std_m = 0.030;
    profile.false_positive_rate = 1.0e-4;
    profile.reference_columns_per_second = 10'240.0;

    // 512x10 is one halving of the reference column rate: range x1.19,
    // precision x0.71 (sqrt(.5)).
    const auto config =
        ouster_sim_core::opticalChannelModelConfigFromProfile(
            profile, 512, 10.0);
    EXPECT_DOUBLE_EQ(config.minimum_range_m, 0.5);
    EXPECT_DOUBLE_EQ(config.maximum_range_m, 233.0);
    EXPECT_DOUBLE_EQ(config.range_resolution_m, 0.001);
    EXPECT_NEAR(config.detection.range_10_d90_m, 90.0 * 1.19, 1.0e-12);
    EXPECT_NEAR(config.detection.range_80_d90_m, 170.0 * 1.19, 1.0e-12);
    EXPECT_NEAR(
        config.range_noise_min_std_m,
        0.005 / std::sqrt(2.0),
        1.0e-15);
    EXPECT_NEAR(
        config.range_noise_max_std_m,
        0.030 / std::sqrt(2.0),
        1.0e-15);
    EXPECT_DOUBLE_EQ(config.false_alarm_rate, 1.0e-4);
}

TEST(OpticalChannelModel, CounterRngHasStableGoldenWordAndIndependentLanes)
{
    static_assert(ouster_sim_core::kOpticalChannelContractVersion == 1);
    OpticalRandomKey key;
    key.sensor_stream_id = 0x1020304050607080ULL;
    key.epoch = 3;
    key.revolution = 99;
    key.measurement_id = 511;
    key.ring_id = 63;
    key.return_index = 1;
    key.effect = OpticalRandomEffect::kRangeNoise;
    key.subdraw = 7;

    constexpr std::uint64_t seed = 0x8877665544332211ULL;
    const auto golden =
        ouster_sim_core::deterministicOpticalRandomBits(seed, key);
    EXPECT_EQ(golden, 0xb6b0d865be56080cULL);
    EXPECT_DOUBLE_EQ(
        ouster_sim_core::deterministicOpticalUniform01(seed, key),
        0.71363594516727613);

    std::set<std::uint64_t> lanes{
        golden,
        ouster_sim_core::deterministicOpticalRandomBits(seed + 1u, key)};
    auto insertChanged = [&](auto change) {
        auto changed = key;
        change(changed);
        lanes.insert(
            ouster_sim_core::deterministicOpticalRandomBits(seed, changed));
    };
    insertChanged([](auto & value) { ++value.sensor_stream_id; });
    insertChanged([](auto & value) { ++value.epoch; });
    insertChanged([](auto & value) { ++value.revolution; });
    insertChanged([](auto & value) { ++value.measurement_id; });
    insertChanged([](auto & value) { ++value.ring_id; });
    insertChanged([](auto & value) { ++value.return_index; });
    insertChanged([](auto & value) {
        value.effect = OpticalRandomEffect::kSignalNoise;
    });
    insertChanged([](auto & value) { ++value.subdraw; });
    EXPECT_EQ(lanes.size(), 10u);

    const double first =
        ouster_sim_core::deterministicOpticalStandardNormal(seed, key);
    const double again =
        ouster_sim_core::deterministicOpticalStandardNormal(seed, key);
    EXPECT_DOUBLE_EQ(first, again);
    // Box-Muller uses platform libm. This freezes the supported toolchain
    // result while allowing the documented cross-libm conformance tolerance.
    EXPECT_NEAR(first, 0.78183201991365625, 1.0e-15);
}

TEST(OpticalChannelModel, UniformDrawsAlwaysExcludeBothEndpoints)
{
    OpticalRandomKey key;
    key.effect = OpticalRandomEffect::kRangeNoise;
    for (std::uint64_t draw = 0; draw < 100'000u; ++draw) {
        key.subdraw = draw;
        const double value =
            ouster_sim_core::deterministicOpticalUniform01(42, key);
        ASSERT_GT(value, 0.0);
        ASSERT_LT(value, 1.0);
    }
}

TEST(OpticalChannelModel, NoiseIsInvariantToInputOrderAndBatchPartition)
{
    OpticalChannelModelConfig config;
    config.range_resolution_m = 0.001;
    config.range_noise_min_std_m = 0.01;
    config.range_noise_max_std_m = 0.05;
    config.signal_noise_scale = 1.0;
    config.near_ir_noise_scale = 1.0;
    const OpticalChannelModel model(config);
    const OpticalChannelContext context{12345, 77, 4};

    auto first = surface(10, 2, 20.0);
    first.apparent_reflectance = 0.2;
    first.ambient_near_ir_factor = 0.8;
    auto second = surface(10, 3, 40.0);
    second.apparent_reflectance = 0.9;
    second.ambient_near_ir_factor = 0.1;
    std::vector<NormalizedOpticalReturn> forward{first, second};
    std::vector<NormalizedOpticalReturn> reverse{second, first};

    const auto forward_output = model.process(forward, context);
    const auto reverse_output = model.process(reverse, context);
    ASSERT_EQ(forward_output.size(), 2u);
    ASSERT_EQ(reverse_output.size(), 2u);
    expectSameChannels(forward_output[0], reverse_output[1]);
    expectSameChannels(forward_output[1], reverse_output[0]);

    const auto separately_first = model.process(first, context);
    const auto separately_second = model.process(second, context);
    expectSameChannels(forward_output[0], separately_first);
    expectSameChannels(forward_output[1], separately_second);
}

TEST(OpticalChannelModel, VersionedNoisyChannelGoldenVector)
{
    static_assert(ouster_sim_core::kOpticalChannelContractVersion == 1);
    OpticalChannelModelConfig config;
    config.range_resolution_m = 0.001;
    config.range_noise_min_std_m = 0.01;
    config.range_noise_max_std_m = 0.05;
    config.signal_noise_scale = 1.0;
    config.near_ir_noise_scale = 1.0;
    const OpticalChannelModel model(config);

    auto input = surface(123, 17, 5.0);
    input.apparent_reflectance = 0.75;
    input.ambient_near_ir_factor = 0.5;
    const auto output = model.process(input, {12345, 77, 4});

    EXPECT_EQ(output.range_mm, 5'001u);
    EXPECT_EQ(output.signal, 23u);
    EXPECT_EQ(output.reflectivity, 75u);
    EXPECT_EQ(output.near_ir, 107u);
}

TEST(OpticalChannelModel, FalseAlarmBecomesAReportableSyntheticReturn)
{
    OpticalChannelModelConfig config;
    config.false_alarm_rate = 1.0;
    config.missing_reflectivity_byte = 37;
    const OpticalChannelModel model(config);
    NormalizedOpticalReturn miss;
    miss.identity.revolution = 2;
    miss.identity.measurement_id = 8;
    miss.identity.ring_id = 5;

    const auto output = model.process(miss, {17, 9, 1});
    EXPECT_EQ(output.return_kind, OpticalReturnKind::kFalseAlarm);
    EXPECT_FALSE(output.geometric_hit);
    EXPECT_GT(output.range_mm, 0u);
    EXPECT_EQ(output.signal, 1u);
    EXPECT_EQ(output.reflectivity, 37u);
    EXPECT_EQ(output.near_ir, 0u);
    EXPECT_TRUE(output.assemblerSample().is_hit);
}

TEST(OpticalChannelModel, RejectsSecondaryReturnsAndInvertedNoiseEnvelope)
{
    OpticalChannelModelConfig invalid_config;
    invalid_config.range_noise_min_std_m = 0.02;
    invalid_config.range_noise_max_std_m = 0.01;
    EXPECT_THROW(
        static_cast<void>(OpticalChannelModel{invalid_config}),
        std::invalid_argument);

    const OpticalChannelModel model(OpticalChannelModelConfig{});
    auto secondary = surface(0, 0, 10.0);
    secondary.identity.return_index = 1;
    EXPECT_THROW(model.process(secondary, {}), std::invalid_argument);
}

}  // namespace
