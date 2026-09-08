// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/metadata.hpp"
#include "ouster_sim_core/optical_channel_model.hpp"
#include "ouster_sim_core/packet_encoder.hpp"

#include <array>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace ouster_sim_core::conformance_v1 {

inline constexpr std::array<const char *, 3> primaryProfiles{
    "LEGACY", "RNG19_RFL8_SIG16_NIR16", "RNG15_RFL8_NIR8"};
inline constexpr std::array<const char *, 3> secondaryProfiles{
    "RNG19_RFL8_SIG16_NIR16_DUAL", "RNG15_RFL8_NIR8_DUAL",
    "FUSA_RNG15_RFL8_NIR8_DUAL"};

inline std::string metadataJson(const std::string & fixture_path,
                                const std::string & profile)
{
    std::ifstream stream(fixture_path);
    if (!stream) throw std::runtime_error("missing conformance metadata fixture");
    std::string json{std::istreambuf_iterator<char>(stream), {}};
    const auto replace = [&](const std::string & before, const std::string & after) {
        std::size_t pos = 0;
        while ((pos = json.find(before, pos)) != std::string::npos) {
            json.replace(pos, before.size(), after);
            pos += after.size();
        }
    };
    replace("RNG19_RFL8_SIG16_NIR16", profile);
    replace("000000000000", "123456789012");
    replace("\"initialization_id\": 1", "\"initialization_id\": 1234");
    return json;
}

inline OusterScanFrame frame(const OusterMetadata & metadata,
                            std::uint64_t revolution = 0,
                            std::int64_t period_ns = 100'000'000)
{
    OusterScanFrame result;
    result.revolution = revolution;
    result.frame_start_timestamp_ns = 900'000'000;
    result.columns_per_frame = metadata.columnsPerFrame();
    result.pixels_per_column = metadata.pixelsPerColumn();
    for (std::int64_t m = 1; m <= result.columns_per_frame; ++m) {
        result.column_timestamp_ns.push_back(result.frame_start_timestamp_ns +
            (period_ns / result.columns_per_frame) * m +
            ((period_ns % result.columns_per_frame) * m) / result.columns_per_frame);
    }
    for (std::size_t i = 0; i < result.sampleCount(); ++i) {
        result.range_mm.push_back(12000 + 8 * (i % 100));
        result.signal.push_back(47);
        result.reflectivity.push_back(33);
        result.near_ir.push_back(256);
    }
    return result;
}

struct OpticalCase {
    const char * name;
    double depth;
    std::optional<double> reflectance;
    std::optional<double> ambient;
    std::uint32_t range_mm;
    std::uint16_t signal;
    std::uint8_t reflectivity;
    std::uint16_t near_ir;

    NormalizedOpticalReturn input() const {
        NormalizedOpticalReturn value;
        if (depth > 0) {
            value.return_kind = OpticalReturnKind::kSurface;
            value.geometric_hit = true;
            value.path_length_m = value.reported_range_m = depth;
            value.apparent_reflectance = reflectance;
            value.ambient_near_ir_factor = ambient;
        }
        return value;
    }
};

// Exact noiseless channel fixtures, base_signal=800, min=.1m, max=120m,
// resolution=1mm. Random draws intentionally remain backend-specific.
inline const std::array<OpticalCase, 9> opticalCases{{
    {"black", 10, 0, std::nullopt, 10000, 0, 0, 0},
    {"missing", 10, std::nullopt, std::nullopt, 10000, 8, 50, 0},
    {"diffuse_ambient", 10, .5, .25, 10000, 4, 50, 64},
    {"explicit_dark_ambient", 2, 1, 0, 2000, 200, 100, 0},
    {"retroreflector", 4, 2, std::nullopt, 4000, 100, 122, 512},
    {"saturated", 2, 512, 512, 2000, 65535, 255, 65535},
    {"true_miss", 0, .5, 1, 0, 0, 0, 0},
    {"near_gate", .05, .5, 1, 0, 0, 0, 0},
    {"far_gate", 120, .5, 1, 0, 0, 0, 0},
}};

}  // namespace ouster_sim_core::conformance_v1
