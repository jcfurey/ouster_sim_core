// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/firing_table.hpp"
#include "ouster_sim_core/return_sample.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace ouster_sim_core {

/// Version of the normalized optical contract and its deterministic random
/// mapping. Increment this value whenever a change intentionally invalidates
/// the frozen random or synthesized-channel golden vectors.
inline constexpr std::uint32_t kOpticalChannelContractVersion = 1;

/// Origin of a simulator-normalized candidate return.
///
/// The distinction is diagnostic and does not change packet encoding. It
/// prevents adapters from conflating a geometry hit with an effective return
/// produced by participating media, transmission, or sensor noise.
enum class OpticalReturnKind : std::uint8_t {
    kNone = 0,
    kSurface = 1,
    kMedium = 2,
    kGhost = 3,
    kFalseAlarm = 4,
};

/// Simulator-neutral optical response before Ouster channel synthesis.
///
/// `apparent_reflectance` is dimensionless 865 nm reflectance after the
/// simulator-specific material, incidence, transmission, and obscurant model.
/// A present value of zero is a real black response and is intentionally
/// different from a missing value. `ambient_near_ir_factor` has the same
/// explicit presence semantics and is multiplied by the configured near-IR
/// scale. `path_length_m` drives received signal attenuation, while
/// `reported_range_m` drives detection, range noise, quantization, and packet
/// range. They are equal for direct returns but intentionally remain separate
/// for transmitted, refracted, and ghost paths. `incident_angle_rad`, when
/// available, is the unsigned angle between the incoming ray and the oriented
/// surface normal in [0, pi/2]. It is retained for future calibrated models;
/// apparent reflectance already includes the adapter's incidence response, so
/// this scalar model does not apply the angle a second time. A kNone input is a
/// true miss and must have both ranges set to zero.
struct NormalizedOpticalReturn {
    OusterFiringIdentity identity;
    OpticalReturnKind return_kind = OpticalReturnKind::kNone;
    bool geometric_hit = false;
    double path_length_m = 0.0;
    double reported_range_m = 0.0;
    std::optional<double> apparent_reflectance;
    std::optional<double> ambient_near_ir_factor;
    std::optional<double> incident_angle_rad;

    bool hasReturn() const noexcept {
        return return_kind != OpticalReturnKind::kNone;
    }
};

/// Exact packet-domain result of optical channel synthesis.
struct QuantizedOpticalReturn {
    OusterFiringIdentity identity;
    OpticalReturnKind return_kind = OpticalReturnKind::kNone;
    bool geometric_hit = false;
    std::uint32_t range_mm = 0;
    std::uint16_t signal = 0;
    std::uint8_t reflectivity = 0;
    std::uint16_t near_ir = 0;

    bool hasReturn() const noexcept {
        return return_kind != OpticalReturnKind::kNone;
    }

    /// Lossless handoff to OusterRevolutionAssembler.
    OusterReturnSample assemblerSample() const noexcept;
};

/// Reset/session context that is stable for one call to the channel model.
struct OpticalChannelContext {
    std::uint64_t seed = 0;
    std::uint64_t sensor_stream_id = 0;
    std::uint64_t epoch = 0;
};

/// Independent random-effect lanes used by the stateless generator.
enum class OpticalRandomEffect : std::uint32_t {
    kFalseAlarmDecision = 1,
    kFalseAlarmRange = 2,
    kDropout = 4,
    kRangeNoise = 5,
    kSignalNoise = 6,
    kNearIrNoise = 7,
};

/// Complete counter key for one deterministic random draw.
///
/// No dispatch index, batch position, thread identity, or mutable generator
/// state participates in this key. Rebatching and reordering therefore cannot
/// change the result for a firing identity.
struct OpticalRandomKey {
    std::uint64_t sensor_stream_id = 0;
    std::uint64_t epoch = 0;
    std::uint64_t revolution = 0;
    std::uint32_t measurement_id = 0;
    std::uint16_t ring_id = 0;
    std::uint8_t return_index = 0;
    OpticalRandomEffect effect = OpticalRandomEffect::kDropout;
    std::uint64_t subdraw = 0;
};

/// Stable 64-bit random word for `seed` and the complete counter key.
std::uint64_t deterministicOpticalRandomBits(
    std::uint64_t seed, const OpticalRandomKey & key) noexcept;

/// Uniform draw in the open interval (0, 1).
double deterministicOpticalUniform01(
    std::uint64_t seed, const OpticalRandomKey & key) noexcept;

/// Standard-normal draw. The key's subdraw selects an independent Gaussian
/// lane; two domain-separated uniform words are derived without shared state.
/// The keyed 64-bit word and uniform mapping are bit-exact contract surfaces.
/// Gaussian and noisy-channel golden vectors additionally depend on the
/// platform libm implementation; conformance across different libm/toolchain
/// combinations should use documented numeric tolerances.
double deterministicOpticalStandardNormal(
    std::uint64_t seed, const OpticalRandomKey & key) noexcept;

/// Power-law interpolation through the calibrated 10% and 80% reflectance
/// detection-range anchors. Invalid/disabled anchors return zero.
double calibratedDetectionRangeM(
    double apparent_reflectance,
    double range_10_m,
    double range_80_m) noexcept;

struct OpticalDetectionParameters {
    double range_10_d90_m = 0.0;
    double range_80_d90_m = 0.0;
    double range_10_d50_m = 0.0;
    double range_80_d50_m = 0.0;
    double rolloff_fraction = 0.15;
};

/// Logistic product-calibrated detection probability. With valid anchors,
/// probability is exactly 0.9 at D90 and 0.5 at D50 (within floating-point
/// evaluation). Disabled D90 anchors give probability one inside max range.
double calibratedDetectionProbability(
    double range_m,
    double apparent_reflectance,
    const OpticalDetectionParameters & parameters,
    double max_range_m) noexcept;

/// Ouster reflectivity encoding: [0,1] maps linearly to [0,100], then a
/// logarithmic retroreflective band extends to 255.
std::uint8_t quantizeOusterReflectivity(double apparent_reflectance) noexcept;

/// Scalar channel-model calibration independent of any simulator SDK.
struct OpticalChannelModelConfig {
    double base_signal = 800.0;
    std::uint8_t missing_reflectivity_byte = 50;
    double missing_reflectance_for_signal = 1.0;
    double missing_reflectance_for_noise = 0.5;
    double near_ir_scale = 256.0;

    double minimum_range_m = 0.1;
    double maximum_range_m = 120.0;
    double range_resolution_m = 0.001;

    double range_noise_min_std_m = 0.0;
    double range_noise_max_std_m = 0.0;
    double range_noise_reference_range_m = 120.0;
    double signal_noise_scale = 0.0;
    double near_ir_noise_scale = 0.0;

    double dropout_rate_close = 0.0;
    double dropout_rate_far = 0.0;
    double false_alarm_rate = 0.0;

    OpticalDetectionParameters detection;
};

struct OusterProductProfile;

/// Derive range, detection, precision-noise, and false-alarm defaults from a
/// supported physical product profile at the active measurement-column rate.
OpticalChannelModelConfig opticalChannelModelConfigFromProfile(
    const OusterProductProfile & profile,
    int columns_per_frame,
    double lidar_hz);

/// Stateless simulator-neutral Ouster channel model.
///
/// The scalar equations intentionally retain the established Gazebo model's
/// inverse-square signal, reflectivity encoding, detection calibration,
/// range quantization, and shot-noise approximations. Randomness is replaced
/// by a counter-keyed generator so output is invariant to call order and batch
/// segmentation. The object is immutable after validated construction and is
/// safe to invoke concurrently. Spatial edge suppression is deliberately not
/// represented by a caller-provided scalar flag: a later frame API must derive
/// it from a canonical raw-frame neighborhood so adapters cannot make divergent
/// parity decisions.
class OpticalChannelModel {
public:
    explicit OpticalChannelModel(OpticalChannelModelConfig config);

    const OpticalChannelModelConfig & config() const noexcept {
        return config_;
    }

    QuantizedOpticalReturn process(
        const NormalizedOpticalReturn & input,
        const OpticalChannelContext & context) const;

    std::vector<QuantizedOpticalReturn> process(
        std::span<const NormalizedOpticalReturn> inputs,
        const OpticalChannelContext & context) const;

private:
    OpticalChannelModelConfig config_;
};

}  // namespace ouster_sim_core
