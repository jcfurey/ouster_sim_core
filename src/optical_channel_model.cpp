// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/optical_channel_model.hpp"

#include "ouster_sim_core/product_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace ouster_sim_core {
namespace {

constexpr double kMinimumSignalDenominator = 0.0001;
constexpr double kRangeFractionDenominatorFloor = 0.1;
constexpr double kLambertianByteMaximum = 100.0;
constexpr double kReflectivityByteMaximum = 255.0;
constexpr double kRetroreflectiveLogSlope = 22.0;
constexpr double kDropoutReflectanceFloor = 0.33;
constexpr double kDropoutReflectanceScaleMaximum = 3.0;
constexpr double kRangeNoiseReflectanceFloor = 0.25;
constexpr double kRangeNoiseReflectanceScaleMaximum = 2.0;
constexpr double kLn9 = 2.1972245773362196;
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kHalfPi = 1.5707963267948966192313216916398;
constexpr double kUniformScale = 1.0 / 4503599627370496.0;  // 2^-52

constexpr std::uint64_t kHashDomain = 0xd4e12c77a65f3b19ULL;
constexpr std::uint64_t kNormalDomainA = 0xb40b54e2d847ba2dULL;
constexpr std::uint64_t kNormalDomainB = 0x698cc23a8c4f47d5ULL;

std::uint64_t splitMix64(std::uint64_t value) noexcept
{
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

void absorb(std::uint64_t & state, std::uint64_t value) noexcept
{
    state = splitMix64(state ^ splitMix64(value));
}

double uniformFromBits(std::uint64_t bits) noexcept
{
    // A 52-bit half-bin mapping is exactly representable throughout its range:
    // the first value is 2^-53 and the last is 1 - 2^-53. Using 53 random bits
    // makes the top half-bin round to 1.0 on binary64 implementations.
    const auto significand = bits >> 12U;
    return (static_cast<double>(significand) + 0.5) * kUniformScale;
}

bool finiteNonnegative(double value) noexcept
{
    return std::isfinite(value) && value >= 0.0;
}

void requireFiniteNonnegative(double value, const char * name)
{
    if (!finiteNonnegative(value)) {
        throw std::invalid_argument(
            std::string(name) + " must be finite and non-negative");
    }
}

void requireProbability(double value, const char * name)
{
    if (!finiteNonnegative(value) || value > 1.0) {
        throw std::invalid_argument(
            std::string(name) + " must be in the closed interval [0, 1]");
    }
}

bool detectionPairDisabled(double range_10_m, double range_80_m) noexcept
{
    return range_10_m == 0.0 && range_80_m == 0.0;
}

void validateDetectionPair(
    double range_10_m, double range_80_m, const char * name)
{
    requireFiniteNonnegative(range_10_m, name);
    requireFiniteNonnegative(range_80_m, name);
    if (!detectionPairDisabled(range_10_m, range_80_m) &&
        (range_10_m <= 0.0 || range_80_m <= range_10_m)) {
        throw std::invalid_argument(
            std::string(name) +
            " must be both zero or positive with the 80% range above 10%");
    }
}

void validateConfig(const OpticalChannelModelConfig & config)
{
    requireFiniteNonnegative(config.base_signal, "base_signal");
    requireFiniteNonnegative(
        config.missing_reflectance_for_signal,
        "missing_reflectance_for_signal");
    requireFiniteNonnegative(
        config.missing_reflectance_for_noise,
        "missing_reflectance_for_noise");
    requireFiniteNonnegative(config.near_ir_scale, "near_ir_scale");

    if (!std::isfinite(config.minimum_range_m) ||
        config.minimum_range_m <= 0.0) {
        throw std::invalid_argument(
            "minimum_range_m must be finite and greater than zero");
    }
    if (!std::isfinite(config.maximum_range_m) ||
        config.maximum_range_m <= config.minimum_range_m) {
        throw std::invalid_argument(
            "maximum_range_m must be finite and greater than minimum_range_m");
    }
    requireFiniteNonnegative(
        config.range_resolution_m, "range_resolution_m");
    requireFiniteNonnegative(
        config.range_noise_min_std_m, "range_noise_min_std_m");
    requireFiniteNonnegative(
        config.range_noise_max_std_m, "range_noise_max_std_m");
    if (config.range_noise_max_std_m < config.range_noise_min_std_m) {
        throw std::invalid_argument(
            "range_noise_max_std_m must be at least range_noise_min_std_m");
    }
    if (!std::isfinite(config.range_noise_reference_range_m) ||
        config.range_noise_reference_range_m <= 0.0) {
        throw std::invalid_argument(
            "range_noise_reference_range_m must be finite and greater than zero");
    }
    requireFiniteNonnegative(
        config.signal_noise_scale, "signal_noise_scale");
    requireFiniteNonnegative(
        config.near_ir_noise_scale, "near_ir_noise_scale");

    requireProbability(config.dropout_rate_close, "dropout_rate_close");
    requireProbability(config.dropout_rate_far, "dropout_rate_far");
    requireProbability(config.false_alarm_rate, "false_alarm_rate");
    validateDetectionPair(
        config.detection.range_10_d90_m,
        config.detection.range_80_d90_m,
        "D90 detection anchors");
    validateDetectionPair(
        config.detection.range_10_d50_m,
        config.detection.range_80_d50_m,
        "D50 detection anchors");
    requireFiniteNonnegative(
        config.detection.rolloff_fraction,
        "detection rolloff_fraction");
}

void validateInput(const NormalizedOpticalReturn & input)
{
    if (input.identity.return_index != 0) {
        throw std::invalid_argument(
            "the initial optical channel model supports primary returns only");
    }
    if (input.hasReturn()) {
        if (!std::isfinite(input.path_length_m) ||
            input.path_length_m <= 0.0) {
            throw std::invalid_argument(
                "a candidate optical return requires a finite positive path_length_m");
        }
        if (!std::isfinite(input.reported_range_m) ||
            input.reported_range_m <= 0.0) {
            throw std::invalid_argument(
                "a candidate optical return requires a finite positive reported_range_m");
        }
    } else if (input.path_length_m != 0.0 ||
        input.reported_range_m != 0.0) {
        throw std::invalid_argument(
            "a true optical miss must have zero path and reported ranges");
    }

    if (input.apparent_reflectance &&
        !finiteNonnegative(*input.apparent_reflectance)) {
        throw std::invalid_argument(
            "apparent_reflectance must be finite and non-negative when present");
    }
    if (input.ambient_near_ir_factor &&
        !finiteNonnegative(*input.ambient_near_ir_factor)) {
        throw std::invalid_argument(
            "ambient_near_ir_factor must be finite and non-negative when present");
    }
    if (input.incident_angle_rad &&
        (!std::isfinite(*input.incident_angle_rad) ||
         *input.incident_angle_rad < 0.0 ||
         *input.incident_angle_rad > kHalfPi)) {
        throw std::invalid_argument(
            "incident_angle_rad must be finite and in [0, pi/2] when present");
    }
}

OpticalRandomKey randomKey(
    const NormalizedOpticalReturn & input,
    const OpticalChannelContext & context,
    OpticalRandomEffect effect,
    std::uint64_t subdraw = 0) noexcept
{
    OpticalRandomKey key;
    key.sensor_stream_id = context.sensor_stream_id;
    key.epoch = context.epoch;
    key.revolution = input.identity.revolution;
    key.measurement_id = input.identity.measurement_id;
    key.ring_id = input.identity.ring_id;
    key.return_index = input.identity.return_index;
    key.effect = effect;
    key.subdraw = subdraw;
    return key;
}

double rangeFraction(double range_m, double reference_range_m) noexcept
{
    return std::min(
        range_m /
            std::max(reference_range_m, kRangeFractionDenominatorFloor),
        1.0);
}

double dropoutProbability(
    double range_m,
    double apparent_reflectance,
    const OpticalChannelModelConfig & config) noexcept
{
    const double interpolation = rangeFraction(
        range_m, config.maximum_range_m);
    const double stochastic_probability =
        config.dropout_rate_close + interpolation *
            (config.dropout_rate_far - config.dropout_rate_close);
    const double reflectance_scale = std::min(
        1.0 / std::max(apparent_reflectance, kDropoutReflectanceFloor),
        kDropoutReflectanceScaleMaximum);
    const double random_keep_probability = 1.0 - std::min(
        stochastic_probability * reflectance_scale, 1.0);
    const double calibrated_keep_probability =
        calibratedDetectionProbability(
            range_m,
            apparent_reflectance,
            config.detection,
            config.maximum_range_m);
    return 1.0 - random_keep_probability * calibrated_keep_probability;
}

double rangeNoiseSigma(
    double range_m,
    double apparent_reflectance,
    const OpticalChannelModelConfig & config) noexcept
{
    const double interpolation = rangeFraction(
        range_m, config.range_noise_reference_range_m);
    double sigma = config.range_noise_min_std_m + interpolation *
        (config.range_noise_max_std_m - config.range_noise_min_std_m);
    sigma *= std::min(
        1.0 /
            std::sqrt(std::max(
                apparent_reflectance, kRangeNoiseReflectanceFloor)),
        kRangeNoiseReflectanceScaleMaximum);
    return sigma;
}

std::uint32_t quantizeRangeMillimeters(
    double range_m, double resolution_m) noexcept
{
    long double millimeters =
        static_cast<long double>(range_m) * 1000.0L;
    if (std::isnan(millimeters) || millimeters <= 0.0L) {
        return 0;
    }

    if (resolution_m > 0.0) {
        long double resolution_mm =
            static_cast<long double>(resolution_m) * 1000.0L;
        // Product profiles use integer-millimetre steps (1, 3, or 8 mm).
        // Snap their binary64 representations back to that exact integer
        // before applying the packet-domain quantizer.
        const long double integral_resolution_mm =
            std::round(resolution_mm);
        if (std::fabs(
                resolution_mm - integral_resolution_mm) <= 1.0e-12L) {
            resolution_mm = integral_resolution_mm;
        }
        millimeters =
            std::floor(millimeters / resolution_mm + 0.5L) * resolution_mm;
    }

    if (millimeters <= 0.0L) {
        return 0;
    }
    constexpr auto maximum = std::numeric_limits<std::uint32_t>::max();
    if (!std::isfinite(millimeters) ||
        millimeters >= static_cast<long double>(maximum)) {
        return maximum;
    }
    return static_cast<std::uint32_t>(std::floor(millimeters + 0.5L));
}

std::uint16_t clampU16(double value) noexcept
{
    if (std::isnan(value) || value <= 0.0) {
        return 0;
    }
    constexpr auto maximum = std::numeric_limits<std::uint16_t>::max();
    if (!std::isfinite(value) || value >= static_cast<double>(maximum)) {
        return maximum;
    }
    return static_cast<std::uint16_t>(value);
}

double nonnegativeProductOverSquaredDenominator(
    double first, double second, double denominator_root) noexcept
{
    if (first <= 0.0 || second <= 0.0) {
        return 0.0;
    }

    int first_exponent = 0;
    int second_exponent = 0;
    const double first_fraction = std::frexp(first, &first_exponent);
    const double second_fraction = std::frexp(second, &second_exponent);

    double denominator_fraction = 0.0;
    int denominator_exponent = 0;
    if (denominator_root <= std::sqrt(kMinimumSignalDenominator)) {
        denominator_fraction = std::frexp(
            kMinimumSignalDenominator, &denominator_exponent);
    } else {
        int root_exponent = 0;
        const double root_fraction =
            std::frexp(denominator_root, &root_exponent);
        denominator_fraction = root_fraction * root_fraction;
        denominator_exponent = 2 * root_exponent;
    }

    const double fraction =
        first_fraction * second_fraction / denominator_fraction;
    return std::scalbn(
        fraction,
        first_exponent + second_exponent - denominator_exponent);
}

QuantizedOpticalReturn missFrom(
    const NormalizedOpticalReturn & input) noexcept
{
    QuantizedOpticalReturn output;
    output.identity = input.identity;
    output.geometric_hit = input.geometric_hit;
    return output;
}

bool detectionEnabled(const OpticalDetectionParameters & parameters) noexcept
{
    return parameters.range_10_d90_m > 0.0 &&
           parameters.range_80_d90_m > parameters.range_10_d90_m;
}

}  // namespace

OusterReturnSample QuantizedOpticalReturn::assemblerSample() const noexcept
{
    OusterReturnSample output;
    output.identity = identity;
    output.range_mm = range_mm;
    output.signal = signal;
    output.reflectivity = reflectivity;
    output.near_ir = near_ir;
    output.is_hit = hasReturn();
    return output;
}

std::uint64_t deterministicOpticalRandomBits(
    std::uint64_t seed, const OpticalRandomKey & key) noexcept
{
    std::uint64_t state = splitMix64(seed ^ kHashDomain);
    absorb(state, key.sensor_stream_id);
    absorb(state, key.epoch);
    absorb(state, key.revolution);
    absorb(state, key.measurement_id);
    absorb(state, key.ring_id);
    absorb(state, key.return_index);
    absorb(state, static_cast<std::uint32_t>(key.effect));
    absorb(state, key.subdraw);
    return state;
}

double deterministicOpticalUniform01(
    std::uint64_t seed, const OpticalRandomKey & key) noexcept
{
    return uniformFromBits(deterministicOpticalRandomBits(seed, key));
}

double deterministicOpticalStandardNormal(
    std::uint64_t seed, const OpticalRandomKey & key) noexcept
{
    const double first = uniformFromBits(
        deterministicOpticalRandomBits(seed ^ kNormalDomainA, key));
    const double second = uniformFromBits(
        deterministicOpticalRandomBits(seed ^ kNormalDomainB, key));
    return std::sqrt(-2.0 * std::log(first)) *
        std::cos(kTwoPi * second);
}

double calibratedDetectionRangeM(
    double apparent_reflectance,
    double range_10_m,
    double range_80_m) noexcept
{
    if (range_10_m <= 0.0 || range_80_m <= range_10_m ||
        apparent_reflectance <= 0.0) {
        return 0.0;
    }
    const double exponent =
        std::log(range_80_m / range_10_m) / std::log(8.0);
    return range_10_m *
        std::exp(exponent * std::log(apparent_reflectance / 0.1));
}

double calibratedDetectionProbability(
    double range_m,
    double apparent_reflectance,
    const OpticalDetectionParameters & parameters,
    double max_range_m) noexcept
{
    if (range_m <= 0.0 || range_m >= max_range_m) {
        return 0.0;
    }
    if (!detectionEnabled(parameters)) {
        return 1.0;
    }
    if (apparent_reflectance <= 0.0) {
        return 0.0;
    }

    const double d90 = calibratedDetectionRangeM(
        apparent_reflectance,
        parameters.range_10_d90_m,
        parameters.range_80_d90_m);
    if (d90 <= 0.0) {
        return 0.0;
    }

    double d50 = calibratedDetectionRangeM(
        apparent_reflectance,
        parameters.range_10_d50_m,
        parameters.range_80_d50_m);
    if (d50 <= d90) {
        d50 = d90 * (1.0 + std::max(parameters.rolloff_fraction, 0.01));
    }

    const double slope = kLn9 / (d50 - d90);
    return 1.0 / (1.0 + std::exp(slope * (range_m - d50)));
}

std::uint8_t quantizeOusterReflectivity(
    double apparent_reflectance) noexcept
{
    if (std::isnan(apparent_reflectance) || apparent_reflectance <= 0.0) {
        return 0;
    }
    if (!std::isfinite(apparent_reflectance)) {
        return std::numeric_limits<std::uint8_t>::max();
    }
    if (apparent_reflectance <= 1.0) {
        const double mapped = std::min(
            apparent_reflectance * kLambertianByteMaximum,
            kLambertianByteMaximum);
        return static_cast<std::uint8_t>(mapped);
    }
    const double mapped = std::min(
        kLambertianByteMaximum +
            std::log2(apparent_reflectance) * kRetroreflectiveLogSlope,
        kReflectivityByteMaximum);
    return static_cast<std::uint8_t>(mapped);
}

OpticalChannelModel::OpticalChannelModel(OpticalChannelModelConfig config)
    : config_(std::move(config))
{
    validateConfig(config_);
}

OpticalChannelModelConfig opticalChannelModelConfigFromProfile(
    const OusterProductProfile & profile,
    int columns_per_frame,
    double lidar_hz)
{
    if (!profile.supported) {
        throw std::invalid_argument(
            "an optical channel model requires a supported product profile");
    }
    if (columns_per_frame <= 0 || !std::isfinite(lidar_hz) ||
        lidar_hz <= 0.0) {
        throw std::invalid_argument(
            "active mode requires positive columns_per_frame and lidar_hz");
    }

    const double range_scale = ousterModeRangeScale(
        profile, columns_per_frame, lidar_hz);
    const double precision_scale = ousterModePrecisionScale(
        profile, columns_per_frame, lidar_hz);

    OpticalChannelModelConfig config;
    config.minimum_range_m = profile.minimum_range_m;
    config.maximum_range_m = profile.representable_range_m;
    config.range_resolution_m = profile.range_resolution_m;
    config.detection.range_10_d90_m =
        profile.detection_range_10_d90_m * range_scale;
    config.detection.range_80_d90_m =
        profile.detection_range_80_d90_m * range_scale;
    config.detection.range_10_d50_m =
        profile.detection_range_10_d50_m * range_scale;
    config.detection.range_80_d50_m =
        profile.detection_range_80_d50_m * range_scale;
    config.range_noise_min_std_m =
        profile.precision_min_std_m * precision_scale;
    config.range_noise_max_std_m =
        profile.precision_max_std_m * precision_scale;
    config.range_noise_reference_range_m =
        config.detection.range_10_d90_m > 0.0
            ? config.detection.range_10_d90_m
            : config.maximum_range_m;
    config.false_alarm_rate = profile.false_positive_rate;

    // Run the same validation as direct construction before exposing the
    // derived values to an embedding package.
    static_cast<void>(OpticalChannelModel(config));
    return config;
}

QuantizedOpticalReturn OpticalChannelModel::process(
    const NormalizedOpticalReturn & input,
    const OpticalChannelContext & context) const
{
    validateInput(input);

    if (!input.hasReturn()) {
        if (config_.false_alarm_rate <= 0.0 ||
            deterministicOpticalUniform01(
                context.seed,
                randomKey(
                    input, context,
                    OpticalRandomEffect::kFalseAlarmDecision)) >=
                config_.false_alarm_rate) {
            return missFrom(input);
        }

        const double unit_range = deterministicOpticalUniform01(
            context.seed,
            randomKey(input, context, OpticalRandomEffect::kFalseAlarmRange));
        const double false_range = config_.minimum_range_m + unit_range *
            (config_.maximum_range_m - config_.minimum_range_m);
        const auto false_range_mm = quantizeRangeMillimeters(
            false_range, config_.range_resolution_m);
        const double packet_false_range_m =
            static_cast<double>(false_range_mm) / 1000.0;
        if (packet_false_range_m < config_.minimum_range_m ||
            packet_false_range_m >= config_.maximum_range_m) {
            return missFrom(input);
        }

        QuantizedOpticalReturn output;
        output.identity = input.identity;
        output.return_kind = OpticalReturnKind::kFalseAlarm;
        output.range_mm = false_range_mm;
        output.signal = 1;
        output.reflectivity = config_.missing_reflectivity_byte;
        return output;
    }

    double reported_range_m = input.reported_range_m;
    if (reported_range_m < config_.minimum_range_m ||
        reported_range_m >= config_.maximum_range_m) {
        return missFrom(input);
    }

    const double reflectance_for_noise = input.apparent_reflectance.value_or(
        config_.missing_reflectance_for_noise);
    if (config_.dropout_rate_close > 0.0 ||
        config_.dropout_rate_far > 0.0 ||
        detectionEnabled(config_.detection)) {
        const double probability = dropoutProbability(
            reported_range_m, reflectance_for_noise, config_);
        if (deterministicOpticalUniform01(
                context.seed,
                randomKey(input, context, OpticalRandomEffect::kDropout)) <
            probability) {
            return missFrom(input);
        }
    }

    if (config_.range_noise_min_std_m > 0.0 ||
        config_.range_noise_max_std_m > 0.0) {
        const double sigma = rangeNoiseSigma(
            reported_range_m, reflectance_for_noise, config_);
        reported_range_m = std::max(
            reported_range_m + deterministicOpticalStandardNormal(
                context.seed,
                randomKey(input, context, OpticalRandomEffect::kRangeNoise)) *
                sigma,
            0.0);
    }

    if (!std::isfinite(reported_range_m)) {
        return missFrom(input);
    }
    const auto range_mm = quantizeRangeMillimeters(
        reported_range_m, config_.range_resolution_m);
    const double packet_range_m = static_cast<double>(range_mm) / 1000.0;
    if (packet_range_m < config_.minimum_range_m ||
        packet_range_m >= config_.maximum_range_m) {
        return missFrom(input);
    }

    QuantizedOpticalReturn output;
    output.identity = input.identity;
    output.return_kind = input.return_kind;
    output.geometric_hit = input.geometric_hit;
    output.range_mm = range_mm;

    const double reflectance_for_signal = input.apparent_reflectance.value_or(
        config_.missing_reflectance_for_signal);
    double signal = nonnegativeProductOverSquaredDenominator(
        config_.base_signal,
        reflectance_for_signal,
        input.path_length_m);
    if (config_.signal_noise_scale > 0.0 && std::isfinite(signal)) {
        signal = std::max(
            signal + deterministicOpticalStandardNormal(
                context.seed,
                randomKey(input, context, OpticalRandomEffect::kSignalNoise)) *
                std::sqrt(std::max(signal, 0.0)) *
                config_.signal_noise_scale,
            0.0);
    }
    output.signal = clampU16(signal);

    output.reflectivity = input.apparent_reflectance
        ? quantizeOusterReflectivity(*input.apparent_reflectance)
        : config_.missing_reflectivity_byte;

    double near_ir = 0.0;
    if (input.ambient_near_ir_factor) {
        near_ir = *input.ambient_near_ir_factor * config_.near_ir_scale;
    } else if (input.apparent_reflectance) {
        // Compatibility fallback for adapters without a passive-NIR plane.
        near_ir = *input.apparent_reflectance * config_.near_ir_scale;
    }
    if (config_.near_ir_noise_scale > 0.0 && near_ir > 0.0 &&
        std::isfinite(near_ir)) {
        near_ir = std::max(
            near_ir + deterministicOpticalStandardNormal(
                context.seed,
                randomKey(input, context, OpticalRandomEffect::kNearIrNoise)) *
                std::sqrt(near_ir) * config_.near_ir_noise_scale,
            0.0);
    }
    output.near_ir = clampU16(near_ir);
    return output;
}

std::vector<QuantizedOpticalReturn> OpticalChannelModel::process(
    std::span<const NormalizedOpticalReturn> inputs,
    const OpticalChannelContext & context) const
{
    std::vector<QuantizedOpticalReturn> outputs;
    outputs.reserve(inputs.size());
    for (const auto & input : inputs) {
        outputs.push_back(process(input, context));
    }
    return outputs;
}

}  // namespace ouster_sim_core
