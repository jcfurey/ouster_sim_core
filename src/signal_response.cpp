// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/signal_response.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ouster_sim_core {
namespace {

// Beyond this ratio of linear to saturation signal, the saturated value
// equals saturation_signal to double precision.
constexpr double kFullySaturatedRatio = 1.0e100;
constexpr double kMaximumModeGain = 4.0;  // two octaves, as the mode scales

bool positiveFinite(double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

// Linear signal of a 100% diffuse target per measurement at the recordings'
// column rate.
double fullScaleSignal(
    const OusterSignalResponse & response, double range_m) noexcept
{
    const auto & range = response.range_m;
    const auto & value = response.signal_m2;
    const int last = response.knot_count - 1;
    if (range_m <= range[0]) {
        return value[0] / (range[0] * range[0]);
    }
    if (range_m >= range[last]) {
        return value[last] / (range_m * range_m);
    }
    int upper = 1;
    while (range[upper] < range_m) {
        ++upper;
    }
    const double t =
        (range_m - range[upper - 1]) / (range[upper] - range[upper - 1]);
    const double interpolated =
        value[upper - 1] + t * (value[upper] - value[upper - 1]);
    return interpolated / (range_m * range_m);
}

}  // namespace

SignalEstimate evaluateSignalResponse(
    const OusterSignalResponse & response,
    double apparent_reflectance,
    double range_m,
    double gain,
    double mode_gain) noexcept
{
    if (!response.measured() || !positiveFinite(apparent_reflectance) ||
        !positiveFinite(range_m) || !positiveFinite(gain) ||
        !positiveFinite(mode_gain)) {
        return {};
    }
    const double linear =
        gain * apparent_reflectance * fullScaleSignal(response, range_m);
    const double ratio = linear / response.saturation_signal;
    double saturated = response.saturation_signal;
    double shot = 0.0;  // the saturated slope dc/dx is zero
    if (ratio < kFullySaturatedRatio) {
        const double root = std::cbrt(ratio);
        const double t = 1.0 + root * root * root * root;  // 1 + ratio^(4/3)
        const double sqrt_t = std::sqrt(t);
        const double fraction = 1.0 / (sqrt_t * std::sqrt(sqrt_t));  // t^(-3/4)
        saturated = linear * fraction;
        // dc/dx = t^(-7/4) = fraction / t
        shot = response.shot_noise_factor * (fraction / t) * std::sqrt(linear);
    }
    const double floor = response.relative_noise * saturated;
    return {mode_gain * saturated,
            std::sqrt(mode_gain) * std::hypot(shot, floor)};
}

double signalModeGain(
    const OusterSignalResponse & response,
    int columns_per_frame,
    double lidar_hz) noexcept
{
    const double active = static_cast<double>(columns_per_frame) * lidar_hz;
    if (!response.measured() ||
        !positiveFinite(response.columns_per_second) ||
        columns_per_frame <= 0 || !positiveFinite(active)) {
        return 1.0;
    }
    return std::clamp(response.columns_per_second / active,
                      1.0 / kMaximumModeGain, kMaximumModeGain);
}

void validateSignalResponse(const OusterSignalResponse & response)
{
    if (response.knot_count == 0) {
        return;
    }
    if (response.knot_count < 0 ||
        response.knot_count > OusterSignalResponse::kMaxKnots) {
        throw std::invalid_argument(
            "signal response knot_count must be in [0, " +
            std::to_string(OusterSignalResponse::kMaxKnots) + "]");
    }
    for (int i = 0; i < response.knot_count; ++i) {
        if (!positiveFinite(response.range_m[i]) ||
            (i > 0 && response.range_m[i] <= response.range_m[i - 1])) {
            throw std::invalid_argument(
                "signal response knot ranges must be positive, finite and "
                "strictly ascending");
        }
        if (!std::isfinite(response.signal_m2[i]) ||
            response.signal_m2[i] < 0.0) {
            throw std::invalid_argument(
                "signal response knot values must be finite and "
                "non-negative");
        }
    }
    if (!positiveFinite(response.columns_per_second) ||
        !positiveFinite(response.saturation_signal)) {
        throw std::invalid_argument(
            "signal response column rate and saturation signal must be "
            "finite and positive");
    }
    if (!std::isfinite(response.shot_noise_factor) ||
        response.shot_noise_factor < 0.0 ||
        !std::isfinite(response.relative_noise) ||
        response.relative_noise < 0.0) {
        throw std::invalid_argument(
            "signal response noise terms must be finite and non-negative");
    }
}

}  // namespace ouster_sim_core
