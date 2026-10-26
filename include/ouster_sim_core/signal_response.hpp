// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

namespace ouster_sim_core {

/// base_signal at which a measured response reports what its recordings
/// showed. Channel models scale a measured response's linear signal by
/// base_signal / kReferenceBaseSignal, so base_signal keeps one meaning,
/// sensor sensitivity, for measured and inverse-square products alike.
inline constexpr double kReferenceBaseSignal = 800.0;

/// SIGNAL response of a product, fitted to recordings of real sensors.
///
/// A return of apparent reflectance rho at range r carries the linear signal
/// x = rho * signal_m2(r) / r^2 per measurement at the recordings' column
/// rate. signal_m2 interpolates the knots linearly in range and keeps the
/// last knot's value beyond it. Below the first knot x keeps its first-knot
/// value: the receiver sees a near return out of focus, so the light it
/// collects stops growing as the target approaches.
///
/// The detector saturates x into
/// c = x / (1 + (x / saturation_signal)^(4/3))^(3/4); the 4/3 knee is what a
/// fit with it free returns, and it evaluates with a cube and square roots.
/// Its noise is shot noise with an excess factor, shot_noise_factor *
/// sqrt(x), carried through the saturation's slope dc/dx, and a floor of
/// relative_noise * c, added in quadrature. A mode that gathers m times the
/// recordings' shots per measurement (m = columns_per_second over the active
/// columns per second) reports m * c with sigma * sqrt(m).
///
/// knot_count == 0 marks a product without recordings; channel models then
/// keep the inverse-square base_signal * rho / r^2.
struct OusterSignalResponse {
    static constexpr int kMaxKnots = 24;

    int knot_count = 0;
    std::array<double, kMaxKnots> range_m{};    ///< Ascending, positive.
    std::array<double, kMaxKnots> signal_m2{};  ///< 100% diffuse x * r^2.
    double columns_per_second = 0.0;  ///< Column rate of the recordings.
    double saturation_signal = 0.0;   ///< SIGNAL the detector saturates at.
    double shot_noise_factor = 0.0;   ///< sigma / sqrt(x) before saturation.
    double relative_noise = 0.0;      ///< Noise floor as a fraction of SIGNAL.

    bool measured() const noexcept { return knot_count > 0; }
};

/// Mean SIGNAL of one return and the standard deviation of its noise.
struct SignalEstimate {
    double mean = 0.0;
    double sigma = 0.0;
};

/// Evaluate a measured response. `gain` scales the linear signal (sensor
/// sensitivity relative to the recordings) and `mode_gain` is m. A response
/// without recordings, or a non-positive or non-finite reflectance, range or
/// gain, gives zero.
SignalEstimate evaluateSignalResponse(
    const OusterSignalResponse & response,
    double apparent_reflectance,
    double range_m,
    double gain,
    double mode_gain) noexcept;

/// m for the active mode: the recordings' column rate over
/// columns_per_frame * lidar_hz, clamped to two octaves like the
/// operating-mode range scale. 1 for a response without recordings or an
/// invalid mode.
double signalModeGain(
    const OusterSignalResponse & response,
    int columns_per_frame,
    double lidar_hz) noexcept;

/// Throw std::invalid_argument unless the response is empty or has ascending
/// positive knot ranges, finite non-negative knot values, a positive column
/// rate and saturation signal, and non-negative noise terms.
void validateSignalResponse(const OusterSignalResponse & response);

}  // namespace ouster_sim_core
