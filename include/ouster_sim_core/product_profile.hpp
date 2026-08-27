// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

namespace ouster_sim_core {

/// Largest decoded millimetre value carried by an Ouster RNG15 field.
///
/// RNG15 packet layouts store 15 range bits with the lowest three decoded
/// bits implicit, giving values from 0 through 32767 * 8 millimetres.
inline constexpr double kRng15MaximumRepresentableRangeM = 262.136;

/// Physical Ouster product family, independent of its packet layout.
enum class OusterModel {
    Unknown,
    OS0,
    OS1,
    OS2,
    OSDome,
    OS1Max,
};

enum class OusterGeneration {
    Unknown,
    Gen1,  ///< Original OS1 / L1.
    Gen2,  ///< Rev C/D/05/06 family (L2/L2X).
    Gen3,  ///< Rev7 / L3.
    Gen4,  ///< Rev8 / L4.
};

enum class OusterRevision {
    Auto,
    Unknown,
    Gen1,
    RevC,
    RevD,
    Rev05,
    Rev06,
    Rev062,
    Rev07,
    Rev071,
    Rev08,
};

/// Simulator-neutral optical and ranging calibration for one product.
///
/// Product identity deliberately remains separate from packet metadata:
/// product_line identifies the optical family, while product_part_number (or
/// an explicit revision) identifies the hardware revision. Firmware may
/// change packet capabilities and, for Gen1, detection performance without
/// changing either field.
struct OusterProductProfile {
    std::string id;
    OusterModel model = OusterModel::Unknown;
    OusterGeneration generation = OusterGeneration::Unknown;
    OusterRevision revision = OusterRevision::Unknown;
    bool supported = false;
    bool revision_inferred = false;
    bool fallback_revision = false;

    // Optical detection calibration at 100 klx. D90 values are vendor
    // specifications. D50 is populated where historical data publishes it;
    // otherwise a channel model can use a documented smooth rolloff anchored
    // exactly at D90.
    double detection_range_10_d90_m = 0.0;
    double detection_range_80_d90_m = 0.0;
    double detection_range_10_d50_m = 0.0;
    double detection_range_80_d50_m = 0.0;
    double representable_range_m = 120.0;
    double minimum_range_m = 0.3;

    // Datasheet precision envelope and systematic accuracy. Precision is a
    // random-noise input; accuracy remains separate as a systematic bound.
    double precision_min_std_m = 0.003;
    double precision_max_std_m = 0.015;
    double lambertian_accuracy_m = 0.03;
    double retroreflector_accuracy_m = 0.10;

    double range_resolution_m = 0.001;
    double beam_diameter_m = 0.0;
    double beam_divergence_fwhm_deg = 0.0;
    double false_positive_rate = 1.0e-4;
    int max_returns = 1;

    // Detection ranges are specified at this column rate. Active-mode range
    // and precision scale relative to this reference rate.
    double reference_columns_per_second = 10240.0;
};

struct OusterProductProfileRequest {
    std::string product_line;
    std::string product_part_number;
    std::string hardware_revision = "auto";
    int firmware_major = 0;
    int firmware_minor = 0;
    int beam_count = 0;
    bool low_data_profile = false;
};

OusterModel parseOusterModel(const std::string & product_line);
OusterRevision parseOusterRevision(const std::string & value);
const char * toString(OusterModel model) noexcept;
const char * toString(OusterGeneration generation) noexcept;
const char * toString(OusterRevision revision) noexcept;

/// Resolve product identity into its calibrated physical profile.
///
/// Unknown or ambiguous part numbers use a conservative model-specific
/// fallback and set fallback_revision. Incompatible model/revision pairs are
/// preserved for diagnostics and returned with supported=false.
OusterProductProfile resolveOusterProductProfile(
    const OusterProductProfileRequest & request);

/// Effective range multiplier relative to the profile's reference mode.
/// Ouster's operating-mode table gives 1.19x per halving of gathered points.
double ousterModeRangeScale(const OusterProductProfile & profile,
                            int columns_per_frame, double lidar_hz);

/// Precision multiplier for the active point-gathering rate. A halving of the
/// gathered-point rate gives 0.71x sigma and a doubling gives 1.41x.
double ousterModePrecisionScale(const OusterProductProfile & profile,
                                int columns_per_frame, double lidar_hz);

}  // namespace ouster_sim_core
