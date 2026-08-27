// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#include "ouster_sim_core/product_profile.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace ouster_sim_core {
namespace {

std::string canonical(const std::string & value)
{
    std::string out;
    out.reserve(value.size());
    for (const unsigned char c : value) {
        if (std::isalnum(c)) {
            out.push_back(static_cast<char>(std::toupper(c)));
        }
    }
    return out;
}

bool endsWith(const std::string & value, const std::string & suffix)
{
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

OusterGeneration generationOf(OusterRevision revision)
{
    switch (revision) {
    case OusterRevision::Gen1:
        return OusterGeneration::Gen1;
    case OusterRevision::RevC:
    case OusterRevision::RevD:
    case OusterRevision::Rev05:
    case OusterRevision::Rev06:
    case OusterRevision::Rev062:
        return OusterGeneration::Gen2;
    case OusterRevision::Rev07:
    case OusterRevision::Rev071:
        return OusterGeneration::Gen3;
    case OusterRevision::Rev08:
        return OusterGeneration::Gen4;
    default:
        return OusterGeneration::Unknown;
    }
}

bool hasDualReturns(OusterRevision revision)
{
    return revision == OusterRevision::Rev06 ||
           revision == OusterRevision::Rev062 ||
           revision == OusterRevision::Rev07 ||
           revision == OusterRevision::Rev071 ||
           revision == OusterRevision::Rev08;
}

OusterRevision revisionFromPartNumber(const std::string & product_part_number)
{
    std::string part_number = product_part_number;
    std::transform(part_number.begin(), part_number.end(), part_number.begin(),
                   [](unsigned char c) {
                       return static_cast<char>(std::toupper(c));
                   });

    // Current part numbers carry 070/071/080 as their second group.
    if (part_number.find("-080-") != std::string::npos) {
        return OusterRevision::Rev08;
    }
    if (part_number.find("-071-") != std::string::npos) {
        return OusterRevision::Rev071;
    }
    if (part_number.find("-070-") != std::string::npos) {
        return OusterRevision::Rev07;
    }

    // Historical 840/860 part numbers carry the revision as their suffix.
    if (endsWith(part_number, "-06.2") || endsWith(part_number, "-062")) {
        return OusterRevision::Rev062;
    }

    // Some exported metadata omits the final separator (for example
    // 840105010C). Require a digit before a bare revision letter so ordinary
    // descriptive strings such as "synthetic" cannot be misclassified.
    const bool compact_c =
        part_number.size() >= 2 && endsWith(part_number, "C") &&
        std::isdigit(static_cast<unsigned char>(
            part_number[part_number.size() - 2]));
    const bool compact_d =
        part_number.size() >= 2 && endsWith(part_number, "D") &&
        std::isdigit(static_cast<unsigned char>(
            part_number[part_number.size() - 2]));
    if (endsWith(part_number, "-C") || compact_c) {
        return OusterRevision::RevC;
    }
    if (endsWith(part_number, "-D") || compact_d) {
        return OusterRevision::RevD;
    }
    if (endsWith(part_number, "-05")) return OusterRevision::Rev05;
    if (endsWith(part_number, "-06")) return OusterRevision::Rev06;
    if (endsWith(part_number, "-07")) return OusterRevision::Rev07;
    if (endsWith(part_number, "-08")) return OusterRevision::Rev08;
    return OusterRevision::Unknown;
}

OusterRevision fallbackRevision(OusterModel model)
{
    if (model == OusterModel::OSDome) return OusterRevision::Rev07;
    if (model == OusterModel::OS1Max) return OusterRevision::Rev08;
    return OusterRevision::Rev06;
}

void setLegacyModelPhysics(OusterProductProfile & profile)
{
    profile.minimum_range_m = 0.3;
    profile.range_resolution_m = 0.001;
    profile.lambertian_accuracy_m = 0.03;
    profile.retroreflector_accuracy_m = 0.10;

    switch (profile.model) {
    case OusterModel::OS0:
        profile.detection_range_10_d90_m = 15.0;
        profile.detection_range_80_d90_m = 45.0;
        profile.detection_range_10_d50_m = 20.0;
        profile.detection_range_80_d50_m = 50.0;
        profile.representable_range_m = 270.0;
        profile.precision_min_std_m = 0.010;
        profile.precision_max_std_m = 0.050;
        profile.beam_diameter_m = 0.005;
        profile.beam_divergence_fwhm_deg = 0.35;
        break;
    case OusterModel::OS1:
        profile.detection_range_10_d90_m = 45.0;
        profile.detection_range_80_d90_m = 100.0;
        profile.detection_range_10_d50_m = 55.0;
        profile.detection_range_80_d50_m = 120.0;
        profile.representable_range_m = 270.0;
        profile.precision_min_std_m = 0.007;
        profile.precision_max_std_m = 0.050;
        profile.beam_diameter_m = 0.0095;
        profile.beam_divergence_fwhm_deg = 0.18;
        break;
    case OusterModel::OS2:
        profile.detection_range_10_d90_m = 80.0;
        profile.detection_range_80_d90_m = 210.0;
        profile.detection_range_10_d50_m = 100.0;
        profile.detection_range_80_d50_m = 240.0;
        profile.representable_range_m = 465.0;
        profile.minimum_range_m = 1.0;
        profile.precision_min_std_m = 0.025;
        profile.precision_max_std_m = 0.080;
        profile.beam_diameter_m = 0.019;
        profile.beam_divergence_fwhm_deg = 0.09;
        // Historical OS2 specifications use 2048x10, unlike OS0/OS1.
        profile.reference_columns_per_second = 20480.0;
        break;
    default:
        break;
    }
}

void setRev7Physics(OusterProductProfile & profile)
{
    profile.minimum_range_m = 0.5;
    profile.range_resolution_m = 0.001;
    profile.lambertian_accuracy_m = 0.025;
    profile.retroreflector_accuracy_m = 0.05;
    profile.max_returns = 2;

    switch (profile.model) {
    case OusterModel::OS0:
        profile.detection_range_10_d90_m = 35.0;
        profile.detection_range_80_d90_m = 75.0;
        profile.representable_range_m = 233.0;
        profile.precision_min_std_m = 0.008;
        profile.precision_max_std_m = 0.040;
        profile.beam_diameter_m = 0.005;
        profile.beam_divergence_fwhm_deg = 0.35;
        break;
    case OusterModel::OS1:
        profile.detection_range_10_d90_m = 90.0;
        profile.detection_range_80_d90_m = 170.0;
        profile.representable_range_m = 233.0;
        profile.precision_min_std_m = 0.005;
        profile.precision_max_std_m = 0.030;
        profile.beam_diameter_m = 0.0095;
        profile.beam_divergence_fwhm_deg = 0.18;
        break;
    case OusterModel::OS2:
        profile.detection_range_10_d90_m = 200.0;
        profile.detection_range_80_d90_m = 350.0;
        profile.representable_range_m = 404.0;
        profile.minimum_range_m = 0.8;
        profile.precision_min_std_m = 0.020;
        profile.precision_max_std_m = 0.100;
        profile.beam_diameter_m = 0.019;
        profile.beam_divergence_fwhm_deg = 0.09;
        profile.reference_columns_per_second = 20480.0;
        break;
    case OusterModel::OSDome:
        profile.detection_range_10_d90_m = 20.0;
        profile.detection_range_80_d90_m = 45.0;
        profile.representable_range_m = 233.0;
        profile.precision_min_std_m = 0.010;
        profile.precision_max_std_m = 0.100;
        profile.beam_diameter_m = 0.005;
        profile.beam_divergence_fwhm_deg = 0.35;
        break;
    default:
        break;
    }
}

void setRev8Physics(OusterProductProfile & profile)
{
    profile.minimum_range_m = 0.5;
    profile.range_resolution_m = 0.001;
    profile.representable_range_m = 500.0;
    profile.precision_min_std_m = 0.0025;
    profile.precision_max_std_m = 0.015;
    profile.lambertian_accuracy_m = 0.0125;
    profile.retroreflector_accuracy_m = 0.025;
    profile.max_returns = 2;

    switch (profile.model) {
    case OusterModel::OS0:
        profile.detection_range_10_d90_m = 35.0;
        profile.detection_range_80_d90_m = 75.0;
        profile.beam_diameter_m = 0.005;
        profile.beam_divergence_fwhm_deg = 0.35;
        break;
    case OusterModel::OS1:
        profile.detection_range_10_d90_m = 90.0;
        profile.detection_range_80_d90_m = 170.0;
        profile.beam_diameter_m = 0.0095;
        profile.beam_divergence_fwhm_deg = 0.18;
        break;
    case OusterModel::OSDome:
        profile.detection_range_10_d90_m = 20.0;
        profile.detection_range_80_d90_m = 45.0;
        profile.precision_min_std_m = 0.005;
        profile.precision_max_std_m = 0.050;
        profile.beam_diameter_m = 0.005;
        profile.beam_divergence_fwhm_deg = 0.35;
        break;
    case OusterModel::OS1Max:
        profile.detection_range_10_d90_m = 200.0;
        profile.detection_range_80_d90_m = 350.0;
        profile.beam_diameter_m = 0.019;
        profile.beam_divergence_fwhm_deg = 0.09;
        break;
    default:
        break;
    }
}

bool compatible(OusterModel model, OusterRevision revision)
{
    switch (revision) {
    case OusterRevision::Gen1:
        return model == OusterModel::OS1;
    case OusterRevision::RevC:
    case OusterRevision::RevD:
    case OusterRevision::Rev05:
    case OusterRevision::Rev06:
    case OusterRevision::Rev062:
        return model == OusterModel::OS0 || model == OusterModel::OS1 ||
               model == OusterModel::OS2;
    case OusterRevision::Rev07:
        return model == OusterModel::OS0 || model == OusterModel::OS1 ||
               model == OusterModel::OS2 || model == OusterModel::OSDome;
    case OusterRevision::Rev071:
        // Rev7.1 was a reliability update to the L3 family. OS2 remained on
        // Rev7.0 / firmware 2.5.
        return model == OusterModel::OS0 || model == OusterModel::OS1 ||
               model == OusterModel::OSDome;
    case OusterRevision::Rev08:
        return model == OusterModel::OS0 || model == OusterModel::OS1 ||
               model == OusterModel::OSDome || model == OusterModel::OS1Max;
    default:
        return false;
    }
}

}  // namespace

OusterModel parseOusterModel(const std::string & product_line)
{
    const std::string value = canonical(product_line);
    if (value.find("OS1MAX") != std::string::npos) return OusterModel::OS1Max;
    if (value.find("OSDOME") != std::string::npos) return OusterModel::OSDome;
    if (value.find("OS0") != std::string::npos) return OusterModel::OS0;
    if (value.find("OS1") != std::string::npos) return OusterModel::OS1;
    if (value.find("OS2") != std::string::npos) return OusterModel::OS2;
    return OusterModel::Unknown;
}

OusterRevision parseOusterRevision(const std::string & value)
{
    const std::string revision = canonical(value);
    if (revision.empty() || revision == "AUTO") return OusterRevision::Auto;
    if (revision == "GEN1" || revision == "REVGEN1") {
        return OusterRevision::Gen1;
    }
    if (revision == "C" || revision == "REVC") return OusterRevision::RevC;
    if (revision == "D" || revision == "REVD") return OusterRevision::RevD;
    if (revision == "05" || revision == "5" || revision == "REV05" ||
        revision == "REV5") {
        return OusterRevision::Rev05;
    }
    if (revision == "06" || revision == "6" || revision == "REV06" ||
        revision == "REV6") {
        return OusterRevision::Rev06;
    }
    if (revision == "062" || revision == "62" || revision == "REV062" ||
        revision == "REV62") {
        return OusterRevision::Rev062;
    }
    if (revision == "07" || revision == "7" || revision == "REV07" ||
        revision == "REV7") {
        return OusterRevision::Rev07;
    }
    if (revision == "071" || revision == "71" || revision == "REV071" ||
        revision == "REV71") {
        return OusterRevision::Rev071;
    }
    if (revision == "08" || revision == "8" || revision == "REV08" ||
        revision == "REV8") {
        return OusterRevision::Rev08;
    }
    return OusterRevision::Unknown;
}

const char * toString(OusterModel model) noexcept
{
    switch (model) {
    case OusterModel::OS0: return "OS0";
    case OusterModel::OS1: return "OS1";
    case OusterModel::OS2: return "OS2";
    case OusterModel::OSDome: return "OSDome";
    case OusterModel::OS1Max: return "OS1 MAX";
    default: return "unknown";
    }
}

const char * toString(OusterGeneration generation) noexcept
{
    switch (generation) {
    case OusterGeneration::Gen1: return "Gen1";
    case OusterGeneration::Gen2: return "Gen2";
    case OusterGeneration::Gen3: return "Gen3/L3";
    case OusterGeneration::Gen4: return "Gen4/L4";
    default: return "unknown";
    }
}

const char * toString(OusterRevision revision) noexcept
{
    switch (revision) {
    case OusterRevision::Auto: return "auto";
    case OusterRevision::Gen1: return "gen1";
    case OusterRevision::RevC: return "revC";
    case OusterRevision::RevD: return "revD";
    case OusterRevision::Rev05: return "rev05";
    case OusterRevision::Rev06: return "rev06";
    case OusterRevision::Rev062: return "rev06.2";
    case OusterRevision::Rev07: return "rev07";
    case OusterRevision::Rev071: return "rev07.1";
    case OusterRevision::Rev08: return "rev08";
    default: return "unknown";
    }
}

OusterProductProfile resolveOusterProductProfile(
    const OusterProductProfileRequest & request)
{
    OusterProductProfile profile;
    profile.model = parseOusterModel(request.product_line);

    const OusterRevision requested =
        parseOusterRevision(request.hardware_revision);
    if (requested != OusterRevision::Auto &&
        requested != OusterRevision::Unknown) {
        profile.revision = requested;
    } else {
        profile.revision =
            revisionFromPartNumber(request.product_part_number);
        profile.revision_inferred =
            profile.revision != OusterRevision::Unknown;

        // Product/firmware deductions that are unambiguous without a PN.
        if (profile.revision == OusterRevision::Unknown &&
            request.firmware_major >= 4 &&
            (profile.model == OusterModel::OS0 ||
             profile.model == OusterModel::OS1 ||
             profile.model == OusterModel::OSDome ||
             profile.model == OusterModel::OS1Max)) {
            profile.revision = OusterRevision::Rev08;
            profile.revision_inferred = true;
        } else if (profile.revision == OusterRevision::Unknown &&
                   profile.model == OusterModel::OS1Max) {
            profile.revision = OusterRevision::Rev08;
            profile.revision_inferred = true;
        } else if (profile.revision == OusterRevision::Unknown &&
                   profile.model == OusterModel::OS1 &&
                   (request.beam_count == 16 ||
                    request.firmware_major == 1)) {
            profile.revision = OusterRevision::Gen1;
            profile.revision_inferred = true;
        } else if (profile.revision == OusterRevision::Unknown &&
                   request.firmware_major >= 3 &&
                   (profile.model == OusterModel::OS0 ||
                    profile.model == OusterModel::OS1 ||
                    profile.model == OusterModel::OS2 ||
                    profile.model == OusterModel::OSDome)) {
            profile.revision = OusterRevision::Rev07;
            profile.revision_inferred = true;
        }
    }

    if (profile.revision == OusterRevision::Unknown ||
        requested == OusterRevision::Unknown) {
        profile.revision = fallbackRevision(profile.model);
        profile.fallback_revision = true;
    }
    profile.generation = generationOf(profile.revision);
    profile.max_returns = hasDualReturns(profile.revision) ? 2 : 1;
    profile.supported = compatible(profile.model, profile.revision);

    // Keep an incompatible selection recognizable instead of silently
    // coercing it into another product (for example OS2 Rev8).
    if (!profile.supported) {
        profile.id = std::string(toString(profile.model)) + "-" +
                     toString(profile.revision);
        return profile;
    }

    if (profile.revision == OusterRevision::Gen1) {
        profile.minimum_range_m = 0.8;
        profile.range_resolution_m = 0.003;
        profile.representable_range_m = 200.0;
        profile.beam_diameter_m = 0.010;
        profile.beam_divergence_fwhm_deg = 0.13;
        profile.max_returns = 1;
        if (request.firmware_major <= 1 && request.firmware_major != 0) {
            profile.detection_range_10_d90_m = 40.0;
            profile.detection_range_80_d90_m = 105.0;
            profile.detection_range_10_d50_m = 60.0;
            profile.detection_range_80_d50_m = 120.0;
            profile.precision_min_std_m = 0.015;
            profile.precision_max_std_m = 0.100;
        } else {
            profile.detection_range_10_d90_m = 50.0;
            profile.detection_range_80_d90_m = 110.0;
            profile.detection_range_10_d50_m = 65.0;
            profile.detection_range_80_d50_m = 150.0;
            profile.precision_min_std_m = 0.010;
            profile.precision_max_std_m = 0.050;
        }
        profile.lambertian_accuracy_m = 0.05;
        profile.retroreflector_accuracy_m = 0.10;
    } else if (profile.revision == OusterRevision::Rev07 ||
               profile.revision == OusterRevision::Rev071) {
        setRev7Physics(profile);
    } else if (profile.revision == OusterRevision::Rev08) {
        setRev8Physics(profile);
    } else {
        setLegacyModelPhysics(profile);
        // Configurable profiles gained 1 mm range resolution in firmware 2.4;
        // older packet generations retain 3 mm physical resolution.
        if (request.firmware_major < 2 ||
            (request.firmware_major == 2 && request.firmware_minor < 4)) {
            profile.range_resolution_m = 0.003;
        }
    }

    if (request.low_data_profile) {
        profile.range_resolution_m = 0.008;
        profile.representable_range_m = std::min(
            profile.representable_range_m,
            kRng15MaximumRepresentableRangeM);
    }

    profile.id = std::string(toString(profile.model)) + "-" +
                 toString(profile.revision);
    return profile;
}

double ousterModeRangeScale(const OusterProductProfile & profile,
                            int columns_per_frame, double lidar_hz)
{
    if (profile.reference_columns_per_second <= 0.0 ||
        columns_per_frame <= 0 || lidar_hz <= 0.0) {
        return 1.0;
    }
    const double actual = static_cast<double>(columns_per_frame) * lidar_hz;
    const double octaves = std::clamp(
        std::log2(profile.reference_columns_per_second / actual), -2.0, 2.0);
    return std::pow(1.19, octaves);
}

double ousterModePrecisionScale(const OusterProductProfile & profile,
                                int columns_per_frame, double lidar_hz)
{
    if (profile.reference_columns_per_second <= 0.0 ||
        columns_per_frame <= 0 || lidar_hz <= 0.0) {
        return 1.0;
    }
    const double actual = static_cast<double>(columns_per_frame) * lidar_hz;
    const double octaves = std::clamp(
        std::log2(actual / profile.reference_columns_per_second), -2.0, 2.0);
    return std::pow(std::sqrt(2.0), octaves);
}

}  // namespace ouster_sim_core
