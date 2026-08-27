// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ouster_sim_core/firing_table.hpp"

#include <cstdint>

namespace ouster_sim_core {

/// Simulator-normalized primary return ready for strict revolution assembly.
///
/// Every channel is already expressed in the exact unsigned-integer domain
/// written to an Ouster packet. `is_hit` means that the sample is a reportable
/// lidar return; it may represent a physical surface, participating medium, or
/// a deliberately synthesized false alarm. A miss has all fields set to zero.
struct OusterReturnSample {
    OusterFiringIdentity identity;
    std::uint32_t range_mm = 0;
    std::uint16_t signal = 0;
    std::uint8_t reflectivity = 0;
    std::uint16_t near_ir = 0;
    bool is_hit = false;
};

}  // namespace ouster_sim_core
