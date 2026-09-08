// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <limits>

#if defined(__CUDACC__) || defined(__HIPCC__) || defined(__HIP__)
#define OUSTER_SIM_CORE_HD __host__ __device__
#else
#define OUSTER_SIM_CORE_HD
#endif

namespace ouster_sim_core {

// Evaluate the standard-library constant outside device functions. CUDA can
// use this scalar constant without calling a host-only constexpr function.
template <typename Real>
inline constexpr Real kMaximumOpticalValue = std::numeric_limits<Real>::max();

/// Optical presence contract for dense CPU/GPU buffers. Zero is an authored
/// value. A null buffer or an invalid element means missing input at the
/// adapter boundary; the typed scalar model requires valid optional values.
/// Comparisons also reject NaN and infinity without device math dependencies.
template <typename Real>
OUSTER_SIM_CORE_HD inline bool opticalValuePresent(
    const Real * values, std::size_t index)
{
    return values && values[index] >= Real{0} &&
        values[index] <= kMaximumOpticalValue<Real>;
}

template <typename Real>
OUSTER_SIM_CORE_HD inline Real opticalValueOrDefault(
    const Real * values, std::size_t index, Real fallback)
{
    return opticalValuePresent(values, index) ? values[index] : fallback;
}

}  // namespace ouster_sim_core

#undef OUSTER_SIM_CORE_HD
