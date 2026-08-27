# ouster_sim_core

Plain-CMake, simulator-neutral infrastructure for Ouster lidar simulation.

The canonical repository is
[github.com/jcfurey/ouster_sim_core](https://github.com/jcfurey/ouster_sim_core).
Simulator repositories embed and pin it as a Git submodule; it is deliberately
not a ROS package and contains no `package.xml`. Its `COLCON_IGNORE` marker
prevents a recursive ROS workspace scan from misclassifying the embedded
plain-CMake project as an independently buildable colcon package.

## Responsibilities

The core currently owns:

- calibrated half-open firing geometry and physics-step-independent scheduling;
- immutable revolution, measurement, ring, return, linear, and timing identity;
- Ouster SDK-backed metadata parsing and validation;
- Gen1 through Gen4 product/revision physics resolution without simulator
  dependencies;
- a normalized optical-return contract with separate physical path length and
  reported Ouster range;
- deterministic scalar detection, range-noise, signal, reflectivity, near-IR,
  dropout, and false-alarm behavior;
- stateless random draws keyed by sensor, epoch, firing identity, and effect;
- strict, transactional assembly of source returns across arbitrary batch
  segmentation;
- exact integer range/channel handoff and all-zero preservation for true
  misses;
- row-major Ouster image layout conversion;
- synchronous standard packet encoding, including headers and modern CRC;
- a pure, clock-injected packet-pacing policy; and
- simulator-neutral identity and full packet round-trip tests.

It never includes Gazebo, Ogre, AGX, CUDA, or simulator-specific types.
Publication lifecycle, clocks, threads, transport, and conversion from
simulator-native material/intensity outputs remain responsibilities of the
embedding package.

## Integration contract

The parent discovers a compatible Ouster SDK provider, creates a CMake target,
and names it in `OUSTER_SIM_CORE_OUSTER_SDK_TARGET` before adding this
directory:

```cmake
add_library(my_ouster_sdk INTERFACE)
target_link_libraries(my_ouster_sdk INTERFACE
  OusterSDK::ouster_client)

set(OUSTER_SIM_CORE_OUSTER_SDK_TARGET my_ouster_sdk)
add_subdirectory(third_party/ouster_sim_core)

target_link_libraries(my_simulator_target PRIVATE
  ouster_sim_core::ouster_sim_core)
```

The core target is static and position-independent. An embedding ROS package
owns dependency declarations, installation, and any parent-scoped shared
facade. Parents should not install an unscoped
`libouster_sim_core.so` into a merged workspace.

Public core headers hide Ouster SDK types. The SDK target is still required to
compile and link the implementation.

## Standalone test superproject

A small superproject can use an installed static Ouster SDK:

```cmake
cmake_minimum_required(VERSION 3.16)
project(ouster_sim_core_checks LANGUAGES CXX)

find_package(OusterSDK REQUIRED COMPONENTS Static)
set(OUSTER_SIM_CORE_OUSTER_SDK_TARGET OusterSDK::ouster_client)
set(OUSTER_SIM_CORE_BUILD_TESTING ON)
add_subdirectory(path/to/ouster_sim_core)
```

Then configure, build, and run `ctest --output-on-failure`. The tested
provider revision is Ouster SDK v0.16.2 commit
`401c647844b801784535e5a5ae2c2962d4f85cd1`.

## Data contract

Dispatch order is column-major:

```text
linear_index = measurement_id * pixels_per_column + ring_id
```

Packet image storage is row-major:

```text
image_index = ring_id * columns_per_frame + measurement_id
```

The assembler performs this conversion only after validating the exact source
identity stream. It rejects gaps, duplicates, reordering, changed time offsets,
secondary returns in a primary-only stream, zero-range hits, and nonzero miss
fields. Channel outputs already carry exact integer `range_mm`; the assembler
does not perform a second floating-point conversion.

The optical channel model accepts explicit presence for apparent 865 nm
reflectance and passive near-IR. A present zero is a real black response, not a
request for fallback material behavior. Its stateless random key includes the
seed, sensor stream, epoch, revolution, measurement, ring, return, effect, and
subdraw, so call order and physics-batch partitioning cannot change a sample.
Physical `path_length_m` drives received-signal attenuation, while
`reported_range_m` drives detection, noise, quantization, and the packet range.
They may differ even for a direct return when the simulator casts from the
translated beam origin but Ouster's packet convention reports the
lidar-origin-equivalent range; transmitted and ghost paths may separate them
further.
The integer mixer and open-interval uniform mapping are bit-exact contract
surfaces. Gaussian and noisy-channel goldens are frozen for the supported
toolchain; different `libm` implementations are compared with documented
numeric tolerances. Optional incident angle is retained for later calibration
without applying incidence twice. Spatial edge suppression is intentionally
deferred until a complete-frame API can derive canonical neighborhoods inside
the core.

Metadata exposes the active UDP lidar profile, return count, and exact RANGE
value mask without exposing SDK types. The current encoder rejects dual-return
profiles at construction and rejects any range that the selected wire layout
cannot represent exactly. In particular, RNG15 values must be no greater than
262,136 mm and aligned to 8 mm, preventing the SDK writer from silently masking
an excessive range into a plausible near hit.

The packet-pacing policy computes absolute delivery deadlines from an injected
monotonic clock. It follows 80% of the observed producer cadence, falls back to
the nominal period after startup/pause/reset, and shifts deadlines after resume
without owning a thread or publishing API.

`OusterMetadata::sourceJson()` preserves the input bytes for audit.
`publishedJson()` normally returns the same JSON; when a modern packet
profile contains the WINDOW field but legacy firmware text would make the
supported decoder discard it, only the published firmware advertisement is
raised to the layout-compatible minimum.

The product catalog covers OS0, OS1, OS2, OSDome, and OS1 MAX across Gen1
through Gen4. The current packet conformance fixture is still calibrated
OS1-64 metadata with a 1024 x 64 frame and 16 columns per packet; additional
metadata and packet fixtures remain required before claiming broad wire-level
conformance.
