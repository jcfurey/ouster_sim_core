# ouster_sim_core

Plain-CMake, simulator-neutral infrastructure for Ouster lidar simulation.

The canonical repository is
[github.com/jcfurey/ouster_sim_core](https://github.com/jcfurey/ouster_sim_core).
Simulator repositories embed and pin it as a Git submodule; it is deliberately
not a ROS package and contains no `package.xml`.

## Responsibilities

The core currently owns:

- calibrated half-open firing geometry and physics-step-independent scheduling;
- immutable revolution, measurement, ring, return, linear, and timing identity;
- Ouster SDK-backed metadata parsing and validation;
- strict, transactional assembly of source returns across arbitrary batch
  segmentation;
- zero-range preservation for true misses;
- row-major Ouster image layout conversion;
- synchronous standard packet encoding, including headers and modern CRC; and
- simulator-neutral identity and full packet round-trip tests.

It never includes Gazebo, Ogre, AGX, CUDA, or simulator-specific types.
Publication lifecycle, transport, pacing, and simulator-native optical inputs
remain responsibilities of the embedding package.

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
secondary returns in a primary-only stream, nonzero miss fields, and invalid
hit ranges.

`OusterMetadata::sourceJson()` preserves the input bytes for audit.
`publishedJson()` normally returns the same JSON; when a modern packet
profile contains the WINDOW field but legacy firmware text would make the
supported decoder discard it, only the published firmware advertisement is
raised to the layout-compatible minimum.

The current conformance fixture is calibrated OS1-64 metadata with a 1024 x 64
frame and 16 columns per packet. Additional product/profile fixtures are
required before claiming broader profile support.
