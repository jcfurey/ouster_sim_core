# Standalone test superproject for ouster_sim_core.
#
# It builds the Ouster SDK client from a source checkout and the core with its
# tests, exactly as an embedding package would:
#
#   cmake -S test/standalone -B build -DOUSTER_SDK_SOURCE_DIR=/path/to/ouster-sdk
#   cmake --build build && ctest --test-dir build/core --output-on-failure
#
# Setting OUSTER_SIM_CORE_SDK_TEST_DATA_DIR=<ouster-sdk>/tests also runs the
# physical-capture wire conformance suite.
cmake_minimum_required(VERSION 3.16)
project(ouster_sim_core_standalone LANGUAGES C CXX)

set(OUSTER_SDK_SOURCE_DIR "" CACHE PATH "Ouster SDK source checkout")
if(NOT OUSTER_SDK_SOURCE_DIR)
  message(FATAL_ERROR "Set OUSTER_SDK_SOURCE_DIR to an Ouster SDK checkout")
endif()

foreach(_option BUILD_SENSOR BUILD_PCAP BUILD_OSF BUILD_VIZ BUILD_MAPPING
                BUILD_PYTHON_MODULE BUILD_EXAMPLES BUILD_SHARED_LIBRARY
                BUILD_TESTING)
  set(${_option} OFF CACHE BOOL "" FORCE)
endforeach()
add_subdirectory("${OUSTER_SDK_SOURCE_DIR}" ouster-sdk EXCLUDE_FROM_ALL)

set(OUSTER_SIM_CORE_OUSTER_SDK_TARGET OusterSDK::ouster_client)
set(OUSTER_SIM_CORE_BUILD_TESTING ON CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../.." core)
