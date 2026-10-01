cmake_minimum_required(VERSION 3.16)
project(ouster_sim_core VERSION 0.4.0 LANGUAGES CXX)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT OUSTER_SIM_CORE_OUSTER_SDK_TARGET)
  message(FATAL_ERROR
    "ouster_sim_core is embedded infrastructure. The outer package must "
    "provide OUSTER_SIM_CORE_OUSTER_SDK_TARGET after discovering the Ouster "
    "SDK or ouster_ros dependency.")
endif()
if(NOT TARGET ${OUSTER_SIM_CORE_OUSTER_SDK_TARGET})
  message(FATAL_ERROR
    "OUSTER_SIM_CORE_OUSTER_SDK_TARGET='${OUSTER_SIM_CORE_OUSTER_SDK_TARGET}' "
    "does not name a CMake target")
endif()

add_library(ouster_sim_core STATIC
  src/firing_table.cpp
  src/imu_packet_pipeline.cpp
  src/metadata.cpp
  src/optical_channel_model.cpp
  src/packet_encoder.cpp
  src/packet_pacing.cpp
  src/product_profile.cpp
  src/revolution_assembler.cpp)
add_library(ouster_sim_core::ouster_sim_core ALIAS ouster_sim_core)

set_target_properties(ouster_sim_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
target_compile_features(ouster_sim_core PUBLIC cxx_std_20)
target_compile_options(ouster_sim_core PRIVATE
  -Wall -Wextra -Wpedantic)
target_include_directories(ouster_sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(ouster_sim_core PUBLIC
  ${OUSTER_SIM_CORE_OUSTER_SDK_TARGET})

option(
  OUSTER_SIM_CORE_BUILD_TESTING
  "Build simulator-neutral core tests when configured standalone"
  OFF)
if(OUSTER_SIM_CORE_BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)

  add_executable(ouster_sim_core_test_revolution_assembler
    test/test_revolution_assembler.cpp)
  target_link_libraries(ouster_sim_core_test_revolution_assembler PRIVATE
    ouster_sim_core
    GTest::gtest_main)
  gtest_discover_tests(ouster_sim_core_test_revolution_assembler)

  add_executable(ouster_sim_core_test_optical_channel_model
    test/test_optical_channel_model.cpp)
  target_link_libraries(ouster_sim_core_test_optical_channel_model PRIVATE
    ouster_sim_core
    GTest::gtest_main)
  gtest_discover_tests(ouster_sim_core_test_optical_channel_model)

  add_executable(ouster_sim_core_test_packet_pacing
    test/test_packet_pacing.cpp)
  target_link_libraries(ouster_sim_core_test_packet_pacing PRIVATE
    ouster_sim_core
    GTest::gtest_main)
  gtest_discover_tests(ouster_sim_core_test_packet_pacing)

  add_executable(ouster_sim_core_test_product_profile
    test/test_product_profile.cpp)
  target_link_libraries(ouster_sim_core_test_product_profile PRIVATE
    ouster_sim_core
    GTest::gtest_main)
  gtest_discover_tests(ouster_sim_core_test_product_profile)

  add_executable(ouster_sim_core_test_metadata_product_profile
    test/test_metadata_product_profile.cpp)
  target_compile_definitions(
    ouster_sim_core_test_metadata_product_profile PRIVATE
    OUSTER_SIM_CORE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_link_libraries(
    ouster_sim_core_test_metadata_product_profile PRIVATE
    ouster_sim_core
    GTest::gtest_main)
  gtest_discover_tests(ouster_sim_core_test_metadata_product_profile)

  add_executable(ouster_sim_core_test_packet_roundtrip
    test/test_packet_roundtrip.cpp)
  target_compile_definitions(ouster_sim_core_test_packet_roundtrip PRIVATE
    OUSTER_SIM_CORE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_link_libraries(ouster_sim_core_test_packet_roundtrip PRIVATE
    ouster_sim_core
    GTest::gtest_main)
  gtest_discover_tests(ouster_sim_core_test_packet_roundtrip)

  add_executable(ouster_sim_core_test_imu_packet_pipeline
    test/test_imu_packet_pipeline.cpp)
  target_compile_definitions(ouster_sim_core_test_imu_packet_pipeline PRIVATE
    OUSTER_SIM_CORE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_link_libraries(ouster_sim_core_test_imu_packet_pipeline PRIVATE
    ouster_sim_core
    GTest::gtest_main)
  gtest_discover_tests(ouster_sim_core_test_imu_packet_pipeline)
endif()
