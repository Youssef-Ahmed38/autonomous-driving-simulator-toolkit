cmake_minimum_required(VERSION 3.24)
project(autonomous_driving_simulator_toolkit VERSION 0.2.0 LANGUAGES CXX)

option(ADSIM_BUILD_TESTS "Build correctness tests" ON)
option(ADSIM_BUILD_BENCHMARKS "Build planner/fusion benchmark" ON)

add_library(autonomous_driving_simulator_toolkit
  src/bridge.cpp
  src/driving_stack.cpp
  src/planner.cpp
  src/risk.cpp
  src/sensor_fusion.cpp
  src/simulator.cpp
  src/vehicle_model.cpp
)
target_include_directories(autonomous_driving_simulator_toolkit PUBLIC include)
target_compile_features(autonomous_driving_simulator_toolkit PUBLIC cxx_std_20)
if(MSVC)
  target_compile_options(autonomous_driving_simulator_toolkit PRIVATE /W4 /permissive-)
else()
  target_compile_options(autonomous_driving_simulator_toolkit PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(adsim app/main.cpp)
target_link_libraries(adsim PRIVATE autonomous_driving_simulator_toolkit)

if(ADSIM_BUILD_TESTS)
  enable_testing()
  add_executable(adsim_tests tests/test_driving.cpp)
  target_link_libraries(adsim_tests PRIVATE autonomous_driving_simulator_toolkit)
  add_test(NAME adsim_tests COMMAND adsim_tests)
  add_test(NAME adsim_demo_smoke COMMAND adsim demo all)
endif()

if(ADSIM_BUILD_BENCHMARKS)
  add_executable(adsim_benchmark benchmarks/benchmark_driving.cpp)
  target_link_libraries(adsim_benchmark PRIVATE autonomous_driving_simulator_toolkit)
endif()
