# Smoke test for an installed OCCT tree: a box minus a cylinder, exported to STEP via XCAF.
#   cmake -S native/smoke -B .deps/build/smoke-<rid> -G Ninja -DCMAKE_PREFIX_PATH=.deps/install/<rid>
cmake_minimum_required(VERSION 3.24)
project(occt_smoke CXX)

set(CMAKE_CXX_STANDARD 17)
# OCCT's package config only accepts an exact version, so check the 8.0 series here instead.
find_package(OpenCASCADE REQUIRED CONFIG)
if(NOT OpenCASCADE_VERSION VERSION_GREATER_EQUAL 8.0 OR OpenCASCADE_VERSION VERSION_GREATER_EQUAL 8.1)
  message(FATAL_ERROR "OCCT 8.0.x required, found ${OpenCASCADE_VERSION}")
endif()

add_executable(occt_smoke main.cpp)
if(MSVC)
  # CMake leaves the exception model unset here; OCCT itself is built with /EHa
  target_compile_options(occt_smoke PRIVATE /EHa)
endif()
target_link_libraries(occt_smoke PRIVATE
  TKernel TKMath TKBRep TKPrim TKBO TKTopAlgo TKGeomBase
  TKLCAF TKXCAF TKDE TKXSBase TKDESTEP)
