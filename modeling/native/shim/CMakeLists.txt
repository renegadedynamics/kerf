# b3docct: the extern "C" shim over OCCT that Build123d.Occt P/Invokes.
#   cmake -S native/shim -B .deps/build/shim-<rid> -G Ninja -DCMAKE_BUILD_TYPE=Release
#         -DCMAKE_PREFIX_PATH=.deps/install/<rid> -DCMAKE_INSTALL_PREFIX=native/out/<rid>
#   cmake --build .deps/build/shim-<rid> --target install
cmake_minimum_required(VERSION 3.24)
project(b3docct CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# OCCT's package config only accepts an exact version, so check the 8.0 series here instead.
find_package(OpenCASCADE REQUIRED CONFIG)
if(NOT OpenCASCADE_VERSION VERSION_GREATER_EQUAL 8.0 OR OpenCASCADE_VERSION VERSION_GREATER_EQUAL 8.1)
  message(FATAL_ERROR "OCCT 8.0.x required, found ${OpenCASCADE_VERSION}")
endif()

file(GLOB GENERATED CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generated/*.cpp)
add_library(b3docct SHARED src/b3d_shim.cpp src/b3d_extras.cpp src/b3d_mesh.cpp ${GENERATED})
target_include_directories(b3docct PRIVATE include)
# Link every OCCT toolkit; the linker drops the ones nothing references
target_link_libraries(b3docct PRIVATE ${OpenCASCADE_LIBRARIES})

if(MSVC)
  # Match OCCT's own exception model; /bigobj for the larger generated sources
  target_compile_options(b3docct PRIVATE /EHa /bigobj /W3)
else()
  target_compile_options(b3docct PRIVATE -Wall)
  # Find the OCCT libraries next to the shim at run time
  set_target_properties(b3docct PROPERTIES INSTALL_RPATH "$<IF:$<PLATFORM_ID:Darwin>,@loader_path,$ORIGIN>")
endif()

install(TARGETS b3docct RUNTIME DESTINATION bin LIBRARY DESTINATION lib)
