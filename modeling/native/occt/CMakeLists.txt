# Superbuild for the native third-party stack: FreeType, then OCCT.
#
#   cmake -S native/occt -B .deps/build/<rid> -G Ninja -DCMAKE_BUILD_TYPE=Release
#         -DCMAKE_INSTALL_PREFIX=.deps/install/<rid>
#   cmake --build .deps/build/<rid>
#
# Everything is installed into CMAKE_INSTALL_PREFIX, which the shim build then consumes.
cmake_minimum_required(VERSION 3.24)
project(build123d_native_deps NONE)

include(ExternalProject)

set(OCCT_GIT_TAG "V8_0_1" CACHE STRING "OCCT release tag")
set(FREETYPE_GIT_TAG "VER-2-14-3" CACHE STRING "FreeType release tag")
set(OCCT_SOURCE_DIR "" CACHE PATH "Existing OCCT checkout (skips the git download when set)")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(DEPS_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Settings every sub-build shares
set(COMMON_ARGS
  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  -DCMAKE_INSTALL_PREFIX=${DEPS_PREFIX}
  -DCMAKE_PREFIX_PATH=${DEPS_PREFIX}
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
)
if(APPLE)
  list(APPEND COMMON_ARGS
    -DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}
    -DCMAKE_OSX_DEPLOYMENT_TARGET=${CMAKE_OSX_DEPLOYMENT_TARGET})
endif()

# FreeType: needed by TKService, which XCAF (STEP with colours/names) depends on.
# Shared, so OCCT's Windows lookup finds a DLL; every optional codec is disabled.
ExternalProject_Add(freetype
  GIT_REPOSITORY https://github.com/freetype/freetype.git
  GIT_TAG        ${FREETYPE_GIT_TAG}
  GIT_SHALLOW    TRUE
  CMAKE_ARGS
    ${COMMON_ARGS}
    -DBUILD_SHARED_LIBS=ON
    -DFT_DISABLE_ZLIB=ON
    -DFT_DISABLE_BZIP2=ON
    -DFT_DISABLE_PNG=ON
    -DFT_DISABLE_HARFBUZZ=ON
    -DFT_DISABLE_BROTLI=ON
)

if(OCCT_SOURCE_DIR)
  set(OCCT_SOURCE_ARGS SOURCE_DIR ${OCCT_SOURCE_DIR} DOWNLOAD_COMMAND "")
else()
  set(OCCT_SOURCE_ARGS
    GIT_REPOSITORY https://github.com/Open-Cascade-SAS/OCCT.git
    GIT_TAG        ${OCCT_GIT_TAG}
    GIT_SHALLOW    TRUE)
endif()

# OCCT: modelling, data exchange and the OCAF/XCAF framework. No GUI pieces
# (OpenGL, X11, Tcl/Tk, Draw), so the build is headless on every platform.
# Release exceptions stay enabled so that OCCT failures surface as .NET exceptions
# instead of undefined behaviour.
ExternalProject_Add(occt
  DEPENDS freetype
  ${OCCT_SOURCE_ARGS}
  CMAKE_ARGS
    ${COMMON_ARGS}
    -DINSTALL_DIR=${DEPS_PREFIX}
    -DINSTALL_DIR_LAYOUT=Unix
    -DBUILD_LIBRARY_TYPE=Shared
    -DBUILD_CPP_STANDARD=C++17
    -DBUILD_RELEASE_DISABLE_EXCEPTIONS=OFF
    -DBUILD_MODULE_FoundationClasses=ON
    -DBUILD_MODULE_ModelingData=ON
    -DBUILD_MODULE_ModelingAlgorithms=ON
    -DBUILD_MODULE_Visualization=ON
    -DBUILD_MODULE_ApplicationFramework=ON
    -DBUILD_MODULE_DataExchange=ON
    -DBUILD_MODULE_Draw=OFF
    -DBUILD_DOC_Overview=OFF
    -DBUILD_SAMPLES_QT=OFF
    -DBUILD_GTEST=OFF
    -DUSE_FREETYPE=ON
    -D3RDPARTY_FREETYPE_DIR=${DEPS_PREFIX}
    -DUSE_OPENGL=OFF
    -DUSE_GLES2=OFF
    -DUSE_D3D=OFF
    -DUSE_XLIB=OFF
    -DUSE_TK=OFF
    -DUSE_TBB=OFF
    -DUSE_VTK=OFF
    -DUSE_FREEIMAGE=OFF
    -DUSE_FFMPEG=OFF
    -DUSE_OPENVR=OFF
    -DUSE_RAPIDJSON=OFF
    -DUSE_DRACO=OFF
)
