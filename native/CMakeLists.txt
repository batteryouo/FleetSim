cmake_minimum_required(VERSION 3.25)
project(CadenceNative LANGUAGES CXX)

include(FetchContent)
set(GODOTCPP_TARGET template_release CACHE STRING "Godot binding target")
set(GODOTCPP_PRECISION single CACHE STRING "Match the engine's real_t precision")
set(GODOTCPP_BUILD_PROFILE "${CMAKE_CURRENT_SOURCE_DIR}/build_profile.json" CACHE FILEPATH "Binding profile")
set(GODOTCPP_USE_HOT_RELOAD OFF CACHE BOOL "Disable extension hot reload")
set(GODOTCPP_SYSTEM_HEADERS ON CACHE BOOL "Treat binding headers as system headers")
FetchContent_Declare(godot_cpp
  URL https://codeload.github.com/godotengine/godot-cpp/zip/e83fd0904c13356ed1d4c3d09f8bb9132bdc6b77
  URL_HASH SHA256=579af30c5f62c1084edb28216788188227d80a583f0114463699ac4edd22140f
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
)
FetchContent_MakeAvailable(godot_cpp)

add_library(cadence_kinematics SHARED
  src/kinematics.cpp
  src/godot_kinematics.cpp
  src/godot_lidar.cpp
  src/register_types.cpp
)
target_link_libraries(cadence_kinematics PRIVATE godot::cpp)
target_include_directories(cadence_kinematics PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../sim/bridge")
target_compile_features(cadence_kinematics PRIVATE cxx_std_17)
if(MSVC)
  target_compile_options(cadence_kinematics PRIVATE /W4 /fp:strict)
else()
  target_compile_options(cadence_kinematics PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math -ffp-contract=off)
endif()
get_filename_component(NATIVE_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/../sim/native/bin" ABSOLUTE)
set_target_properties(cadence_kinematics PROPERTIES
  CXX_EXTENSIONS OFF
  PREFIX ""
  RUNTIME_OUTPUT_DIRECTORY "$<1:${NATIVE_OUTPUT}>"
  LIBRARY_OUTPUT_DIRECTORY "$<1:${NATIVE_OUTPUT}>"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
