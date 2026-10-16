# Build the public player from its own source inventory, without starting a match.
cmake_minimum_required(VERSION 3.16)
project(R2DRLPlayerRegression LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(R2DRL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../R2DRL" ABSOLUTE)
set(LIBRCSC_INCLUDE_DIR "${R2DRL_ROOT}/librcsc")
set(LIBRCSC_LIB "${R2DRL_ROOT}/librcsc/rcsc/.libs/librcsc.so")
find_package(Boost REQUIRED COMPONENTS system)
find_package(ZLIB REQUIRED)

# The bundled top-level CMake configuration references release templates not
# included in this checkout. The player target itself is complete; reuse it.
set(PROJECT_SOURCE_DIR "${R2DRL_ROOT}/helios-base")
set(PROJECT_BINARY_DIR "${CMAKE_BINARY_DIR}")
add_subdirectory("${R2DRL_ROOT}/helios-base/src/player" player)
target_link_libraries(sample_player PRIVATE rt)
set_target_properties(sample_player PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/../../../librcsc/rcsc/.libs;$ORIGIN/../../../runtime_libs")

add_executable(test_goalie_dash_compat test_goalie_dash_compat.cpp)
target_include_directories(test_goalie_dash_compat PRIVATE
    "${R2DRL_ROOT}/helios-base/src/player")
# Keep assertions active even in Release builds.
target_compile_options(test_goalie_dash_compat PRIVATE -UNDEBUG)
enable_testing()
add_test(NAME goalie_dash_compat COMMAND test_goalie_dash_compat)
add_test(NAME player_runtime_help COMMAND ${CMAKE_COMMAND}
    "-DPLAYER=$<TARGET_FILE:sample_player>"
    "-DPLAYER_LIBS=${R2DRL_ROOT}/librcsc/rcsc/.libs:${R2DRL_ROOT}/runtime_libs"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/check_player_help.cmake")
