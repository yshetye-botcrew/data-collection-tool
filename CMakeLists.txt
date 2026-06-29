cmake_minimum_required(VERSION 3.14)
project(dds_mcap_bag LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# message_manager debian package lives under /opt/botcrew (same as the
# foxglove-aggregator). It pulls in Fast-DDS 3.4.1 + fastcdr as dependencies
# and exports the message_manager::message_manager imported target.
list(PREPEND CMAKE_PREFIX_PATH "/opt/botcrew")
find_package(message_manager REQUIRED)
message(STATUS "Found message_manager ${message_manager_VERSION}")

# MCAP is header-only. The system package (/usr/include/mcap, from data-logger)
# ships the .hpp but NOT the .inl implementation files, so we vendor a complete
# copy of mcap 2.1.1 under third_party/ for the *declarations*.
#
# libdata_logger.a (pulled in transitively by message_manager, built with
# DATA_LOGGER_ENABLED) contains only a PARTIAL copy of these mcap symbols —
# enough for its writer-only use, but missing reader decompression classes the
# player needs. We compile the COMPLETE implementation here and link with
# --allow-multiple-definition so this copy wins and the identical (same 2.1.1
# source) data_logger duplicates are ignored. lz4/zstd back chunk compression.
set(MCAP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mcap/include)
find_library(LZ4_LIB lz4 REQUIRED)
find_library(ZSTD_LIB zstd REQUIRED)

add_library(mcap_impl STATIC src/mcap_impl.cpp)
target_include_directories(mcap_impl PUBLIC ${MCAP_INCLUDE_DIR})
target_link_libraries(mcap_impl PUBLIC ${LZ4_LIB} ${ZSTD_LIB})

# ---- recorder: DDS subscribe -> CDR -> MCAP ----
add_executable(dds_recorder src/recorder.cpp)
target_include_directories(dds_recorder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${MCAP_INCLUDE_DIR})
target_link_libraries(dds_recorder PRIVATE
    mcap_impl message_manager::message_manager)
target_link_options(dds_recorder PRIVATE -Wl,--allow-multiple-definition)

# ---- player: MCAP -> CDR -> DDS publish ----
add_executable(dds_player src/player.cpp)
target_include_directories(dds_player PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${MCAP_INCLUDE_DIR})
target_link_libraries(dds_player PRIVATE
    mcap_impl message_manager::message_manager)
target_link_options(dds_player PRIVATE -Wl,--allow-multiple-definition)

# ---- loopback test helpers ----
add_executable(test_publisher test/test_publisher.cpp)
target_link_libraries(test_publisher PRIVATE message_manager::message_manager)

add_executable(test_subscriber test/test_subscriber.cpp)
target_link_libraries(test_subscriber PRIVATE message_manager::message_manager)

install(TARGETS dds_recorder dds_player RUNTIME DESTINATION bin)
