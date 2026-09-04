cmake_minimum_required(VERSION 3.20)

project(VCanSim
    VERSION 1.0.0
    DESCRIPTION "Virtual SocketCAN ECU & Telemetry Simulator"
    LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "VCanSim requires Linux SocketCAN support")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
add_compile_options($<$<NOT:$<CONFIG:Debug>>:-O2>)

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Core SocketCAN driver
# ---------------------------------------------------------------------------

add_library(vcansim_core STATIC
    src/socketcan_driver.cpp)

target_include_directories(vcansim_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(vcansim_core
    PUBLIC
        Threads::Threads)

# ---------------------------------------------------------------------------
# Applications
# ---------------------------------------------------------------------------

add_executable(engine_ecu src/engine_ecu.cpp)
target_link_libraries(engine_ecu PRIVATE vcansim_core)

add_executable(dashboard_rx src/dashboard_rx.cpp)
target_link_libraries(dashboard_rx PRIVATE vcansim_core)

# ---------------------------------------------------------------------------
# Tests (GoogleTest from the system, or fetched automatically)
# ---------------------------------------------------------------------------

include(CTest)

if(BUILD_TESTING)
    find_package(GTest QUIET)

    if(NOT GTest_FOUND)
        include(FetchContent)
        FetchContent_Declare(googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.15.2
            GIT_SHALLOW TRUE
            GIT_PROGRESS TRUE)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()

    add_executable(test_can_protocol tests/test_can_protocol.cpp)
    target_link_libraries(test_can_protocol
        PRIVATE
            vcansim_core
            GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(test_can_protocol)
endif()

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------

install(TARGETS engine_ecu dashboard_rx
    RUNTIME DESTINATION bin)
