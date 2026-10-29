cmake_minimum_required(VERSION 3.16)
project(ShuttleSim C)
include(CTest)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
add_library(flight_physics STATIC
    src/math3.c
    src/quat.c
    src/world.c
    src/aero.c
    src/attitude.c
    src/flight_physics.c
)
target_include_directories(flight_physics PUBLIC include)
target_compile_options(flight_physics PRIVATE -Wall -Wextra -Wpedantic)
if(UNIX)
    target_link_libraries(flight_physics PUBLIC m)
endif()

add_executable(shuttlesim
    src/main.c
    src/scenario.c
    src/replay.c
    src/protocol.c
    src/sim.c
)
target_include_directories(shuttlesim PRIVATE include)
target_link_libraries(shuttlesim PRIVATE flight_physics)
if(UNIX)
    target_link_libraries(shuttlesim PRIVATE m)
endif()
target_compile_options(shuttlesim PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
    add_executable(flight_physics_parity tests/flight_physics_parity.c)
    target_link_libraries(flight_physics_parity flight_physics)
    target_compile_options(flight_physics_parity PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME flight_physics_parity COMMAND flight_physics_parity)

    add_executable(runway_collision tests/runway_collision.c src/sim.c)
    target_link_libraries(runway_collision flight_physics)
    target_compile_options(runway_collision PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME runway_collision COMMAND runway_collision)

    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME guidance_runner
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_guidance_runner.py)
    add_test(NAME simulator_execution
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_sim_execution.py)
    set_tests_properties(simulator_execution PROPERTIES
        ENVIRONMENT "SHUTTLESIM_BINARY=$<TARGET_FILE:shuttlesim>")
endif()


set(CLANDING_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../CLanding" CACHE PATH
    "Root of the landing backend's C ABI headers")
# The landing backend is the Rust crate rust/landing; its static library
# exports the C interface declared by CLanding/include.
set(LANDING_RUST_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../rust/landing/target/release/liblanding.a" CACHE FILEPATH
    "Static library built by `cargo build --release --manifest-path rust/landing/Cargo.toml`")

# Explicitly opt in: this raw physics ABI is not a safety-projected RL driver.
option(SHUTTLESIM_OFFLINE_API "Build simulator-only in-process physics API" OFF)
if(SHUTTLESIM_OFFLINE_API)
    add_library(shuttlesim_offline SHARED
        rl/physics.c
        src/scenario.c
        src/sim.c
    )
    target_include_directories(shuttlesim_offline PRIVATE "${CLANDING_ROOT}" "${CLANDING_ROOT}/include")
    target_link_libraries(shuttlesim_offline PRIVATE flight_physics)
    target_compile_options(shuttlesim_offline PRIVATE -Wall -Wextra -Wpedantic)
    if(UNIX)
        target_link_libraries(shuttlesim_offline PRIVATE m)
    endif()
endif()

if(SHUTTLESIM_OFFLINE_API)
    if(NOT EXISTS "${LANDING_RUST_LIB}")
        message(FATAL_ERROR "Build the landing backend first: make -C CLanding (missing ${LANDING_RUST_LIB})")
    endif()
    add_library(shuttlesim_expert SHARED rl/expert.c)
    target_include_directories(shuttlesim_expert PRIVATE "${CLANDING_ROOT}" "${CLANDING_ROOT}/include" include)
    target_compile_options(shuttlesim_expert PRIVATE -Wall -Wextra -Wpedantic)
    # The Rust library carries its own copy of the flight-physics model, so the
    # C flight_physics archive is deliberately not linked here.
    target_link_libraries(shuttlesim_expert PRIVATE "${LANDING_RUST_LIB}")
    if(UNIX)
        target_link_libraries(shuttlesim_expert PRIVATE sqlite3 m dl pthread)
    endif()
endif()
