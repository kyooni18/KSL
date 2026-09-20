cmake_minimum_required(VERSION 3.16)
project(ShuttleSim C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
add_executable(shuttlesim
    src/main.c
    src/math3.c
    src/quat.c
    src/world.c
    src/aero.c
    src/attitude.c
    src/scenario.c
    src/replay.c
    src/protocol.c
    src/sim.c
)
target_include_directories(shuttlesim PRIVATE include)
if(UNIX)
    target_link_libraries(shuttlesim m)
endif()
target_compile_options(shuttlesim PRIVATE -Wall -Wextra -Wpedantic)

# Explicitly opt in: this raw physics ABI is not a safety-projected RL driver.
option(SHUTTLESIM_OFFLINE_API "Build simulator-only in-process physics API" OFF)
if(SHUTTLESIM_OFFLINE_API)
    add_library(shuttlesim_offline SHARED
        rl/physics.c
        src/math3.c
        src/quat.c
        src/world.c
        src/aero.c
        src/attitude.c
        src/scenario.c
        src/sim.c
    )
    target_include_directories(shuttlesim_offline PRIVATE include ../CLanding)
    target_compile_options(shuttlesim_offline PRIVATE -Wall -Wextra -Wpedantic)
    if(UNIX)
        target_link_libraries(shuttlesim_offline m)
    endif()
endif()

if(SHUTTLESIM_OFFLINE_API)
    add_library(shuttlesim_expert SHARED
        rl/expert.c
        ../CLanding/sim_telemetry.c
        ../CLanding/guidance.c
        ../CLanding/predictor.c
        ../CLanding/models.c
        ../CLanding/math.c
        ../CLanding/decision_envelope.c
        ../CLanding/json.c
        ../CLanding/entry_alpha.c
        ../CLanding/entry_drag_reference.c
        ../CLanding/entry_exec.c
        ../CLanding/entry_lateral.c
        ../CLanding/taem_exec.c
        ../CLanding/trajectory_calibration.c
        ../CLanding/vessel_physics.c
        ../CLanding/calibration.c
        ../CLanding/calibration_observation.c
    )
    target_include_directories(shuttlesim_expert PRIVATE ../CLanding)
    target_compile_options(shuttlesim_expert PRIVATE -Wall -Wextra -Wpedantic)
    if(UNIX)
        target_link_libraries(shuttlesim_expert m)
    endif()
endif()
