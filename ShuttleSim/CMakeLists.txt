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
    "Root of the modular CLanding source tree")

function(resolve_clanding_source out_var filename)
    set(candidates
        "${CLANDING_ROOT}/${filename}"
        "${CLANDING_ROOT}/app/${filename}"
        "${CLANDING_ROOT}/guidance/${filename}"
        "${CLANDING_ROOT}/prediction/${filename}"
        "${CLANDING_ROOT}/control/${filename}"
        "${CLANDING_ROOT}/vehicle/${filename}"
        "${CLANDING_ROOT}/calibration/${filename}"
        "${CLANDING_ROOT}/transport/${filename}"
        "${CLANDING_ROOT}/telemetry/${filename}"
        "${CLANDING_ROOT}/persistence/${filename}"
    )
    foreach(candidate IN LISTS candidates)
        if(EXISTS "${candidate}")
            set(${out_var} "${candidate}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    message(FATAL_ERROR "Unable to resolve CLanding source '${filename}' under ${CLANDING_ROOT}")
endfunction()


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
    resolve_clanding_source(CLANDING_SIM_TELEMETRY sim_telemetry.c)
    resolve_clanding_source(CLANDING_GUIDANCE_COMMON guidance_common.c)
    resolve_clanding_source(CLANDING_GUIDANCE_CORE guidance_core.c)
    resolve_clanding_source(CLANDING_GUIDANCE_ENTRY guidance_entry.c)
    resolve_clanding_source(CLANDING_GUIDANCE_TAEM guidance_taem.c)
    resolve_clanding_source(CLANDING_GUIDANCE_FINAL guidance_final.c)
    resolve_clanding_source(CLANDING_GUIDANCE_TERMINAL guidance_terminal.c)
    resolve_clanding_source(CLANDING_GUIDANCE guidance.c)
    resolve_clanding_source(CLANDING_PREDICTOR predictor.c)
    resolve_clanding_source(CLANDING_ASYNC_PREDICTION_POLICY async_prediction_policy.c)
    resolve_clanding_source(CLANDING_MODELS models.c)
    resolve_clanding_source(CLANDING_MATH math.c)
    resolve_clanding_source(CLANDING_DECISION_ENVELOPE decision_envelope.c)
    resolve_clanding_source(CLANDING_JSON json.c)
    resolve_clanding_source(CLANDING_ENTRY_ALPHA entry_alpha.c)
    resolve_clanding_source(CLANDING_ENTRY_DRAG_REFERENCE entry_drag_reference.c)
    resolve_clanding_source(CLANDING_ENTRY_ENERGY_CONTROL entry_energy_control.c)
    resolve_clanding_source(CLANDING_ENTRY_EXEC entry_exec.c)
    resolve_clanding_source(CLANDING_ENTRY_LATERAL entry_lateral.c)
    resolve_clanding_source(CLANDING_TAEM_EXEC taem_exec.c)
    resolve_clanding_source(CLANDING_TAEM_FRAMES_ENERGY taem_frames_energy.c)
    resolve_clanding_source(CLANDING_TAEM_GEOMETRY taem_geometry.c)
    resolve_clanding_source(CLANDING_TAEM_REACHABILITY taem_reachability.c)
    resolve_clanding_source(CLANDING_TAEM_TRACKER taem_tracker.c)
    resolve_clanding_source(CLANDING_TAEM_PLANNER taem_planner.c)
    resolve_clanding_source(CLANDING_TAEM_CANDIDATE_SEARCH taem_candidate_search.c)
    resolve_clanding_source(CLANDING_TERMINAL_MODEL terminal_model.c)
    resolve_clanding_source(CLANDING_TERMINAL_PROPAGATOR terminal_propagator.c)
    resolve_clanding_source(CLANDING_TERMINAL_SOLVER terminal_solver.c)
    resolve_clanding_source(CLANDING_TRAJECTORY_CALIBRATION trajectory_calibration.c)
    resolve_clanding_source(CLANDING_VESSEL_PHYSICS vessel_physics.c)
    resolve_clanding_source(CLANDING_CALIBRATION calibration.c)
    resolve_clanding_source(CLANDING_CALIBRATION_OBSERVATION calibration_observation.c)

    add_library(shuttlesim_expert SHARED
        rl/expert.c
        ${CLANDING_SIM_TELEMETRY}
        ${CLANDING_GUIDANCE_COMMON}
        ${CLANDING_GUIDANCE_CORE}
        ${CLANDING_GUIDANCE_ENTRY}
        ${CLANDING_GUIDANCE_TAEM}
        ${CLANDING_GUIDANCE_FINAL}
        ${CLANDING_GUIDANCE_TERMINAL}
        ${CLANDING_GUIDANCE}
        ${CLANDING_PREDICTOR}
        ${CLANDING_ASYNC_PREDICTION_POLICY}
        ${CLANDING_MODELS}
        ${CLANDING_MATH}
        ${CLANDING_DECISION_ENVELOPE}
        ${CLANDING_JSON}
        ${CLANDING_ENTRY_ALPHA}
        ${CLANDING_ENTRY_DRAG_REFERENCE}
        ${CLANDING_ENTRY_ENERGY_CONTROL}
        ${CLANDING_ENTRY_EXEC}
        ${CLANDING_ENTRY_LATERAL}
        ${CLANDING_TAEM_EXEC}
        ${CLANDING_TAEM_FRAMES_ENERGY}
        ${CLANDING_TAEM_GEOMETRY}
        ${CLANDING_TAEM_REACHABILITY}
        ${CLANDING_TAEM_TRACKER}
        ${CLANDING_TAEM_PLANNER}
        ${CLANDING_TAEM_CANDIDATE_SEARCH}
        ${CLANDING_TERMINAL_MODEL}
        ${CLANDING_TERMINAL_PROPAGATOR}
        ${CLANDING_TERMINAL_SOLVER}
        ${CLANDING_TRAJECTORY_CALIBRATION}
        ${CLANDING_VESSEL_PHYSICS}
        ${CLANDING_CALIBRATION}
        ${CLANDING_CALIBRATION_OBSERVATION}
    )
    target_include_directories(shuttlesim_expert PRIVATE "${CLANDING_ROOT}" "${CLANDING_ROOT}/include")
    target_compile_options(shuttlesim_expert PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(shuttlesim_expert PRIVATE flight_physics)
    if(UNIX)
        target_link_libraries(shuttlesim_expert PRIVATE m)
    endif()
endif()
