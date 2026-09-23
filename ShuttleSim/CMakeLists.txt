cmake_minimum_required(VERSION 3.16)
project(ShuttleSim C)
set(CMAKE_C_STANDARD 17)
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


set(CLANDING_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../CLanding" CACHE PATH
    "CLanding source root (canonical CLanding or explicit Legacy/CLanding)")

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
        src/math3.c
        src/quat.c
        src/world.c
        src/aero.c
        src/attitude.c
        src/scenario.c
        src/sim.c
    )
    target_include_directories(shuttlesim_offline PRIVATE include "${CLANDING_ROOT}" "${CLANDING_ROOT}/include")
    target_compile_options(shuttlesim_offline PRIVATE -Wall -Wextra -Wpedantic)
    if(UNIX)
        target_link_libraries(shuttlesim_offline m)
    endif()
endif()

if(SHUTTLESIM_OFFLINE_API)
    resolve_clanding_source(CLANDING_SIM_TELEMETRY sim_telemetry.c)
    resolve_clanding_source(CLANDING_GUIDANCE_HAC_PATH guidance_hac_path.c)
    resolve_clanding_source(CLANDING_GUIDANCE_CORE guidance_core.c)
    resolve_clanding_source(CLANDING_GUIDANCE_ENTRY guidance_entry.c)
    resolve_clanding_source(CLANDING_GUIDANCE_TAEM guidance_taem.c)
    resolve_clanding_source(CLANDING_GUIDANCE_FINAL guidance_final.c)
    resolve_clanding_source(CLANDING_GUIDANCE_HAC_PLANNER guidance_hac_planner.c)
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
    resolve_clanding_source(CLANDING_ENTRY_EXEC entry_exec.c)
    resolve_clanding_source(CLANDING_ENTRY_LATERAL entry_lateral.c)
    resolve_clanding_source(CLANDING_TAEM_EXEC taem_exec.c)
    resolve_clanding_source(CLANDING_TRAJECTORY_CALIBRATION trajectory_calibration.c)
    resolve_clanding_source(CLANDING_VESSEL_PHYSICS vessel_physics.c)
    resolve_clanding_source(CLANDING_CALIBRATION calibration.c)
    resolve_clanding_source(CLANDING_CALIBRATION_OBSERVATION calibration_observation.c)

    add_library(shuttlesim_expert SHARED
        rl/expert.c
        ${CLANDING_SIM_TELEMETRY}
        ${CLANDING_GUIDANCE_HAC_PATH}
        ${CLANDING_GUIDANCE_CORE}
        ${CLANDING_GUIDANCE_ENTRY}
        ${CLANDING_GUIDANCE_TAEM}
        ${CLANDING_GUIDANCE_FINAL}
        ${CLANDING_GUIDANCE_HAC_PLANNER}
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
        ${CLANDING_ENTRY_EXEC}
        ${CLANDING_ENTRY_LATERAL}
        ${CLANDING_TAEM_EXEC}
        ${CLANDING_TRAJECTORY_CALIBRATION}
        ${CLANDING_VESSEL_PHYSICS}
        ${CLANDING_CALIBRATION}
        ${CLANDING_CALIBRATION_OBSERVATION}
    )
    target_include_directories(shuttlesim_expert PRIVATE "${CLANDING_ROOT}" "${CLANDING_ROOT}/include")
    target_compile_options(shuttlesim_expert PRIVATE -Wall -Wextra -Wpedantic)
    if(UNIX)
        target_link_libraries(shuttlesim_expert m)
    endif()
endif()
