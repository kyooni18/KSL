#ifndef CLANDING_TAEM_GEOMETRY_H
#define CLANDING_TAEM_GEOMETRY_H

#include <stdbool.h>

#include "terminal_model.h"
#include "taem_frames_energy.h"

typedef struct {
    double runway_along_m;
    double runway_cross_m;
    double altitude_above_runway_m;
    double ground_speed_mps;
    double airspeed_mps;
    double course_deg;
    double flight_path_angle_deg;
    double heading_error_deg;
} TaemGeometryState;

bool taem_geometry_world(const TerminalModel *model, TaemFrameWorld *world);
bool taem_geometry_runway(const TerminalModel *model,
                          TaemFrameWorld *world,
                          TaemRunwayFrame *runway);
bool taem_geometry_state(const TerminalModel *model,
                         const TerminalDynamicState *state,
                         TaemGeometryState *geometry);

#endif
