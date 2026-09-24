#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
static double terminal_preflare_altitude(const GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s);
static double terminal_final_distance(const GuidanceMachine*g,const GuidanceSettings*s);
static double terminal_final_alignment_speed(const GuidanceSettings*s,
        const VehicleProfile*v);
static double terminal_final_speed_target(const GuidanceMachine*g,
        const VehicleProfile*v,const GuidanceSettings*s,double distance);
static void terminal_observe_response(GuidanceMachine*g,const Telemetry*t,double dt);
static TerminalPreflarePlan terminal_preflare_plan(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
/* terminal_preflare_alignment_valid declared in guidance_internal.h */
__attribute__((unused)) static void terminal_update_gear_latch(GuidanceMachine*g,const Telemetry*t,const GuidanceSettings*s);
static bool terminal_update_ground_latch(GuidanceMachine*g,const Telemetry*t,const LandingConfiguration*cfg,double dt);
static double terminal_approach_drag_aoa(const Telemetry*t,const VehicleProfile*v,
        double measured_loss,double required_loss,double nominal_aoa);
__attribute__((unused)) static GuidanceResult preflare_guidance(GuidanceMachine*g,const Telemetry*t,double course,const LandingConfiguration*cfg,const Trajectory*ref,double dt);
__attribute__((unused)) static GuidanceResult inner_final_guidance(GuidanceMachine*g,const Telemetry*t,double course,const LandingConfiguration*cfg,const Trajectory*ref,double dt);
__attribute__((unused)) static GuidanceResult flare_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const LandingConfiguration*cfg,const Trajectory*ref,double dt);
__attribute__((unused)) static double terminal_touchdown_flare_altitude(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg);
__attribute__((unused)) static bool terminal_airborne_corridor_valid(const GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg);
static double observed_signed_roll_rate(const Telemetry *t);


/* Internal source slices; compiled as this single translation unit. */
#include "final/planning.inc"
#include "final/phases.inc"
#include "final/sequence.inc"
#include "final/recovery.inc"
