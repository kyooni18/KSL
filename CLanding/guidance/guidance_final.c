#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
typedef struct {
    double arrest_accel,sgs_height,sgs_sink,final_accel,float_height,float_sink,lift_ratio,aoa_max,g0;
    const PlanetModel*planet;double site_altitude;
    double preflare_trigger_altitude;
} TerminalFlarePlan;

static double final_lift_aoa_limit(const Telemetry *t, const VehicleProfile *v);
static void terminal_learn_aero(GuidanceMachine*g,const Telemetry*t,const VehicleProfile*v);
static double terminal_predict_touchdown_speed(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,double height,double V,double sink,bool latched,
        const TerminalFlarePlan*p);
static double terminal_final_distance(const GuidanceMachine*g,const GuidanceSettings*s);
static double terminal_final_alignment_speed(const GuidanceMachine*g,
        const GuidanceSettings*s,const VehicleProfile*v);
static double terminal_final_speed_target(const GuidanceMachine*g,
        const VehicleProfile*v,const GuidanceSettings*s,double distance);
static void terminal_observe_response(GuidanceMachine*g,const Telemetry*t,double dt);
static TerminalPreflarePlan terminal_preflare_plan(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg);
/* terminal_preflare_alignment_valid declared in guidance_internal.h */
static bool terminal_update_ground_latch(GuidanceMachine*g,const Telemetry*t,const LandingConfiguration*cfg,double dt);
static double observed_signed_roll_rate(const Telemetry *t);


/* Internal source slices; compiled as this single translation unit. */
#include "final/planning.inc"
#include "final/phases.inc"
#include "final/sequence.inc"
#include "final/recovery.inc"
