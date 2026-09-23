#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
static double hac_transition_rate_ratio_limit(void);
static void append_veto_reason(char *buf,size_t n,bool *first,const char *text);
static HACPoint2 hac_bezier_second_derivative(const HACTransitionPlan*p,double u);
static bool hac_transition_plan_entry(HACTransitionPlan*out,double current_e,double current_n,
        double future_e,double future_n,double start_course,double future_course,
        const LandingSite*site,const GuidanceSettings*s,double hac_radius,double side,
        double start_air_speed,double end_air_speed,double speed_loss_accel,
        double max_lateral_accel,double preferred_fixed_path,
        double min_fixed_path,double max_fixed_path,double entry_course);
static double hac_tangent_entry_course(double e,double n,HACPoint2 center,
        double radius,double side);
static HACGuidance hac_circle_path_guidance(const Telemetry*t,const LandingSite*site,
        const GuidanceSettings*s,double planet_radius,double side,double gravity,
        double course,double hac_radius);
static double hac_prefinal_rollout_length(const Telemetry*t,const GuidanceSettings*s);
static HACGuidance hac_prefinal_path_guidance(const Telemetry*t,const LandingSite*site,
        const GuidanceSettings*s,double planet_radius,double side,double gravity,
        double course,double hac_radius,double*out_progress);

                                                                      
                                
                                                                      
                                
                                                                        
                                                                          
                                            
                                                                  
                               

/* A fixed angular limit can reject compact low-speed energy paths even
   when measured lift provides sufficient turn authority. Use one
   low-speed tracking envelope for selection, curvature and reference slew.
   Lift/bank/load qualification remains independent of this bandwidth cap. */

/* Internal source slices; compiled as this single translation unit. */
#include "hac_path/geometry.inc"
#include "hac_path/transition.inc"
#include "hac_path/vertical_profile.inc"
#include "hac_path/tracking.inc"
#include "hac_path/reference.inc"
