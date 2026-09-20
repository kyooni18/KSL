#include "planner_log.h"
#include "json.h"
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

struct PlannerLog {
    FILE *file;
    char directory[1300], stamp[96], vessel[160], session_id[192], current_path[1700], native_build[96], vehicle_peer_path[1700];
    bool vehicle_peer_available;
    LandingConfiguration configuration;
    PlannerLogPolicyState policy;
    size_t max_segment_bytes, segment_bytes;
    unsigned segment_index;
    uint64_t record_sequence;
};

static bool materially_different(double a,double b,double eps){
    if(!isfinite(a)||!isfinite(b))return isfinite(a)!=isfinite(b);
    return fabs(a-b)>eps;
}
static bool planner_plan_changed(const EntryControlPlan*a,const EntryControlPlan*b){
    if(!a||!b)return a!=b;
    if(a->valid!=b->valid)return true;
    if(!a->valid)return false;
    return a->plan_id!=b->plan_id||a->plan_version!=b->plan_version||a->parent_plan_id!=b->parent_plan_id||a->parent_plan_version!=b->parent_plan_version||
        a->terminal_ready!=b->terminal_ready||a->has_planned_reversal!=b->has_planned_reversal||
        a->planned_reversal_is_final!=b->planned_reversal_is_final||a->final_heading_lock!=b->final_heading_lock||
        materially_different(a->target_bank,b->target_bank,.25)||materially_different(a->target_aoa,b->target_aoa,.20)||
        materially_different(norm_signed_deg(a->target_heading-b->target_heading),0,.50)||
        materially_different(a->segment_duration,b->segment_duration,.75)||
        materially_different(a->planned_reversal_ut,b->planned_reversal_ut,.25)||
        materially_different(a->planned_reversal_sign,b->planned_reversal_sign,.1)||
        materially_different(a->taem_range_error,b->taem_range_error,250.0)||
        materially_different(a->taem_speed,b->taem_speed,2.0)||
        materially_different(a->taem_energy_error,b->taem_energy_error,2500.0);
}

void planner_log_policy_init(PlannerLogPolicyState*s){
    if(!s)return;memset(s,0,sizeof(*s));s->last_scalar_ut=s->last_trajectory_ut=s->last_ut=-DBL_MAX;
}

PlannerLogDecision planner_log_policy_step(PlannerLogPolicyState*s,double ut,GuidancePhase phase,
        const EntryControlPlan*plan,bool constraint_active,bool predictor_discontinuity,bool explicit_force){
    PlannerLogDecision d;memset(&d,0,sizeof(d));if(!s)return d;
    bool plan_valid=plan&&plan->valid;
    bool executable_lineage=plan_valid&&plan->plan_id!=0;
    d.executable_lineage=executable_lineage;
    d.time_rewind=s->initialized&&isfinite(ut)&&ut+1e-6<s->last_ut;
    d.phase_changed=!s->initialized||phase!=s->last_phase;
    d.plan_changed=!s->initialized||(plan_valid!=s->last_plan_valid)||(plan_valid&&planner_plan_changed(plan,&s->last_plan));
    d.constraint_changed=!s->initialized||constraint_active!=s->last_constraint_active;
    d.predictor_discontinuity=predictor_discontinuity;
    if(d.phase_changed)d.event_flags|=PLANNER_LOG_EVENT_PHASE;
    if(d.plan_changed)d.event_flags|=PLANNER_LOG_EVENT_PLAN;
    if(d.constraint_changed)d.event_flags|=PLANNER_LOG_EVENT_CONSTRAINT;
    if(predictor_discontinuity)d.event_flags|=PLANNER_LOG_EVENT_PREDICTOR_DISCONTINUITY;
    if(explicit_force)d.event_flags|=PLANNER_LOG_EVENT_EXPLICIT;
    if(d.time_rewind)d.event_flags|=PLANNER_LOG_EVENT_TIME_REWIND;
    d.forced=d.event_flags!=0;
    d.durable=d.phase_changed||d.plan_changed||explicit_force||d.time_rewind;

    if(executable_lineage){
        s->plan_id=plan->plan_id;s->plan_version=plan->plan_version;
        s->parent_plan_id=plan->parent_plan_id;s->parent_plan_version=plan->parent_plan_version;
        if(plan->plan_id>s->next_plan_id)s->next_plan_id=plan->plan_id;
    }else if(plan_valid){
        bool new_family=!s->initialized||!s->last_plan_valid||d.phase_changed||d.time_rewind||s->last_plan.plan_id!=0;
        if(new_family){
            uint64_t old_id=s->last_plan_valid?s->plan_id:0,old_version=s->last_plan_valid?s->plan_version:0;
            s->parent_plan_id=old_id;s->parent_plan_version=old_version;
            s->plan_id=++s->next_plan_id;s->plan_version=1;
        }else if(d.plan_changed){
            s->parent_plan_id=s->plan_id;s->parent_plan_version=s->plan_version;
            s->plan_version++;
        }
    }else{
        s->plan_id=s->plan_version=s->parent_plan_id=s->parent_plan_version=0;
    }
    d.plan_id=s->plan_id;d.plan_version=s->plan_version;
    d.parent_plan_id=s->parent_plan_id;d.parent_plan_version=s->parent_plan_version;

    if(d.time_rewind){s->last_scalar_ut=s->last_trajectory_ut=-DBL_MAX;}
    d.emit_scalar=d.forced||!isfinite(s->last_scalar_ut)||ut-s->last_scalar_ut>=PLANNER_LOG_SCALAR_INTERVAL_SECONDS-1e-9;
    d.emit_trajectory=d.phase_changed||d.plan_changed||d.constraint_changed||d.predictor_discontinuity||explicit_force||d.time_rewind||
        !isfinite(s->last_trajectory_ut)||ut-s->last_trajectory_ut>=PLANNER_LOG_TRAJECTORY_INTERVAL_SECONDS-1e-9;
    if(d.emit_scalar)s->last_scalar_ut=ut;
    if(d.emit_trajectory)s->last_trajectory_ut=ut;
    s->initialized=true;s->last_phase=phase;s->last_plan_valid=plan_valid;s->last_constraint_active=constraint_active;s->last_ut=ut;
    if(plan_valid)s->last_plan=*plan;else memset(&s->last_plan,0,sizeof(s->last_plan));
    return d;
}

static const char*phase_name(GuidancePhase p){
    static const char*n[]={"idle","planning","calibration","coast","burnSetup","deorbitBurn","entryInterface","entryEnergy","taem","headingAlignment","final","flare","touchdown","rollout","complete","paused","abort","fault","attitudeRecovery"};
    return (unsigned)p<sizeof(n)/sizeof(n[0])?n[p]:"unknown";
}
static const char*source_name(PredictorPlanCandidateSource s){return s==PREDICTOR_PLAN_CANDIDATE_BOUNDED?"bounded":s==PREDICTOR_PLAN_CANDIDATE_LEGACY?"legacy":"nominal";}
static const char*mode_name(EntrySupervisionMode m){return m==ENTRY_SUPERVISION_PASS_THROUGH?"passThrough":m==ENTRY_SUPERVISION_BOUNDED_CORRECTION?"boundedCorrection":m==ENTRY_SUPERVISION_FALLBACK?"fallback":"infeasible";}
static void key(JsonWriter*w,const char*k){jw_string(w,k);jw_char(w,':');}
static void comma(JsonWriter*w){jw_char(w,',');}
static void num(JsonWriter*w,const char*k,double v){key(w,k);if(isfinite(v))jw_number(w,v);else jw_null(w);comma(w);}
static void boolean(JsonWriter*w,const char*k,bool v){key(w,k);jw_bool(w,v);comma(w);}
static void integer(JsonWriter*w,const char*k,long long v){key(w,k);jw_integer(w,v);comma(w);}

static void plan_json(JsonWriter*w,const EntryControlPlan*p){
    if(!p||!p->valid){jw_null(w);return;}jw_char(w,'{');
    boolean(w,"valid",p->valid);boolean(w,"terminalReady",p->terminal_ready);
    integer(w,"planId",p->plan_id);integer(w,"planVersion",p->plan_version);integer(w,"parentPlanId",p->parent_plan_id);integer(w,"parentPlanVersion",p->parent_plan_version);
    num(w,"plannedUT",p->planned_ut);num(w,"targetBank",p->target_bank);num(w,"targetAoA",p->target_aoa);num(w,"targetHeading",p->target_heading);
    num(w,"bankCap",p->bank_cap);num(w,"turnRadius",p->target_turn_radius);num(w,"segmentDuration",p->segment_duration);num(w,"plannerCost",p->cost);
    num(w,"taemRangeError",p->taem_range_error);num(w,"taemSpeed",p->taem_speed);num(w,"taemEnergyError",p->taem_energy_error);num(w,"closestDistance",p->closest_distance);
    /* Explicit alias for web/replay gating: a visible plan is not merely a valid
       propagated target; it must be a ready TAEM capture.  More detailed veto
       masks are emitted by TAEM interface diagnostics when available. */
    boolean(w,"taemCaptureReady",p->terminal_ready);
    key(w,"taemCaptureVeto");if(p->terminal_ready)jw_integer(w,0);else jw_null(w);comma(w);
    key(w,"taemCaptureVetoReasons");jw_raw(w,p->terminal_ready?"[]":"[\"not-ready\"]");comma(w);
    boolean(w,"hasPlannedReversal",p->has_planned_reversal);boolean(w,"plannedReversalFinal",p->planned_reversal_is_final);boolean(w,"finalHeadingLock",p->final_heading_lock);
    num(w,"plannedReversalUT",p->planned_reversal_ut);num(w,"plannedReversalRange",p->planned_reversal_range);num(w,"plannedReversalSign",p->planned_reversal_sign);
    key(w,"predictedReversals");jw_integer(w,p->predicted_reversals);jw_char(w,'}');
}
static void assessment_json(JsonWriter*w,const EntryPredictionAssessment*a){
    jw_char(w,'{');boolean(w,"valid",a->valid);boolean(w,"safe",a->safe);boolean(w,"terminalFeasible",a->terminal_feasible);
    boolean(w,"dynamicPressureViolation",a->dynamic_pressure_violation);boolean(w,"gLoadViolation",a->g_load_violation);boolean(w,"stallRisk",a->stall_risk);boolean(w,"controlMarginViolation",a->control_margin_violation);
    num(w,"dynamicPressureRatio",a->dynamic_pressure_ratio);num(w,"gLoadRatio",a->g_load_ratio);num(w,"minimumSpeedRatio",a->minimum_speed_ratio);num(w,"bankMargin",a->bank_margin);
    key(w,"aoaMargin");if(isfinite(a->aoa_margin))jw_number(w,a->aoa_margin);else jw_null(w);jw_char(w,'}');
}
static void cost_terms_json(JsonWriter*w,const PredictorPlannerCostTerms*c){
    jw_char(w,'{');num(w,"terminalRange",c->terminal_range);num(w,"terminalEnergy",c->terminal_energy);num(w,"hacGeometry",c->hac_geometry);num(w,"pendingTerminal",c->pending_terminal);
    num(w,"altitude",c->altitude);num(w,"interfaceMatch",c->interface_match);num(w,"penetration",c->penetration);num(w,"shortHorizon",c->short_horizon);num(w,"transition",c->transition);
    num(w,"dynamicPressure",c->dynamic_pressure);num(w,"gLoad",c->g_load);num(w,"reversalCount",c->reversal_count);num(w,"turnTightness",c->turn_tightness);
    num(w,"verticalLift",c->vertical_lift);num(w,"bankChange",c->bank_change);num(w,"aoaChange",c->aoa_change);key(w,"boundedAdjustment");jw_number(w,c->bounded_adjustment);jw_char(w,'}');
}
static unsigned selection_rank(const PredictorPlannerTrace*t,unsigned idx){
    const PredictorPlannerCandidateTrace*c=&t->candidates[idx];unsigned rank=1;
    for(unsigned j=0;j<t->candidate_count;j++)if(j!=idx&&t->candidates[j].source==c->source&&isfinite(t->candidates[j].evaluation_cost)&&t->candidates[j].evaluation_cost<c->evaluation_cost)rank++;
    return rank;
}
static void reasons_json(JsonWriter*w,const PredictorPlannerCandidateTrace*c){
    jw_char(w,'[');bool first=true;
#define REASON(flag,text) do{if(c->rejection_flags&(flag)){if(!first)comma(w);jw_string(w,text);first=false;}}while(0)
    REASON(PREDICTOR_CANDIDATE_REJECT_INVALID,"invalidAssessment");REASON(PREDICTOR_CANDIDATE_REJECT_DYNAMIC_PRESSURE,"dynamicPressure");REASON(PREDICTOR_CANDIDATE_REJECT_G_LOAD,"gLoad");
    REASON(PREDICTOR_CANDIDATE_REJECT_STALL,"stallRisk");REASON(PREDICTOR_CANDIDATE_REJECT_CONTROL_MARGIN,"controlMargin");REASON(PREDICTOR_CANDIDATE_REJECT_NO_TAEM,"noTAEMCapture");
    REASON(PREDICTOR_CANDIDATE_REJECT_TERMINAL_INFEASIBLE,"terminalInfeasible");REASON(PREDICTOR_CANDIDATE_REJECT_HIGHER_COST,"higherObjectiveCost");REASON(PREDICTOR_CANDIDATE_REJECT_POLICY_PRIORITY,"policyPriority");
#undef REASON
    jw_char(w,']');
}
static void candidate_path_json(JsonWriter*w,const PredictorPlannerCandidateTrace*c){
    if(!c||c->trajectory_sample_count==0){jw_raw(w,"[]");return;}
    Trajectory path={.points=(TrajectoryPoint*)c->trajectory_points,.count=c->trajectory_sample_count,.capacity=c->trajectory_sample_count};
    trajectory_json(w,&path);
}
static void candidates_json(JsonWriter*w,const PredictorPlannerTrace*t,bool include_geometry){
    jw_char(w,'[');
    if(t){
        for(unsigned i=0;i<t->candidate_count;i++){
            if(i)comma(w);const PredictorPlannerCandidateTrace*c=&t->candidates[i];jw_char(w,'{');
            integer(w,"evaluationIndex",c->evaluation_index);key(w,"source");jw_string(w,source_name(c->source));comma(w);
            boolean(w,"selected",c->selected);integer(w,"selectionRank",selection_rank(t,i));integer(w,"rejectionFlags",c->rejection_flags);num(w,"evaluationCost",c->evaluation_cost);
            key(w,"costTerms");cost_terms_json(w,&c->cost_terms);comma(w);key(w,"assessment");assessment_json(w,&c->assessment);comma(w);
            boolean(w,"reachedTAEM",c->reached_taem);num(w,"peakDynamicPressure",c->peak_dynamic_pressure);num(w,"peakGLoad",c->peak_g_load);num(w,"minimumEntrySpeed",c->minimum_entry_speed);num(w,"terminalCaptureScore",c->terminal_capture_score);
            integer(w,"trajectoryTotalPoints",c->trajectory_total_points);integer(w,"trajectorySamplePoints",c->trajectory_sample_count);boolean(w,"trajectoryTruncated",c->trajectory_truncated);
            if(include_geometry){key(w,"candidatePath");candidate_path_json(w,c);comma(w);}
            key(w,"rejectionReasons");reasons_json(w,c);comma(w);key(w,"plan");plan_json(w,&c->plan);jw_char(w,'}');
        }
    }
    jw_char(w,']');
}
static void trajectory_sample_json(JsonWriter*w,const Trajectory*t){
    if(!t||!t->count){jw_raw(w,"[]");return;}Trajectory r;trajectory_init(&r);
    size_t max=PLANNER_LOG_MAX_TRAJECTORY_POINTS,step=t->count<=max?1:(t->count-1)/(max-1)+(((t->count-1)%(max-1))?1:0);
    for(size_t i=0;i<t->count;i+=step)(void)trajectory_append(&r,t->points[i]);
    if(r.count==0||r.points[r.count-1].ut!=t->points[t->count-1].ut)(void)trajectory_append(&r,t->points[t->count-1]);
    trajectory_json(w,&r);trajectory_clear(&r);
}
static void session_start_json(JsonWriter*w,const PlannerLog*l){
    const char*campaign_identity=getenv("KSP_LANDER_CAMPAIGN_IDENTITY");
    jw_char(w,'{');integer(w,"schemaVersion",5);integer(w,"sessionSchemaVersion",1);key(w,"recordType");jw_string(w,"sessionStart");comma(w);key(w,"stream");jw_string(w,"planner");comma(w);
    key(w,"startedUTC");jw_string(w,l->stamp);comma(w);key(w,"sessionId");jw_string(w,l->session_id);comma(w);key(w,"campaignIdentity");jw_string(w,(campaign_identity&&*campaign_identity)?campaign_identity:"");comma(w);key(w,"vessel");jw_string(w,l->vessel);comma(w);
    key(w,"peer");jw_char(w,'{');key(w,"stream");jw_string(w,"vehicle");comma(w);boolean(w,"available",l->vehicle_peer_available);key(w,"path");if(l->vehicle_peer_path[0])jw_string(w,l->vehicle_peer_path);else jw_null(w);jw_char(w,'}');comma(w);
    key(w,"runtime");jw_char(w,'{');key(w,"nativeBuild");jw_string(w,l->native_build);jw_char(w,'}');comma(w);
    key(w,"modelIdentity");jw_char(w,'{');key(w,"vehicleModelId");jw_string(w,l->configuration.vehicle.model_id);jw_char(w,'}');comma(w);
    integer(w,"rotationIndex",l->segment_index);key(w,"encoding");jw_string(w,"lower-rate exhaustive planner summaries; event-forced <=160-point selected raw/published paths; event-rate <=24-point candidate paths");comma(w);
    key(w,"configuration");landing_configuration_json(w,&l->configuration);jw_char(w,'}');
}
static void planner_log_durable_flush(PlannerLog*l){
    if(!l||!l->file)return;fflush(l->file);int fd=fileno(l->file);if(fd>=0)(void)fsync(fd);
}
static bool direct_write(PlannerLog*l,JsonWriter*w){
    if(!l||!l->file||!w||w->failed||!w->data)return false;size_t n=w->length+1;
    if(fwrite(w->data,1,w->length,l->file)!=w->length||fputc('\n',l->file)==EOF)return false;l->segment_bytes+=n;fflush(l->file);return true;
}
static bool open_segment(PlannerLog*l){
    if(!l)return false;if(l->file){fflush(l->file);int fd=fileno(l->file);if(fd>=0)(void)fsync(fd);fclose(l->file);l->file=NULL;}
    if(l->segment_index==0)snprintf(l->current_path,sizeof(l->current_path),"%s/%s-%s-planner.jsonl",l->directory,l->stamp,l->vessel);
    else snprintf(l->current_path,sizeof(l->current_path),"%s/%s-%s-planner-%03u.jsonl",l->directory,l->stamp,l->vessel,l->segment_index);
    l->file=fopen(l->current_path,"w");if(!l->file)return false;l->segment_bytes=0;JsonWriter w;jw_init(&w);session_start_json(&w,l);bool ok=direct_write(l,&w);jw_free(&w);return ok;
}
static bool write_record(PlannerLog*l,JsonWriter*w){
    if(!l||!w||w->failed||!w->data)return false;size_t n=w->length+1;
    if(l->file&&l->segment_bytes+n>l->max_segment_bytes&&l->segment_bytes>0){l->segment_index++;if(!open_segment(l))return false;}
    return direct_write(l,w);
}

PlannerLog*planner_log_open(const char*directory,const char*stamp,const char*vessel,const char*session_id,const char*native_build,const char*vehicle_peer_path,bool vehicle_peer_available,const LandingConfiguration*cfg,size_t max_bytes){
    if(!directory||!stamp||!vessel||!session_id||!native_build||!cfg)return NULL;PlannerLog*l=calloc(1,sizeof(*l));if(!l)return NULL;
    snprintf(l->directory,sizeof(l->directory),"%s",directory);snprintf(l->stamp,sizeof(l->stamp),"%s",stamp);snprintf(l->vessel,sizeof(l->vessel),"%s",vessel);snprintf(l->session_id,sizeof(l->session_id),"%s",session_id);snprintf(l->native_build,sizeof(l->native_build),"%s",native_build);if(vehicle_peer_path)snprintf(l->vehicle_peer_path,sizeof(l->vehicle_peer_path),"%s",vehicle_peer_path);l->vehicle_peer_available=vehicle_peer_available;
    l->configuration=*cfg;l->max_segment_bytes=max_bytes?fmax((double)max_bytes,65536.0):PLANNER_LOG_DEFAULT_MAX_SEGMENT_BYTES;planner_log_policy_init(&l->policy);
    if(!open_segment(l)){planner_log_close(l);return NULL;}return l;
}
void planner_log_close(PlannerLog*l){if(!l)return;if(l->file){fflush(l->file);int fd=fileno(l->file);if(fd>=0)(void)fsync(fd);fclose(l->file);}free(l);}
unsigned planner_log_segment_index(const PlannerLog*l){return l?l->segment_index:0;}
size_t planner_log_segment_bytes(const PlannerLog*l){return l?l->segment_bytes:0;}
const char*planner_log_current_path(const PlannerLog*l){return l?l->current_path:"";}

PlannerLogDecision planner_log_record(PlannerLog*l,const PlannerLogSample*s){
    PlannerLogDecision none;memset(&none,0,sizeof(none));if(!l||!s)return none;
    PlannerLogDecision d=planner_log_policy_step(&l->policy,s->ut,s->phase,s->plan,s->constraint_active,s->predictor_discontinuity,s->explicit_force);
    if(!d.emit_scalar)return d;
    PredictorPlannerTrace trace_copy,*trace=NULL;
    if(s->trace&&s->trace->valid){
        trace_copy=*s->trace;
        trace_copy.plan_id=d.plan_id;trace_copy.plan_version=d.plan_version;trace_copy.parent_plan_id=d.parent_plan_id;trace_copy.parent_plan_version=d.parent_plan_version;
        trace=&trace_copy;
    }
    JsonWriter w;jw_init(&w);jw_char(&w,'{');integer(&w,"schemaVersion",5);key(&w,"recordType");jw_string(&w,"plannerSample");comma(&w);key(&w,"stream");jw_string(&w,"planner");comma(&w);
    key(&w,"sessionId");jw_string(&w,l->session_id);comma(&w);integer(&w,"plannerRecordSequence",++l->record_sequence);integer(&w,"tickSequence",s->tick_sequence);num(&w,"ut",s->ut);
    key(&w,"phase");jw_string(&w,phase_name(s->phase));comma(&w);integer(&w,"eventFlags",d.event_flags);boolean(&w,"forced",d.forced);boolean(&w,"durable",d.durable);boolean(&w,"phaseChanged",d.phase_changed);boolean(&w,"planChanged",d.plan_changed);boolean(&w,"constraintChanged",d.constraint_changed);boolean(&w,"predictorDiscontinuity",d.predictor_discontinuity);boolean(&w,"trajectoryIncluded",d.emit_trajectory);
    key(&w,"eventReason");if(s->event_reason&&*s->event_reason)jw_string(&w,s->event_reason);else jw_null(&w);comma(&w);
    key(&w,"lineage");jw_char(&w,'{');integer(&w,"planId",d.plan_id);integer(&w,"version",d.plan_version);integer(&w,"parentPlanId",d.parent_plan_id);integer(&w,"parentVersion",d.parent_plan_version);key(&w,"lineageSource");jw_string(&w,d.executable_lineage?"executablePlan":(s->plan&&s->plan->valid?"loggerFallback":"none"));jw_char(&w,'}');comma(&w);
    key(&w,"currentPlan");plan_json(&w,s->plan);comma(&w);
    key(&w,"plannerTrace");if(trace&&trace->valid){jw_char(&w,'{');key(&w,"mode");jw_string(&w,mode_name(trace->mode));comma(&w);integer(&w,"producingTickSequence",trace->producing_tick_sequence);integer(&w,"acceptedTickSequence",trace->accepted_tick_sequence);integer(&w,"planId",trace->plan_id);integer(&w,"planVersion",trace->plan_version);integer(&w,"parentPlanId",trace->parent_plan_id);integer(&w,"parentPlanVersion",trace->parent_plan_version);num(&w,"inputAltitude",trace->altitude);num(&w,"inputAirspeed",trace->airspeed);num(&w,"inputRange",trace->range);num(&w,"inputBank",trace->current_bank);num(&w,"inputAoA",trace->current_aoa);num(&w,"inputSign",trace->current_sign);num(&w,"legElapsed",trace->current_leg_elapsed);num(&w,"commitRemaining",trace->current_commit_remaining);boolean(&w,"legEstablished",trace->current_leg_established);boolean(&w,"sideLocked",trace->side_locked);boolean(&w,"finalHeadingLock",trace->final_heading_lock);integer(&w,"selectedIndex",trace->selected_index);integer(&w,"candidateBudget",trace->candidate_budget);integer(&w,"evaluatedCandidates",trace->candidate_count);integer(&w,"droppedCandidates",trace->dropped_candidate_count);num(&w,"searchHorizonSeconds",trace->search_horizon_seconds);num(&w,"maximumBankCorrection",trace->maximum_bank_correction);num(&w,"maximumAoACorrection",trace->maximum_aoa_correction);boolean(&w,"selectionConverged",trace->selection_converged);boolean(&w,"traceTruncated",trace->dropped_candidate_count>0);key(&w,"candidates");candidates_json(&w,trace,d.emit_trajectory);jw_char(&w,'}');}else jw_null(&w);comma(&w);
    if(s->telemetry){const Telemetry*t=s->telemetry;key(&w,"constraints");jw_char(&w,'{');num(&w,"dynamicPressure",t->dynamic_pressure);num(&w,"gForce",t->g_force);num(&w,"stallFraction",t->stall_fraction);boolean(&w,"stallFractionMeasured",t->stall_fraction_is_measured);boolean(&w,"active",s->constraint_active);key(&w,"physicsUncertainty");jw_number(&w,t->physics_certified_uncertainty);jw_char(&w,'}');comma(&w);
        key(&w,"predictionAccuracy");jw_char(&w,'{');num(&w,"altitudeResidual",t->trajectory_altitude_residual);num(&w,"speedResidual",t->trajectory_speed_residual);num(&w,"rangeResidual",t->trajectory_range_residual);num(&w,"rawPublishedDelta30s",t->predicted_raw_published_position_delta_30s);num(&w,"rawPublishedDelta60s",t->predicted_raw_published_position_delta_60s);num(&w,"rawPublishedDelta120s",t->predicted_raw_published_position_delta_120s);num(&w,"physicsRelativeUncertainty",t->predicted_physics_relative_uncertainty);num(&w,"terminalSurvivabilityStressScore",t->predicted_terminal_survivability_stress_score);key(&w,"uncertaintyScenarios");jw_integer(&w,t->predicted_uncertainty_scenarios);jw_char(&w,'}');comma(&w);}
    if(s->calibration){key(&w,"calibration");jw_char(&w,'{');num(&w,"densityScale",s->calibration->density_scale);num(&w,"dragScale",s->calibration->drag_scale);num(&w,"liftScale",s->calibration->lift_scale);num(&w,"bankEffectiveness",s->calibration->bank_effectiveness);num(&w,"confidence",s->calibration->confidence);key(&w,"acceptedSamples");jw_integer(&w,s->calibration->accepted_samples);jw_char(&w,'}');comma(&w);}
    if(s->envelope){key(&w,"aeroEnvelope");jw_char(&w,'[');for(int i=0;i<4;i++){if(i)comma(&w);jw_char(&w,'{');num(&w,"liftToDrag",s->envelope->regimes[i].lift_to_drag);num(&w,"ballisticCoefficient",s->envelope->regimes[i].ballistic_coefficient);key(&w,"confidence");jw_number(&w,s->envelope->regimes[i].confidence);jw_char(&w,'}');}jw_char(&w,']');comma(&w);}
    if(s->snapshot){const LandingSnapshot*q=s->snapshot;key(&w,"terminalGeometry");jw_char(&w,'{');boolean(&w,"terminalPredictionValid",q->guidance_terminal_prediction_valid);boolean(&w,"terminalCandidateValid",q->guidance_terminal_candidate_valid);boolean(&w,"terminalCommitted",q->guidance_terminal_committed);num(&w,"candidateRadius",q->guidance_terminal_candidate_radius);num(&w,"candidateAltitude",q->guidance_terminal_candidate_altitude);num(&w,"candidateSpeed",q->guidance_terminal_candidate_speed);num(&w,"candidateKind",q->guidance_terminal_candidate_kind);boolean(&w,"candidateGeometryDegraded",q->guidance_terminal_candidate_geometry_degraded);boolean(&w,"candidateEnergyDegraded",q->guidance_terminal_candidate_energy_degraded);boolean(&w,"candidateShellDegraded",q->guidance_terminal_candidate_shell_degraded);num(&w,"candidateSide",q->guidance_terminal_candidate_side);num(&w,"candidateSlope",q->guidance_terminal_candidate_slope);num(&w,"candidateCurveLength",q->guidance_terminal_candidate_curve_length);num(&w,"candidateLeadLength",q->guidance_terminal_candidate_lead_length);num(&w,"candidateArcRemaining",q->guidance_terminal_candidate_arc_remaining);num(&w,"candidateFinalDistance",q->guidance_terminal_candidate_final_distance);num(&w,"candidateQuality",q->guidance_terminal_candidate_quality);num(&w,"candidateArrivalUT",q->guidance_terminal_candidate_arrival_ut);num(&w,"candidateCourse",q->guidance_terminal_candidate_course);num(&w,"candidatePeakLateral",q->guidance_terminal_candidate_peak_lateral);num(&w,"candidatePeakRateRatio",q->guidance_terminal_candidate_peak_rate_ratio);num(&w,"candidateExitSpeed",q->guidance_terminal_candidate_exit_speed);boolean(&w,"candidatePathDegraded",q->guidance_terminal_candidate_path_degraded);boolean(&w,"candidateControlDegraded",q->guidance_terminal_candidate_control_degraded);boolean(&w,"candidateRateDegraded",q->guidance_terminal_candidate_rate_degraded);boolean(&w,"candidateEndDegraded",q->guidance_terminal_candidate_end_degraded);num(&w,"referenceFPA",q->guidance_terminal_reference_fpa);num(&w,"mix",q->guidance_terminal_mix);num(&w,"hacRemaining",q->guidance_hac_remaining);num(&w,"hacRadius",q->guidance_hac_radius);num(&w,"minimumTurnRadius",q->guidance_minimum_turn_radius);num(&w,"hacTransitionProgress",q->guidance_hac_transition_progress);num(&w,"commandedCourseRate",q->guidance_commanded_course_rate_estimate);key(&w,"taemTerminalEvaluationValid");jw_bool(&w,q->guidance_taem_exec.terminal_evaluation.valid);comma(&w);key(&w,"taemTerminalPolicyFeasible");jw_bool(&w,q->guidance_taem_exec.terminal_evaluation.feasible);comma(&w);key(&w,"taemTerminalBlockReason");jw_string(&w,taem_terminal_block_reason_string(q->guidance_taem_exec.terminal_evaluation.block_reason));comma(&w);key(&w,"taemTerminalPathCommitted");jw_bool(&w,q->guidance_taem_exec.terminal_contract.path_committed);comma(&w);num(&w,"taemTerminalRangeMargin",q->guidance_taem_exec.terminal_contract.range_margin);num(&w,"taemTerminalQMargin",q->guidance_taem_exec.terminal_contract.dynamic_pressure_margin);num(&w,"taemTerminalSpeedbrakeQMargin",q->guidance_taem_exec.terminal_contract.speedbrake_dynamic_pressure_margin);num(&w,"taemTerminalAltitudeMargin",q->guidance_taem_exec.terminal_contract.altitude_margin);num(&w,"taemTerminalFPAMargin",q->guidance_taem_exec.terminal_contract.flight_path_angle_margin);num(&w,"taemTerminalSpecificEnergyMargin",q->guidance_taem_exec.terminal_contract.specific_energy_margin);num(&w,"taemTerminalResponseAvailable",q->guidance_taem_exec.terminal_contract.response_time_available);num(&w,"taemTerminalResponseRequired",q->guidance_taem_exec.terminal_contract.response_time_required);key(&w,"taemTerminalAttitudeResponseQualified");jw_bool(&w,q->guidance_taem_exec.terminal_contract.attitude_response_qualified);comma(&w);key(&w,"hacCircuitCount");jw_integer(&w,q->guidance_hac_circuit_count);jw_char(&w,'}');comma(&w);}
    if(d.emit_trajectory){key(&w,"rawPrediction");trajectory_sample_json(&w,s->raw_prediction);comma(&w);key(&w,"publishedPrediction");trajectory_sample_json(&w,s->published_prediction);comma(&w);}
    key(&w,"modelIdentity");jw_char(&w,'{');key(&w,"vehicleModelId");jw_string(&w,l->configuration.vehicle.model_id);comma(&w);key(&w,"plannerSchema");jw_string(&w,"entry-predictor-v3-shadow-parity");comma(&w);integer(&w,"calibrationAcceptedSamples",s->calibration?s->calibration->accepted_samples:0);num(&w,"calibrationConfidence",s->calibration?s->calibration->confidence:NAN);const VesselPhysicsModel*physics=s->calibration?s->calibration->physics:NULL;integer(&w,"physicsSampleCount",physics?physics->count:0);integer(&w,"physicsAcceptedSamples",physics?physics->accepted:0);integer(&w,"physicsLiveObservations",physics?physics->live_observations:0);num(&w,"physicsCertifiedConfidence",physics?physics->certified_confidence:NAN);key(&w,"physicsCertifiedUncertainty");if(physics&&isfinite(physics->certified_uncertainty))jw_number(&w,physics->certified_uncertainty);else jw_null(&w);jw_char(&w,'}');jw_char(&w,'}');
    bool wrote=write_record(l,&w);if(wrote&&d.durable)planner_log_durable_flush(l);jw_free(&w);return d;
}
