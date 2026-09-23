static double observed_signed_roll_rate(const Telemetry *t) {
    /* Use one signed rate source for both departure magnitude and braking
       direction.  The September 9 false recoveries chose the large Euler-rate
       magnitude but the opposite-sign body-rate for the braking test, so a
       saturated aileron that was actually arresting the observed motion was
       mislabeled as divergent.  Near knife-edge Euler roll is geometrically
       contaminated; fade it out and select whichever signed signal still has
       the larger trustworthy magnitude. */
    double euler=t->roll_rate;
    if(!t->has_body_roll_rate||!isfinite(t->body_roll_rate))return euler;
    double c=cos(norm_signed_deg(t->roll)*DEG2RAD),euler_weight=c*c;
    double weighted_euler=euler*euler_weight;
    return fabs(t->body_roll_rate)>=fabs(weighted_euler)?t->body_roll_rate:weighted_euler;
}

       bool control_recovery_needed(GuidanceMachine *g, const Telemetry *t,
        const GuidanceSettings *s, const VehicleProfile *v, double dt) {
    bool hac_control_tuning=g->terminal_region_entered&&
        (g->phase==PHASE_TAEM||g->hac_side_selected)&&
        (!g->terminal_glide_mode||!g->terminal_test_capture_active||g->phase==PHASE_TAEM);
    double signed_roll_rate=observed_signed_roll_rate(t);
    double roll_rate=fabs(signed_roll_rate);
    double pitch_rate = fmax(fabs(t->pitch_rate),
        t->has_body_pitch_rate ? fabs(t->body_pitch_rate) : 0);
    double signed_pitch_rate = t->has_body_pitch_rate && isfinite(t->body_pitch_rate) ?
        t->body_pitch_rate : t->pitch_rate;
    double body_yaw_rate = t->has_body_yaw_rate ? fabs(t->body_yaw_rate) : 0;
    double sideslip = fabs(t->sideslip);
    double beta_rate = 0;
    if(g->has_previous_sideslip&&dt>1e-4)
        beta_rate=norm_signed_deg(t->sideslip-g->previous_sideslip)/dt;
    g->previous_sideslip=t->sideslip;
    g->has_previous_sideslip=true;
    bool roll_saturated = t->has_controls && fabs(t->control_roll) >= .35;
    bool pitch_saturated = t->has_controls && fabs(t->control_pitch) >= .35;
    bool yaw_saturated = t->has_controls && fabs(t->control_yaw) >= .20;
    bool saturated = roll_saturated || pitch_saturated || (!hac_control_tuning&&yaw_saturated);
    double nominal_roll_rate = s->entry_roll_rate;
    if (g->phase == PHASE_TAEM || g->phase == PHASE_HEADING_ALIGNMENT)
        nominal_roll_rate = s->taem_roll_rate;
    else if (g->phase == PHASE_FINAL || g->phase == PHASE_FLARE)
        nominal_roll_rate = s->approach_roll_rate;
    double bad_roll_rate = fmax(12, nominal_roll_rate * 3);
    double extreme_roll_rate = fmax(24, nominal_roll_rate * 6);
    double previous_roll_target = g->roll_limiter.has_value ?
        norm_signed_deg(g->roll_limiter.value) : norm_signed_deg(t->roll);
    double roll_error = norm_signed_deg(previous_roll_target - norm_signed_deg(t->roll));
    double reference_roll_rate = g->roll_limiter.has_value && isfinite(g->roll_limiter.rate) ?
        g->roll_limiter.rate : 0;
    /* Once the fixed-HAC limiter has already arrived at its bank reference,
       its stored rate is a command-history value, not unobserved airframe
       motion. ShuttleSim publishes zero bank rate when that direct command is
       reached; subtracting the stale reference rate then manufactures a
       limit-cycle signal and enters attitude recovery in the middle of the
       analytic arc. Keep the independent measured-rate and departure gates,
       but only compare against the reference rate while an angular error still
       exists. */
    bool fixed_hac_bank_captured=g->fixed_alignment_hac_latched&&
        g->hac_transition_heading_cone&&fabs(roll_error)<=1.5;
    double relative_roll_rate=fixed_hac_bank_captured?
        signed_roll_rate:signed_roll_rate-reference_roll_rate;

    /* A sampled-data limit cycle looks deceptively healthy to an
       instantaneous braking test: each aileron command is usually opposite
       the current body rate because it is undoing the previous overshoot.
       Track relative-rate energy and repeated sign reversals directly.  One
       deliberate S-turn reversal is allowed; a second energetic crossing, or
       sustained rate far outside the guidance envelope, is a control-system
       instability even if every individual command has the right sign. */
    double energetic_crossing_rate=fmax(8.0,nominal_roll_rate*1.30);
    double sustained_excess_rate=fmax(12.0,nominal_roll_rate*2.0);
    bool commanded_roll_motion = fabs(roll_error) > 1.5 &&
        roll_error * signed_roll_rate > 0;
    bool roll_rate_braking = t->has_controls && fabs(t->control_roll) > .03 &&
        t->control_roll * signed_roll_rate < 0;
    bool energetic_crossing=g->has_previous_relative_roll_rate&&
        g->previous_relative_roll_rate*relative_roll_rate<0&&
        fabs(g->previous_relative_roll_rate)>energetic_crossing_rate&&
        fabs(relative_roll_rate)>energetic_crossing_rate;
    g->roll_oscillation_score=fmax(0,g->roll_oscillation_score-dt*.35);
    if(energetic_crossing)g->roll_oscillation_score+=1.0;
    /* A large commanded bank is an intentional transient, not a sampled-data
       limit cycle.  The first exact postburn replay reached the simulator's
       modeled 12.45 deg/s roll response while still 50 deg away from the
       requested bank; counting that capture as sustained excess rate forced
       attitude recovery in near-vacuum before aerodynamic control existed.
       Keep the excess-rate timer for wrong-way motion or motion that has
       already crossed the reference; the independent emergency-rate gate
       still protects gross departures. */
    bool excess_roll_motion=fabs(relative_roll_rate)>sustained_excess_rate&&
        !commanded_roll_motion&&!roll_rate_braking;
    if(excess_roll_motion)
        g->roll_rate_excess_duration+=dt;
    else
        g->roll_rate_excess_duration=fmax(0,g->roll_rate_excess_duration-dt*1.5);
    g->previous_relative_roll_rate=relative_roll_rate;
    g->has_previous_relative_roll_rate=true;
    bool oscillatory_roll=g->roll_oscillation_score>=1.8||g->roll_rate_excess_duration>=1.25;
    /* Near capture the bank error can already be small while the airframe
       still carries substantial roll inertia. If the live aileron command is
       opposing that body rate, the controller is doing exactly the right
       thing: braking through the setpoint. Do not relabel that transient as a
       divergent roll merely because the angle error has fallen below 1.5 deg. */
    /* Atmospheric control is intentionally direct aircraft control. Large
       bank-angle error and brief saturation are normal during an S-turn
       reversal, so departure detection must key off actual angular motion,
       not generic AutoPilot attitude error. */
    /* Use measured aerodynamic force rather than a q threshold to decide
       whether the atmosphere owns the motion. This removes the old dead zone
       where the shuttle could already be aerodynamically diverging below the
       fixed 120 Pa gate. */
    double aero_accel=0;
    if(t->mass>1&&isfinite(t->lift_force)&&isfinite(t->drag_force))
        aero_accel=hypot(t->lift_force,t->drag_force)/t->mass;
    bool aero_loaded=aero_accel>.02;
    /* Heading/course rate is expected in a coordinated banked turn and is
       not a body-yaw departure signal. kRPC 0.5.4 can report an unusable
       constant-zero body yaw stream on this vessel, so only use heading rate
       as a fallback when substantial sideslip proves the turn is
       uncoordinated. */
    double bank_excess=fabs(norm_signed_deg(t->roll))-v->maximum_bank_angle;
    bool bank_departure=bank_excess>fmax(15.0,v->maximum_bank_angle*.22);
    double beta_growth=t->sideslip*beta_rate;
    double beta_growth_fraction=beta_growth/(t->sideslip*t->sideslip+25.0);
    /* A saturated rudder command can already be doing the right thing while
       the airframe still carries yaw inertia in the old direction. Attempt 13
       hit recovery at beta=-11 deg with yaw=-0.79 but body yaw still +3.2
       deg/s: beta was worsening for only the braking transient, and recovery
       interrupted the controller that was actively arresting it. Mirror the
       roll/pitch braking exemptions here. For kRPC's beta convention
       (approximately course-heading), corrective yaw has the same sign as
       beta; if that command opposes the measured body-yaw rate, let the
       ordinary beta loop finish the reversal unless a separate gross-rate
       departure gate fires. */
    double signed_body_yaw_rate=t->has_body_yaw_rate&&isfinite(t->body_yaw_rate)?
        t->body_yaw_rate:t->heading_rate;
    bool corrective_beta_yaw=t->has_controls&&fabs(t->control_yaw)>.08&&
        t->sideslip*t->control_yaw>0;
    bool yaw_rate_braking=corrective_beta_yaw&&
        t->control_yaw*signed_body_yaw_rate<0;
    bool beta_runaway=sideslip>10&&beta_growth_fraction>.12&&
        (saturated||bank_departure||body_yaw_rate>6)&&!yaw_rate_braking;
    bool uncoordinated_yaw=!hac_control_tuning&&(body_yaw_rate>20||
        (sideslip>20&&fabs(t->heading_rate)>12));
    bool extreme_yaw=!hac_control_tuning&&(body_yaw_rate>35||
        (sideslip>30&&fabs(t->heading_rate)>20));
    bool yaw_saturated_departure=!hac_control_tuning&&yaw_saturated&&
        (body_yaw_rate>10||(sideslip>12&&fabs(t->heading_rate)>8))&&!yaw_rate_braking;
    /* Beta magnitude by itself is not a control departure. A banked shuttle
       can carry a transient 10-20 deg sideslip while the rudder/RCS loop is
       actively damping it, especially near the first S-turn reversal. The
       old absolute |beta|>10 deg gate therefore latched recovery on a stable
       vehicle as soon as q crossed 120 Pa. Treat sideslip as a departure only
       when it is accompanied by actual yaw motion or saturated yaw authority;
       the normal entry law already unloads bank continuously as beta grows. */
    bool emergency_rate = roll_rate > 75 || pitch_rate > 45 ||
        (aero_loaded&&extreme_yaw)||(bank_departure&&sideslip>25);
    /* A roll rate above the nominal comfort envelope is not a departure when
       the live aileron command is already opposing that rate.  S-turn capture
       can briefly carry substantial angular momentum through the bank
       setpoint; the old extreme-rate branch ignored the braking exemption
       used by divergent_roll below and switched to recovery exactly while the
       controller was arresting the motion. Keep the unconditional 75 deg/s
       emergency gate, but let correctly-signed braking own the intermediate
       rate band. */
    bool extreme_roll_motion=roll_rate > extreme_roll_rate && !roll_rate_braking;
    bool extreme_rate = emergency_rate || (aero_loaded &&
        (extreme_roll_motion || pitch_rate > 35 || extreme_yaw));
    bool divergent_roll = !commanded_roll_motion && !roll_rate_braking &&
        (roll_rate > bad_roll_rate ||
         (roll_saturated && roll_rate > nominal_roll_rate * 2));
    double target_aoa = g->pitch_limiter.has_value ?
        clampd(g->pitch_limiter.value,0,v->maximum_angle_of_attack) : t->angle_of_attack;
    double aoa_error = target_aoa - t->angle_of_attack;
    /* Saturation is not a departure by itself when the elevator is saturated
       in the direction needed to recover a large incidence error.  The HAC
       test can begin with the shuttle below its requested AoA and still carry
       several deg/s of nose-down inertia; treating that corrective pulse as a
       failure switches to recovery before the elevator has time to arrest it.
       Keep the unconditional high-rate gate, and keep the lower saturated-rate
       gate whenever the actuator is neutral, wrong-signed, or near capture. */
    bool corrective_pitch_saturation = pitch_saturated && fabs(aoa_error) > 3 &&
        t->has_controls && aoa_error * t->control_pitch > 0;
    /* Near the AoA setpoint the terminal pitch loop can legitimately hit a
       large opposite elevator command to arrest accumulated body pitch rate.
       The failed HAC run entered recovery with only ~1-3 deg AoA error while
       +0.8 elevator was braking a -12 deg/s nose-down rate.  Treat that like
       the equivalent roll-rate braking case: the hard 18 deg/s rate gate
       remains, but a saturated actuator that is demonstrably removing rate is
       not itself evidence of a departure. */
    bool pitch_rate_braking = t->has_controls && fabs(t->control_pitch) > .08 &&
        t->control_pitch * signed_pitch_rate < 0;
    bool pitch_departure = pitch_rate > 18 ||
        (pitch_saturated && pitch_rate > 10 && !corrective_pitch_saturation && !pitch_rate_braking);
    bool bad = beta_runaway || (aero_loaded && (oscillatory_roll || extreme_rate || divergent_roll ||
        pitch_departure || uncoordinated_yaw || yaw_saturated_departure));
    g->control_bad_duration = bad ? g->control_bad_duration + dt :
        fmax(0, g->control_bad_duration - dt * 1.5);
    bool recovery_trigger=!g->attitude_recovery&&
        (extreme_rate||oscillatory_roll||g->control_bad_duration>=.35);
    if(recovery_trigger){
        const char*diag_env=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        if(diag_env&&strcmp(diag_env,"1")==0&&!g->diagnostic_shadow)
            fprintf(stderr,
                "CONTROL RECOVERY trigger: UT %.2f phase %d alt %.0f q %.3f V %.1f roll %.2f rollRate %.2f bodyRoll %.2f pitchRate %.2f bodyPitch %.2f beta %.2f betaRate %.2f bodyYaw %.2f headingRate %.2f controls %.3f/%.3f/%.3f aeroAccel %.4f loaded=%d badDur %.2f relRate %.2f refRate %.2f rollErr %.2f aoa %.2f targetAoA %.2f flags bad=%d beta=%d oscillatory=%d extreme=%d divergent=%d pitch=%d yaw=%d yawSat=%d emergency=%d braking=%d rollSat=%d pitchSat=%d yawSatCmd=%d\n",
                t->ut,g->phase,t->mean_altitude,t->dynamic_pressure,t->true_air_speed,
                t->roll,signed_roll_rate,t->has_body_roll_rate?t->body_roll_rate:NAN,
                pitch_rate,t->has_body_pitch_rate?t->body_pitch_rate:NAN,
                t->sideslip,beta_rate,body_yaw_rate,t->heading_rate,
                t->has_controls?t->control_roll:NAN,t->has_controls?t->control_pitch:NAN,
                t->has_controls?t->control_yaw:NAN,aero_accel,aero_loaded,
                g->control_bad_duration,relative_roll_rate,reference_roll_rate,roll_error,
                t->angle_of_attack,target_aoa,bad,beta_runaway,oscillatory_roll,
                extreme_rate,divergent_roll,pitch_departure,uncoordinated_yaw,
                yaw_saturated_departure,emergency_rate,roll_rate_braking,roll_saturated,
                pitch_saturated,yaw_saturated);
        g->attitude_recovery = true;
        g->recovery_duration = g->control_good_duration = 0;
        /* Recovery is an aerodynamic attitude-unload maneuver, not a hold of
           whatever heading happened to exist when the upset was detected.
           Preserve the prograde surface direction so that, once roll/yaw
           rates are quiet again, the bridge can reacquire the velocity
           direction instead of stabilizing a nose-backwards attitude. */
        g->recovery_heading = norm_deg(t->ground_track_heading);
        /* Terminal TAEM may intentionally be flying at very low incidence to
           trade altitude for airspeed.  Recovery must stabilize the airframe,
           not command a discontinuous return to the 18 deg entry trim.  That
           entry-AoA jump caused the 50 km replay to spend the full recovery
           timeout pitching toward a state the terminal energy law explicitly
           did not want. Freeze a modest unloaded incidence for terminal
           recovery; normal entry recovery retains the entry trim. */
        if(g->terminal_region_entered&&g->terminal_glide_mode)
            g->recovery_aoa=clampd(fmax(4.0,t->angle_of_attack+1.5),4.0,10.0);
        else
            g->recovery_aoa=clampd(v->entry_angle_of_attack,10.0,18.0);
        reset_limiters(g);
        g->attitude_recovery = true;
    }
    if (!g->attitude_recovery) return false;
    g->recovery_duration += dt;
    double recovery_aoa=g->recovery_aoa>0?g->recovery_aoa:
        clampd(v->entry_angle_of_attack,10,18);
    /* Low angular rate can mean the shuttle has settled at the wrong pitch.
       In the failed flight this released recovery more than 10 degrees below
       its pitch target and immediately re-entered the departure state. */
    /* Recovery is an aerodynamic maneuver.  Surface Euler pitch becomes a
       poor state coordinate once bank/yaw depart, so release on the same AoA
       state the direct controller is actually stabilizing. */
    bool pitch_captured=fabs(t->angle_of_attack-recovery_aoa)<4.0&&t->angle_of_attack>=0;
    /* Recovery release is about whether the airframe state has converged,
       not whether every actuator is already near neutral.  The 50 km replay
       on 2026-09-09 reached |roll| < 1 deg, |body roll rate| < 1 deg/s and
       sub-degree pitch error while the elevator still used a short braking
       pulse above the generic saturation threshold.  Treating that corrective
       pulse as instability kept control_good_duration pinned at zero until
       the 15 s recovery timeout aborted an otherwise recovered vehicle. */
    bool destabilizing_roll_saturation = roll_saturated && !roll_rate_braking;
    bool destabilizing_pitch_saturation = pitch_saturated &&
        !corrective_pitch_saturation && !pitch_rate_braking;
    bool destabilizing_yaw_saturation = !hac_control_tuning && yaw_saturated &&
        (sideslip > 8 || body_yaw_rate > 5);
    bool stable = pitch_captured&&roll_rate < 5 && pitch_rate < 5 && fabs(norm_signed_deg(t->roll)) < 10 &&
        (!aero_loaded || (sideslip < 8 && !destabilizing_roll_saturation &&
            !destabilizing_pitch_saturation && !destabilizing_yaw_saturation));
    g->control_good_duration = stable ? g->control_good_duration + dt : 0;
    if (g->control_good_duration >= 2) {
        g->attitude_recovery = false;
        g->control_bad_duration = 0;
        g->roll_oscillation_score=0;
        g->roll_rate_excess_duration=0;
        g->has_previous_relative_roll_rate=false;
        g->recovery_aoa=0;
        return false;
    }
    return true;
}




       bool terminal_entry_recovery_available(const GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v,const GuidanceSettings*s){
    /* TAEM acquisition is a one-way ownership boundary.  Before terminal
       acquisition this predicate can still describe whether Entry has enough
       altitude/energy to recover.  After terminal_region_entered is latched,
       no planning, capture or attitude-recovery failure is allowed to send
       the state machine back to S-turn; terminal guidance must replan in place
       or abort if the remaining envelope is unrecoverable. */
    if(!g||g->terminal_region_entered||g->terminal_rehearsal_mode||g->final_approach_captured||
       t->true_air_speed<v->minimum_safe_speed*2.0)return false;
    double response=fmax(4.0,hac_response_lead_time(t,s,0.0));
    double recovery_height=fmax(fmax(g->terminal_test_preflare_altitude,s->flare_altitude*8.0)*3.0,
        fmax(0.0,-t->vertical_speed)*(response+4.0));
    double runway_margin=-s->final_approach_distance-t->runway_along_track;
    double recovery_distance=t->horizontal_speed*response*1.5;
    return t->radar_altitude>recovery_height&&runway_margin>recovery_distance;
}

       void terminal_invalidate_frozen_path(GuidanceMachine*g){
    /* Recovery or a deliberate return to Entry moves the aircraft away from
       the state used to freeze the join.  Never resume that stale geometry.
       Keep terminal mode itself intact here so a low-altitude recovery can
       immediately search a fresh path if there is no safe S-turn margin. */
    g->terminal_candidate.valid=false;g->terminal_path_kind=TERMINAL_PATH_NONE;g->terminal_prediction_valid=false;g->terminal_mix=0;
    g->terminal_prediction_ut=-INFINITY;g->terminal_path_committed=false;
    g->hac_plan_degraded=false;g->hac_plan_geometry_degraded=false;
    g->hac_plan_energy_degraded=false;g->hac_plan_violation_score=0;g->hac_commit_blend=0;
    g->hac_side_selected=false;g->hac_captured=false;g->hac_completed=false;g->hac_progress_valid=false;
    g->hac_transition_active=false;g->hac_transition_heading_cone=false;g->hac_transition_lead_curve=false;g->hac_transition_progress=0;g->hac_transition_lead_progress=0;
    g->hac_transition_lead_rebase_attempted=false;
    g->fixed_hac_reference_valid=false;
    g->fixed_hac_reference_start_altitude=0.0;
    g->fixed_hac_reference_final_altitude=0.0;
    g->fixed_hac_reference_slope_deg=0.0;
    g->fixed_hac_reference_initial_slope_deg=0.0;
    g->fixed_hac_reference_settled_slope_deg=0.0;
    g->fixed_hac_reference_final_slope_deg=0.0;
    g->fixed_hac_reference_transition_distance=0.0;
    g->fixed_hac_reference_exit_transition_distance=0.0;
    g->hac_transition_cone_arc_length=0.0;
    g->hac_remaining=0;g->hac_radius=0;g->hac_capture_lost_duration=0;
    g->hac_transition_handoff_lateral_valid=false;
    g->hac_transition_handoff_lateral_acceleration=0.0;
    g->hac_transition_handoff_blend=0.0;
    g->hac_transition_handoff_aoa_valid=false;
    g->hac_transition_handoff_aoa=0.0;
    g->fixed_hac_lead_speed_target_valid=false;
    g->fixed_hac_lead_target_speed=0.0;
    g->terminal_energy_mismatch_duration=0;
    g->hac_circuit_count=0;g->hac_circuit_slope=0;g->terminal_test_spiral_active=false;
    g->terminal_test_capture_active=g->terminal_glide_mode;
    g->terminal_energy_sample_valid=false;g->terminal_energy_loss_accel_ema=0;g->terminal_speed_loss_accel_ema=0;
    g->terminal_preflare_plan_valid=false;g->final_invalid_duration=0;
}

       void terminal_invalidate_frozen_path_preserve_energy(GuidanceMachine*g){
    /* Geometry may become stale while the aerodynamic observation that proved
       it stale is still exactly the information the replacement selector
       needs. Preserve the filtered loss estimates, but intentionally restart
       the finite-difference sample so the next observation cannot span the
       geometry/state transition. */
    double energy_loss=g->terminal_energy_loss_accel_ema;
    double speed_loss=g->terminal_speed_loss_accel_ema;
    terminal_invalidate_frozen_path(g);
    g->terminal_energy_loss_accel_ema=energy_loss;
    g->terminal_speed_loss_accel_ema=speed_loss;
    g->terminal_energy_sample_valid=false;
}

       GuidanceResult terminal_return_to_entry(GuidanceMachine*g,const Telemetry*t,
        const VehicleState*state,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double dt){
    const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;
    if(taem_exec_owns_vehicle(&g->taem_exec)){
        /* Unified MM305 ownership is one-way.  A circle/spline/HAC-like path may
           become stale, but that invalidates geometry only; it never hands the
           aircraft back to MM304.  Preserve measured energy, drop the frozen path,
           let TAEMEXEC choose S-turn vs path acquisition, and keep a bounded
           terminal command while the path provider replans. */
        terminal_invalidate_frozen_path_preserve_energy(g);
        g->terminal_region_entered=true;g->terminal_glide_mode=true;
        g->attitude_recovery=false;g->control_bad_duration=0;g->control_good_duration=0;g->recovery_duration=0;
        g->airbrakes_deployed=false;
        robust_pid_reset(&g->taem_altitude_pid);robust_pid_reset(&g->speed_pid);
        taem_exec_sync(g,t,s,p,aero,cfg);
        g->phase=PHASE_TAEM;
        if(g->taem_exec.phase==TAEM_PHASE_S_TURN)
            return taem_s_turn_guidance(g,t,course,p,aero,cfg,dt);
        terminal_predict(g,t,course,p,aero,cfg,dt);
        GuidanceCommand c=atmospheric(t,t->ground_track_heading,0.0,v,0.0,false,PROFILE_TAEM);
        c.heading_control_enabled=false;c.has_target_aoa=true;
        c.target_aoa=clampd(v->entry_angle_of_attack,10.0,fmin(18.0,v->maximum_angle_of_attack));
        c.target_pitch=t->flight_path_angle+c.target_aoa;
        return stabilized(g,result_make(PHASE_TAEM,c,
            "TAEM terminal path invalidated; replanning energy and runway geometry without releasing MM305 ownership.",
            "The previous terminal path was discarded, but TAEM remains the sole terminal-area guidance owner."),
            t,v,s,dt);
    }

    /* Before the one-way MM304->MM305 latch, a discarded advisory preview may
       still return to bounded Entry planning. */
    terminal_invalidate_frozen_path(g);
    g->terminal_region_entered=false;g->terminal_glide_mode=false;g->terminal_rehearsal_mode=false;
    g->terminal_test_capture_active=false;g->terminal_reentry_after_ut=0.0;
    reset_taem_handoff_state(g);
    g->attitude_recovery=false;g->control_bad_duration=0;g->control_good_duration=0;g->recovery_duration=0;
    g->entry_control_plan_valid=false;g->entry_control_terminal_ready=false;
    g->entry_control_segment_until_ut=0;g->entry_reversal_scheduled=false;
    g->entry_reversal_ut=0;g->entry_reversal_range=0;g->entry_reversal_sign=0;g->entry_reversal_bank=0;
    g->has_entry_predictive_score=false;g->entry_predictive_score=0;g->airbrakes_deployed=false;
    g->entry_reversal_is_final=false;g->entry_final_reversal_pending=false;
    g->entry_final_reversal_completed=false;g->has_s_turn_reversal_requested=false;
    g->has_s_turn_leg_started=false;
    if(fabs(norm_signed_deg(t->roll))>=4)g->s_turn_sign=t->roll>=0?1:-1;
    robust_pid_reset(&g->taem_altitude_pid);robust_pid_reset(&g->speed_pid);
    g->phase=PHASE_ENTRY_ENERGY;
    GuidanceResult r=entry_program_guidance(g,t,state,course,p,aero,cfg,dt);
    snprintf(r.status,sizeof(r.status),"Pre-TAEM recovery: discarded advisory terminal geometry; MM304 must re-qualify the interface contract before TAEM ownership.");
    r.has_warning=true;
    snprintf(r.warning,sizeof(r.warning),"Previous advisory terminal preview discarded before TAEM ownership; an explicit MM304 recovery frame is permitted.");
    return r;
}

