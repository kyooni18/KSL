#define _POSIX_C_SOURCE 200809L
#include "landing.h"
#include "calibration_observation.h"
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdio.h>

static void near(double a,double b){assert(fabs(a-b)<1e-7);}
int main(void){
    VesselPhysicsModel historical;vessel_physics_init(&historical);
    VesselAeroSample prior={.q=100,.mach=.3,.aoa=10,.beta=0,.mass=1000,.force_per_q={2,4,0},.airbrakes=0,.observations=3,.trust=.15};
    vessel_physics_import_sample(&historical,&prior);
    assert(historical.count==1&&historical.samples[0].observations==3&&historical.samples[0].trust<1);
    historical.airbrakes=0;
    Vector3 prior_accel;double prior_confidence;
    assert(vessel_physics_aero(&historical,100,.3,10,0,1000,&prior_accel,&prior_confidence));
    near(prior_accel.x,.2);near(prior_accel.y,.4);near(prior_confidence,.15);

    VesselPhysicsModel m;vessel_physics_init(&m);
    PlanetModel p={.radius=600000,.gravitational_parameter=3.5e12};
    p.atmosphere_depth=70000;p.atmosphere_adiabatic_index=1.4;p.atmosphere_sample_count=2;
    p.atmosphere_altitude[0]=0;p.atmosphere_altitude[1]=1000;
    p.atmosphere_pressure[0]=p.atmosphere_pressure[1]=101325;
    p.atmosphere_density[0]=p.atmosphere_density[1]=1.225;
    near(planet_atmospheric_speed_of_sound(&p,500),sqrt(1.4*101325.0/1.225));

    VehicleState state={.ut=1,.mass=1000,.position={600100,0,0},.velocity={0,100,0}};
    Telemetry t={.ut=1,.mass=1000,.dry_mass=500,.dynamic_pressure=100,.mach=.3,.angle_of_attack=10,
        .physics_sample_valid=true,.has_airbrakes=true,.airbrakes=false,
        .has_force_vectors=true,.lift_vector={200,0,0},.drag_vector={0,-100,0},
        .has_center_of_mass=true,.center_of_mass={600100,0,0},.has_center_of_mass_root=true,.center_of_mass_root={1,-2,3},.has_torque=true,.has_inertia=true,
        .available_pitch_torque=100,.available_roll_torque=200,.available_yaw_torque=300,
        .pitch_moment_of_inertia=1000,.roll_moment_of_inertia=1000,.yaw_moment_of_inertia=1000,
        .has_controls=true,.has_body_pitch_rate=true,.control_pitch=.5,.current_thrust=1000};
    VesselPhysicsModel airbrake_model;vessel_physics_init(&airbrake_model);
    VesselAeroSample deployed={.q=100,.mach=.3,.aoa=10,.beta=0,.mass=1000,
        .force_per_q={60,4,0},.airbrakes=1,.observations=25,.trust=1.0};
    vessel_physics_import_sample(&airbrake_model,&deployed);
    Telemetry airbrake_t=t;airbrake_t.physics_sample_valid=false;
    vessel_physics_observe(&airbrake_model,&airbrake_t,&state,&p);
    assert(airbrake_t.physics_airbrake_model_available);
    assert(airbrake_t.physics_airbrake_model_confidence>.12);
    near(airbrake_t.physics_airbrake_drag_accel,6.0);
    vessel_physics_observe(&historical,&t,&state,&p);
    assert(historical.count==1&&historical.samples[0].trust==.15&&historical.samples[0].observations==3);
    assert(historical.live_observations==1&&historical.model_residual_confidence>0);
    /* Current-flight data is shadow evidence only. It must not move the
       certified prior that this same entry is using for prediction. */
    assert(vessel_physics_aero(&historical,100,.3,10,0,1000,&prior_accel,&prior_confidence));
    near(prior_accel.x,.2);near(prior_accel.y,.4);
    /* Repeated same-flight evidence may create a bounded predictor correction,
       but must still leave the certified lookup bit-for-bit unchanged. */
    Telemetry residual_t=t;VehicleState residual_state=state;
    for(int i=0;i<24;i++){residual_t.ut=residual_state.ut=2.0+i*.6;vessel_physics_observe(&historical,&residual_t,&residual_state,&p);}
    assert(historical.force_residual_confidence>.02&&historical.force_residual_per_q.x<0&&historical.force_residual_per_q.y<0);
    Vector3 certified_again;double certified_again_confidence;
    assert(vessel_physics_aero(&historical,100,.3,10,0,1000,&certified_again,&certified_again_confidence));
    near(certified_again.x,.2);near(certified_again.y,.4);
    LandingConfiguration residual_cfg=landing_configuration_default();
    TrajectoryCalibrationModel residual_cal={.density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.physics=&historical};
    AerodynamicModel residual_fallback={.lift_to_drag=.5,.ballistic_coefficient=1000,.confidence=.1};
    bool residual_observed=false;double residual_confidence=0,residual_uncertainty=0;
    Vector3 best=vessel_physics_force_best_estimate_config(&historical,100,.3,10,0,1000,false,false,0,residual_fallback,&residual_cal,&residual_cfg.vehicle,&residual_confidence,&residual_observed,&residual_uncertainty);
    assert(residual_observed&&best.x<certified_again.x&&best.y<certified_again.y);
    assert(best.x>=certified_again.x*.70-1e-9&&best.y>=certified_again.y*.70-1e-9);
    assert(residual_uncertainty>=.06&&residual_uncertainty<=.90);

    VesselAeroSample cert={.q=100,.mach=.3,.aoa=10,.beta=0,.mass=1000,.force_per_q={1,2,0},.airbrakes=0,.observations=4,.trust=.8};
    VesselAeroSample cert2={.q=100,.mach=.3,.aoa=20,.beta=0,.mass=1000,.force_per_q={1,4,0},.airbrakes=0,.observations=4,.trust=.8};
    vessel_physics_import_sample(&m,&cert);vessel_physics_import_sample(&m,&cert2);
    vessel_physics_observe(&m,&t,&state,&p);
    assert(m.count==2&&m.live_observations==1&&m.has_center_of_mass&&m.has_center_of_mass_root);near(m.center_of_mass_root.y,-2);near(m.center_of_mass.x,state.position.x);
    near(m.authority[0],.1*RAD2DEG);near(m.authority[1],.2*RAD2DEG);near(m.authority[2],.3*RAD2DEG);
    Vector3 a;double confidence;
    assert(vessel_physics_aero(&m,100,.3,10,0,1000,&a,&confidence));near(a.x,.1);near(a.y,.2);near(confidence,.8);
    assert(vessel_physics_aero(&m,110,.3,10,0,1000,&a,&confidence));near(a.x,.11);near(a.y,.22);assert(confidence>0&&confidence<.8);
    assert(vessel_physics_aero(&m,100,.3,10,0,500,&a,&confidence));near(a.y,.4);near(confidence,.8);
    /* Local confidence is geometric/reliability coverage, not a count-to-trust
       curve. Repeating an identical independent cell cannot improve it. */
    VesselPhysicsModel repeated_lookup;vessel_physics_init(&repeated_lookup);
    VesselAeroSample repeated=cert;repeated.observations=100;
    vessel_physics_import_sample(&repeated_lookup,&repeated);repeated_lookup.airbrakes=0;
    double repeated_confidence=0;Vector3 repeated_a;
    assert(vessel_physics_aero(&repeated_lookup,100,.3,10,0,1000,&repeated_a,&repeated_confidence));
    near(repeated_confidence,.8);
    assert(vessel_physics_aero(&repeated_lookup,100*sqrt(2.0),.3,10,0,1000,&repeated_a,&repeated_confidence));
    near(repeated_confidence,.4);
    assert(!vessel_physics_aero(&repeated_lookup,200,.3,10,0,1000,&repeated_a,&repeated_confidence));near(repeated_confidence,0);
    assert(!vessel_physics_aero(&m,1000,.3,10,0,1000,&a,&confidence));
    assert(!vessel_physics_aero(&m,100,5,10,0,1000,&a,&confidence));near(confidence,0);
    VesselPhysicsModel configured;vessel_physics_init(&configured);
    VesselAeroSample clean={.q=100,.mach=.3,.aoa=10,.mass=1000,.force_per_q={1,2,0},.airbrakes=0,.observations=2,.trust=.8};
    VesselAeroSample speedbrake={.q=100,.mach=.3,.aoa=10,.mass=1000,.force_per_q={1,4,0},.airbrakes=1,.observations=2,.trust=.8};
    vessel_physics_import_sample(&configured,&clean);vessel_physics_import_sample(&configured,&speedbrake);
    assert(configured.count==2);configured.airbrakes=1;
    assert(vessel_physics_aero(&configured,100,.3,10,0,1000,&a,&confidence));near(a.y,.4);
    configured.airbrakes=0;assert(vessel_physics_aero(&configured,100,.3,10,0,1000,&a,&confidence));near(a.y,.2);
    bool observed=false;
    LandingConfiguration cfg=landing_configuration_default();
    TrajectoryCalibrationModel cal={.density_scale=1,.drag_scale=2,.lift_scale=3,.bank_effectiveness=2,.speed_of_sound=340,.speed_of_sound_scale=1,.physics=&m};
    AerodynamicModel fallback={2,100,.05};
    a=vessel_physics_force(&m,100,.3,10,0,1000,fallback,&cal,&cfg.vehicle,&confidence,&observed);
    assert(observed);near(a.x,.1);near(a.y,.2); /* Legacy scales must not multiply direct forces. */
    Vector3 acceleration=vessel_physics_acceleration(state.position,state.velocity,&p,a,90);
    Vector3 grav=vessel_physics_gravity(state.position,&p);
    near(acceleration.x,grav.x);near(acceleration.y,-.1);near(acceleration.z,-.2);
    a=vessel_physics_force(&m,100,8,10,0,1000,fallback,&cal,&cfg.vehicle,&confidence,&observed);near(confidence,.05);assert(a.x>0&&!observed);
    t.ut=state.ut=1.1;t.angle_of_attack=20;t.lift_vector.x=400;t.body_pitch_rate=.5;
    t.mass=state.mass=999.9;vessel_physics_observe(&m,&t,&state,&p);
    assert(m.count==2&&m.mass_flow>0&&m.authority_confidence[0]>.25);
    assert(vessel_physics_aero(&m,100,.3,15,0,1000,&a,&confidence));near(a.y,.3);
    double mass=vessel_physics_burn_mass(&m,1000,1000,10);assert(mass<1000&&mass>500);
    near(vessel_physics_burn_mass(&m,501,1e9,10),500);
    m.thrust=0;near(vessel_physics_burn_mass(&m,1000,1000,10),mass);
    double angle=0,rate=0;vessel_physics_axis_step(&m,1,30,100,10,.1,&angle,&rate);assert(angle>0&&rate<=m.authority[1]*.1+1e-8);
    /* A huge instantaneous torque/inertia value is an actuator ceiling, not
       an ideal bank servo.  Predictor roll response must stay inside the
       achieved closed-loop bandwidth used by the real bridge. */
    m.authority[1]=1200;m.authority_q=100;angle=rate=0;
    vessel_physics_axis_step(&m,1,30,100,10,.1,&angle,&rate);
    assert(angle>0&&rate>0&&rate<3.0);
    t.ut=state.ut=1.2;t.has_force_vectors=false;vessel_physics_observe(&m,&t,&state,&p);assert(m.count==2);
    /* A degraded startup frame must not replace the current state or erase the
       learned bank when its synthetic mass later snaps to the real value. */
    t.ut=state.ut=1.3;t.mass=state.mass=20000;t.physics_sample_valid=false;
    vessel_physics_observe(&m,&t,&state,&p);assert(m.count==2);near(m.state.mass,999.9);
    t.ut=state.ut=1.4;t.mass=state.mass=1500;t.physics_sample_valid=true;
    vessel_physics_observe(&m,&t,&state,&p);assert(m.count==2);near(m.state.mass,1500);
    LandingSnapshot snapshot={0};landing_snapshot_set_sample(&snapshot,&t,&state);assert(snapshot.has_vehicle_state);near(snapshot.telemetry.ut,snapshot.vehicle_state.ut);
    JsonWriter w;jw_init(&w);snapshot_log_json(&w,&snapshot);
    JsonToken tokens[4096];JsonDoc doc;assert(json_parse(w.data,tokens,4096,&doc)>0);
    int telemetry=json_object_get(&doc,0,"telemetry"),vehicle=json_object_get(&doc,0,"vehicleState");
    near(json_number(&doc,json_object_get(&doc,telemetry,"ut"),-1),json_number(&doc,json_object_get(&doc,vehicle,"ut"),-2));jw_free(&w);
    state.ut-=8;landing_snapshot_set_sample(&snapshot,&t,&state);assert(!snapshot.has_vehicle_state);
    t.ut=state.ut=.5;vessel_physics_observe(&m,&t,&state,&p);assert(m.count==2);

    /* Prior-flight cells derive a Mach-scheduled operational envelope without
       importing current-flight shadow observations into it. */
    VesselPhysicsModel book;vessel_physics_init(&book);
    const double machs[4]={.3,1.1,3.0,6.2};
    for(int i=0;i<4;i++){double lf=0,df=1;aerodynamic_force_factors_mach(machs[i],12,&cfg.vehicle,&lf,&df);VesselAeroSample s={.q=500,.mach=machs[i],.aoa=12,.mass=700,.force_per_q={df,.4*lf,0},.airbrakes=0,.observations=5,.trust=.85};vessel_physics_import_sample(&book,&s);}
    AerodynamicEnvelope certified_env;AerodynamicModel certified_plan;
    assert(vessel_physics_derive_envelope(&book,&cfg.vehicle,&certified_env,&certified_plan));
    assert(certified_plan.confidence>0);near(certified_plan.lift_to_drag,.4);near(certified_plan.ballistic_coefficient,700);
    /* Four represented Mach regions with trust .85 have exactly .85 global
       coverage confidence when their identified factors have zero scatter. */
    near(book.certified_confidence,.85);
    near(book.certified_uncertainty,.15);
    near(certified_plan.confidence,.85);

    VesselPhysicsModel hyp_only;vessel_physics_init(&hyp_only);
    for(int i=0;i<4;i++){double lf=0,df=1;aerodynamic_force_factors_mach(6.2,12,&cfg.vehicle,&lf,&df);VesselAeroSample s={.q=500,.mach=6.2,.aoa=12,.mass=700,.force_per_q={df,.4*lf,0},.airbrakes=0,.observations=100,.trust=.95};vessel_physics_import_sample(&hyp_only,&s);}
    AerodynamicEnvelope hyp_env;AerodynamicModel hyp_plan;
    assert(vessel_physics_derive_envelope(&hyp_only,&cfg.vehicle,&hyp_env,&hyp_plan));
    /* Dense repetition in only one of four Mach regions cannot manufacture
       global certainty. Coverage is explicit: .95/4 confidence, hence .7625
       epistemic uncertainty; Entry planning covers one of its two target
       high-Mach regions. */
    near(hyp_only.certified_confidence,.2375);
    near(hyp_only.certified_uncertainty,.7625);
    near(hyp_plan.confidence,.475);

    VesselPhysicsModel dense_book;vessel_physics_init(&dense_book);
    for(int i=0;i<4;i++){double lf=0,df=1;aerodynamic_force_factors_mach(machs[i],12,&cfg.vehicle,&lf,&df);VesselAeroSample s={.q=500,.mach=machs[i],.aoa=12,.mass=700,.force_per_q={df,.4*lf,0},.airbrakes=0,.observations=100,.trust=.95};vessel_physics_import_sample(&dense_book,&s);}
    AerodynamicEnvelope dense_env;AerodynamicModel dense_plan;
    assert(vessel_physics_derive_envelope(&dense_book,&cfg.vehicle,&dense_env,&dense_plan));
    near(dense_book.certified_confidence,.95);
    near(dense_book.certified_uncertainty,.05);
    near(dense_plan.confidence,.95);
    Vector3 deployed_lookup;double deployed_lookup_confidence=0;
    assert(!vessel_physics_aero_config(&book,500,3.0,12,0,700,false,false,1,&deployed_lookup,&deployed_lookup_confidence));
    VesselAeroSample deployed_cell={.q=500,.mach=3.0,.aoa=12,.mass=700,.force_per_q={1.45,.35,0},.airbrakes=1,.observations=3,.trust=.8};
    vessel_physics_import_sample(&book,&deployed_cell);
    assert(vessel_physics_aero_config(&book,500,3.0,12,0,700,false,false,1,&deployed_lookup,&deployed_lookup_confidence));
    assert(deployed_lookup.x>(500.0/700.0));

    /* Explicit stress multipliers perturb a simulation, not the data book. */
    book.airbrakes=0;TrajectoryCalibrationModel stress={.physics=&book,.density_scale=1,.drag_scale=1,.lift_scale=1,.stress_drag_scale=1.2,.stress_lift_scale=.8};
    Vector3 nominal_stress_cell;double nominal_stress_confidence=0;
    assert(vessel_physics_aero_config(&book,500,3.0,12,0,700,false,false,0,&nominal_stress_cell,&nominal_stress_confidence));
    bool stress_observed=false;Vector3 stressed=vessel_physics_force_config(&book,500,3.0,12,0,700,false,false,0,certified_plan,&stress,&cfg.vehicle,&confidence,&stress_observed);
    assert(stress_observed);near(stressed.x,nominal_stress_cell.x*1.2);near(stressed.y,nominal_stress_cell.y*.8);

    SpeedbrakeController sb;speedbrake_controller_reset(&sb,false);
    assert(!speedbrake_controller_update(&sb,20000,10000,120000,100000,45000,true,.1));
    assert(speedbrake_controller_update(&sb,20000,10000,120000,100000,45000,false,.1));
    for(int i=0;i<20;i++)speedbrake_controller_update(&sb,5000,10000,-100000,100000,45000,false,.1);
    assert(!sb.deployed);

    /* The manual glide-test path must never command an airbrake pulse. */
    GlideCalibrationMachine glide_machine;Telemetry glide_t={0};
    glide_t.ut=100;glide_t.radar_altitude=10000;glide_t.dynamic_pressure=1000;glide_t.g_force=1;glide_t.vertical_speed=-10;glide_t.true_air_speed=200;glide_t.flight_path_angle=-5;
    snprintf(glide_t.vessel_situation,sizeof(glide_t.vessel_situation),"flying");
    glide_calibration_start(&glide_machine,&glide_t,&cfg.calibration);
    glide_machine.angle_index=glide_machine.angle_count/2;glide_machine.state_start_ut=100;
    glide_t.ut=100+cfg.calibration.settling_duration+cfg.calibration.sampling_duration*.80;
    bool glide_sampling=false,glide_finished=false;double glide_progress=0,glide_aoa=0;
    GuidanceResult glide_result=glide_calibration_update(&glide_machine,&glide_t,&cfg.vehicle,&cfg.calibration,&glide_sampling,&glide_finished,&glide_progress,&glide_aoa);
    assert(glide_sampling&&!glide_finished&&!glide_result.command.airbrakes);
    trajectory_clear(&glide_result.reference);

    /* Direct-force telemetry is the normal modern path.  Its sample quality is
       a state/model-domain question, not a fixed angular-rate/tracking gate. */
    InFlightTrajectoryCalibrator tc;trajectory_calibrator_init(&tc);
    LandingConfiguration direct_cfg=landing_configuration_default();
    Telemetry direct_t={.ut=10,.mass=1000,.dynamic_pressure=1000,.mach=.3,.angle_of_attack=10,
        .mean_altitude=100,.true_air_speed=100,.horizontal_speed=100,
        .atmospheric_density=1.225,.speed_of_sound=340,.g_force=1,.physics_sample_valid=true,
        .has_force_vectors=true,.has_airbrakes=true,.airbrakes=false};
    VehicleState direct_state={.ut=10,.mass=1000,.position={600100,0,0},.velocity={0,100,0}};
    GuidanceCommand direct_command;guidance_command_init(&direct_command);
    AerodynamicEnvelope direct_env={0};
    for(int i=0;i<4;i++)direct_env.regimes[i]=(AerodynamicModel){.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=.2};

    double force_lf=0,force_df=1;
    aerodynamic_force_factors_mach(direct_t.mach,direct_t.angle_of_attack,&direct_cfg.vehicle,&force_lf,&force_df);
    direct_t.drag_force=direct_t.dynamic_pressure*direct_t.mass*force_df/700.0;
    direct_t.lift_force=direct_t.drag_force*(.4*fmax(DBL_MIN,fabs(force_lf))/force_df);
    direct_t.has_body_pitch_rate=direct_t.has_body_roll_rate=direct_t.has_body_yaw_rate=true;
    direct_t.has_angle_of_attack_rate=true;
    snprintf(direct_t.vessel_situation,sizeof(direct_t.vessel_situation),"flying");

    CalibrationObservationEnvelope short_observation=
        calibration_observation_envelope(&direct_t,&direct_cfg.vehicle,&direct_cfg.calibration,.05);
    CalibrationObservationEnvelope long_observation=
        calibration_observation_envelope(&direct_t,&direct_cfg.vehicle,&direct_cfg.calibration,.20);
    assert(short_observation.passive_aero_usable&&short_observation.trajectory_usable);
    assert(long_observation.information_increment>short_observation.information_increment);
    assert(long_observation.information_blend>short_observation.information_blend);
    assert(long_observation.aerodynamic_response_time_s==short_observation.aerodynamic_response_time_s);

    Telemetry rejected=direct_t;
    rejected.current_thrust=1;
    assert(!calibration_observation_envelope(&rejected,&direct_cfg.vehicle,&direct_cfg.calibration,.1).passive_aero_usable);
    rejected=direct_t;rejected.has_controls=true;rejected.control_roll=1.0;
    assert(!calibration_observation_envelope(&rejected,&direct_cfg.vehicle,&direct_cfg.calibration,.1).passive_aero_usable);
    rejected=direct_t;rejected.dynamic_pressure=direct_cfg.calibration.minimum_dynamic_pressure*.5;
    assert(!calibration_observation_envelope(&rejected,&direct_cfg.vehicle,&direct_cfg.calibration,.1).passive_aero_usable);

    /*
       The measured force vector is already expressed at the actual Mach/AoA.
       High body or coordinate rates therefore remain usable instead of being
       rejected by arbitrary 8/12 degree-per-second gates.
    */
    AdaptiveFlightCalibrator rate_ac;adaptive_calibrator_init(&rate_ac,&direct_cfg.vehicle);
    AerodynamicModel rate_current={0},rate_planning={0};AerodynamicEnvelope rate_env={0};
    VehicleProfile rate_recommended={0};CalibrationSnapshot rate_snap;
    direct_t.roll_rate=20;direct_t.angle_of_attack_rate=10;
    adaptive_calibrator_update(&rate_ac,&direct_t,&direct_cfg.calibration,&direct_cfg.vehicle,&direct_command,.1,
        true,false,false,CAL_IDLE,0,0,NULL,NULL,&rate_current,&rate_planning,&rate_env,&rate_recommended,&rate_snap);
    assert(rate_ac.accepted==1);
    direct_t.ut+=.2;direct_t.roll_rate=0;direct_t.angle_of_attack_rate=0;direct_t.body_roll_rate=20;
    adaptive_calibrator_update(&rate_ac,&direct_t,&direct_cfg.calibration,&direct_cfg.vehicle,&direct_command,.1,
        true,false,false,CAL_IDLE,0,0,NULL,NULL,&rate_current,&rate_planning,&rate_env,&rate_recommended,&rate_snap);
    assert(rate_ac.accepted==2);

    /*
       Information confidence is cadence-independent for a constant physical
       observation because the update integrates dt/(V/|a_aero|).
    */
    AdaptiveFlightCalibrator fine_ac,coarse_ac;
    adaptive_calibrator_init(&fine_ac,&direct_cfg.vehicle);
    adaptive_calibrator_init(&coarse_ac,&direct_cfg.vehicle);
    for(int i=0;i<10;i++){
        direct_t.ut=30+i*.1;
        adaptive_calibrator_update(&fine_ac,&direct_t,&direct_cfg.calibration,&direct_cfg.vehicle,&direct_command,.1,
            true,false,false,CAL_IDLE,0,0,NULL,NULL,&rate_current,&rate_planning,&rate_env,&rate_recommended,&rate_snap);
    }
    for(int i=0;i<5;i++){
        direct_t.ut=40+i*.2;
        adaptive_calibrator_update(&coarse_ac,&direct_t,&direct_cfg.calibration,&direct_cfg.vehicle,&direct_command,.2,
            true,false,false,CAL_IDLE,0,0,NULL,NULL,&rate_current,&rate_planning,&rate_env,&rate_recommended,&rate_snap);
    }
    assert(fabs(fine_ac.confidence[0]-coarse_ac.confidence[0])<1e-12);

    InFlightTrajectoryCalibrator rate_tc;trajectory_calibrator_init(&rate_tc);
    TrajectoryCalibrationModel rate_model={0};
    direct_t.ut=direct_state.ut=50;direct_t.roll_rate=20;direct_t.angle_of_attack_rate=10;
    rate_model=trajectory_calibrator_update(&rate_tc,&direct_t,&direct_state,&p,&direct_cfg.site,&direct_env,
        &direct_cfg.vehicle,&direct_command,&direct_cfg.calibration,true,.1);
    assert(rate_model.accepted_samples==1&&rate_model.confidence>0);
    direct_t.ut=direct_state.ut=50.2;direct_t.roll_rate=0;direct_t.angle_of_attack_rate=0;direct_t.body_roll_rate=20;
    rate_model=trajectory_calibrator_update(&rate_tc,&direct_t,&direct_state,&p,&direct_cfg.site,&direct_env,
        &direct_cfg.vehicle,&direct_command,&direct_cfg.calibration,true,.1);
    assert(rate_model.accepted_samples==2);
    trajectory_calibrator_clear(&rate_tc);

    direct_t.roll_rate=0;direct_t.angle_of_attack_rate=0;direct_t.body_roll_rate=0;
    TrajectoryCalibrationModel direct_model={0};
    for(int i=0;i<20;i++){
        direct_t.ut=direct_state.ut=60+i*.1;
        direct_model=trajectory_calibrator_update(&tc,&direct_t,&direct_state,&p,&direct_cfg.site,&direct_env,
            &direct_cfg.vehicle,&direct_command,&direct_cfg.calibration,true,.1);
    }
    assert(direct_model.accepted_samples==20&&direct_model.confidence>0);
    trajectory_calibrator_clear(&tc);
    /* kRPC transport/session behavior is covered by the C-Nano fake-transport
       gate. Vessel-physics tests intentionally remain pure and never open a
       live serial endpoint. */
    puts("Vessel physics tests passed");return 0;
}
