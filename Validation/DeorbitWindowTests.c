/* Inspect the actual cheap orbital timing implementation, without a live link
   or the expensive atmospheric certification search. */
#include "../CLanding/prediction/predictor.c"
#include <assert.h>
#include "sim_telemetry.h"
/* Scheduled entry must use the captured whole-vessel model, even when no
   online force samples exist. A scalar residual must not distort that table. */
static void entry_policy_model_parity(void){
 LandingConfiguration cfg=landing_configuration_default();
 snprintf(cfg.vehicle.model_id,sizeof(cfg.vehicle.model_id),"STS");
 cfg.guidance.taem_energy_law=true;
 TerminalModelSourceFiles files={
  "ShuttleSim/reference-model/kerbin_atmosphere_identified.csv",
  "ShuttleSim/reference-model/sts_aero_cells20261015_live_only.csv",
  "none","ShuttleSim/reference-model/stsn_attitude_identified.ini"};
 TerminalModel *model=calloc(1,sizeof(*model));char reason[256];PlanetModel planet;
 assert(model&&shuttle_sim_load_planet(files.atmosphere_csv,&planet,reason,sizeof(reason)));
 assert(terminal_model_capture(model,&files,&cfg,9001,reason,sizeof(reason)));
 assert(terminal_model_align_planet_frame(model,&planet,reason,sizeof(reason)));
 AerodynamicModel fallback={.lift_to_drag=.3,.ballistic_coefficient=900,.confidence=.1};
 AerodynamicEnvelope env={.regimes={fallback,fallback,fallback,fallback}};
 TrajectoryCalibrationModel cal={.density_scale=1.2,.speed_of_sound_scale=.9,
  .drag_scale=2,.lift_scale=.5,.bank_effectiveness=1};
 VehicleState state={.ut=148743700,.mass=25602};
 GeoPoint geo={25,-100,32000};
 state.position=predictor_inertial_position(geo,&planet,state.ut);
 Vector3 up=vnorm(state.position,v3(0,0,1));
 Vector3 east=vnorm(vcross(planet.north_axis,up),v3(0,1,0));
 Vec3 native=world_lla_to_inertial(&model->world,geo.latitude*DEG2RAD,
  geo.longitude*DEG2RAD,geo.altitude,state.ut);
 AtmosphereSample air=world_atmosphere_sample_state(&model->world,native,state.ut);
 for(double mach=2.5;mach<=5;mach+=.5)for(double alpha=18;alpha<=28;alpha+=5){
  state.velocity=vadd(vscale(east,mach*air.speed_of_sound_mps),
   vcross(planet_rotation_vector(&planet),state.position));
  AeroContext ctx={.p=&planet,.e=&env,.c=&cal,.v=&cfg.vehicle,
   .entry_table=model,.aoa=alpha,.bank=30,.mass=state.mass,.ut=state.ut};
  AtmosState a=atmosphere(state.position,state.velocity,&ctx);
  double lift,drag;aero_force_per_q(&model->aero,a.q,mach,alpha,&lift,&drag);
  assert(fabs(a.density/air.density_kg_m3-1)<1e-9);
  assert(fabs(a.mach-mach)<1e-9);
  assert(fabs(a.drag_accel*state.mass/a.q-drag)<1e-8);
  assert(fabs(a.lift_accel*state.mass/a.q-lift)<1e-8);
 }
 EntryPrediction a=predictor_simulate_entry_policy_reversal(state,&planet,fallback,&env,&cal,
  &cfg.vehicle,&cfg.site,&cfg.guidance,30,23,1,0,NULL,.01,model);
 cal.density_scale=.7;cal.speed_of_sound_scale=1.2;cal.drag_scale=.5;cal.lift_scale=2;
 EntryPrediction b=predictor_simulate_entry_policy_reversal(state,&planet,fallback,&env,&cal,
  &cfg.vehicle,&cfg.site,&cfg.guidance,30,23,1,0,NULL,.01,model);
 assert(vmag(vsub(a.final_state.velocity,b.final_state.velocity))<1e-9);
 entry_prediction_clear(&a);entry_prediction_clear(&b);free(model);
}
int main(void){
 entry_policy_model_parity();
 PlanetModel p={0};p.radius=600000;p.gravitational_parameter=3.5316e12;p.rotational_speed=.0002915709;p.atmosphere_depth=70000;p.north_axis=v3(0,0,1);p.prime_meridian_at_epoch=v3(1,0,0);
 LandingConfiguration cfg=landing_configuration_default();
 GeoPoint west=destination_point((GeoPoint){cfg.site.latitude,
     cfg.site.longitude,cfg.site.altitude},270.0,5000.0,p.radius);
 DeorbitEnergyMargin runway_margin={0};
 deorbit_passive_runway_margin(west,&cfg.site,p.radius,&runway_margin);
 assert(runway_margin.impact_runway_end==1&&
     runway_margin.impact_along_track>7000.0);
 cfg.site.allow_reciprocal_runway=false;
 deorbit_passive_runway_margin(west,&cfg.site,p.radius,&runway_margin);
 assert(runway_margin.impact_runway_end==0&&
     runway_margin.impact_along_track<-4000.0);
 cfg.site.allow_reciprocal_runway=true;
 cfg.guidance.target_post_burn_periapsis_altitude=50000;
 cfg.guidance.target_deorbit_capture_radius=15000;
 VehicleState state={0};state.position=v3(686000,0,0);state.velocity=v3(0,sqrt(p.gravitational_parameter/686000),0);state.mass=26000;
 double period=2*acos(-1)*sqrt(pow(686000,3)/p.gravitational_parameter);
 double horizon=deorbit_search_horizon(state,period,&p,&cfg.guidance);
 assert(horizon<2*period);
 VehicleState inclined=state;inclined.velocity=v3(0,state.velocity.y*.5,state.velocity.y*sqrt(.75));
 assert(deorbit_search_horizon(inclined,period,&p,&cfg.guidance)>horizon);
 VehicleState eccentric=state;eccentric.velocity=vscale(state.velocity,1.1);
 assert(deorbit_search_horizon(eccentric,period,&p,&cfg.guidance)>horizon);
 DeorbitTimingProbe early={.valid=true,.closing=true,.entry_miss=10000,.burn_start=100},late={.valid=true,.closing=true,.entry_miss=1,.burn_start=10000};
 assert(deorbit_timing_probe_better(&early,&late,true,&cfg.guidance));
 TrajectoryCalibrationModel cal={0};DeorbitTimingProbe straight={0},steered={0};
 assert(deorbit_timing_probe_start(&straight,state,100,0,&p,&cal,&cfg.site,&cfg.guidance,60000,NULL));
 assert(deorbit_timing_probe_start(&steered,state,100,5,&p,&cal,&cfg.site,&cfg.guidance,60000,NULL));
 assert(fabs(steered.peri-50000)<100);
 assert(fabs(steered.entry_crossrange-straight.entry_crossrange)>1);
 DeorbitTimingProbe best={0};bool found=false;
 for(int i=0;i<=360;i++){
  DeorbitTimingProbe probe={0};
  if(deorbit_timing_probe_start(&probe,state,cfg.guidance.minimum_planning_lead_time+horizon*i/360,0,&p,&cal,&cfg.site,&cfg.guidance,60000,NULL)&&deorbit_timing_probe_better(&probe,&best,found,&cfg.guidance)){best=probe;found=true;}
 }
 assert(found&&best.burn_start<=horizon+cfg.guidance.minimum_planning_lead_time);
 /* A nearby deorbit opportunity may have a worse cheap orbital miss
    but a physically usable and still-closing MM305 acquisition range. The
    2026-10-12 fourth polar live plan was 153 km away at the Mach boundary;
    retiming one ~43 s entry-flight increment gave a 71 km inlet. */
 cfg.guidance.target_deorbit_capture_radius=100000;
 cfg.guidance.taem_interface_range=96500;
 cfg.guidance.hac_radius=12000;
 cfg.guidance.final_approach_distance=8000;
 Candidate previous={.peri=50000,.burn_ut=100};
 Candidate improved={.peri=50000,.burn_ut=143};
 EntryPrediction poor={.entered_atmosphere=true,.reached_taem=true,
  .entry_flight_path_angle=-2,.entry_range=1019000,
  .peak_dynamic_pressure=6000,.peak_g_load=2,.taem_altitude=25750,
  .taem_speed=863,.taem_distance=153500,.taem_along_track=8000,
  .taem_cross_track=-153300,.taem_course=185,
  .taem_interface_energy_margin=-270000,.taem_ownership_boundary_missed=true};
 EntryPrediction usable=poor;usable.taem_distance=71400;
 usable.taem_along_track=1400;usable.taem_cross_track=-71400;
 usable.taem_interface_energy_margin=-246000;
 assert(deorbit_witness_inlet_geometry(&usable,&p,&cfg.site,&cfg.guidance));
 assert(!deorbit_witness_inlet_geometry(&poor,&p,&cfg.site,&cfg.guidance));
 assert(deorbit_guided_candidate_better(&improved,&usable,&previous,&poor,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance,NAN));
 EntryPrediction overshot=usable;
 overshot.taem_distance=90000;overshot.taem_along_track=4000;
 overshot.taem_cross_track=89900;overshot.taem_course=195;
 assert(!deorbit_witness_closing(&overshot,&cfg.site));
 assert(!deorbit_guided_candidate_better(&improved,&overshot,&previous,&usable,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance,NAN));
 EntryPrediction too_close=usable;too_close.taem_distance=8000;
 too_close.taem_along_track=1000;too_close.taem_cross_track=-8000;
 assert(!deorbit_witness_inlet_geometry(&too_close,&p,&cfg.site,&cfg.guidance));
 assert(!deorbit_guided_candidate_better(&improved,&too_close,&previous,&usable,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance,NAN));
 EntryPrediction unsafe=usable;unsafe.entry_flight_path_angle=-15;
 unsafe.taem_dynamic_interface_captured=true;
 unsafe.taem_ownership_boundary_missed=false;
 assert(!deorbit_guided_candidate_better(&improved,&unsafe,&previous,&usable,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance,NAN));
 /* The 34-degree return's course fails the perpendicular inlet heuristic,
    but a safe closing inlet still deserves a native MM305 continuation. */
 EntryPrediction oblique=usable;oblique.taem_course=60;
 oblique.taem_along_track=-40000;oblique.taem_cross_track=-40000;
 assert(!deorbit_witness_inlet_geometry(&oblique,&p,&cfg.site,&cfg.guidance));
 assert(deorbit_witness_terminal_trial(&improved,&oblique,&p,&cfg.site,
     &cfg.vehicle,&cfg.guidance));
 assert(!deorbit_witness_terminal_trial(&improved,&unsafe,&p,&cfg.site,
     &cfg.vehicle,&cfg.guidance));
 /* Among two completed uncertified attempts, prefer the nearer ground
    endpoint even if the other candidate better matches the nominal range. */
 EntryPrediction near=usable,nominal=usable;
 near.shadow_guidance_used=nominal.shadow_guidance_used=true;
 near.taem_distance=70000;nominal.taem_distance=96500;
 near.final_state.ut=nominal.final_state.ut=1000;
 near.final_state.position=predictor_inertial_position((GeoPoint){
     cfg.site.latitude,cfg.site.longitude+.1,50},&p,1000);
 nominal.final_state.position=predictor_inertial_position((GeoPoint){
     cfg.site.latitude,cfg.site.longitude+1,50},&p,1000);
 previous.executable_witness_evaluated=improved.executable_witness_evaluated=true;
 /* The nearer ground endpoint cannot compensate for a missed MM305
    ownership boundary or a larger terminal energy deficit. */
 nominal.taem_dynamic_interface_captured=true;
 nominal.taem_ownership_boundary_missed=false;
 assert(deorbit_guided_candidate_better(&previous,&nominal,&improved,&near,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance,NAN));
 nominal.taem_dynamic_interface_captured=false;
 nominal.taem_interface_energy_margin=-200000;
 assert(deorbit_guided_candidate_better(&previous,&nominal,&improved,&near,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance,NAN));
 nominal.taem_interface_energy_margin=near.taem_interface_energy_margin;
 assert(deorbit_guided_candidate_better(&improved,&near,&previous,&nominal,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance,NAN));
 /* A closer point high above KSC is not a completed approach. */
 near.final_state.position=predictor_inertial_position((GeoPoint){
     cfg.site.latitude,cfg.site.longitude,15000},&p,1000);
 assert(!isfinite(deorbit_guided_terminal_miss(&improved,&near,&p,&cfg.site)));
 printf("selected nearby OMS burn %.1fs, miss %.1fm\n",best.burn_start,best.entry_miss);
 printf("equatorial horizon %.1fs (orbit %.1fs); steered peri %.1fm; lateral response %.1fm\n",horizon,period,steered.peri,steered.entry_crossrange-straight.entry_crossrange);
}
