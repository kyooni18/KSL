/* Inspect the actual cheap orbital timing implementation, without a live link
   or the expensive atmospheric certification search. */
#include "../CLanding/prediction/predictor.c"
#include <assert.h>
int main(void){
 PlanetModel p={0};p.radius=600000;p.gravitational_parameter=3.5316e12;p.rotational_speed=.0002915709;p.atmosphere_depth=70000;p.north_axis=v3(0,0,1);p.prime_meridian_at_epoch=v3(1,0,0);
 LandingConfiguration cfg=landing_configuration_default();
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
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance));
 EntryPrediction overshot=usable;
 overshot.taem_distance=90000;overshot.taem_along_track=4000;
 overshot.taem_cross_track=89900;overshot.taem_course=195;
 assert(!deorbit_witness_closing(&overshot,&cfg.site));
 assert(!deorbit_guided_candidate_better(&improved,&overshot,&previous,&usable,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance));
 EntryPrediction too_close=usable;too_close.taem_distance=8000;
 too_close.taem_along_track=1000;too_close.taem_cross_track=-8000;
 assert(!deorbit_witness_inlet_geometry(&too_close,&p,&cfg.site,&cfg.guidance));
 assert(!deorbit_guided_candidate_better(&improved,&too_close,&previous,&usable,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance));
 EntryPrediction unsafe=usable;unsafe.entry_flight_path_angle=-15;
 unsafe.taem_dynamic_interface_captured=true;
 unsafe.taem_ownership_boundary_missed=false;
 assert(!deorbit_guided_candidate_better(&improved,&unsafe,&previous,&usable,
     &p,&cfg.site,&cfg.vehicle,&cfg.guidance));
 printf("selected nearby OMS burn %.1fs, miss %.1fm\n",best.burn_start,best.entry_miss);
 printf("equatorial horizon %.1fs (orbit %.1fs); steered peri %.1fm; lateral response %.1fm\n",horizon,period,steered.peri,steered.entry_crossrange-straight.entry_crossrange);
}
