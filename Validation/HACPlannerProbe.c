#include "../CLanding/guidance.c"
#include <stdio.h>

int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    PlanetModel p={.radius=600000.0,.gravitational_parameter=3531600000000.0};
    AerodynamicModel aero={.lift_to_drag=.4,.ballistic_coefficient=700,.confidence=1};
    GuidanceMachine g;guidance_machine_init(&g);
    terminal_glide_initialize(&g,&cfg.vehicle,&cfg.guidance);
    Telemetry t;telemetry_init(&t);
    t.ut=66564.289814;
    t.mean_altitude=19998.442;t.radar_altitude=19998.442;
    t.mass=40252.917969;t.true_air_speed=725.767;t.horizontal_speed=721.579;
    t.vertical_speed=-77.855;t.flight_path_angle=-6.158087;
    t.bank_effectiveness=1;t.lift_force=443563.385621;t.drag_force=626650.101017;
    t.angle_of_attack=18;t.angle_of_attack_rate=0;t.dynamic_pressure=10517.949802;
    t.mach=2.433371;t.latitude=-0.049940569;t.longitude=-75.492297034;
    t.ground_track_heading=180.8461;t.heading=180.8461;t.roll=0;t.roll_rate=0;
    t.g_force=1.94491813506751;t.stall_fraction=0;t.stall_fraction_is_measured=true;t.physics_certified_uncertainty=.288567364929955;
    t.runway_along_track=-8000.20612873747;t.runway_cross_track=13.9673015230379;

    double preferred=NAN;EnergyPathEnvelope env={0};
    double slope=terminal_test_energy_selected_slope(&g,&t,&p,aero,&cfg,&preferred,&env);
    double bank=fmin(fabs(cfg.vehicle.maximum_bank_angle),dynamic_bank_limit(&t,&cfg.vehicle));
    double live=live_turn_radius(&t,aero,&cfg.vehicle,bank);
    double projected=terminal_test_projected_turn_radius(&t,&p,aero,&cfg.vehicle,bank);
    double lift=live_lift_accel(&t,aero,&cfg.vehicle);
    double drag=terminal_expected_speed_loss_accel(&g,&t,&p,aero,&cfg.vehicle);
    double minSlope=0,maxSlope=0; terminal_path_slope_bounds(&t,&cfg.vehicle,&cfg.guidance,&minSlope,&maxSlope);
    printf("preflare alt=%.3f target=%.3f min=%.3f\n",g.terminal_test_preflare_altitude,g.terminal_test_preflare_target_speed,g.terminal_test_preflare_min_speed);
    printf("energy slope=%.9f preferred=%.3f env valid=%d feasible=%d margin=%.3f available=%.3f dragWork=%.3f uncertainty=%.3f\n",
        slope,preferred,env.valid,env.feasible,env.energy.margin,
        env.available_specific_energy,env.modeled_drag_work,
        env.uncertainty_specific_energy);
    printf("slope bounds=%.6f..%.6f bank=%.3f liveTurn=%.3f projectedTurn=%.3f lift=%.6f drag=%.6f\n",minSlope,maxSlope,bank,live,projected,lift,drag);


    double gate_height=fmax(0.0,g.terminal_test_preflare_altitude);
    double gate_ground_distance=gate_height/tan(slope*DEG2RAD);
    GeoPoint site_point={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    GeoPoint current_point={t.latitude,t.longitude,t.mean_altitude};
    double current_e=0,current_n=0;
    local_offsets(site_point,current_point,p.radius,&current_e,&current_n);
    double effectiveness=t.bank_effectiveness>0?t.bank_effectiveness:1.0;
    double effective_bank=fmin(bank*effectiveness,nextafter(90.0,0.0));
    double max_lateral=lift*fabs(sin(effective_bank*DEG2RAD));
    double circle_design_speed=fmax(g.terminal_test_preflare_target_speed,
        g.terminal_test_preflare_min_speed);
    double design_scale=circle_design_speed/t.true_air_speed;
    double min_turn=fmin(live,projected);
    double design_turn=min_turn*design_scale*design_scale;
    double rate_cap=hac_course_rate_cap_for_speed(circle_design_speed);
    double tracking_radius=circle_design_speed/(rate_cap*DEG2RAD);
    double min_radius=fmax(design_turn,tracking_radius);
    double max_radius=fmin(p.radius,fmax(min_radius,preferred));
    double height=fmax(0.0,t.mean_altitude-
        (cfg.site.altitude+g.terminal_test_preflare_altitude));
    double min_total=height/tan(maxSlope*DEG2RAD);
    double max_total=height/tan(minSlope*DEG2RAD);
    double straight_to_gate=fmax(0.0,cfg.guidance.final_approach_distance-gate_ground_distance);
    double preferred_fixed_path=fmax(0.0,preferred-straight_to_gate);
    double min_fixed_path=fmax(0.0,min_total-straight_to_gate);
    double max_fixed_path=fmax(0.0,max_total-straight_to_gate);
    printf("local e=%.1f n=%.1f minR=%.1f maxR=%.1f maxLat=%.3f gateGround=%.1f totalBand=%.1f..%.1f\n",
        current_e,current_n,min_radius,max_radius,max_lateral,gate_ground_distance,min_total,max_total);
    int transitions=0,handoff_ok=0,execution_ok=0;
    for(int ri=0;ri<13;ri++){
        double u=(double)ri/12.0;
        double radius=min_radius+(max_radius-min_radius)*u*u;
        for(int si=-1;si<=1;si+=2){
            double desired_lateral=circle_design_speed*circle_design_speed/radius;
            double desired_sine=desired_lateral/lift;
            if(desired_lateral>max_lateral||desired_sine<0||desired_sine>1)continue;
            double candidate_bank=asin(desired_sine)/effectiveness*RAD2DEG;
            if(!isfinite(candidate_bank)||fabs(candidate_bank)>bank)continue;
            candidate_bank*=si;
            double response=hac_response_lead_time(&t,&cfg.guidance,candidate_bank);
            double response_distance=t.horizontal_speed*response;
            double projected_v2=t.true_air_speed*t.true_air_speed-
                2.0*drag*fmax(0.0,response_distance);
            double projected_join_speed=sqrt(fmax(
                circle_design_speed*circle_design_speed,projected_v2));
            double fe=0,fn=0,fc=t.ground_track_heading;
            hac_projected_local_state(&t,&cfg.site,p.radius,t.ground_track_heading,
                response,candidate_bank,cfg.guidance.taem_roll_rate,
                cfg.guidance.entry_roll_acceleration,lift,effectiveness,drag,
                &fe,&fn,&fc);
            HACTransitionPlan tr={0};
            bool tok=hac_transition_plan(&tr,current_e,current_n,fe,fn,
                t.ground_track_heading,fc,&cfg.site,&cfg.guidance,
                radius,si,projected_join_speed,circle_design_speed,drag,max_lateral,
                preferred_fixed_path,min_fixed_path,max_fixed_path);
            if(!tok)continue;
            transitions++;
            double runway=cfg.site.runway_heading*DEG2RAD;
            double exit_along=tr.p3.e*sin(runway)+tr.p3.n*cos(runway);
            double final=fmax(0.0,-exit_along);
            double line=final-gate_ground_distance;
            double path=tr.lead_length+tr.length+fmax(0.0,line);
            double pslope=atan2(height,path)*RAD2DEG;
            double pv=terminal_normalized_band_violation(path,min_total,max_total);
            double ev=line>=0?0:(gate_ground_distance-final)/fmax(1000.0,gate_ground_distance);
            bool hv=hac_handoff_geometry_valid(&tr,current_e,current_n,
                t.ground_track_heading,&cfg.site,&cfg.guidance,radius,si);
            if(hv)handoff_ok++;
            if(radius>16000.0&&radius<18000.0&&si==1){
                double a0=atan2(tr.p0.n-tr.cone_center.n,
                    tr.p0.e-tr.cone_center.e);
                double a3=atan2(tr.p3.n-tr.cone_center.n,
                    tr.p3.e-tr.cone_center.e);
                double directed=fmod(-si*(tr.cone_end_angle-
                    tr.cone_start_angle),2.0*LANDER_PI);
                if(directed<0)directed+=2.0*LANDER_PI;
                double end_heading=norm_deg(atan2(si*sin(tr.cone_end_angle),
                    -si*cos(tr.cone_end_angle))*RAD2DEG);
                printf("DETAIL start stored=%+.9f actual=%+.9f end stored=%+.9f actual=%+.9f endAngle=%+.9f arcAngle=%.9f arcStored=%.3f arcDerived=%.3f cross=%.6f endHeading=%.9f runway=%.9f leadErr=%.6f startErr=%.6f\n",
                    tr.cone_start_angle,a0,tr.cone_end_angle,a3,tr.end_angle,
                    directed,tr.cone_arc_length,radius*directed,
                    tr.p3.e*cos(runway)-tr.p3.n*sin(runway),end_heading,
                    cfg.site.runway_heading,
                    tr.lead_length-hypot(tr.p0.e-current_e,tr.p0.n-current_n),
                    hypot(tr.lead_start.e-current_e,tr.lead_start.n-current_n));
            }
            TerminalCandidate c={.valid=true,.join=tr,.kind=TERMINAL_PATH_HAC,
                .radius=radius,.side=si,.final_distance=final,.slope=pslope,
                .response=response,.speed=t.true_air_speed,.selected_ut=t.ut,.arrival_ut=t.ut};
            terminal_candidate_execution_cost(&c,&g,&t,t.ground_track_heading,&p,aero,&cfg);
            if(c.execution_margin>=0)execution_ok++;
            printf("R=%8.1f side=%+d bank=%6.2f resp=%5.2f fc=%7.2f lead=%8.1f arc=%8.1f final=%8.1f path=%8.1f slope=%6.2f pv=%7.3f ev=%7.3f trv=%7.3f hand=%d exec=%7.2f/%7.2f margin=%8.2f radial=%8.2f\n",
                radius,si,candidate_bank,response,fc,tr.lead_length,tr.length,final,path,pslope,pv,ev,tr.violation_score,hv,
                c.execution_time,c.execution_horizon,c.execution_margin,c.execution_radial_closure);
        }
    }
    printf("transitions=%d handoff_ok=%d execution_ok=%d\n",transitions,handoff_ok,execution_ok);

    bool ok=terminal_test_start_spiral(&g,&t,t.ground_track_heading,&p,aero,&cfg);
    printf("selector ok=%d R=%.3f side=%+.0f degraded=%d geom=%d energy=%d viol=%.9f final=%.3f lead=%.3f arc=%.3f cone=%d\n",
        ok,g.hac_radius,g.hac_side,g.hac_plan_degraded,g.hac_plan_geometry_degraded,g.hac_plan_energy_degraded,
        g.hac_plan_violation_score,g.terminal_test_final_approach_distance,
        g.hac_transition_lead_length,g.hac_transition_cone_arc_length,g.hac_transition_heading_cone);
    return ok?0:2;
}
