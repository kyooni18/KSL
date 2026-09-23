static AerodynamicModel aero_at(const AerodynamicEnvelope*e,double mach){double a[]={.35,1.05,2.6,6};double x=fmax(0,mach);if(x<=a[0])return e->regimes[0];if(x>=a[3])return e->regimes[3];for(int i=0;i<3;i++)if(x<=a[i+1]){double f=(x-a[i])/(a[i+1]-a[i]);AerodynamicModel r={e->regimes[i].lift_to_drag+(e->regimes[i+1].lift_to_drag-e->regimes[i].lift_to_drag)*f,e->regimes[i].ballistic_coefficient+(e->regimes[i+1].ballistic_coefficient-e->regimes[i].ballistic_coefficient)*f,e->regimes[i].confidence+(e->regimes[i+1].confidence-e->regimes[i].confidence)*f};return r;}return e->regimes[3];}
static double scheduled_aoa(double altitude,const PlanetModel*p,const VehicleProfile*v){double lower=fmax(5,fmin(v->entry_angle_of_attack,10)),f=clampd(altitude/fmax(p->atmosphere_depth,1),0,1);return lower+(v->entry_angle_of_attack-lower)*f;}
static Vector3 gravity(Vector3 pos,const PlanetModel*p){return vessel_physics_gravity(pos,p);}
static double context_aoa(const AeroContext*ctx,double altitude){return isfinite(ctx->aoa)?ctx->aoa:scheduled_aoa(altitude,ctx->p,ctx->v);}
static AtmosState atmosphere(Vector3 pos,Vector3 vel,const AeroContext*ctx){
    double alt=vmag(pos)-ctx->p->radius;
    double d,sos;vessel_physics_environment(ctx->p,ctx->c,alt,&d,&sos);
    Vector3 air=vsub(vel,vcross(planet_rotation_vector(ctx->p),pos));
    double sp=vmag(air);
    double mach=sp/fmax(sos,1),q=.5*fmax(0,d)*sp*sp;
    AtmosState s={.density=d,.air_velocity=air,.speed=sp,.speed_of_sound=sos,.mach=mach,.q=q};
    s.specific_force=vessel_physics_force_best_estimate_config(ctx->c->physics,q,mach,context_aoa(ctx,alt),0,ctx->mass,ctx->gear,ctx->brakes,ctx->airbrakes,aero_at(ctx->e,mach),ctx->c,ctx->v,&s.physics_confidence,&s.direct_aero,&s.physics_uncertainty);
    s.drag_accel=s.specific_force.x;s.lift_accel=s.specific_force.y;
    s.non_gravity=vmag(s.specific_force);
    return s;
}
static Vector3 total_accel(Vector3 pos,Vector3 vel,const AeroContext*ctx){
    AtmosState s=atmosphere(pos,vel,ctx);
    /* A physics model pointer does not guarantee local direct-force support.
       In uncovered Mach/q/incidence regions vessel_physics_force() falls back
       to the aerodynamic envelope, which still needs the learned turn
       effectiveness. Only an actually observed local force sample gets the
       ideal geometric bank rotation. */
    double effectiveness=s.direct_aero?1:clampd(ctx->c->bank_effectiveness,.55,1.35);
    return vessel_physics_acceleration(pos,s.air_velocity,ctx->p,s.specific_force,ctx->bank*effectiveness);
}
static VehicleState rk4_gravity(VehicleState s,double dt,const PlanetModel*p){Vector3 p1=s.velocity,v1=gravity(s.position,p),p2=vadd(s.position,vscale(p1,dt*.5)),u2=vadd(s.velocity,vscale(v1,dt*.5)),v2=gravity(p2,p),p3=vadd(s.position,vscale(u2,dt*.5)),u3=vadd(s.velocity,vscale(v2,dt*.5)),v3a=gravity(p3,p),p4=vadd(s.position,vscale(u3,dt)),u4=vadd(s.velocity,vscale(v3a,dt)),v4=gravity(p4,p);s.position=vadd(s.position,vscale(vadd(vadd(p1,vscale(u2,2)),vadd(vscale(u3,2),u4)),dt/6));s.velocity=vadd(s.velocity,vscale(vadd(vadd(v1,vscale(v2,2)),vadd(vscale(v3a,2),v4)),dt/6));s.ut+=dt;return s;}
static VehicleState rk4_aero(VehicleState s,double dt,AeroContext*ctx){ctx->ut=s.ut;ctx->mass=s.mass;Vector3 k1p=s.velocity,k1v=total_accel(s.position,s.velocity,ctx),p2=vadd(s.position,vscale(k1p,dt*.5)),u2=vadd(s.velocity,vscale(k1v,dt*.5)),k2p=u2,k2v=total_accel(p2,u2,ctx),p3=vadd(s.position,vscale(k2p,dt*.5)),u3=vadd(s.velocity,vscale(k2v,dt*.5)),k3p=u3,k3v=total_accel(p3,u3,ctx),p4=vadd(s.position,vscale(k3p,dt)),u4=vadd(s.velocity,vscale(k3v,dt)),k4p=u4,k4v=total_accel(p4,u4,ctx);s.position=vadd(s.position,vscale(vadd(vadd(k1p,vscale(k2p,2)),vadd(vscale(k3p,2),k4p)),dt/6));s.velocity=vadd(s.velocity,vscale(vadd(vadd(k1v,vscale(k2v,2)),vadd(vscale(k3v,2),k4v)),dt/6));s.ut+=dt;return s;}

static void body_basis(const PlanetModel*p,double ut,Vector3*prime,Vector3*east,Vector3*north){*north=vnorm(p->north_axis,v3(0,0,1));*prime=vnorm(vproject_plane(vrotate(p->prime_meridian_at_epoch,*north,p->rotational_speed*(ut-p->epoch_ut)),*north),v3(1,0,0));*east=vnorm(vcross(*north,*prime),v3(0,1,0));}
GeoPoint predictor_geo_point(Vector3 pos,const PlanetModel*p,double ut){Vector3 prime,east,north;body_basis(p,ut,&prime,&east,&north);Vector3 up=vnorm(pos,prime);double sl=clampd(vdot(up,north),-1,1),lat=asin(sl);Vector3 eq=vnorm(vsub(up,vscale(north,sl)),prime);GeoPoint g={lat*RAD2DEG,norm_signed_deg(atan2(vdot(eq,east),vdot(eq,prime))*RAD2DEG),vmag(pos)-p->radius};return g;}
Vector3 predictor_inertial_position(GeoPoint g,const PlanetModel*p,double ut){Vector3 prime,east,north;body_basis(p,ut,&prime,&east,&north);double lat=g.latitude*DEG2RAD,lon=g.longitude*DEG2RAD,h=cos(lat);return vscale(vadd(vadd(vscale(prime,h*cos(lon)),vscale(east,h*sin(lon))),vscale(north,sin(lat))),p->radius+g.altitude);}
VehicleState predictor_propagate_vacuum(VehicleState s,double target,const PlanetModel*p,double step){if(!p||!isfinite(target)||!isfinite(s.ut)||target<=s.ut)return s;step=clampd(step,.01,120);while(s.ut<target){double dt=fmin(step,target-s.ut);s=rk4_gravity(s,dt,p);}return s;}
double predictor_postburn_periapsis(VehicleState s,const PlanetModel*p){if(!p||!isfinite(s.mass)||!isfinite(s.position.x)||!isfinite(s.position.y)||!isfinite(s.position.z)||!isfinite(s.velocity.x)||!isfinite(s.velocity.y)||!isfinite(s.velocity.z))return -DBL_MAX;double r=vmag(s.position);if(!isfinite(r)||r<=1||!isfinite(p->gravitational_parameter)||p->gravitational_parameter<=0)return -DBL_MAX;Vector3 h=vcross(s.position,s.velocity);double h2=vdot(h,h),e=.5*vdot(s.velocity,s.velocity)-p->gravitational_parameter/r;if(!isfinite(h2)||!isfinite(e))return -DBL_MAX;double ecc=sqrt(fmax(0,1+2*e*h2/(p->gravitational_parameter*p->gravitational_parameter))),pr=h2/fmax(p->gravitational_parameter*(1+ecc),1);return isfinite(pr)?pr-p->radius:-DBL_MAX;}
double predictor_directed_taem_range_error(double range,double closest_distance,double target_range){
    if(!isfinite(range)||!isfinite(closest_distance)||!isfinite(target_range))return NAN;
    range=fmax(0,range);target_range=fmax(0,target_range);closest_distance=clampd(closest_distance,0,range);
    if(closest_distance<target_range){
        /* The trajectory entered the desired TAEM shell before reaching TAEM
           altitude. Preserve that penetration even when the TAEM point is
           still numerically inside the shell: an aircraft that passed close
           to KSC and is now 20 km away on the far side is not equivalent to
           an aircraft approaching the 35 km interface from outside. */
        double passed_distance=range+target_range-2.0*closest_distance;
        return -fmax(fabs(range-target_range),passed_distance);
    }
    return range-target_range;
}


static double entry_topology_shadow_refresh_interval(const Telemetry*t,
        const GuidanceMachine*g,const PlanetModel*p,
        const LandingConfiguration*cfg);

bool entry_topology_search_needed(const GuidanceMachine*g){
    /* A geometry-only MM304 inlet is an acquisition preview, not a route that
       can be certified or installed. In the upper atmosphere its instantaneous
       turn radius can be tens of thousands of kilometres, which makes the full
       two-event topology grid enormously expensive while every candidate is
       guaranteed to remain non-owning. Keep flying the provisional MM304 law
       and defer the propagated topology solve until the published inlet has a
       structurally valid unpowered energy projection. */
    if(!g||!g->taem_interface_target.valid||
       !g->taem_interface_target.energy_qualified||
       g->entry_topology.valid||g->entry_control_reversals||
       (g->entry_reversal_scheduled&&g->entry_reversal_is_final)||
       g->entry_final_reversal_pending||g->entry_final_reversal_completed)
        return false;
    bool fast_reversal_available=g->entry_reversal_scheduled||
        (g->entry_s_turn_plan.valid&&
         g->entry_s_turn_plan.has_planned_reversal&&
         isfinite(g->entry_s_turn_plan.planned_reversal_ut));
    /* A fast reversal is only a complete route when its own propagated plan is
       terminal-ready. Otherwise the existing two-event topology planner may
       replace an ordinary shaping reversal, never a committed final event. */
    return !fast_reversal_available||!g->entry_s_turn_plan.terminal_ready;
}


static Vector3 burn_direction(Vector3 pos,Vector3 vel,double radial_error_deg,double normal_error_deg){
    Vector3 retro=vnorm(vscale(vel,-1),vnorm(vscale(pos,-1),v3(-1,0,0)));
    Vector3 normal=vnorm(vcross(pos,vel),v3(0,0,1));
    Vector3 radial=vnorm(vproject_plane(vnorm(pos,v3(1,0,0)),retro),vnorm(vcross(normal,retro),v3(0,1,0)));
    double radial_error=tan(radial_error_deg*DEG2RAD),normal_error=tan(normal_error_deg*DEG2RAD);
    return vnorm(vadd(retro,vadd(vscale(radial,radial_error),vscale(normal,normal_error))),retro);
}

static Vector3 burn_accel(Vector3 pos,Vector3 vel,const PlanetModel*p,double thrust_accel,double radial_error_deg,double normal_error_deg){
    return vadd(gravity(pos,p),vscale(burn_direction(pos,vel,radial_error_deg,normal_error_deg),thrust_accel));
}

static VehicleState retro_burn_scenario(VehicleState s,double dv,double thrust,double throttle,double ramp,const PlanetModel*p,double radial_error_deg,double normal_error_deg,double *duration,double *projected_delta_v,const VesselPhysicsModel *physics){
    double maxa=thrust*clampd(throttle,0,1)/fmax(s.mass,1);
    if(!isfinite(maxa)||!isfinite(dv)||!isfinite(ramp))maxa=0;
    ramp=clampd(isfinite(ramp)?ramp:0,0,60);
    if(duration)*duration=0;
    if(projected_delta_v)*projected_delta_v=0;
    if(dv<=0||maxa<=.01)return s;
    double delivered=0,elapsed=0,step=.25;
    while(delivered<dv-GUIDANCE_BURN_TERMINAL_DV_TOLERANCE_MPS){
        if(physics&&physics->dry_mass>0&&s.mass<=physics->dry_mass)break;
        maxa=thrust*clampd(throttle,0,1)/fmax(s.mass,1);
        double rem=fmax(0,dv-delivered),f=burn_fraction(elapsed,rem,maxa,ramp);
        if(f<=.0001){elapsed+=step;continue;}
        double dt=fmin(step,fmax(.02,(dv-delivered)/fmax(maxa*f,.01))),ta=maxa*f;
        if(!isfinite(dt)||!(dt>0.0)||!isfinite(ta)||!(ta>0.0))break;
        double previous_delivered=delivered;
        Vector3 retro=vnorm(vscale(s.velocity,-1),v3(-1,0,0));
        Vector3 dir=burn_direction(s.position,s.velocity,radial_error_deg,normal_error_deg);
        double projection=clampd(vdot(dir,retro),0,1);
        Vector3 p1=s.velocity,v1=burn_accel(s.position,s.velocity,p,ta,radial_error_deg,normal_error_deg);
        Vector3 p2=vadd(s.position,vscale(p1,dt*.5)),u2=vadd(s.velocity,vscale(v1,dt*.5)),v2=burn_accel(p2,u2,p,ta,radial_error_deg,normal_error_deg);
        Vector3 p3=vadd(s.position,vscale(u2,dt*.5)),u3=vadd(s.velocity,vscale(v2,dt*.5)),v3a=burn_accel(p3,u3,p,ta,radial_error_deg,normal_error_deg);
        Vector3 p4=vadd(s.position,vscale(u3,dt)),u4=vadd(s.velocity,vscale(v3a,dt)),v4=burn_accel(p4,u4,p,ta,radial_error_deg,normal_error_deg);
        s.position=vadd(s.position,vscale(vadd(vadd(p1,vscale(u2,2)),vadd(vscale(u3,2),u4)),dt/6));
        s.velocity=vadd(s.velocity,vscale(vadd(vadd(v1,vscale(v2,2)),vadd(vscale(v3a,2),v4)),dt/6));
        s.ut+=dt;
        s.mass=vessel_physics_burn_mass(physics,s.mass,thrust*throttle*f,dt);
        delivered+=ta*projection*dt;
        elapsed+=dt;
        if(!isfinite(s.ut)||!isfinite(vmag(s.position))||!isfinite(vmag(s.velocity)))break;
        if(!(delivered>previous_delivered))break;
    }
    if(duration)*duration=elapsed;
    if(projected_delta_v)*projected_delta_v=delivered;
    return s;
}

static double bank_limit(const AtmosState*a,const PlanetModel*p,const VehicleProfile*v,double max_bank){
    double g_load=a->non_gravity/fmax(planet_surface_gravity(p),.1);
    return entry_bank_authority_limit(a->speed,a->q,g_load,v,max_bank);
}
static double turn_radius_for_bank(const AtmosState*a,const TrajectoryCalibrationModel*c,double bank){
    if(!a||!c||a->speed<=1||a->lift_accel<=.005||fabs(bank)<.05)return INFINITY;
    double effective=fabs(bank)*clampd(c->bank_effectiveness,.35,1.8)*DEG2RAD;
    effective=clampd(effective,0,89*DEG2RAD);
    double lateral=a->lift_accel*fabs(sin(effective));
    return lateral>.005?a->speed*a->speed/lateral:INFINITY;
}
static double bank_for_turn_radius(const AtmosState*a,const TrajectoryCalibrationModel*c,double radius,double max_bank){
    if(!a||!c||!isfinite(radius)||radius<=1||a->speed<=1||a->lift_accel<=.005)return 0;
    double needed=a->speed*a->speed/radius,ratio=needed/fmax(a->lift_accel,.005);
    if(ratio<=0)return 0;
    double eff=clampd(c->bank_effectiveness,.35,1.8),limit=fabs(max_bank)*eff*DEG2RAD;
    limit=clampd(limit,0,89*DEG2RAD);
    double angle=asin(clampd(ratio,0,sin(limit)))/eff*RAD2DEG;
    return clampd(angle,0,fabs(max_bank));
}
double entry_taem_speed_target(const VehicleProfile*v,const GuidanceSettings*s,const PlanetModel*p){
    if(!v||!s||!p)return NAN;
    TaemSpeedEnvelope envelope=decision_taem_speed_envelope(
        v,p,s->taem_interface_altitude);
    if(!envelope.valid)
        return fmax(v->minimum_safe_speed,v->final_approach_speed);
    /*
     * MM305 is designed to begin near the explicit mission Mach target rather
     * than at the structural ceiling.  Clamp the Mach-derived speed only by the
     * actual TAEM aerodynamic envelope so entry guidance and terminal planning
     * share one engagement-energy reference.
     */
    double sound=planet_atmospheric_speed_of_sound(p,s->taem_interface_altitude);
    double nominal=isfinite(sound)&&sound>DBL_MIN?
        sound*s->mm305_target_mach:envelope.maximum_speed_mps;
    if(!envelope.feasible)return envelope.minimum_speed_mps;
    return clampd(nominal,envelope.minimum_speed_mps,envelope.maximum_speed_mps);
}
static int predictor_speedbrake_state(PredictorGuidance*g,GeoPoint geo,const AtmosState*a,const PlanetModel*p,const TrajectoryCalibrationModel*c,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,double mass,bool reached_taem,double dt){
    if(!reached_taem){speedbrake_controller_reset(&g->speedbrake,false);return 0;}
    bool predicted_gear=geo.altitude-site->altitude<=s->gear_deployment_altitude;
    Vector3 deployed_force;double deployed_confidence=0;
    bool speedbrake_model_ready=c&&c->physics&&
        vessel_physics_aero_config(c->physics,a->q,a->mach,g->actual_aoa,0,mass,predicted_gear,false,1,&deployed_force,&deployed_confidence)&&
        deployed_confidence>=.12;
    if(!speedbrake_model_ready){speedbrake_controller_reset(&g->speedbrake,false);return 0;}
    GeoPoint target={site->latitude,site->longitude,site->altitude};
    double range=great_circle_distance(geo,target,p->radius),taem_range=fmax(s->taem_interface_range,s->final_approach_distance+1000);
    double blend=clampd((range-s->final_approach_distance)/fmax(taem_range-s->final_approach_distance,1),0,1);
    blend=sqrt(blend);
    double taem_speed=entry_taem_speed_target(v,s,p);
    double final_speed=fmax(v->minimum_safe_speed*1.08,v->final_approach_speed);
    double target_speed=final_speed+(taem_speed-final_speed)*blend;
    double altitude_blend=clampd(range/fmax(taem_range,1),0,1);
    double target_altitude=site->altitude+(s->taem_interface_altitude-site->altitude)*altitude_blend;
    double density=planet_atmospheric_density(p,target_altitude)*clampd(c->density_scale,.35,2.8);
    double target_q=.5*fmax(0,density)*target_speed*target_speed;
    double current_energy=rotating_specific_energy(geo.latitude,geo.altitude,a->speed,p);
    double target_energy=rotating_specific_energy(geo.latitude,target_altitude,target_speed,p);
    double energy_scale=fmax(.5*target_speed*target_speed,planet_surface_gravity(p)*fmax(1000,target_altitude-site->altitude));
    return speedbrake_controller_update(&g->speedbrake,a->q,target_q,current_energy-target_energy,energy_scale,v->maximum_dynamic_pressure,false,dt)?1:0;
}
static double entry_energy_to_taem(double latitude,double altitude,double speed,const PlanetModel*p,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s){
    double target_speed=entry_taem_speed_target(v,s,p);
    return fmax(0,entry_remaining_specific_energy(latitude,altitude,speed,site->latitude,s->taem_interface_altitude,target_speed,p));
}
static double entry_required_average_drag(GeoPoint geo,double speed,double range,const PlanetModel*p,
        const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*s,
        const TaemInterfaceTarget*interface_target){
    if(interface_target&&interface_target->valid&&isfinite(interface_target->specific_energy)&&
       isfinite(interface_target->along_track)&&isfinite(interface_target->cross_track)){
        double rh=site->runway_heading*DEG2RAD;
        double e=interface_target->along_track*sin(rh)+interface_target->cross_track*cos(rh);
        double n=interface_target->along_track*cos(rh)-interface_target->cross_track*sin(rh);
        GeoPoint origin={site->latitude,site->longitude,site->altitude};
        GeoPoint gate=local_point(origin,e,n,p->radius,interface_target->altitude);
        double distance=great_circle_distance(geo,gate,p->radius);
        double current=rotating_specific_energy(geo.latitude,geo.altitude,speed,p);
        if(isfinite(distance)&&isfinite(current))
            return fmax(0.0,current-interface_target->specific_energy)/fmax(5000.0,distance);
    }
    double reserve=fmax(entry_taem_range_target(p,s),s->hac_radius*3.0);
    double usable=fmax(5000,range-reserve);
    return entry_energy_to_taem(geo.latitude,geo.altitude,speed,p,v,site,s)/usable;
}
static void predictor_change_sign(PredictorGuidance*g,double sign){sign=sign>=0?1:-1;if(g->sign==sign)return;if(g->sign!=0)g->reversals++;g->sign=sign;g->leg_elapsed=0;g->established=false;g->configured_leg_dwell_satisfied=false;}
static void attitude_step(PredictorGuidance*g,double target_bank,double target_aoa,const AtmosState*a,const VehicleProfile*v,const GuidanceSettings*set,double dt,const VesselPhysicsModel *physics){
    bool measured_roll=physics&&physics->authority_confidence[1]>0&&isfinite(physics->authority[1])&&physics->authority[1]>1e-6;
    bool measured_pitch=physics&&physics->authority_confidence[0]>0&&isfinite(physics->authority[0])&&physics->authority[0]>1e-6;
    if(measured_roll)
        vessel_physics_axis_step(physics,1,target_bank,a->q,set->entry_roll_rate,dt,&g->actual_bank,&g->actual_bank_rate);
    else{
        FlightControlRegime regime=flight_control_regime(a->mach,a->speed,a->q,v->minimum_safe_speed);
        double bank_error=norm_signed_deg(target_bank-g->actual_bank),max_rr=fmax(.8,set->entry_roll_rate*1.12*regime.response_scale),max_ra=fmax(.45,set->entry_roll_acceleration*1.15*regime.response_scale);
        double desired_rr=clampd(bank_error*.72,-max_rr,max_rr),rr_delta=clampd(desired_rr-g->actual_bank_rate,-max_ra*dt,max_ra*dt);
        g->actual_bank_rate=clampd(g->actual_bank_rate+rr_delta,-max_rr*1.35,max_rr*1.35);g->actual_bank=norm_signed_deg(g->actual_bank+g->actual_bank_rate*dt);
    }
    if(measured_pitch)
        vessel_physics_axis_step(physics,0,target_aoa,a->q,set->entry_roll_rate,dt,&g->actual_aoa,&g->actual_aoa_rate);
    else{
        double aoa_error=target_aoa-g->actual_aoa,max_ar=4.2,max_aa=2.4,desired_ar=clampd(aoa_error*.62,-max_ar,max_ar),ar_delta=clampd(desired_ar-g->actual_aoa_rate,-max_aa*dt,max_aa*dt);
        g->actual_aoa_rate=clampd(g->actual_aoa_rate+ar_delta,-max_ar*1.25,max_ar*1.25);g->actual_aoa=clampd(g->actual_aoa+g->actual_aoa_rate*dt,-v->maximum_angle_of_attack*1.5,v->maximum_angle_of_attack*1.5);
    }
}
static double predictor_terminal_hac_radius(const AtmosState*a,const PlanetModel*p,const TrajectoryCalibrationModel*c,const VehicleProfile*v,const GuidanceSettings*set,double maxbank);
static double predictor_interface_heading(const PredictorGuidance*g,GeoPoint geo,const PlanetModel*p,
        const LandingSite*site,const GuidanceSettings*set,double fallback_bearing){
    if(!g||!g->has_interface_target||!g->interface_target.valid)return fallback_bearing;
    (void)set;
    GeoPoint site_geo={site->latitude,site->longitude,site->altitude};
    double along=0.0,cross=0.0;
    runway_coordinates(geo,site_geo,site->runway_heading,p->radius,&along,&cross);
    return taem_interface_line_heading(along,cross,site->runway_heading,&g->interface_target);
}

static bool predictor_terminal_arc_ready(GeoPoint geo,double course,const PlanetModel*p,
        const LandingSite*site,const TaemInterfaceTarget*q,double final_sign){
    if(!p||!site||!q||!q->valid||!isfinite(course))return true;
    GeoPoint site_geo={site->latitude,site->longitude,site->altitude};
    double along=0.0,cross=0.0;
    runway_coordinates(geo,site_geo,site->runway_heading,p->radius,&along,&cross);
    double da=q->along_track-along,dc=q->cross_track-cross;
    if(hypot(da,dc)<1000.0)return true;
    double bearing=norm_deg(site->runway_heading+atan2(dc,da)*RAD2DEG);
    double side=final_sign>=0.0?1.0:-1.0;
    double gate_turn=norm_signed_deg(bearing-course);
    double tangent_turn=norm_signed_deg(q->course-course);
    return side*gate_turn>=1.0&&side*tangent_turn>=-.5;
}
static double entry_bank(PredictorGuidance*g,VehicleState s,GeoPoint geo,const AtmosState*a,const PlanetModel*p,AerodynamicModel aero,const TrajectoryCalibrationModel*c,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double dt,double*aoa_out){
    (void)aero;
    Vector3 up=vnorm(s.position,v3(0,1,0)),air=vsub(s.velocity,vcross(planet_rotation_vector(p),s.position)),hv=vproject_plane(air,up),north=vnorm(vsub(p->north_axis,vscale(up,vdot(p->north_axis,up))),v3(0,0,1)),east=vnorm(vcross(north,up),v3(1,0,0));
    GeoPoint target={site->latitude,site->longitude,site->altitude};
    bool tangent_lock=g->final_heading_lock&&g->has_interface_target&&g->interface_target.valid;
    GeoPoint heading_target=g->final_heading_lock&&!tangent_lock?
        runway_approach_aimpoint(site,p->radius,500.0):target;
    double course=vmag(hv)>1?norm_deg(atan2(vdot(hv,east),vdot(hv,north))*RAD2DEG):initial_bearing(geo,heading_target);
    double bearing=initial_bearing(geo,heading_target);
    if(g->has_interface_target&&g->interface_target.valid)
        bearing=predictor_interface_heading(g,geo,p,site,set,bearing);
    double he=norm_signed_deg(bearing-course),range=great_circle_distance(geo,target,p->radius);
    double grav=p->gravitational_parameter/pow(p->radius+fmax(0,geo.altitude),2);
    if(g->entry_speed<=0&&a->speed>100)g->entry_speed=a->speed;
    const TaemInterfaceTarget*delivery_target=g->has_interface_target&&g->interface_target.valid?
        &g->interface_target:NULL;
    LandingConfiguration shaping_cfg={.site=*site,.vehicle=*v,.guidance=*set};
    double required_drag=entry_required_average_drag(geo,a->speed,range,p,v,site,set,delivery_target);
    double drag_ratio=required_drag>1e-6?a->drag_accel/required_drag:1.0;
    double drag_deficit=clampd(1.0-drag_ratio,0,1);
    /* Autonomous predictor policy keeps a conservative 60 deg planning ceiling,
       but a committed MM304 plan may already be legal above that private ceiling.
       Replay the committed command against the same shared authority limit as live
       guidance instead of silently clipping it and manufacturing a margin failure. */
    double shared_bank_limit=bank_limit(a,p,v,maxbank);
    double env=fmin(60.0,shared_bank_limit),minimum_radius=turn_radius_for_bank(a,c,env),mag=0;
    bool forced_policy=g->force_control&&s.ut<g->forced_until_ut;
    if(forced_policy)env=shared_bank_limit;
    if(forced_policy){
        /* MPC candidates are policies, not one short control pulse followed by
           an optimistic fallback. Keep the selected bank magnitude and AoA as
           the entry-energy policy for the whole forecast, while still letting
           the S-turn side reverse when the predicted ground track crosses its
           heading corridor. This removes receding-horizon procrastination: a
           wings-level candidate is scored as wings-level all the way to TAEM
           instead of assuming it will suddenly bank hard after 60-70 seconds. */
        mag=clampd(fabs(g->forced_bank),0,env);
    }else if(drag_deficit>.01&&isfinite(minimum_radius)){
        /* Bank is an energy-path actuator: reducing the vertical component of
           lift lets a hot trajectory descend into denser air. Express the
           command as a broad turn-radius family so energy correction never
           degenerates into the old 70 deg knife-edge S-turn. */
        double radius_scale=8.0-(8.0-1.20)*drag_deficit;
        mag=bank_for_turn_radius(a,c,minimum_radius*radius_scale,env);
    }
    if(g->sign==0)predictor_change_sign(g,he>=0?1:-1);
    double effective_min_leg=entry_s_turn_effective_minimum_leg(a->speed,entry_taem_speed_target(v,set,p),set);
    /* The longer dwell governs creation of new S-turn topology. A reversal that
       was already committed remains an executable event and keeps the original
       configured actuator dwell; otherwise a model update could move the goal
       posts after the aircraft has already committed to the roll-through. */
    double executable_min_leg=g->replay_event?set->s_turn_minimum_leg_duration:effective_min_leg;
    if(g->established&&g->leg_elapsed>=set->s_turn_minimum_leg_duration)g->configured_leg_dwell_satisfied=true;
    bool configured_minleg=g->configured_leg_dwell_satisfied;
    bool minleg=g->established&&g->leg_elapsed>=executable_min_leg;
    /* Tighten the bank-reversal corridor as terminal range closes. The old
       geometry widened near KSC because the fixed HAC-width term was divided
       by a shrinking range, so predicted reversals accumulated in the last
       few dozen kilometres. Use the configured HAC look-ahead angle for the
       broad early-entry corridor and the apparent angular width of one HAC
       diameter from the nominal entry range for the terminal corridor.

       Evaluate the corridor at the range where the shuttle will be after a
       full sign reversal has had time to establish. This starts the roll soon
       enough that the new bank, rather than wings-level roll-through, reaches
       the terminal region. */
    FlightControlRegime reversal_regime=flight_control_regime(a->mach,a->speed,a->q,v->minimum_safe_speed);
    double reversal_rate=fmax(.5,set->entry_roll_rate*fmax(.42,reversal_regime.command_rate_scale));
    double reversal_accel=fmax(.3,set->entry_roll_acceleration*fmax(.30,reversal_regime.command_accel_scale));
    double reversal_span=fabs(g->actual_bank)+fmax(4.0,mag);
    double reversal_time=reversal_span/reversal_rate+reversal_rate/reversal_accel;
    double range_after_roll=fmax(0,range-a->speed*reversal_time);
    double terminal_join_range=entry_taem_range_target(p,set);
    double corridor_fraction=clampd((range_after_roll-terminal_join_range)/
        fmax(set->target_entry_range-terminal_join_range,1),0,1);
    corridor_fraction=corridor_fraction*corridor_fraction*(3.0-2.0*corridor_fraction);
    double mission_entry_range=fmax(set->target_entry_range,terminal_join_range+1000.0);
    double near_corridor=atan2(2.0*set->hac_radius,fmax(mission_entry_range,1))*RAD2DEG;
    double far_corridor=fmax(near_corridor,set->hac_look_ahead_angle);
    double heading_corridor=near_corridor+(far_corridor-near_corridor)*corridor_fraction;
    /* Timed S-turn reversals are only useful after the current bank has created
       meaningful lateral/vertical work. When the propagated shuttle is high
       for the mission altitude-vs-range corridor, widen the signed course
       corridor toward 22 deg. This keeps one bank side long enough to reduce
       vertical lift and build real crossrange instead of alternating around
       the site bearing every few tens of seconds. The extra corridor tapers to
       zero as TAEM range is consumed. */
    double mission_entry_altitude=entry_guidance_start_altitude(p,set);
    double desired_range_altitude=entry_altitude_target_for_range(mission_entry_altitude,
        mission_entry_range,range,set->taem_interface_altitude,terminal_join_range);
    double altitude_debt=fmax(0.0,geo.altitude-desired_range_altitude);
    double debt_scale=fmax(4500.0,(mission_entry_altitude-set->taem_interface_altitude)*.18);
    double altitude_debt_fraction=clampd(altitude_debt/debt_scale,0.0,1.0);
    double predicted_fpa=atan2(vdot(air,up),fmax(1.0,vmag(hv)))*RAD2DEG;
    double fpa_delivery_debt=entry_s_turn_fpa_delivery_debt(predicted_fpa,delivery_target);
    double interface_distance=INFINITY;
    if(delivery_target){
        double ih=site->runway_heading*DEG2RAD;
        double ie=delivery_target->along_track*sin(ih)+delivery_target->cross_track*cos(ih);
        double in=delivery_target->along_track*cos(ih)-delivery_target->cross_track*sin(ih);
        GeoPoint origin={site->latitude,site->longitude,site->altitude};
        GeoPoint interface_geo=local_point(origin,ie,in,p->radius,delivery_target->altitude);
        interface_distance=great_circle_distance(geo,interface_geo,p->radius);
    }
    heading_corridor=entry_s_turn_reversal_corridor(heading_corridor,range,
        terminal_join_range,altitude_debt_fraction,fpa_delivery_debt);
    /* With an explicit MM304 tangent state the last reversal is scheduled against
       distance to that gate, not radial distance to KSC.  The old KSC-range deadline
       could fire repeatedly while the vehicle was still hundreds of kilometres from
       the lateral tangent, producing alternating +/-60 deg legs that erased the very
       crossrange MM304 needed to deliver.  Reserve one roll-through plus one established
       final leg, then let that reversal become a one-way tangent capture. */
    double terminal_reversal_reserve;
    double terminal_reversal_metric;
    if(delivery_target){
        terminal_reversal_reserve=fmax(delivery_target->acquisition_lead,
            a->speed*(reversal_time+1.35*effective_min_leg));
        terminal_reversal_metric=fmax(0.0,interface_distance-a->speed*reversal_time);
    }else{
        terminal_reversal_reserve=terminal_join_range+
            a->speed*(reversal_time+1.35*effective_min_leg);
        terminal_reversal_metric=range_after_roll;
    }
    /* A high/FPA-indebted trajectory should keep the established energy-management
       leg rather than spend its remaining sink authority in a premature roll-through. */
    double remaining_taem_energy;
    if(delivery_target&&isfinite(delivery_target->specific_energy)){
        double current_energy=rotating_specific_energy(geo.latitude,geo.altitude,a->speed,p);
        remaining_taem_energy=isfinite(current_energy)?
            fmax(0.0,current_energy-delivery_target->specific_energy):INFINITY;
    }else{
        remaining_taem_energy=entry_energy_to_taem(geo.latitude,geo.altitude,a->speed,p,v,site,set);
    }
    double terminal_removable_energy=fmax(.03,a->drag_accel)*terminal_reversal_reserve*1.35;
    bool terminal_energy_ready=isfinite(remaining_taem_energy)&&remaining_taem_energy<=terminal_removable_energy;
    bool terminal_altitude_ready=altitude_debt<=600.0||
        a->speed<=set->taem_force_handoff_speed;
    bool terminal_reversal_deadline=terminal_reversal_metric<=terminal_reversal_reserve;
    bool terminal_roll_has_delivery_time=true;
    if(g->enforce_terminal_delivery_budget&&delivery_target&&fpa_delivery_debt>1e-9){
        /* Compare the remaining speed clock with the actual work a reversal would
           consume. Gravity partly offsets drag on a descending trajectory, so use
           the along-path net deceleration rather than drag alone. Then estimate the
           vertical-speed change still required to reach the latest legal TAEM FPA
           using the sink authority of the *currently established* bank. */
        double net_deceleration=fmax(.5,a->drag_accel+grav*sin(predicted_fpa*DEG2RAD));
        double handoff_eta=a->speed>set->taem_force_handoff_speed?
            (a->speed-set->taem_force_handoff_speed)/net_deceleration:0.0;
        double latest_delivery_fpa=clampd(delivery_target->flight_path_angle+
            TAEM_INTERFACE_FPA_DEBT_LIMIT_DEG,-24.0,-2.5);
        double required_sink_delta=a->speed*fmax(0.0,
            sin(predicted_fpa*DEG2RAD)-sin(latest_delivery_fpa*DEG2RAD));
        double hold_bank=clampd(fabs(g->actual_bank),0.0,89.0)*DEG2RAD;
        double sink_authority=grav-a->lift_accel*cos(hold_bank);
        double fpa_recovery_eta=sink_authority>.25?required_sink_delta/sink_authority:INFINITY;
        terminal_roll_has_delivery_time=handoff_eta>reversal_time+fpa_recovery_eta;
    }
    bool terminal_deadline_roll_ready=terminal_reversal_deadline&&terminal_roll_has_delivery_time;
    double terminal_final_sign=fabs(g->sign)>.1?-g->sign:(he>=0.0?1.0:-1.0);
    bool terminal_arc_ready=!delivery_target||predictor_terminal_arc_ready(geo,course,p,site,
        delivery_target,terminal_final_sign);
    bool terminal_reversal_due=terminal_deadline_roll_ready&&terminal_energy_ready&&
        terminal_altitude_ready&&terminal_arc_ready;
    bool measured_reversal_due=g->sign*he<-heading_corridor;
    /* Predictor and live MM304 must agree on when a nonfinal sign change is executable.
       The failed live run published an early reversal time while the vehicle had only a
       fraction of the setup-side crossrange that live guidance requires, so replay and
       execution diverged by tens of seconds. Apply the same chord-aware setup contract
       here before exporting or replaying that event. */
    Telemetry reversal_geometry={0};
    runway_coordinates(geo,target,site->runway_heading,p->radius,
        &reversal_geometry.runway_along_track,&reversal_geometry.runway_cross_track);
    double candidate_sign=fabs(g->sign)>.1?-g->sign:0.0;
    bool shaping_ready=!delivery_target||entry_taem_shaping_crossrange_ready(delivery_target,
        &reversal_geometry,p,&shaping_cfg,candidate_sign,NULL,NULL);
    bool shaping_endpoint_ready=!delivery_target||predictor_terminal_arc_ready(geo,course,p,site,
        delivery_target,candidate_sign);
    /* Mirror live MM304's setup-leg release exactly.  A fixed perpendicular inlet
       starts on the opposite setup side; once that measured side has built the
       configured crossrange proof and the ordinary actuator dwell is satisfied,
       the next sign is the inlet/final side.  The predictor previously required
       an unrelated heading-corridor or terminal-energy deadline as well, so the
       deorbit execution witness could fly the setup leg hundreds of kilometres
       after live guidance would have reversed. */
    double target_relative=delivery_target?
        norm_signed_deg(delivery_target->course-site->runway_heading):NAN;
    double final_side=isfinite(target_relative)&&fabs(target_relative)>1e-6?
        (target_relative>=0.0?1.0:-1.0):0.0;
    bool live_setup_reversal_due=delivery_target&&candidate_sign!=0.0&&
        final_side!=0.0&&candidate_sign*final_side>0.0&&shaping_ready;
    /* Once the terminal deadline is active, keep building the current S-turn until
       the opposite-side arc can actually reach the precomputed tangent without a
       second bank reversal. The same rule now also protects an ordinary corridor
       reversal that would switch onto the selected terminal side too early. */
    if(delivery_target&&measured_reversal_due&&!shaping_ready&&!shaping_endpoint_ready)
        measured_reversal_due=false;
    if(delivery_target&&terminal_reversal_deadline&&!terminal_arc_ready)measured_reversal_due=false;
    bool event_due=g->replay_event&&s.ut>=g->replay_event_ut;
    bool replay_arc_ready=!delivery_target||
        predictor_terminal_arc_ready(geo,course,p,site,delivery_target,g->replay_event_sign);
    bool replay_shaping_ready=!delivery_target||entry_taem_shaping_crossrange_ready(delivery_target,
        &reversal_geometry,p,&shaping_cfg,g->replay_event_sign,NULL,NULL);
    /* Final events require the actual tangent arc. Nonfinal committed events use the
       same live OR-contract: sufficient setup crossrange or a directly reachable
       endpoint may release the sign change. */
    bool replay_geometry_ready=g->replay_event_final?replay_arc_ready:
        (replay_shaping_ready||replay_arc_ready);
    bool replay_due=event_due&&replay_geometry_ready;
    bool deadline_action=delivery_target?terminal_reversal_due:terminal_deadline_roll_ready;
    bool reverse_due=g->replay_event?replay_due:
        (live_setup_reversal_due||measured_reversal_due||deadline_action);
    bool reversal_dwell_ready=minleg||(!g->replay_event&&
        ((deadline_action&&configured_minleg)||(live_setup_reversal_due&&configured_minleg)));
    if(!g->final_heading_lock&&mag>=4&&reversal_dwell_ready&&reverse_due){
        double old_sign=g->sign;
        bool tangent_final_available=delivery_target!=NULL;
        bool fallback_final_available=isfinite(predictor_terminal_hac_radius(a,p,c,v,set,maxbank));
        bool final_reversal=g->replay_event?
            (g->replay_event_final&&(tangent_final_available||fallback_final_available)):
            (terminal_reversal_due&&(tangent_final_available||fallback_final_available));
        double event_sign;
        if(g->replay_event)event_sign=g->replay_event_sign;
        else if(final_reversal&&tangent_final_available){
            /* Nominal MM304 terminal topology contains an actual final reversal.
               Its timing is delayed until the opposite-side arc can join the tangent
               line monotonically, so exporting that opposite side is now consistent
               with the trajectory the predictor continues to fly. */
            event_sign=terminal_final_sign;
        }else event_sign=-g->sign;
        predictor_change_sign(g,event_sign);
        g->replay_event=false;
        if(final_reversal)g->final_heading_lock=true;
        bool side_changed=old_sign!=0&&g->sign!=old_sign;
        if(!g->has_first_reversal&&(side_changed||final_reversal)){
            g->has_first_reversal=true;
            g->first_reversal_final=final_reversal;
            g->first_reversal_ut=s.ut;
            g->first_reversal_range=range;
            g->first_reversal_sign=g->sign;
        }
    }

    double qr=a->q/fmax(v->maximum_dynamic_pressure,1),gr=(a->non_gravity/fmax(grav,.1))/fmax(v->maximum_g_load,.1);
    double protective_aoa=entry_low_q_protective_aoa_floor(a->q,v);
    double aoa=forced_policy?clampd(g->forced_aoa,protective_aoa,v->maximum_angle_of_attack):
        v->entry_angle_of_attack+(v->maximum_angle_of_attack-v->entry_angle_of_attack)*drag_deficit;
    /* Vertical debt is paid with bank/path geometry. The predictor must mirror
       live guidance and never buy sink by exposing the orbiter at low incidence. */
    aoa=fmax(aoa,protective_aoa);
    if(!forced_policy&&a->mach>4&&qr<.45&&geo.altitude<=desired_range_altitude+300.0)
        aoa=fmax(aoa,v->maximum_angle_of_attack-1.0);
    if(qr>.92){aoa=fmin(aoa+1.0,v->maximum_angle_of_attack);mag=fmin(v->maximum_bank_angle,mag+5);}
    if(gr>.92){aoa-=clampd((gr-.9)*18,2,7);mag*=.6;}
    aoa=clampd(aoa,0,v->maximum_angle_of_attack);if(aoa_out)*aoa_out=aoa;
    mag=fmin(mag,env);
    FlightControlRegime regime=flight_control_regime(a->mach,a->speed,a->q,v->minimum_safe_speed);double rs=fmax(.42,regime.command_rate_scale),as=fmax(.30,regime.command_accel_scale);
    double command_mag=forced_policy?mag:mag/clampd(c->bank_effectiveness,.55,1.45);
    double targetbank;
    if(g->final_heading_lock&&g->has_interface_target&&g->interface_target.valid){
        /* Final heading lock is a line-capture controller, not a second S-turn.
           Use the shared course-rate inversion and permit only a small opposite-bank
           correction after the perpendicular course has been acquired. */
        double response=fmax(4.0,g->interface_target.response_time);
        targetbank=taem_course_rate_bank_command(a->speed,a->lift_accel,c->bank_effectiveness,
            env,course,bearing,response);
        if(targetbank*g->sign<0.0&&fabs(targetbank)>18.0)targetbank=-g->sign*18.0;
    }else if(g->final_heading_lock){
        /* Legacy no-interface recovery only. Nominal MM304 always has a tangent
           target and therefore never previews or owns a HAC here. */
        double hac_radius=predictor_terminal_hac_radius(a,p,c,v,set,maxbank);
        if(isfinite(hac_radius)){
            double side=g->sign>=0?1:-1;
            HACGuidance h=hac_guidance_compute_radius(geo,a->speed,course,site,set,p->radius,side,grav,hac_radius);
            double lateral=h.lateral_acceleration,eff=clampd(c->bank_effectiveness,.35,1.8);
            double ratio=fabs(lateral)/fmax(a->lift_accel,.01);
            targetbank=copysign(asin(clampd(ratio,0,sin(clampd(env*eff,0,89)*DEG2RAD)))/eff*RAD2DEG,lateral);
            targetbank=clampd(targetbank,-env,env);
        }else targetbank=g->sign*fmin(env,command_mag);
    }else targetbank=g->sign*fmin(env,command_mag);
    double limited=norm_signed_deg(jerk_angle_update(&g->limiter,targetbank,set->entry_roll_rate*rs,set->entry_roll_acceleration*as,dt));
    double predicted_g_force=a->non_gravity/fmax(planet_surface_gravity(p),.1);
    double predicted_stall_fraction=vessel_physics_conservative_stall_fraction(a->speed,g->actual_aoa,a->q,v);
    if(!g->entry_loaded&&a->non_gravity>=1.5)g->entry_loaded=true;
    bool aerodynamic_authority=entry_s_turn_bank_authority_available(a->q,a->speed,
        predicted_stall_fraction,predicted_g_force,v);
    bool loaded=g->entry_loaded||aerodynamic_authority;
    double magnitude=fabs(limited),threshold=fmin(12,fmax(4,magnitude*.35));
    bool meaningful=loaded&&magnitude>=threshold;
    bool captured=meaningful&&g->sign*g->actual_bank>0&&fabs(g->actual_bank)>=threshold;
    if(!captured){g->established=false;g->leg_elapsed=0;}
    else if(!g->established){g->established=true;g->leg_elapsed=0;}
    else g->leg_elapsed+=dt;
    return limited;
}

static double predictor_local_log_density_gradient(const PlanetModel*p,double altitude){
    if(!p||p->atmosphere_sample_count<2||!isfinite(altitude))return 0.0; /* decision-literal-ok: interpolation requires two samples */
    size_t hi=1;
    while(hi<p->atmosphere_sample_count&&p->atmosphere_altitude[hi]<altitude)hi++;
    if(hi>=p->atmosphere_sample_count)hi=p->atmosphere_sample_count-1;
    size_t lo=hi-1;
    double h0=p->atmosphere_altitude[lo],h1=p->atmosphere_altitude[hi];
    double r0=p->atmosphere_density[lo],r1=p->atmosphere_density[hi];
    if(!(h1>h0)||!(r0>0.0)||!(r1>0.0))return 0.0;
    return fabs(log(r1/r0)/(h1-h0));
}

static double predictor_step_size(const AtmosState*a,const PlanetModel*p,
        const VehicleProfile*v,const GuidanceSettings*set,double altitude,
        double vertical_speed,double entry_alt,bool entered,bool reached_taem,
        double roll_rate,double base){
    (void)v;(void)set;(void)reached_taem;
    if(!(base>0.0)||!isfinite(base))return 0.0;

    double dt=base;
    if(a&&a->density>0.0){
        if(a->non_gravity>DBL_EPSILON&&a->speed>DBL_EPSILON){
            double acceleration_timescale=a->speed/a->non_gravity;
            if(isfinite(acceleration_timescale)&&acceleration_timescale>0.0)
                dt=fmin(dt,acceleration_timescale);
        }

        double gradient=predictor_local_log_density_gradient(p,altitude);
        if(gradient>DBL_EPSILON&&fabs(vertical_speed)>DBL_EPSILON){
            double density_timescale=1.0/(gradient*fabs(vertical_speed));
            if(isfinite(density_timescale)&&density_timescale>0.0)
                dt=fmin(dt,density_timescale);
        }
    }

    if(fabs(roll_rate)>DBL_EPSILON){
        /* One radian is the natural angular state scale; resolve any faster
           roll motion rather than switching on arbitrary deg/s bands. */
        double roll_timescale=1.0/(fabs(roll_rate)*DEG2RAD);
        if(isfinite(roll_timescale)&&roll_timescale>0.0)
            dt=fmin(dt,roll_timescale);
    }

    if(!entered&&vertical_speed<0.0&&altitude>entry_alt){
        double event_time=(altitude-entry_alt)/(-vertical_speed);
        if(isfinite(event_time)&&event_time>0.0)
            dt=fmin(dt,event_time);
    }

    /* Lower bound is purely floating-point/numerical, not flight policy. */
    double numerical_floor=base*sqrt(DBL_EPSILON); /* decision-literal-ok: numerical precision floor */
    return fmax(numerical_floor,fmin(dt,base));
}

static bool predictor_terminal_hac_radius_bounds(const AtmosState*a,
        const PlanetModel*p,const TrajectoryCalibrationModel*c,
        const VehicleProfile*v,const GuidanceSettings*set,double maxbank,
        double*minimum_out,double*maximum_out){
    if(!a||!p||!c||!v||!set||!(a->speed>0.0))return false;

    double usable_bank=bank_limit(a,p,v,maxbank);
    if(!(usable_bank>0.0))return false;
    double live_turn=turn_radius_for_bank(a,c,usable_bank);
    if(!isfinite(live_turn)||!(live_turn>0.0))return false;

    double design_speed=entry_taem_speed_target(v,set,p);
    double minimum_speed=fmax(v->minimum_safe_speed,v->final_approach_speed);
    design_speed=fmax(design_speed,minimum_speed);
    if(!(design_speed>0.0))return false;

    /*
     * At unchanged aerodynamic state turn radius scales with V^2.  This is the
     * only scaling needed here; the actual candidate must still pass the full
     * maneuver-feasibility test below.
     */
    double scale=design_speed/a->speed;
    double minimum_radius=live_turn*scale*scale;
    double maximum_radius=p->radius; /* geometric limit of the local body */
    if(!isfinite(minimum_radius)||!(minimum_radius>0.0)||
       !isfinite(maximum_radius)||maximum_radius<minimum_radius)
        return false;

    if(minimum_out)*minimum_out=minimum_radius;
    if(maximum_out)*maximum_out=maximum_radius;
    return true;
}

static double predictor_terminal_hac_radius(const AtmosState*a,
        const PlanetModel*p,const TrajectoryCalibrationModel*c,
        const VehicleProfile*v,const GuidanceSettings*set,double maxbank){
    double minimum_radius=0.0,maximum_radius=0.0;
    if(!predictor_terminal_hac_radius_bounds(
            a,p,c,v,set,maxbank,&minimum_radius,&maximum_radius))
        return INFINITY;
    /*
     * Legacy callers need one representative radius only to keep a continuous
     * path law alive.  The nominal configured radius is an objective, not a
     * feasibility threshold; clamp it only to the physical authority interval.
     */
    return clampd(set->hac_radius,minimum_radius,maximum_radius);
}

static double predictor_hac_candidate_residual(const PredictorGuidance*g,
        GeoPoint geo,const AtmosState*a,double course,const PlanetModel*p,
        const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,
        double hac_side,double hac_radius,double minimum_turn_radius,
        double*path_out){
    HACGuidance h=hac_guidance_compute_radius(
        geo,a->speed,course,site,set,p->radius,hac_side,
        planet_surface_gravity(p),hac_radius);
    double path=0.0,slope=0.0;
    double offset=fabs(norm_signed_deg(course-site->runway_heading));
    bool maneuver=taem_alignment_maneuver_feasible(
        geo.altitude,a->speed,hac_radius,minimum_turn_radius,
        a->drag_accel,offset,p,site,v,set,&path,&slope);
    if(path_out)*path_out=path;

    double minimum_speed=fmax(v->minimum_safe_speed,v->final_approach_speed);
    double target_speed=entry_taem_speed_target(v,set,p);
    double remaining=g->hac_progress_valid?g->hac_remaining:h.arc_remaining;
    double desired=h.desired_altitude+
        (remaining-h.arc_remaining)*tan(set->taem_glide_slope*DEG2RAD);
    double minimum_energy=entry_remaining_specific_energy(
        geo.latitude,geo.altitude,a->speed,site->latitude,desired,
        minimum_speed,p);

    double radius_residual=fabs(h.radial_error)/fmax(hac_radius,DBL_MIN);
    double course_residual=fabs(h.course_error)/180.0;
    double speed_residual=a->speed>=minimum_speed?0.0:
        (minimum_speed-a->speed)/fmax(minimum_speed,DBL_MIN);
    double energy_scale=fmax(
        planet_surface_gravity(p)*fmax(geo.altitude-site->altitude,0.0),
        .5*fmax(a->speed*a->speed,DBL_MIN));
    double energy_residual=isfinite(minimum_energy)?
        fmax(0.0,-minimum_energy)/fmax(energy_scale,DBL_MIN):INFINITY;
    double maneuver_residual=maneuver?0.0:1.0;

    (void)target_speed;(void)slope;
    return fmax(radius_residual,
        fmax(course_residual,
            fmax(speed_residual,
                fmax(energy_residual,maneuver_residual))));
}

static bool predictor_hac_ready(const PredictorGuidance*g,VehicleState state,
        GeoPoint geo,const AtmosState*a,const PlanetModel*p,
        const TrajectoryCalibrationModel*c,const VehicleProfile*v,
        const LandingSite*site,const GuidanceSettings*set,double maxbank,
        double hac_side,double*radius_out,double*score_out){
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    double course=surface_course(state.position,state.velocity,
        planet_rotation_vector(p),p->north_axis,initial_bearing(geo,sp));

    double minimum_radius=0.0,maximum_radius=0.0;
    if(!predictor_terminal_hac_radius_bounds(
            a,p,c,v,set,maxbank,&minimum_radius,&maximum_radius))
        return false;
    if(radius_out)*radius_out=INFINITY;
    if(score_out)*score_out=INFINITY;

    double minimum_speed=fmax(v->minimum_safe_speed,v->final_approach_speed);
    if(a->speed<minimum_speed)return false;

    double usable_bank=bank_limit(a,p,v,maxbank);
    double minimum_turn=turn_radius_for_bank(a,c,usable_bank);
    if(!isfinite(minimum_turn))return false;

    bool found=false;
    double best_radius=INFINITY,best_delivery=INFINITY;
    const int samples=25; /* numerical resolution only */
    double log_min=log(minimum_radius);
    double log_max=log(maximum_radius);

    for(int i=0;i<samples;i++){
        double f=(double)i/(double)(samples-1);
        double hac_radius=exp(log_min+(log_max-log_min)*f);
        double path=0.0;
        double residual=predictor_hac_candidate_residual(
            g,geo,a,course,p,v,site,set,hac_side,hac_radius,
            minimum_turn,&path);
        bool ready=isfinite(residual)&&residual<=sqrt(DBL_EPSILON);
        double delivery=isfinite(path)&&a->speed>DBL_MIN?
            path/a->speed:INFINITY;
        if(ready&&(!found||delivery<best_delivery)){
            found=true;
            best_radius=hac_radius;
            best_delivery=delivery;
        }

        if(getenv("KSP_LANDER_TRACE_TAEM")&&i==samples-1){
            fprintf(stderr,
                "taem-hac feasibility: alt=%.0f speed=%.0f R=[%.0f,%.0f]km side=%+.0f best=%s residual=%.4f\n",
                geo.altitude,a->speed,minimum_radius/1000.0,
                maximum_radius/1000.0,hac_side,
                found?"yes":"no",residual);
        }
    }

    if(found){
        if(radius_out)*radius_out=best_radius;
        if(score_out)*score_out=best_delivery;
    }
    return found;
}

static double predictor_hac_soft_cost(const PredictorGuidance*g,
        VehicleState state,GeoPoint geo,const AtmosState*a,
        const PlanetModel*p,const TrajectoryCalibrationModel*c,
        const VehicleProfile*v,const LandingSite*site,
        const GuidanceSettings*set,double maxbank,double hac_side,
        double hac_radius){
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    double course=surface_course(state.position,state.velocity,
        planet_rotation_vector(p),p->north_axis,initial_bearing(geo,sp));
    double usable_bank=bank_limit(a,p,v,maxbank);
    double minimum_turn=turn_radius_for_bank(a,c,usable_bank);
    if(!isfinite(minimum_turn))return INFINITY;
    return predictor_hac_candidate_residual(
        g,geo,a,course,p,v,site,set,hac_side,hac_radius,
        minimum_turn,NULL);
}

static double post_taem_bank(PredictorGuidance*g,VehicleState state,GeoPoint geo,const AtmosState*a,const PlanetModel*p,const TrajectoryCalibrationModel*c,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double hac_side,double dt){
    GeoPoint sp={site->latitude,site->longitude,site->altitude};
    double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,sp));
    double along=0,cross=0;runway_coordinates(geo,sp,site->runway_heading,p->radius,&along,&cross);
    Vector3 up=vnorm(state.position,v3(0,1,0));
    double vertical=vdot(a->air_velocity,up),horizontal=vmag(vproject_plane(a->air_velocity,up));
    double fpa=atan2(vertical,fmax(horizontal,.1))*RAD2DEG,range=great_circle_distance(geo,sp,p->radius);
    double gravity=planet_surface_gravity(p);
    double hac_radius=g->hac_radius>0?g->hac_radius:set->hac_radius;
    HACGuidance h=hac_guidance_compute_radius(geo,a->speed,course,site,set,p->radius,hac_side,gravity,hac_radius);

    bool fresh_hac=!g->hac_progress_valid;
    if(fresh_hac){
        g->hac_remaining=h.arc_remaining;
        g->hac_previous_angle=h.angle;
        g->hac_progress_valid=fabs(h.radial_error)<=fmax(250,fmin(1500.0,hac_radius*.10))&&fabs(h.course_error)<24;
    }else if(!g->hac_completed){
        double progress=-hac_side*norm_signed_deg((h.angle-g->hac_previous_angle)*RAD2DEG)*DEG2RAD;
        g->hac_remaining-=progress*hac_radius;
        g->hac_previous_angle=h.angle;
    }

    g->hac_previous_angle=h.angle;
    double lead=clampd(horizontal*2.0,180.0,2400.0),station=-set->final_approach_distance;
    double min_turn=turn_radius_for_bank(a,c,fmin(55.0,bank_limit(a,p,v,maxbank)));
    bool approach=terminal_approach_valid(geo,range,along,cross,course,a->speed,horizontal,vertical,fpa,site,v,set);
    if(fresh_hac&&along>=station-lead&&along<=station+fmax(200,hac_radius*.012)&&
       g->hac_remaining>LANDER_PI*hac_radius)g->hac_remaining-=2*LANDER_PI*hac_radius;
    if(!g->hac_completed&&!approach&&g->hac_remaining<=lead+fmax(200,hac_radius*.012)&&
       along>=station-lead&&along<=station+fmax(200,hac_radius*.012)&&
       fabs(cross)<fmax(250,fmin(1000,hac_radius*.10))&&fabs(norm_signed_deg(course-site->runway_heading))<12){
        double arc=0,slope=0;
        if(hac_high_pass_circuit(geo.altitude,a->speed,hac_radius,g->hac_remaining,min_turn,a->drag_accel,
            p,site,v,set,&arc,&slope)){
            g->hac_remaining=arc;g->hac_circuit_slope=slope;g->hac_progress_valid=true;
        }
    }
    double slope=g->hac_circuit_slope>0?g->hac_circuit_slope:set->taem_glide_slope;
    double desired_altitude=site->altitude+set->final_approach_distance*tan(set->final_glide_slope*DEG2RAD)+
        (g->hac_remaining+fabs(h.radial_error)*.65)*tan(slope*DEG2RAD);
    double path=fmax(set->final_approach_distance,g->hac_remaining+set->final_approach_distance);
    double target_speed=fmax(v->final_approach_speed*1.12,v->minimum_safe_speed*1.08);
    double energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,a->speed,site->latitude,site->altitude,target_speed,p);
    double removable=fmax(.03,a->drag_accel)*path*1.25;
    bool energy_ok=fabs(geo.altitude-desired_altitude)<=8000&&a->speed>=v->minimum_safe_speed&&isfinite(energy_error)&&energy_error>=-.04*target_speed*target_speed&&energy_error<=removable;
    bool hac_captured=min_turn*1.15<=hac_radius&&energy_ok&&fabs(h.radial_error)<=fmax(250,fmin(1500.0,hac_radius*.10))&&fabs(h.course_error)<24;
    double exit_window=fmax(200,hac_radius*.012);
    bool lost=!isfinite(min_turn)||min_turn*1.15>hac_radius||
        fabs(h.radial_error)>fmax(750.0,fmin(5000.0,hac_radius*.25))||fabs(h.course_error)>60;
    g->hac_lost_duration=lost?g->hac_lost_duration+dt:fmax(0,g->hac_lost_duration-dt*2);
    bool missed=!approach&&along>station+exit_window&&fabs(norm_signed_deg(course-site->runway_heading))<45&&
        (!g->hac_progress_valid||g->hac_remaining<=exit_window);
    if(!g->final_captured&&(g->hac_lost_duration>=5||missed)){
        /* Terrain is unavailable in the reduced model: site-relative height
           approximates the live radar-height guard. Arrival metrics remain
           historical; only the active guidance ownership is reset. */
        if(geo.altitude>=fmax(12000.0,set->taem_interface_altitude*.5)&&
           geo.altitude-site->altitude>=8000&&a->speed>=v->minimum_safe_speed*2){
            g->hac_return_to_entry=true;g->terminal_reentry_after_ut=state.ut+15;
            g->hac_progress_valid=false;g->hac_completed=false;g->hac_remaining=0;g->hac_radius=0;
            g->hac_circuit_slope=0;g->hac_lost_duration=0;g->final_heading_lock=false;
            g->replay_event=false;g->force_control=false;g->established=false;
            if(fabs(g->actual_bank)>=4)g->sign=g->actual_bank>=0?1:-1;
            return norm_signed_deg(jerk_angle_update(&g->limiter,0,set->entry_roll_rate,set->entry_roll_acceleration,dt));
        }
        if(missed)g->hac_failed=true;
    }
    if(!g->hac_completed&&g->hac_progress_valid&&g->hac_remaining<=exit_window){
        if(g->hac_remaining>=-exit_window&&hac_captured&&approach&&along<0){g->hac_completed=true;g->hac_remaining=0;}
        /* Preserve signed overshoot; never recycle the exit window. */
    }

    if(g->hac_completed&&approach&&along<0)g->final_captured=true;

    double raw,rate,acc,floor;
    if(g->final_captured){
        double dist=fmax(0,-along),look=clampd(a->speed*5.5,450,2800);
        double inter=clampd(atan2(-cross,look)*RAD2DEG,-28,28),target=norm_deg(site->runway_heading+inter);
        double err=norm_signed_deg(target-course),lat=2*horizontal*horizontal/fmax(look,1)*sin(err*DEG2RAD);
        double frac=clampd(dist/fmax(set->final_approach_distance,1),0,1),lim=fmin(30,bank_limit(a,p,v,maxbank));
        lim=fmin(lim,8+22*sqrt(frac));
        double radar=fmax(0,geo.altitude-site->altitude);
        if(radar<set->flare_altitude*4)lim=fmin(lim,6+10*clampd(radar/fmax(set->flare_altitude*4,1),0,1));
        raw=clampd(atan(lat/fmax(gravity,.1))*RAD2DEG,-lim,lim);
        rate=set->approach_roll_rate;acc=fmax(1.4,set->entry_roll_acceleration*.55);floor=.60;
    }else{
        double lateral=h.lateral_acceleration;
        double eff=clampd(c->bank_effectiveness,.35,1.8);
        double ratio=fabs(lateral)/fmax(a->lift_accel,.01);
        raw=copysign(asin(clampd(ratio,0,sin(clampd(fmin(maxbank,75)*eff,0,89)*DEG2RAD)))/eff*RAD2DEG,lateral);
        raw=clampd(raw,-maxbank,maxbank);
        rate=set->taem_roll_rate;acc=fmax(2,set->entry_roll_acceleration*.85);floor=.50;
    }
    FlightControlRegime regime=flight_control_regime(a->mach,a->speed,a->q,v->minimum_safe_speed);
    rate*=fmax(floor,regime.command_rate_scale);acc*=fmax(floor*.72,regime.command_accel_scale);
    return norm_signed_deg(jerk_angle_update(&g->limiter,raw,rate,acc,dt));
}

void entry_prediction_clear(EntryPrediction*p){if(!p)return;trajectory_clear(&p->trajectory);memset(p,0,sizeof(*p));}
static EntryPrediction predictor_simulate_entry_core(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env0,const TrajectoryCalibrationModel*cal0,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double initial_sign,double initial_leg,bool initial_final_heading_lock,double maxdur,bool include,bool force_control,double forced_bank,double forced_aoa,double forced_duration,double control_horizon_duration,bool stop_at_taem,bool enforce_terminal_delivery_budget,const EntryControlPlan*replay,const TaemInterfaceTarget*interface_target){
    EntryPrediction out;memset(&out,0,sizeof(out));trajectory_init(&out.trajectory);out.closest_distance=DBL_MAX;out.taem_distance=DBL_MAX;out.taem_hac_capture_score=DBL_MAX;out.best_terminal_capture_cost=DBL_MAX;out.taem_altitude=NAN;out.taem_desired_altitude=NAN;out.minimum_entry_speed=DBL_MAX;out.minimum_bank_control_margin=INFINITY;out.minimum_aoa_control_margin=INFINITY;
    AerodynamicEnvelope uniform;if(!env0){for(int i=0;i<4;i++)uniform.regimes[i]=aero;env0=&uniform;}
    TrajectoryCalibrationModel dc={
        .density_scale=1,.drag_scale=1,.lift_scale=1,.bank_effectiveness=1,.speed_of_sound=340,
        .speed_of_sound_scale=1,
        .altitude_residual=0,.speed_residual=0,.range_residual=0,.confidence=.05,.accepted_samples=0
    };if(!cal0)cal0=&dc;
    PredictorGuidance g;memset(&g,0,sizeof(g));g.sign=fabs(initial_sign)>.1?(initial_sign>=0?1:-1):0;g.leg_elapsed=fmax(0,initial_leg);g.established=initial_leg>0;g.final_heading_lock=initial_final_heading_lock;g.limiter.has_value=true;g.limiter.value=norm_deg(initial_bank);g.limiter.rate=0;g.actual_bank=norm_signed_deg(initial_bank);g.actual_bank_rate=clampd(initial_bank_rate,-60,60);g.actual_aoa=isfinite(initial_aoa)?clampd(initial_aoa,-v->maximum_angle_of_attack*1.5,v->maximum_angle_of_attack*1.5):scheduled_aoa(vmag(state.position)-p->radius,p,v);g.actual_aoa_rate=isfinite(initial_aoa_rate)?clampd(initial_aoa_rate,-15,15):0;g.force_control=force_control&&forced_duration>0;g.forced_until_ut=state.ut+fmax(0,forced_duration);g.control_horizon_ut=state.ut+fmax(0,control_horizon_duration);g.forced_bank=forced_bank;g.forced_aoa=forced_aoa;g.enforce_terminal_delivery_budget=enforce_terminal_delivery_budget;
    g.entry_loaded=initial_leg>0;
    g.configured_leg_dwell_satisfied=initial_leg+1e-6>=set->s_turn_minimum_leg_duration;
    if(interface_target&&interface_target->valid){g.has_interface_target=true;g.interface_target=*interface_target;}
    LandingConfiguration interface_cfg={.site=*site,.vehicle=*v,.guidance=*set};
    /* Never seed an established leg on the opposite measured bank. */
    if(g.sign*g.actual_bank<=0||fabs(g.actual_bank)<4){g.established=false;g.leg_elapsed=0;}
    if(replay&&replay->has_planned_reversal&&isfinite(replay->planned_reversal_ut)&&
            fabs(replay->planned_reversal_sign)>.1){
        g.replay_event=true;g.replay_event_ut=replay->planned_reversal_ut;
        g.replay_event_sign=replay->planned_reversal_sign;
        g.replay_event_final=replay->planned_reversal_is_final;
    }
    GeoPoint target={site->latitude,site->longitude,site->altitude};double entry_alt=entry_guidance_start_altitude(p,set),elapsed=0,lastsample=-DBL_MAX,hac_side=1;bool hac_set=false;
    double mm304_gate_altitude=fmin(50000.0,entry_alt-5000.0);
    bool initial_gear=cal0->physics?cal0->physics->gear:false,initial_brakes=cal0->physics?cal0->physics->brakes:false;
    int initial_airbrakes=cal0->physics&&cal0->physics->airbrakes>=0?cal0->physics->airbrakes:0;
    speedbrake_controller_reset(&g.speedbrake,initial_airbrakes!=0);
    AeroContext ctx={.p=p,.e=env0,.c=cal0,.v=v,.bank=g.actual_bank,.aoa=g.actual_aoa,.ut=state.ut,.mass=state.mass,.gear=initial_gear,.brakes=initial_brakes,.airbrakes=initial_airbrakes};
    while(elapsed<maxdur){
        GeoPoint geo=predictor_geo_point(state.position,p,state.ut);double range=great_circle_distance(geo,target,p->radius);
        AtmosState at=atmosphere(state.position,state.velocity,&ctx);Vector3 up=vnorm(state.position,v3(0,1,0));double vs=vdot(at.air_velocity,up),hs=vmag(vproject_plane(at.air_velocity,up)),fpa=atan2(vs,fmax(hs,.1))*RAD2DEG;
        if(force_control&&!out.control_horizon_recorded&&state.ut>=g.control_horizon_ut){
            out.control_horizon_recorded=true;out.control_horizon_speed=at.speed;out.control_horizon_altitude=geo.altitude;out.control_horizon_range=range;
            out.control_horizon_specific_energy=rotating_specific_energy(geo.latitude,geo.altitude,at.speed,p);
            double along=0,cross=0;runway_coordinates(geo,target,site->runway_heading,p->radius,&along,&cross);
            out.control_horizon_alignment_distance=-along-set->final_approach_distance;
        }
        if(!out.entered_atmosphere&&geo.altitude<=entry_alt&&vs<0){out.entered_atmosphere=true;out.entry_range=range;out.entry_flight_path_angle=fpa;out.entry_speed=at.speed;double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));out.entry_course_error=norm_signed_deg(initial_bearing(geo,target)-course);}
        if(out.entered_atmosphere&&!out.mm304_gate_recorded&&geo.altitude<=mm304_gate_altitude&&vs<0.0){
            double gate_along=0.0,gate_cross=0.0;
            runway_coordinates(geo,target,site->runway_heading,p->radius,&gate_along,&gate_cross);
            double gate_course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
            out.mm304_gate_recorded=true;out.mm304_gate_range=range;
            out.mm304_gate_along_track=gate_along;out.mm304_gate_cross_track=gate_cross;
            out.mm304_gate_course=gate_course;out.mm304_gate_altitude=geo.altitude;
            out.mm304_gate_speed=at.speed;out.mm304_gate_flight_path_angle=fpa;
        }
        /* Closest approach is an entry-trajectory quantity. Counting orbital
           ground-track passes before atmospheric interface can falsely mark
           the TAEM shell as already crossed and can also make a bad deorbit
           candidate look like a close KSC capture. */
        if(out.entered_atmosphere)out.closest_distance=fmin(out.closest_distance,range);
        double ownership_along=0.0,ownership_cross=0.0;
        bool ownership_ready=false,ownership_boundary_missed=false;
        if(stop_at_taem&&out.entered_atmosphere){
            runway_coordinates(geo,target,site->runway_heading,p->radius,&ownership_along,&ownership_cross);
            if(g.has_interface_target){
                Telemetry capture_state={0};
                capture_state.latitude=geo.latitude;capture_state.mean_altitude=geo.altitude;
                capture_state.true_air_speed=at.speed;capture_state.horizontal_speed=hs;
                capture_state.vertical_speed=vs;capture_state.flight_path_angle=fpa;
                capture_state.runway_along_track=ownership_along;capture_state.runway_cross_track=ownership_cross;
                capture_state.dynamic_pressure=at.q;capture_state.g_force=at.non_gravity/fmax(.1,planet_surface_gravity(p));
                capture_state.bank_effectiveness=cal0->bank_effectiveness;
                double capture_course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
                TaemInterfaceCapture capture=entry_taem_interface_capture(
                    &g.interface_target,&capture_state,capture_course,p,&interface_cfg);
                ownership_ready=capture.ready;
                /*
                 * The fixed MM304 point is a one-way ownership surface.  Once
                 * the propagated state has passed it without satisfying the
                 * mission contract, that forecast is a miss.  Energy or minimum
                 * safe-speed deficits are independently irreversible in an
                 * unpowered vehicle.  No downstream/crossrange grace box is
                 * invented here.
                 */
                ownership_boundary_missed=capture.valid&&!capture.ready&&
                    (capture.along<0.0||capture.energy_margin<0.0||
                     at.speed<v->minimum_safe_speed);
            }else{
                ownership_ready=entry_taem_handoff_geometry_ready(geo.altitude,ownership_along,vs,hs,set);
                if(!ownership_ready){
                    double handoff_low=0.0,handoff_high=0.0;
                    entry_taem_handoff_altitude_bounds(set,&handoff_low,&handoff_high);
                    ownership_boundary_missed=
                        ownership_along>=-set->final_approach_distance||
                        geo.altitude<handoff_low;
                }
            }
        }
        if(out.entered_atmosphere&&!out.reached_taem){
            out.minimum_entry_speed=fmin(out.minimum_entry_speed,at.speed);
            out.maximum_abs_angle_of_attack=fmax(out.maximum_abs_angle_of_attack,fabs(g.actual_aoa));
            if(force_control&&state.ut<g.forced_until_ut){
                /* Forced replay evaluates an already-committed live command. Do not
                   apply the autonomous 60 deg candidate-search ceiling here. */
                double available_bank=bank_limit(&at,p,v,maxbank);
                out.minimum_bank_control_margin=fmin(out.minimum_bank_control_margin,available_bank-fabs(g.forced_bank));
                out.minimum_aoa_control_margin=fmin(out.minimum_aoa_control_margin,
                    fmin(g.forced_aoa,v->maximum_angle_of_attack-g.forced_aoa));
            }
        }
        out.peak_dynamic_pressure=fmax(out.peak_dynamic_pressure,at.q);out.peak_g_load=fmax(out.peak_g_load,at.non_gravity/fmax(planet_surface_gravity(p),.1));
        double terminal_radius=INFINITY;
        bool terminal_ready=false;
        if(out.entered_atmosphere&&!hac_set&&state.ut>=g.terminal_reentry_after_ut){
            double grav=planet_surface_gravity(p),candidate_radius=predictor_terminal_hac_radius(&at,p,cal0,v,set,maxbank);
            if(isfinite(candidate_radius)){
                double course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
                double positive_radius=INFINITY,negative_radius=INFINITY;
                double ps=INFINITY,ns=INFINITY;
                bool positive_ready=predictor_hac_ready(&g,state,geo,&at,p,cal0,v,site,set,maxbank,1,&positive_radius,&ps);
                bool negative_ready=predictor_hac_ready(&g,state,geo,&at,p,cal0,v,site,set,maxbank,-1,&negative_radius,&ns);
                terminal_ready=positive_ready||negative_ready;
                /* Match live acquisition: compare scores only among feasible
                   circles. The geometrically closer side may fail energy. */
                hac_side=terminal_ready?((positive_ready&&(!negative_ready||ps<=ns))?1:-1):(ps<=ns?1:-1);
                terminal_radius=hac_side>0?positive_radius:negative_radius;
                hac_set=terminal_ready;
                double positive_soft=predictor_hac_soft_cost(&g,state,geo,&at,p,cal0,v,site,set,maxbank,1,candidate_radius);
                double negative_soft=predictor_hac_soft_cost(&g,state,geo,&at,p,cal0,v,site,set,maxbank,-1,candidate_radius);
                double soft=fmin(positive_soft,negative_soft);
                if(isfinite(soft))out.best_terminal_capture_cost=fmin(out.best_terminal_capture_cost,soft);
                if(terminal_ready&&!stop_at_taem){
                    g.hac_radius=terminal_radius;
                    HACGuidance h=hac_guidance_compute_radius(geo,at.speed,course,site,set,p->radius,hac_side,grav,terminal_radius);
                    if(!out.reached_taem){
                    double target_speed=entry_taem_speed_target(v,set,p);
                    out.reached_taem=true;out.taem_distance=range;out.taem_range_error=h.radial_error;out.taem_speed=at.speed;out.taem_flight_path_angle=fpa;
                    out.taem_altitude=geo.altitude;out.taem_desired_altitude=set->taem_interface_altitude;
                    runway_coordinates(geo,target,site->runway_heading,p->radius,&out.taem_along_track,&out.taem_cross_track);
                    out.taem_course=course;
                    /* Entry owns energy management only down to the TAEM
                       interface. The previous residual still targeted final-
                       approach speed here, so a perfectly valid 500-700 m/s
                       TAEM handoff was scored as huge excess energy and MPC
                       kept dragging the shuttle long after it should have
                       handed control to HAC. TAEM/HAC owns the subsequent
                       deceleration toward final approach. */
                    out.taem_hac_capture_score=hac_side>0?ps:ns;out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,site->latitude,h.desired_altitude,target_speed,p);
                    }
                }
            }
        }
        if(ownership_ready&&!out.reached_taem){
            double target_speed=g.has_interface_target?g.interface_target.speed:entry_taem_speed_target(v,set,p);
            double target_altitude=g.has_interface_target?g.interface_target.altitude:set->taem_interface_altitude;
            out.reached_taem=true;out.taem_distance=range;
            out.taem_range_error=range-entry_taem_range_target(p,set);out.taem_speed=at.speed;out.taem_flight_path_angle=fpa;
            out.taem_altitude=geo.altitude;out.taem_desired_altitude=target_altitude;
            out.taem_along_track=ownership_along;out.taem_cross_track=ownership_cross;
            out.taem_course=surface_course(state.position,state.velocity,planet_rotation_vector(p),p->north_axis,initial_bearing(geo,target));
            out.taem_energy_error=entry_remaining_specific_energy(geo.latitude,geo.altitude,at.speed,site->latitude,
                target_altitude,target_speed,p);
        }
        if(stop_at_taem&&(out.reached_taem||ownership_boundary_missed)){
            out.taem_ownership_boundary_missed=ownership_boundary_missed;
            /* Preserve the exact accepted/missed interface state even when the
               prediction terminates before the normal six-second trace cadence. */
            if(include&&(out.trajectory.count==0||
               fabs(out.trajectory.points[out.trajectory.count-1].ut-state.ut)>1e-6)){
                GuidancePhase stop_phase=out.reached_taem?PHASE_TAEM:PHASE_ENTRY_ENERGY;
                TrajectoryPoint tp={state.ut,geo.latitude,geo.longitude,geo.altitude,at.speed,stop_phase,TRAJ_PLANNED};
                trajectory_append(&out.trajectory,tp);lastsample=state.ut;
            }
            /* A forced Entry-plan forecast is an executable shadow, not permission
               to invent another Entry segment after live MM304 crosses its one-way
               V_TAEM ownership boundary. A miss stays reached_taem=false so
               supervision cannot discover a later synthetic recovery. */
            break;
        }
        GuidancePhase phase=geo.altitude>entry_alt?PHASE_COAST:g.final_captured?PHASE_FINAL:(g.hac_progress_valid||hac_set)?PHASE_TAEM:PHASE_ENTRY_ENERGY;
        if(include&&state.ut-lastsample>=6){TrajectoryPoint tp={state.ut,geo.latitude,geo.longitude,geo.altitude,at.speed,phase,TRAJ_PLANNED};trajectory_append(&out.trajectory,tp);lastsample=state.ut;}
        if(geo.altitude<=fmax(site->altitude,0)&&vs<0)break;if(out.reached_taem&&range<500&&geo.altitude<1000)break;if(vmag(state.position)<p->radius*.8||!isfinite(vmag(state.position)))break;
        double frac=clampd(geo.altitude/fmax(1,p->atmosphere_depth),0,1),base_dt=geo.altitude>p->atmosphere_depth?8:clampd(.25+frac*1.55,.25,1.8);
        double dt=predictor_step_size(&at,p,v,set,geo.altitude,vs,entry_alt,out.entered_atmosphere,out.reached_taem,g.limiter.rate,base_dt),aoa=NAN;AerodynamicModel local=aero_at(env0,at.mach);
        double bank=0,target_aoa=0;
        if(!out.entered_atmosphere){
            /* Match live Entry Interface: wings level and maximum entry AoA,
               including the thin-air interval before S-turn engagement. */
            FlightControlRegime regime=flight_control_regime(at.mach,at.speed,at.q,v->minimum_safe_speed);
            double rate=set->entry_roll_rate*fmax(.55,regime.command_rate_scale),acc=set->entry_roll_acceleration*fmax(.45,regime.command_accel_scale);
            bank=norm_signed_deg(jerk_angle_update(&g.limiter,0,rate,acc,dt));
            target_aoa=v->maximum_angle_of_attack;
        }else{
            if(out.reached_taem&&hac_set){
                bank=post_taem_bank(&g,state,geo,&at,p,cal0,v,site,set,maxbank,hac_side,dt);
                if(g.hac_return_to_entry){hac_set=false;g.hac_return_to_entry=false;}
                if(g.hac_failed||(!g.final_captured&&g.hac_remaining < -fmax(200,g.hac_radius*.012)))break;
                target_aoa=clampd(scheduled_aoa(geo.altitude,p,v),4,14);
                if(g.hac_circuit_slope>0&&!g.final_captured)
                    target_aoa=clampd(-g.hac_circuit_slope+7.0-fpa,3.0,14.0);
            }else{
                bank=entry_bank(&g,state,geo,&at,p,local,cal0,v,site,set,maxbank,dt,&aoa);
                target_aoa=aoa;
            }
        }
        attitude_step(&g,bank,target_aoa,&at,v,set,dt,cal0->physics);
        if(out.entered_atmosphere&&!out.reached_taem)out.maximum_abs_angle_of_attack=fmax(out.maximum_abs_angle_of_attack,fabs(g.actual_aoa));
        ctx.bank=g.actual_bank;ctx.aoa=g.actual_aoa;ctx.ut=state.ut;
        ctx.airbrakes=predictor_speedbrake_state(&g,geo,&at,p,cal0,v,site,set,state.mass,out.reached_taem,dt);
        /* Wheel brakes belong only to rollout; gear follows the same terminal
           altitude gate as live guidance so direct-force lookup does not use
           clean-airframe cells after predicted deployment. */
        ctx.brakes=false;ctx.gear=out.reached_taem&&geo.altitude-site->altitude<=set->gear_deployment_altitude;
        out.s_turn_max_bank=fmax(out.s_turn_max_bank,fabs(g.actual_bank));out.s_turn_reversals=g.reversals;
        if(at.q>1){if(at.direct_aero)out.physics_observed_seconds+=dt;else out.physics_fallback_seconds+=dt;}
        state=rk4_aero(state,dt,&ctx);elapsed+=dt;
    }
    if(force_control&&!out.control_horizon_recorded){GeoPoint geo=predictor_geo_point(state.position,p,state.ut);AtmosState at=atmosphere(state.position,state.velocity,&ctx);out.control_horizon_recorded=true;out.control_horizon_speed=at.speed;out.control_horizon_altitude=geo.altitude;out.control_horizon_range=great_circle_distance(geo,target,p->radius);out.control_horizon_specific_energy=rotating_specific_energy(geo.latitude,geo.altitude,at.speed,p);
            double along=0,cross=0;runway_coordinates(geo,target,site->runway_heading,p->radius,&along,&cross);
            out.control_horizon_alignment_distance=-along-set->final_approach_distance;}
    out.has_first_s_turn_reversal=g.has_first_reversal;
    out.first_s_turn_reversal_is_final=g.first_reversal_final;
    out.first_s_turn_reversal_ut=g.first_reversal_ut;
    out.first_s_turn_reversal_range=g.first_reversal_range;
    out.first_s_turn_reversal_sign=g.first_reversal_sign;
    if(out.minimum_entry_speed==DBL_MAX)out.minimum_entry_speed=NAN;
    if(!out.reached_taem){GeoPoint geo=predictor_geo_point(state.position,p,state.ut);out.taem_distance=great_circle_distance(geo,target,p->radius);out.taem_range_error=isfinite(out.closest_distance)?out.taem_distance-out.closest_distance:out.taem_distance;out.taem_speed=atmosphere(state.position,state.velocity,&ctx).speed;out.taem_energy_error=INFINITY;}out.final_state=state;return out;
}

EntryPrediction predictor_simulate_entry_with_attitude(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double initial_bank,double initial_bank_rate,double initial_aoa,double initial_aoa_rate,double initial_sign,double initial_leg,double maxdur,bool include){
    return predictor_simulate_entry_core(state,p,aero,env,cal,v,site,set,maxbank,initial_bank,initial_bank_rate,initial_aoa,initial_aoa_rate,initial_sign,initial_leg,false,maxdur,include,false,0,0,0,0,false,false,NULL,NULL);
}

EntryPrediction predictor_simulate_entry_with_rate(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double initial_bank,double initial_bank_rate,double initial_sign,double initial_leg,double maxdur,bool include){
    return predictor_simulate_entry_with_attitude(state,p,aero,env,cal,v,site,set,maxbank,initial_bank,initial_bank_rate,NAN,NAN,initial_sign,initial_leg,maxdur,include);
}

EntryPrediction predictor_simulate_entry(VehicleState state,const PlanetModel*p,AerodynamicModel aero,const AerodynamicEnvelope*env,const TrajectoryCalibrationModel*cal,const VehicleProfile*v,const LandingSite*site,const GuidanceSettings*set,double maxbank,double initial_bank,double initial_sign,double initial_leg,double maxdur,bool include){
    return predictor_simulate_entry_with_rate(state,p,aero,env,cal,v,site,set,maxbank,initial_bank,0,initial_sign,initial_leg,maxdur,include);
}

static double predictor_normalized_upper_violation(double value,double limit){
    if(!isfinite(value)||!isfinite(limit)||!(limit>0.0))return INFINITY;
    return fmax(0.0,value/limit-1.0);
}

