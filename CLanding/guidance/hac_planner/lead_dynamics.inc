static bool fixed_hac_lead_vertical_step(const Telemetry*t,
        const PlanetModel*p,const VehicleProfile*v,double curvature,
        double lift_accel,double dt,double*altitude_out,double*fpa_out){
    if(!t||!p||!v||!altitude_out||!fpa_out||!(dt>0.0)||
       !isfinite(dt)||!(t->true_air_speed>0.0))return false;
    double speed=t->true_air_speed;
    double gamma=t->flight_path_angle*DEG2RAD;
    double altitude=t->mean_altitude;
    double radius=fmax(1.0,p->radius+altitude);
    double gravity=p->gravitational_parameter/(radius*radius);
    double lift=fmin(fmax(0.0,lift_accel),
        gravity*fmax(0.0,v->maximum_g_load));
    /*
     * HAC curvature is horizontal-ground-track curvature.  Only the horizontal
     * velocity component produces the required course-normal acceleration.
     * Using TAS^2 here made a steeply descending shuttle look unable to fly a
     * radius that is physically reachable once descent is accounted for.
     */
    double horizontal=speed*fabs(cos(gamma));
    double lateral=horizontal*horizontal*fabs(curvature);
    double effectiveness=clampd(t->bank_effectiveness>0.0?
        t->bank_effectiveness:1.0,.35,1.8);
    double bank_limit=fmin(v->maximum_bank_angle,dynamic_bank_limit(t,v));
    double effective_limit=clampd(bank_limit*effectiveness,0.0,
        nextafter(89.0,0.0))*DEG2RAD;
    if(lateral>lift*sin(effective_limit)+1e-6)return false;
    double effective_bank=lift>DBL_MIN?asin(clampd(lateral/lift,0.0,1.0)):0.0;
    double vertical_lift=lift*cos(effective_bank/effectiveness);
    double gamma_rate=(vertical_lift-gravity*cos(gamma))/speed+
        speed*cos(gamma)/radius;
    double next_gamma=gamma+gamma_rate*dt;
    double mid_gamma=.5*(gamma+next_gamma);
    double next_altitude=altitude+speed*sin(mid_gamma)*dt;
    if(!isfinite(next_gamma)||!isfinite(next_altitude)||next_altitude<0.0||
       next_gamma < -60.0*DEG2RAD||next_gamma > 20.0*DEG2RAD)return false;
    *altitude_out=next_altitude;
    *fpa_out=next_gamma;
    return true;
}
