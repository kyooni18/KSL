/* Shared HAC selection geometry.
   This module owns only runway-relative side selection; trajectory construction
   remains in hac_path and candidate search remains in hac_planner. */

double terminal_default_hac_side(const Telemetry*t,const LandingSite*site,double course){
    if(!site)return 1.0;
    double reference=isfinite(course)?course:
        (t&&isfinite(t->ground_track_heading)?t->ground_track_heading:
         (t&&isfinite(t->heading)?t->heading:site->runway_heading));
    double turn_delta=norm_signed_deg(site->runway_heading-reference);
    double angular_resolution=sqrt(DBL_EPSILON)*RAD2DEG;
    if(turn_delta<-angular_resolution)return 1.0;
    if(turn_delta>angular_resolution)return -1.0;
    if(t&&isfinite(t->runway_cross_track)&&
       fabs(t->runway_cross_track)>sqrt(DBL_EPSILON))
        return t->runway_cross_track>=0.0?1.0:-1.0;
    return 1.0;
}
