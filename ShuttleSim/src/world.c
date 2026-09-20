#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "shuttlesim/world.h"
#include "shuttlesim/math3.h"

static const double PI = 3.14159265358979323846;

void world_seed_kerbin(KerbinWorld *w) {
    memset(w,0,sizeof(*w));
    w->radius_m = 600000.0;
    w->mu_m3_s2 = 3.5316e12;
    w->rotation_rate_rad_s = 2.0*PI/21549.425;
    w->atmosphere_top_m = 70000.0;
    w->rotation_phase_rad_at_ut0 = PI/2.0; /* KSP inertial frame: +90 deg phase at UT=0. */
    /* Seed table: deliberately replaceable by log/export-derived KSP table. */
    const double h[] = {0,5000,10000,15000,20000,25000,30000,35000,40000,45000,50000,55000,60000,65000,70000};
    const double rho[] = {1.225,0.736,0.414,0.211,0.098,0.040,0.015,0.0053,0.00175,0.00055,0.00017,0.000050,0.000014,0.0000034,0.0};
    const size_t n=sizeof(h)/sizeof(h[0]);
    w->atmosphere.count=n;
    for(size_t i=0;i<n;i++){
        w->atmosphere.p[i].altitude_m=h[i];
        w->atmosphere.p[i].sample.density_kg_m3=rho[i];
        w->atmosphere.p[i].sample.pressure_pa=101325.0*(rho[i]/1.225);
        w->atmosphere.p[i].sample.temperature_k=clampd(288.15-0.0030*h[i],170.0,288.15);
        w->atmosphere.p[i].sample.speed_of_sound_mps=sqrt(1.4*287.05*w->atmosphere.p[i].sample.temperature_k);
    }
}

bool world_load_atmosphere_csv(KerbinWorld *w, const char *path) {
    FILE *f=fopen(path,"r"); if(!f) return false;
    char line[512]; size_t n=0;
    while(fgets(line,sizeof(line),f) && n<ATM_TABLE_MAX){
        if(line[0]=='#' || strstr(line,"altitude")) continue;
        double h,rho,p,t,a;
        if(sscanf(line,"%lf,%lf,%lf,%lf,%lf",&h,&rho,&p,&t,&a)==5){
            w->atmosphere.p[n].altitude_m=h;
            w->atmosphere.p[n].sample=(AtmosphereSample){rho,p,t,a};
            n++;
        }
    }
    fclose(f);
    if(n<2) return false;
    w->atmosphere.count=n;
    /* Fitted tables may be padded with zero-density rows above the physical
       atmosphere.  The physical top is the first zero-density boundary after
       a positive-density sample, not the storage extent of the table. */
    w->atmosphere_top_m=w->atmosphere.p[n-1].altitude_m;
    for(size_t i=1;i<n;i++){
        if(w->atmosphere.p[i-1].sample.density_kg_m3>0.0 &&
           !(w->atmosphere.p[i].sample.density_kg_m3>0.0)){
            w->atmosphere_top_m=w->atmosphere.p[i].altitude_m;
            break;
        }
    }
    return true;
}

Vec3 world_gravity_accel(const KerbinWorld *w, Vec3 r) {
    double rn=v3_norm(r); if(rn<1.0) return v3(0,0,0);
    return v3_scale(r,-w->mu_m3_s2/(rn*rn*rn));
}

static Vec3 rotz(Vec3 v,double a){ double c=cos(a),s=sin(a); return v3(c*v.x-s*v.y,s*v.x+c*v.y,v.z); }
Vec3 world_fixed_to_inertial(const KerbinWorld *w, Vec3 fixed, double ut) { return rotz(fixed,w->rotation_phase_rad_at_ut0+w->rotation_rate_rad_s*ut); }
Vec3 world_inertial_to_fixed(const KerbinWorld *w, Vec3 inertial, double ut) { return rotz(inertial,-(w->rotation_phase_rad_at_ut0+w->rotation_rate_rad_s*ut)); }
Vec3 world_atmosphere_velocity_i(const KerbinWorld *w, Vec3 p) { return v3(-w->rotation_rate_rad_s*p.y,w->rotation_rate_rad_s*p.x,0); }

LLA world_lla(const KerbinWorld *w, Vec3 p_i, double ut) {
    Vec3 p=world_inertial_to_fixed(w,p_i,ut); double rn=v3_norm(p);
    LLA out={asin(clampd(p.z/rn,-1,1)),atan2(p.y,p.x),rn-w->radius_m}; return out;
}
Vec3 world_lla_to_inertial(const KerbinWorld *w, double lat, double lon, double alt, double ut) {
    double r=w->radius_m+alt, cl=cos(lat);
    Vec3 f=v3(r*cl*cos(lon),r*cl*sin(lon),r*sin(lat));
    return world_fixed_to_inertial(w,f,ut);
}
LocalFrame world_local_frame_i(const KerbinWorld *w, Vec3 p_i, double ut) {
    (void)w;
    (void)ut;
    Vec3 up=v3_normalized(p_i);
    Vec3 z=v3(0,0,1);
    Vec3 east=v3_normalized(v3_cross(z,up));
    if(v3_norm(east)<1e-8) east=v3(0,1,0);
    Vec3 north=v3_normalized(v3_cross(up,east));
    LocalFrame f={north,east,up}; return f;
}
AtmosphereSample world_atmosphere_sample(const KerbinWorld *w, double h) {
    AtmosphereSample z={0,0,0,0};
    if(h>=w->atmosphere_top_m || w->atmosphere.count<2) return z;
    if(h<=w->atmosphere.p[0].altitude_m) return w->atmosphere.p[0].sample;
    for(size_t i=1;i<w->atmosphere.count;i++){
        if(h<=w->atmosphere.p[i].altitude_m){
            const AtmospherePoint *a=&w->atmosphere.p[i-1], *b=&w->atmosphere.p[i];
            double t=(h-a->altitude_m)/(b->altitude_m-a->altitude_m);
            AtmosphereSample s;
            s.density_kg_m3=a->sample.density_kg_m3+(b->sample.density_kg_m3-a->sample.density_kg_m3)*t;
            s.pressure_pa=a->sample.pressure_pa+(b->sample.pressure_pa-a->sample.pressure_pa)*t;
            s.temperature_k=a->sample.temperature_k+(b->sample.temperature_k-a->sample.temperature_k)*t;
            s.speed_of_sound_mps=a->sample.speed_of_sound_mps+(b->sample.speed_of_sound_mps-a->sample.speed_of_sound_mps)*t;
            return s;
        }
    }
    return z;
}
void runway_seed_ksp09(Runway *r) {
    r->lat_rad=deg2rad(-0.0486);
    r->lon_rad=deg2rad(-74.7240);
    r->elevation_m=70.0;
    r->heading_rad=deg2rad(90.0);
    r->length_m=2500.0;
    r->width_m=70.0;
}
bool runway_contains(const Runway *r, double along_m, double cross_m) {
    if(!r || !isfinite(along_m) || !isfinite(cross_m) ||
       !isfinite(r->length_m) || !(r->length_m>0.0) ||
       !isfinite(r->width_m) || !(r->width_m>0.0)) return false;
    return along_m>=0.0 && along_m<=r->length_m &&
           fabs(cross_m)<=r->width_m*0.5;
}

void runway_coordinates(const KerbinWorld *w, const Runway *r, Vec3 p_i, double ut,double *along,double *cross,double *vertical) {
    LLA p = world_lla(w, p_i, ut);
    double lat1 = r->lat_rad, lon1 = r->lon_rad;
    double lat2 = p.lat_rad, lon2 = p.lon_rad;
    double dlon = wrap_pi(lon2 - lon1);
    double cos_delta = sin(lat1)*sin(lat2) + cos(lat1)*cos(lat2)*cos(dlon);
    double delta = acos(clampd(cos_delta, -1.0, 1.0));
    double bearing = atan2(sin(dlon)*cos(lat2),
                           cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlon));
    double rel = wrap_pi(bearing - r->heading_rad);
    double xt = asin(clampd(sin(delta)*sin(rel), -1.0, 1.0));
    double at = atan2(sin(delta)*cos(rel), cos(delta));
    if(along) *along = at * w->radius_m;
    if(cross) *cross = xt * w->radius_m;
    if(vertical) *vertical = p.altitude_m - r->elevation_m;
}
