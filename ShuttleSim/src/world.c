#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "shuttlesim/world.h"
#include "shuttlesim/math3.h"

static const double PI = 3.14159265358979323846;
static const double KERBIN_MOLAR_MASS_KG_MOL = 0.0289644002914429;
static const double KERBIN_ADIABATIC_INDEX = 1.39999997615814;
static const double UNIVERSAL_GAS_CONSTANT = 8.31446261815324;

typedef struct {
    double x;
    double y;
    double in_tangent;
    double out_tangent;
} CurveKey;

static double curve_eval(const CurveKey *curve,size_t count,double x){
    if(count==0)return 0.0;
    if(x<=curve[0].x)return curve[0].y;
    if(x>=curve[count-1].x)return curve[count-1].y;
    for(size_t i=1;i<count;i++){
        if(x<=curve[i].x){
            const CurveKey *a=&curve[i-1],*b=&curve[i];
            double dx=b->x-a->x,t=(x-a->x)/dx,t2=t*t,t3=t2*t;
            double h00=2.0*t3-3.0*t2+1.0;
            double h10=t3-2.0*t2+t;
            double h01=-2.0*t3+3.0*t2;
            double h11=t3-t2;
            return h00*a->y+h10*dx*a->out_tangent+h01*b->y+h11*dx*b->in_tangent;
        }
    }
    return curve[count-1].y;
}

static const CurveKey KERBIN_PRESSURE_CURVE[]={
    {0.0,101.325,0.0,-0.01501631},{1241.025,84.02916,-0.01289846,-0.01289826},
    {2439.593,69.68138,-0.01107876,-0.01107859},{3597.11,57.78001,-0.009515483,-0.009515338},
    {4714.942,47.90862,-0.00817254,-0.008172415},{5794.409,39.72148,-0.00701892,-0.007018813},
    {6836.791,32.93169,-0.006027969,-0.006027877},{7843.328,27.30109,-0.005176778,-0.0051767},
    {8815.22,22.63206,-0.004445662,-0.004445578},{10786.42,15.3684,-0.003016528,-0.00301646},
    {12101.4,11.87313,-0.002329273,-0.00232922},{13417.05,9.172798,-0.001798594,-0.001798554},
    {16678.47,4.842261,-0.0009448537,-0.0009448319},{21143.1,2.050097,-0.0003894095,-0.0003894005},
    {26977.92,0.6905929,-0.0001252565,-0.0001252534},{33593.82,0.2201734,-3.626878e-05,-3.626788e-05},
    {42081.87,0.05768469,-9.063159e-06,-9.062975e-06},{49312.13,0.01753794,-3.029397e-06,-3.029335e-06},
    {56669.95,0.004591824,-8.827175e-07,-8.826996e-07},{62300.84,0.001497072,-3.077091e-07,-3.077031e-07},
    {70000.0,0.0,0.0,0.0}
};
static const CurveKey KERBIN_TEMPERATURE_CURVE[]={
    {0.0,288.15,0.0,-0.008125},{8815.22,216.65,-0.008096968,0.0},
    {16050.39,216.65,0.0,0.001242164},{25729.23,228.65,0.001237475,0.003464929},
    {37879.44,270.65,0.00344855,0.0},{41129.24,270.65,0.0,-0.003444189},
    {57440.13,214.65,-0.003422425,-0.002444589},{68797.88,186.946,-0.002433851,0.0},
    {70000.0,186.946,0.0,0.0}
};
static const CurveKey KERBIN_TEMPERATURE_SUN_MULT_CURVE[]={
    {0.0,1.0,0.0,0.0},{8815.22,0.3,-5.91316e-05,-5.91316e-05},
    {16050.39,0.0,0.0,0.0},{25729.23,0.0,0.0,0.0},
    {37879.44,0.2,0.0,0.0},{57440.13,0.2,0.0,0.0},
    {63902.72,1.0,0.0001012837,0.0001012837},{70000.0,1.2,0.0,0.0}
};
static const CurveKey KERBIN_LATITUDE_BIAS_CURVE[]={
    {0.0,17.0,0.0,-0.3316494},{10.0,12.0,-0.65,-0.65},
    {18.0,6.36371,-0.4502313,-0.4502313},{30.0,0.0,-1.3,-1.3},
    {35.0,-10.0,-1.65,-1.65},{45.0,-23.0,-1.05,-1.05},
    {55.0,-31.0,-0.6,-0.6},{70.0,-37.0,-0.6689383,-0.6689383},
    {90.0,-50.0,-0.02418368,0.0}
};
static const CurveKey KERBIN_LATITUDE_SUN_MULT_CURVE[]={
    {0.0,9.0,0.0,0.1554984},{40.0,14.2,0.08154097,0.08154097},
    {55.0,14.9,-0.006055089,-0.006055089},{68.0,12.16518,-0.2710912,-0.2710912},
    {76.0,8.582909,-0.6021729,-0.6021729},{90.0,5.0,0.0,0.0}
};

#define ARRAY_COUNT(a) (sizeof(a)/sizeof((a)[0]))

static double kerbin_pressure_pa(double altitude_m){
    if(altitude_m>=70000.0)return 0.0;
    return fmax(0.0,1000.0*curve_eval(KERBIN_PRESSURE_CURVE,ARRAY_COUNT(KERBIN_PRESSURE_CURVE),fmax(0.0,altitude_m)));
}

static double kerbin_temperature_k(const KerbinWorld *w,double altitude_m,double lat_rad,double lon_rad,double ut){
    double h=clampd(altitude_m,0.0,70000.0);
    double lat_deg=fabs(rad2deg(lat_rad));
    double hour_angle=lon_rad+(w->rotation_rate_rad_s-w->orbital_rate_rad_s)*ut-w->solar_phase_rad_at_ut0;
    double sun_dot_normalized=0.5*cos(hour_angle-PI/4.0)+0.5;
    double offset=curve_eval(KERBIN_LATITUDE_BIAS_CURVE,ARRAY_COUNT(KERBIN_LATITUDE_BIAS_CURVE),lat_deg)+
        curve_eval(KERBIN_LATITUDE_SUN_MULT_CURVE,ARRAY_COUNT(KERBIN_LATITUDE_SUN_MULT_CURVE),lat_deg)*sun_dot_normalized;
    return curve_eval(KERBIN_TEMPERATURE_CURVE,ARRAY_COUNT(KERBIN_TEMPERATURE_CURVE),h)+
        curve_eval(KERBIN_TEMPERATURE_SUN_MULT_CURVE,ARRAY_COUNT(KERBIN_TEMPERATURE_SUN_MULT_CURVE),h)*offset;
}

static AtmosphereSample kerbin_stock_atmosphere(const KerbinWorld *w,double altitude_m,double lat_rad,double lon_rad,double ut){
    AtmosphereSample s={0,0,0,0};
    if(altitude_m>=w->atmosphere_top_m)return s;
    s.pressure_pa=kerbin_pressure_pa(altitude_m);
    s.temperature_k=kerbin_temperature_k(w,altitude_m,lat_rad,lon_rad,ut);
    if(s.pressure_pa>0.0&&s.temperature_k>0.0)
        s.density_kg_m3=s.pressure_pa*KERBIN_MOLAR_MASS_KG_MOL/(UNIVERSAL_GAS_CONSTANT*s.temperature_k);
    if(s.temperature_k>0.0)
        s.speed_of_sound_mps=sqrt(KERBIN_ADIABATIC_INDEX*(UNIVERSAL_GAS_CONSTANT/KERBIN_MOLAR_MASS_KG_MOL)*s.temperature_k);
    return s;
}

void world_seed_kerbin(KerbinWorld *w) {
    memset(w,0,sizeof(*w));
    w->radius_m=600000.0;
    w->mu_m3_s2=3.5316e12;
    w->rotation_rate_rad_s=2.0*PI/21549.425;
    w->atmosphere_top_m=70000.0;
    w->rotation_phase_rad_at_ut0=PI/2.0;
    /* Stock Kerbin orbital phase/period.  Together with the +90 deg body-frame
       phase this reproduces the longitude-dependent KSP temperature field. */
    w->orbital_rate_rad_s=2.0*PI/9203544.61750141;
    w->solar_phase_rad_at_ut0=3.14000010490417+PI/2.0;
    w->stock_spatial_atmosphere=true;

    /* Keep an altitude-only nominal table for compatibility and diagnostics.
       Flight physics uses world_atmosphere_sample_state(), which applies the
       stock latitude/day-night thermal modifiers. */
    for(size_t i=0;i<=700;i++){
        double h=100.0*(double)i;
        double temp=curve_eval(KERBIN_TEMPERATURE_CURVE,ARRAY_COUNT(KERBIN_TEMPERATURE_CURVE),h)+
            curve_eval(KERBIN_TEMPERATURE_SUN_MULT_CURVE,ARRAY_COUNT(KERBIN_TEMPERATURE_SUN_MULT_CURVE),h)*
            (curve_eval(KERBIN_LATITUDE_BIAS_CURVE,ARRAY_COUNT(KERBIN_LATITUDE_BIAS_CURVE),0.0)+
             0.5*curve_eval(KERBIN_LATITUDE_SUN_MULT_CURVE,ARRAY_COUNT(KERBIN_LATITUDE_SUN_MULT_CURVE),0.0));
        double pressure=kerbin_pressure_pa(h);
        AtmosphereSample s={0,pressure,temp,0};
        if(pressure>0.0)s.density_kg_m3=pressure*KERBIN_MOLAR_MASS_KG_MOL/(UNIVERSAL_GAS_CONSTANT*temp);
        s.speed_of_sound_mps=sqrt(KERBIN_ADIABATIC_INDEX*(UNIVERSAL_GAS_CONSTANT/KERBIN_MOLAR_MASS_KG_MOL)*temp);
        w->atmosphere.p[w->atmosphere.count++]=(AtmospherePoint){h,s};
    }
}

bool world_load_atmosphere_csv(KerbinWorld *w, const char *path) {
    FILE *f=fopen(path,"r"); if(!f) return false;
    AtmosphereTable loaded={0};
    char line[512]; bool overflow=false,order_error=false;
    bool stock_spatial=false;
    while(fgets(line,sizeof(line),f)){
        if(line[0]=='#'){
            if(strstr(line,"model=kerbin_stock_spatial"))stock_spatial=true;
            continue;
        }
        if(strstr(line,"altitude"))continue;
        double h,rho,p,t,a;
        if(sscanf(line,"%lf,%lf,%lf,%lf,%lf",&h,&rho,&p,&t,&a)==5){
            if(!isfinite(h)||!isfinite(rho)||!isfinite(p)||!isfinite(t)||!isfinite(a)||
               rho<0.0||p<0.0){ order_error=true; break; }
            if(loaded.count>0&&!(h>loaded.p[loaded.count-1].altitude_m)){ order_error=true; break; }
            if(loaded.count>=ATM_TABLE_MAX){ overflow=true; break; }
            loaded.p[loaded.count].altitude_m=h;
            loaded.p[loaded.count].sample=(AtmosphereSample){rho,p,t,a};
            loaded.count++;
        }
    }
    fclose(f);
    if(overflow||order_error||loaded.count<2) return false;

    /* Commit only after the entire file has validated.  A failed calibration
       load must never partially overwrite the seeded Kerbin atmosphere. */
    w->atmosphere=loaded;
    w->stock_spatial_atmosphere=stock_spatial;
    w->atmosphere_top_m=w->atmosphere.p[w->atmosphere.count-1].altitude_m;
    for(size_t i=1;i<w->atmosphere.count;i++){
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
            if(a->sample.density_kg_m3>0.0&&b->sample.density_kg_m3>0.0)
                s.density_kg_m3=exp(log(a->sample.density_kg_m3)+
                    (log(b->sample.density_kg_m3)-log(a->sample.density_kg_m3))*t);
            else
                s.density_kg_m3=a->sample.density_kg_m3+
                    (b->sample.density_kg_m3-a->sample.density_kg_m3)*t;
            if(a->sample.pressure_pa>0.0&&b->sample.pressure_pa>0.0)
                s.pressure_pa=exp(log(a->sample.pressure_pa)+
                    (log(b->sample.pressure_pa)-log(a->sample.pressure_pa))*t);
            else
                s.pressure_pa=a->sample.pressure_pa+(b->sample.pressure_pa-a->sample.pressure_pa)*t;
            s.temperature_k=a->sample.temperature_k+(b->sample.temperature_k-a->sample.temperature_k)*t;
            s.speed_of_sound_mps=a->sample.speed_of_sound_mps+(b->sample.speed_of_sound_mps-a->sample.speed_of_sound_mps)*t;
            return s;
        }
    }
    return z;
}
AtmosphereSample world_atmosphere_sample_state(const KerbinWorld *w,Vec3 position_i,double ut){
    LLA l=world_lla(w,position_i,ut);
    if(!w->stock_spatial_atmosphere)return world_atmosphere_sample(w,l.altitude_m);
    return kerbin_stock_atmosphere(w,l.altitude_m,l.lat_rad,l.lon_rad,ut);
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
