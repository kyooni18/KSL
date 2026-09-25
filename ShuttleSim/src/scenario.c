#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include "shuttlesim/scenario.h"

static double initial_bearing_deg(double lat1_deg, double lon1_deg,
        double lat2_deg, double lon2_deg) {
    const double deg2rad = acos(-1.0) / 180.0;
    double lat1 = lat1_deg * deg2rad;
    double lat2 = lat2_deg * deg2rad;
    double dlon = (lon2_deg - lon1_deg) * deg2rad;
    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) -
        sin(lat1) * cos(lat2) * cos(dlon);
    double bearing = atan2(y, x) / deg2rad;
    bearing = fmod(bearing + 360.0, 360.0);
    return bearing < 0.0 ? bearing + 360.0 : bearing;
}

void scenario_seed_86km(Scenario *s) {
    memset(s, 0, sizeof(*s));
    strcpy(s->name, "orbit86km-seed");
    s->ut0 = 0;
    s->altitude_m = 86000;
    s->latitude_deg = -0.05;
    s->longitude_deg = 105.0;
    s->mass_kg = 85000.0;
    s->initial_aoa_deg = 0;
    s->initial_bank_deg = 0;
    s->deorbit_delta_v_mps = 60.0;
    s->deorbit_duration_s = 0.0;
    s->deorbit_delay_s = 0.0;
    s->runway_latitude_deg = -0.0486;
    s->runway_longitude_deg = -74.7240;
    s->runway_elevation_m = 70.0;
    s->runway_heading_deg = 90.0;
    s->runway_length_m = 2500.0;
    s->runway_width_m = 70.0;
    s->heading_deg = initial_bearing_deg(
        s->latitude_deg, s->longitude_deg,
        s->runway_latitude_deg, s->runway_longitude_deg);
}

static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}

bool scenario_load(const char *path, Scenario *destination) {
    if (!path || !destination) return false;
    FILE *f = fopen(path, "r");
    if (!f) return false;
    Scenario sample;
    scenario_seed_86km(&sample);
    Scenario *s = &sample;
    enum { POSITION_X=1, POSITION_Y=2, POSITION_Z=4,
           VELOCITY_X=8, VELOCITY_Y=16, VELOCITY_Z=32,
           CARTESIAN=63, RUNWAY=64, SURFACE=128, HEADING=256 };
    struct Field { const char *name; double *value; unsigned flags; } fields[] = {
        {"ut0", &s->ut0, 0},
        {"latitude_deg", &s->latitude_deg, 0},
        {"longitude_deg", &s->longitude_deg, 0},
        {"altitude_m", &s->altitude_m, 0},
        {"heading_deg", &s->heading_deg, HEADING},
        {"surface_speed_mps", &s->surface_speed_mps, SURFACE},
        {"speed_mps", &s->surface_speed_mps, SURFACE},
        {"flight_path_angle_deg", &s->flight_path_angle_deg, 0},
        {"mass_kg", &s->mass_kg, 0},
        {"initial_aoa_deg", &s->initial_aoa_deg, 0},
        {"initial_bank_deg", &s->initial_bank_deg, 0},
        {"deorbit_delta_v_mps", &s->deorbit_delta_v_mps, 0},
        {"deorbit_duration_s", &s->deorbit_duration_s, 0},
        {"deorbit_delay_s", &s->deorbit_delay_s, 0},
        {"orbital_engine_available_thrust_n", &s->orbital_engine_available_thrust_n, 0},
        {"runway_latitude_deg", &s->runway_latitude_deg, RUNWAY},
        {"runway_longitude_deg", &s->runway_longitude_deg, RUNWAY},
        {"runway_elevation_m", &s->runway_elevation_m, RUNWAY},
        {"runway_heading_deg", &s->runway_heading_deg, RUNWAY},
        {"runway_length_m", &s->runway_length_m, RUNWAY},
        {"runway_width_m", &s->runway_width_m, RUNWAY},
        {"position_x_m", &s->position_i_m.x, POSITION_X},
        {"position_y_m", &s->position_i_m.y, POSITION_Y},
        {"position_z_m", &s->position_i_m.z, POSITION_Z},
        {"velocity_x_mps", &s->velocity_i_mps.x, VELOCITY_X},
        {"velocity_y_mps", &s->velocity_i_mps.y, VELOCITY_Y},
        {"velocity_z_mps", &s->velocity_i_mps.z, VELOCITY_Z},
    };
    unsigned seen = 0;
    bool valid = true;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        if (!strchr(line, '\n') && !feof(f)) { valid=false; break; }
        char *p=trim(line);
        if (!*p || *p=='#') continue;
        char *eq=strchr(p, '=');
        if (!eq) { valid=false; break; }
        *eq=0;
        char *key=trim(p), *value=trim(eq+1);
        if (!strcmp(key,"name")) {
            if (!*value || strlen(value)>=sizeof(s->name)) { valid=false; break; }
            memcpy(s->name,value,strlen(value)+1);
            continue;
        }
        char *comment=strchr(value,'#');
        if (comment) { *comment=0; value=trim(value); }
        char *end=NULL;
        errno=0;
        double number=strtod(value,&end);
        if (errno || end==value || *end || !isfinite(number)) { valid=false; break; }
        size_t i=0;
        for (;i<sizeof(fields)/sizeof(fields[0]);++i) {
            if (!strcmp(key,fields[i].name)) {
                *fields[i].value=number;
                seen|=fields[i].flags;
                break;
            }
        }
        if (i==sizeof(fields)/sizeof(fields[0])) { valid=false; break; }
    }
    valid=valid&&!ferror(f);
    fclose(f);
    unsigned cartesian=seen&CARTESIAN;
    if (!valid || (cartesian && cartesian!=CARTESIAN) ||
        (cartesian && (seen&SURFACE)) || s->mass_kg<=0 ||
        s->runway_length_m<=0 || s->runway_width_m<=0 ||
        fabs(s->latitude_deg)>90 || fabs(s->runway_latitude_deg)>90 ||
        fabs(s->flight_path_angle_deg)>90 || s->surface_speed_mps<0 ||
        s->deorbit_duration_s<0 || s->deorbit_delay_s<0 ||
        s->orbital_engine_available_thrust_n<0) return false;
    s->has_cartesian_state=cartesian==CARTESIAN;
    s->has_surface_flight_state=(seen&SURFACE)!=0;
    s->runway_override=(seen&RUNWAY)!=0;
    if (!s->has_cartesian_state && !(seen&HEADING))
        s->heading_deg=initial_bearing_deg(s->latitude_deg,s->longitude_deg,
                                         s->runway_latitude_deg,s->runway_longitude_deg);
    *destination=*s;
    return true;
}
