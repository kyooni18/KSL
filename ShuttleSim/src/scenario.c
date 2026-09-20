#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "shuttlesim/scenario.h"

void scenario_seed_86km(Scenario *s){
    memset(s,0,sizeof(*s));
    strcpy(s->name,"orbit86km-seed"); s->ut0=0; s->altitude_m=86000; s->latitude_deg=-0.05; s->longitude_deg=105.0;
    s->heading_deg=90.0; s->mass_kg=85000.0; s->initial_aoa_deg=0; s->initial_bank_deg=0;
    s->deorbit_delta_v_mps=60.0; s->deorbit_duration_s=0.0; s->deorbit_delay_s=0.0;
    s->runway_latitude_deg=-0.0486; s->runway_longitude_deg=-74.7240; s->runway_elevation_m=70.0;
    s->runway_heading_deg=90.0; s->runway_length_m=2500.0; s->runway_width_m=70.0;
}
static char *trim(char *s){ while(isspace((unsigned char)*s))s++; char *e=s+strlen(s); while(e>s&&isspace((unsigned char)e[-1]))*--e=0; return s; }
bool scenario_load(const char *path,Scenario *s){
    FILE *f=fopen(path,"r"); if(!f)return false; scenario_seed_86km(s); char line[512];
    while(fgets(line,sizeof(line),f)){
        char *p=trim(line); if(!*p||*p=='#')continue; char *eq=strchr(p,'='); if(!eq)continue; *eq=0;
        char *k=trim(p), *v=trim(eq+1);
        if(!strcmp(k,"name"))snprintf(s->name,sizeof(s->name),"%s",v);
        else if(!strcmp(k,"ut0"))s->ut0=strtod(v,NULL);
        else if(!strcmp(k,"latitude_deg"))s->latitude_deg=strtod(v,NULL);
        else if(!strcmp(k,"longitude_deg"))s->longitude_deg=strtod(v,NULL);
        else if(!strcmp(k,"altitude_m"))s->altitude_m=strtod(v,NULL);
        else if(!strcmp(k,"heading_deg"))s->heading_deg=strtod(v,NULL);
        else if(!strcmp(k,"mass_kg"))s->mass_kg=strtod(v,NULL);
        else if(!strcmp(k,"initial_aoa_deg"))s->initial_aoa_deg=strtod(v,NULL);
        else if(!strcmp(k,"initial_bank_deg"))s->initial_bank_deg=strtod(v,NULL);
        else if(!strcmp(k,"deorbit_delta_v_mps"))s->deorbit_delta_v_mps=strtod(v,NULL);
        else if(!strcmp(k,"deorbit_duration_s"))s->deorbit_duration_s=strtod(v,NULL);
        else if(!strcmp(k,"deorbit_delay_s"))s->deorbit_delay_s=strtod(v,NULL);
        else if(!strcmp(k,"runway_latitude_deg")){s->runway_latitude_deg=strtod(v,NULL);s->runway_override=true;}
        else if(!strcmp(k,"runway_longitude_deg")){s->runway_longitude_deg=strtod(v,NULL);s->runway_override=true;}
        else if(!strcmp(k,"runway_elevation_m")){s->runway_elevation_m=strtod(v,NULL);s->runway_override=true;}
        else if(!strcmp(k,"runway_heading_deg")){s->runway_heading_deg=strtod(v,NULL);s->runway_override=true;}
        else if(!strcmp(k,"runway_length_m")){s->runway_length_m=strtod(v,NULL);s->runway_override=true;}
        else if(!strcmp(k,"runway_width_m")){s->runway_width_m=strtod(v,NULL);s->runway_override=true;}
        else if(!strcmp(k,"position_x_m")){s->position_i_m.x=strtod(v,NULL);s->has_cartesian_state=true;}
        else if(!strcmp(k,"position_y_m")){s->position_i_m.y=strtod(v,NULL);s->has_cartesian_state=true;}
        else if(!strcmp(k,"position_z_m")){s->position_i_m.z=strtod(v,NULL);s->has_cartesian_state=true;}
        else if(!strcmp(k,"velocity_x_mps")){s->velocity_i_mps.x=strtod(v,NULL);s->has_cartesian_state=true;}
        else if(!strcmp(k,"velocity_y_mps")){s->velocity_i_mps.y=strtod(v,NULL);s->has_cartesian_state=true;}
        else if(!strcmp(k,"velocity_z_mps")){s->velocity_i_mps.z=strtod(v,NULL);s->has_cartesian_state=true;}
    }
    fclose(f); return true;
}
