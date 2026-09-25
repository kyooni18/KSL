#include <stdio.h>
#include "shuttlesim/world.h"
#include "shuttlesim/aero.h"
#include "shuttlesim/attitude.h"
int main(int argc,char**argv){
  const char*dir=argv[1];char p[512];
  KerbinWorld w;world_seed_kerbin(&w);
  snprintf(p,sizeof p,"%s/kerbin_atmosphere_seed.csv",dir);FILE*f=fopen(p,"w");
  fprintf(f,"# model=kerbin_stock_spatial (dumped from world_seed_kerbin nominal table)\naltitude_m,density_kg_m3,pressure_pa,temperature_k,speed_of_sound_mps\n");
  for(size_t i=0;i<w.atmosphere.count;i++){AtmospherePoint*a=&w.atmosphere.p[i];
    fprintf(f,"%.1f,%.12g,%.9g,%.6f,%.6f\n",a->altitude_m,a->sample.density_kg_m3,a->sample.pressure_pa,a->sample.temperature_k,a->sample.pressure_pa>0?a->sample.speed_of_sound_mps:0.0);}
  fclose(f);
  AeroTable t;aero_seed_stsn(&t);
  snprintf(p,sizeof p,"%s/stsn_aero_seed.csv",dir);f=fopen(p,"w");fprintf(f,"mach,alpha_deg,cl,cd\n");
  for(size_t i=0;i<t.mach_count;i++)for(size_t j=0;j<t.alpha_count;j++)fprintf(f,"%.6g,%.6g,%.9g,%.9g\n",t.mach[i],t.alpha_deg[j],t.cl[i][j],t.cd[i][j]);
  fclose(f);
  AttitudeModel a;attitude_seed(&a);
  snprintf(p,sizeof p,"%s/stsn_attitude_seed.ini",dir);f=fopen(p,"w");
  fprintf(f,"pitch_wn=%g\npitch_zeta=%g\nroll_wn=%g\nroll_zeta=%g\nmax_pitch_rate_deg_s=8\nmax_roll_rate_deg_s=18\nmax_pitch_accel_deg_s2=5\nmax_roll_accel_deg_s2=15\npitch_full_authority_q_pa=%g\nroll_full_authority_q_pa=%g\n",a.pitch_wn,a.pitch_zeta,a.roll_wn,a.roll_zeta,a.pitch_full_authority_q_pa,a.roll_full_authority_q_pa);
  fclose(f);return 0;}
