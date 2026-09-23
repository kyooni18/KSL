#include <math.h>
#include <string.h>
#include "shuttlesim/attitude.h"
#include "shuttlesim/math3.h"
#include "shuttlesim/quat.h"

void attitude_seed(AttitudeModel *a){
    memset(a,0,sizeof(*a));
    a->pitch_wn=1.4; a->pitch_zeta=0.9; a->roll_wn=1.8; a->roll_zeta=0.85;
    a->max_pitch_rate_rad_s=deg2rad(8.0); a->max_roll_rate_rad_s=deg2rad(18.0);
    a->max_pitch_accel_rad_s2=deg2rad(5.0); a->max_roll_accel_rad_s2=deg2rad(15.0);
}
void attitude_set_command(AttitudeModel *a,double aoa,double bank){
    a->requested_aoa_rad=aoa;
    a->requested_bank_rad=wrap_pi(bank);
}
static void __attribute__((unused)) command_slew(double requested,double *command,double vmax,double dt,bool wrap){
    if(!command||!isfinite(requested)||!isfinite(dt)||!(dt>0.0)||
       !isfinite(vmax)||!(vmax>0.0))return;
    double error=requested-*command;
    if(wrap)error=wrap_pi(error);
    *command+=clampd(error,-vmax*dt,vmax*dt);
    if(wrap)*command=wrap_pi(*command);
}
static void __attribute__((unused)) axis_step(double target,double *x,double *v,double vmax,double dt,bool wrap){
    if (!isfinite(dt) || !(dt > 0.0) || !isfinite(vmax) || vmax < 0.0) {
        *v = 0.0;
        return;
    }
    double err = target - *x;
    if (wrap) err = wrap_pi(err);
    double rate = fabs(vmax);
    double step = clampd(err, -rate * dt, rate * dt);
    *x += step;
    if (wrap) *x = wrap_pi(*x);
    *v = step / dt;
    /* The target is followed exactly as soon as the bounded move reaches it.
       There is no second-order servo, acceleration lag, or overshoot model in
       the simulator; only the configured vehicle-followable rate remains. */
    if (fabs(err) <= rate * dt) {
        *x = target;
        *v = 0.0;
    }
}
void attitude_step(AttitudeModel *a,double dt){
    /* ShuttleSim is a guidance-trajectory validator, not an FCS/actuator
       simulator.  The guidance backend already rate-limits the commanded
       attitude with stabilized(); the sim should therefore treat that command
       as perfectly followed so path errors come from trajectory law, energy,
       and aerodynamics rather than an extra hidden attitude lag model. */
    double previous_aoa=a->aoa_rad;
    double previous_bank=a->bank_rad;
    a->cmd_aoa_rad=a->requested_aoa_rad;
    a->cmd_bank_rad=wrap_pi(a->requested_bank_rad);
    a->aoa_rad=a->cmd_aoa_rad;
    a->bank_rad=a->cmd_bank_rad;
    if(isfinite(dt)&&dt>0.0){
        a->aoa_rate_rad_s=(a->aoa_rad-previous_aoa)/dt;
        a->bank_rate_rad_s=wrap_pi(a->bank_rad-previous_bank)/dt;
    }else{
        a->aoa_rate_rad_s=0.0;
        a->bank_rate_rad_s=0.0;
    }
}
Quat attitude_body_quat(Vec3 p,Vec3 vair,double aoa,double bank){
    Vec3 flight=v3_normalized(vair), radial=v3_normalized(p);
    Vec3 right=v3_normalized(v3_cross(flight,radial)); if(v3_norm(right)<1e-8)right=v3(0,1,0);
    Vec3 normal=v3_normalized(v3_cross(right,flight));
    Vec3 body_fwd=v3_normalized(v3_add(v3_scale(flight,cos(aoa)),v3_scale(normal,sin(aoa))));
    Vec3 body_up=v3_normalized(v3_add(v3_scale(normal,cos(aoa)),v3_scale(flight,-sin(aoa))));
    body_up=v3_rotate_axis(body_up,body_fwd,bank);
    Vec3 body_right=v3_normalized(v3_cross(body_fwd,body_up));
    body_up=v3_normalized(v3_cross(body_right,body_fwd));
    Vec3 body_down=v3_scale(body_up,-1.0);
    /* Aerospace right-handed body frame: +X forward, +Y right, +Z down. */
    return quat_from_basis(body_fwd,body_right,body_down);
}

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

static char *att_trim(char *s){
    while(isspace((unsigned char)*s)) s++;
    char *e=s; while(*e) e++;
    while(e>s && isspace((unsigned char)e[-1])) *--e=0;
    return s;
}
bool attitude_load_ini(AttitudeModel *a,const char *path){
    FILE *f=fopen(path,"r"); if(!f) return false;
    char line[256];
    while(fgets(line,sizeof(line),f)){
        char *p=att_trim(line); if(!*p||*p=='#') continue;
        char *eq=strchr(p,'='); if(!eq) continue; *eq=0;
        char *k=att_trim(p),*v=att_trim(eq+1); double x=strtod(v,NULL);
        if(!strcmp(k,"pitch_wn"))a->pitch_wn=x;
        else if(!strcmp(k,"pitch_zeta"))a->pitch_zeta=x;
        else if(!strcmp(k,"roll_wn"))a->roll_wn=x;
        else if(!strcmp(k,"roll_zeta"))a->roll_zeta=x;
        else if(!strcmp(k,"max_pitch_rate_deg_s"))a->max_pitch_rate_rad_s=deg2rad(x);
        else if(!strcmp(k,"max_roll_rate_deg_s"))a->max_roll_rate_rad_s=deg2rad(x);
        else if(!strcmp(k,"max_pitch_accel_deg_s2"))a->max_pitch_accel_rad_s2=deg2rad(x);
        else if(!strcmp(k,"max_roll_accel_deg_s2"))a->max_roll_accel_rad_s2=deg2rad(x);
    }
    fclose(f);return true;
}
