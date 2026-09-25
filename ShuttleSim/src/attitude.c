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
    a->pitch_full_authority_q_pa=15.0; a->roll_full_authority_q_pa=10.0;
    a->direct_min_q_pa=500.0;
    a->direct_pitch_accel_per_kpa=deg2rad(4.0);
    a->direct_pitch_accel_max=deg2rad(40.0);
    a->direct_pitch_stiffness_per_kpa=0.12;
    a->direct_trim_aoa_rad=deg2rad(6.0);
    a->direct_pitch_damping_ratio=0.25;
    a->direct_roll_accel_per_kpa=deg2rad(6.0);
    a->direct_roll_accel_max=deg2rad(60.0);
    a->direct_roll_damping_s_inv=0.8;
    a->direct_yaw_accel_per_kpa=deg2rad(3.0);
    a->direct_yaw_accel_max=deg2rad(30.0);
    a->direct_yaw_stiffness_per_kpa=0.10;
}
void attitude_set_inputs(AttitudeModel *a,double pitch,double roll,double yaw){
    if(!a)return;
    a->direct=true;
    a->input_pitch=isfinite(pitch)?ss_clampd(pitch,-1.0,1.0):0.0;
    a->input_roll=isfinite(roll)?ss_clampd(roll,-1.0,1.0):0.0;
    a->input_yaw=isfinite(yaw)?ss_clampd(yaw,-1.0,1.0):0.0;
}
void attitude_set_command(AttitudeModel *a,double aoa,double bank){
    a->requested_aoa_rad=aoa;
    a->requested_bank_rad=wrap_pi(bank);
}
static void axis_step(double target,double *x,double *v,double wn,double zeta,
        double vmax,double amax,double authority,double dt,bool wrap){
    if(!x||!v||!isfinite(target)||!isfinite(*x)||!isfinite(*v)||
       !isfinite(wn)||!(wn>0.0)||!isfinite(zeta)||zeta<0.0||
       !isfinite(vmax)||!(vmax>0.0)||!isfinite(amax)||!(amax>0.0)||
       !isfinite(authority)||authority<0.0||authority>1.0||
       !isfinite(dt)||!(dt>0.0)){
        if(v)*v=0.0;
        return;
    }
    double err=target-*x;
    if(wrap)err=wrap_pi(err);

    /* KSP target-vs-actual telemetry identifies a damped second-order closed-loop
       response.  The physical axis is also bounded by independently measured
       rate and angular-acceleration envelopes, so reversals consume real time
       instead of teleporting the lift vector. */
    if(authority<=1e-12){
        *v=0.0;
        return;
    }
    /* Aerodynamic control moment is proportional to q.  The identified
       high-q closed-loop response is therefore reduced below the measured
       full-authority pressure: acceleration scales linearly, while response
       frequency and attainable rate scale with sqrt(authority). */
    double root=sqrt(authority);
    double effective_wn=wn*root;
    double effective_vmax=vmax*root;
    double effective_amax=amax*authority;
    double accel=effective_wn*effective_wn*err-2.0*zeta*effective_wn*(*v);
    accel=ss_clampd(accel,-effective_amax,effective_amax);
    *v=ss_clampd(*v+accel*dt,-effective_vmax,effective_vmax);
    *x+=(*v)*dt;
    if(wrap)*x=wrap_pi(*x);
}
/* Surface-moment rotational dynamics in the controlled coordinates
   (AoA, bank about the air-relative velocity, sideslip). */
static void direct_step(AttitudeModel *a,double q,double dt){
    double kpa=q/1000.0;
    double pitch_ctrl=fmin(a->direct_pitch_accel_max,a->direct_pitch_accel_per_kpa*kpa);
    double stiffness=a->direct_pitch_stiffness_per_kpa*kpa;
    double pitch_damp=2.0*a->direct_pitch_damping_ratio*sqrt(fmax(stiffness,0.0));
    double aoa_acc=pitch_ctrl*a->input_pitch-stiffness*(a->aoa_rad-a->direct_trim_aoa_rad)-
        pitch_damp*a->aoa_rate_rad_s;
    a->aoa_rate_rad_s+=aoa_acc*dt;
    a->aoa_rad+=a->aoa_rate_rad_s*dt;

    double roll_ctrl=fmin(a->direct_roll_accel_max,a->direct_roll_accel_per_kpa*kpa);
    double roll_damp=a->direct_roll_damping_s_inv*sqrt(fmax(kpa,0.0)/10.0);
    double bank_acc=roll_ctrl*a->input_roll-roll_damp*a->bank_rate_rad_s;
    a->bank_rate_rad_s+=bank_acc*dt;
    a->bank_rad=wrap_pi(a->bank_rad+a->bank_rate_rad_s*dt);

    double yaw_ctrl=fmin(a->direct_yaw_accel_max,a->direct_yaw_accel_per_kpa*kpa);
    double weathercock=a->direct_yaw_stiffness_per_kpa*kpa;
    double beta_acc=-yaw_ctrl*a->input_yaw-weathercock*a->sideslip_rad-
        2.0*0.3*sqrt(fmax(weathercock,0.0))*a->sideslip_rate_rad_s;
    a->sideslip_rate_rad_s+=beta_acc*dt;
    a->sideslip_rad=ss_clampd(a->sideslip_rad+a->sideslip_rate_rad_s*dt,-0.5,0.5);

    /* Telemetry consumers read cmd_* as "what the plant is being asked to do";
       in direct mode that is the present state. */
    a->cmd_aoa_rad=a->aoa_rad;
    a->cmd_bank_rad=a->bank_rad;
}

void attitude_step(AttitudeModel *a,double dynamic_pressure_pa,double dt){
    if(!a)return;
    if(a->direct&&isfinite(dynamic_pressure_pa)&&dynamic_pressure_pa>=a->direct_min_q_pa&&
       isfinite(dt)&&dt>0.0){
        direct_step(a,dynamic_pressure_pa,dt);
        return;
    }
    a->cmd_aoa_rad=a->requested_aoa_rad;
    a->cmd_bank_rad=wrap_pi(a->requested_bank_rad);
    double q=fmax(0.0,isfinite(dynamic_pressure_pa)?dynamic_pressure_pa:0.0);
    double pitch_authority=ss_clampd(q/fmax(a->pitch_full_authority_q_pa,1e-9),0.0,1.0);
    double roll_authority=ss_clampd(q/fmax(a->roll_full_authority_q_pa,1e-9),0.0,1.0);
    axis_step(a->cmd_aoa_rad,&a->aoa_rad,&a->aoa_rate_rad_s,
              a->pitch_wn,a->pitch_zeta,a->max_pitch_rate_rad_s,
              a->max_pitch_accel_rad_s2,pitch_authority,dt,false);
    axis_step(a->cmd_bank_rad,&a->bank_rad,&a->bank_rate_rad_s,
              a->roll_wn,a->roll_zeta,a->max_roll_rate_rad_s,
              a->max_roll_accel_rad_s2,roll_authority,dt,true);
}
Quat attitude_body_quat(Vec3 p,Vec3 vair,double aoa,double bank){
    Vec3 flight=v3_normalized(vair), radial=v3_normalized(p);
    Vec3 right=v3_normalized(v3_cross(flight,radial)); if(v3_norm(right)<1e-8)right=ss_v3(0,1,0);
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
        else if(!strcmp(k,"pitch_full_authority_q_pa"))a->pitch_full_authority_q_pa=x;
        else if(!strcmp(k,"roll_full_authority_q_pa"))a->roll_full_authority_q_pa=x;
        else if(!strcmp(k,"direct_min_q_pa"))a->direct_min_q_pa=x;
        else if(!strcmp(k,"direct_pitch_accel_per_kpa_deg"))a->direct_pitch_accel_per_kpa=deg2rad(x);
        else if(!strcmp(k,"direct_pitch_accel_max_deg"))a->direct_pitch_accel_max=deg2rad(x);
        else if(!strcmp(k,"direct_pitch_stiffness_per_kpa"))a->direct_pitch_stiffness_per_kpa=x;
        else if(!strcmp(k,"direct_trim_aoa_deg"))a->direct_trim_aoa_rad=deg2rad(x);
        else if(!strcmp(k,"direct_pitch_damping_ratio"))a->direct_pitch_damping_ratio=x;
        else if(!strcmp(k,"direct_roll_accel_per_kpa_deg"))a->direct_roll_accel_per_kpa=deg2rad(x);
        else if(!strcmp(k,"direct_roll_accel_max_deg"))a->direct_roll_accel_max=deg2rad(x);
        else if(!strcmp(k,"direct_roll_damping_s_inv"))a->direct_roll_damping_s_inv=x;
    }
    fclose(f);return true;
}
