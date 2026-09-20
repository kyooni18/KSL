#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "shuttlesim/replay.h"

bool replay_load_csv(AttitudeReplay *r,const char *path){
    memset(r,0,sizeof(*r));
    FILE *f=fopen(path,"r"); if(!f) return false;
    r->points=calloc(REPLAY_MAX,sizeof(ReplayPoint)); if(!r->points){fclose(f);return false;}
    char line[512];
    while(fgets(line,sizeof(line),f)&&r->count<REPLAY_MAX){
        if(line[0]=='#'||strstr(line,"time_s")) continue;
        ReplayPoint p={0}; int gear=0;
        int n=sscanf(line,"%lf,%lf,%lf,%d",&p.time_s,&p.aoa_deg,&p.bank_deg,&gear);
        if(n>=3){p.gear_down=n>=4?(gear!=0):false;r->points[r->count++]=p;}
    }
    fclose(f);
    if(r->count<1){replay_free(r);return false;}
    return true;
}
void replay_free(AttitudeReplay *r){free(r->points);memset(r,0,sizeof(*r));}
bool replay_sample(AttitudeReplay *r,double t,ReplayPoint *out){
    if(!r->points||r->count==0)return false;
    while(r->cursor+1<r->count && r->points[r->cursor+1].time_s<=t)r->cursor++;
    if(r->cursor+1>=r->count){*out=r->points[r->count-1];return true;}
    ReplayPoint a=r->points[r->cursor],b=r->points[r->cursor+1];
    if(t<=a.time_s){*out=a;return true;}
    double u=(t-a.time_s)/(b.time_s-a.time_s);if(u<0)u=0;if(u>1)u=1;
    out->time_s=t;out->aoa_deg=a.aoa_deg+(b.aoa_deg-a.aoa_deg)*u;
    double db=b.bank_deg-a.bank_deg;while(db>180)db-=360;while(db<-180)db+=360;
    out->bank_deg=a.bank_deg+db*u;out->gear_down=a.gear_down;return true;
}
