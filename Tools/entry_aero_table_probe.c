/* Offline force/q probe for exact Mach/AoA comparisons with recorded flight.
 * Build from repo root:
 * cc -O2 -IShuttleSim/include Tools/entry_aero_table_probe.c \
 *   ShuttleSim/src/{aero,math3,world}.c -lm -o /tmp/entry_aero_table_probe
 * Input rows: q(Pa) Mach AoA(deg). Output rows: drag/q lift/q (m^2).
 */
#include "shuttlesim/aero.h"
#include <stdio.h>

int main(int argc,char **argv){
    AeroTable model;
    if(argc!=2){fprintf(stderr,"usage: %s AERO.csv < samples.txt\n",argv[0]);return 2;}
    aero_seed_stsn(&model);
    if(!aero_load_csv(&model,argv[1]))return 2;
    double q,mach,aoa,lift,drag;
    int fields;
    while((fields=scanf("%lf %lf %lf",&q,&mach,&aoa))==3){
        aero_force_per_q(&model,q,mach,aoa,&lift,&drag);
        printf("%.12g %.12g\n",drag,lift);
    }
    return fields==EOF&&!ferror(stdin)?0:2;
}
