#include "../CLanding/sim_telemetry.h"
#include "../CLanding/decision_envelope.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    PlanetModel p={0};
    char error[256];
    assert(shuttle_sim_load_planet("../ShuttleSim/data/fitted/kerbin_atmosphere_ksp.csv",
        &p,error,sizeof(error)));
    assert(p.atmosphere_sample_count>2 && p.atmosphere_depth==70000.0);
    assert(p.atmosphere_altitude[p.atmosphere_sample_count-1]>p.atmosphere_depth);
    for(size_t i=0;i<p.atmosphere_sample_count;i++) {
        double density=planet_atmospheric_density(&p,p.atmosphere_altitude[i]);
        assert(fabs(density-p.atmosphere_density[i])<=1e-12);
    }
    LandingConfiguration cfg=landing_configuration_default();
    TaemSpeedEnvelope speed=decision_taem_speed_envelope(&cfg.vehicle,&p,
        cfg.guidance.taem_interface_altitude);
    assert(speed.valid && speed.feasible);
    PlanetModel missing=p;
    missing.atmosphere_sample_count=0;
    assert(!decision_taem_speed_envelope(&cfg.vehicle,&missing,
        cfg.guidance.taem_interface_altitude).valid);

    PlanetModel saved=p;
    assert(!shuttle_sim_load_planet("build/no-such-atmosphere.csv",&p,error,sizeof(error)));
    assert(memcmp(&p,&saved,sizeof(p))==0);
    const char *path="build/invalid-atmosphere-test.csv";
    const char *cases[]={
        "altitude_m,density_kg_m3,pressure_pa,temperature_k,speed_of_sound_mps\n",
        "0,1,100000,280,330\n0,0,0,0,0\n",
        "0,1,100000,280,330\n100,NaN,0,0,0\n",
        "0,1,100000,280,330\n100,0,0,0,0\n200,1,1000,280,330\n",
        "0,1,100000,280,330\n100,0,0,0,0,extra\n",
        "0,1,100000,280,330\n100,0.5,50000,280,330\n"
    };
    for(size_t i=0;i<sizeof(cases)/sizeof(cases[0]);i++) {
        FILE *f=fopen(path,"w");assert(f);fputs(cases[i],f);fclose(f);
        assert(!shuttle_sim_load_planet(path,&p,error,sizeof(error)));
        assert(memcmp(&p,&saved,sizeof(p))==0);
    }
    remove(path);
    printf("sim_planet_model_tests: PASS (%zu samples, TAEM speed %.1f..%.1f m/s)\n",
        p.atmosphere_sample_count,speed.minimum_speed_mps,speed.maximum_speed_mps);
    return 0;
}
