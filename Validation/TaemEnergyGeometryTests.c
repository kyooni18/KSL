/* White-box, offline MM305 mode-selection regression.
 * A polar inlet that is abeam the runway is not a Final approach just
 * because the along-track station lies behind its current x coordinate.
 */
#include "../CLanding/guidance/guidance_internal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../CLanding/include/sim_telemetry.h"
#include "../CLanding/guidance/taem/energy_law.inc"

/* The 14:24 KST run reached this inlet before the operator's engine
   firings.  Maximum drag modulation must not be selected while the actual
   high-speed capture turn already leaves the station beyond glide range. */
static void recorded_capture_range(void) {
    LandingConfiguration cfg=landing_configuration_default();
    cfg.vehicle.maximum_bank_angle=70.0;
    cfg.guidance.final_approach_distance=9000.0;
    cfg.guidance.final_glide_slope=20.0;
    cfg.guidance.final_alignment_speed=160.0;
    char attitude[512],reason[192];
    TerminalModelSourceFiles files={
        .atmosphere_csv="ShuttleSim/reference-model/kerbin_atmosphere_identified.csv",
        .aero_csv="ShuttleSim/reference-model/sts_aero_live20261010.csv",
        .attitude_ini=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATTITUDE,
            attitude,sizeof(attitude))
    };
    TerminalModel *model=calloc(1,sizeof(*model));
    assert(model&&terminal_model_capture(model,&files,&cfg,1,reason,sizeof(reason)));
    TelContext c={.model=model,.d_station=9000.0,.v_station=160.0,.aoa_max=15.0};
    c.h_station=c.d_station*tan(20.0*DEG2RAD);
    c.e_station=c.h_station+160.0*160.0/(2.0*TEL_G0);
    TelPoint inlet={.x=-38498.0,.y=41727.0,.h=25789.0,.v=864.0,
        .g=-6.788758*DEG2RAD,.psi=28.984301*DEG2RAD,.m=24073.0,
        .aoa=17.83,.bank=37.3};
    TaemEnergyLawState route={.initialized=true,.loop=true,.capture_forecast=true,.leg=TEL_LEG_DOWN,
        .side=-1,.radius=4207.0};
    TelSegment segs[5];
    double path=tel_sum(segs,tel_segments(&route,&c,&inlet,segs));
    double predicted;
    assert(tel_solve_k(&route,&c,&inlet,&predicted)==0.0);
    assert(predicted<path-20000.0);
    /* Once abeam, a wide capture must point back toward the actual station,
       rather than the always-forward centreline lookahead. */
    route.loop=false;route.leg=TEL_LEG_OUT;
    TelPoint abeam=inlet;
    abeam.x=10000.0;abeam.y=40000.0;abeam.v=160.0;
    abeam.psi=-100.0*DEG2RAD;
    assert(tel_lateral(&route,&c,&abeam)>0.0);
    assert(route.leg==TEL_LEG_OUT);
    /* 2026-10-14 live MM305 handoff: past the RW09 station abeam at 42 km
       crossrange, Mach 2.85, flying away from the runway line. The straight-in
       path must charge the turn back, not price the chord as |y| (37.7 km). */
    TaemEnergyLawState straight={.initialized=true,.leg=TEL_LEG_OUT,.loop=false,
        .x_turn=-9000.0,.radius=5000.0};
    TelPoint passed={.x=9014.0,.y=37660.0,.h=21043.0,.v=592.0,
        .g=-13.8*DEG2RAD,.psi=-19.5*DEG2RAD,.m=25670.0,.aoa=14.8,.bank=44.0};
    double passed_path=tel_sum(segs,tel_segments(&straight,&c,&passed,segs));
    assert(passed_path>hypot(passed.x+c.d_station,passed.y)+20000.0);
    model->guidance.sts_trajectory_delivery=true;
    snprintf(model->vehicle.model_id,sizeof(model->vehicle.model_id),"STS");
    tel_plan(&straight,&c,&passed);
    assert(straight.loop); /* price and fly a capture route, not a pursuit/Dubins mix */
    TaemEnergyLawState final_line={.initialized=true,.loop=true,
        .capture_forecast=true,.leg=TEL_LEG_FINAL,.radius=2200.0};
    TelPoint low={.x=-9500.0,.y=0.0,.h=1680.0,.v=91.5,
        .g=-19.3*DEG2RAD,.m=25660.0,.aoa=4.8};
    double final_path=tel_sum(segs,tel_segments(&final_line,&c,&low,segs));
    assert(tel_predict_pattern(&final_line,&c,&low,0.0,NULL)<final_path);
    c.slope_deg=20.0;
    low.x=-5000.0;
    TelContext priced=tel_effective_station(&final_line,&c,&low,0.3,low.v);
    assert(priced.d_station==5000.0); /* loop exits price their actual capture too */
    /* Live runway switch: near the centreline but pointing across it. The
       frame scorer priced a HAC, while the old first-tick replan gate left
       live guidance on an unplanned direct pursuit with the opposite bank. */
    TaemGeometryState switched={.runway_along_m=33767.0,.runway_cross_m=-5006.0,
        .altitude_above_runway_m=23459.0,.airspeed_mps=768.7,
        .ground_speed_mps=754.6,.flight_path_angle_deg=-11.0,
        .heading_error_deg=105.8};
    TaemEnergyLawState fresh={0};double aoa,bank;
    assert(taem_energy_law_step(&fresh,model,&switched,25660.0,13.4,26015.1,
        44.8,&aoa,&bank));
    assert(fresh.capture_forecast&&fresh.loop);
    (void)taem_energy_law_frame_score;
    free(model);
}

int main(void){
    /* Whole-include white-box test: the full static law is linked for ABI
       fidelity, although this fixture exercises its pure geometry gates. */
    (void)taem_energy_law_step;
    TelContext c={.d_station=8000.0};
    TaemEnergyLawState law={.initialized=true,.leg=TEL_LEG_OUT,
        .loop=false,.x_turn=-8000.0,.radius=5000.0};
    TelPoint polar={.x=973.0,.y=67957.0,.psi=95.0*DEG2RAD};
    assert(tel_plan_open(&law,&c,&polar));
    assert(!tel_final_line_ready(&c,&polar));
    polar.x=-1552.0;polar.y=38725.0;polar.psi=94.0*DEG2RAD;
    assert(tel_plan_open(&law,&c,&polar));
    assert(!tel_final_line_ready(&c,&polar));
    /* Even an in-range abeam approach does not shortcut the bounded turn. */
    polar.x=-10000.0;polar.y=3000.0;polar.psi=5.0*DEG2RAD;
    assert(!tel_final_line_ready(&c,&polar));
    polar.y=1000.0;polar.psi=30.0*DEG2RAD;
    assert(!tel_final_line_ready(&c,&polar));
    polar.psi=5.0*DEG2RAD;
    assert(tel_final_line_ready(&c,&polar));
    polar.x=973.0;
    assert(tel_final_line_ready(&c,&polar));
    polar.y=NAN;
    assert(!tel_final_line_ready(&c,&polar));
    recorded_capture_range();
    puts("MM305 off-axis and Final-mode geometry tests passed.");
    return 0;
}
