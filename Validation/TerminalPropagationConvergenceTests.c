#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "landing_api.h"
#include "sim_telemetry.h"
#include "terminal_propagator.h"
#include "shuttlesim/flight_physics.h"
#include "shuttlesim/math3.h"

/* Compare a held planner command against the actual 50 Hz plant kernel.
 * This is numerical parity, not a claim about STS aerodynamic validation. */
static bool test_held_command_parity(const TerminalModel *model, double outer_dt) {
    const double rad=acos(-1.0)/180.0;
    TerminalDynamicState state={.ut_s=1000.0,.mass_kg=40000.0};
    state.attitude=model->attitude;
    state.attitude.aoa_rad=state.attitude.cmd_aoa_rad=
        state.attitude.requested_aoa_rad=18.0*rad;
    state.attitude.bank_rad=state.attitude.cmd_bank_rad=
        state.attitude.requested_bank_rad=0.0;
    state.position_i_m=world_lla_to_inertial(&model->world,
        model->site.latitude*rad,model->site.longitude*rad,26000.0,state.ut_s);
    LocalFrame frame=world_local_frame_i(&model->world,state.position_i_m,state.ut_s);
    Vec3 air=v3_add(v3_scale(frame.east,790.0*cos(-8.0*rad)),
                    v3_scale(frame.up,790.0*sin(-8.0*rad)));
    state.velocity_i_mps=v3_add(air,
        world_atmosphere_velocity_i(&model->world,state.position_i_m));
    FlightAirborneState plant={.position_i_m=state.position_i_m,
        .velocity_i_mps=state.velocity_i_mps,.attitude=state.attitude,
        .body_q_i={1.0,0.0,0.0,0.0}};
    TerminalDynamicState bounded_state=state;
    TerminalControl oversized={.dt_s=1.0e9,
        .angle_of_attack_rad=18.0*rad,.bank_rad=0.0};
    assert(terminal_propagator_step(model,&bounded_state,&oversized)==
           TERMINAL_STEP_INVALID_INPUT);
    assert(bounded_state.ut_s==state.ut_s &&
           v3_norm(v3_sub(bounded_state.position_i_m,state.position_i_m))==0.0 &&
           v3_norm(v3_sub(bounded_state.velocity_i_mps,state.velocity_i_mps))==0.0);

    double max_position_error=0.0,max_speed_error=0.0,max_aoa_error=0.0;
    int steps=(int)lround(30.0/outer_dt);
    int substeps=(int)lround(outer_dt/0.02);
    for(int step=0;step<steps;++step){
        double elapsed=step*outer_dt;
        TerminalControl command={.dt_s=outer_dt,
            .angle_of_attack_rad=(elapsed<15.0?28.0:18.0)*rad,
            .bank_rad=(elapsed<15.0?70.0:-70.0)*rad};
        double start_ut=state.ut_s;
        attitude_set_command(&plant.attitude,command.angle_of_attack_rad,command.bank_rad);
        for(int j=0;j<substeps;++j){
            FlightAirborneStepInput input={.mass_kg=state.mass_kg,
                .ut=start_ut+j*0.02,.dt_s=0.02,
                .burn_accel_mps2=0.0,.engine_thrust_n=0.0};
            flight_physics_airborne_step(&model->world,&model->aero,&input,&plant);
        }
        assert(terminal_propagator_step(model,&state,&command)==TERMINAL_STEP_OK);
        max_position_error=fmax(max_position_error,
            v3_norm(v3_sub(state.position_i_m,plant.position_i_m)));
        max_speed_error=fmax(max_speed_error,
            v3_norm(v3_sub(state.velocity_i_mps,plant.velocity_i_mps)));
        max_aoa_error=fmax(max_aoa_error,
            fabs(state.attitude.aoa_rad-plant.attitude.aoa_rad)/rad);
    }
    fprintf(stderr,"held-command propagation dt=%.2f: max position=%.6f m velocity=%.6f m/s AoA=%.6f deg\n",
        outer_dt,max_position_error,max_speed_error,max_aoa_error);
    /* Same kernel, inputs and cadence must agree to floating-point precision. */
    return max_position_error<1e-4 && max_speed_error<1e-6 && max_aoa_error<1e-7;
}

int main(void){
    LandingConfiguration config=landing_configuration_default();
    char atmosphere[512],aero[512],book[512],attitude[512];
    TerminalModelSourceFiles files={
        .atmosphere_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,atmosphere,sizeof(atmosphere)),
        .aero_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_AERO,aero,sizeof(aero)),
        .aero_book_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_FORCE_BOOK,book,sizeof(book)),
        .attitude_ini=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATTITUDE,attitude,sizeof(attitude))
    };
    TerminalModel model;
    char reason[192];
    assert(terminal_model_capture(&model,&files,&config,1,reason,sizeof(reason)));
    bool fine=test_held_command_parity(&model,0.02);
    bool medium=test_held_command_parity(&model,0.1);
    bool coarse=test_held_command_parity(&model,0.5);
    assert(fine&&medium&&coarse);
    puts("Terminal propagation cadence parity tests passed.");
    return 0;
}
