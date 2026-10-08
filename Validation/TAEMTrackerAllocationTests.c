#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "landing_api.h"
#include "sim_telemetry.h"
#include "taem_geometry.h"
#include "taem_tracker.h"
#include "shuttlesim/math3.h"

/* Allocation regression only, not STS qualification. The active native model
 * supplies forces; NaN command history bypasses shaping, as in the existing
 * normal-lift balance tests, so this isolates the static force allocator. */
static void saturated_turn_uses_available_lift(const TerminalModel *model) {
    const double rad = acos(-1.0) / 180.0;
    TerminalDynamicState state = {.ut_s=1000.0, .mass_kg=40000.0};
    state.attitude=model->attitude;
    state.attitude.aoa_rad=state.attitude.requested_aoa_rad=NAN;
    state.attitude.bank_rad=state.attitude.requested_bank_rad=NAN;
    state.position_i_m=world_lla_to_inertial(&model->world,
        model->site.latitude*rad,model->site.longitude*rad,26000.0,state.ut_s);
    LocalFrame frame=world_local_frame_i(&model->world,state.position_i_m,state.ut_s);
    Vec3 air=v3_add(v3_scale(frame.east,790.0*cos(-8.0*rad)),
                    v3_scale(frame.up,790.0*sin(-8.0*rad)));
    state.velocity_i_mps=v3_add(air,
        world_atmosphere_velocity_i(&model->world,state.position_i_m));
    TaemGeometryState geometry;
    assert(taem_geometry_state(model,&state,&geometry));
    double radius=v3_norm(state.position_i_m);
    double g_limit=model->vehicle.maximum_g_load*model->world.mu_m3_s2/(radius*radius);
    double bank=fmin(model->vehicle.maximum_bank_angle,80.0)*rad;
    double max_aoa=fmin(model->vehicle.maximum_angle_of_attack,
                       model->aero.alpha_deg[model->aero.alpha_count-1]);
    double maximum_lift=0.0;
    for(double aoa=0.0;aoa<=max_aoa+1e-9;aoa+=0.1){
        AeroForces f=aero_compute(&model->world,&model->aero,state.position_i_m,
            state.velocity_i_mps,state.ut_s,state.mass_kg,aoa*rad,0.0);
        maximum_lift=fmax(maximum_lift,fmin(fabs(f.lift_n)/state.mass_kg,g_limit));
    }
    assert(maximum_lift>0.0);
    const double demand_factors[]={1.25,2.0,4.0};
    for(size_t i=0;i<sizeof(demand_factors)/sizeof(demand_factors[0]);++i){
        double lateral=demand_factors[i]*maximum_lift*sin(bank);
        double reference_error=hypot(maximum_lift*cos(bank),
                                     maximum_lift*sin(bank)-lateral);
        for(int sign=-1;sign<=1;sign+=2){
            TaemPathReference reference={
                .runway_along_m=geometry.runway_along_m,
                .runway_cross_m=geometry.runway_cross_m,
                .course_deg=geometry.course_deg,
                .curvature_right_per_m=sign*lateral/
                    (geometry.ground_speed_mps*geometry.ground_speed_mps),
                .altitude_m=NAN,
                .flight_path_angle_deg=geometry.flight_path_angle_deg,
                .vertical_curvature_per_m=-1.0/1000.0
            };
            TaemTrackerOutput out=taem_tracker_update(model,&state,&geometry,&reference,0.1);
            assert(out.valid&&!out.lateral_authority_ok);
            assert(out.required_vertical_lift_mps2==0.0);
            AeroForces force=aero_compute(&model->world,&model->aero,state.position_i_m,
                state.velocity_i_mps,state.ut_s,state.mass_kg,
                out.control.angle_of_attack_rad,out.control.bank_rad);
            double lift=fmin(fabs(force.lift_n)/state.mass_kg,g_limit);
            double residual=hypot(lift*cos(out.control.bank_rad),
                lift*sin(out.control.bank_rad)-sign*lateral);
            fprintf(stderr,"saturated lift allocation: sign=%+d demand=%.4f aoa=%.2f residual=%.4f available-vector=%.4f\n",
                sign,lateral,out.control.angle_of_attack_rad/rad,residual,reference_error);
            /* Any valid bounded allocation must be no worse in acceleration
             * vector error than this independently evaluated available vector.
             * Tolerance covers the allocator's 0.1-degree numerical search. */
            assert(residual<=reference_error+0.01*maximum_lift);
            assert(fabs(out.control.bank_rad)<=bank+1e-9);
            assert(out.control.angle_of_attack_rad<=max_aoa*rad+1e-9);
        }
    }
}

int main(void){
    LandingConfiguration configuration=landing_configuration_default();
    char atmosphere[512],aero[512],book[512],attitude[512];
    TerminalModelSourceFiles files={
        .atmosphere_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,atmosphere,sizeof(atmosphere)),
        .aero_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_AERO,aero,sizeof(aero)),
        .aero_book_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_FORCE_BOOK,book,sizeof(book)),
        .attitude_ini=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATTITUDE,attitude,sizeof(attitude))
    };
    TerminalModel model;
    char reason[192];
    assert(terminal_model_capture(&model,&files,&configuration,1,reason,sizeof(reason)));
    saturated_turn_uses_available_lift(&model);
    puts("TAEM tracker allocation tests passed.");
    return 0;
}
