#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "landing_api.h"
#include "sim_telemetry.h"
#include "taem_spiral.h"
#include "terminal_solver.h"
#include "shuttlesim/math3.h"

static const double rad=.017453292519943295;

static void mathematics(void) {
    double sweeps[]={.4,1.5,4.7,9.5};
    for(int side=-1;side<=1;side+=2) for(int j=0;j<4;++j) {
        double sweep=sweeps[j],rf=4000,a=2000/pow(sweep,4),length=0;
        TaemSpiralSample prev,q;
        assert(taem_spiral_sample(rf,a,0,side,0,0,0,&prev));
        assert(fabs(prev.curvature-side/rf)<1e-14);
        assert(fabs(prev.heading_rad)<1e-14);
        for(int i=1;i<=10000;++i) {
            double p=sweep*i/10000;
            assert(taem_spiral_sample(rf,a,p,side,0,0,0,&q));
            assert(q.radius_m>=prev.radius_m);
            length+=hypot(q.x-prev.x,q.y-prev.y);
            if(i>1&&i<10000) {
                double eps=1e-5;
                TaemSpiralSample l,h,mirror;
                assert(taem_spiral_sample(rf,a,p-eps,side,0,0,0,&l));
                assert(taem_spiral_sample(rf,a,p+eps,side,0,0,0,&h));
                double dx=(l.x-h.x)/(2*eps),dy=(l.y-h.y)/(2*eps);
                assert(fabs(remainder(atan2(dy,dx)-q.heading_rad,2*acos(-1)))<1e-7);
                double curvature=-remainder(h.heading_rad-l.heading_rad,2*acos(-1))/(2*eps*q.ds_dpsi);
                assert(fabs(curvature-q.curvature)<1e-9);
                assert(fabs(-(h.curvature-l.curvature)/(2*eps*q.ds_dpsi)-q.curvature_rate)<1e-10);
                assert(fabs(-(h.curvature_rate-l.curvature_rate)/(2*eps*q.ds_dpsi)-q.curvature_acceleration)<1e-11);
                assert(taem_spiral_sample(rf,a,p,-side,0,0,0,&mirror));
                assert(fabs(mirror.x-q.x)<1e-9&&fabs(mirror.y+q.y)<1e-9);
            }
            prev=q;
        }
        assert(fabs(length/taem_spiral_arc(rf,a,sweep)-1)<1e-7);
    }
}

static TerminalDynamicState state_at(const TerminalModel *m,const TaemGeometryState *geo) {
    TaemFrameWorld w;TaemRunwayFrame f;TaemVec3 pb;
    assert(taem_geometry_runway(m,&w,&f));
    assert(taem_runway_unproject(&w,&f,geo->runway_along_m,geo->runway_cross_m,geo->altitude_above_runway_m,&pb));
    TerminalDynamicState s={.mass_kg=25000,.ut_s=1000,.attitude=m->attitude};
    s.position_i_m=world_fixed_to_inertial(&m->world,ss_v3(pb.x,pb.y,pb.z),s.ut_s);
    LocalFrame l=world_local_frame_i(&m->world,s.position_i_m,s.ut_s);
    double heading=geo->course_deg*rad,gamma=geo->flight_path_angle_deg*rad,v=geo->airspeed_mps;
    Vec3 horizontal=v3_add(v3_scale(l.north,cos(heading)),v3_scale(l.east,sin(heading)));
    s.velocity_i_mps=v3_add(world_atmosphere_velocity_i(&m->world,s.position_i_m),
        v3_add(v3_scale(horizontal,v*cos(gamma)),v3_scale(l.up,v*sin(gamma))));
    s.attitude.aoa_rad=10*rad;
    return s;
}

int main(int argc,char **argv) {
    mathematics();
    LandingConfiguration cfg=landing_configuration_default();
    char atmosphere[512],aero[512],book[512],attitude[512],reason[256];
    TerminalModelSourceFiles files={
        .atmosphere_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATMOSPHERE,atmosphere,sizeof atmosphere),
        .aero_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_AERO,aero,sizeof aero),
        .aero_book_csv=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_FORCE_BOOK,book,sizeof book),
        .attitude_ini=shuttle_sim_model_path(NULL,SHUTTLE_SIM_MODEL_ATTITUDE,attitude,sizeof attitude)};
    TerminalModel m;assert(terminal_model_capture(&m,&files,&cfg,1,reason,sizeof reason));
    m.guidance.final_alignment_speed=160;
    m.guidance.final_approach_distance=8000;m.guidance.final_glide_slope=20;
    for(int side=-1;side<=1;side+=2) {
        TaemGeometryState start={.runway_along_m=-30000,.runway_cross_m=side*15000,
            .ground_speed_mps=280,.airspeed_mps=280,.course_deg=m.site.runway_heading,
            .flight_path_angle_deg=-15,.altitude_above_runway_m=18000};
        TaemRoute r;
        assert(taem_route_build_hac(&m,&start,12000,side,4.7,200,.001,&r,reason,sizeof reason));
        /* Seed a measured, tangent capture 3 km upstream of the true join. */
        double h=(r.hac.capture_course_deg-m.site.runway_heading)*rad;
        start.runway_along_m=r.hac.entry.x-3000*cos(h);
        start.runway_cross_m=r.hac.entry.y-3000*sin(h);start.course_deg=r.hac.capture_course_deg;
        assert(taem_route_build_hac(&m,&start,12000,side,4.7,200,.001,&r,reason,sizeof reason));
        double joints[]={r.lead_length_m,r.lead_length_m+r.hac.arc_length_m,
            r.lead_length_m+r.hac.arc_length_m+r.unload_length_m};
        for(int i=0;i<3;++i) {
            TaemRoutePoint a,b;
            assert(taem_route_sample(&r,joints[i]-.01,&a)&&taem_route_sample(&r,joints[i]+.01,&b));
            assert(hypot(a.along_m-b.along_m,a.cross_m-b.cross_m)<.03);
            assert(fabs(remainder(a.course_deg-b.course_deg,360))<.01);
            assert(fabs(a.curvature_right_per_m-b.curvature_right_per_m)<1e-7);
        }
        TaemRoutePoint end;assert(taem_route_sample(&r,r.length_m,&end));
        assert(fabs(end.along_m+8000)<1e-6&&fabs(end.cross_m)<1e-6);
        assert(fabs(end.curvature_right_per_m)<1e-12);
        assert(fabs(end.course_deg-m.site.runway_heading)<1e-9);
        printf("geometry side=%d Ri=%.1f Rf=%.1f sweep=%.3f length=%.1f lead=%.1f spiral=%.1f unload=%.1f\n",
            side,r.hac.radius_m,r.spiral_exit_radius_m,fabs(r.hac.arc_sweep_rad),r.length_m,r.lead_length_m,r.hac.arc_length_m,r.unload_length_m);
        if(argc>1) {
            (void)argv;
            for(int altitude=14000;altitude<=19500;altitude+=500) {
                start.altitude_above_runway_m=altitude;
                TerminalDynamicState s=state_at(&m,&start);
                TaemRoute trial=r;trial.profile_start_altitude_m=m.site.altitude+altitude;
                TerminalProfileResult profile=terminal_solver_generate_profile(&m,&s,&trial,.5,600);
                printf("native side=%d h=%d profile=%d exitV=%.1f reason=%s\n",side,altitude,profile.valid,profile.exit_speed_mps,profile.reason);
                if(profile.valid) {
                    TerminalSolverResult replay=terminal_solver_replay(&m,&s,&trial,.5,600);
                    printf("replay status=%d Final=%d crossMax=%.1f courseMax=%.1f exitV=%.1f bank=%.1f reason=%s\n",
                        replay.status,replay.final_interface_qualified,replay.maximum_cross_track_m,replay.maximum_course_error_deg,
                        replay.final_geometry.airspeed_mps,replay.final_state.attitude.bank_rad/rad,replay.reason);
                }
            }
        }
    }
    puts("Spiral mathematics and geometric continuity passed (not a flight qualification).");
}
