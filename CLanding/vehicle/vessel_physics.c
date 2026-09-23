#include "landing.h"
#include <math.h>
#include <float.h>
#include <string.h>

static bool finite_vector(Vector3 v) { return isfinite(v.x)&&isfinite(v.y)&&isfinite(v.z); }
void vessel_physics_init(VesselPhysicsModel *m) { memset(m,0,sizeof(*m));m->airbrakes=-1; }
static void vessel_physics_reset_transients(VesselPhysicsModel *m){
    if(!m)return;
    /* Rewinds, resource transfers, and scene-load mass corrections break
       finite differences, not already observed airframe aerodynamics.  Keep
       the sample bank while dropping derivative, mass-flow, and actuator
       state.  Structural compatibility is handled by the persisted structure
       identity; wet mass by itself is deliberately not an airframe identity. */
    VesselAeroSample samples[VESSEL_AERO_SAMPLES];
    unsigned count=m->count,next=m->next,accepted=m->accepted,rejected=m->rejected;
    double certified_uncertainty=m->certified_uncertainty,certified_confidence=m->certified_confidence;
    memcpy(samples,m->samples,sizeof(samples));
    vessel_physics_init(m);
    memcpy(m->samples,samples,sizeof(samples));
    m->count=count;m->next=next;m->accepted=accepted;m->rejected=rejected;
    m->certified_uncertainty=certified_uncertainty;m->certified_confidence=certified_confidence;
}
void vessel_physics_import_sample(VesselPhysicsModel *m,const VesselAeroSample *input){
    if(!m||!input||!isfinite(input->q)||input->q<=0.0||!isfinite(input->mach)||
       !isfinite(input->aoa)||!isfinite(input->beta)||!isfinite(input->mass)||input->mass<=0||
       (input->airbrakes!=0&&input->airbrakes!=1)||!finite_vector(input->force_per_q))return;
    VesselAeroSample sample=*input;
    sample.trust=clampd(isfinite(sample.trust)?sample.trust:0,0,1);
    if(sample.trust<=0)return;
    if(sample.observations==0)sample.observations=1;
    if(m->count==0&&m->airbrakes<0){
        /* Before the first live frame the model's configuration is unknown.
           A persisted sample with an explicit configuration may be used as a
           provisional prior; live telemetry overwrites these state bits on
           the first observation. Unknown/legacy (-1) samples are rejected by
           the bridge and therefore cannot establish this provisional state. */
        m->gear=sample.gear;
        m->brakes=sample.brakes;
        m->airbrakes=sample.airbrakes;
    }
    if(m->count<VESSEL_AERO_SAMPLES){
        m->samples[m->count++]=sample;
        m->next=m->count%VESSEL_AERO_SAMPLES;
    }
}
Vector3 vessel_physics_gravity(Vector3 pos,const PlanetModel *p) {
    double r=fmax(vmag(pos),1); return vscale(pos,-p->gravitational_parameter/(r*r*r));
}
void vessel_physics_environment(const PlanetModel *p,const TrajectoryCalibrationModel *cal,double altitude,double *density,double *sound){
    *density=planet_atmospheric_density(p,altitude)*clampd(cal->density_scale,.35,2.5);
    double scale=cal->speed_of_sound_scale;
    if(!isfinite(scale)||scale<=0)scale=1;
    double profile=planet_atmospheric_speed_of_sound(p,altitude);
    *sound=profile>1?profile*clampd(scale,.7,1.3):clampd(cal->speed_of_sound,180,450);
}

double vessel_physics_conservative_stall_fraction(double true_air_speed,double angle_of_attack,
        double dynamic_pressure,const VehicleProfile *v){
    if(!v||!isfinite(true_air_speed)||!isfinite(angle_of_attack)||!isfinite(dynamic_pressure)||
       !isfinite(v->minimum_safe_speed)||v->minimum_safe_speed<=1.0||
       !isfinite(v->maximum_angle_of_attack)||v->maximum_angle_of_attack<=0.0)return 1.0;
    double speed_margin=clampd((v->minimum_safe_speed*1.15-true_air_speed)/
        fmax(10.0,v->minimum_safe_speed*.35),0.0,1.0);
    /* This is a fallback risk proxy, not a measured stall sensor. Its AoA branch
       must not classify guidance's own q-dependent protective Entry reference --
       or ordinary tracking overshoot around that reference -- as a stall.  Preserve
       the legacy .75*max onset once airload is established, but add a bounded thin-
       air tracking margin while the protective envelope is active. */
    double protective=entry_low_q_protective_aoa_floor(dynamic_pressure,v);
    double thin_air=clampd((700.0-fmax(0.0,dynamic_pressure))/350.0,0.0,1.0);
    double tracking_margin=thin_air*.35*fmax(0.0,v->maximum_angle_of_attack-protective);
    double onset=fmax(v->maximum_angle_of_attack*.75,protective+tracking_margin);
    double aoa_margin=clampd((fabs(angle_of_attack)-onset)/
        fmax(1e-6,v->maximum_angle_of_attack-onset),0.0,1.0);
    return fmax(speed_margin,aoa_margin);
}
double vessel_physics_burn_mass(const VesselPhysicsModel *m,double mass,double thrust,double dt){
    if(!m||m->propellant_per_impulse<=0||m->dry_mass<=0||dt<=0||mass<=m->dry_mass)return mass;
    return fmax(m->dry_mass,mass-m->propellant_per_impulse*fmax(0,thrust)*dt);
}
void landing_snapshot_set_sample(LandingSnapshot *s,const Telemetry *t,const VehicleState *state) {
    s->telemetry=*t;
    s->has_vehicle_state=state&&isfinite(t->ut)&&fabs(state->ut-t->ut)<1e-6;
    if(s->has_vehicle_state)s->vehicle_state=*state;
    s->has_attitude_quaternion=t->has_attitude_quaternion;
    memcpy(s->attitude_quaternion,t->attitude_quaternion,sizeof(s->attitude_quaternion));
    snprintf(s->attitude_reference_frame,sizeof(s->attitude_reference_frame),"%s",t->attitude_reference_frame);
}
static void physics_diagnostics(VesselPhysicsModel *m,Telemetry *t){
    Vector3 specific;double confidence=0;
    vessel_physics_aero(m,t->dynamic_pressure,t->mach,t->angle_of_attack,t->sideslip,t->mass,&specific,&confidence);
    m->confidence=confidence;t->physics_confidence=confidence;t->physics_samples=m->count;t->physics_mass_flow=m->mass_flow;
    t->physics_live_samples=m->live_observations;
    t->physics_model_residual=m->model_residual;
    t->physics_model_residual_confidence=m->model_residual_confidence;
    t->physics_certified_uncertainty=m->certified_uncertainty;
    t->physics_force_residual_per_q=m->force_residual_per_q;
    t->physics_force_residual_sigma_per_q=v3(sqrt(fmax(0,m->force_residual_variance_per_q.x)),sqrt(fmax(0,m->force_residual_variance_per_q.y)),sqrt(fmax(0,m->force_residual_variance_per_q.z)));
    t->physics_force_residual_confidence=m->force_residual_confidence;
    /* A binary KSP speedbrake is a distinct aerodynamic configuration.  Do
       not let guidance deploy it merely because clean-airframe coverage
       exists: the predictor must have a prior-flight deployed cell near the
       current q/Mach/incidence before the device can be used automatically. */
    Vector3 airbrake_specific;double airbrake_confidence=0;
    t->physics_airbrake_model_available=t->has_airbrakes&&
        vessel_physics_aero_config(m,t->dynamic_pressure,t->mach,t->angle_of_attack,t->sideslip,t->mass,
            t->gear,t->brakes,1,&airbrake_specific,&airbrake_confidence);
    t->physics_airbrake_model_confidence=t->physics_airbrake_model_available?airbrake_confidence:0;
    t->physics_airbrake_drag_accel=t->physics_airbrake_model_available&&isfinite(airbrake_specific.x)?
        fmax(0.0,airbrake_specific.x):0.0;
    for(int i=0;i<3;i++){t->physics_authority[i]=m->authority[i];t->physics_authority_confidence[i]=m->authority_confidence[i];}
}
void vessel_physics_observe(VesselPhysicsModel *m,Telemetry *t,const VehicleState *s,const PlanetModel *p) {
    if(!isfinite(t->ut)||!isfinite(s->ut)||fabs(s->ut-t->ut)>1e-6||!isfinite(s->mass)||s->mass<=0||!finite_vector(s->position)||!finite_vector(s->velocity))return;
    if(!t->physics_sample_valid){physics_diagnostics(m,t);return;}
    if(m->has_state&&s->ut<m->state.ut)vessel_physics_reset_transients(m);
    double dt=m->has_state?s->ut-m->state.ut:0;
    if(m->has_state&&dt==0){physics_diagnostics(m,t);return;}
    /* A mass discontinuity invalidates derivatives and fuel-flow learning but
       does not prove that the part topology changed.  The old 2% rule erased
       valid history when the startup mass stream corrected from a fallback to
       the real shuttle mass. */
    if(m->has_state&&dt>0&&fabs(s->mass-m->state.mass)>.02*m->state.mass){
        vessel_physics_reset_transients(m);dt=0;
    }
    if(m->has_state&&dt>.02&&dt<2&&t->current_thrust>0&&m->thrust>0){
        double flow=(m->state.mass-s->mass)/dt;
        if(flow>=0&&flow<s->mass*.01){
            m->mass_flow+=.2*(flow-m->mass_flow);
            double consumption=flow/fmax(.5*(m->thrust+t->current_thrust),1);
            m->propellant_per_impulse+=.2*(consumption-m->propellant_per_impulse);
        }
    }
    double torque[]={t->available_pitch_torque,t->available_roll_torque,t->available_yaw_torque};
    double inertia[]={t->pitch_moment_of_inertia,t->roll_moment_of_inertia,t->yaw_moment_of_inertia};
    double rate[]={t->body_pitch_rate,t->body_roll_rate,t->body_yaw_rate};
    double control[]={t->control_pitch,t->control_roll,t->control_yaw};
    bool valid[]={t->has_body_pitch_rate,t->has_body_roll_rate,t->has_body_yaw_rate};
    for(int i=0;i<3;i++){
        m->torque[i]=t->has_torque&&isfinite(torque[i])?torque[i]:0;
        m->inertia[i]=t->has_inertia&&isfinite(inertia[i])?inertia[i]:0;
        if(t->has_torque&&t->has_inertia&&isfinite(torque[i])&&fabs(torque[i])>1e-6&&inertia[i]>0&&isfinite(inertia[i])&&m->authority_confidence[i]<.3){
            m->authority[i]=clampd(fabs(torque[i])/inertia[i]*RAD2DEG,0,10000);
            m->authority_confidence[i]=.25;
        }
        if(valid[i]&&m->previous_rate_valid[i]&&t->has_controls&&dt>.02&&dt<1&&fabs(m->previous_control[i])>.15){
            double measured=(rate[i]-m->previous_rate[i])/dt/m->previous_control[i];
            if(isfinite(measured)&&measured>0&&measured<10000){
                m->authority[i]+=.15*(measured-m->authority[i]);
                m->authority_confidence[i]=fmin(.85,m->authority_confidence[i]+.025);
            }
        }
        m->previous_rate[i]=rate[i];m->previous_control[i]=control[i];
        m->previous_rate_valid[i]=valid[i]&&isfinite(rate[i])&&t->has_controls;
        t->physics_authority[i]=m->authority[i];t->physics_authority_confidence[i]=m->authority_confidence[i];
    }
    m->state=*s;m->has_state=true;m->thrust=fmax(0,t->current_thrust);m->available_thrust=fmax(0,t->available_thrust);m->throttle=t->throttle;
    m->dry_mass=isfinite(t->dry_mass)&&t->dry_mass>0&&t->dry_mass<=s->mass?t->dry_mass:0;
    m->authority_q=t->dynamic_pressure;m->sideslip=t->sideslip;m->gear=t->gear;m->brakes=t->brakes;
    m->airbrakes=t->has_airbrakes?(t->airbrakes?1:0):-1;
    m->has_center_of_mass=t->has_center_of_mass&&finite_vector(t->center_of_mass);
    if(m->has_center_of_mass)m->center_of_mass=t->center_of_mass;
    m->has_center_of_mass_root=t->has_center_of_mass_root&&finite_vector(t->center_of_mass_root);
    if(m->has_center_of_mass_root)m->center_of_mass_root=t->center_of_mass_root;
    m->lift_vector=t->lift_vector;m->drag_vector=t->drag_vector;
    Vector3 air=vsub(s->velocity,vcross(planet_rotation_vector(p),s->position));
    if(t->has_force_vectors&&t->has_airbrakes&&finite_vector(t->lift_vector)&&finite_vector(t->drag_vector)&&t->dynamic_pressure>0.0&&isfinite(t->dynamic_pressure)&&isfinite(t->mach)&&isfinite(t->angle_of_attack)&&isfinite(t->sideslip)&&vmag(air)>DBL_MIN){
        Vector3 dir=vnorm(air,v3(1,0,0)),up=vnorm(vproject_plane(s->position,dir),v3(0,0,1));
        Vector3 side=vnorm(vcross(dir,up),v3(0,1,0));
        double bank=t->roll*DEG2RAD;
        Vector3 lift=vadd(vscale(up,cos(bank)),vscale(side,sin(bank)));
        Vector3 lateral=vsub(vscale(side,cos(bank)),vscale(up,sin(bank)));
        Vector3 f=vadd(t->lift_vector,t->drag_vector);
        VesselAeroSample sample={
            .q=t->dynamic_pressure,.mach=t->mach,.aoa=t->angle_of_attack,.beta=t->sideslip,.mass=s->mass,
            .force_per_q=vscale(v3(-vdot(f,dir),vdot(f,lift),vdot(f,lateral)),1/t->dynamic_pressure),
            .gear=t->gear,.brakes=t->brakes,.airbrakes=t->airbrakes?1:0,.observations=1,
            .trust=1.0,.last_observation_ut=t->ut
        };
        if(sample.force_per_q.x>=0&&finite_vector(sample.force_per_q)){
            /* Current-flight data is a shadow flight-test measurement.  It is
               persisted by the bridge for the NEXT session, but never edits
               the predictor's certified prior during this entry.  Compare it
               against that prior to expose a bounded model-mismatch signal
               for health monitoring and robustness stress instead. */
            m->accepted++;
            bool independent=!m->has_last_residual_ut||t->ut<m->last_residual_ut||t->ut-m->last_residual_ut>=.5;
            if(independent){
                m->live_observations++;
                m->last_residual_ut=t->ut;m->has_last_residual_ut=true;
                Vector3 expected;double certified_confidence=0;
                if(vessel_physics_aero_config(m,t->dynamic_pressure,t->mach,t->angle_of_attack,t->sideslip,s->mass,t->gear,t->brakes,t->airbrakes?1:0,&expected,&certified_confidence)){
                    Vector3 actual=vscale(sample.force_per_q,t->dynamic_pressure/fmax(s->mass,1));
                    double scale=fmax(.05,vmag(actual));
                    double residual=clampd(vmag(vsub(actual,expected))/scale,0,2.0);
                    double alpha=.10*clampd(certified_confidence,.15,1.0);
                    if(m->model_residual_confidence<=0)m->model_residual=residual;
                    else m->model_residual+=alpha*(residual-m->model_residual);
                    m->model_residual_confidence=clampd(m->model_residual_confidence+.035*certified_confidence,0,1);

                    /* Keep the certified prior immutable, but estimate a bounded
                       current-flight correction in the same force/q wind axes.
                       The correction is consumed only by the predictor-side best
                       estimate API; it never edits or imports a certified cell. */
                    Vector3 expected_per_q=vscale(expected,s->mass/fmax(t->dynamic_pressure,1e-9));
                    Vector3 raw=vsub(sample.force_per_q,expected_per_q);
                    double basis=fmax(vmag(expected_per_q),1e-6);
                    Vector3 limits=v3(fmax(.35*fabs(expected_per_q.x),.05*basis),
                        fmax(.35*fabs(expected_per_q.y),.08*basis),.20*basis);
                    Vector3 bounded=v3(clampd(raw.x,-limits.x,limits.x),clampd(raw.y,-limits.y,limits.y),clampd(raw.z,-limits.z,limits.z));
                    Vector3 previous=m->force_residual_per_q;
                    Vector3 innovation=vsub(bounded,previous);
                    double residual_alpha=.06*clampd(certified_confidence,.15,1.0);
                    m->force_residual_per_q=vadd(previous,vscale(innovation,residual_alpha));
                    double variance_alpha=clampd(.08*certified_confidence,.02,.10);
                    Vector3 innovation_sq=v3(innovation.x*innovation.x,innovation.y*innovation.y,innovation.z*innovation.z);
                    m->force_residual_variance_per_q=vadd(vscale(m->force_residual_variance_per_q,1.0-variance_alpha),vscale(innovation_sq,variance_alpha));
                    m->force_residual_confidence=clampd(m->force_residual_confidence+.025*certified_confidence,0,.95);
                }
            }
        }else m->rejected++;
    }else m->rejected++;
    physics_diagnostics(m,t);
}
bool vessel_physics_aero_config(
        const VesselPhysicsModel *m,
        double q,
        double mach,
        double aoa,
        double beta,
        double mass,
        bool gear,
        bool brakes,
        int airbrakes,
        Vector3 *specific,
        double *confidence) {
    if (!specific || !confidence) return false;
    *specific = v3(0, 0, 0);
    *confidence = 0.0;

    if (!m || (airbrakes != 0 && airbrakes != 1) ||
        !(q > 0.0) || !(mass > 0.0) ||
        !isfinite(q) || !isfinite(mach) ||
        !isfinite(aoa) || !isfinite(beta) || !isfinite(mass))
        return false;

    double estimate_weight = 0.0;
    double locality_weight = 0.0;
    double confidence_weight = 0.0;

    for (unsigned i = 0; i < m->count; ++i) {
        const VesselAeroSample *sample = &m->samples[i];
        if (sample->gear != gear ||
            sample->brakes != brakes ||
            sample->airbrakes != airbrakes ||
            !(sample->q > 0.0))
            continue;

        /*
         * Interpolation resolution, not a flight gate:
         *   q: one octave (factor two)
         *   Mach: 0.5
         *   incidence coordinates: 10 deg
         * These are the current certified data-book cell scales and are shared
         * with archive coverage reduction. At the support boundary the compact
         * kernel reaches zero continuously rather than abruptly switching a
         * confidence heuristic.
         * decision-literal-ok: interpolation/discretization resolution.
         */
        double dq = log(q / sample->q) / log(2.0);
        double dm = (mach - sample->mach) / 0.5;
        double da = (aoa - sample->aoa) / 10.0;
        double db = (beta - sample->beta) / 10.0;
        double radius_squared =
            dq * dq + dm * dm + da * da + db * db;
        if (!(radius_squared < 1.0)) continue;

        double trust =
            clampd(isfinite(sample->trust) ? sample->trust : 0.0, 0.0, 1.0);
        if (!(trust > 0.0)) continue;

        double radius = sqrt(fmax(0.0, radius_squared));
        double compact = 1.0 - radius;
        double kernel = compact * compact;
        double independent =
            (double)(sample->observations ? sample->observations : 1U);

        /*
         * Independent observations are precision weight for the interpolated
         * mean. They do not pass through a saturation curve and therefore
         * cannot manufacture confidence simply by repeating the same cell.
         */
        double base_weight = kernel * independent;
        double weight = base_weight * trust;
        *specific =
            vadd(*specific, vscale(sample->force_per_q, weight));
        estimate_weight += weight;
        locality_weight += base_weight;

        /*
         * Confidence is directly the reliability-weighted remaining distance
         * to the interpolation support boundary. An exact certified sample has
         * confidence=trust; confidence approaches zero continuously at the
         * boundary. Sample multiplicity changes the estimate precision but not
         * the meaning of locality confidence.
         */
        confidence_weight += base_weight * trust * compact;
    }

    if (!(estimate_weight > 0.0) || !(locality_weight > 0.0))
        return false;

    *specific =
        vscale(*specific, q / (mass * estimate_weight));
    *confidence =
        clampd(confidence_weight / locality_weight, 0.0, 1.0);
    return true;
}
bool vessel_physics_aero(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,Vector3 *specific,double *confidence){
    if(!m||m->airbrakes<0){if(specific)*specific=v3(0,0,0);if(confidence)*confidence=0;return false;}
    return vessel_physics_aero_config(m,q,mach,aoa,beta,mass,m->gear,m->brakes,m->airbrakes,specific,confidence);
}
Vector3 vessel_physics_force_config(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,bool gear,bool brakes,int airbrakes,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed){
    Vector3 specific;
    *observed=false;*confidence=0;
    if(!isfinite(q)||q<0||!isfinite(mach)||!isfinite(aoa)||!isfinite(mass)||mass<=0)return v3(0,0,0);
    double stress_drag=cal&&cal->stress_drag_scale>0?clampd(cal->stress_drag_scale,.5,1.5):1;
    double stress_lift=cal&&cal->stress_lift_scale>0?clampd(cal->stress_lift_scale,.5,1.5):1;
    *observed=vessel_physics_aero_config(m,q,mach,aoa,beta,mass,gear,brakes,airbrakes,&specific,confidence);
    if(*observed)return v3(specific.x*stress_drag,specific.y*stress_lift,specific.z*stress_lift);
    double lf,df;aerodynamic_force_factors_mach(mach,aoa,v,&lf,&df);
    double drag_scale=cal?clampd(cal->drag_scale,.3,3):1;
    double lift_scale=cal?clampd(cal->lift_scale,.3,3):1;
    double base=q/fmax(20,fallback.ballistic_coefficient)*drag_scale*stress_drag;
    *confidence=.05;
    return v3(base*df,base*fmax(0,fallback.lift_to_drag)*lift_scale*stress_lift*lf,0);
}

Vector3 vessel_physics_force_best_estimate_config(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,bool gear,bool brakes,int airbrakes,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed,double *relative_uncertainty){
    Vector3 prior=vessel_physics_force_config(m,q,mach,aoa,beta,mass,gear,brakes,airbrakes,fallback,cal,v,confidence,observed);
    double certified=m&&m->certified_uncertainty>0&&isfinite(m->certified_uncertainty)?clampd(m->certified_uncertainty,.06,.90):.45;
    if(relative_uncertainty)*relative_uncertainty=(*observed)?certified:fmax(certified,.45);
    if(!m||!(*observed)||q<=0||mass<=0||m->force_residual_confidence<=.02)return prior;

    double gain=clampd((m->force_residual_confidence-.02)/.55,0,.78);
    Vector3 correction=vscale(m->force_residual_per_q,q/fmax(mass,1.0)*gain);
    double basis=fmax(vmag(prior),.02);
    Vector3 limits=v3(fmax(.30*fabs(prior.x),.04*basis),fmax(.30*fabs(prior.y),.06*basis),.20*basis);
    correction.x=clampd(correction.x,-limits.x,limits.x);
    correction.y=clampd(correction.y,-limits.y,limits.y);
    correction.z=clampd(correction.z,-limits.z,limits.z);
    Vector3 best=vadd(prior,correction);best.x=fmax(0,best.x);

    if(relative_uncertainty){
        Vector3 sigma_per_q=v3(sqrt(fmax(0,m->force_residual_variance_per_q.x)),sqrt(fmax(0,m->force_residual_variance_per_q.y)),sqrt(fmax(0,m->force_residual_variance_per_q.z)));
        double sigma_rel=vmag(vscale(sigma_per_q,q/fmax(mass,1.0)))/fmax(vmag(best),.05);
        double prior_term=certified*(1.0-.25*gain);
        *relative_uncertainty=clampd(hypot(prior_term,clampd(sigma_rel,0,.80)*gain),.06,.90);
    }
    return best;
}
Vector3 vessel_physics_force(const VesselPhysicsModel *m,double q,double mach,double aoa,double beta,double mass,AerodynamicModel fallback,const TrajectoryCalibrationModel *cal,const VehicleProfile *v,double *confidence,bool *observed){
    bool gear=m?m->gear:false,brakes=m?m->brakes:false;int airbrakes=m&&m->airbrakes>=0?m->airbrakes:0;
    return vessel_physics_force_config(m,q,mach,aoa,beta,mass,gear,brakes,airbrakes,fallback,cal,v,confidence,observed);
}

Vector3 vessel_physics_acceleration(Vector3 position,Vector3 air,const PlanetModel *p,Vector3 force,double bank){
    Vector3 dir=vnorm(air,v3(0,0,0)),up=vnorm(vproject_plane(position,dir),v3(0,0,1));
    Vector3 side=vnorm(vcross(dir,up),v3(0,1,0));
    double b=bank*DEG2RAD;
    Vector3 lift=vadd(vscale(up,cos(b)),vscale(side,sin(b)));
    Vector3 lateral=vsub(vscale(side,cos(b)),vscale(up,sin(b)));
    return vadd(vessel_physics_gravity(position,p),vadd(vscale(dir,-force.x),vadd(vscale(lift,force.y),vscale(lateral,force.z))));
}
void vessel_physics_axis_step(const VesselPhysicsModel *m,int axis,double target,double q,double limit,double dt,double *angle,double *rate){
    if(!m||axis<0||axis>2||dt<=0||!isfinite(dt)||!isfinite(q)||!isfinite(target)||!isfinite(limit)||!isfinite(*angle)||!isfinite(*rate))return;
    double physical_authority=m->authority[axis];
    /* Do not extrapolate aerodynamic authority upward with q. */
    if(m->authority_q>1&&q<m->authority_q)physical_authority*=clampd(q/m->authority_q,0,1);

    /* The predictor used to treat available torque/inertia as an ideal servo:
       at high q a 1000+ deg/s^2 reported surface could jump to the requested
       bank rate almost instantaneously.  The real kRPC controller is a sampled
       closed loop with a deliberately bounded rate bandwidth.  Model that
       achieved bandwidth here instead of actuator ceiling so planning cannot
       assume bank captures the live controller cannot reproduce. */
    double rate_limit=fabs(limit);
    double closed_loop_authority=fmin(fmax(physical_authority,0.0),
        fmax(.5,rate_limit/.75*2.0));
    if(closed_loop_authority<=0)return;
    unsigned steps=(unsigned)ceil(fmin(dt,120)/.05);
    double h=dt/fmax(steps,1);
    for(unsigned i=0;i<steps;i++){
        double error=norm_signed_deg(target-*angle);
        double desired=copysign(fmin(rate_limit,sqrt(2*closed_loop_authority*fabs(error))),error);
        double previous=*rate;
        *rate+=clampd(desired-*rate,-closed_loop_authority*h,closed_loop_authority*h);
        *angle=norm_signed_deg(*angle+(*rate+previous)*.5*h);
    }
}
