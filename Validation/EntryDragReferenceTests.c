#include "entry_drag_reference.h"
#include "entry_alpha.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static PlanetModel kerbin(void){
    PlanetModel p;
    memset(&p,0,sizeof(p));
    snprintf(p.name,sizeof(p.name),"Kerbin");
    p.radius=600000.0;
    p.gravitational_parameter=3.5316e12;
    p.rotational_speed=2.0*LANDER_PI/21549.425;
    p.atmosphere_depth=70000.0;
    p.surface_density=1.225;
    p.atmosphere_adiabatic_index=1.4;
    /* Explicit stock-Kerbin density anchors for these offline reference tests.
       Production receives the body atmosphere table from kRPC; the math layer no
       longer invents a generic atmosphere when samples are absent. */
    static const double h[]={0.0,30000.0,40000.0,50000.0,60000.0,65000.0,67500.0,70000.0};
    static const double rho[]={1.139922941,.005714044845,.001000742403,.0002217244997,
        .00003817946362,.00001147843415,.000003268873144,0.0};
    p.atmosphere_sample_count=sizeof(h)/sizeof(h[0]);
    for(size_t n=0;n<p.atmosphere_sample_count;n++){
        p.atmosphere_altitude[n]=h[n];
        p.atmosphere_density[n]=rho[n];
        p.atmosphere_pressure[n]=0.0;
    }
    p.north_axis=v3(0,0,1);
    p.prime_meridian_at_epoch=v3(1,0,0);
    return p;
}

static EntryDragReferenceInput fixture(PlanetModel*p,LandingConfiguration*cfg){
    EntryDragReferenceInput i;
    memset(&i,0,sizeof(i));
    i.phase=ENTRY_PHASE_CONSTANT_DRAG;
    i.relative_velocity=1500.0;
    i.latitude=-.05;
    i.altitude=30000.0;
    i.measured_drag_accel=4.0;
    i.modeled_drag_accel=4.2;
    i.aero_confidence=.85;
    i.range_to_site=360000.0;
    i.taem_range=cfg->guidance.taem_interface_range;
    i.taem_latitude=cfg->site.latitude;
    i.taem_altitude=cfg->guidance.taem_interface_altitude;
    i.taem_velocity=690.0;
    i.planet=p;
    i.vehicle=&cfg->vehicle;
    return i;
}

static void set_range_for_average_drag(EntryDragReferenceInput*i,double drag){
    double energy=entry_remaining_specific_energy(i->latitude,i->altitude,i->relative_velocity,
        i->taem_latitude,i->taem_altitude,i->taem_velocity,i->planet);
    assert(isfinite(energy)&&energy>0&&drag>0);
    i->range_to_site=i->taem_range+energy/drag;
}

static double velocity_at_progress(const EntryDragReferenceInput*i,
        const EntryDragReferenceConfig*c,double progress){
    double entry=i->taem_velocity*c->entry_velocity_ratio;
    return entry-progress*(entry-i->taem_velocity);
}

static void assert_finite_output(EntryDragReferenceOutput r){
    assert(isfinite(r.confidence));
    assert(isfinite(r.entry_anchor_velocity));
    assert(isfinite(r.profile_progress));
    assert(isfinite(r.range_authority));
    assert(isfinite(r.required_average_drag_accel));
    assert(isfinite(r.equilibrium_drag_accel));
    assert(isfinite(r.constant_drag_accel));
    assert(isfinite(r.taem_drag_accel));
    assert(isfinite(r.nominal_drag_accel));
    assert(isfinite(r.reference_drag_accel));
    assert(isfinite(r.observed_drag_accel));
    assert(isfinite(r.drag_error_accel));
    assert(isfinite(r.drag_error_fraction));
    assert(isfinite(r.available_range_to_taem));
    assert(isfinite(r.unshaped_predicted_range));
    assert(isfinite(r.predicted_range_to_taem));
    assert(isfinite(r.unshaped_range_error));
    assert(isfinite(r.predicted_range_error));
    assert(isfinite(r.range_error_scale));
    assert(isfinite(r.drag_range_scale));
    assert(isfinite(r.reference_drag_dv));
    assert(isfinite(r.predicted_range_d_drag));
    assert(isfinite(r.required_vertical_lift_accel));
    assert(isfinite(r.minimum_achievable_drag_accel));
}

static void test_default_config(void){
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    assert(entry_drag_reference_config_valid(&c));
    assert(c.temperature_end_progress<c.equilibrium_end_progress);
    assert(c.equilibrium_end_progress<c.constant_end_progress);
}

static void test_piecewise_velocity_continuity(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();c.range_gain=0.0;
    EntryDragReferenceInput base=fixture(&p,&cfg);base.phase=ENTRY_PHASE_EQUILIBRIUM_GLIDE;
    const double boundaries[]={c.temperature_end_progress,c.equilibrium_end_progress,c.constant_end_progress};
    for(unsigned n=0;n<sizeof(boundaries)/sizeof(boundaries[0]);n++){
        double boundary=velocity_at_progress(&base,&c,boundaries[n]);
        EntryDragReferenceInput hi=base,lo=base;
        hi.relative_velocity=boundary+.05;lo.relative_velocity=boundary-.05;
        set_range_for_average_drag(&hi,3.0);set_range_for_average_drag(&lo,3.0);
        EntryDragReferenceOutput a=entry_drag_reference_compute(&hi,&c);
        EntryDragReferenceOutput b=entry_drag_reference_compute(&lo,&c);
        assert(a.valid&&b.valid);
        double scale=fmax(1.0,.5*(fabs(a.reference_drag_accel)+fabs(b.reference_drag_accel)));
        assert(fabs(a.reference_drag_accel-b.reference_drag_accel)/scale<.002);
    }
}

static void test_short_range_demands_more_drag(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput short_case=fixture(&p,&cfg),long_case=short_case;
    short_case.range_to_site=155000.0;long_case.range_to_site=520000.0;
    EntryDragReferenceOutput short_r=entry_drag_reference_compute(&short_case,&c);
    EntryDragReferenceOutput long_r=entry_drag_reference_compute(&long_case,&c);
    assert(short_r.valid&&long_r.valid);
    assert(short_r.required_average_drag_accel>long_r.required_average_drag_accel);
    assert(short_r.reference_drag_accel>long_r.reference_drag_accel);
}

static void test_constant_drag_segment_converges(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();c.range_gain=0.0;
    EntryDragReferenceInput a=fixture(&p,&cfg),b=a;
    a.phase=b.phase=ENTRY_PHASE_CONSTANT_DRAG;
    a.relative_velocity=velocity_at_progress(&a,&c,.66);
    b.relative_velocity=velocity_at_progress(&b,&c,.75);
    set_range_for_average_drag(&a,3.1);set_range_for_average_drag(&b,3.1);
    EntryDragReferenceOutput ra=entry_drag_reference_compute(&a,&c);
    EntryDragReferenceOutput rb=entry_drag_reference_compute(&b,&c);
    assert(ra.valid&&rb.valid);
    assert(fabs(ra.constant_drag_accel-rb.constant_drag_accel)<1e-9);
    assert(fabs(ra.nominal_drag_accel-ra.constant_drag_accel)<1e-9);
    assert(fabs(rb.nominal_drag_accel-rb.constant_drag_accel)<1e-9);
}

static void test_transition_tends_to_taem_drag(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();c.range_gain=0.0;
    EntryDragReferenceInput start=fixture(&p,&cfg),late=start;
    start.phase=late.phase=ENTRY_PHASE_TRANSITION;
    start.relative_velocity=velocity_at_progress(&start,&c,c.constant_end_progress);
    late.relative_velocity=velocity_at_progress(&late,&c,.98);
    set_range_for_average_drag(&start,3.0);set_range_for_average_drag(&late,3.0);
    EntryDragReferenceOutput a=entry_drag_reference_compute(&start,&c);
    EntryDragReferenceOutput b=entry_drag_reference_compute(&late,&c);
    assert(a.valid&&b.valid);
    assert(fabs(b.nominal_drag_accel-b.taem_drag_accel)<
        fabs(a.nominal_drag_accel-a.taem_drag_accel));
}

static void test_weak_aero_confidence_stays_finite(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput i=fixture(&p,&cfg);
    i.measured_drag_accel=NAN;i.modeled_drag_accel=NAN;i.aero_confidence=NAN;
    EntryDragReferenceOutput r=entry_drag_reference_compute(&i,&c);
    assert(r.valid&&r.degraded);assert_finite_output(r);
    assert(fabs(r.drag_error_accel)<1e-12);
    assert(r.confidence>0&&r.confidence<.5);
}

static void test_range_prediction_is_monotonic_and_physical(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();c.range_gain=0.0;
    EntryDragReferenceInput low=fixture(&p,&cfg),high=low;
    low.relative_velocity=1150.0;high.relative_velocity=1900.0;
    set_range_for_average_drag(&low,3.0);set_range_for_average_drag(&high,3.0);
    EntryDragReferenceOutput a=entry_drag_reference_compute(&low,&c);
    EntryDragReferenceOutput b=entry_drag_reference_compute(&high,&c);
    assert(a.valid&&b.valid);
    assert(a.unshaped_predicted_range>=0&&b.unshaped_predicted_range>=0);
    assert(b.unshaped_predicted_range>a.unshaped_predicted_range);
    assert(a.predicted_range_d_drag<=0&&b.predicted_range_d_drag<=0);
}

static void test_range_feedback_reduces_miss(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceInput i=fixture(&p,&cfg);i.range_to_site=250000.0;
    EntryDragReferenceConfig open=entry_drag_reference_default_config();open.range_gain=0.0;
    EntryDragReferenceConfig closed=entry_drag_reference_default_config();
    EntryDragReferenceOutput a=entry_drag_reference_compute(&i,&open);
    EntryDragReferenceOutput b=entry_drag_reference_compute(&i,&closed);
    assert(a.valid&&b.valid);
    assert(fabs(b.predicted_range_error)<=fabs(a.predicted_range_error)+1e-6);
    if(a.unshaped_range_error>0)assert(b.drag_range_scale>=1.0);
    if(a.unshaped_range_error<0)assert(b.drag_range_scale<=1.0);
}

static void test_vertical_lift_demand_tracks_drag_error(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput low_drag=fixture(&p,&cfg),high_drag=low_drag;
    low_drag.measured_drag_accel=.2;high_drag.measured_drag_accel=20.0;
    EntryDragReferenceOutput low=entry_drag_reference_compute(&low_drag,&c);
    EntryDragReferenceOutput high=entry_drag_reference_compute(&high_drag,&c);
    assert(low.valid&&high.valid);
    assert(low.drag_error_accel>high.drag_error_accel);
    assert(low.required_vertical_lift_accel<high.required_vertical_lift_accel);
}

static void test_exec_profile_gates_are_ordered(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput i=fixture(&p,&cfg);i.phase=ENTRY_PHASE_TEMPERATURE_CONTROL;
    i.relative_velocity=2000.0;
    EntryDragReferenceOutput r=entry_drag_reference_compute(&i,&c);
    EntryExecProfile profile=entry_drag_reference_exec_profile(&i,&c,&r);
    assert(profile.has_temperature_velocity_gate&&profile.equilibrium_intercept_valid);
    assert(profile.has_constant_drag_velocity_gate);
    assert(profile.temperature_end_velocity>profile.constant_drag_end_velocity);
    assert(profile.temperature_end_velocity>i.taem_velocity);
    assert(profile.constant_drag_end_velocity>i.taem_velocity);
    assert(profile.has_phase_floor&&profile.phase_floor==ENTRY_PHASE_TEMPERATURE_CONTROL);

    i.phase=ENTRY_PHASE_CONSTANT_DRAG;i.relative_velocity=1200.0;
    r=entry_drag_reference_compute(&i,&c);
    profile=entry_drag_reference_exec_profile(&i,&c,&r);
    assert(profile.phase_floor==ENTRY_PHASE_CONSTANT_DRAG);
    assert(profile.equilibrium_intercept);
}

static void test_target_speed_above_taem_still_has_range_budget(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();c.range_gain=0.0;
    EntryDragReferenceInput i=fixture(&p,&cfg);
    i.phase=ENTRY_PHASE_TRANSITION;i.relative_velocity=i.taem_velocity;i.altitude=26000.0;
    double energy=entry_remaining_specific_energy(i.latitude,i.altitude,i.relative_velocity,
        i.taem_latitude,i.taem_altitude,i.taem_velocity,i.planet);
    assert(energy>0);i.range_to_site=i.taem_range+energy/5.0;
    EntryDragReferenceOutput r=entry_drag_reference_compute(&i,&c);
    assert(r.valid);assert(r.unshaped_predicted_range>0);
    assert(r.taem_drag_accel>0);
}

static void test_exec_profile_does_not_encode_taem_ownership(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput i=fixture(&p,&cfg);
    i.phase=ENTRY_PHASE_TRANSITION;
    i.relative_velocity=i.taem_velocity;
    i.altitude=i.taem_altitude;
    i.latitude=i.taem_latitude;

    i.range_to_site=i.taem_range+400000.0;
    EntryDragReferenceOutput far=entry_drag_reference_compute(&i,&c);
    EntryExecProfile far_profile=entry_drag_reference_exec_profile(&i,&c,&far);
    assert(far.valid);

    i.range_to_site=i.taem_range;
    EntryDragReferenceOutput near=entry_drag_reference_compute(&i,&c);
    EntryExecProfile near_profile=entry_drag_reference_exec_profile(&i,&c,&near);
    assert(near.valid);

    /* TAEM reachability belongs to the live handoff contract. Changing only the
       predicted range margin must not mutate the longitudinal executive profile. */
    assert(memcmp(&far_profile,&near_profile,sizeof(far_profile))==0);
}

static void test_recorded_1003_local_drag_authority_bounds_reference(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig bounded=entry_drag_reference_default_config();
    EntryDragReferenceConfig structural=bounded;structural.minimum_local_drag_confidence=1.0;
    EntryDragReferenceInput i=fixture(&p,&cfg);
    /* 2026-09-11T10-03-34Z near 35.94 km: the old reference sat at its
       structural 23.42 m/s2 ceiling while the vessel achieved 3.04 m/s2. */
    i.phase=ENTRY_PHASE_TEMPERATURE_CONTROL;
    i.relative_velocity=1803.92834472656;i.altitude=35942.5688262481;
    i.measured_drag_accel=3.044307714730498;i.modeled_drag_accel=i.measured_drag_accel;
    i.aero_confidence=.90;i.range_to_site=76243.7566930268;
    i.taem_range=cfg.guidance.taem_interface_range;i.taem_altitude=cfg.guidance.taem_interface_altitude;
    i.taem_velocity=cfg.guidance.taem_force_handoff_speed;
    EntryDragReferenceOutput local=entry_drag_reference_compute(&i,&bounded);
    EntryDragReferenceOutput legacy=entry_drag_reference_compute(&i,&structural);
    assert(local.valid&&legacy.valid);
    assert(local.reference_drag_accel<=i.measured_drag_accel*bounded.local_drag_authority_multiplier+1e-6);
    assert(local.reference_drag_accel<legacy.reference_drag_accel);
    assert(local.predicted_range_to_taem>legacy.predicted_range_to_taem);
}

static void test_recorded_v26_thermal_floor_rejects_fictitious_low_drag(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput i=fixture(&p,&cfg);
    /* heading-offset-v26 around UT 67500: h 30.6 km, V 1531 m/s, q about
       5.94 kPa, measured D/m 9.86 m/s2, actual AoA 22.16 deg. MM304's
       low-single-digit drag request was below what Entry could fly without
       violating the 18 deg thermal-protection floor. */
    i.phase=ENTRY_PHASE_CONSTANT_DRAG;
    i.relative_velocity=1530.52;i.altitude=30617.0;
    i.measured_drag_accel=9.8635;i.modeled_drag_accel=9.8635;i.aero_confidence=.95;
    i.range_to_site=141500.0;i.taem_range=0.0;
    i.taem_altitude=26500.0;i.taem_velocity=1247.2;
    EntryDragReferenceInput unconstrained=i;
    i.has_incidence=true;i.angle_of_attack=22.16;i.sideslip=0.0;
    EntryDragReferenceOutput bounded=entry_drag_reference_compute(&i,&c);
    EntryDragReferenceOutput legacy=entry_drag_reference_compute(&unconstrained,&c);
    assert(bounded.valid&&legacy.valid);
    assert(bounded.minimum_achievable_drag_accel>7.0);
    assert(bounded.drag_floor_active);
    assert(fabs(bounded.reference_drag_accel-bounded.minimum_achievable_drag_accel)<1e-9);
    assert(bounded.reference_drag_accel>legacy.reference_drag_accel+2.0);
    assert(bounded.predicted_range_to_taem<legacy.predicted_range_to_taem-20000.0);
    assert(bounded.predicted_range_error<legacy.predicted_range_error-20000.0);
    assert(isfinite(bounded.required_vertical_lift_accel));
}

static void test_recorded_v29_reference_never_requests_drag_below_executable_alpha_floor(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput i=fixture(&p,&cfg);
    /* heading-offset-v29 live continuation near UT 67459: the fixed inlet still
       had ~205 km to go while measured drag exceeded Dref.  The production
       alpha law cannot remove that mismatch below its non-emergency tracking floor. */
    i.phase=ENTRY_PHASE_EQUILIBRIUM_GLIDE;
    i.relative_velocity=1808.6;i.altitude=37025.0;
    i.measured_drag_accel=4.97;i.modeled_drag_accel=4.97;i.aero_confidence=.90;
    i.range_to_site=hypot(204786.0,28008.0);i.taem_range=0.0;
    i.taem_altitude=26500.0;i.taem_velocity=1247.2;
    i.has_incidence=true;i.angle_of_attack=24.47;i.sideslip=0.0;

    EntryAlphaSchedule alpha;
    entry_alpha_schedule_default(&alpha,&cfg.vehicle,i.taem_velocity);
    double q=.5*planet_atmospheric_density(&p,i.altitude)*i.relative_velocity*i.relative_velocity;
    double minimum_alpha=entry_alpha_minimum_drag_tracking_aoa(&alpha,
        ENTRY_ALPHA_EQUILIBRIUM_GLIDE,i.relative_velocity,q,cfg.vehicle.maximum_dynamic_pressure,
        i.aero_confidence);
    EntryDragReferenceOutput r=entry_drag_reference_compute(&i,&c);

    assert(r.valid&&isfinite(minimum_alpha));
    assert(minimum_alpha>21.5);
    assert(r.drag_floor_active);
    assert(r.reference_drag_accel>=r.minimum_achievable_drag_accel-1e-9);
}

static void test_positive_low_q_uses_trusted_local_drag_evidence(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput i=fixture(&p,&cfg);
    i.phase=ENTRY_PHASE_TRANSITION;
    i.altitude=69990.0;
    i.relative_velocity=700.0;
    i.taem_velocity=690.0;
    i.measured_drag_accel=.20;
    i.modeled_drag_accel=.20;
    i.aero_confidence=1.0;
    i.has_incidence=true;
    i.angle_of_attack=20.0;
    i.sideslip=0.0;
    i.range_to_site=i.taem_range+1000.0;

    double q=.5*planet_atmospheric_density(&p,i.altitude)*i.relative_velocity*i.relative_velocity;
    assert(isfinite(q)&&q>0.0);
    EntryDragReferenceOutput r=entry_drag_reference_compute(&i,&c);
    assert(r.valid);
    /* A positive finite flow state with high-confidence measured drag remains a
       usable calibration anchor even when q is numerically tiny. Its validity is
       determined by units-consistent evidence, not by the acceleration floor. */
    assert(r.minimum_achievable_drag_accel>c.minimum_drag_accel);
}

static void test_invalid_input_is_rejected(void){
    PlanetModel p=kerbin();LandingConfiguration cfg=landing_configuration_default();
    EntryDragReferenceConfig c=entry_drag_reference_default_config();
    EntryDragReferenceInput i=fixture(&p,&cfg);i.relative_velocity=NAN;
    EntryDragReferenceOutput r=entry_drag_reference_compute(&i,&c);
    assert(!r.valid);
}

int main(void){
    test_default_config();
    test_piecewise_velocity_continuity();
    test_short_range_demands_more_drag();
    test_constant_drag_segment_converges();
    test_transition_tends_to_taem_drag();
    test_weak_aero_confidence_stays_finite();
    test_range_prediction_is_monotonic_and_physical();
    test_range_feedback_reduces_miss();
    test_vertical_lift_demand_tracks_drag_error();
    test_exec_profile_gates_are_ordered();
    test_target_speed_above_taem_still_has_range_budget();
    test_exec_profile_does_not_encode_taem_ownership();
    test_recorded_1003_local_drag_authority_bounds_reference();
    test_recorded_v26_thermal_floor_rejects_fictitious_low_drag();
    test_recorded_v29_reference_never_requests_drag_below_executable_alpha_floor();
    test_positive_low_q_uses_trusted_local_drag_evidence();
    test_invalid_input_is_rejected();
    puts("Entry drag reference tests passed.");
    return 0;
}
