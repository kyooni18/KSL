#include "entry_drag_reference.h"
#include "landing_api.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(double actual, double expected) {
    if (!isfinite(actual) || fabs(actual - expected) > 1e-9) {
        fprintf(stderr, "vertical support %.12f != inertial expectation %.12f\n", actual, expected);
        assert(false);
    }
}

static EntryDragReferenceInput fixture(const PlanetModel *planet, const VehicleProfile *vehicle) {
    return (EntryDragReferenceInput){
        .phase = ENTRY_PHASE_CONSTANT_DRAG, .relative_velocity = 1800.0,
        .latitude = 0.0, .altitude = 38000.0,
        .measured_drag_accel = 2.0, .modeled_drag_accel = 2.0, .aero_confidence = 0.8,
        .range_to_site = 300000.0, .taem_range = 90000.0,
        .taem_latitude = 0.0, .taem_altitude = 22000.0, .taem_velocity = 750.0,
        .planet = planet, .vehicle = vehicle,
    };
}

int main(void) {
    LandingConfiguration configuration = landing_configuration_default();
    PlanetModel planet = {
        .radius = 600000.0, .gravitational_parameter = 3.5316e12,
        .rotational_speed = 0.000291570900559802, .atmosphere_depth = 70000.0,
        .surface_density = 1.225, .atmosphere_adiabatic_index = 1.4,
        .north_axis = {0.0, 0.0, 1.0}, .prime_meridian_at_epoch = {1.0, 0.0, 0.0},
    };
    EntryDragReferenceConfig cfg = entry_drag_reference_default_config();
    cfg.vertical_drag_gain = 0.0;
    cfg.vertical_range_gain = 0.0;
    EntryDragReferenceInput input = fixture(&planet, &configuration.vehicle);
    double radius = planet.radius + input.altitude;
    double gravity = planet.gravitational_parameter / (radius * radius);
    double rotation = planet.rotational_speed * radius;
    double airspeed = input.relative_velocity;
    VehicleState state = {.position = {radius, 0.0, 0.0}, .mass = 25816.0};
    input.state = &state;

    /* Identical airspeed does not imply identical inertial curvature support. */
    const Vector3 velocities[] = {
        {-100.0, airspeed + rotation, 0.0},       /* east */
        {-100.0, -airspeed + rotation, 0.0},      /* west */
        {-100.0, rotation, airspeed},            /* north */
        {0.0, airspeed, 0.0},                    /* no-rotation reference */
        {-500.0, airspeed + rotation, 0.0},       /* radial velocity excluded */
    };
    double references[5];
    for (unsigned n = 0; n < 5; ++n) {
        state.velocity = velocities[n];
        double horizontal_squared = state.velocity.y * state.velocity.y + state.velocity.z * state.velocity.z;
        double expected = clampd(gravity - horizontal_squared / radius, 0.12 * gravity, 0.995 * gravity);
        EntryDragReferenceOutput output = entry_drag_reference_compute(&input, &cfg);
        assert(output.valid);
        near(output.required_vertical_lift_accel, expected);
        references[n] = output.reference_drag_accel;
    }
    /* Correcting the current inertial support must not substitute orbital speed
       into air-relative drag, Mach, energy, or range-profile calculations. */
    for (unsigned n = 1; n < 5; ++n) near(references[n], references[0]);

    /* The projection is coordinate-independent, including a polar position. */
    state.position = (Vector3){0.0, 0.0, radius};
    state.velocity = (Vector3){airspeed, 0.0, -300.0};
    near(entry_drag_reference_compute(&input, &cfg).required_vertical_lift_accel,
         clampd(gravity - airspeed * airspeed / radius, 0.12 * gravity, 0.995 * gravity));

    /* Existing support bounds still apply in the near-orbital and slow limits. */
    state.velocity = (Vector3){sqrt(planet.gravitational_parameter / radius), 0.0, 0.0};
    near(entry_drag_reference_compute(&input, &cfg).required_vertical_lift_accel, 0.12 * gravity);
    state.velocity = (Vector3){0.0, 0.0, 0.0};
    near(entry_drag_reference_compute(&input, &cfg).required_vertical_lift_accel, 0.995 * gravity);

    state.velocity.x = NAN;
    assert(!entry_drag_reference_compute(&input, &cfg).valid);
    state.velocity = (Vector3){airspeed, 0.0, 0.0};
    state.position = (Vector3){0.0, 0.0, 0.0};
    assert(!entry_drag_reference_compute(&input, &cfg).valid);

    input.state = NULL;
    EntryDragReferenceOutput reduced = entry_drag_reference_compute(&input, &cfg);
    assert(reduced.valid);
    near(reduced.required_vertical_lift_accel,
         clampd(gravity - airspeed * airspeed / radius, 0.12 * gravity, 0.995 * gravity));
    puts("Entry inertial-frame support regressions passed.");
    return 0;
}
