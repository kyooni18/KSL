//! The flight-physics plant, shared with the landing backend's predictor.

pub use landing::physics::flight_physics::{
    AeroForces, AeroTable, AtmosphereSample, AttitudeModel, FlightAirborneState, FlightAirborneStepInput, KerbinWorld,
    Quat, Vec3,
};

#[derive(Copy, Clone, Debug, Default)]
#[repr(C)]
pub struct Lla {
    pub lat_rad: f64,
    pub lon_rad: f64,
    pub altitude_m: f64,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct LocalFrame {
    pub north: Vec3,
    pub east: Vec3,
    pub up: Vec3,
}

#[derive(Copy, Clone, Debug, Default)]
#[repr(C)]
pub struct Runway {
    pub lat_rad: f64,
    pub lon_rad: f64,
    pub elevation_m: f64,
    pub heading_rad: f64,
    pub length_m: f64,
    pub width_m: f64,
}

#[allow(clashing_extern_declarations)]
extern "C" {
    pub fn world_seed_kerbin(w: *mut KerbinWorld);
    pub fn world_load_atmosphere_csv(w: *mut KerbinWorld, path: *const libc::c_char) -> bool;
    pub fn world_lla(w: *const KerbinWorld, p: Vec3, ut: f64) -> Lla;
    pub fn world_lla_to_inertial(w: *const KerbinWorld, lat: f64, lon: f64, alt: f64, ut: f64) -> Vec3;
    pub fn world_local_frame_i(w: *const KerbinWorld, p: Vec3, ut: f64) -> LocalFrame;
    pub fn world_atmosphere_velocity_i(w: *const KerbinWorld, p: Vec3) -> Vec3;
    pub fn world_atmosphere_sample_state(w: *const KerbinWorld, p: Vec3, ut: f64) -> AtmosphereSample;
    pub fn runway_seed_ksp09(r: *mut Runway);
    pub fn runway_contains(r: *const Runway, along: f64, cross: f64) -> bool;
    pub fn ss_runway_coordinates(
        w: *const KerbinWorld,
        r: *const Runway,
        p: Vec3,
        ut: f64,
        along: *mut f64,
        cross: *mut f64,
        vertical: *mut f64,
    );
    pub fn aero_seed_stsn(a: *mut AeroTable);
    pub fn aero_load_csv(a: *mut AeroTable, path: *const libc::c_char) -> bool;
    pub fn aero_load_book_csv(a: *mut AeroTable, path: *const libc::c_char) -> bool;
    pub fn aero_compute(
        w: *const KerbinWorld,
        a: *const AeroTable,
        p: Vec3,
        v: Vec3,
        ut: f64,
        mass: f64,
        aoa: f64,
        bank: f64,
    ) -> AeroForces;
    pub fn attitude_seed(a: *mut AttitudeModel);
    pub fn attitude_load_ini(a: *mut AttitudeModel, path: *const libc::c_char) -> bool;
    pub fn attitude_set_command(a: *mut AttitudeModel, aoa: f64, bank: f64);
    pub fn attitude_set_inputs(a: *mut AttitudeModel, pitch: f64, roll: f64, yaw: f64);
    pub fn attitude_body_quat(p: Vec3, vair: Vec3, aoa: f64, bank: f64) -> Quat;
    pub fn attitude_body_quat_wind(p: Vec3, vair: Vec3, aoa: f64, bank: f64) -> Quat;
    pub fn attitude_body_quat_euler(frame: LocalFrame, pitch: f64, heading: f64, roll: f64) -> Quat;
    pub fn quat_rotate(q: Quat, v: Vec3) -> Vec3;
    pub fn quat_from_basis(forward: Vec3, right: Vec3, up: Vec3) -> Quat;
    pub fn flight_physics_airborne_step(
        w: *const KerbinWorld,
        a: *const AeroTable,
        input: *const FlightAirborneStepInput,
        state: *mut FlightAirborneState,
    );
}

/// Vector algebra, operation for operation as the plant's `math3`.
pub const fn v3(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3 { x, y, z }
}
pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    v3(a.x + b.x, a.y + b.y, a.z + b.z)
}
pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    v3(a.x - b.x, a.y - b.y, a.z - b.z)
}
pub fn scale(a: Vec3, s: f64) -> Vec3 {
    v3(a.x * s, a.y * s, a.z * s)
}
pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}
pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}
pub fn normalized(a: Vec3) -> Vec3 {
    let n = norm(a);
    if n > 1e-12 { scale(a, 1.0 / n) } else { v3(0.0, 0.0, 0.0) }
}
pub fn rotate_axis(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let axis = normalized(axis);
    let (s, c) = (angle.sin(), angle.cos());
    add(add(scale(v, c), scale(cross(axis, v), s)), scale(axis, dot(axis, v) * (1.0 - c)))
}
pub fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo { lo } else if x > hi { hi } else { x }
}
pub fn deg2rad(x: f64) -> f64 {
    x * (core::f64::consts::PI / 180.0)
}
pub fn rad2deg(x: f64) -> f64 {
    x * (180.0 / core::f64::consts::PI)
}
pub fn rotate(q: Quat, v: Vec3) -> Vec3 {
    unsafe { quat_rotate(q, v) }
}
