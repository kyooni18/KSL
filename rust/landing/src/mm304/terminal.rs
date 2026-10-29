//! HAC-entrance feasibility. A predicted handoff state is scored by MM305's own
//! spiral HAC (`mm305_hac_handoff_energy_error`: both turn sides, the quartic
//! spiral path and its energy reference), so MM304 aims at what MM305 can fly.

use core::ffi::c_void;

use super::state::Mm304Terminal;
use crate::prediction::predictor::GeoPoint;

/// Mirror of the C `TaemGeometryState`.
#[derive(Copy, Clone, Debug, Default)]
#[repr(C)]
pub struct HandoffGeometry {
    pub runway_along_m: f64,
    pub runway_cross_m: f64,
    pub altitude_above_runway_m: f64,
    pub ground_speed_mps: f64,
    pub airspeed_mps: f64,
    pub course_deg: f64,
    pub flight_path_angle_deg: f64,
    pub heading_error_deg: f64,
    pub curvature_right_per_m: f64,
}

extern "C" {
    fn runway_coordinates(point: GeoPoint, site: GeoPoint, heading: f64, radius: f64, along: *mut f64, cross: *mut f64);
    fn mm305_hac_handoff_energy_error(
        model: *const c_void,
        live: *const c_void,
        geo: *const HandoffGeometry,
        mass_kg: f64,
        error_m: *mut f64,
        band_m: *mut f64,
    ) -> bool;
}

pub struct Site {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub runway_heading: f64,
    pub radius: f64,
}

pub struct Handoff {
    pub ut: f64,
    pub point: GeoPoint,
    pub airspeed: f64,
    pub ground_speed: f64,
    pub course: f64,
    pub flight_path_angle: f64,
    pub mass: f64,
}

pub unsafe fn evaluate(
    h: &Handoff,
    site: &Site,
    model: *const c_void,
    live: *const c_void,
    min_altitude: f64,
    max_altitude: f64,
) -> Mm304Terminal {
    let mut along = f64::NAN;
    let mut cross = f64::NAN;
    let site_point = GeoPoint { latitude: site.latitude, longitude: site.longitude, altitude: site.altitude };
    runway_coordinates(h.point, site_point, site.runway_heading, site.radius, &mut along, &mut cross);
    let heading_error = norm_signed(h.course - site.runway_heading);
    let geo = HandoffGeometry {
        runway_along_m: along,
        runway_cross_m: cross,
        altitude_above_runway_m: h.point.altitude - site.altitude,
        ground_speed_mps: h.ground_speed,
        airspeed_mps: h.airspeed,
        course_deg: h.course,
        flight_path_angle_deg: h.flight_path_angle,
        heading_error_deg: heading_error,
        curvature_right_per_m: 0.0,
    };
    let mut error = f64::NAN;
    let mut band = f64::NAN;
    let scored = !model.is_null() && mm305_hac_handoff_energy_error(model, live, &geo, h.mass, &mut error, &mut band);
    let in_band = h.point.altitude >= min_altitude && h.point.altitude <= max_altitude;
    Mm304Terminal {
        valid: scored && error.is_finite(),
        admissible: scored && in_band && error >= 0.0 && error <= band,
        ut: h.ut,
        along,
        cross,
        heading_error,
        altitude: h.point.altitude,
        speed: h.airspeed,
        flight_path_angle: h.flight_path_angle,
        energy_error: if scored { error } else { f64::NAN },
        band: if scored { band } else { f64::NAN },
    }
}

pub fn norm_signed(deg: f64) -> f64 {
    let mut d = deg % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d <= -180.0 {
        d += 360.0;
    }
    d
}
