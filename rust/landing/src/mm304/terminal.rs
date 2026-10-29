//! HAC-entrance feasibility. A predicted handoff state is scored by MM305's own
//! spiral HAC (`mm305_hac_spiral_energy_error`: the side whose route turns furthest
//! around the HAC, its acquisition, quartic spiral path and energy reference), so
//! MM304 delivers the energy MM305 needs to fly the full spiral.

use core::ffi::c_void;

use super::state::{Mm304Terminal, MM304_ROUTE_SHORT};
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
    fn mm305_hac_spiral_tangent(
        model: *const c_void,
        live: *const c_void,
        geo: *const HandoffGeometry,
        mass_kg: f64,
        error_m: *mut f64,
        band_m: *mut f64,
        psi: *mut f64,
        tangent_x: *mut f64,
        tangent_y: *mut f64,
        tangent_course: *mut f64,
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
    let mut turn = f64::NAN;
    let (mut tx, mut ty, mut tcourse) = (f64::NAN, f64::NAN, f64::NAN);
    // Scored against MM305's spiral side, the route that turns furthest around the HAC.
    let scored = !model.is_null()
        && mm305_hac_spiral_tangent(model, live, &geo, h.mass, &mut error, &mut band, &mut turn, &mut tx, &mut ty, &mut tcourse);
    let (mut short_error, mut short_band) = (f64::NAN, f64::NAN);
    if !model.is_null() && !mm305_hac_handoff_energy_error(model, live, &geo, h.mass, &mut short_error, &mut short_band) {
        short_error = f64::NAN;
    }
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
        spiral_turn: if scored { turn.to_degrees() } else { f64::NAN },
        short_error,
        short_band,
        // MM305 frame: y = -cross, angle counter-clockwise from the runway heading.
        tangent_along: if scored { tx } else { f64::NAN },
        tangent_cross: if scored { -ty } else { f64::NAN },
        tangent_course: if scored { norm_signed(site.runway_heading - tcourse.to_degrees()).rem_euclid(360.0) } else { f64::NAN },
    }
}

impl Mm304Terminal {
    /// Energy error and band against the route's requirement.
    pub fn route_energy(&self, route: u32) -> (f64, f64) {
        if route == MM304_ROUTE_SHORT { (self.short_error, self.short_band) } else { (self.energy_error, self.band) }
    }

    /// Inside MM305's window for the route: energy in [0, band] within the altitude band.
    pub fn admissible_for(&self, route: u32, min_altitude: f64, max_altitude: f64) -> bool {
        let (e, b) = self.route_energy(route);
        e.is_finite() && e >= 0.0 && e <= b && self.altitude >= min_altitude && self.altitude <= max_altitude
    }
}

/// MM305 admission under the HAC law: the live state inside the spiral's window, or
/// inside the short route's window when the active plan has fallen back to it.
pub fn live_admissible(m: &super::state::Mm304State, min_altitude: f64, max_altitude: f64) -> bool {
    let t = &m.live_terminal;
    t.valid
        && (t.admissible_for(super::state::MM304_ROUTE_SPIRAL, min_altitude, max_altitude)
            || m.plan.valid && m.plan.route == MM304_ROUTE_SHORT && t.admissible_for(MM304_ROUTE_SHORT, min_altitude, max_altitude))
}

/// The point at runway coordinates (along, cross) of `site`: an inverse of
/// `runway_coordinates`, solved by correcting a spherical first guess with it.
pub unsafe fn runway_point(site: &Site, along: f64, cross: f64) -> GeoPoint {
    let origin = GeoPoint { latitude: site.latitude, longitude: site.longitude, altitude: site.altitude };
    let offset = |p: GeoPoint, north: f64, east: f64| GeoPoint {
        latitude: p.latitude + (north / site.radius).to_degrees(),
        longitude: p.longitude + (east / (site.radius * p.latitude.to_radians().cos().max(1e-6))).to_degrees(),
        altitude: p.altitude,
    };
    let h = site.runway_heading.to_radians();
    // Guess with cross positive to the right of the runway heading, then iterate.
    let mut p = offset(origin, along * h.cos() - cross * h.sin(), along * h.sin() + cross * h.cos());
    for _ in 0..4 {
        let (mut a, mut c) = (0.0, 0.0);
        runway_coordinates(p, origin, site.runway_heading, site.radius, &mut a, &mut c);
        let (da, dc) = (along - a, cross - c);
        p = offset(p, da * h.cos() - dc * h.sin(), da * h.sin() + dc * h.cos());
    }
    p
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

#[cfg(test)]
mod tests {
    use super::*;

    /// `runway_point` inverts `runway_coordinates` on both sides of the runway.
    #[test]
    fn runway_point_round_trips() {
        let site = Site { latitude: -0.0486, longitude: -74.7283, altitude: 68.0, runway_heading: 90.0, radius: 600_000.0 };
        let origin = GeoPoint { latitude: site.latitude, longitude: site.longitude, altitude: site.altitude };
        for (along, cross) in [(-20_000.0, 8_000.0), (-12_000.0, -9_000.0), (3_000.0, 500.0)] {
            let p = unsafe { runway_point(&site, along, cross) };
            let (mut a, mut c) = (0.0, 0.0);
            unsafe { runway_coordinates(p, origin, site.runway_heading, site.radius, &mut a, &mut c) };
            assert!((a - along).abs() < 1.0 && (c - cross).abs() < 1.0, "{along},{cross} -> {a},{c}");
        }
    }
}
