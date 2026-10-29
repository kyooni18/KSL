//! Scenario INI: `key = value` lines over the 86 km seed.

use crate::ffi::{v3, Vec3};

#[derive(Copy, Clone)]
pub struct Scenario {
    pub name: [u8; 96],
    pub ut0: f64,
    pub has_cartesian_state: bool,
    pub position_i_m: Vec3,
    pub velocity_i_mps: Vec3,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
    pub heading_deg: f64,
    pub has_surface_flight_state: bool,
    pub surface_speed_mps: f64,
    pub flight_path_angle_deg: f64,
    pub mass_kg: f64,
    pub initial_aoa_deg: f64,
    pub initial_bank_deg: f64,
    pub initial_sideslip_deg: f64,
    /// Optional complete measured body pose/rate groups for live restarts.
    pub has_body_attitude: bool,
    pub has_body_rates: bool,
    pub initial_body_pitch_deg: f64,
    pub initial_body_roll_deg: f64,
    pub initial_body_heading_deg: f64,
    pub initial_body_pitch_rate_deg_s: f64,
    pub initial_body_roll_rate_deg_s: f64,
    pub initial_body_yaw_rate_deg_s: f64,
    pub deorbit_delta_v_mps: f64,
    pub deorbit_duration_s: f64,
    pub deorbit_delay_s: f64,
    pub orbital_engine_available_thrust_n: f64,
    /// Vacuum specific impulse of the orbital engines; 0 keeps mass constant.
    pub orbital_engine_isp_s: f64,
    /// Maximum thrust-axis slew toward the commanded direction; 0 = ideal pointing.
    pub orbital_attitude_slew_dps: f64,
    pub runway_override: bool,
    pub runway_latitude_deg: f64,
    pub runway_longitude_deg: f64,
    pub runway_elevation_m: f64,
    /// Longitudinal deck angle relative to the origin's horizon; negative descends.
    pub runway_slope_deg: f64,
    pub runway_heading_deg: f64,
    pub runway_length_m: f64,
    pub runway_width_m: f64,
}

impl Scenario {
    pub fn name(&self) -> &str {
        let n = self.name.iter().position(|&b| b == 0).unwrap_or(self.name.len());
        core::str::from_utf8(&self.name[..n]).unwrap_or("")
    }

    fn set_name(&mut self, name: &str) {
        self.name = [0; 96];
        self.name[..name.len()].copy_from_slice(name.as_bytes());
    }

    pub fn seed_86km() -> Scenario {
        let mut s = Scenario {
            name: [0; 96],
            ut0: 0.0,
            has_cartesian_state: false,
            position_i_m: v3(0.0, 0.0, 0.0),
            velocity_i_mps: v3(0.0, 0.0, 0.0),
            latitude_deg: -0.05,
            longitude_deg: 105.0,
            altitude_m: 86000.0,
            heading_deg: 0.0,
            has_surface_flight_state: false,
            surface_speed_mps: 0.0,
            flight_path_angle_deg: 0.0,
            mass_kg: 85000.0,
            initial_aoa_deg: 0.0,
            initial_bank_deg: 0.0,
            initial_sideslip_deg: 0.0,
            has_body_attitude: false,
            has_body_rates: false,
            initial_body_pitch_deg: 0.0,
            initial_body_roll_deg: 0.0,
            initial_body_heading_deg: 0.0,
            initial_body_pitch_rate_deg_s: 0.0,
            initial_body_roll_rate_deg_s: 0.0,
            initial_body_yaw_rate_deg_s: 0.0,
            deorbit_delta_v_mps: 60.0,
            deorbit_duration_s: 0.0,
            deorbit_delay_s: 0.0,
            orbital_engine_available_thrust_n: 0.0,
            orbital_engine_isp_s: 0.0,
            orbital_attitude_slew_dps: 0.0,
            runway_override: false,
            runway_latitude_deg: -0.0486,
            runway_longitude_deg: -74.7240,
            runway_elevation_m: 70.0,
            runway_slope_deg: 0.0,
            runway_heading_deg: 90.0,
            runway_length_m: 2500.0,
            runway_width_m: 70.0,
        };
        s.set_name("orbit86km-seed");
        s.heading_deg = initial_bearing_deg(s.latitude_deg, s.longitude_deg, s.runway_latitude_deg, s.runway_longitude_deg);
        s
    }

    /// Parse a scenario file; `None` on any malformed, unknown or inconsistent entry.
    pub fn load(path: &str) -> Option<Scenario> {
        let text = std::fs::read(path).ok()?;
        Self::parse(&String::from_utf8_lossy(&text))
    }

    pub fn parse(text: &str) -> Option<Scenario> {
        const POSITION: [u32; 3] = [1, 2, 4];
        const VELOCITY: [u32; 3] = [8, 16, 32];
        const CARTESIAN: u32 = 63;
        const RUNWAY: u32 = 64;
        const SURFACE: u32 = 128;
        const HEADING: u32 = 256;
        const BODY_ATTITUDE: u32 = 512 | 1024 | 2048;
        const BODY_RATES: u32 = 4096 | 8192 | 16384;
        let mut s = Scenario::seed_86km();
        let mut seen = 0u32;
        for raw in text.lines() {
            if raw.len() >= 511 {
                return None;
            }
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let (key, value) = line.split_once('=')?;
            let (key, value) = (key.trim(), value.trim());
            if key == "name" {
                if value.is_empty() || value.len() >= s.name.len() {
                    return None;
                }
                s.set_name(value);
                continue;
            }
            let value = value.split('#').next().unwrap_or("").trim();
            let number: f64 = value.parse().ok().filter(|v: &f64| v.is_finite())?;
            let (slot, flag): (&mut f64, u32) = match key {
                "ut0" => (&mut s.ut0, 0),
                "latitude_deg" => (&mut s.latitude_deg, 0),
                "longitude_deg" => (&mut s.longitude_deg, 0),
                "altitude_m" => (&mut s.altitude_m, 0),
                "heading_deg" => (&mut s.heading_deg, HEADING),
                "surface_speed_mps" | "speed_mps" => (&mut s.surface_speed_mps, SURFACE),
                "flight_path_angle_deg" => (&mut s.flight_path_angle_deg, 0),
                "mass_kg" => (&mut s.mass_kg, 0),
                "initial_aoa_deg" => (&mut s.initial_aoa_deg, 0),
                "initial_bank_deg" => (&mut s.initial_bank_deg, 0),
                "initial_sideslip_deg" => (&mut s.initial_sideslip_deg, 0),
                "initial_body_pitch_deg" => (&mut s.initial_body_pitch_deg, 512),
                "initial_body_roll_deg" => (&mut s.initial_body_roll_deg, 1024),
                "initial_body_heading_deg" => (&mut s.initial_body_heading_deg, 2048),
                "initial_body_pitch_rate_deg_s" => (&mut s.initial_body_pitch_rate_deg_s, 4096),
                "initial_body_roll_rate_deg_s" => (&mut s.initial_body_roll_rate_deg_s, 8192),
                "initial_body_yaw_rate_deg_s" => (&mut s.initial_body_yaw_rate_deg_s, 16384),
                "deorbit_delta_v_mps" => (&mut s.deorbit_delta_v_mps, 0),
                "deorbit_duration_s" => (&mut s.deorbit_duration_s, 0),
                "deorbit_delay_s" => (&mut s.deorbit_delay_s, 0),
                "orbital_engine_available_thrust_n" => (&mut s.orbital_engine_available_thrust_n, 0),
                "orbital_engine_isp_s" => (&mut s.orbital_engine_isp_s, 0),
                "orbital_attitude_slew_dps" => (&mut s.orbital_attitude_slew_dps, 0),
                "runway_latitude_deg" => (&mut s.runway_latitude_deg, RUNWAY),
                "runway_longitude_deg" => (&mut s.runway_longitude_deg, RUNWAY),
                "runway_elevation_m" => (&mut s.runway_elevation_m, RUNWAY),
                "runway_slope_deg" => (&mut s.runway_slope_deg, RUNWAY),
                "runway_heading_deg" => (&mut s.runway_heading_deg, RUNWAY),
                "runway_length_m" => (&mut s.runway_length_m, RUNWAY),
                "runway_width_m" => (&mut s.runway_width_m, RUNWAY),
                "position_x_m" => (&mut s.position_i_m.x, POSITION[0]),
                "position_y_m" => (&mut s.position_i_m.y, POSITION[1]),
                "position_z_m" => (&mut s.position_i_m.z, POSITION[2]),
                "velocity_x_mps" => (&mut s.velocity_i_mps.x, VELOCITY[0]),
                "velocity_y_mps" => (&mut s.velocity_i_mps.y, VELOCITY[1]),
                "velocity_z_mps" => (&mut s.velocity_i_mps.z, VELOCITY[2]),
                _ => return None,
            };
            *slot = number;
            seen |= flag;
        }
        let cartesian = seen & CARTESIAN;
        let (body_attitude, body_rates) = (seen & BODY_ATTITUDE, seen & BODY_RATES);
        if (cartesian != 0 && cartesian != CARTESIAN)
            || (body_attitude != 0 && body_attitude != BODY_ATTITUDE)
            || (body_rates != 0 && body_rates != BODY_RATES)
            || s.initial_body_pitch_deg.abs() > 90.0
            || s.initial_sideslip_deg.abs() > 90.0
            || (cartesian != 0 && seen & SURFACE != 0)
            || s.mass_kg <= 0.0
            || s.runway_length_m <= 0.0
            || s.runway_width_m <= 0.0
            || s.runway_slope_deg.abs() >= 90.0
            || s.latitude_deg.abs() > 90.0
            || s.runway_latitude_deg.abs() > 90.0
            || s.flight_path_angle_deg.abs() > 90.0
            || s.surface_speed_mps < 0.0
            || s.deorbit_duration_s < 0.0
            || s.deorbit_delay_s < 0.0
            || s.orbital_engine_available_thrust_n < 0.0
            || s.orbital_engine_isp_s < 0.0
            || s.orbital_attitude_slew_dps < 0.0
        {
            return None;
        }
        s.has_cartesian_state = cartesian == CARTESIAN;
        s.has_body_attitude = body_attitude == BODY_ATTITUDE;
        s.has_body_rates = body_rates == BODY_RATES;
        s.has_surface_flight_state = seen & SURFACE != 0;
        s.runway_override = seen & RUNWAY != 0;
        if !s.has_cartesian_state && seen & HEADING == 0 {
            s.heading_deg = initial_bearing_deg(s.latitude_deg, s.longitude_deg, s.runway_latitude_deg, s.runway_longitude_deg);
        }
        Some(s)
    }
}

fn initial_bearing_deg(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) -> f64 {
    let deg2rad = (-1.0f64).acos() / 180.0;
    let (lat1, lat2) = (lat1_deg * deg2rad, lat2_deg * deg2rad);
    let dlon = (lon2_deg - lon1_deg) * deg2rad;
    let y = dlon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    let bearing = ((y.atan2(x) / deg2rad) + 360.0) % 360.0;
    if bearing < 0.0 { bearing + 360.0 } else { bearing }
}
