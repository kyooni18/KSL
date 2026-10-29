use crate::sys::*;
extern "C" {
    fn acos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn asin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn atan2(
        __y: ::core::ffi::c_double,
        __x: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn cos(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn ss_v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vec3;
    fn v3_scale(a: Vec3, s: ::core::ffi::c_double) -> Vec3;
    fn v3_cross(a: Vec3, b: Vec3) -> Vec3;
    fn v3_norm(a: Vec3) -> ::core::ffi::c_double;
    fn v3_normalized(a: Vec3) -> Vec3;
    fn ss_clampd(
        x: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn deg2rad(x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn rad2deg(x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn wrap_pi(x: ::core::ffi::c_double) -> ::core::ffi::c_double;
}
pub type size_t = usize;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vec3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LLA {
    pub lat_rad: ::core::ffi::c_double,
    pub lon_rad: ::core::ffi::c_double,
    pub altitude_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LocalFrame {
    pub north: Vec3,
    pub east: Vec3,
    pub up: Vec3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmosphereSample {
    pub density_kg_m3: ::core::ffi::c_double,
    pub pressure_pa: ::core::ffi::c_double,
    pub temperature_k: ::core::ffi::c_double,
    pub speed_of_sound_mps: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmospherePoint {
    pub altitude_m: ::core::ffi::c_double,
    pub sample: AtmosphereSample,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AtmosphereTable {
    pub p: [AtmospherePoint; 2048],
    pub count: size_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct KerbinWorld {
    pub radius_m: ::core::ffi::c_double,
    pub mu_m3_s2: ::core::ffi::c_double,
    pub rotation_rate_rad_s: ::core::ffi::c_double,
    pub atmosphere_top_m: ::core::ffi::c_double,
    pub rotation_phase_rad_at_ut0: ::core::ffi::c_double,
    pub orbital_rate_rad_s: ::core::ffi::c_double,
    pub solar_phase_rad_at_ut0: ::core::ffi::c_double,
    pub stock_spatial_atmosphere: bool,
    pub atmosphere: AtmosphereTable,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Runway {
    pub lat_rad: ::core::ffi::c_double,
    pub lon_rad: ::core::ffi::c_double,
    pub elevation_m: ::core::ffi::c_double,
    pub heading_rad: ::core::ffi::c_double,
    pub length_m: ::core::ffi::c_double,
    pub width_m: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CurveKey {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub in_tangent: ::core::ffi::c_double,
    pub out_tangent: ::core::ffi::c_double,
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const ATM_TABLE_MAX: ::core::ffi::c_int = 2048 as ::core::ffi::c_int;
static mut PI: ::core::ffi::c_double = 3.14159265358979323846f64;
static mut KERBIN_MOLAR_MASS_KG_MOL: ::core::ffi::c_double = 0.0289644002914429f64;
static mut KERBIN_ADIABATIC_INDEX: ::core::ffi::c_double = 1.39999997615814f64;
static mut UNIVERSAL_GAS_CONSTANT: ::core::ffi::c_double = 8.31446261815324f64;
unsafe extern "C" fn curve_eval(
    mut curve: *const CurveKey,
    mut count: size_t,
    mut x: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if count == 0 as size_t {
        return 0.0f64;
    }
    if x <= (*curve.offset(0 as ::core::ffi::c_int as isize)).x {
        return (*curve.offset(0 as ::core::ffi::c_int as isize)).y;
    }
    if x >= (*curve.offset(count.wrapping_sub(1 as size_t) as isize)).x {
        return (*curve.offset(count.wrapping_sub(1 as size_t) as isize)).y;
    }
    let mut i: size_t = 1 as size_t;
    while i < count {
        if x <= (*curve.offset(i as isize)).x {
            let mut a: *const CurveKey = curve
                .offset(i.wrapping_sub(1 as size_t) as isize) as *const CurveKey;
            let mut b: *const CurveKey = curve.offset(i as isize) as *const CurveKey;
            let mut dx: ::core::ffi::c_double = (*b).x - (*a).x;
            let mut t: ::core::ffi::c_double = (x - (*a).x) / dx;
            let mut t2: ::core::ffi::c_double = t * t;
            let mut t3: ::core::ffi::c_double = t2 * t;
            let mut h00: ::core::ffi::c_double = 2.0f64 * t3 - 3.0f64 * t2 + 1.0f64;
            let mut h10: ::core::ffi::c_double = t3 - 2.0f64 * t2 + t;
            let mut h01: ::core::ffi::c_double = -2.0f64 * t3 + 3.0f64 * t2;
            let mut h11: ::core::ffi::c_double = t3 - t2;
            return h00 * (*a).y + h10 * dx * (*a).out_tangent + h01 * (*b).y
                + h11 * dx * (*b).in_tangent;
        }
        i = i.wrapping_add(1);
    }
    return (*curve.offset(count.wrapping_sub(1 as size_t) as isize)).y;
}
static mut KERBIN_PRESSURE_CURVE: [CurveKey; 21] = [
    CurveKey {
        x: 0.0f64,
        y: 101.325f64,
        in_tangent: 0.0f64,
        out_tangent: -0.01501631f64,
    },
    CurveKey {
        x: 1241.025f64,
        y: 84.02916f64,
        in_tangent: -0.01289846f64,
        out_tangent: -0.01289826f64,
    },
    CurveKey {
        x: 2439.593f64,
        y: 69.68138f64,
        in_tangent: -0.01107876f64,
        out_tangent: -0.01107859f64,
    },
    CurveKey {
        x: 3597.11f64,
        y: 57.78001f64,
        in_tangent: -0.009515483f64,
        out_tangent: -0.009515338f64,
    },
    CurveKey {
        x: 4714.942f64,
        y: 47.90862f64,
        in_tangent: -0.00817254f64,
        out_tangent: -0.008172415f64,
    },
    CurveKey {
        x: 5794.409f64,
        y: 39.72148f64,
        in_tangent: -0.00701892f64,
        out_tangent: -0.007018813f64,
    },
    CurveKey {
        x: 6836.791f64,
        y: 32.93169f64,
        in_tangent: -0.006027969f64,
        out_tangent: -0.006027877f64,
    },
    CurveKey {
        x: 7843.328f64,
        y: 27.30109f64,
        in_tangent: -0.005176778f64,
        out_tangent: -0.0051767f64,
    },
    CurveKey {
        x: 8815.22f64,
        y: 22.63206f64,
        in_tangent: -0.004445662f64,
        out_tangent: -0.004445578f64,
    },
    CurveKey {
        x: 10786.42f64,
        y: 15.3684f64,
        in_tangent: -0.003016528f64,
        out_tangent: -0.00301646f64,
    },
    CurveKey {
        x: 12101.4f64,
        y: 11.87313f64,
        in_tangent: -0.002329273f64,
        out_tangent: -0.00232922f64,
    },
    CurveKey {
        x: 13417.05f64,
        y: 9.172798f64,
        in_tangent: -0.001798594f64,
        out_tangent: -0.001798554f64,
    },
    CurveKey {
        x: 16678.47f64,
        y: 4.842261f64,
        in_tangent: -0.0009448537f64,
        out_tangent: -0.0009448319f64,
    },
    CurveKey {
        x: 21143.1f64,
        y: 2.050097f64,
        in_tangent: -0.0003894095f64,
        out_tangent: -0.0003894005f64,
    },
    CurveKey {
        x: 26977.92f64,
        y: 0.6905929f64,
        in_tangent: -0.0001252565f64,
        out_tangent: -0.0001252534f64,
    },
    CurveKey {
        x: 33593.82f64,
        y: 0.2201734f64,
        in_tangent: -3.626878e-05f64,
        out_tangent: -3.626788e-05f64,
    },
    CurveKey {
        x: 42081.87f64,
        y: 0.05768469f64,
        in_tangent: -9.063159e-06f64,
        out_tangent: -9.062975e-06f64,
    },
    CurveKey {
        x: 49312.13f64,
        y: 0.01753794f64,
        in_tangent: -3.029397e-06f64,
        out_tangent: -3.029335e-06f64,
    },
    CurveKey {
        x: 56669.95f64,
        y: 0.004591824f64,
        in_tangent: -8.827175e-07f64,
        out_tangent: -8.826996e-07f64,
    },
    CurveKey {
        x: 62300.84f64,
        y: 0.001497072f64,
        in_tangent: -3.077091e-07f64,
        out_tangent: -3.077031e-07f64,
    },
    CurveKey {
        x: 70000.0f64,
        y: 0.0f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
];
static mut KERBIN_TEMPERATURE_CURVE: [CurveKey; 9] = [
    CurveKey {
        x: 0.0f64,
        y: 288.15f64,
        in_tangent: 0.0f64,
        out_tangent: -0.008125f64,
    },
    CurveKey {
        x: 8815.22f64,
        y: 216.65f64,
        in_tangent: -0.008096968f64,
        out_tangent: 0.0f64,
    },
    CurveKey {
        x: 16050.39f64,
        y: 216.65f64,
        in_tangent: 0.0f64,
        out_tangent: 0.001242164f64,
    },
    CurveKey {
        x: 25729.23f64,
        y: 228.65f64,
        in_tangent: 0.001237475f64,
        out_tangent: 0.003464929f64,
    },
    CurveKey {
        x: 37879.44f64,
        y: 270.65f64,
        in_tangent: 0.00344855f64,
        out_tangent: 0.0f64,
    },
    CurveKey {
        x: 41129.24f64,
        y: 270.65f64,
        in_tangent: 0.0f64,
        out_tangent: -0.003444189f64,
    },
    CurveKey {
        x: 57440.13f64,
        y: 214.65f64,
        in_tangent: -0.003422425f64,
        out_tangent: -0.002444589f64,
    },
    CurveKey {
        x: 68797.88f64,
        y: 186.946f64,
        in_tangent: -0.002433851f64,
        out_tangent: 0.0f64,
    },
    CurveKey {
        x: 70000.0f64,
        y: 186.946f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
];
static mut KERBIN_TEMPERATURE_SUN_MULT_CURVE: [CurveKey; 8] = [
    CurveKey {
        x: 0.0f64,
        y: 1.0f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
    CurveKey {
        x: 8815.22f64,
        y: 0.3f64,
        in_tangent: -5.91316e-05f64,
        out_tangent: -5.91316e-05f64,
    },
    CurveKey {
        x: 16050.39f64,
        y: 0.0f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
    CurveKey {
        x: 25729.23f64,
        y: 0.0f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
    CurveKey {
        x: 37879.44f64,
        y: 0.2f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
    CurveKey {
        x: 57440.13f64,
        y: 0.2f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
    CurveKey {
        x: 63902.72f64,
        y: 1.0f64,
        in_tangent: 0.0001012837f64,
        out_tangent: 0.0001012837f64,
    },
    CurveKey {
        x: 70000.0f64,
        y: 1.2f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
];
static mut KERBIN_LATITUDE_BIAS_CURVE: [CurveKey; 9] = [
    CurveKey {
        x: 0.0f64,
        y: 17.0f64,
        in_tangent: 0.0f64,
        out_tangent: -0.3316494f64,
    },
    CurveKey {
        x: 10.0f64,
        y: 12.0f64,
        in_tangent: -0.65f64,
        out_tangent: -0.65f64,
    },
    CurveKey {
        x: 18.0f64,
        y: 6.36371f64,
        in_tangent: -0.4502313f64,
        out_tangent: -0.4502313f64,
    },
    CurveKey {
        x: 30.0f64,
        y: 0.0f64,
        in_tangent: -1.3f64,
        out_tangent: -1.3f64,
    },
    CurveKey {
        x: 35.0f64,
        y: -10.0f64,
        in_tangent: -1.65f64,
        out_tangent: -1.65f64,
    },
    CurveKey {
        x: 45.0f64,
        y: -23.0f64,
        in_tangent: -1.05f64,
        out_tangent: -1.05f64,
    },
    CurveKey {
        x: 55.0f64,
        y: -31.0f64,
        in_tangent: -0.6f64,
        out_tangent: -0.6f64,
    },
    CurveKey {
        x: 70.0f64,
        y: -37.0f64,
        in_tangent: -0.6689383f64,
        out_tangent: -0.6689383f64,
    },
    CurveKey {
        x: 90.0f64,
        y: -50.0f64,
        in_tangent: -0.02418368f64,
        out_tangent: 0.0f64,
    },
];
static mut KERBIN_LATITUDE_SUN_MULT_CURVE: [CurveKey; 6] = [
    CurveKey {
        x: 0.0f64,
        y: 9.0f64,
        in_tangent: 0.0f64,
        out_tangent: 0.1554984f64,
    },
    CurveKey {
        x: 40.0f64,
        y: 14.2f64,
        in_tangent: 0.08154097f64,
        out_tangent: 0.08154097f64,
    },
    CurveKey {
        x: 55.0f64,
        y: 14.9f64,
        in_tangent: -0.006055089f64,
        out_tangent: -0.006055089f64,
    },
    CurveKey {
        x: 68.0f64,
        y: 12.16518f64,
        in_tangent: -0.2710912f64,
        out_tangent: -0.2710912f64,
    },
    CurveKey {
        x: 76.0f64,
        y: 8.582909f64,
        in_tangent: -0.6021729f64,
        out_tangent: -0.6021729f64,
    },
    CurveKey {
        x: 90.0f64,
        y: 5.0f64,
        in_tangent: 0.0f64,
        out_tangent: 0.0f64,
    },
];
unsafe extern "C" fn kerbin_pressure_pa(
    mut altitude_m: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if altitude_m >= 70000.0f64 {
        return 0.0f64;
    }
    return fmax(
        0.0f64,
        1000.0f64
            * curve_eval(
                &raw const KERBIN_PRESSURE_CURVE as *const CurveKey,
                (::core::mem::size_of::<[CurveKey; 21]>() as size_t)
                    .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
                fmax(0.0f64, altitude_m),
            ),
    );
}
unsafe extern "C" fn kerbin_temperature_k(
    mut w: *const KerbinWorld,
    mut altitude_m: ::core::ffi::c_double,
    mut lat_rad: ::core::ffi::c_double,
    mut lon_rad: ::core::ffi::c_double,
    mut ut: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut h: ::core::ffi::c_double = ss_clampd(altitude_m, 0.0f64, 70000.0f64);
    let mut lat_deg: ::core::ffi::c_double = fabs(rad2deg(lat_rad));
    let mut hour_angle: ::core::ffi::c_double = lon_rad
        + ((*w).rotation_rate_rad_s - (*w).orbital_rate_rad_s) * ut
        - (*w).solar_phase_rad_at_ut0;
    let mut sun_dot_normalized: ::core::ffi::c_double = 0.5f64
        * cos(hour_angle - PI / 4.0f64) + 0.5f64;
    let mut offset: ::core::ffi::c_double = curve_eval(
        &raw const KERBIN_LATITUDE_BIAS_CURVE as *const CurveKey,
        (::core::mem::size_of::<[CurveKey; 9]>() as size_t)
            .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
        lat_deg,
    )
        + curve_eval(
            &raw const KERBIN_LATITUDE_SUN_MULT_CURVE as *const CurveKey,
            (::core::mem::size_of::<[CurveKey; 6]>() as size_t)
                .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
            lat_deg,
        ) * sun_dot_normalized;
    return curve_eval(
        &raw const KERBIN_TEMPERATURE_CURVE as *const CurveKey,
        (::core::mem::size_of::<[CurveKey; 9]>() as size_t)
            .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
        h,
    )
        + curve_eval(
            &raw const KERBIN_TEMPERATURE_SUN_MULT_CURVE as *const CurveKey,
            (::core::mem::size_of::<[CurveKey; 8]>() as size_t)
                .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
            h,
        ) * offset;
}
unsafe extern "C" fn kerbin_stock_atmosphere(
    mut w: *const KerbinWorld,
    mut altitude_m: ::core::ffi::c_double,
    mut lat_rad: ::core::ffi::c_double,
    mut lon_rad: ::core::ffi::c_double,
    mut ut: ::core::ffi::c_double,
) -> AtmosphereSample {
    let mut s: AtmosphereSample = AtmosphereSample {
        density_kg_m3: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        pressure_pa: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        temperature_k: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_of_sound_mps: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    if altitude_m >= (*w).atmosphere_top_m {
        return s;
    }
    s.pressure_pa = kerbin_pressure_pa(altitude_m);
    s.temperature_k = kerbin_temperature_k(w, altitude_m, lat_rad, lon_rad, ut);
    if s.pressure_pa > 0.0f64 && s.temperature_k > 0.0f64 {
        s.density_kg_m3 = s.pressure_pa * KERBIN_MOLAR_MASS_KG_MOL
            / (UNIVERSAL_GAS_CONSTANT * s.temperature_k);
    }
    if s.temperature_k > 0.0f64 {
        s.speed_of_sound_mps = sqrt(
            KERBIN_ADIABATIC_INDEX * (UNIVERSAL_GAS_CONSTANT / KERBIN_MOLAR_MASS_KG_MOL)
                * s.temperature_k,
        );
    }
    return s;
}
#[no_mangle]
pub unsafe extern "C" fn world_seed_kerbin(mut w: *mut KerbinWorld) {
    memset(
        w as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<KerbinWorld>() as size_t,
    );
    (*w).radius_m = 600000.0f64;
    (*w).mu_m3_s2 = 3.5316e12f64;
    (*w).rotation_rate_rad_s = 2.0f64 * PI / 21549.425f64;
    (*w).atmosphere_top_m = 70000.0f64;
    (*w).rotation_phase_rad_at_ut0 = PI / 2.0f64;
    (*w).orbital_rate_rad_s = 2.0f64 * PI / 9203544.61750141f64;
    (*w).solar_phase_rad_at_ut0 = 3.14000010490417f64 + PI / 2.0f64;
    (*w).stock_spatial_atmosphere = true_0 != 0;
    let mut i: size_t = 0 as size_t;
    while i <= 700 as size_t {
        let mut h: ::core::ffi::c_double = 100.0f64 * i as ::core::ffi::c_double;
        let mut temp: ::core::ffi::c_double = curve_eval(
            &raw const KERBIN_TEMPERATURE_CURVE as *const CurveKey,
            (::core::mem::size_of::<[CurveKey; 9]>() as size_t)
                .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
            h,
        )
            + curve_eval(
                &raw const KERBIN_TEMPERATURE_SUN_MULT_CURVE as *const CurveKey,
                (::core::mem::size_of::<[CurveKey; 8]>() as size_t)
                    .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
                h,
            )
                * (curve_eval(
                    &raw const KERBIN_LATITUDE_BIAS_CURVE as *const CurveKey,
                    (::core::mem::size_of::<[CurveKey; 9]>() as size_t)
                        .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
                    0.0f64,
                )
                    + 0.5f64
                        * curve_eval(
                            &raw const KERBIN_LATITUDE_SUN_MULT_CURVE as *const CurveKey,
                            (::core::mem::size_of::<[CurveKey; 6]>() as size_t)
                                .wrapping_div(::core::mem::size_of::<CurveKey>() as size_t),
                            0.0f64,
                        ));
        let mut pressure: ::core::ffi::c_double = kerbin_pressure_pa(h);
        let mut s: AtmosphereSample = AtmosphereSample {
            density_kg_m3: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            pressure_pa: pressure,
            temperature_k: temp,
            speed_of_sound_mps: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        };
        if pressure > 0.0f64 {
            s.density_kg_m3 = pressure * KERBIN_MOLAR_MASS_KG_MOL
                / (UNIVERSAL_GAS_CONSTANT * temp);
        }
        s.speed_of_sound_mps = sqrt(
            KERBIN_ADIABATIC_INDEX * (UNIVERSAL_GAS_CONSTANT / KERBIN_MOLAR_MASS_KG_MOL)
                * temp,
        );
        let fresh0 = (*w).atmosphere.count;
        (*w).atmosphere.count = (*w).atmosphere.count.wrapping_add(1);
        (*w).atmosphere.p[fresh0 as usize] = AtmospherePoint {
            altitude_m: h,
            sample: s,
        };
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn world_load_atmosphere_csv(
    mut w: *mut KerbinWorld,
    mut path: *const ::core::ffi::c_char,
) -> bool {
    let mut f: *mut FILE = fopen(path, b"r\0" as *const u8 as *const ::core::ffi::c_char)
        as *mut FILE;
    if f.is_null() {
        return false_0 != 0;
    }
    let mut loaded: AtmosphereTable = AtmosphereTable {
        p: [
            AtmospherePoint {
                altitude_m: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
            AtmospherePoint {
                altitude_m: 0.,
                sample: AtmosphereSample {
                    density_kg_m3: 0.,
                    pressure_pa: 0.,
                    temperature_k: 0.,
                    speed_of_sound_mps: 0.,
                },
            },
        ],
        count: 0,
    };
    let mut line: [::core::ffi::c_char; 512] = [0; 512];
    let mut overflow: bool = false_0 != 0;
    let mut order_error: bool = false_0 != 0;
    let mut stock_spatial: bool = false_0 != 0;
    while !fgets(
            &raw mut line as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as ::core::ffi::c_int,
            f,
        )
        .is_null()
    {
        if line[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int == '#' as i32 {
            if !strstr(
                    &raw mut line as *mut ::core::ffi::c_char,
                    b"model=kerbin_stock_spatial\0" as *const u8
                        as *const ::core::ffi::c_char,
                )
                .is_null()
            {
                stock_spatial = true_0 != 0;
            }
        } else {
            if !strstr(
                    &raw mut line as *mut ::core::ffi::c_char,
                    b"altitude\0" as *const u8 as *const ::core::ffi::c_char,
                )
                .is_null()
            {
                continue;
            }
            let mut h: ::core::ffi::c_double = 0.;
            let mut rho: ::core::ffi::c_double = 0.;
            let mut p: ::core::ffi::c_double = 0.;
            let mut t: ::core::ffi::c_double = 0.;
            let mut a: ::core::ffi::c_double = 0.;
            if !(sscanf(
                &raw mut line as *mut ::core::ffi::c_char,
                b"%lf,%lf,%lf,%lf,%lf\0" as *const u8 as *const ::core::ffi::c_char,
                &raw mut h,
                &raw mut rho,
                &raw mut p,
                &raw mut t,
                &raw mut a,
            ) == 5 as ::core::ffi::c_int)
            {
                continue;
            }
            if h.is_finite() as i32 == 0 || rho.is_finite() as i32 == 0
                || p.is_finite() as i32 == 0 || t.is_finite() as i32 == 0
                || a.is_finite() as i32 == 0 || rho < 0.0f64 || p < 0.0f64
            {
                order_error = true_0 != 0;
                break;
            } else if loaded.count > 0 as size_t
                && !(h
                    > loaded
                        .p[loaded.count.wrapping_sub(1 as size_t) as usize]
                        .altitude_m)
            {
                order_error = true_0 != 0;
                break;
            } else if loaded.count >= ATM_TABLE_MAX as size_t {
                overflow = true_0 != 0;
                break;
            } else {
                loaded.p[loaded.count as usize].altitude_m = h;
                loaded.p[loaded.count as usize].sample = AtmosphereSample {
                    density_kg_m3: rho,
                    pressure_pa: p,
                    temperature_k: t,
                    speed_of_sound_mps: a,
                };
                loaded.count = loaded.count.wrapping_add(1);
            }
        }
    }
    fclose(f);
    if overflow as ::core::ffi::c_int != 0 || order_error as ::core::ffi::c_int != 0
        || loaded.count < 2 as size_t
    {
        return false_0 != 0;
    }
    (*w).atmosphere = loaded;
    (*w).stock_spatial_atmosphere = stock_spatial;
    (*w).atmosphere_top_m = (*w)
        .atmosphere
        .p[(*w).atmosphere.count.wrapping_sub(1 as size_t) as usize]
        .altitude_m;
    let mut i: size_t = 1 as size_t;
    while i < (*w).atmosphere.count {
        if (*w).atmosphere.p[i.wrapping_sub(1 as size_t) as usize].sample.density_kg_m3
            > 0.0f64 && !((*w).atmosphere.p[i as usize].sample.density_kg_m3 > 0.0f64)
        {
            (*w).atmosphere_top_m = (*w).atmosphere.p[i as usize].altitude_m;
            break;
        } else {
            i = i.wrapping_add(1);
        }
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn world_gravity_accel(
    mut w: *const KerbinWorld,
    mut r: Vec3,
) -> Vec3 {
    let mut rn: ::core::ffi::c_double = v3_norm(r);
    if rn < 1.0f64 {
        return ss_v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    }
    return v3_scale(r, -(*w).mu_m3_s2 / (rn * rn * rn));
}
unsafe extern "C" fn rotz(mut v: Vec3, mut a: ::core::ffi::c_double) -> Vec3 {
    let mut c: ::core::ffi::c_double = cos(a);
    let mut s: ::core::ffi::c_double = sin(a);
    return ss_v3(c * v.x - s * v.y, s * v.x + c * v.y, v.z);
}
#[no_mangle]
pub unsafe extern "C" fn world_fixed_to_inertial(
    mut w: *const KerbinWorld,
    mut fixed: Vec3,
    mut ut: ::core::ffi::c_double,
) -> Vec3 {
    return rotz(fixed, (*w).rotation_phase_rad_at_ut0 + (*w).rotation_rate_rad_s * ut);
}
#[no_mangle]
pub unsafe extern "C" fn world_inertial_to_fixed(
    mut w: *const KerbinWorld,
    mut inertial: Vec3,
    mut ut: ::core::ffi::c_double,
) -> Vec3 {
    return rotz(
        inertial,
        -((*w).rotation_phase_rad_at_ut0 + (*w).rotation_rate_rad_s * ut),
    );
}
#[no_mangle]
pub unsafe extern "C" fn world_atmosphere_velocity_i(
    mut w: *const KerbinWorld,
    mut p: Vec3,
) -> Vec3 {
    return ss_v3(
        -(*w).rotation_rate_rad_s * p.y,
        (*w).rotation_rate_rad_s * p.x,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
}
#[no_mangle]
pub unsafe extern "C" fn world_lla(
    mut w: *const KerbinWorld,
    mut p_i: Vec3,
    mut ut: ::core::ffi::c_double,
) -> LLA {
    let mut p: Vec3 = world_inertial_to_fixed(w, p_i, ut);
    let mut rn: ::core::ffi::c_double = v3_norm(p);
    let mut out: LLA = LLA {
        lat_rad: asin(
            ss_clampd(
                p.z / rn,
                -(1 as ::core::ffi::c_int) as ::core::ffi::c_double,
                1 as ::core::ffi::c_int as ::core::ffi::c_double,
            ),
        ),
        lon_rad: atan2(p.y, p.x),
        altitude_m: rn - (*w).radius_m,
    };
    return out;
}
#[no_mangle]
pub unsafe extern "C" fn world_lla_to_inertial(
    mut w: *const KerbinWorld,
    mut lat: ::core::ffi::c_double,
    mut lon: ::core::ffi::c_double,
    mut alt: ::core::ffi::c_double,
    mut ut: ::core::ffi::c_double,
) -> Vec3 {
    let mut r: ::core::ffi::c_double = (*w).radius_m + alt;
    let mut cl: ::core::ffi::c_double = cos(lat);
    let mut f: Vec3 = ss_v3(r * cl * cos(lon), r * cl * sin(lon), r * sin(lat));
    return world_fixed_to_inertial(w, f, ut);
}
#[no_mangle]
pub unsafe extern "C" fn world_local_frame_i(
    mut w: *const KerbinWorld,
    mut p_i: Vec3,
    mut ut: ::core::ffi::c_double,
) -> LocalFrame {
    let mut up: Vec3 = v3_normalized(p_i);
    let mut z: Vec3 = ss_v3(
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut east: Vec3 = v3_normalized(v3_cross(z, up));
    if v3_norm(east) < 1e-8f64 {
        east = ss_v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    }
    let mut north: Vec3 = v3_normalized(v3_cross(up, east));
    let mut f: LocalFrame = LocalFrame {
        north: north,
        east: east,
        up: up,
    };
    return f;
}
#[no_mangle]
pub unsafe extern "C" fn world_atmosphere_sample(
    mut w: *const KerbinWorld,
    mut h: ::core::ffi::c_double,
) -> AtmosphereSample {
    let mut z: AtmosphereSample = AtmosphereSample {
        density_kg_m3: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        pressure_pa: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        temperature_k: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
        speed_of_sound_mps: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
    };
    if h >= (*w).atmosphere_top_m || (*w).atmosphere.count < 2 as size_t {
        return z;
    }
    if h <= (*w).atmosphere.p[0 as ::core::ffi::c_int as usize].altitude_m {
        return (*w).atmosphere.p[0 as ::core::ffi::c_int as usize].sample;
    }
    let mut i: size_t = 1 as size_t;
    while i < (*w).atmosphere.count {
        if h <= (*w).atmosphere.p[i as usize].altitude_m {
            let mut a: *const AtmospherePoint = (&raw const (*w).atmosphere.p
                as *const AtmospherePoint)
                .offset(i.wrapping_sub(1 as size_t) as isize) as *const AtmospherePoint;
            let mut b: *const AtmospherePoint = (&raw const (*w).atmosphere.p
                as *const AtmospherePoint)
                .offset(i as isize) as *const AtmospherePoint;
            let mut t: ::core::ffi::c_double = (h - (*a).altitude_m)
                / ((*b).altitude_m - (*a).altitude_m);
            let mut s: AtmosphereSample = AtmosphereSample {
                density_kg_m3: 0.,
                pressure_pa: 0.,
                temperature_k: 0.,
                speed_of_sound_mps: 0.,
            };
            if (*a).sample.density_kg_m3 > 0.0f64 && (*b).sample.density_kg_m3 > 0.0f64 {
                s.density_kg_m3 = exp(
                    log((*a).sample.density_kg_m3)
                        + (log((*b).sample.density_kg_m3)
                            - log((*a).sample.density_kg_m3)) * t,
                );
            } else {
                s.density_kg_m3 = (*a).sample.density_kg_m3
                    + ((*b).sample.density_kg_m3 - (*a).sample.density_kg_m3) * t;
            }
            if (*a).sample.pressure_pa > 0.0f64 && (*b).sample.pressure_pa > 0.0f64 {
                s.pressure_pa = exp(
                    log((*a).sample.pressure_pa)
                        + (log((*b).sample.pressure_pa) - log((*a).sample.pressure_pa))
                            * t,
                );
            } else {
                s.pressure_pa = (*a).sample.pressure_pa
                    + ((*b).sample.pressure_pa - (*a).sample.pressure_pa) * t;
            }
            s.temperature_k = (*a).sample.temperature_k
                + ((*b).sample.temperature_k - (*a).sample.temperature_k) * t;
            s.speed_of_sound_mps = (*a).sample.speed_of_sound_mps
                + ((*b).sample.speed_of_sound_mps - (*a).sample.speed_of_sound_mps) * t;
            return s;
        }
        i = i.wrapping_add(1);
    }
    return z;
}
#[no_mangle]
pub unsafe extern "C" fn world_atmosphere_sample_state(
    mut w: *const KerbinWorld,
    mut position_i: Vec3,
    mut ut: ::core::ffi::c_double,
) -> AtmosphereSample {
    let mut l: LLA = world_lla(w, position_i, ut);
    if !(*w).stock_spatial_atmosphere {
        return world_atmosphere_sample(w, l.altitude_m);
    }
    return kerbin_stock_atmosphere(w, l.altitude_m, l.lat_rad, l.lon_rad, ut);
}
#[no_mangle]
pub unsafe extern "C" fn runway_seed_ksp09(mut r: *mut Runway) {
    (*r).lat_rad = deg2rad(-0.0486f64);
    (*r).lon_rad = deg2rad(-74.7240f64);
    (*r).elevation_m = 70.0f64;
    (*r).heading_rad = deg2rad(90.0f64);
    (*r).length_m = 2500.0f64;
    (*r).width_m = 70.0f64;
}
#[no_mangle]
pub unsafe extern "C" fn runway_contains(
    mut r: *const Runway,
    mut along_m: ::core::ffi::c_double,
    mut cross_m: ::core::ffi::c_double,
) -> bool {
    if r.is_null() || along_m.is_finite() as i32 == 0 || cross_m.is_finite() as i32 == 0
        || (*r).length_m.is_finite() as i32 == 0 || !((*r).length_m > 0.0f64)
        || (*r).width_m.is_finite() as i32 == 0 || !((*r).width_m > 0.0f64)
    {
        return false_0 != 0;
    }
    return along_m >= 0.0f64 && along_m <= (*r).length_m
        && fabs(cross_m) <= (*r).width_m * 0.5f64;
}
#[no_mangle]
pub unsafe extern "C" fn ss_runway_coordinates(
    mut w: *const KerbinWorld,
    mut r: *const Runway,
    mut p_i: Vec3,
    mut ut: ::core::ffi::c_double,
    mut along: *mut ::core::ffi::c_double,
    mut cross: *mut ::core::ffi::c_double,
    mut vertical: *mut ::core::ffi::c_double,
) {
    let mut p: LLA = world_lla(w, p_i, ut);
    let mut lat1: ::core::ffi::c_double = (*r).lat_rad;
    let mut lon1: ::core::ffi::c_double = (*r).lon_rad;
    let mut lat2: ::core::ffi::c_double = p.lat_rad;
    let mut lon2: ::core::ffi::c_double = p.lon_rad;
    let mut dlon: ::core::ffi::c_double = wrap_pi(lon2 - lon1);
    let mut cos_delta: ::core::ffi::c_double = sin(lat1) * sin(lat2)
        + cos(lat1) * cos(lat2) * cos(dlon);
    let mut delta: ::core::ffi::c_double = acos(ss_clampd(cos_delta, -1.0f64, 1.0f64));
    let mut bearing: ::core::ffi::c_double = atan2(
        sin(dlon) * cos(lat2),
        cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon),
    );
    let mut rel: ::core::ffi::c_double = wrap_pi(bearing - (*r).heading_rad);
    let mut xt: ::core::ffi::c_double = asin(
        ss_clampd(sin(delta) * sin(rel), -1.0f64, 1.0f64),
    );
    let mut at: ::core::ffi::c_double = atan2(sin(delta) * cos(rel), cos(delta));
    if !along.is_null() {
        *along = at * (*w).radius_m;
    }
    if !cross.is_null() {
        *cross = xt * (*w).radius_m;
    }
    if !vertical.is_null() {
        *vertical = p.altitude_m - (*r).elevation_m;
    }
}
