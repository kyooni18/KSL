use crate::sys::*;
extern "C" {
    fn sin(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn pow(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn world_atmosphere_velocity_i(w: *const KerbinWorld, position_i: Vec3) -> Vec3;
    fn world_atmosphere_sample_state(
        w: *const KerbinWorld,
        position_i: Vec3,
        ut: ::core::ffi::c_double,
    ) -> AtmosphereSample;
    fn ss_v3(
        x: ::core::ffi::c_double,
        y: ::core::ffi::c_double,
        z: ::core::ffi::c_double,
    ) -> Vec3;
    fn v3_add(a: Vec3, b: Vec3) -> Vec3;
    fn v3_sub(a: Vec3, b: Vec3) -> Vec3;
    fn v3_scale(a: Vec3, s: ::core::ffi::c_double) -> Vec3;
    fn v3_dot(a: Vec3, b: Vec3) -> ::core::ffi::c_double;
    fn v3_cross(a: Vec3, b: Vec3) -> Vec3;
    fn v3_norm(a: Vec3) -> ::core::ffi::c_double;
    fn v3_normalized(a: Vec3) -> Vec3;
    fn v3_rotate_axis(v: Vec3, axis: Vec3, angle_rad: ::core::ffi::c_double) -> Vec3;
    fn ss_clampd(
        x: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn deg2rad(x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn rad2deg(x: ::core::ffi::c_double) -> ::core::ffi::c_double;
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
pub struct AtmosphereSample {
    pub density_kg_m3: ::core::ffi::c_double,
    pub pressure_pa: ::core::ffi::c_double,
    pub temperature_k: ::core::ffi::c_double,
    pub speed_of_sound_mps: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroForces {
    pub lift_n: ::core::ffi::c_double,
    pub drag_n: ::core::ffi::c_double,
    pub side_n: ::core::ffi::c_double,
    pub force_i: Vec3,
    pub mach: ::core::ffi::c_double,
    pub dynamic_pressure_pa: ::core::ffi::c_double,
    pub airspeed_mps: ::core::ffi::c_double,
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
pub struct AeroBookPoint {
    pub q_pa: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub alpha_deg: ::core::ffi::c_double,
    pub lift_per_q_m2: ::core::ffi::c_double,
    pub drag_per_q_m2: ::core::ffi::c_double,
    pub support: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AeroTable {
    pub mach: [::core::ffi::c_double; 40],
    pub alpha_deg: [::core::ffi::c_double; 40],
    pub cl: [[::core::ffi::c_double; 40]; 40],
    pub cd: [[::core::ffi::c_double; 40]; 40],
    pub mach_count: size_t,
    pub alpha_count: size_t,
    pub reference_area_m2: ::core::ffi::c_double,
    pub book: [AeroBookPoint; 2048],
    pub book_count: size_t,
    pub book_enabled: bool,
    pub lift_scale: ::core::ffi::c_double,
    pub drag_scale: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Row {
    pub m: ::core::ffi::c_double,
    pub al: ::core::ffi::c_double,
    pub cl: ::core::ffi::c_double,
    pub cd: ::core::ffi::c_double,
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const AERO_MACH_MAX: ::core::ffi::c_int = 40 as ::core::ffi::c_int;
pub const AERO_ALPHA_MAX: ::core::ffi::c_int = 40 as ::core::ffi::c_int;
pub const AERO_BOOK_MAX: ::core::ffi::c_int = 2048 as ::core::ffi::c_int;
unsafe extern "C" fn lower_index(
    mut a: *const ::core::ffi::c_double,
    mut n: size_t,
    mut x: ::core::ffi::c_double,
) -> size_t {
    if n < 2 as size_t {
        return 0 as size_t;
    }
    if x <= *a.offset(0 as ::core::ffi::c_int as isize) {
        return 0 as size_t;
    }
    let mut i: size_t = 1 as size_t;
    while i < n {
        if x <= *a.offset(i as isize) {
            return i.wrapping_sub(1 as size_t);
        }
        i = i.wrapping_add(1);
    }
    return n.wrapping_sub(2 as size_t);
}
#[no_mangle]
pub unsafe extern "C" fn aero_seed_stsn(mut a: *mut AeroTable) {
    memset(
        a as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<AeroTable>() as size_t,
    );
    let m: [::core::ffi::c_double; 8] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.8f64,
        1.2f64,
        2 as ::core::ffi::c_int as ::core::ffi::c_double,
        5 as ::core::ffi::c_int as ::core::ffi::c_double,
        10 as ::core::ffi::c_int as ::core::ffi::c_double,
        15 as ::core::ffi::c_int as ::core::ffi::c_double,
        25 as ::core::ffi::c_int as ::core::ffi::c_double,
    ];
    let al: [::core::ffi::c_double; 11] = [
        -(5 as ::core::ffi::c_int) as ::core::ffi::c_double,
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        5 as ::core::ffi::c_int as ::core::ffi::c_double,
        10 as ::core::ffi::c_int as ::core::ffi::c_double,
        15 as ::core::ffi::c_int as ::core::ffi::c_double,
        20 as ::core::ffi::c_int as ::core::ffi::c_double,
        25 as ::core::ffi::c_int as ::core::ffi::c_double,
        30 as ::core::ffi::c_int as ::core::ffi::c_double,
        35 as ::core::ffi::c_int as ::core::ffi::c_double,
        40 as ::core::ffi::c_int as ::core::ffi::c_double,
        45 as ::core::ffi::c_int as ::core::ffi::c_double,
    ];
    (*a).mach_count = (::core::mem::size_of::<[::core::ffi::c_double; 8]>() as usize)
        .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
        as size_t;
    (*a).alpha_count = (::core::mem::size_of::<[::core::ffi::c_double; 11]>() as usize)
        .wrapping_div(::core::mem::size_of::<::core::ffi::c_double>() as usize)
        as size_t;
    memcpy(
        &raw mut (*a).mach as *mut ::core::ffi::c_double as *mut ::core::ffi::c_void,
        &raw const m as *const ::core::ffi::c_double as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_double; 8]>() as size_t,
    );
    memcpy(
        &raw mut (*a).alpha_deg as *mut ::core::ffi::c_double
            as *mut ::core::ffi::c_void,
        &raw const al as *const ::core::ffi::c_double as *const ::core::ffi::c_void,
        ::core::mem::size_of::<[::core::ffi::c_double; 11]>() as size_t,
    );
    (*a).reference_area_m2 = 250.0f64;
    let mut i: size_t = 0 as size_t;
    while i < (*a).mach_count {
        let mut j: size_t = 0 as size_t;
        while j < (*a).alpha_count {
            let mut ar: ::core::ffi::c_double = deg2rad(al[j as usize]);
            let mut hyp: ::core::ffi::c_double = if m[i as usize] >= 5.0f64 {
                1.0f64
            } else {
                0.82f64 + 0.036f64 * m[i as usize]
            };
            let mut cl: ::core::ffi::c_double = hyp * (1.15f64 * sin(2.0f64 * ar));
            let mut cd0: ::core::ffi::c_double = if m[i as usize] < 0.8f64 {
                0.16f64
            } else if m[i as usize] < 1.2f64 {
                0.28f64
            } else if m[i as usize] < 3 as ::core::ffi::c_int as ::core::ffi::c_double {
                0.34f64
            } else {
                0.42f64
            };
            let mut cd: ::core::ffi::c_double = cd0 + hyp * 1.55f64 * sin(ar) * sin(ar);
            if al[j as usize] < 0 as ::core::ffi::c_int as ::core::ffi::c_double {
                cl *= 0.8f64;
            }
            (*a).cl[i as usize][j as usize] = cl;
            (*a).cd[i as usize][j as usize] = cd;
            j = j.wrapping_add(1);
        }
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn aero_load_csv(
    mut a: *mut AeroTable,
    mut path: *const ::core::ffi::c_char,
) -> bool {
    let mut f: *mut FILE = fopen(path, b"r\0" as *const u8 as *const ::core::ffi::c_char)
        as *mut FILE;
    if f.is_null() {
        return false_0 != 0;
    }
    let mut rows: [Row; 1600] = [Row {
        m: 0.,
        al: 0.,
        cl: 0.,
        cd: 0.,
    }; 1600];
    let mut nr: size_t = 0 as size_t;
    let mut ms: [::core::ffi::c_double; 40] = [0.; 40];
    let mut as_0: [::core::ffi::c_double; 40] = [0.; 40];
    let mut nm: size_t = 0 as size_t;
    let mut na: size_t = 0 as size_t;
    let mut line: [::core::ffi::c_char; 512] = [0; 512];
    while !fgets(
            &raw mut line as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as ::core::ffi::c_int,
            f,
        )
        .is_null() && nr < 1600 as size_t
    {
        if line[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int == '#' as i32
            || !strstr(
                    &raw mut line as *mut ::core::ffi::c_char,
                    b"mach\0" as *const u8 as *const ::core::ffi::c_char,
                )
                .is_null()
        {
            continue;
        }
        let mut r: Row = Row {
            m: 0.,
            al: 0.,
            cl: 0.,
            cd: 0.,
        };
        if sscanf(
            &raw mut line as *mut ::core::ffi::c_char,
            b"%lf,%lf,%lf,%lf\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut r.m,
            &raw mut r.al,
            &raw mut r.cl,
            &raw mut r.cd,
        ) == 4 as ::core::ffi::c_int
        {
            let fresh0 = nr;
            nr = nr.wrapping_add(1);
            rows[fresh0 as usize] = r;
            let mut found: bool = false_0 != 0;
            let mut i: size_t = 0 as size_t;
            while i < nm {
                if fabs(ms[i as usize] - r.m) < 1e-9f64 {
                    found = true_0 != 0;
                }
                i = i.wrapping_add(1);
            }
            if !found && nm < AERO_MACH_MAX as size_t {
                let fresh1 = nm;
                nm = nm.wrapping_add(1);
                ms[fresh1 as usize] = r.m;
            }
            found = false_0 != 0;
            let mut i_0: size_t = 0 as size_t;
            while i_0 < na {
                if fabs(as_0[i_0 as usize] - r.al) < 1e-9f64 {
                    found = true_0 != 0;
                }
                i_0 = i_0.wrapping_add(1);
            }
            if !found && na < AERO_ALPHA_MAX as size_t {
                let fresh2 = na;
                na = na.wrapping_add(1);
                as_0[fresh2 as usize] = r.al;
            }
        }
    }
    fclose(f);
    if nm < 2 as size_t || na < 2 as size_t {
        return false_0 != 0;
    }
    let mut i_1: size_t = 0 as size_t;
    while i_1 < nm {
        let mut j: size_t = i_1.wrapping_add(1 as size_t);
        while j < nm {
            if ms[j as usize] < ms[i_1 as usize] {
                let mut t: ::core::ffi::c_double = ms[i_1 as usize];
                ms[i_1 as usize] = ms[j as usize];
                ms[j as usize] = t;
            }
            j = j.wrapping_add(1);
        }
        i_1 = i_1.wrapping_add(1);
    }
    let mut i_2: size_t = 0 as size_t;
    while i_2 < na {
        let mut j_0: size_t = i_2.wrapping_add(1 as size_t);
        while j_0 < na {
            if as_0[j_0 as usize] < as_0[i_2 as usize] {
                let mut t_0: ::core::ffi::c_double = as_0[i_2 as usize];
                as_0[i_2 as usize] = as_0[j_0 as usize];
                as_0[j_0 as usize] = t_0;
            }
            j_0 = j_0.wrapping_add(1);
        }
        i_2 = i_2.wrapping_add(1);
    }
    memset(
        a as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<AeroTable>() as size_t,
    );
    (*a).mach_count = nm;
    (*a).alpha_count = na;
    (*a).reference_area_m2 = 250.0f64;
    memcpy(
        &raw mut (*a).mach as *mut ::core::ffi::c_double as *mut ::core::ffi::c_void,
        &raw mut ms as *mut ::core::ffi::c_double as *const ::core::ffi::c_void,
        nm.wrapping_mul(::core::mem::size_of::<::core::ffi::c_double>() as size_t),
    );
    memcpy(
        &raw mut (*a).alpha_deg as *mut ::core::ffi::c_double
            as *mut ::core::ffi::c_void,
        &raw mut as_0 as *mut ::core::ffi::c_double as *const ::core::ffi::c_void,
        na.wrapping_mul(::core::mem::size_of::<::core::ffi::c_double>() as size_t),
    );
    let mut seen: [[bool; 40]; 40] = [
        [
            false_0 != 0,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
        ],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
        [false; 40],
    ];
    let mut r_0: size_t = 0 as size_t;
    while r_0 < nr {
        let mut im: size_t = 0 as size_t;
        let mut ia: size_t = 0 as size_t;
        let mut i_3: size_t = 0 as size_t;
        while i_3 < nm {
            if fabs(ms[i_3 as usize] - rows[r_0 as usize].m) < 1e-9f64 {
                im = i_3;
            }
            i_3 = i_3.wrapping_add(1);
        }
        let mut j_1: size_t = 0 as size_t;
        while j_1 < na {
            if fabs(as_0[j_1 as usize] - rows[r_0 as usize].al) < 1e-9f64 {
                ia = j_1;
            }
            j_1 = j_1.wrapping_add(1);
        }
        (*a).cl[im as usize][ia as usize] = rows[r_0 as usize].cl;
        (*a).cd[im as usize][ia as usize] = rows[r_0 as usize].cd;
        seen[im as usize][ia as usize] = true_0 != 0;
        r_0 = r_0.wrapping_add(1);
    }
    let mut i_4: size_t = 0 as size_t;
    while i_4 < nm {
        let mut j_2: size_t = 0 as size_t;
        while j_2 < na {
            if !seen[i_4 as usize][j_2 as usize] {
                return false_0 != 0;
            }
            j_2 = j_2.wrapping_add(1);
        }
        i_4 = i_4.wrapping_add(1);
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn aero_load_book_csv(
    mut a: *mut AeroTable,
    mut path: *const ::core::ffi::c_char,
) -> bool {
    let mut f: *mut FILE = fopen(path, b"r\0" as *const u8 as *const ::core::ffi::c_char)
        as *mut FILE;
    if f.is_null() {
        return false_0 != 0;
    }
    let mut line: [::core::ffi::c_char; 512] = [0; 512];
    let mut n: size_t = 0 as size_t;
    let mut overflow: bool = false_0 != 0;
    while !fgets(
            &raw mut line as *mut ::core::ffi::c_char,
            ::core::mem::size_of::<[::core::ffi::c_char; 512]>() as ::core::ffi::c_int,
            f,
        )
        .is_null()
    {
        if line[0 as ::core::ffi::c_int as usize] as ::core::ffi::c_int == '#' as i32
            || !strstr(
                    &raw mut line as *mut ::core::ffi::c_char,
                    b"q_pa\0" as *const u8 as *const ::core::ffi::c_char,
                )
                .is_null()
        {
            continue;
        }
        let mut p: AeroBookPoint = AeroBookPoint {
            q_pa: 0 as ::core::ffi::c_int as ::core::ffi::c_double,
            mach: 0.,
            alpha_deg: 0.,
            lift_per_q_m2: 0.,
            drag_per_q_m2: 0.,
            support: 0.,
        };
        let mut got: ::core::ffi::c_int = sscanf(
            &raw mut line as *mut ::core::ffi::c_char,
            b"%lf,%lf,%lf,%lf,%lf,%lf\0" as *const u8 as *const ::core::ffi::c_char,
            &raw mut p.q_pa,
            &raw mut p.mach,
            &raw mut p.alpha_deg,
            &raw mut p.lift_per_q_m2,
            &raw mut p.drag_per_q_m2,
            &raw mut p.support,
        );
        if got >= 5 as ::core::ffi::c_int
            && p.q_pa > 0 as ::core::ffi::c_int as ::core::ffi::c_double
            && p.q_pa.is_finite() as i32 != 0 && p.mach.is_finite() as i32 != 0
            && p.alpha_deg.is_finite() as i32 != 0
            && p.lift_per_q_m2.is_finite() as i32 != 0
            && p.drag_per_q_m2.is_finite() as i32 != 0
        {
            if got < 6 as ::core::ffi::c_int {
                p.support = 1.0f64;
            }
            if n < AERO_BOOK_MAX as size_t {
                let fresh3 = n;
                n = n.wrapping_add(1);
                (*a).book[fresh3 as usize] = p;
            } else {
                overflow = true_0 != 0;
            }
        }
    }
    fclose(f);
    if overflow {
        (*a).book_count = 0 as size_t;
        (*a).book_enabled = false_0 != 0;
        return false_0 != 0;
    }
    (*a).book_count = n;
    (*a).book_enabled = n > 0 as size_t;
    return n > 0 as size_t;
}
unsafe extern "C" fn aero_book_lookup(
    mut a: *const AeroTable,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut alpha: ::core::ffi::c_double,
    mut lift_per_q: *mut ::core::ffi::c_double,
    mut drag_per_q: *mut ::core::ffi::c_double,
    mut coverage_out: *mut ::core::ffi::c_double,
) -> bool {
    if !coverage_out.is_null() {
        *coverage_out = 0.0f64;
    }
    if !(*a).book_enabled || (*a).book_count == 0 as size_t || q < 1.0f64 {
        return false_0 != 0;
    }
    let log2v: ::core::ffi::c_double = 0.69314718055994530942f64;
    let mut best_d2: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut second_d2: ::core::ffi::c_double = ::core::f32::INFINITY
        as ::core::ffi::c_double;
    let mut best: *const AeroBookPoint = ::core::ptr::null::<AeroBookPoint>();
    let mut second: *const AeroBookPoint = ::core::ptr::null::<AeroBookPoint>();
    let mut i: size_t = 0 as size_t;
    while i < (*a).book_count {
        let mut p: *const AeroBookPoint = (&raw const (*a).book as *const AeroBookPoint)
            .offset(i as isize) as *const AeroBookPoint;
        let mut dq: ::core::ffi::c_double = log(q / (*p).q_pa) / log2v / 1.0f64;
        let mut dm: ::core::ffi::c_double = (mach - (*p).mach) / 0.5f64;
        let mut da: ::core::ffi::c_double = (alpha - (*p).alpha_deg) / 3.0f64;
        let mut d2: ::core::ffi::c_double = dq * dq + dm * dm + da * da;
        if d2 < best_d2 {
            second_d2 = best_d2;
            second = best;
            best_d2 = d2;
            best = p;
        } else if d2 < second_d2 {
            second_d2 = d2;
            second = p;
        }
        i = i.wrapping_add(1);
    }
    if best.is_null() {
        return false_0 != 0;
    }
    if second.is_null() {
        second = best;
        second_d2 = best_d2;
    }
    let mut w0: ::core::ffi::c_double = pow(fmax(1.0f64, (*best).support), 0.25f64)
        / (0.02f64 + best_d2);
    let mut w1: ::core::ffi::c_double = pow(fmax(1.0f64, (*second).support), 0.25f64)
        / (0.02f64 + second_d2);
    let mut sw: ::core::ffi::c_double = w0 + w1;
    if !(sw > 0.0f64) || sw.is_finite() as i32 == 0 {
        return false_0 != 0;
    }
    *lift_per_q = (w0 * (*best).lift_per_q_m2 + w1 * (*second).lift_per_q_m2) / sw;
    *drag_per_q = (w0 * (*best).drag_per_q_m2 + w1 * (*second).drag_per_q_m2) / sw;
    let mut nearest: ::core::ffi::c_double = sqrt(best_d2);
    let mut coverage: ::core::ffi::c_double = ss_clampd(
        (1.5f64 - nearest) / 0.75f64,
        0.0f64,
        1.0f64,
    );
    if !coverage_out.is_null() {
        *coverage_out = coverage;
    }
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn aero_coefficients(
    mut a: *const AeroTable,
    mut mach: ::core::ffi::c_double,
    mut alpha: ::core::ffi::c_double,
    mut cl: *mut ::core::ffi::c_double,
    mut cd: *mut ::core::ffi::c_double,
) {
    let mut i: size_t = lower_index(
        &raw const (*a).mach as *const ::core::ffi::c_double,
        (*a).mach_count,
        mach,
    );
    let mut j: size_t = lower_index(
        &raw const (*a).alpha_deg as *const ::core::ffi::c_double,
        (*a).alpha_count,
        alpha,
    );
    let mut m0: ::core::ffi::c_double = (*a).mach[i as usize];
    let mut m1: ::core::ffi::c_double = (*a).mach[i.wrapping_add(1 as size_t) as usize];
    let mut a0: ::core::ffi::c_double = (*a).alpha_deg[j as usize];
    let mut a1: ::core::ffi::c_double = (*a)
        .alpha_deg[j.wrapping_add(1 as size_t) as usize];
    let mut tm: ::core::ffi::c_double = ss_clampd(
        (mach - m0) / (m1 - m0),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut ta: ::core::ffi::c_double = ss_clampd(
        (alpha - a0) / (a1 - a0),
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        1 as ::core::ffi::c_int as ::core::ffi::c_double,
    );
    let mut cl0: ::core::ffi::c_double = (*a).cl[i as usize][j as usize]
        * (1 as ::core::ffi::c_int as ::core::ffi::c_double - ta)
        + (*a).cl[i as usize][j.wrapping_add(1 as size_t) as usize] * ta;
    let mut cl1: ::core::ffi::c_double = (*a)
        .cl[i.wrapping_add(1 as size_t) as usize][j as usize]
        * (1 as ::core::ffi::c_int as ::core::ffi::c_double - ta)
        + (*a)
            .cl[i.wrapping_add(1 as size_t)
            as usize][j.wrapping_add(1 as size_t) as usize] * ta;
    let mut cd0: ::core::ffi::c_double = (*a).cd[i as usize][j as usize]
        * (1 as ::core::ffi::c_int as ::core::ffi::c_double - ta)
        + (*a).cd[i as usize][j.wrapping_add(1 as size_t) as usize] * ta;
    let mut cd1: ::core::ffi::c_double = (*a)
        .cd[i.wrapping_add(1 as size_t) as usize][j as usize]
        * (1 as ::core::ffi::c_int as ::core::ffi::c_double - ta)
        + (*a)
            .cd[i.wrapping_add(1 as size_t)
            as usize][j.wrapping_add(1 as size_t) as usize] * ta;
    *cl = cl0 * (1 as ::core::ffi::c_int as ::core::ffi::c_double - tm) + cl1 * tm;
    *cd = cd0 * (1 as ::core::ffi::c_int as ::core::ffi::c_double - tm) + cd1 * tm;
}
#[no_mangle]
pub unsafe extern "C" fn aero_observation_mach_weight(
    mut mach: ::core::ffi::c_double,
    mut observed_mach: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    if mach.is_finite() as i32 == 0 || observed_mach.is_finite() as i32 == 0 {
        return 0.0f64;
    }
    return fmax(0.0f64, 1.0f64 - fabs(mach - observed_mach) / 0.5f64);
}
#[no_mangle]
pub unsafe extern "C" fn aero_apply_mach_observation(
    mut a: *mut AeroTable,
    mut observed_mach: ::core::ffi::c_double,
    mut lift_scale: ::core::ffi::c_double,
    mut drag_scale: ::core::ffi::c_double,
) {
    if a.is_null() || lift_scale.is_finite() as i32 == 0
        || drag_scale.is_finite() as i32 == 0
    {
        return;
    }
    if !(fabs(lift_scale - 1.0f64) > 1e-9f64) && !(fabs(drag_scale - 1.0f64) > 1e-9f64) {
        return;
    }
    let mut i: size_t = 0 as size_t;
    while i < (*a).mach_count {
        let mut weight: ::core::ffi::c_double = aero_observation_mach_weight(
            (*a).mach[i as usize],
            observed_mach,
        );
        let mut ls: ::core::ffi::c_double = 1.0f64 + weight * (lift_scale - 1.0f64);
        let mut ds: ::core::ffi::c_double = 1.0f64 + weight * (drag_scale - 1.0f64);
        let mut j: size_t = 0 as size_t;
        while j < (*a).alpha_count {
            (*a).cl[i as usize][j as usize] *= ls;
            (*a).cd[i as usize][j as usize] *= ds;
            j = j.wrapping_add(1);
        }
        i = i.wrapping_add(1);
    }
    let mut i_0: size_t = 0 as size_t;
    while i_0 < (*a).book_count {
        let mut weight_0: ::core::ffi::c_double = aero_observation_mach_weight(
            (*a).book[i_0 as usize].mach,
            observed_mach,
        );
        (*a).book[i_0 as usize].lift_per_q_m2
            *= 1.0f64 + weight_0 * (lift_scale - 1.0f64);
        (*a).book[i_0 as usize].drag_per_q_m2
            *= 1.0f64 + weight_0 * (drag_scale - 1.0f64);
        i_0 = i_0.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn aero_force_per_q(
    mut a: *const AeroTable,
    mut q: ::core::ffi::c_double,
    mut mach: ::core::ffi::c_double,
    mut alpha_deg: ::core::ffi::c_double,
    mut lift_per_q_out: *mut ::core::ffi::c_double,
    mut drag_per_q_out: *mut ::core::ffi::c_double,
) {
    let mut cl: ::core::ffi::c_double = 0.;
    let mut cd: ::core::ffi::c_double = 0.;
    aero_coefficients(a, mach, alpha_deg, &raw mut cl, &raw mut cd);
    let mut fallback_lift_per_q: ::core::ffi::c_double = (*a).reference_area_m2 * cl;
    let mut fallback_drag_per_q: ::core::ffi::c_double = (*a).reference_area_m2 * cd;
    let mut lift_per_q: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut drag_per_q: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    let mut book_coverage: ::core::ffi::c_double = 0 as ::core::ffi::c_int
        as ::core::ffi::c_double;
    if aero_book_lookup(
        a,
        q,
        mach,
        alpha_deg,
        &raw mut lift_per_q,
        &raw mut drag_per_q,
        &raw mut book_coverage,
    ) {
        lift_per_q = fallback_lift_per_q
            + book_coverage * (lift_per_q - fallback_lift_per_q);
        drag_per_q = fallback_drag_per_q
            + book_coverage * (drag_per_q - fallback_drag_per_q);
    } else {
        lift_per_q = fallback_lift_per_q;
        drag_per_q = fallback_drag_per_q;
    }
    if (*a).lift_scale > 0.0f64 {
        lift_per_q *= (*a).lift_scale;
    }
    if (*a).drag_scale > 0.0f64 {
        drag_per_q *= (*a).drag_scale;
    }
    *lift_per_q_out = lift_per_q;
    *drag_per_q_out = drag_per_q;
}
#[no_mangle]
pub unsafe extern "C" fn aero_compute(
    mut w: *const KerbinWorld,
    mut a: *const AeroTable,
    mut p: Vec3,
    mut v: Vec3,
    mut ut: ::core::ffi::c_double,
    mut mass: ::core::ffi::c_double,
    mut aoa: ::core::ffi::c_double,
    mut bank: ::core::ffi::c_double,
) -> AeroForces {
    let mut out: AeroForces = AeroForces {
        lift_n: 0.,
        drag_n: 0.,
        side_n: 0.,
        force_i: Vec3 { x: 0., y: 0., z: 0. },
        mach: 0.,
        dynamic_pressure_pa: 0.,
        airspeed_mps: 0.,
    };
    memset(
        &raw mut out as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<AeroForces>() as size_t,
    );
    let mut atm: AtmosphereSample = world_atmosphere_sample_state(w, p, ut);
    let mut vair: Vec3 = v3_sub(v, world_atmosphere_velocity_i(w, p));
    let mut speed: ::core::ffi::c_double = v3_norm(vair);
    out.airspeed_mps = speed;
    if speed < 1e-6f64
        || atm.density_kg_m3 <= 0 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        return out;
    }
    out.mach = if atm.speed_of_sound_mps
        > 1 as ::core::ffi::c_int as ::core::ffi::c_double
    {
        speed / atm.speed_of_sound_mps
    } else {
        0 as ::core::ffi::c_int as ::core::ffi::c_double
    };
    out.dynamic_pressure_pa = 0.5f64 * atm.density_kg_m3 * speed * speed;
    let mut lift_per_q: ::core::ffi::c_double = 0.;
    let mut drag_per_q: ::core::ffi::c_double = 0.;
    aero_force_per_q(
        a,
        out.dynamic_pressure_pa,
        out.mach,
        rad2deg(aoa),
        &raw mut lift_per_q,
        &raw mut drag_per_q,
    );
    out.lift_n = out.dynamic_pressure_pa * lift_per_q;
    out.drag_n = out.dynamic_pressure_pa * drag_per_q;
    let mut fwd: Vec3 = v3_normalized(vair);
    let mut up: Vec3 = v3_normalized(p);
    let mut right: Vec3 = v3_normalized(v3_cross(fwd, up));
    if v3_norm(right) < 1e-8f64 {
        right = ss_v3(
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
            1 as ::core::ffi::c_int as ::core::ffi::c_double,
            0 as ::core::ffi::c_int as ::core::ffi::c_double,
        );
    }
    let mut lift0: Vec3 = v3_normalized(v3_cross(right, fwd));
    if v3_dot(lift0, up) < 0 as ::core::ffi::c_int as ::core::ffi::c_double {
        lift0 = v3_scale(lift0, -(1 as ::core::ffi::c_int) as ::core::ffi::c_double);
    }
    let mut lift: Vec3 = v3_rotate_axis(lift0, fwd, bank);
    let mut drag: Vec3 = v3_scale(fwd, -out.drag_n);
    out.force_i = v3_add(drag, v3_scale(lift, out.lift_n));
    return out;
}
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
