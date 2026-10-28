#[repr(C)]
pub struct Input {
    pub rotation: [f64; 4],
    pub angular_velocity: [f64; 3],
    pub forward: [f64; 3],
    pub roof: [f64; 3],
    pub has_roof: bool,
    pub torque: [f64; 3],
    pub moi: [f64; 3],
    pub brake: [f64; 3],
    pub max_rate: [f64; 3],
    pub tau: [f64; 3],
    pub integral: [f64; 3],
    pub disturbance: [f64; 3],
    pub translation: [f64; 3],
    pub rate_tau: f64,
    pub int_limit: f64,
    pub int_time: f64,
    pub dt: f64,
    pub roll_authority: f64,
    pub roll_limit: f64,
    pub roll_start: f64,
    pub roll_full: f64,
    pub roll_min: f64,
    pub deadband: f64,
    pub previous_roll_error: f64,
}
#[repr(C)]
pub struct Output {
    pub inputs: [f64; 3],
    pub integral: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub error_deg: f64,
    pub roll_error_deg: f64,
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}
fn unit(a: [f64; 3]) -> [f64; 3] {
    let n = norm(a);
    if n <= 1e-12 {
        [0.; 3]
    } else {
        a.map(|x| x / n)
    }
}
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let qv = [q[0], q[1], q[2]];
    let t = cross(qv, v).map(|x| 2. * x);
    let c = cross(qv, t);
    std::array::from_fn(|i| v[i] + q[3] * t[i] + c[i])
}
fn step(p: &Input) -> Output {
    let q = [
        -p.rotation[0],
        -p.rotation[1],
        -p.rotation[2],
        p.rotation[3],
    ];
    let w = rotate(q, p.angular_velocity);
    let f = unit(rotate(q, p.forward));
    let nose = [0., 1., 0.];
    let axis = cross(nose, f);
    let s = norm(axis);
    let angle = s.atan2(dot(nose, f));
    let mut phi = if s > 1e-9 {
        axis.map(|x| x / s * angle)
    } else if angle > 1. {
        [angle, 0., 0.]
    } else {
        [0.; 3]
    };
    let error = angle.to_degrees();
    let mut roll_error = p.previous_roll_error;
    if p.has_roof {
        let mut u = rotate(q, p.roof);
        u[1] = 0.;
        if norm(u) > 1e-6 {
            let cur = [0., 0., -1.];
            let roll = dot(nose, cross(cur, u)).atan2(dot(cur, unit(u)));
            let roll_on = ((p.roll_start - error) / (p.roll_start - p.roll_full).max(1e-3))
                .min(1.)
                .max(0.)
                .max(p.roll_min);
            phi[1] = roll * roll_on;
            roll_error = roll.to_degrees();
        }
    }
    let mut inputs = [0.; 3];
    let mut integral = p.integral;
    for i in 0..3 {
        let alpha = p.torque[i] / p.moi[i];
        let brake = (2. * p.brake[i] * alpha * phi[i].abs()).sqrt();
        let desired = p.max_rate[i]
            .min(phi[i].abs() / p.tau[i])
            .min(brake)
            .copysign(phi[i]);
        let err = desired - w[i];
        let available = p.torque[i] * if i == 1 { p.roll_authority } else { 1. };
        let cmd = p.moi[i] * err / p.rate_tau + integral[i] - p.disturbance[i] - p.translation[i];
        let mut u = -cmd / available;
        let limit = if i == 1 { p.roll_limit } else { 1. };
        if u.abs() < limit || (u > 0.) != (-err > 0.) {
            let lim = p.int_limit * available;
            integral[i] = (integral[i] + p.moi[i] * err / p.int_time * p.dt)
                .min(lim)
                .max(-lim);
        }
        u = u.min(limit).max(-limit);
        inputs[i] = if u.abs() < p.deadband { 0. } else { u };
    }
    Output {
        inputs,
        integral,
        angular_velocity: w,
        error_deg: error,
        roll_error_deg: roll_error,
    }
}
#[no_mangle]
pub unsafe extern "C" fn ksl_attitude_step(input: *const Input, output: *mut Output) {
    *output = step(&*input);
}
