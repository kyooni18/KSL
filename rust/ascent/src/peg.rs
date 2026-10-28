//! Planar powered explicit guidance, advanced using simulation time.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PegState {
    pub mu: f64,
    pub freeze_tgo: f64,
    pub a: f64,
    pub b: f64,
    pub tgo: f64,
    pub last_t: f64,
    pub hold: f64,
    pub peak: f64,
    pub has_solution: bool,
    pub has_last_t: bool,
    pub has_hold: bool,
    pub converged: bool,
}
impl PegState {
    pub fn new(mu: f64, freeze_tgo: f64) -> Self {
        Self {
            mu,
            freeze_tgo,
            a: 0.,
            b: 0.,
            tgo: 0.,
            last_t: 0.,
            hold: 0.,
            peak: 0.,
            has_solution: false,
            has_last_t: false,
            has_hold: false,
            converged: false,
        }
    }
    pub fn update(
        &mut self,
        t: f64,
        r: f64,
        vr: f64,
        vh: f64,
        accel: f64,
        ve: f64,
        rt: f64,
        vrt: f64,
        vht: f64,
        constant: bool,
    ) -> (Option<f64>, f64) {
        let accel = accel.max(0.05);
        let ve = ve.max(50.);
        let tau = ve / accel;
        if self.has_last_t && self.has_solution {
            let dt = (t - self.last_t).max(0.);
            self.a += self.b * dt;
            self.tgo = (self.tgo - dt).max(0.);
        }
        self.last_t = t;
        self.has_last_t = true;
        if !self.has_solution {
            let dv = (vht - vh).hypot(vrt - vr);
            self.tgo = if constant {
                dv / accel
            } else {
                tau * (1. - (-dv.min(5. * ve) / ve).exp())
            };
        }
        let mut time = self.tgo.max(0.1);
        let c = (self.mu / (r * r) - vh * vh / r) / accel;
        let need = (rt * vht - r * vh) / ((r + rt) / 2.);
        let mut peak: f64 = 0.;
        let time_cap = if constant { 5000. } else { 0.98 * tau };
        for _ in 0..25 {
            if time > self.freeze_tgo || !self.has_solution {
                let (b0, b1, c0, c1) = integrals(time, accel, ve, constant);
                let det = b0 * c1 - b1 * c0;
                if det.abs() < 1e-9 {
                    break;
                }
                let k1 = vrt - vr;
                let k2 = rt - r - vr * time;
                self.a = (c1 * k1 - b1 * k2) / det;
                self.b = (b0 * k2 - c0 * k1) / det;
                self.has_solution = true;
            }
            let at = if constant {
                accel
            } else {
                accel / (1. - time.min(0.999 * tau) / tau)
            };
            let ct = (self.mu / (rt * rt) - vht * vht / rt) / at;
            let mut gain = 0.;
            peak = 0.;
            let n = 16;
            for i in 0..=n {
                let x = time * i as f64 / n as f64;
                let ax = if constant {
                    accel
                } else {
                    accel / (1. - x / tau)
                };
                let fr = self.a + self.b * x + c + (ct - c) * x / time;
                peak = peak.max(fr.abs());
                let fr = fr.max(-1.).min(1.);
                let w = if i == 0 || i == n {
                    1.
                } else if i % 2 != 0 {
                    4.
                } else {
                    2.
                };
                gain += w * ax * (1. - fr * fr).sqrt();
            }
            gain *= time / (3. * n as f64);
            if need <= 0. {
                time = 0.;
                break;
            }
            let ratio = (need / gain.max(1e-6)).max(0.7).min(1.4);
            let next = (time * ratio).min(time_cap);
            let done = (next - time).abs() < 0.02;
            time = next;
            if done {
                break;
            }
        }
        if !self.has_solution {
            return (None, time);
        }
        self.peak = if time > 0. { peak } else { 0. };
        self.converged = time.is_finite() && time < time_cap && self.peak < 0.95;
        self.tgo = time;
        let pitch = (self.a + c).max(-1.).min(1.).asin().to_degrees();
        if time <= self.freeze_tgo {
            if !self.has_hold {
                self.hold = pitch;
                self.has_hold = true;
            }
            return (Some(self.hold), time);
        }
        self.has_hold = false;
        (Some(pitch), time)
    }
}
fn integrals(mut time: f64, accel: f64, ve: f64, constant: bool) -> (f64, f64, f64, f64) {
    if constant {
        return (
            accel * time,
            accel * time * time / 2.,
            accel * time * time / 2.,
            accel * time.powi(3) / 6.,
        );
    }
    let tau = ve / accel;
    time = time.min(0.999 * tau);
    let b0 = -ve * (1. - time / tau).ln();
    let b1 = b0 * tau - ve * time;
    let c0 = b0 * time - b1;
    let c1 = c0 * tau - ve * time * time / 2.;
    (b0, b1, c0, c1)
}
#[no_mangle]
pub unsafe extern "C" fn ksl_peg_update(
    state: *mut PegState,
    t: f64,
    r: f64,
    vr: f64,
    vh: f64,
    accel: f64,
    ve: f64,
    rt: f64,
    vrt: f64,
    vht: f64,
    constant: bool,
    pitch: *mut f64,
    time: *mut f64,
) -> bool {
    let (cmd, tgo) = (*state).update(t, r, vr, vh, accel, ve, rt, vrt, vht, constant);
    *time = tgo;
    if let Some(value) = cmd {
        *pitch = value;
        true
    } else {
        false
    }
}
