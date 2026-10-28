//! Vertex search for throttle allocation under moment and thrust constraints.
use std::slice;

fn vertices(n: usize, free: &[usize], mut visit: impl FnMut(Vec<f64>)) {
    let others: Vec<_> = (0..n).filter(|i| !free.contains(i)).collect();
    for bits in 0..(1usize << others.len()) {
        let mut u = vec![0.; n];
        for (position, &i) in others.iter().enumerate() {
            u[i] = ((bits >> (others.len() - position - 1)) & 1) as f64;
        }
        visit(u);
    }
}
fn sum(a: &[f64], b: &[f64]) -> f64 {
    let mut hi: f64 = 0.;
    let mut lo: f64 = 0.;
    for (x, y) in a.iter().zip(b) {
        let value = x * y;
        let next = hi + value;
        lo += if hi.abs() >= value.abs() {
            (hi - next) + value
        } else {
            (value - next) + hi
        };
        hi = next;
    }
    hi + lo
}

#[no_mangle]
pub unsafe extern "C" fn ksl_allocate(
    n: usize,
    thrusts: *const f64,
    arms: *const f64,
    bounds: *const f64,
    flows: *const f64,
    fixed: f64,
    target: f64,
    cap: f64,
    tolerance: f64,
    output: *mut f64,
) -> bool {
    if n == 0 || n >= usize::BITS as usize {
        return false;
    }
    let t = slice::from_raw_parts(thrusts, n);
    let a = slice::from_raw_parts(arms, n);
    let b = slice::from_raw_parts(bounds, n);
    let m = slice::from_raw_parts(flows, n);
    let et: Vec<_> = (0..n).map(|i| t[i] * b[i]).collect();
    let em: Vec<_> = (0..n).map(|i| m[i] * b[i]).collect();
    let coeff: Vec<_> = (0..n).map(|i| et[i] * a[i]).collect();
    let mut candidates = Vec::new();
    let mut add = |mut u: Vec<f64>| {
        if u.iter().any(|x| !(-1e-9..=1. + 1e-9).contains(x)) {
            return;
        }
        for x in &mut u {
            *x = x.max(0.).min(1.);
        }
        if sum(&et, &u) <= cap + 1e-6 {
            candidates.push(u);
        }
    };
    vertices(n, &[], |u| add(u));
    let wanted = target - fixed;
    let mut planes = vec![wanted];
    if tolerance > 0. {
        planes.extend([wanted - tolerance, wanted + tolerance]);
    }
    for &plane in &planes {
        for free in 0..n {
            vertices(n, &[free], |mut u| {
                let rem = plane - sum(&coeff, &u);
                let c = coeff[free];
                if c.abs() < 1e-12 {
                    if rem.abs() <= 1e-6 {
                        u[free] = 1.;
                        add(u);
                    }
                } else {
                    u[free] = rem / c;
                    add(u);
                }
            });
        }
    }
    if cap.is_finite() {
        for free in 0..n {
            vertices(n, &[free], |mut u| {
                let rem = cap - sum(&et, &u);
                let x = et[free];
                if x <= 1e-12 {
                    if rem.abs() <= 1e-6 {
                        u[free] = 1.;
                        add(u);
                    }
                } else {
                    u[free] = rem / x;
                    add(u);
                }
            });
        }
        for &plane in &planes {
            for i in 0..n {
                for j in i + 1..n {
                    vertices(n, &[i, j], |mut u| {
                        let rm = plane - sum(&coeff, &u);
                        let rt = cap - sum(&et, &u);
                        let det = coeff[i] * et[j] - coeff[j] * et[i];
                        if det.abs() >= 1e-12 {
                            u[i] = (rm * et[j] - coeff[j] * rt) / det;
                            u[j] = (coeff[i] * rt - rm * et[i]) / det;
                            add(u);
                        }
                    });
                }
            }
        }
    }
    let score = |u: &Vec<f64>| {
        let residual = (fixed + sum(&coeff, u) - target).abs();
        [
            (residual - tolerance - 1e-5).max(0.),
            -sum(&et, u),
            residual,
            sum(&em, u),
        ]
    };
    let Some(best) = candidates.iter().min_by(|x, y| {
        let sx = score(x);
        let sy = score(y);
        sx.iter()
            .zip(sy)
            .find_map(|(a, b)| {
                let c = a.total_cmp(&b);
                if c.is_eq() {
                    None
                } else {
                    Some(c)
                }
            })
            .unwrap_or(std::cmp::Ordering::Equal)
    }) else {
        return false;
    };
    let out = slice::from_raw_parts_mut(output, n);
    for i in 0..n {
        out[i] = best[i] * b[i];
    }
    true
}
