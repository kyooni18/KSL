//! Attitude replay CSV: `time_s,aoa_deg,bank_deg[,gear_down]`.

#[derive(Copy, Clone, Debug, Default)]
pub struct ReplayPoint {
    pub time_s: f64,
    pub aoa_deg: f64,
    pub bank_deg: f64,
    pub gear_down: bool,
}

pub struct AttitudeReplay {
    points: Vec<ReplayPoint>,
    cursor: usize,
}

const REPLAY_MAX: usize = 200_000;

impl AttitudeReplay {
    pub fn load_csv(path: &str) -> Option<AttitudeReplay> {
        let text = std::fs::read_to_string(path).ok()?;
        let mut points = Vec::new();
        for line in text.lines() {
            if points.len() >= REPLAY_MAX {
                break;
            }
            if line.starts_with('#') || line.contains("time_s") {
                continue;
            }
            let f: Vec<&str> = line.split(',').map(str::trim).collect();
            let num = |i: usize| f.get(i).and_then(|v| v.parse::<f64>().ok());
            if let (Some(t), Some(a), Some(b)) = (num(0), num(1), num(2)) {
                let gear = f.get(3).and_then(|v| v.parse::<i64>().ok()).is_some_and(|g| g != 0);
                points.push(ReplayPoint { time_s: t, aoa_deg: a, bank_deg: b, gear_down: gear });
            }
        }
        (!points.is_empty()).then_some(AttitudeReplay { points, cursor: 0 })
    }

    /// The commanded attitude at `t`, interpolated; bank through the short way.
    pub fn sample(&mut self, t: f64) -> ReplayPoint {
        let p = &self.points;
        while self.cursor + 1 < p.len() && p[self.cursor + 1].time_s <= t {
            self.cursor += 1;
        }
        if self.cursor + 1 >= p.len() {
            return p[p.len() - 1];
        }
        let (a, b) = (p[self.cursor], p[self.cursor + 1]);
        if t <= a.time_s {
            return a;
        }
        let u = ((t - a.time_s) / (b.time_s - a.time_s)).clamp(0.0, 1.0);
        let mut db = b.bank_deg - a.bank_deg;
        while db > 180.0 {
            db -= 360.0;
        }
        while db < -180.0 {
            db += 360.0;
        }
        ReplayPoint {
            time_s: t,
            aoa_deg: a.aoa_deg + (b.aoa_deg - a.aoa_deg) * u,
            bank_deg: a.bank_deg + db * u,
            gear_down: a.gear_down,
        }
    }
}
