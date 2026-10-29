//! UDP JSON command input and telemetry output on the loopback interface.

use std::ffi::CString;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};

#[derive(Copy, Clone, Debug, Default)]
pub struct SimCommand {
    pub has_attitude: bool,
    pub aoa_deg: f64,
    pub bank_deg: f64,
    pub has_gear: bool,
    pub gear_down: bool,
    pub has_brakes: bool,
    pub brakes: bool,
    pub has_airbrakes: bool,
    pub airbrakes: bool,
    pub has_speedbrake: bool,
    /// Split-fin opening, 0 (stowed) .. 1 (full).
    pub speedbrake: f64,
    pub has_wheel_steering: bool,
    pub wheel_steering: f64,
    /// Commanded inertial thrust axis (unit, sim frame).
    pub has_thrust_dir: bool,
    pub thrust_dir: [f64; 3],
    pub has_throttle: bool,
    pub throttle: f64,
    /// Direct-control stick inputs in [-1, 1].
    pub has_inputs: bool,
    pub pitch_input: f64,
    pub roll_input: f64,
    pub yaw_input: f64,
    pub step: bool,
    pub pause: bool,
    pub resume: bool,
}

/// `strtod` over the text after `"key"`'s colon (0 when no number follows).
fn json_number(s: &str, key: &str) -> Option<f64> {
    let at = s.find(&format!("\"{key}\""))?;
    let colon = at + s[at..].find(':')?;
    Some(strtod(&s[colon + 1..]).0)
}

fn json_bool(s: &str, key: &str) -> Option<bool> {
    let at = s.find(&format!("\"{key}\""))?;
    let colon = at + s[at..].find(':')?;
    let rest = s[colon + 1..].trim_start_matches([' ', '\t']);
    if rest.starts_with("true") {
        Some(true)
    } else if rest.starts_with("false") {
        Some(false)
    } else {
        None
    }
}

/// C `strtod`: the parsed prefix and the remaining text.
fn strtod(s: &str) -> (f64, &str) {
    let Ok(c) = CString::new(s) else { return (0.0, s) };
    let mut end: *mut libc::c_char = core::ptr::null_mut();
    // SAFETY: `c` is NUL-terminated and outlives the call; `end` points into it.
    let v = unsafe { libc::strtod(c.as_ptr(), &mut end) };
    let used = end as usize - c.as_ptr() as usize;
    (v, &s[used..])
}

fn thrust_dir(json: &str) -> Option<[f64; 3]> {
    let at = json.find("\"thrust_dir\"")?;
    let open = at + json[at..].find('[')?;
    let (x, r) = strtod(&json[open + 1..]);
    let r = r.strip_prefix(',')?;
    let (y, r) = strtod(r);
    let r = r.strip_prefix(',')?;
    let (z, r) = strtod(r);
    let n = (x * x + y * y + z * z).sqrt();
    (r.starts_with(']') && n > 1e-9 && n.is_finite()).then(|| [x / n, y / n, z / n])
}

impl SimCommand {
    /// Parse a command packet; `None` when it carries nothing actionable.
    pub fn parse(json: &str) -> Option<SimCommand> {
        let mut c = SimCommand::default();
        let aoa = json_number(json, "aoa_deg").or_else(|| json_number(json, "aoa"));
        let bank = json_number(json, "bank_deg").or_else(|| json_number(json, "bank"));
        if let (Some(a), Some(b)) = (aoa, bank) {
            c.has_attitude = true;
            c.aoa_deg = a;
            c.bank_deg = b;
        }
        if let Some(t) = json_number(json, "throttle") {
            c.has_throttle = true;
            c.throttle = t;
        }
        if let Some(d) = thrust_dir(json) {
            c.has_thrust_dir = true;
            c.thrust_dir = d;
        }
        if let Some(g) = json_bool(json, "gear_down") {
            c.has_gear = true;
            c.gear_down = g;
        }
        if let Some(b) = json_bool(json, "brakes") {
            c.has_brakes = true;
            c.brakes = b;
        }
        if let Some(b) = json_bool(json, "airbrakes") {
            c.has_airbrakes = true;
            c.airbrakes = b;
        }
        if let Some(sb) = json_number(json, "speedbrake").filter(|v| v.is_finite()) {
            c.has_speedbrake = true;
            c.speedbrake = sb.clamp(0.0, 1.0);
        }
        if let Some(w) = json_number(json, "wheel_steering").or_else(|| json_number(json, "wheelSteering")) {
            c.has_wheel_steering = true;
            c.wheel_steering = if w > 1.0 { 1.0 } else if w < -1.0 { -1.0 } else { w };
        }
        if let (Some(p), Some(r), Some(y)) =
            (json_number(json, "pitch_input"), json_number(json, "roll_input"), json_number(json, "yaw_input"))
        {
            c.has_inputs = true;
            c.pitch_input = p;
            c.roll_input = r;
            c.yaw_input = y;
        }
        c.step = json.contains("\"step\":true");
        c.pause = json.contains("\"pause\":true") || json.contains("\"type\":\"pause\"") || json.contains("\"type\": \"pause\"");
        c.resume =
            json.contains("\"resume\":true") || json.contains("\"type\":\"resume\"") || json.contains("\"type\": \"resume\"");
        let any = c.has_attitude
            || c.has_inputs
            || c.has_gear
            || c.has_brakes
            || c.has_airbrakes
            || c.has_speedbrake
            || c.has_wheel_steering
            || c.has_throttle
            || c.pause
            || c.resume
            || c.step;
        any.then_some(c)
    }
}

pub struct Protocol {
    command: Option<UdpSocket>,
    telemetry: Option<UdpSocket>,
    host: Option<Ipv4Addr>,
    telemetry_port: u16,
    web_telemetry_port: u16,
}

impl Protocol {
    /// Bind the command port and open the telemetry socket; a port of 0 disables it.
    pub fn open(command_port: u16, host: &str, telemetry_port: u16, web_telemetry_port: u16) -> Result<Protocol, String> {
        let mut p = Protocol { command: None, telemetry: None, host: host.parse().ok(), telemetry_port, web_telemetry_port };
        if command_port > 0 {
            let s = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, command_port))
                .map_err(|e| format!("ShuttleSim protocol: command UDP bind 127.0.0.1:{command_port} failed: {e}"))?;
            s.set_nonblocking(true).map_err(|e| e.to_string())?;
            p.command = Some(s);
        }
        if telemetry_port > 0 || web_telemetry_port > 0 {
            let s = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0))
                .map_err(|e| format!("ShuttleSim protocol: telemetry UDP socket failed: {e}"))?;
            p.telemetry = Some(s);
        }
        Ok(p)
    }

    pub fn poll_command(&self) -> Option<SimCommand> {
        let s = self.command.as_ref()?;
        let mut buf = [0u8; 2048];
        let n = s.recv(&mut buf[..2047]).ok().filter(|&n| n > 0)?;
        // A packet with nothing actionable ends this poll.
        SimCommand::parse(&String::from_utf8_lossy(&buf[..n]))
    }

    fn send_port(&self, json: &str, port: u16) {
        if let (Some(s), Some(h), true) = (&self.telemetry, self.host, port > 0) {
            let _ = s.send_to(json.as_bytes(), SocketAddrV4::new(h, port));
        }
    }

    pub fn send_telemetry(&self, json: &str) {
        self.send_port(json, self.telemetry_port);
        if self.web_telemetry_port != self.telemetry_port {
            self.send_port(json, self.web_telemetry_port);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_lockstep_command() {
        let c = SimCommand::parse(r#"{"aoa_deg":40.5,"bank_deg":-12,"gear_down":false,"throttle":0.25,"thrust_dir":[0,0,2],"step":true}"#)
            .unwrap();
        assert!(c.has_attitude && c.aoa_deg == 40.5 && c.bank_deg == -12.0);
        assert!(c.has_gear && !c.gear_down && c.has_throttle && c.step);
        assert_eq!(c.thrust_dir, [0.0, 0.0, 1.0]);
        assert!(SimCommand::parse(r#"{"noise":1}"#).is_none());
    }
}
