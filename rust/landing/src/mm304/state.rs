//! MM304 state. Plain data stored by value inside `GuidanceMachine`, so every
//! copy (worker request, diagnostic shadow, deorbit witness) is independent.
//! The layout is mirrored by `Mm304State` in `CLanding/include/guidance_types.h`.

pub const MM304_SAMPLES: usize = 48;
pub const MM304_SCHEDULE_POINTS: usize = 8;
/// How long one forecast is compared against the flown state before the next.
pub const MM304_RESIDUAL_HORIZON_S: f64 = 60.0;

/// Why the active plan is what it is.
pub const MM304_DECISION_NONE: u32 = 0;
pub const MM304_DECISION_INITIAL: u32 = 1;
pub const MM304_DECISION_KEPT: u32 = 2;
pub const MM304_DECISION_REPLACED: u32 = 3;
pub const MM304_DECISION_INFEASIBLE_DEFICIT: u32 = 4;
pub const MM304_DECISION_INFEASIBLE_SURPLUS: u32 = 5;
pub const MM304_DECISION_UNCERTAIN: u32 = 6;

/// The constraint that limits the plan (none when the target energy is met).
pub const MM304_LIMIT_NONE: u32 = 0;
pub const MM304_LIMIT_ENERGY_DEFICIT: u32 = 1;
pub const MM304_LIMIT_ENERGY_SURPLUS: u32 = 2;
pub const MM304_LIMIT_NOT_CONVERGED: u32 = 3;
pub const MM304_LIMIT_LOAD_ENVELOPE: u32 = 4;
pub const MM304_LIMIT_HANDOFF_ALTITUDE: u32 = 5;
pub const MM304_LIMIT_HAC_GEOMETRY: u32 = 6;

/// The MM305 route a plan delivers to: the full spiral, or the least-energy HAC
/// route when the spiral is out of reach even at maximum range.
pub const MM304_ROUTE_SPIRAL: u32 = 0;
pub const MM304_ROUTE_SHORT: u32 = 1;

pub const MM304_REGIME_ENERGY: u32 = 0;
pub const MM304_REGIME_LATERAL: u32 = 1;

#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct Mm304Sample {
    pub ut: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub speed: f64,
    /// Energy height above the site, m.
    pub energy: f64,
    pub aoa: f64,
    pub bank: f64,
    /// Predicted lateral miss of the target, m, positive right of track.
    pub miss: f64,
}

/// The AoA schedule the law flies, identical live and predicted.
#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct Mm304Schedule {
    pub valid: bool,
    pub count: u32,
    pub velocity: [f64; MM304_SCHEDULE_POINTS],
    pub alpha: [f64; MM304_SCHEDULE_POINTS],
    /// Test staircase (Mach threshold → AoA), flown in place of the schedule when set.
    pub stair_count: u32,
    pub stair_mach: [f64; MM304_SCHEDULE_POINTS],
    pub stair_alpha: [f64; MM304_SCHEDULE_POINTS],
    /// AoA command slew, deg/s.
    pub rate_limit: f64,
    pub min_aoa: f64,
    pub max_aoa: f64,
}

/// Predicted state at the MM305 admission boundary, scored by MM305's spiral HAC.
#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct Mm304Terminal {
    pub valid: bool,
    pub admissible: bool,
    pub ut: f64,
    pub along: f64,
    pub cross: f64,
    pub heading_error: f64,
    pub altitude: f64,
    pub speed: f64,
    pub flight_path_angle: f64,
    /// Energy minus MM305's HAC requirement along its own drag path, m.
    pub energy_error: f64,
    /// Surplus MM305 can still shed, m.
    pub band: f64,
    /// HAC turn of the spiral-side route from this state, deg.
    pub spiral_turn: f64,
    /// The same handoff scored on MM305's least-energy route (its drag-path target).
    pub short_error: f64,
    pub short_band: f64,
    /// Where the spiral-side route joins MM305's HAC (runway along/cross, m) and
    /// its course there, deg; MM304 steers onto this tangent line.
    pub tangent_along: f64,
    pub tangent_cross: f64,
    pub tangent_course: f64,
}

#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct Mm304Plan {
    pub valid: bool,
    pub converged: bool,
    pub in_envelope: bool,
    pub sequence: u32,
    pub decision: u32,
    pub limiting: u32,
    pub forecasts: u32,
    pub reversals_predicted: u32,
    pub route: u32,
    pub ut: f64,
    /// Energy control: -1 max L/D wings level, 0 schedule, +1 schedule at max bank.
    pub sigma: f64,
    pub aoa_bias: f64,
    pub bank: f64,
    pub target_energy: f64,
    pub predicted_energy: f64,
    pub uncertainty: f64,
    pub cost: f64,
    pub energy_at_min: f64,
    pub energy_at_max: f64,
    /// Spiral-route energy error and turn at maximum range (the reachability test).
    pub spiral_at_max_range: f64,
    pub spiral_turn_at_max_range: f64,
    pub terminal: Mm304Terminal,
    pub sample_count: u32,
    pub samples: [Mm304Sample; MM304_SAMPLES],
}

#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct Mm304Law {
    pub initialized: bool,
    pub regime: u32,
    pub reversals: u32,
    pub bank_sign: f64,
    pub aoa_cmd: f64,
    pub bank_cmd: f64,
    pub last_ut: f64,
    pub last_reversal_ut: f64,
    pub t_go: f64,
    pub miss: f64,
    pub miss_hold: f64,
    pub miss_reverse: f64,
    pub accel_capacity: f64,
    pub accel_required: f64,
}

/// Predicted-vs-actual residuals of the active plan at matched UT.
#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct Mm304Residual {
    pub count: u32,
    pub plan_sequence: u32,
    pub ut: f64,
    pub horizon: f64,
    pub energy: f64,
    pub altitude: f64,
    pub speed: f64,
    pub miss: f64,
    /// Guidance-model part: commanded attitude versus the forecast's.
    pub aoa: f64,
    pub bank: f64,
    /// RMS growth rate of the energy residual, m/s; drives plan hysteresis.
    pub energy_rate_rms: f64,
    pub aoa_rms: f64,
    pub bank_rms: f64,
}

#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct Mm304State {
    pub schedule: Mm304Schedule,
    pub plan: Mm304Plan,
    /// The plan residuals are measured against, held for `MM304_RESIDUAL_HORIZON_S`.
    pub reference: Mm304Plan,
    pub law: Mm304Law,
    pub residual: Mm304Residual,
    /// Lateral target (final alignment aimpoint) and the handoff it must reach.
    pub target_valid: bool,
    pub target_latitude: f64,
    pub target_longitude: f64,
    pub station_range: f64,
    pub handoff_speed: f64,
    pub bank_limit: f64,
    pub cross_uncertainty: f64,
    /// The live state scored as a handoff by MM305's HAC; MM305 admission under the
    /// HAC law uses exactly this window.
    pub live_terminal: Mm304Terminal,
}

impl Mm304State {
    // SAFETY: every field is a number, a bool or an array of them; all-zero is valid.
    pub const ZERO: Mm304State = unsafe { core::mem::zeroed() };
}
