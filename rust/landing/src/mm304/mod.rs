//! MM304 entry guidance: one law flown live, in the worker's planner and in
//! every forecast; a closed-loop predictor; a coupled AoA/bank receding-horizon
//! planner aimed at MM305's spiral HAC; physically timed bank reversals; and the
//! telemetry block that shows all of it.

pub mod law;
pub mod lateral;
pub mod live;
pub mod plan;
pub mod predict;
pub mod state;
pub mod telemetry;
pub mod terminal;

pub use state::Mm304State;
