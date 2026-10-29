#![allow(dead_code, non_camel_case_types, non_snake_case, non_upper_case_globals)]
#![allow(unused_assignments, unused_mut, unused_variables, unused_unsafe, unused_parens)]
#![allow(clashing_extern_declarations, unused_must_use, unused_imports, static_mut_refs)]
#![allow(unsafe_op_in_unsafe_fn, improper_ctypes, improper_ctypes_definitions, mutable_transmutes, unused_comparisons, overflowing_literals, arithmetic_overflow, unconditional_panic)]
pub mod sys;
pub mod taem_candidate_search;
pub mod taem_frames_energy;
pub mod taem_geometry;
pub mod taem_planner;
pub mod taem_reachability;
pub mod taem_tracker;
pub mod terminal_model;
pub mod terminal_propagator;
pub mod terminal_solver;
pub mod app {
pub mod controller;
pub mod main;
}
pub mod calibration {
pub mod calibration;
pub mod calibration_observation;
pub mod trajectory_calibration;
}
pub mod control {
pub mod flight_control;
}
pub mod guidance {
pub mod entry_alpha;
pub mod entry_drag_reference;
pub mod entry_energy_control;
pub mod entry_exec;
pub mod entry_lateral;
pub mod guidance;
pub mod guidance_common;
pub mod guidance_core;
pub mod guidance_entry;
pub mod guidance_final;
pub mod guidance_taem;
pub mod guidance_terminal;
pub mod taem_exec;
}
pub mod krpc_cnano {
pub mod communication;
pub mod decoder;
pub mod encoder;
pub mod error;
pub mod krpc;
pub mod krpc_pb;
pub mod memory;
pub mod utils;
}
pub mod nanopb {
pub mod pb_common;
pub mod pb_decode;
pub mod pb_encode;
}
pub mod persistence {
pub mod physics_store;
}
pub mod physics {
pub mod aero;
pub mod attitude;
pub mod flight_physics;
pub mod math3;
pub mod quat;
pub mod world;
}
pub mod prediction {
pub mod async_prediction_policy;
pub mod decision_envelope;
pub mod predictor;
}
pub mod telemetry {
pub mod flight_log_codec;
pub mod json;
pub mod planner_log;
pub mod sim_telemetry;
}
pub mod transport {
pub mod krpc;
pub mod krpc_cnano_batch;
pub mod krpc_cnano_client;
pub mod krpc_cnano_transport;
}
pub mod vehicle {
pub mod aero_certification;
pub mod math;
pub mod models;
pub mod vessel_physics;
}
