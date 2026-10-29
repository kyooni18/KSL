//! ShuttleSim: a deterministic Kerbin shuttle simulator driven over UDP in
//! lockstep with the landing backend, plus an in-process offline ABI.

pub mod ffi;
pub mod expert;
pub mod offline;
pub mod protocol;
pub mod replay;
pub mod scenario;
pub mod sim;

// The plant lives in the landing crate; reference it so its symbols link.
extern crate landing;
