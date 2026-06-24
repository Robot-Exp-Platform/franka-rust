//! Realtime UDP control sessions for Franka FCI.
//!
//! The TCP command plane starts and stops a motion. The UDP session owns the
//! per-cycle path: receive state, run the user controller directly, filter the
//! command against the same state, and send it back.

pub(crate) mod std_udp;
pub(crate) mod tokio_udp;
