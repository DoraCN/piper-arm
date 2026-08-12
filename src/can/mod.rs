//! CAN bus abstraction layer.

pub mod mock;
pub mod socketcan;

pub use mock::MockBus;

use crate::error::Result;

/// A CAN bus backend.
///
/// Implementations must be safe to share across threads: `send_frame` and
/// `read_frame` take `&self` (interior mutability where required).
pub trait CanBus: Send + Sync {
    /// Transmit a standard CAN data frame.
    fn send_frame(&self, id: u32, data: &[u8]) -> Result<()>;

    /// Block until a CAN frame is received, returning `(id, data)`.
    fn read_frame(&self) -> Result<(u32, Vec<u8>)>;
}


