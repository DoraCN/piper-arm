//! Error types for the Piper SDK.

use std::fmt;

/// Errors that can occur while using the Piper SDK.
#[derive(Debug)]
pub enum Error {
    /// I/O or CAN bus related error.
    Io(std::io::Error),
    /// The CAN interface could not be opened or the requested channel does not exist.
    Can(String),
    /// The requested CAN port is not in a usable state (e.g. not UP).
    CanNotUp(String),
    /// A value passed to a control/encode function is out of the allowed range.
    ValueError(String),
    /// Unknown CAN id or message type.
    UnknownId(u32),
    /// A CAN frame received has an unexpected payload length.
    UnexpectedFrameLength { id: u32, len: usize },
    /// The SDK is not connected (no CAN bus / thread running).
    NotConnected,
    /// Internal channel / mutex error.
    Lock,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "io error: {e}"),
            Error::Can(s) => write!(f, "can error: {s}"),
            Error::CanNotUp(s) => write!(f, "can port not up: {s}"),
            Error::ValueError(s) => write!(f, "value error: {s}"),
            Error::UnknownId(id) => write!(f, "unknown can id: 0x{id:X}"),
            Error::UnexpectedFrameLength { id, len } => {
                write!(f, "unexpected frame length {len} for id 0x{id:X}")
            }
            Error::NotConnected => write!(f, "interface is not connected"),
            Error::Lock => write!(f, "internal lock poisoned"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

/// Convenience result alias.
pub type Result<T> = std::result::Result<T, Error>;
