//! Protocol codec layer.

pub mod base;
pub mod v2;

pub use v2::protocol_v2::*;
pub use v2::{can_id as id, can_id, mapping, msg_type, ControlFrame, DecodedMessage, MsgType};
