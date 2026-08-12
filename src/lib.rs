//! # piper-arm
//!
//! A pure-Rust driver for the AgileX / Songling **Piper** robotic arm, using the
//! V2 CAN protocol.
#![doc = include_str!("../README.md")]

pub mod can;
pub mod error;
pub mod interface;
pub mod kinematics;
pub mod param;
pub mod protocol;
pub mod utils;

pub use can::{mock::MockBus, CanBus};
pub use error::{Error, Result};
pub use interface::{
    LatestArmStatus, LatestEndPose, LatestGripper, LatestJoint, LatestJointCtrl,
    LatestMotionCtrl2, LatestState, PiperInterface,
};
pub use kinematics::PiperForwardKinematics;
pub use param::PiperParamManager;
pub use protocol::{ControlFrame, DecodedMessage, MsgType};
