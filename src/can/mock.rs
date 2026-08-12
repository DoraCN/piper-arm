//! Mock CAN bus backend for offline testing and simulation.

use std::collections::VecDeque;
use std::sync::Mutex;

use crate::error::{Error, Result};

use super::CanBus;

/// A CAN bus that stores frames in memory. Sent frames are optionally looped
/// back into the receive queue so protocol round-trips can be tested without
/// hardware.
#[derive(Debug, Default)]
pub struct MockBus {
    /// Frames sent via `send_frame`.
    tx: Mutex<Vec<(u32, Vec<u8>)>>,
    /// Frames that will be returned by `read_frame`.
    rx: Mutex<VecDeque<(u32, Vec<u8>)>>,
}

impl MockBus {
    pub fn new() -> Self {
        Self::default()
    }

    /// Push an incoming frame to be read by `read_frame`.
    pub fn push_incoming(&self, id: u32, data: &[u8]) {
        self.rx.lock().unwrap().push_back((id, data.to_vec()));
    }

    /// Return all frames that have been sent so far.
    pub fn sent_frames(&self) -> Vec<(u32, Vec<u8>)> {
        self.tx.lock().unwrap().clone()
    }
}

impl CanBus for MockBus {
    fn send_frame(&self, id: u32, data: &[u8]) -> Result<()> {
        self.tx.lock().unwrap().push((id, data.to_vec()));
        Ok(())
    }

    fn read_frame(&self) -> Result<(u32, Vec<u8>)> {
        let mut rx = self.rx.lock().unwrap();
        if let Some(frame) = rx.pop_front() {
            Ok(frame)
        } else {
            Err(Error::NotConnected)
        }
    }
}
