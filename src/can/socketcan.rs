//! SocketCAN backend built on the `socketcan` crate (Linux).

use socketcan::{CanDataFrame, CanFrame, CanSocket, EmbeddedFrame, Frame, Socket};

use crate::error::{Error, Result};

use super::CanBus;

/// A SocketCAN bus (e.g. `can0`, `vcan0`).
pub struct SocketCanBus {
    socket: CanSocket,
}

impl SocketCanBus {
    /// Open a CAN interface by name.
    pub fn open(ifname: &str) -> Result<Self> {
        let socket = CanSocket::open(ifname).map_err(|e| {
            Error::Can(format!("failed to open CAN interface '{ifname}': {e}"))
        })?;
        Ok(Self { socket })
    }

    /// Open a CAN interface and set a read timeout (useful for `is_ok` monitoring).
    pub fn open_with_timeout(ifname: &str, timeout: std::time::Duration) -> Result<Self> {
        let socket = CanSocket::open(ifname).map_err(|e| {
            Error::Can(format!("failed to open CAN interface '{ifname}': {e}"))
        })?;
        socket.set_read_timeout(timeout).map_err(Error::Io)?;
        Ok(Self { socket })
    }
}

impl CanBus for SocketCanBus {
    fn send_frame(&self, id: u32, data: &[u8]) -> Result<()> {
        let frame = CanDataFrame::from_raw_id(id, data)
            .ok_or_else(|| Error::ValueError(format!("invalid CAN frame id 0x{id:X}")))?;
        self.socket.write_frame(&frame).map_err(Error::Io)
    }

    fn read_frame(&self) -> Result<(u32, Vec<u8>)> {
        let frame = self.socket.read_frame().map_err(Error::Io)?;
        match frame {
            CanFrame::Data(frame) => Ok((frame.raw_id(), frame.data().to_vec())),
            CanFrame::Remote(_) | CanFrame::Error(_) => {
                Err(Error::Can("received non-data frame".into()))
            }
        }
    }
}
