use core::fmt::Debug;

use embedded_hal::can::Error as CanError;

use crate::{CanSpeed, McpSpeed};

pub type Result<T, SPI> = core::result::Result<T, Error<SPI>>;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error<SPI: Debug> {
    /// MCP2515 did not respond to mode change.
    NewModeTimeout,
    /// Tx buffers are full and therefore cannot send another message.
    TxBusy,
    /// Failed to send a message.
    TxFailed,
    /// There was no message to be received in the Rx buffers.
    NoMessage,
    /// Received an invalid frame ID.
    InvalidFrameId,
    /// Received an invalid DLC (CAN frame data length).
    InvalidDlc,
    /// Invalid configuration options.
    InvalidConfiguration(CanSpeed, McpSpeed),
    /// SPI error.
    Spi(SPI),
}

impl<SPI: Debug> CanError for Error<SPI> {
    fn kind(&self) -> embedded_hal::can::ErrorKind {
        embedded_hal::can::ErrorKind::Other
    }
}
