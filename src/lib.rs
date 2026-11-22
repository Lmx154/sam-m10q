//! Driver for the u-blox SAM-M10Q GNSS receiver on a UART, UBX protocol only.
//!
//! Built on the `embedded-io-async` 0.7 and `embedded-hal-async` 1.0 traits only: it works with any
//! executor or none. Two ways in:
//!
//! * the sans-IO core: [`ubx::Parser`] turns bytes (any chunking, NMEA or noise in between) into
//!   [`Message`]s, [`ubx::write_frame`] and [`ValSet`] encode frames. Use this when your firmware
//!   drains its own DMA ring buffer;
//! * the async driver [`Receiver`] on `embedded_io_async::Read + Write` plus [`SetBaud`].
//!
//! The driver cannot change the UART's baud rate itself: the UART type you hand over implements
//! [`SetBaud`] (one line over the HAL's baud call). [`Receiver::configure`] probes the candidate
//! baud rates with a UBX-MON-VER poll, moves the receiver to the target rate, applies one
//! UBX-CFG-VALSET to RAM only (nothing is saved to flash or battery-backed RAM) and waits for its
//! acknowledgement. After that [`Receiver::read_pvt`] returns the next UBX-NAV-PVT, on the byte of
//! its checksum.
//!
//! Timeouts come from the delay you lend to `configure` (a raced `delay_ms`); `read_pvt` waits
//! forever on a silent line, wrap it in your executor's timeout. Dropping any future of the driver
//! loses nothing the driver already read. A `configure` dropped or failed part-way leaves
//! [`Receiver::is_configured`] false and `read_pvt` returns [`Error::NotConfigured`]; call
//! `configure` again (it probes every candidate baud, so the rate the receiver was left at is
//! found). This assumes the UART's `read` is cancel safe (true for `BufferedUart`): it is dropped
//! when a reply wait times out. The UART's `flush` must return when the last byte has left, because
//! the baud rate is changed right after it.
//!
//! Async:
//! ```ignore
//! let mut gnss = sam_m10q::Receiver::new(uart);
//! let version = gnss.configure(&sam_m10q::Config::default(), &mut delay).await?;
//! loop {
//!     let pvt = gnss.read_pvt().await?;
//!     if pvt.is_3d_fix_ok() { /* pvt.lat, pvt.lon ... */ }
//! }
//! ```
//!
//! Poll time: the parser feeds at most 64 received bytes per call to `read`, plus bytes it scans
//! again after a bad frame. Typical cost is tens of cycles per byte. The worst case is an
//! adversarial stream whose false frames each fail at their last byte: a rescan is at most 264
//! bytes and repeats at most that many times, so one poll is bounded by about 35 000 byte steps,
//! which is above lc76g's 250 us at 150 MHz. It needs a stream built for it and no real receiver
//! sends one; the on-board poll-timing figure is not measured yet (no hil program).
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

pub mod config;
pub mod msg;
pub mod ubx;

use core::future::{Future, poll_fn};
use core::pin::{Pin, pin};
use core::task::Poll;

use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, Write};

pub use config::{Config, ValSet};
pub use msg::{Message, Pvt, Version};
pub use ubx::{Parser, ParserStats};

use config::{KEY_UART1_BAUDRATE, VALSET_FRAME_MAX};
use ubx::{CFG_VALSET, MON_VER, write_frame};

/// Bytes read from the UART at a time.
const RX_CHUNK: usize = 64;

/// A UART whose baud rate the caller's code can change.
pub trait SetBaud {
    /// Switches the UART to `baud` (8N1). Bytes already queued for sending are not this method's
    /// concern: the driver flushes before it calls it.
    fn set_baud(&mut self, baud: u32);
}

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// UART error.
    Uart(E),
    /// The UART returned zero bytes from a read or a write: the line is closed.
    Eof,
    /// No candidate baud rate got a UBX-MON-VER reply.
    NoReceiver,
    /// The receiver did not answer after the switch to the target baud rate.
    BaudSwitchFailed,
    /// The configuration got no acknowledgement within the reply timeout.
    NoAck,
    /// The receiver answered UBX-ACK-NAK to the configuration.
    Rejected,
    /// `read_pvt` before a successful `configure`, or after one that was dropped or failed.
    NotConfigured,
    /// The [`Config`] has a value the receiver does not accept. Nothing was sent.
    InvalidConfig,
}

/// SAM-M10Q on an async UART.
pub struct Receiver<UART> {
    uart: UART,
    rx: [u8; RX_CHUNK],
    pos: usize,
    end: usize,
    parser: Parser,
    tx: [u8; VALSET_FRAME_MAX],
    configured: bool,
    version: Option<Version>,
}

impl<UART> Receiver<UART> {
    /// Wraps the UART. Nothing is sent or read until a method is called.
    pub const fn new(uart: UART) -> Self {
        Self {
            uart,
            rx: [0; RX_CHUNK],
            pos: 0,
            end: 0,
            parser: Parser::new(),
            tx: [0; VALSET_FRAME_MAX],
            configured: false,
            version: None,
        }
    }

    /// Returns the UART. Bytes read from it but not yet consumed are lost.
    pub fn release(self) -> UART {
        self.uart
    }

    /// Parser counters since construction.
    pub const fn stats(&self) -> ParserStats {
        self.parser.stats()
    }

    /// Whether the last `configure` completed. `read_pvt` needs it.
    pub const fn is_configured(&self) -> bool {
        self.configured
    }

    /// Version strings from the last `configure` (`None` while not configured).
    pub const fn version(&self) -> Option<Version> {
        self.version
    }

    /// Next message already in the receive buffer or the parser.
    fn buffered(&mut self) -> Option<Message> {
        let mut input = self.rx.get(self.pos..self.end).unwrap_or_default();
        let before = input.len();
        let message = self.parser.next(&mut input);
        self.pos += before - input.len();
        message
    }
}

impl<UART: Read> Receiver<UART> {
    /// Next message, or `None` when `deadline` completes first.
    async fn message_before<F: Future<Output = ()>>(
        &mut self,
        mut deadline: Pin<&mut F>,
    ) -> Result<Option<Message>, Error<UART::Error>> {
        loop {
            if let Some(message) = self.buffered() {
                return Ok(Some(message));
            }
            let mut read = pin!(self.uart.read(&mut self.rx));
            let read = poll_fn(|cx| {
                if let Poll::Ready(result) = read.as_mut().poll(cx) {
                    return Poll::Ready(Some(result));
                }
                if deadline.as_mut().poll(cx).is_ready() {
                    return Poll::Ready(None);
                }
                Poll::Pending
            })
            .await;
            let Some(read) = read else {
                return Ok(None);
            };
            let n = read.map_err(Error::Uart)?;
            if n == 0 {
                return Err(Error::Eof);
            }
            self.pos = 0;
            self.end = n.min(RX_CHUNK);
        }
    }

    /// Next UBX-NAV-PVT; other messages on the way are consumed. Returns on the checksum byte of
    /// the frame, never waiting for another message. A silent line waits forever.
    pub async fn read_pvt(&mut self) -> Result<Pvt, Error<UART::Error>> {
        if !self.configured {
            return Err(Error::NotConfigured);
        }
        let mut never = pin!(core::future::pending::<()>());
        loop {
            if let Some(Message::Pvt(pvt)) = self.message_before(never.as_mut()).await? {
                return Ok(pvt);
            }
        }
    }
}

impl<UART: Read + Write + SetBaud> Receiver<UART> {
    async fn send(&mut self, len: usize) -> Result<(), Error<UART::Error>> {
        let mut rest = self.tx.get(..len).unwrap_or_default();
        while !rest.is_empty() {
            match self.uart.write(rest).await.map_err(Error::Uart)? {
                0 => return Err(Error::Eof),
                n => rest = rest.get(n..).unwrap_or_default(),
            }
        }
        self.uart.flush().await.map_err(Error::Uart)
    }

    fn switch_baud(&mut self, baud: u32) {
        self.uart.set_baud(baud);
        self.parser.reset();
        self.pos = 0;
        self.end = 0;
    }

    /// Polls UBX-MON-VER and waits up to `ms` for the answer.
    async fn poll_version(
        &mut self,
        delay: &mut impl DelayNs,
        ms: u32,
    ) -> Result<Option<Version>, Error<UART::Error>> {
        let len = write_frame(MON_VER.0, MON_VER.1, &[], &mut self.tx).unwrap_or(0);
        self.send(len).await?;
        let mut sleep = pin!(delay.delay_ms(ms));
        while let Some(message) = self.message_before(sleep.as_mut()).await? {
            if let Message::Version(version) = message {
                return Ok(Some(version));
            }
        }
        Ok(None)
    }

    /// Waits up to `ms` for the ACK-ACK or ACK-NAK of the CFG-VALSET.
    async fn wait_valset_ack(
        &mut self,
        delay: &mut impl DelayNs,
        ms: u32,
    ) -> Result<(), Error<UART::Error>> {
        let mut sleep = pin!(delay.delay_ms(ms));
        while let Some(message) = self.message_before(sleep.as_mut()).await? {
            match message {
                Message::Ack { class, id } if (class, id) == CFG_VALSET => return Ok(()),
                Message::Nak { class, id } if (class, id) == CFG_VALSET => {
                    return Err(Error::Rejected);
                }
                _ => {}
            }
        }
        Err(Error::NoAck)
    }

    /// Finds the receiver, moves it to `config.target_baud`, applies the settings and waits for
    /// them to take effect. Returns the version strings.
    ///
    /// For each candidate baud rate (in order): `set_baud`, poll UBX-MON-VER, wait
    /// `reply_timeout_ms` for the answer; none at any rate is [`Error::NoReceiver`]. If the
    /// rate found is not the target, one CFG-VALSET sets CFG-UART1-BAUDRATE (its acknowledgement
    /// is not required, it arrives at the old rate), the UART goes to the target, the driver waits
    /// `baud_settle_ms`, polls UBX-MON-VER again and requires the answer
    /// ([`Error::BaudSwitchFailed`] otherwise). Then the settings of [`Config::valset`] go out in
    /// one CFG-VALSET to RAM; ACK-NAK is [`Error::Rejected`], no answer [`Error::NoAck`]. Last,
    /// `signal_settle_ms` is waited, because a signal change restarts the GNSS.
    ///
    /// [`Receiver::is_configured`] is cleared before the first await and set after the last. The
    /// receiver's output (NMEA at the factory default) before the settings apply is noise to the
    /// parser.
    pub async fn configure(
        &mut self,
        config: &Config,
        delay: &mut impl DelayNs,
    ) -> Result<Version, Error<UART::Error>> {
        self.configured = false;
        self.version = None;
        if !config.is_valid() {
            return Err(Error::InvalidConfig);
        }
        let settings = config.valset().ok_or(Error::InvalidConfig)?;
        let mut baud_set = ValSet::new();
        baud_set
            .push(KEY_UART1_BAUDRATE, config.target_baud)
            .ok_or(Error::InvalidConfig)?;

        let mut found = None;
        for &baud in config.candidate_bauds.iter().filter(|&&b| b != 0) {
            self.switch_baud(baud);
            if let Some(version) = self.poll_version(delay, config.reply_timeout_ms).await? {
                found = Some((baud, version));
                break;
            }
        }
        let (baud, mut version) = found.ok_or(Error::NoReceiver)?;

        if baud != config.target_baud {
            let len = baud_set
                .write_frame(&mut self.tx)
                .ok_or(Error::InvalidConfig)?;
            self.send(len).await?;
            self.switch_baud(config.target_baud);
            delay.delay_ms(config.baud_settle_ms).await;
            version = self
                .poll_version(delay, config.reply_timeout_ms)
                .await?
                .ok_or(Error::BaudSwitchFailed)?;
        }

        let len = settings
            .write_frame(&mut self.tx)
            .ok_or(Error::InvalidConfig)?;
        self.send(len).await?;
        self.wait_valset_ack(delay, config.reply_timeout_ms).await?;
        delay.delay_ms(config.signal_settle_ms).await;
        self.version = Some(version);
        self.configured = true;
        Ok(version)
    }
}

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;

#[cfg(test)]
mod tests;
