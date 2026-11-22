//! UBX framing: checksum, frame encoder and the streaming [`Parser`].

use crate::msg::{Message, Pvt, Version};

/// First sync byte.
pub const SYNC1: u8 = 0xB5;
/// Second sync byte.
pub const SYNC2: u8 = 0x62;
/// Bytes around the payload: two sync bytes, class, id, length (2) and the two checksum bytes.
pub const FRAME_OVERHEAD: usize = 8;
/// Header length: sync bytes, class, id and length.
const HEADER: usize = 6;
/// Largest payload the parser buffers. MON-VER is 40 + 30 * n bytes (n extension strings), so 256
/// covers six extensions; NAV-PVT is 92. Longer frames are counted in [`ParserStats::oversize`].
pub const MAX_PAYLOAD: usize = 256;
const BUF_LEN: usize = HEADER + MAX_PAYLOAD + 2;

/// Class and ID of UBX-ACK-NAK.
pub const ACK_NAK: (u8, u8) = (0x05, 0x00);
/// Class and ID of UBX-ACK-ACK.
pub const ACK_ACK: (u8, u8) = (0x05, 0x01);
/// Class and ID of UBX-MON-VER.
pub const MON_VER: (u8, u8) = (0x0A, 0x04);
/// Class and ID of UBX-NAV-PVT.
pub const NAV_PVT: (u8, u8) = (0x01, 0x07);
/// Class and ID of UBX-CFG-VALSET.
pub const CFG_VALSET: (u8, u8) = (0x06, 0x8A);

/// 8-bit Fletcher checksum over `data` (class, id, length and payload of a frame): `(CK_A, CK_B)`.
pub fn checksum(data: &[u8]) -> (u8, u8) {
    let (mut a, mut b) = (0u8, 0u8);
    for &byte in data {
        a = a.wrapping_add(byte);
        b = b.wrapping_add(a);
    }
    (a, b)
}

/// Writes a whole UBX frame into `out`; returns its length, or `None` if the payload is over 65535
/// bytes or `out` is too small.
pub fn write_frame(class: u8, id: u8, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let len = u16::try_from(payload.len()).ok()?;
    let total = payload.len().checked_add(FRAME_OVERHEAD)?;
    let frame = out.get_mut(..total)?;
    let (head, rest) = frame.split_first_chunk_mut::<HEADER>()?;
    let [l0, l1] = len.to_le_bytes();
    *head = [SYNC1, SYNC2, class, id, l0, l1];
    let (body, _) = rest.split_at_mut_checked(payload.len())?;
    body.copy_from_slice(payload);
    let sum = checksum(frame.get(2..total - 2)?);
    let tail = frame.get_mut(total - 2..)?;
    tail.copy_from_slice(&[sum.0, sum.1]);
    Some(total)
}

/// Parser counters. They saturate at `u32::MAX`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParserStats {
    /// Frames with a valid checksum, decoded or not.
    pub frames: u32,
    /// Frames with a bad checksum (the bytes are scanned again for a frame inside them).
    pub bad_checksum: u32,
    /// Frames announcing a payload over [`MAX_PAYLOAD`]; their header is dropped and the rest is
    /// scanned as garbage.
    pub oversize: u32,
    /// Valid frames the parser does not decode (other class/id, or a decoded one of the wrong length).
    pub ignored: u32,
}

/// Streaming UBX parser. Feed it bytes in any chunking with [`Parser::push`]; it resynchronises on
/// garbage (NMEA text included), rejects bad checksums and returns decoded [`Message`]s.
///
/// After a bad frame the bytes it swallowed are scanned again, so a real frame that started inside
/// the false one is still found. A false header (garbage `B5 62` and a length) holds the bytes after it
/// until its frame completes or fails, up to [`MAX_PAYLOAD`] + 8 bytes, so a real frame right behind
/// noise can arrive that much late.
pub struct Parser {
    buf: [u8; BUF_LEN],
    len: usize,
    at: usize,
    end: usize,
    stats: ParserStats,
}

enum Step {
    More,
    Fail,
    Frame(usize),
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    /// Empty parser, hunting for a sync byte.
    pub const fn new() -> Self {
        Self {
            buf: [0; BUF_LEN],
            len: 0,
            at: 0,
            end: 0,
            stats: ParserStats {
                frames: 0,
                bad_checksum: 0,
                oversize: 0,
                ignored: 0,
            },
        }
    }

    /// Counters since construction.
    pub const fn stats(&self) -> ParserStats {
        self.stats
    }

    /// Forgets a partly received frame (after a baud change, for instance). Counters are kept.
    pub fn reset(&mut self) {
        self.len = 0;
        self.at = 0;
        self.end = 0;
    }

    fn payload_len(&self) -> usize {
        match (self.buf.get(4), self.buf.get(5)) {
            (Some(&lo), Some(&hi)) => usize::from(u16::from_le_bytes([lo, hi])),
            _ => 0,
        }
    }

    fn step(&mut self, byte: u8) -> Step {
        if self.len == 0 {
            if byte == SYNC1 {
                self.store(byte);
            }
            return Step::More;
        }
        self.store(byte);
        match self.len {
            2 if byte != SYNC2 => return Step::Fail,
            HEADER if self.payload_len() > MAX_PAYLOAD => {
                self.stats.oversize = self.stats.oversize.saturating_add(1);
                return Step::Fail;
            }
            _ => {}
        }
        if self.len < HEADER {
            return Step::More;
        }
        let total = self.payload_len() + FRAME_OVERHEAD;
        if self.len < total {
            return Step::More;
        }
        let Some(sum) = self.buf.get(2..total - 2).map(checksum) else {
            return Step::Fail;
        };
        if self.buf.get(total - 2..total) == Some(&[sum.0, sum.1]) {
            Step::Frame(total)
        } else {
            self.stats.bad_checksum = self.stats.bad_checksum.saturating_add(1);
            Step::Fail
        }
    }

    fn store(&mut self, byte: u8) {
        if let Some(slot) = self.buf.get_mut(self.len) {
            *slot = byte;
            self.len += 1;
        }
    }

    fn finish(&mut self, total: usize) -> Option<Message> {
        self.len = 0;
        self.stats.frames = self.stats.frames.saturating_add(1);
        let message = self.decode(total);
        if message.is_none() {
            self.stats.ignored = self.stats.ignored.saturating_add(1);
        }
        message
    }

    fn decode(&self, total: usize) -> Option<Message> {
        let class = *self.buf.get(2)?;
        let id = *self.buf.get(3)?;
        let payload = self.buf.get(HEADER..total - 2)?;
        match (class, id) {
            ACK_ACK | ACK_NAK => {
                let [cls, msg] = *payload.first_chunk::<2>()?;
                if payload.len() != 2 {
                    return None;
                }
                Some(if (class, id) == ACK_ACK {
                    Message::Ack { class: cls, id: msg }
                } else {
                    Message::Nak { class: cls, id: msg }
                })
            }
            MON_VER => Version::decode(payload).map(Message::Version),
            NAV_PVT => Pvt::decode(payload).map(Message::Pvt),
            _ => None,
        }
    }

    /// Consumes bytes from the front of `input` until a frame decodes to a [`Message`], which is
    /// returned; the checksum byte of that frame is the last byte consumed for it, so the rest of
    /// `input` is untouched. Returns `None` when `input` is used up. Call again with the same slice
    /// until it is empty: after a bad frame the parser holds bytes of its own to scan again, which it
    /// feeds before any new input, so a call can return a message without consuming input.
    pub fn next(&mut self, input: &mut &[u8]) -> Option<Message> {
        loop {
            let byte = if self.at < self.end {
                let byte = *self.buf.get(self.at)?;
                self.at += 1;
                byte
            } else {
                let (&byte, rest) = input.split_first()?;
                *input = rest;
                byte
            };
            match self.step(byte) {
                Step::More => {}
                Step::Frame(total) => {
                    if let Some(message) = self.finish(total) {
                        return Some(message);
                    }
                }
                Step::Fail => self.rescan(),
            }
        }
    }

    /// Drops the first byte of the failed frame; its other bytes, then the unread rest of an earlier
    /// rescan, are fed again from the front of `buf`. `step` stores at `len`, which never passes `at`,
    /// so the unread bytes are never overwritten.
    fn rescan(&mut self) {
        let kept = self.len.saturating_sub(1);
        self.buf.copy_within(1..kept + 1, 0);
        let (at, end) = (self.at.min(self.end), self.end);
        self.buf.copy_within(at..end, kept);
        self.end = kept + (end - at);
        self.at = 0;
        self.len = 0;
    }
}
