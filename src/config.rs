//! Receiver settings: configuration keys, [`Config`] and the UBX-CFG-VALSET builder [`ValSet`].
//!
//! Sources: "IM" is the u-blox M10 SPG 5.10 interface description (UBX-21035062), "ArduPilot" is
//! `libraries/AP_GPS/AP_GPS_UBLOX_CFG_List.h` (`config_M10`) and `AP_GPS_UBLOX_CFG_Keys.h`.

use crate::ubx::{CFG_VALSET, write_frame};

/// Candidate UART baud rates tried in order by [`Receiver::configure`](crate::Receiver::configure):
/// 9600 (u-blox default), 38400 (a third-party MAX-M10S report) and 115200 (PX4 `ubx.cpp` scan list
/// and the rate the firmware leaves the port at).
pub const DEFAULT_BAUDS: [u32; 3] = [9600, 38400, 115_200];
/// Baud rate configured by default: 115200, the rate the board's UART runs at.
pub const DEFAULT_TARGET_BAUD: u32 = 115_200;
/// Wait for a reply, ms: the receiver answers a CFG-VALSET with ACK-ACK or ACK-NAK within one
/// second (IM, 3.9 "UBX-ACK").
pub const DEFAULT_REPLY_TIMEOUT_MS: u32 = 1000;
/// Wait after a CFG-SIGNAL change, ms: it restarts the GNSS; the integration manual says to wait
/// for the acknowledgement plus 0.5 s.
pub const DEFAULT_SIGNAL_SETTLE_MS: u32 = 500;
/// Wait after switching the UART baud rate, ms: the receiver needs about 100 ms (IM, page 22).
pub const DEFAULT_BAUD_SETTLE_MS: u32 = 100;
/// Measurement period, ms: 200 (5 Hz). The data sheet (tables 1 and 2) allows 200, 500 and 1000 only
/// for this constellation set (8 Hz maximum).
pub const DEFAULT_MEAS_RATE_MS: u16 = 200;
/// Navigation solutions per measurement: 1.
pub const DEFAULT_NAV_RATE: u16 = 1;
/// Dynamic model 8, airborne below 4 g (ArduPilot `config_M10`).
pub const DEFAULT_DYN_MODEL: u8 = 8;
/// Static hold threshold, cm/s (CFG-MOT-GNSSSPEED_THRS): 0 is off, the u-blox default (IM, 2.2.5).
pub const DEFAULT_STATIC_HOLD_THRS: u8 = 0;

/// Measurement periods the receiver accepts here, ms (data sheet tables 1 and 2).
pub const MEAS_RATES_MS: [u16; 3] = [200, 500, 1000];

/// Most key/value pairs in one CFG-VALSET (IM, 3.10.5).
pub const MAX_PAIRS: usize = 64;
/// Layers byte of CFG-VALSET: bit 0, RAM only. Nothing reaches battery-backed RAM or flash.
pub const LAYER_RAM: u8 = 0x01;

/// CFG-UART1-BAUDRATE, U4.
pub const KEY_UART1_BAUDRATE: u32 = 0x4052_0001;
/// CFG-UART1INPROT-UBX, L.
pub const KEY_UART1INPROT_UBX: u32 = 0x1073_0001;
/// CFG-UART1INPROT-NMEA, L.
pub const KEY_UART1INPROT_NMEA: u32 = 0x1073_0002;
/// CFG-UART1OUTPROT-UBX, L.
pub const KEY_UART1OUTPROT_UBX: u32 = 0x1074_0001;
/// CFG-UART1OUTPROT-NMEA, L.
pub const KEY_UART1OUTPROT_NMEA: u32 = 0x1074_0002;
/// CFG-RATE-MEAS, U2, ms.
pub const KEY_RATE_MEAS: u32 = 0x3021_0001;
/// CFG-RATE-NAV, U2.
pub const KEY_RATE_NAV: u32 = 0x3021_0002;
/// CFG-MSGOUT-UBX_NAV_PVT_UART1, U1.
pub const KEY_MSGOUT_NAV_PVT_UART1: u32 = 0x2091_0007;
/// CFG-NAVSPG-DYNMODEL, E1.
pub const KEY_NAVSPG_DYNMODEL: u32 = 0x2011_0021;
/// CFG-MOT-GNSSSPEED_THRS, U1, cm/s.
pub const KEY_MOT_GNSSSPEED_THRS: u32 = 0x2025_0038;
/// CFG-SIGNAL-GPS_ENA, L.
pub const KEY_SIGNAL_GPS_ENA: u32 = 0x1031_001f;
/// CFG-SIGNAL-GPS_L1CA_ENA, L.
pub const KEY_SIGNAL_GPS_L1CA_ENA: u32 = 0x1031_0001;
/// CFG-SIGNAL-GAL_ENA, L.
pub const KEY_SIGNAL_GAL_ENA: u32 = 0x1031_0021;
/// CFG-SIGNAL-GAL_E1_ENA, L.
pub const KEY_SIGNAL_GAL_E1_ENA: u32 = 0x1031_0007;
/// CFG-SIGNAL-BDS_ENA, L.
pub const KEY_SIGNAL_BDS_ENA: u32 = 0x1031_0022;
/// CFG-SIGNAL-BDS_B1_ENA, L.
pub const KEY_SIGNAL_BDS_B1_ENA: u32 = 0x1031_000d;
/// CFG-SIGNAL-BDS_B1C_ENA, L.
pub const KEY_SIGNAL_BDS_B1C_ENA: u32 = 0x1031_000f;
/// CFG-SIGNAL-QZSS_ENA, L.
pub const KEY_SIGNAL_QZSS_ENA: u32 = 0x1031_0024;
/// CFG-SIGNAL-QZSS_L1CA_ENA, L.
pub const KEY_SIGNAL_QZSS_L1CA_ENA: u32 = 0x1031_0012;
/// CFG-SIGNAL-GLO_ENA, L.
pub const KEY_SIGNAL_GLO_ENA: u32 = 0x1031_0025;

/// Constellation and signal switches, as ArduPilot's `config_M10` plus the explicit GPS and Galileo
/// defaults: GPS L1C/A, Galileo E1, BeiDou B1C (not B1I), QZSS L1C/A; GLONASS off. The data sheet's
/// rate limits (tables 1 and 2) are given for this set.
pub const SIGNAL_KEYS: [(u32, bool); 10] = [
    (KEY_SIGNAL_GPS_ENA, true),
    (KEY_SIGNAL_GPS_L1CA_ENA, true),
    (KEY_SIGNAL_GAL_ENA, true),
    (KEY_SIGNAL_GAL_E1_ENA, true),
    (KEY_SIGNAL_BDS_ENA, true),
    (KEY_SIGNAL_BDS_B1_ENA, false),
    (KEY_SIGNAL_BDS_B1C_ENA, true),
    (KEY_SIGNAL_QZSS_ENA, true),
    (KEY_SIGNAL_QZSS_L1CA_ENA, true),
    (KEY_SIGNAL_GLO_ENA, false),
];

/// Largest CFG-VALSET payload [`ValSet`] holds: the 4-byte header and 64 pairs of a key and a value
/// of up to 4 bytes (the 8-byte value size is not supported).
pub const VALSET_PAYLOAD_MAX: usize = 4 + MAX_PAIRS * 8;
/// Largest CFG-VALSET frame.
pub const VALSET_FRAME_MAX: usize = VALSET_PAYLOAD_MAX + crate::ubx::FRAME_OVERHEAD;

/// Value size in bytes from bits 28..30 of `key`: 1 and 2 are one byte (L, U1/E1/X1), 3 is two, 4 is
/// four. Size 5 (eight bytes) and the reserved codes are `None`.
pub const fn key_size(key: u32) -> Option<usize> {
    match (key >> 28) & 7 {
        1 | 2 => Some(1),
        3 => Some(2),
        4 => Some(4),
        _ => None,
    }
}

/// Receiver settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Baud rates to probe, in order; 0 entries are not used.
    pub candidate_bauds: [u32; 3],
    /// Baud rate to leave the receiver at.
    pub target_baud: u32,
    /// Reply timeout, ms.
    pub reply_timeout_ms: u32,
    /// Wait after the CFG-VALSET that changes signals, ms.
    pub signal_settle_ms: u32,
    /// Wait after the baud rate switch, ms.
    pub baud_settle_ms: u32,
    /// Measurement period, ms; one of [`MEAS_RATES_MS`].
    pub meas_rate_ms: u16,
    /// Navigation solutions per measurement, at least 1.
    pub nav_rate: u16,
    /// Dynamic platform model (CFG-NAVSPG-DYNMODEL).
    pub dyn_model: u8,
    /// Static hold threshold, cm/s (CFG-MOT-GNSSSPEED_THRS).
    pub static_hold_thrs: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            candidate_bauds: DEFAULT_BAUDS,
            target_baud: DEFAULT_TARGET_BAUD,
            reply_timeout_ms: DEFAULT_REPLY_TIMEOUT_MS,
            signal_settle_ms: DEFAULT_SIGNAL_SETTLE_MS,
            baud_settle_ms: DEFAULT_BAUD_SETTLE_MS,
            meas_rate_ms: DEFAULT_MEAS_RATE_MS,
            nav_rate: DEFAULT_NAV_RATE,
            dyn_model: DEFAULT_DYN_MODEL,
            static_hold_thrs: DEFAULT_STATIC_HOLD_THRS,
        }
    }
}

impl Config {
    /// Whether every value is one the receiver accepts: a nonzero target baud and first candidate,
    /// nonzero timeout, a measurement period from [`MEAS_RATES_MS`] and a nonzero navigation rate.
    pub fn is_valid(&self) -> bool {
        self.target_baud != 0
            && self.candidate_bauds.iter().any(|&b| b != 0)
            && self.reply_timeout_ms != 0
            && MEAS_RATES_MS.contains(&self.meas_rate_ms)
            && self.nav_rate != 0
    }

    /// The one CFG-VALSET with every setting except the baud rate: protocols, rates, the NAV-PVT
    /// output, dynamic model, static hold and the signals. `None` if a value does not fit.
    pub fn valset(&self) -> Option<ValSet> {
        let mut set = ValSet::new();
        set.push_bool(KEY_UART1INPROT_UBX, true)?;
        set.push_bool(KEY_UART1INPROT_NMEA, false)?;
        set.push_bool(KEY_UART1OUTPROT_UBX, true)?;
        set.push_bool(KEY_UART1OUTPROT_NMEA, false)?;
        set.push(KEY_RATE_MEAS, u32::from(self.meas_rate_ms))?;
        set.push(KEY_RATE_NAV, u32::from(self.nav_rate))?;
        set.push(KEY_MSGOUT_NAV_PVT_UART1, 1)?;
        set.push(KEY_NAVSPG_DYNMODEL, u32::from(self.dyn_model))?;
        set.push(KEY_MOT_GNSSSPEED_THRS, u32::from(self.static_hold_thrs))?;
        for (key, on) in SIGNAL_KEYS {
            set.push_bool(key, on)?;
        }
        Some(set)
    }
}

/// Builder of a UBX-CFG-VALSET payload: version 0, layers [`LAYER_RAM`], two reserved bytes, then
/// key/value pairs (little endian), at most [`MAX_PAIRS`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValSet {
    buf: [u8; VALSET_PAYLOAD_MAX],
    len: usize,
    pairs: usize,
}

impl Default for ValSet {
    fn default() -> Self {
        Self::new()
    }
}

impl ValSet {
    /// Empty message: header only.
    pub const fn new() -> Self {
        let mut buf = [0; VALSET_PAYLOAD_MAX];
        #[allow(clippy::indexing_slicing)]
        {
            buf[1] = LAYER_RAM; // bound: constant index 1 of a 516-byte array
        }
        Self {
            buf,
            len: 4,
            pairs: 0,
        }
    }

    /// Number of pairs added.
    pub const fn pairs(&self) -> usize {
        self.pairs
    }

    /// The payload so far.
    pub fn payload(&self) -> &[u8] {
        self.buf.get(..self.len).unwrap_or_default()
    }

    /// Adds `key` with the low bytes of `value`, sized by the key. `None` (nothing added) if the key
    /// size is unsupported, the value does not fit that size, or there are already [`MAX_PAIRS`].
    pub fn push(&mut self, key: u32, value: u32) -> Option<()> {
        let size = key_size(key)?;
        if self.pairs >= MAX_PAIRS || (size < 4 && value >> (8 * size) != 0) {
            return None;
        }
        let end = self.len + 4 + size;
        let dest = self.buf.get_mut(self.len..end)?;
        let (k, v) = dest.split_at_mut_checked(4)?;
        k.copy_from_slice(&key.to_le_bytes());
        v.copy_from_slice(value.to_le_bytes().get(..size)?);
        self.len = end;
        self.pairs += 1;
        Some(())
    }

    /// Adds a boolean key (L, one byte).
    pub fn push_bool(&mut self, key: u32, value: bool) -> Option<()> {
        self.push(key, u32::from(value))
    }

    /// Writes the whole frame to `out`; returns its length.
    pub fn write_frame(&self, out: &mut [u8]) -> Option<usize> {
        write_frame(CFG_VALSET.0, CFG_VALSET.1, self.payload(), out)
    }
}
