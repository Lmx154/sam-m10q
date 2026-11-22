//! Decoded UBX messages: ACK, MON-VER and NAV-PVT.

/// Length of the NAV-PVT payload on the M10.
pub const PVT_LEN: usize = 92;
/// Length of MON-VER's `swVersion` field.
pub const SW_LEN: usize = 30;
/// Length of MON-VER's `hwVersion` field.
pub const HW_LEN: usize = 10;

fn u16_at(p: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes(*p.get(at..)?.first_chunk::<2>()?))
}

fn u32_at(p: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes(*p.get(at..)?.first_chunk::<4>()?))
}

fn i32_at(p: &[u8], at: usize) -> Option<i32> {
    Some(i32::from_le_bytes(*p.get(at..)?.first_chunk::<4>()?))
}

fn u8_at(p: &[u8], at: usize) -> Option<u8> {
    p.get(at).copied()
}

/// A message the [`Parser`](crate::ubx::Parser) decodes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Message {
    /// UBX-ACK-ACK: the receiver accepted the message of this class and ID.
    Ack {
        /// Class of the acknowledged message.
        class: u8,
        /// ID of the acknowledged message.
        id: u8,
    },
    /// UBX-ACK-NAK: the receiver refused the message of this class and ID.
    Nak {
        /// Class of the refused message.
        class: u8,
        /// ID of the refused message.
        id: u8,
    },
    /// UBX-MON-VER.
    Version(Version),
    /// UBX-NAV-PVT.
    Pvt(Pvt),
}

/// UBX-MON-VER: the software and hardware version strings (NUL-padded).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Version {
    /// `swVersion`, 30 bytes, NUL-terminated.
    pub sw: [u8; SW_LEN],
    /// `hwVersion`, 10 bytes, NUL-terminated.
    pub hw: [u8; HW_LEN],
}

fn text(bytes: &[u8]) -> &str {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    bytes
        .get(..end)
        .and_then(|b| core::str::from_utf8(b).ok())
        .unwrap_or("")
}

impl Version {
    /// Decodes a MON-VER payload: at least 40 bytes, then 30-byte extension strings, which are not
    /// kept. Any other length is `None`.
    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() < SW_LEN + HW_LEN || !(payload.len() - SW_LEN - HW_LEN).is_multiple_of(30) {
            return None;
        }
        Some(Self {
            sw: *payload.first_chunk::<SW_LEN>()?,
            hw: *payload.get(SW_LEN..)?.first_chunk::<HW_LEN>()?,
        })
    }

    /// `sw` up to its first NUL; empty if it is not UTF-8.
    pub fn sw_str(&self) -> &str {
        text(&self.sw)
    }

    /// `hw` up to its first NUL; empty if it is not UTF-8.
    pub fn hw_str(&self) -> &str {
        text(&self.hw)
    }
}

/// UBX-NAV-PVT (92 bytes), fields as in the u-blox M10 interface description.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pvt {
    /// GPS time of week of the navigation epoch, ms.
    pub itow: u32,
    /// UTC year.
    pub year: u16,
    /// UTC month, 1..=12.
    pub month: u8,
    /// UTC day, 1..=31.
    pub day: u8,
    /// UTC hour.
    pub hour: u8,
    /// UTC minute.
    pub min: u8,
    /// UTC second.
    pub sec: u8,
    /// Validity flags: bit 0 validDate, bit 1 validTime, bit 2 fullyResolved.
    pub valid: u8,
    /// Time accuracy estimate, ns.
    pub t_acc: u32,
    /// Fraction of a second, ns, may be negative.
    pub nano: i32,
    /// 0 no fix, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + dead reckoning, 5 time only.
    pub fix_type: u8,
    /// Fix status flags: bit 0 gnssFixOK.
    pub flags: u8,
    /// Additional flags (`flags2`).
    pub flags2: u8,
    /// Satellites used in the solution.
    pub num_sv: u8,
    /// Longitude, 1e-7 degree.
    pub lon: i32,
    /// Latitude, 1e-7 degree.
    pub lat: i32,
    /// Height above the ellipsoid, mm.
    pub height: i32,
    /// Height above mean sea level, mm.
    pub h_msl: i32,
    /// Horizontal accuracy estimate, mm.
    pub h_acc: u32,
    /// Vertical accuracy estimate, mm.
    pub v_acc: u32,
    /// North velocity, mm/s.
    pub vel_n: i32,
    /// East velocity, mm/s.
    pub vel_e: i32,
    /// Down velocity, mm/s (positive is down).
    pub vel_d: i32,
    /// Ground speed (2D), mm/s.
    pub g_speed: i32,
    /// Heading of motion (2D), 1e-5 degree.
    pub head_mot: i32,
    /// Speed accuracy estimate, mm/s.
    pub s_acc: u32,
    /// Heading accuracy estimate, 1e-5 degree.
    pub head_acc: u32,
    /// Position DOP, 0.01.
    pub p_dop: u16,
    /// `flags3`: bit 0 invalidLlh.
    pub flags3: u8,
}

impl Pvt {
    /// `valid` bit 0.
    pub const VALID_DATE: u8 = 1;
    /// `valid` bit 1.
    pub const VALID_TIME: u8 = 2;
    /// `valid` bit 2.
    pub const FULLY_RESOLVED: u8 = 4;
    /// `flags` bit 0.
    pub const GNSS_FIX_OK: u8 = 1;
    /// `flags3` bit 0.
    pub const INVALID_LLH: u8 = 1;

    /// Decodes a payload of exactly [`PVT_LEN`] bytes; any other length is `None`.
    pub fn decode(p: &[u8]) -> Option<Self> {
        if p.len() != PVT_LEN {
            return None;
        }
        Some(Self {
            itow: u32_at(p, 0)?,
            year: u16_at(p, 4)?,
            month: u8_at(p, 6)?,
            day: u8_at(p, 7)?,
            hour: u8_at(p, 8)?,
            min: u8_at(p, 9)?,
            sec: u8_at(p, 10)?,
            valid: u8_at(p, 11)?,
            t_acc: u32_at(p, 12)?,
            nano: i32_at(p, 16)?,
            fix_type: u8_at(p, 20)?,
            flags: u8_at(p, 21)?,
            flags2: u8_at(p, 22)?,
            num_sv: u8_at(p, 23)?,
            lon: i32_at(p, 24)?,
            lat: i32_at(p, 28)?,
            height: i32_at(p, 32)?,
            h_msl: i32_at(p, 36)?,
            h_acc: u32_at(p, 40)?,
            v_acc: u32_at(p, 44)?,
            vel_n: i32_at(p, 48)?,
            vel_e: i32_at(p, 52)?,
            vel_d: i32_at(p, 56)?,
            g_speed: i32_at(p, 60)?,
            head_mot: i32_at(p, 64)?,
            s_acc: u32_at(p, 68)?,
            head_acc: u32_at(p, 72)?,
            p_dop: u16_at(p, 76)?,
            flags3: u8_at(p, 78)?,
        })
    }

    /// A usable 3D fix: `fix_type` 3, gnssFixOK set and invalidLlh clear.
    pub const fn is_3d_fix_ok(&self) -> bool {
        self.fix_type == 3
            && self.flags & Self::GNSS_FIX_OK != 0
            && self.flags3 & Self::INVALID_LLH == 0
    }
}
