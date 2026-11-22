//! Host tests: framing, decoding, configuration bytes and the driver against a fake receiver.

use core::future::Future;
use core::pin::{Pin, pin};
use core::task::{Context, Poll, Waker};
use std::collections::VecDeque;

use proptest::prelude::*;

use crate::config::{self, Config, ValSet};
use crate::msg::{Message, PVT_LEN, Pvt, Version};
use crate::ubx::{self, Parser, checksum, write_frame};
use crate::{Error, Receiver, SetBaud};

// ---------------------------------------------------------------- helpers

fn noop_waker() -> Waker {
    Waker::noop().clone()
}

fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let waker = noop_waker();
    let mut cx = Context::from_waker(&waker);
    for _ in 0..100_000 {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
    }
    panic!("future did not finish");
}

fn frame(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = vec![0; payload.len() + 8];
    let n = write_frame(class, id, payload, &mut out).unwrap();
    out.truncate(n);
    out
}

fn parse_all(bytes: &[u8]) -> (Vec<Message>, ubx::ParserStats) {
    let mut parser = Parser::new();
    let mut out = Vec::new();
    let mut input = bytes;
    while !input.is_empty() {
        while let Some(m) = parser.next(&mut input) {
            out.push(m);
        }
    }
    // Drain a rescan still pending at the end.
    while let Some(m) = parser.next(&mut input) {
        out.push(m);
    }
    (out, parser.stats())
}

fn put<const N: usize>(p: &mut [u8], at: usize, value: [u8; N]) {
    p[at..at + N].copy_from_slice(&value);
}

/// A NAV-PVT payload built field by field at the offsets of the interface description.
fn pvt_payload() -> Vec<u8> {
    let mut p = vec![0u8; PVT_LEN];
    put(&mut p, 0, 0x0102_0304u32.to_le_bytes());
    put(&mut p, 4, 2026u16.to_le_bytes());
    p[6] = 11;
    p[7] = 22;
    p[8] = 13;
    p[9] = 14;
    p[10] = 15;
    p[11] = 0b111;
    put(&mut p, 12, 25u32.to_le_bytes());
    put(&mut p, 16, (-123_456i32).to_le_bytes());
    p[20] = 3;
    p[21] = 0b0000_0001;
    p[22] = 0x5A;
    p[23] = 17;
    put(&mut p, 24, (-1_223_456_789i32).to_le_bytes());
    put(&mut p, 28, 377_654_321i32.to_le_bytes());
    put(&mut p, 32, 40_000i32.to_le_bytes());
    put(&mut p, 36, (-1_500i32).to_le_bytes());
    put(&mut p, 40, 1_800u32.to_le_bytes());
    put(&mut p, 44, 2_900u32.to_le_bytes());
    put(&mut p, 48, 1_000i32.to_le_bytes());
    put(&mut p, 52, (-2_000i32).to_le_bytes());
    put(&mut p, 56, 3_000i32.to_le_bytes());
    put(&mut p, 60, 2_236i32.to_le_bytes());
    put(&mut p, 64, 12_345_678i32.to_le_bytes());
    put(&mut p, 68, 150u32.to_le_bytes());
    put(&mut p, 72, 4_500_000u32.to_le_bytes());
    put(&mut p, 76, 123u16.to_le_bytes());
    p[78] = 0;
    p
}

fn pvt_frame() -> Vec<u8> {
    frame(1, 7, &pvt_payload())
}

fn ver_payload(ext: usize) -> Vec<u8> {
    let mut p = vec![0u8; 40 + 30 * ext];
    p[..13].copy_from_slice(b"EXT CORE 5.10");
    p[30..39].copy_from_slice(b"000A0000\0");
    p
}

// ---------------------------------------------------------------- checksum and encoding

#[test]
fn checksum_of_mon_ver_poll() {
    assert_eq!(checksum(&[0x0A, 0x04, 0x00, 0x00]), (0x0E, 0x34));
    assert_eq!(
        frame(0x0A, 0x04, &[]),
        [0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34]
    );
}

#[test]
fn valset_by_hand() {
    // One pair: CFG-UART1INPROT-UBX (0x10730001, L, one byte) = 1. Checksum worked out by hand:
    // CK_A = 0x1F, CK_B = 0xB0 over 06 8A 09 00 | 00 01 00 00 | 01 00 73 10 | 01.
    let mut set = ValSet::new();
    set.push_bool(config::KEY_UART1INPROT_UBX, true).unwrap();
    let mut out = [0u8; 32];
    let n = set.write_frame(&mut out).unwrap();
    assert_eq!(
        &out[..n],
        [
            0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x73, 0x10,
            0x01, 0x1F, 0xB0
        ]
    );
}

#[test]
fn valset_default_golden() {
    let set = Config::default().valset().unwrap();
    let expected: Vec<u8> = vec![
        0x00, 0x01, 0x00, 0x00, // version 0, RAM, reserved
        0x01, 0x00, 0x73, 0x10, 0x01, // INPROT-UBX = 1
        0x02, 0x00, 0x73, 0x10, 0x00, // INPROT-NMEA = 0
        0x01, 0x00, 0x74, 0x10, 0x01, // OUTPROT-UBX = 1
        0x02, 0x00, 0x74, 0x10, 0x00, // OUTPROT-NMEA = 0
        0x01, 0x00, 0x21, 0x30, 0xC8, 0x00, // RATE-MEAS = 200
        0x02, 0x00, 0x21, 0x30, 0x01, 0x00, // RATE-NAV = 1
        0x07, 0x00, 0x91, 0x20, 0x01, // MSGOUT-UBX_NAV_PVT_UART1 = 1
        0x21, 0x00, 0x11, 0x20, 0x08, // NAVSPG-DYNMODEL = 8
        0x38, 0x00, 0x25, 0x20, 0x00, // MOT-GNSSSPEED_THRS = 0
        0x1F, 0x00, 0x31, 0x10, 0x01, // GPS_ENA
        0x01, 0x00, 0x31, 0x10, 0x01, // GPS_L1CA_ENA
        0x21, 0x00, 0x31, 0x10, 0x01, // GAL_ENA
        0x07, 0x00, 0x31, 0x10, 0x01, // GAL_E1_ENA
        0x22, 0x00, 0x31, 0x10, 0x01, // BDS_ENA
        0x0D, 0x00, 0x31, 0x10, 0x00, // BDS_B1_ENA = 0
        0x0F, 0x00, 0x31, 0x10, 0x01, // BDS_B1C_ENA
        0x24, 0x00, 0x31, 0x10, 0x01, // QZSS_ENA
        0x12, 0x00, 0x31, 0x10, 0x01, // QZSS_L1CA_ENA
        0x25, 0x00, 0x31, 0x10, 0x00, // GLO_ENA = 0
        0x18, 0x00, 0x31, 0x10, 0x00, // GLO_L1_ENA = 0 (added with the constellation sets)
    ];
    assert_eq!(set.payload(), expected);
    assert_eq!(set.pairs(), 20);
    assert!(set.pairs() <= config::MAX_PAIRS);
    // Walk the pairs: each value's size follows from the key.
    let mut at = 4;
    let mut n = 0;
    while at < expected.len() {
        let key = u32::from_le_bytes(expected[at..at + 4].try_into().unwrap());
        at += 4 + config::key_size(key).unwrap();
        n += 1;
    }
    assert_eq!((at, n), (expected.len(), 20));
}

#[test]
fn valset_limits() {
    assert_eq!(config::key_size(config::KEY_UART1_BAUDRATE), Some(4));
    assert_eq!(config::key_size(config::KEY_RATE_MEAS), Some(2));
    assert_eq!(config::key_size(config::KEY_NAVSPG_DYNMODEL), Some(1));
    assert_eq!(config::key_size(config::KEY_SIGNAL_GPS_ENA), Some(1));
    assert_eq!(config::key_size(0x5000_0001), None);
    let mut set = ValSet::new();
    for i in 0..64 {
        assert!(set.push(0x1031_0000 + i, 1).is_some());
    }
    assert!(set.push(0x1031_0100, 1).is_none());
    assert_eq!(set.pairs(), 64);
    assert!(ValSet::new().push(config::KEY_RATE_MEAS, 0x1_0000).is_none());
    let mut baud = ValSet::new();
    baud.push(config::KEY_UART1_BAUDRATE, 115_200).unwrap();
    assert_eq!(
        baud.payload(),
        [0, 1, 0, 0, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00]
    );
}

#[test]
fn valset_signals_per_set() {
    use config::Constellations as C;
    // GPS, GPS_L1CA, GAL, GAL_E1, BDS, BDS_B1, BDS_B1C, QZSS, QZSS_L1CA, GLO, GLO_L1
    for (set, bits, min) in [
        (C::GpsGalBdsQzss, [1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0], 125),
        (C::GpsGal, [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0], 100),
        (C::GpsGalGlo, [1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1], 167),
    ] {
        let cfg = Config { constellations: set, ..Config::default() };
        let payload = cfg.valset().unwrap().payload().to_vec();
        let keys: [u32; 11] = [
            0x1031_001f, 0x1031_0001, 0x1031_0021, 0x1031_0007, 0x1031_0022, 0x1031_000d,
            0x1031_000f, 0x1031_0024, 0x1031_0012, 0x1031_0025, 0x1031_0018,
        ];
        let mut tail = Vec::new();
        for (k, b) in keys.iter().zip(bits) {
            tail.extend_from_slice(&k.to_le_bytes());
            tail.push(b);
        }
        assert!(payload.ends_with(&tail), "{set:?}");
        assert_eq!(payload.len(), 106, "{set:?}");
        assert_eq!(set.min_meas_rate_ms(), min);
        assert!(cfg.is_valid());
    }
    assert_eq!(Config::default().constellations, C::GpsGalBdsQzss);
}

#[test]
fn config_validation() {
    assert!(Config::default().is_valid());
    assert!(Config { meas_rate_ms: 100, ..Config::default() }.is_valid());
    for bad in [
        Config { meas_rate_ms: 150, ..Config::default() },
        Config { nav_rate: 0, ..Config::default() },
        Config { target_baud: 0, ..Config::default() },
        Config { reply_timeout_ms: 0, ..Config::default() },
        Config { candidate_bauds: [0; 3], ..Config::default() },
    ] {
        assert!(!bad.is_valid());
    }
}

// ---------------------------------------------------------------- decoding

#[test]
fn pvt_golden_decode() {
    let (messages, stats) = parse_all(&pvt_frame());
    let Some(Message::Pvt(pvt)) = messages.first().copied() else {
        panic!("no pvt: {messages:?}");
    };
    assert_eq!(messages.len(), 1);
    assert_eq!(stats.frames, 1);
    assert_eq!(pvt.itow, 0x0102_0304);
    assert_eq!(pvt.year, 2026);
    assert_eq!((pvt.month, pvt.day, pvt.hour, pvt.min, pvt.sec), (11, 22, 13, 14, 15));
    assert_eq!(pvt.valid, 7);
    assert_eq!(pvt.t_acc, 25);
    assert_eq!(pvt.nano, -123_456);
    assert_eq!(pvt.fix_type, 3);
    assert_eq!(pvt.flags, 1);
    assert_eq!(pvt.flags2, 0x5A);
    assert_eq!(pvt.num_sv, 17);
    assert_eq!(pvt.lon, -1_223_456_789);
    assert_eq!(pvt.lat, 377_654_321);
    assert_eq!(pvt.height, 40_000);
    assert_eq!(pvt.h_msl, -1_500);
    assert_eq!(pvt.h_acc, 1_800);
    assert_eq!(pvt.v_acc, 2_900);
    assert_eq!((pvt.vel_n, pvt.vel_e, pvt.vel_d), (1_000, -2_000, 3_000));
    assert_eq!(pvt.g_speed, 2_236);
    assert_eq!(pvt.head_mot, 12_345_678);
    assert_eq!(pvt.s_acc, 150);
    assert_eq!(pvt.head_acc, 4_500_000);
    assert_eq!(pvt.p_dop, 123);
    assert_eq!(pvt.flags3, 0);
    assert!(pvt.is_3d_fix_ok());
}

#[test]
fn pvt_wrong_length_is_ignored() {
    let (messages, stats) = parse_all(&frame(1, 7, &pvt_payload()[..91]));
    assert!(messages.is_empty());
    assert_eq!((stats.frames, stats.ignored), (1, 1));
}

#[test]
fn fix_truth_table() {
    let base = Pvt::decode(&pvt_payload()).unwrap();
    assert!(base.is_3d_fix_ok());
    for fix_type in [0, 1, 2, 4, 5] {
        assert!(!Pvt { fix_type, ..base }.is_3d_fix_ok(), "fix_type {fix_type}");
    }
    assert!(!Pvt { flags: 0, ..base }.is_3d_fix_ok());
    assert!(!Pvt { flags3: 1, ..base }.is_3d_fix_ok());
    assert!(Pvt { flags: 0xFF, flags3: 0xFE, ..base }.is_3d_fix_ok());
}

#[test]
fn ack_nak_and_version() {
    let mut bytes = frame(5, 1, &[6, 0x8A]);
    bytes.extend(frame(5, 0, &[6, 0x8A]));
    bytes.extend(frame(0x0A, 0x04, &ver_payload(2)));
    let (messages, _) = parse_all(&bytes);
    assert_eq!(messages[0], Message::Ack { class: 6, id: 0x8A });
    assert_eq!(messages[1], Message::Nak { class: 6, id: 0x8A });
    let Message::Version(v) = messages[2] else { panic!() };
    assert_eq!(v.sw_str(), "EXT CORE 5.10");
    assert_eq!(v.hw_str(), "000A0000");
    assert!(Version::decode(&[0; 39]).is_none());
    assert!(Version::decode(&[0; 41]).is_none());
}

// ---------------------------------------------------------------- parser

#[test]
fn parser_resync_and_counters() {
    let mut bytes = b"$GNGGA,1,2,3*00\r\n\xB5\xB5".to_vec();
    bytes.extend(frame(5, 1, &[6, 0x8A]));
    // Bad checksum.
    let mut bad = frame(5, 1, &[6, 0x8A]);
    *bad.last_mut().unwrap() ^= 0xFF;
    bytes.extend(&bad);
    // Valid but not decoded.
    bytes.extend(frame(0x0A, 0x09, &[1, 2, 3]));
    // Oversize (300-byte payload), skipped and counted.
    bytes.extend(frame(0x02, 0x15, &[0u8; 300]));
    bytes.extend(frame(5, 0, &[6, 0x8A]));
    let (messages, stats) = parse_all(&bytes);
    assert_eq!(
        messages,
        [Message::Ack { class: 6, id: 0x8A }, Message::Nak { class: 6, id: 0x8A }]
    );
    assert_eq!(stats.bad_checksum, 1);
    assert_eq!(stats.oversize, 1);
    assert_eq!(stats.ignored, 1);
    assert_eq!(stats.frames, 3);
}

#[test]
fn parser_finds_frame_inside_false_frame() {
    // A false header announcing 5 payload bytes swallows the start of a real frame.
    let mut bytes = vec![0xB5, 0x62, 0x0A, 0x04, 0x05, 0x00];
    bytes.extend(frame(5, 1, &[6, 0x8A]));
    bytes.extend(frame(5, 0, &[1, 2]));
    let (messages, stats) = parse_all(&bytes);
    assert_eq!(
        messages,
        [Message::Ack { class: 6, id: 0x8A }, Message::Nak { class: 1, id: 2 }]
    );
    assert!(stats.bad_checksum >= 1);
}

#[test]
fn parser_returns_on_checksum_byte() {
    let mut bytes = pvt_frame();
    bytes.extend(frame(5, 1, &[1, 2]));
    let mut parser = Parser::new();
    let mut input = &bytes[..];
    let first = parser.next(&mut input);
    assert!(matches!(first, Some(Message::Pvt(_))));
    assert_eq!(input.len(), 10);
}

#[derive(Clone, Debug)]
enum Item {
    Ack(u8, u8),
    Nak(u8, u8),
    Ver(usize),
    Pvt(Vec<u8>),
    Other(u8, u8, Vec<u8>),
    Garbage(Vec<u8>),
}

fn item() -> impl Strategy<Value = Item> {
    let byte = prop_oneof![3 => any::<u8>(), 1 => Just(0xB5u8), 1 => Just(0x62u8)];
    prop_oneof![
        (any::<u8>(), any::<u8>()).prop_map(|(a, b)| Item::Ack(a, b)),
        (any::<u8>(), any::<u8>()).prop_map(|(a, b)| Item::Nak(a, b)),
        (0usize..4).prop_map(Item::Ver),
        proptest::collection::vec(any::<u8>(), PVT_LEN).prop_map(Item::Pvt),
        (0x20u8..0x80, any::<u8>(), proptest::collection::vec(any::<u8>(), 0..40))
            .prop_map(|(c, i, p)| Item::Other(c, i, p)),
        proptest::collection::vec(byte, 0..24).prop_map(Item::Garbage),
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    #[test]
    fn stream_any_chunking(
        items in proptest::collection::vec(item(), 0..24),
        cuts in proptest::collection::vec(1usize..40, 1..64),
    ) {
        let mut bytes = Vec::new();
        let mut expected = Vec::new();
        for item in &items {
            match item {
                Item::Ack(c, i) => {
                    bytes.extend(frame(5, 1, &[*c, *i]));
                    expected.push(Message::Ack { class: *c, id: *i });
                }
                Item::Nak(c, i) => {
                    bytes.extend(frame(5, 0, &[*c, *i]));
                    expected.push(Message::Nak { class: *c, id: *i });
                }
                Item::Ver(n) => {
                    let p = ver_payload(*n);
                    bytes.extend(frame(0x0A, 0x04, &p));
                    expected.push(Message::Version(Version::decode(&p).unwrap()));
                }
                Item::Pvt(p) => {
                    bytes.extend(frame(1, 7, p));
                    expected.push(Message::Pvt(Pvt::decode(p).unwrap()));
                }
                Item::Other(c, i, p) => bytes.extend(frame(*c, *i, p)),
                Item::Garbage(g) => bytes.extend(g),
            }
        }
        // A false header can hold the frames after it back until it completes: up to 264 more bytes.
        bytes.extend([0x20u8; 300]);
        let mut parser = Parser::new();
        let mut got = Vec::new();
        let mut at = 0;
        let mut k = 0;
        while at < bytes.len() {
            let end = (at + cuts[k % cuts.len()]).min(bytes.len());
            k += 1;
            let mut input = &bytes[at..end];
            while !input.is_empty() {
                while let Some(m) = parser.next(&mut input) {
                    got.push(m);
                }
            }
            at = end;
        }
        let mut empty: &[u8] = &[];
        while let Some(m) = parser.next(&mut empty) {
            got.push(m);
        }
        prop_assert_eq!(got, expected);
    }

    #[test]
    fn arbitrary_bytes_never_panic(bytes in proptest::collection::vec(any::<u8>(), 0..2000)) {
        let (_, stats) = parse_all(&bytes);
        prop_assert!(stats.frames <= 2000);
    }
}

// ---------------------------------------------------------------- fake receiver, UART and delay

#[derive(Default)]
struct Fake {
    host_baud: u32,
    /// Baud rate the receiver listens at.
    rx_baud: u32,
    /// Ignores the baud-rate CFG-VALSET.
    ignore_baud_set: bool,
    /// Answers the configuration CFG-VALSET with ACK-NAK.
    nak: bool,
    /// Never answers the configuration CFG-VALSET.
    no_ack: bool,
    /// Answers nothing at all.
    dead: bool,
    /// Every read, write, flush and delay returns Pending once first.
    yield_ops: bool,
    /// At most this many bytes per read (0: no limit).
    chunk: usize,
    to_host: VecDeque<u8>,
    from_host: Vec<u8>,
    /// Payloads of every CFG-VALSET seen at the right baud rate.
    valsets: Vec<Vec<u8>>,
    bauds: Vec<u32>,
}

impl Fake {
    fn at(rx_baud: u32) -> Self {
        Self { rx_baud, host_baud: 9600, ..Self::default() }
    }

    fn reply(&mut self, bytes: Vec<u8>) {
        self.to_host.extend(bytes);
    }

    fn handle(&mut self) {
        loop {
            let Some(start) = self.from_host.iter().position(|&b| b == 0xB5) else {
                self.from_host.clear();
                return;
            };
            self.from_host.drain(..start);
            if self.from_host.len() < 6 {
                return;
            }
            let len = usize::from(u16::from_le_bytes([self.from_host[4], self.from_host[5]]));
            if self.from_host.len() < len + 8 {
                return;
            }
            let msg: Vec<u8> = self.from_host.drain(..len + 8).collect();
            if self.dead || self.host_baud != self.rx_baud {
                continue;
            }
            let payload = msg[6..6 + len].to_vec();
            match (msg[2], msg[3]) {
                (0x0A, 0x04) => {
                    self.reply(b"$GNTXT,01,01,02,u-blox ag*00\r\n".to_vec());
                    self.reply(frame(0x0A, 0x04, &ver_payload(2)));
                }
                (0x06, 0x8A) => {
                    self.valsets.push(payload.clone());
                    let key = u32::from_le_bytes(payload[4..8].try_into().unwrap());
                    if key == config::KEY_UART1_BAUDRATE {
                        if !self.ignore_baud_set {
                            self.rx_baud = u32::from_le_bytes(payload[8..12].try_into().unwrap());
                        }
                        self.reply(frame(5, 1, &[6, 0x8A]));
                    } else if self.nak {
                        self.reply(frame(5, 0, &[6, 0x8A]));
                    } else if !self.no_ack {
                        self.reply(frame(5, 1, &[6, 0x8A]));
                        self.reply(pvt_frame());
                        self.reply(pvt_frame());
                    }
                }
                _ => {}
            }
        }
    }
}

struct YieldOnce(bool);

impl Future for YieldOnce {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if self.0 {
            Poll::Ready(())
        } else {
            self.0 = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

impl embedded_io::ErrorType for Fake {
    type Error = embedded_io::ErrorKind;
}

impl SetBaud for Fake {
    fn set_baud(&mut self, baud: u32) {
        self.host_baud = baud;
        self.bauds.push(baud);
        self.to_host.clear();
        self.from_host.clear();
    }
}

impl embedded_io_async::Read for Fake {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if self.yield_ops {
            YieldOnce(false).await;
        }
        // Empty line: stay pending until the caller gives up.
        core::future::poll_fn(|cx| {
            if self.to_host.is_empty() {
                cx.waker().wake_by_ref();
                return Poll::Pending;
            }
            let max = if self.chunk == 0 { buf.len() } else { self.chunk.min(buf.len()) };
            let mut n = 0;
            while n < max {
                let Some(b) = self.to_host.pop_front() else { break };
                buf[n] = b;
                n += 1;
            }
            Poll::Ready(Ok(n))
        })
        .await
    }
}

impl embedded_io_async::Write for Fake {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if self.yield_ops {
            YieldOnce(false).await;
        }
        self.from_host.extend_from_slice(buf);
        self.handle();
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        if self.yield_ops {
            YieldOnce(false).await;
        }
        Ok(())
    }
}

/// Delay that records the request and completes after one Pending.
struct FakeDelay(Vec<u32>);

impl embedded_hal_async::delay::DelayNs for FakeDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.push(ns / 1_000_000);
        // Longer than any single UART operation takes in polls, so a timeout is never first.
        for _ in 0..20 {
            YieldOnce(false).await;
        }
    }
}

fn configure(fake: Fake) -> (Receiver<Fake>, Result<Version, Error<embedded_io::ErrorKind>>, Vec<u32>) {
    let mut receiver = Receiver::new(fake);
    let mut delay = FakeDelay(Vec::new());
    let result = block_on(receiver.configure(&Config::default(), &mut delay));
    (receiver, result, delay.0)
}

// ---------------------------------------------------------------- driver

#[test]
fn configures_from_each_baud() {
    for start in [9600, 38_400, 115_200] {
        let (mut receiver, result, delays) = configure(Fake::at(start));
        let version = result.unwrap_or_else(|e| panic!("start {start}: {e:?}"));
        assert_eq!(version.sw_str(), "EXT CORE 5.10");
        assert_eq!(receiver.version(), Some(version));
        assert!(receiver.is_configured());
        let fake = &receiver.uart;
        assert_eq!((fake.host_baud, fake.rx_baud), (115_200, 115_200));
        let settings = Config::default().valset().unwrap();
        assert_eq!(fake.valsets.last().unwrap(), settings.payload());
        let baud_sets = fake.valsets.len() - 1;
        assert_eq!(baud_sets, usize::from(start != 115_200), "start {start}");
        // Reply waits of 1000 ms for the probes before the right rate, and the settle at the end.
        assert_eq!(delays.last(), Some(&500));
        if start != 115_200 {
            assert!(delays.contains(&100));
        }
        let pvt = block_on(receiver.read_pvt()).unwrap();
        assert!(pvt.is_3d_fix_ok());
        assert_eq!(pvt.itow, 0x0102_0304);
    }
}

#[test]
fn probes_in_order() {
    let (receiver, result, _) = configure(Fake::at(38_400));
    result.unwrap();
    assert_eq!(receiver.uart.bauds, [9600, 38_400, 115_200]);
}

#[test]
fn no_receiver() {
    let (mut receiver, result, delays) = configure(Fake { dead: true, ..Fake::at(9600) });
    assert_eq!(result, Err(Error::NoReceiver));
    assert_eq!(receiver.uart.bauds, [9600, 38_400, 115_200]);
    assert_eq!(delays, [1000, 1000, 1000]);
    assert!(!receiver.is_configured());
    assert_eq!(block_on(receiver.read_pvt()), Err(Error::NotConfigured));
}

#[test]
fn nak_is_rejected() {
    let (mut receiver, result, _) = configure(Fake { nak: true, ..Fake::at(115_200) });
    assert_eq!(result, Err(Error::Rejected));
    assert!(!receiver.is_configured());
    assert_eq!(block_on(receiver.read_pvt()), Err(Error::NotConfigured));
}

#[test]
fn silence_after_config_is_no_ack() {
    let (receiver, result, _) = configure(Fake { no_ack: true, ..Fake::at(115_200) });
    assert_eq!(result, Err(Error::NoAck));
    assert!(!receiver.is_configured());
}

#[test]
fn baud_switch_not_followed() {
    let (receiver, result, _) = configure(Fake { ignore_baud_set: true, ..Fake::at(38_400) });
    assert_eq!(result, Err(Error::BaudSwitchFailed));
    assert!(!receiver.is_configured());
}

#[test]
fn invalid_config_sends_nothing() {
    let mut receiver = Receiver::new(Fake::at(9600));
    let mut delay = FakeDelay(Vec::new());
    let config = Config { meas_rate_ms: 123, ..Config::default() };
    assert_eq!(block_on(receiver.configure(&config, &mut delay)), Err(Error::InvalidConfig));
    assert!(receiver.uart.bauds.is_empty());
}

#[test]
fn pvt_one_byte_at_a_time_returns_on_last_byte() {
    let (mut receiver, result, _) = configure(Fake { chunk: 1, ..Fake::at(115_200) });
    result.unwrap();
    // Two PVTs are queued; the second must still be there after the first is returned.
    block_on(receiver.read_pvt()).unwrap();
    assert_eq!(receiver.uart.to_host.len() + (receiver.end - receiver.pos), 100);
    block_on(receiver.read_pvt()).unwrap();
}

#[test]
fn stale_bytes_and_noise_are_skipped() {
    let (mut receiver, result, _) = configure(Fake::at(115_200));
    result.unwrap();
    let mut noise = b"$GNRMC,garbage\r\n".to_vec();
    noise.extend(frame(5, 1, &[1, 2]));
    noise.extend(pvt_frame());
    receiver.uart.to_host.extend(noise);
    // Two queued PVTs from configure, then noise and a third.
    for _ in 0..3 {
        block_on(receiver.read_pvt()).unwrap();
    }
}

/// A3: drop `configure` after k polls for every k; the driver is never configured then, never
/// panics, and a new `configure` works.
#[test]
fn drop_at_every_await_leaves_not_configured() {
    for start in [9600, 115_200] {
        let mut completed_at = None;
        for k in 0..4000 {
            let mut receiver = Receiver::new(Fake { yield_ops: true, ..Fake::at(start) });
            let mut delay = FakeDelay(Vec::new());
            let waker = noop_waker();
            let mut cx = Context::from_waker(&waker);
            let mut done = false;
            {
                let config = Config::default();
                let future = receiver.configure(&config, &mut delay);
                let mut future = pin!(future);
                for _ in 0..k {
                    if let Poll::Ready(result) = future.as_mut().poll(&mut cx) {
                        assert!(result.is_ok());
                        done = true;
                        break;
                    }
                }
            }
            if done {
                completed_at = Some(k);
                assert!(receiver.is_configured());
                break;
            }
            assert!(!receiver.is_configured(), "start {start} k {k}");
            assert!(receiver.version().is_none());
            assert_eq!(block_on(receiver.read_pvt()), Err(Error::NotConfigured));
            // Usable again.
            let mut delay = FakeDelay(Vec::new());
            block_on(receiver.configure(&Config::default(), &mut delay)).unwrap_or_else(|e| panic!("start {start} k {k}: {e:?} {:?}", receiver.uart.bauds));
            assert!(receiver.is_configured());
            block_on(receiver.read_pvt()).unwrap();
        }
        assert!(completed_at.is_some_and(|k| k > 10), "start {start}: {completed_at:?}");
    }
}

#[test]
fn reconfigure_clears_state_first() {
    let (mut receiver, result, _) = configure(Fake::at(115_200));
    result.unwrap();
    receiver.uart.dead = true;
    let mut delay = FakeDelay(Vec::new());
    assert_eq!(
        block_on(receiver.configure(&Config::default(), &mut delay)),
        Err(Error::NoReceiver)
    );
    assert_eq!(block_on(receiver.read_pvt()), Err(Error::NotConfigured));
}
