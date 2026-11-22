//! On-board check of the sam-m10q driver: SAM-M10Q on UART1 (GPIO24 = TX to the module, GPIO25 = RX from
//! the module, 8N1). `configure` runs with the default `Config` except the baud candidates, probed
//! 115200, 9600, 38400 (the firmware's order); it leaves the receiver at 115200, 5 Hz, UBX only, RAM
//! layer only (nothing is saved). Then NAV-PVT is read for 10 s. Fix quality is reported, never judged
//! (indoors there is no fix), and position values are never printed.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_time::{Delay, Duration, Instant, with_timeout};
use embedded_io_async::{ErrorType, Read, Write};
use panic_probe as _;
use sam_m10q::{Config, Receiver, SetBaud};

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

const RX_BUFFER: usize = 4096;
const TX_BUFFER: usize = 64;

/// Seconds of the NAV-PVT count, and the frames expected in them at 200 ms.
const WINDOW_SECS: u64 = 10;
const FRAMES_MIN: u32 = 48;
const FRAMES_MAX: u32 = 52;

/// `BufferedUart` with the baud rate settable, and a read that skips receive errors (a framing error is
/// expected while the baud rate is being probed) after a yield.
struct GnssUart(BufferedUart);

impl ErrorType for GnssUart {
    type Error = uart::Error;
}

impl Read for GnssUart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            match self.0.read(buf).await {
                Ok(n) => return Ok(n),
                Err(_) => {}
            }
            yield_now().await;
        }
    }
}

impl Write for GnssUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf).await
    }

    /// Returns when the last byte has left the UART: the driver changes the baud rate right after.
    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush().await?;
        while self.0.busy() {
            embassy_time::Timer::after(Duration::from_micros(200)).await;
        }
        Ok(())
    }
}

impl SetBaud for GnssUart {
    fn set_baud(&mut self, baud: u32) {
        self.0.set_baudrate(baud);
    }
}

/// Reports one verdict line; the format string starts with `{=str}`, which receives the tag.
macro_rules! check {
    ($failures:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "[PASS]" $(, $arg)*);
        } else {
            $failures += 1;
            defmt::error!($fmt, "[FAIL]" $(, $arg)*);
        }
    };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;

    let mut tx_buf = [0u8; TX_BUFFER];
    let mut rx_buf = [0u8; RX_BUFFER];
    let uart = BufferedUart::new(
        p.UART1,
        p.PIN_24,
        p.PIN_25,
        Irqs,
        &mut tx_buf,
        &mut rx_buf,
        uart::Config::default(),
    );
    let mut gnss = Receiver::new(GnssUart(uart));

    let config = Config {
        candidate_bauds: [115_200, 9_600, 38_400],
        ..Config::default()
    };
    let configured = gnss.configure(&config, &mut Delay).await;
    check!(
        failures,
        configured.is_ok(),
        "{=str} configure (default Config, bauds 115200, 9600, 38400)"
    );
    match configured {
        Ok(version) => {
            defmt::info!(
                "[INFO] MON-VER sw '{=str}' hw '{=str}'",
                version.sw_str(),
                version.hw_str()
            );
        }
        Err(e) => {
            defmt::error!("[INFO] configure error: {}", e);
            hil_common::exit(failures);
        }
    }

    let before = gnss.stats();
    let mut frames = 0u32;
    let mut last = None;
    let end = Instant::now() + Duration::from_secs(WINDOW_SECS);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, gnss.read_pvt()).await {
            Ok(Ok(pvt)) => {
                frames += 1;
                last = Some(pvt);
            }
            Ok(Err(e)) => {
                defmt::error!("[INFO] read_pvt error: {}", e);
                break;
            }
            Err(_) => break,
        }
    }
    let after = gnss.stats();
    let bad_checksum = after.bad_checksum - before.bad_checksum;
    let oversize = after.oversize - before.oversize;
    let ignored = after.ignored - before.ignored;

    check!(
        failures,
        (FRAMES_MIN..=FRAMES_MAX).contains(&frames),
        "{=str} NAV-PVT frames in 10 s: {=u32} (expect 48..=52)",
        frames
    );
    check!(
        failures,
        bad_checksum == 0,
        "{=str} checksum errors: {=u32}",
        bad_checksum
    );
    defmt::info!(
        "[INFO] driver stats during the 10 s: bad_checksum {=u32}, oversize {=u32}, ignored {=u32}",
        bad_checksum,
        oversize,
        ignored
    );
    match last {
        Some(pvt) => defmt::info!(
            "[INFO] last frame: fixType {=u8}, numSV {=u8}, hAcc {=u32} mm, vAcc {=u32} mm",
            pvt.fix_type,
            pvt.num_sv,
            pvt.h_acc,
            pvt.v_acc
        ),
        None => defmt::info!("[INFO] no NAV-PVT frame received"),
    }

    hil_common::exit(failures)
}

