# sam-m10q

Driver for the u-blox SAM-M10Q GNSS receiver on a UART. UBX protocol only: NMEA is switched off. `no_std`, no
allocation, no panics outside tests, on the `embedded-io-async` 0.7 and `embedded-hal-async` 1.0 traits (optional
`defmt`). Works with any executor or none.

* Sans-IO core: `ubx::Parser` (streaming, resynchronises on garbage, counts bad checksums, ignored and oversize
  frames), `ubx::write_frame`, `config::ValSet`.
* Async driver: `Receiver<UART>` with `configure` and `read_pvt`.

Decoded: UBX-ACK-ACK, ACK-NAK, MON-VER (software and hardware strings) and NAV-PVT (all 92 bytes).

```ignore
let mut gnss = sam_m10q::Receiver::new(uart); // uart: Read + Write + sam_m10q::SetBaud
let version = gnss.configure(&sam_m10q::Config::default(), &mut delay).await?;
loop {
    let pvt = gnss.read_pvt().await?;
    if pvt.is_3d_fix_ok() { /* lat/lon in 1e-7 degree, height mm, velocities mm/s */ }
}
```

## configure

The driver cannot change the UART's baud rate, so the UART type implements `SetBaud` (`fn set_baud(&mut self, u32)`).
Flow: for each candidate baud (9600, 38400, 115200): set it, poll MON-VER, wait up to 1000 ms for the reply; none
at any rate gives `Error::NoReceiver`. If the rate found is not the target (115200), send a CFG-VALSET with
CFG-UART1-BAUDRATE (its acknowledgement is not required), set the UART to the target, wait 100 ms and poll MON-VER
again; the reply is the success check (`Error::BaudSwitchFailed` otherwise). Then one CFG-VALSET (layer RAM only,
nothing saved) with the UBX-only protocol switches, 5 Hz rate, NAV-PVT output on UART1, dynamic model 8, static hold
off and the signal set; ACK-ACK is required (`Error::Rejected` on NAK, `Error::NoAck` on silence). Last it waits
500 ms because a signal change restarts the GNSS. Every default is a named constant in `config` with its source.

`read_pvt` returns `Error::NotConfigured` until `configure` has completed (the flag is cleared before its first await
and set after its last). It returns on the checksum byte of the NAV-PVT frame.

## Checks

`cargo test` (host): checksum and VALSET golden bytes, NAV-PVT golden decode, parser proptest with random chunking
and garbage, a fake receiver at 9600 / 38400 / 115200, `NoReceiver`, `Rejected`, `NoAck`, `BaudSwitchFailed`, and a
drop of `configure` at every poll. `cargo clippy --all-targets -- -D warnings`;
`cargo build --target thumbv8m.main-none-eabihf`. Board checks: see below. Not measured yet: the poll
time on the board.

## On the board (hil)

`hil/` is its own workspace (the only place that uses embassy-rp) with two programs for the MARV V2 board
(RP2350, SAM-M10Q on UART1: TX GPIO24, RX GPIO25). It flashes through a debug probe (probe-rs, chip RP235x);
`tools/run.sh` takes the shared lock `$XDG_RUNTIME_DIR/marv-v2-probe.lock`, so runs do not overlap.

* `tools/run.sh alive`: boots and prints the system clock (tool chain, probe and board check).
* `tools/run.sh sam_m10q_smoke`: `configure` with the default `Config` (bauds probed 115200, 9600, 38400), prints the
  MON-VER strings, then reads NAV-PVT for 10 s. PASS when `configure` is Ok, 48 to 52 frames arrive (50 at 200 ms)
  and there are no checksum errors. It also prints the oversize and ignored counts and the last frame's fixType,
  numSV, hAcc and vAcc (information only: indoors there may be no fix). It never prints a position.

The exit status is 0 on pass, 1 on fail. The `[PASS]`/`[FAIL]`/`[INFO]` lines are printed and the full log is kept
under `hil/logs/` (not committed). `hil/src/bin/sam_m10q_smoke.rs` holds the `SetBaud` newtype over
`embassy_rp::uart::BufferedUart`.
