# hil

Part of [sam-m10q](../README.md)

On-board test programs for the MARV V2 board (RP2350, SAM-M10Q on UART1: TX GPIO24, RX GPIO25). This folder is its own workspace and the only user of embassy-rp. `tools/run.sh <name> [timeout_s]` builds one program, flashes it through a debug probe (probe-rs, chip RP235x) under a lock that serialises probe use, and runs it. The exit status is 0 on pass, 1 on fail; `[PASS]`, `[FAIL]` and `[INFO]` lines are printed and the full log is kept under `hil/logs/`.

| Program | What it checks |
|---|---|
| `alive` | boots and prints the system clock (tool chain, probe and board) |
| `sam_m10q_smoke` | `configure` with the default `Config` (bauds probed 115200, 9600, 38400), prints the MON-VER strings, then reads NAV-PVT for 10 s. Passes when `configure` is Ok, 48 to 52 frames arrive (50 at 200 ms) and there are no checksum errors. It also prints the oversize and ignored counts and the last frame's fixType, numSV, hAcc and vAcc (information only: indoors there may be no fix); it never prints a position |

`hil/src/bin/sam_m10q_smoke.rs` holds the `SetBaud` newtype over `embassy_rp::uart::BufferedUart`.

## Related

[device](../docs/device.md) · [README](../README.md)
