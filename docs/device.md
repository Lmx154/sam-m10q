# SAM-M10Q device facts

Sources are the u-blox documents for the M10 family (SAM-M10Q data sheet, integration manual, M10 SPG 5.10
interface description "IM") and ArduPilot's `AP_GPS_UBLOX_CFG_List.h`. Nothing here is measured on the board yet.

- UBX frame: `B5 62 | class | id | len U2 LE | payload | CK_A CK_B`, 8-bit Fletcher over class to payload end.
  MON-VER poll: `B5 62 0A 04 00 00 0E 34`.
- CFG-VALSET (06 8A), version 0, layers 0x01 (RAM), 2 reserved bytes, then key U4 LE and value; value size from
  key bits 28..30 (1 and 2 one byte, 3 two bytes, 4 four bytes, 5 eight bytes); at most 64 pairs.
- ACK-ACK / ACK-NAK (05 01 / 05 00) answer within 1 s (IM 3.9). A baud-rate change takes about 100 ms (IM p22).
- A CFG-SIGNAL change restarts the GNSS; wait for the acknowledgement plus 0.5 s (integration manual).
- Measurement period 200, 500 or 1000 ms only for this constellation set (data sheet tables 1 and 2).
- Defaults of `Config`: see `src/config.rs` (each constant cites its source). ArduPilot's `config_M10` also sets
  CFG-SIGNAL-QZSS_L1S_ENA = 1; `Config` does not (the owner's list omits it).
- UNKNOWN until the board runs it: default baud of the module as delivered, whether NMEA is on at start, the
  real worst poll time of `Receiver`.
