# CLAUDE.md — sdg2122 MIDAS frontend

## What this is

C++ MIDAS frontend for the **SIGLENT SDG2122X** dual-channel function/arbitrary
waveform generator. Follows the sibling SPD3303X frontend (`../spd3303x/`)
skeleton: same ODB layout philosophy, periodic equipment, history Variables and
event banks — different SCPI command set and a deliberately different USB
transport (see **Transport** below).

The Python control library (`sdg2122-cli` at `/home/muon/Installs/sdg2122-cli/`)
is the source of truth for the SCPI command set (`sdg2122_cli/catalog.py` +
`parsing.py`). This frontend implements the subset the ODB exposes and mirrors
the CLI's *wire* conventions: quantities are sent as base-unit NR3 numbers
without unit suffixes (`C1:BSWV FRQ,1000`); readbacks echo units glued to the
number (`FRQ,1000HZ`, `AMP,6.6V`) and are parsed with SI-multiplier awareness.

## Scope

**Currently: core wave control only.** Per channel: Output, Wave Type,
Frequency, Amplitude, Offset, Phase, Duty, Symmetry, Pulse Width, Load,
Polarity, Arb Index, plus readbacks, Status string, history and one event
bank per device. Kept deliberately small to debug against the bench.

**Modulation / sweep / burst / harmonic / frequency counter (and the
`MODE`/`ROSC` globals) are implemented but parked** in the backup source
`sdg2122_fe_full.cxx` (with the matching full-version doc in
`CLAUDE.md.full-backup`). Once the core behaves well, diff those back in.
Upgrading from the backed-up full tree already in ODB: delete
`/Equipment/SDG2122X` once and let the trimmed frontend rebuild it.

## Build

```bash
make                # produces ./sdg2122_fe
make clean
```

Requires `g++` (C++17), the MIDAS tree at `/home/muon/Installs/midas/` and
`libusb-1.0` dev headers. Edit `MIDASSYS` in the `Makefile` if the MIDAS path
moves; adjust `pkg-config libusb-1.0` if needed. The build links
`-lusb-1.0` (no `-ludev` — libusb pulls it).

## Transport (why not the kernel usbtmc driver)

The SDG2122X firmware on this bench (seller ID `f4ec:1102`, firmware
`2.01.01.37R8`) is broken in two documented ways:

1. It rejects the USBTMC class `CLEAR` request and returns legacy-query
   responses tagged with a **stale USBTMC bTag**. The Linux kernel usbtmc
   driver discards those frames, so `/dev/usbtmcN` reads come back empty —
   the first query sometimes succeeds, the rest time out. (Verified: `*IDN?`
   once via the char device, then empty on the next query.)
2. It **defers** one response per command — plain `write` and `query`
   commands alike — so a naive write-then-read protocol always reads the
   *previous* command's delayed answer.

The CLI works around this with libusb framing + double-writes; the frontend
instead uses the simpler and empirically reliable **write-once + drain-until-
match** discipline, which is what this unit's deferred pipeline actually needs:

- Every command goes out with its own USBTMC bulk-out frame (we build the
  12-byte header ourselves; the kernel driver is detached).
- After a `query`, we keep issuing bulk-in request frames until a response
  whose header matches the outstanding query arrives; stale prefix responses
  (leftovers from previous commands) are discarded.
- After a plain `write`, we do a short bounded drain so a deferred echo cannot
  contaminate the next query.
- The four-byte all-zero packet this firmware emits (while applying coupling
  parameters) is dropped.

This is verified end-to-end by a read-only harness: compile the real
`sdg2122_fe.cxx` (no `main()` — mfe provides it) with a test main that links
libmidas + libusb and exercises `parse_scpi`/`discover_sdg2122`/`ScpiUsb`
against the live device. The transport passes 10 live queries + 19 parser
asserts.

### LAN fallback

If `Device0/Host` is set to anything that does not start with `/dev/usbtmc`,
the frontend uses raw SCPI over TCP/5025 (the SDG2000X family has LAN). No
framing, no deferred responses, no drains — plain `cmd\n` in, response until
`\n` out. Auto-discovery fills `Host` with a `/dev/usbtmcN` path only.

## USB permissions

The unit is `f4ec:1102` (the sdg2122-cli udev rule names `f4ec:ee38`; add your
unit's product id if the node is not world-accessible). On this bench
`/dev/usbtmc0` is `0666`, so the frontend can open it directly. If a rule is
needed:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="f4ec", MODE="0666", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usbmisc", KERNEL=="usbtmc*", ATTRS{idVendor}=="f4ec", MODE="0666", GROUP="plugdev", TAG+="uaccess"
```

The frontend detaches the kernel usbtmc driver while it holds the interface
(standard libusb) and re-attaches it on close.

## Run

```
./sdg2122_fe [-h <hostname>] [-e <experiment>] [-d] [-v N]
```

Start it under the experiment (mhttpd → Programs → Start, or a shell with
`MIDAS_EXPTAB`/`-e`). The device tree appears at
`/Equipment/SDG2122X00/Settings/...`.

**First connect applies the ODB defaults to the instrument** (SINE, 1 kHz,
2 Vpp, 0 V offset, HZ load, polarity NOR, everything else off/disabled, both
outputs OFF). This matches the SPD3303X frontend: ODB is the source of truth
once the frontend is running. If the instrument was left on a custom waveform
from the front panel, the first poll writes the defaults over it; outputs stay
OFF so it is electrically safe, but the previous manual configuration is not
restored.

**Settings survive frontend restarts.** The `Settings/DeviceN` subtree is built
only when it does not already exist — `connect_and_fix_structure` with the
`{"CH1", {}}` outer wallet deletes existing CH1 children on a re-run (any key
not represented in the wallet is deleted), so re-running it would reset every
user-set value to its default on each start. This is a deliberate divergence
from the SPD3303X frontend, which restructures on every start and therefore
resets its Vset/Iset to defaults on restart. To repair a half-built subtree
after a schema change, delete `/Equipment/SDG2122X/Settings/Device0` and
restart — it is recreated with defaults.

Equipment name: without `-i`, mfe strips `%02d`, so the subtree lives at
`/Equipment/SDG2122X` (add `-i 0` → `SDG2122X00`).

Only run one controller at a time: the frontend, the `sdg2122` CLI, and the
instrument's front panel all fight over the same session. The deferred-
response protocol assumes no other session interleaves.

## SCPI command set used

Everything is channel-prefixed with `C1:`/`C2:`. (The clipped `MODE`, `ROSC`,
`FCNT`, `MDWV`, `SWWV`, `BTWV`, `HARM` commands live in the backup source.)

| Function                    | Command (example)                              |
|-----------------------------|-------------------------------------------------|
| Probe                       | `*IDN?` → contains `SDG2122`                    |
| Output on/off               | `C1:OUTP ON` / `C1:OUTP OFF`                    |
| Output load                 | `C1:OUTP LOAD,HZ` / `C1:OUTP LOAD,50`           |
| Output polarity             | `C1:OUTP PLRT,NOR` / `INVT`                     |
| Output snapshot             | `C1:OUTP?` → `C1:OUTP OFF,LOAD,HZ,PLRT,NOR`     |
| Basic wave per field        | `C1:BSWV WVTP,SINE`, `FRQ,1000`, `AMP,2`, `OFST,0`, `PHSE,0`, `DUTY,50`, `SYM,50`, `WIDTH,0.0001` |
| Basic wave snapshot         | `C1:BSWV?` → `C1:BSWV WVTP,PULSE,FRQ,1000HZ,PERI,0.001S,AMP,6.6V,AMPVRMS,3.3Vrms,AMPDBM,0dBm,MAX_OUTPUT_AMP,20V,OFST,3.3V,HLEV,6.6V,LLEV,0V,...` |
| Arbitrary wave              | `C1:ARWV INDEX,N` / `C1:ARWV?` → `INDEX,2,NAME,StairUp` |

Per-field BSWV writes are kept **separate** (not combined into one command) —
the CLI found this firmware wedges on combined BSWV writes followed by a query.
`LOAD` is applied before amplitude/offset because switching `HZ`↔`50` rescales
the displayed `AMP` convention by two.

## ODB layout

```
/Equipment/SDG2122X/
  Settings/
    Device0/
      Host:           "/dev/usbtmc0"   # or "192.168.x.y" for LAN
      Status:         "CH1 OFF | SINE 1kHz 2Vpp; CH2 OFF | SINE 1kHz 2Vpp"
      CH1/  (CH2 identical)
        Output:              bool      # write to enable/disable
        Wave Type:           SINE|SQUARE|RAMP|PULSE|NOISE|ARB|DC   (dropdown)
        Frequency (Hz)       float     # write to apply
        Amplitude (Vpp)      float
        Offset (V)           float
        Phase (deg)          float
        Duty (%) / Symmetry (%) / Pulse Width (s)  float
        Load:                HZ | 50   (dropdown)
        Polarity:            NOR | INVT
        Arb Index:           int 1..196 (CH2 has a firmware table quirk, see below)
        # --- readbacks (frontend-written) ---
        Wave Rdbk / Freq Rdbk (Hz) / Amp Rdbk (Vpp) / Offs Rdbk (V)
          / Phase Rdbk (deg) / Load Rdbk / Arb Name Rdbk
  Variables/
    Freq_B0_CH1, Amp_B0_CH1, Offs_B0_CH1, Phase_B0_CH1, Output_B0_CH1
    ...same for CH2
    ... same pattern for B1..B3 if those devices exist
```

The full (mod/sweep/burst/harmonic/counter) layout is in `CLAUDE.md.full-backup`.

### Adding more devices

Same pattern as SPD3303X: create `Settings/Device1` (DIR) with the same
children, restart the frontend (or edit `Host` — `num_devices` is recomputed
at startup). Up to `SDG2122X_MAX_DEV = 4`.

## Poll cycle (every 2000 ms, change-driven)

1. Read ODB settings (`Host`, then per-channel core keys).
2. Connect/reconnect if `Host` changed. On (re)connect the per-channel caches
   reset to `-1`, so the next poll re-applies ODB settings.
3. Per channel, in this order (matches the CLI's `output` workflow):
   `OUTP LOAD` → `ARWV INDEX` (only when Wave Type==ARB) → `BSWV WVTP` →
   `BSWV FRQ/AMP/OFST/PHSE/DUTY/SYM/WIDTH` → `OUTP PLRT` → `OUTP ON|OFF`
   **last**. Only keys whose cached value changed are written (steady state is
   silent). An amplitude-rail guard blocks `AMP/OFST` writes that would exceed
   ±10 V (HZ) or ±5 V (50 Ω) — same numbers as the CLI.
4. Read back per channel: `OUTP?` (state — if it disagrees with ODB, e.g. the
   front panel was used, ODB `Output` is reconciled), `BSWV?` (fills all
   `Rdbk` keys from the device), `ARWV?` (arb name).
5. Compose the `Status` string; push history Variables; emit one event bank
   `VAR0`/`VAR1`/… per device containing **10 floats**:

   ```
   [CH1.freq, CH1.amp, CH1.offset, CH1.phase, CH1.out,
    CH2.freq, CH2.amp, CH2.offset, CH2.phase, CH2.out]
   ```

## Known quirks / limits

- **CH2 arbitrary-wave table**: this firmware omits built-ins M29–M31 on CH2
  and shifts the write-side table by 3 above that; `arb_wire_index` mirrors the
  CLI's `_arb_wire_index`.
- **Writes are not individually readback-verified.** On failure the device
  disagrees with the ODB setpoint, which shows up in the `Rdbk` keys on the
  next poll — soft verification, same spirit as SPD3303X's monitor fields.
- **Recovery**: the frontend re-drains on every open, disconnects cleanly on
  read errors, and retries the whole open+`*IDN?` probe up to 4 times on
  connect (this firmware is erratic right after USB re-enumeration). A
  genuinely wedged endpoint (user-wave upload, firmware lockup) still needs
  the `sdg2122 recover` ladder (`/home/muon/Installs/sdg2122-cli`, needs
  `--backend libusb` on the `1102` unit because `auto` only selects libusb
  for `ee38`).
- Reconnect is `Host`-change driven
  (`device_initialized`/`last_host`), like SPD3303X — unplugging the cable
  leaves `Status = Not connected` until `Host` is edited.

## Self-test

```bash
g++ -O2 -std=c++17 -pthread -I$(MIDASSYS)/include $(MIDASSYS)/mxml $(MIDASSYS)/mjson \
    -D_DEFAULT_SOURCE -D_XOPEN_SOURCE=600 -I. /tmp/sdg2122_selftest.cxx \
    -L$(MIDASSYS)/lib -lmidas $(MIDASSYS)/lib/libmidas.a $(pkg-config --libs libusb-1.0) \
    -lrt -pthread -lm -ldl -rdynamic -o /tmp/sdg2122_selftest
timeout 90 /tmp/sdg2122_selftest /dev/usbtmc0
```

Read-only against the device; run it whenever the transport is touched.