# SDG2122X MIDAS Frontend

C++ [MIDAS](https://midas.triumf.ca/) frontend for the **SIGLENT SDG2122X**
dual-channel function / arbitrary waveform generator.

The frontend exposes the generator's core waveform controls as a MIDAS
*periodic equipment*: every 2 s it reads settings from the ODB, writes only the
values that changed to the instrument, reads the actual configuration back,
and records it in ODB history. It talks to the instrument over **USB-TMC using
libusb directly**, because this firmware's USBTMC responses are unusable
through the stock Linux `usbtmc` driver (see
[Why libusb?](#why-libusb-and-not-the-kernel-usbtmc-driver) below). A raw-SCPI
LAN transport (TCP port 5025) is also supported.

**Current scope: core wave control only** — output, wave type, frequency,
amplitude, offset, phase, duty, symmetry, pulse width, load, polarity and
arbitrary-wave index per channel. Modulation, sweep, burst, harmonic, the
frequency counter and the `MODE`/`ROSC` globals are implemented but parked in
`sdg2122_fe_full.cxx` / `CLAUDE.md.full-backup`; they will be merged back once
the core is validated on the bench.

---

## Features

- **Two channels**, each with independent control of:
  - Output on/off
  - Wave type: `SINE`, `SQUARE`, `RAMP`, `PULSE`, `NOISE`, `ARB`, `DC`
  - Frequency, amplitude (Vpp), DC offset, phase
  - Duty cycle, symmetry, pulse width
  - Output load (`HZ` high-impedance / `50` Ω) and polarity (`NOR`/`INVT`)
  - Built-in arbitrary-wave index (with the channel-2 firmware table quirk
    handled transparently)
- **Device readbacks** written back into ODB (`Wave Rdbk`, `Freq Rdbk (Hz)`,
  `Amp Rdbk (Vpp)`, `Offs Rdbk (V)`, `Phase Rdbk (deg)`, `Load Rdbk`,
  `Arb Name Rdbk`) — so you can see what the instrument actually reports.
- **Change-driven writes**: steady state produces no SCPI traffic; only edited
  ODB values are sent.
- **Amplitude rail guard** blocks amplitude/offset combinations that exceed the
  selected-load voltage rail (±10 V for `HZ`, ±5 V for `50 Ω`).
- **Status string**, per-device **history Variables**, and a **10-float event
  bank** (`VAR0`, `VAR1`, …) for MIDAS history logging.
- **Multi-device** support (`Device0`…`Device3`) and **auto-discovery** of the
  instrument at `/dev/usbtmc*`.
- **Settings survive frontend restarts** — the ODB subtree is only created when
  missing, never restructured.

## Requirements

- Linux (tested with kernel 7.0 / Ubuntu)
- `g++` with C++17
- A MIDAS installation (`libmidas.a`, `mfe.o`, headers)
- `libusb-1.0` development headers (`libusb-1.0-0-dev`, `pkg-config`)

## Build

```bash
make          # produces ./sdg2122_fe
make clean
```

The MIDAS path is set at the top of the `Makefile`:

```make
MIDASSYS = /home/muon/Installs/midas
```

Change it to match your installation. libusb flags come from
`pkg-config --cflags --libs libusb-1.0`.

## USB permissions

The Linux `usbtmc` driver creates `/dev/usbtmc*` as `root:root 0600`. To let a
non-root user run the frontend, install a udev rule
(`/etc/udev/rules.d/99-siglent-sdg2122.rules`):

```
SUBSYSTEM=="usb", ATTR{idVendor}=="f4ec", MODE="0666", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usbmisc", KERNEL=="usbtmc*", ATTRS{idVendor}=="f4ec", MODE="0666", GROUP="plugdev", TAG+="uaccess"
```

Then:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger        # or unplug/replug the generator
```

The vendor ID is Siglent's `f4ec`. Product IDs seen in the wild include `ee38`
and `1102`; the rule above matches by vendor so both work. For a one-shot test
without a rule: `sudo chmod 666 /dev/usbtmc0`.

The frontend detaches the kernel `usbtmc` driver while it holds the interface
(standard libusb behaviour) and re-attaches it on close.

## Run

```bash
./sdg2122_fe [-h <host>] [-e <experiment>] [-d] [-D] [-O] [-v <n>] [-i <n>]
```

| Flag | Meaning |
|------|---------|
| `-e <experiment>` | MIDAS experiment name (e.g. `-e miumiu`) |
| `-h <host>` | MIDAS host (empty = local shared memory) |
| `-d` | Debug / stay in the foreground |
| `-D` | Daemonise |
| `-O` | Daemonise but keep stdout |
| `-v <n>` | Verbosity |
| `-i <n>` | Frontend index (used for multi-frontend event building) |

Typical foreground run:

```bash
MIDAS_EXPTAB=/path/to/exptab ./sdg2122_fe -e <experiment> -d -v 2
```

Or register it in `mhttpd` → **Programs** → *Start*.

The equipment name is `SDG2122X%02d`. Without `-i`, `mfe` strips the `%02d`, so
the ODB tree lives at `/Equipment/SDG2122X`. With `-i 0` it becomes
`/Equipment/SDG2122X00`.

### First connect

On the first successful poll the frontend **applies the ODB defaults** to the
instrument: `SINE`, 1 kHz, 2 Vpp, 0 V offset, `HZ` load, `NOR` polarity,
both outputs **OFF**. This makes the ODB the source of truth once the frontend
is running. If the instrument was configured manually from the front panel, the
first poll overwrites that configuration (outputs stay off, so it is
electrically safe).

## ODB layout

```
/Equipment/SDG2122X/
  Settings/
    Device0/
      Host:     "/dev/usbtmc0"     # or an IP/hostname for LAN
      Status:   "CH1 ON | SQUARE 1kHz 2Vpp; CH2 OFF | SINE 1kHz 2Vpp"
      CH1/                           # CH2 is identical
        Output:              bool    # write to enable/disable
        Wave Type:           string  # dropdown (Options Wave Type)
        Frequency (Hz):      double
        Amplitude (Vpp):     double
        Offset (V):          double
        Phase (deg):         double
        Duty (%):            double
        Symmetry (%):        double
        Pulse Width (s):     double
        Load:                string  # dropdown: HZ | 50
        Polarity:            string  # dropdown: NOR | INVT
        Arb Index:           int     # 1..196
        # readbacks (frontend-written, do not edit)
        Wave Rdbk / Freq Rdbk (Hz) / Amp Rdbk (Vpp) / Offs Rdbk (V)
        / Phase Rdbk (deg) / Load Rdbk / Arb Name Rdbk
  Variables/
    Freq_B0_CH1, Amp_B0_CH1, Offs_B0_CH1, Phase_B0_CH1, Output_B0_CH1
    Freq_B0_CH2, ... (same for CH2)
    ... same pattern for B1..B3 if those devices exist
```

Dropdowns are rendered by `mhttpd` from the sibling `Options <key>` string
arrays.

### Adding a device

Create `Settings/Device1` (DIR) with the same children as `Device0` and
restart the frontend — `num_devices` is recomputed at startup. Up to
`SDG2122X_MAX_DEV = 4` devices are supported.

## MIDAS event bank

Every poll emits one bank per device, named `VAR0`, `VAR1`, … (equipment
history is enabled). The layout is **10 floats**:

```
[ CH1.freq, CH1.amp, CH1.offset, CH1.phase, CH1.out,
  CH2.freq, CH2.amp, CH2.offset, CH2.phase, CH2.out ]
```

## Why libusb and not the kernel usbtmc driver?

The SDG2122X firmware on our bench (`2.01.01.37R8`, USB `f4ec:1102`) has two
defects that break the standard `usbtmc` path:

1. **Stale USBTMC bTags.** It answers legacy queries with the bTag of the
   *previous* Bulk-IN request. The Linux kernel correctly rejects those frames
   and discards them, so `/dev/usbtmcN` reads come back empty — typically the
   first query succeeds and everything after it times out.
2. **Deferred responses.** It defers one response per command, so a naive
   write-then-read always reads the *previous* command's answer.

The frontend therefore speaks USBTMC itself through libusb:

- Every command is framed as a USBTMC `DEV_DEP_MSG_OUT` bulk transfer
  (12-byte header built by the frontend, 4-byte padded payload).
- For a query it writes the command **once**, then repeatedly issues Bulk-IN
  request frames and only accepts a response whose header matches the
  outstanding query. Stale responses are discarded; the loop retries until the
  right one arrives (bounded).
- Plain writes are followed by a short bounded drain so a deferred echo cannot
  contaminate the next query.
- The four-byte all-zero packet this firmware occasionally emits is dropped.

On connect the whole open + `*IDN?` probe is retried up to 4 times, because the
unit is erratic immediately after USB re-enumeration.

### LAN transport

If `Host` is set to anything that does not start with `/dev/usbtmc`, the
frontend uses raw SCPI over TCP port 5025 (the SDG2000X family supports LAN).
There is no USBTMC framing, no deferred responses and no drains — plain
`cmd\n` out, response terminated by `\n` in.

## SCPI commands used

All channel commands are prefixed `C1:` / `C2:`.

| Function | Example |
|---|---|
| Probe | `*IDN?` (must contain `SDG2122`) |
| Output on/off | `C1:OUTP ON` / `C1:OUTP OFF` |
| Output load | `C1:OUTP LOAD,HZ` / `C1:OUTP LOAD,50` |
| Output polarity | `C1:OUTP PLRT,NOR` / `INVT` |
| Output snapshot | `C1:OUTP?` → `C1:OUTP OFF,LOAD,HZ,PLRT,NOR` |
| Basic wave (per field) | `C1:BSWV WVTP,SINE`, `C1:BSWV FRQ,1000`, `C1:BSWV AMP,2`, `C1:BSWV OFST,0`, `C1:BSWV PHSE,0`, `C1:BSWV DUTY,50`, `C1:BSWV SYM,50`, `C1:BSWV WIDTH,0.0001` |
| Basic wave snapshot | `C1:BSWV?` → `C1:BSWV WVTP,SINE,FRQ,1000HZ,PERI,0.001S,AMP,2V,AMPVRMS,...` |
| Arbitrary wave | `C1:ARWV INDEX,N` / `C1:ARWV?` → `C1:ARWV INDEX,2,NAME,StairUp` |

Writes carry base-unit NR3 numbers with no unit suffix; readbacks echo units
glued to the number (`1000HZ`, `6.6V`, `-53dBm`) and are parsed with
SI-multiplier awareness. Each BSWV field is written separately (this firmware
wedges on combined BSWV writes followed by a query), and `LOAD` is applied
before amplitude/offset because switching `HZ`↔`50` rescales the displayed
amplitude convention by two.

## Poll cycle (every 2000 ms)

1. Read `Host` and the per-channel settings from ODB.
2. Connect / reconnect if `Host` changed.
3. Per channel, write only changed values in the order
   `OUTP LOAD` → `ARWV INDEX` (if `Wave Type == ARB`) → `BSWV WVTP` →
   `BSWV FRQ/AMP/OFST/PHSE/DUTY/SYM/WIDTH` → `OUTP PLRT` → `OUTP ON|OFF`.
4. Read back `OUTP?` (reconciles `Output` if the front panel changed it),
   `BSWV?` (fills the `Rdbk` keys) and `ARWV?`.
5. Update `Status`, history Variables and the event bank.

## Testing

The transport can be validated read-only against a live instrument by compiling
the real source with a small test `main()` (the frontend itself has no `main()`
— `mfe.o` provides it):

```bash
g++ -O2 -std=c++17 -pthread -I$(MIDASSYS)/include $(MIDASSYS)/mxml $(MIDASSYS)/mjson \
    -D_DEFAULT_SOURCE -D_XOPEN_SOURCE=600 -I. /tmp/sdg2122_selftest.cxx \
    -L$(MIDASSYS)/lib -lmidas $(MIDASSYS)/lib/libmidas.a $(pkg-config --libs libusb-1.0) \
    -lrt -pthread -lm -ldl -rdynamic -o /tmp/sdg2122_selftest
timeout 90 /tmp/sdg2122_selftest /dev/usbtmc0
```

(`/tmp/sdg2122_selftest.cxx` is a local harness; it exercises the response
parser, USB discovery and live queries.)

## Troubleshooting

- **`*IDN?` probe fails / empty reads through `/dev/usbtmcN`** — expected with
  the kernel driver on this firmware; the frontend does not use it. If the
  *frontend's* probe still fails, unplug/replug USB, or run the reference
  CLI's recovery:

  ```bash
  sdg2122 --backend libusb recover
  ```

  (`--backend libusb` is required for product ID `1102`; the CLI's `auto`
  backend only selects libusb for `ee38`.)
- **Only one controller at a time** — stop the frontend before using the
  `sdg2122` CLI or the front panel, and vice versa. The deferred-response
  protocol assumes no other USBTMC session interleaves.
- **Reconnect** is only attempted when `Host` changes (or at startup). If the
  cable is unplugged mid-run, `Status` shows `Not connected` until `Host` is
  edited and the frontend reconnects.
- **To repair a half-built ODB subtree** after a schema change, delete
  `/Equipment/SDG2122X/Settings/Device0` and restart — it is recreated with
  defaults. (User settings are preserved across normal restarts, but deleting
  the subtree resets them.)

## Repository layout

```
sdg2122_fe.cxx          # active frontend (core wave control)
sdg2122_fe_full.cxx     # backup: full version incl. modulation/sweep/burst/
                        #   harmonic/counter and MODE/ROSC globals
Makefile                # g++ build against MIDAS + libusb
CLAUDE.md               # developer notes for the active version
CLAUDE.md.full-backup   # developer notes for the full version
README.md               # this file
```

To restore the full functionality later, merge the parked blocks from
`sdg2122_fe_full.cxx` back into `sdg2122_fe.cxx`, then delete
`/Equipment/SDG2122X` once so the extended ODB schema is rebuilt.

## References

- [sdg2122-cli](https://github.com/LHX369963/sdg2122-cli) — Python CLI that
  documents the SDG2122X command set and firmware quirks; the SCPI catalog and
  parsing rules here follow it.
- SIGLENT *SDG Programming Guide* (`PG02`) for the `OUTP`, `BSWV` and `ARWV`
  command syntax.
