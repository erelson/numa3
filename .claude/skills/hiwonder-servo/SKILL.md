---
name: hiwonder-servo
description: Manual, one-off operations on a HiWonder serial bus servo (LX-16A family) through the MicroPython PyBoard — scan/ping for IDs, read position/temperature/voltage, command a move, stop, load/unload torque, read or change servo ID, offset, and limits. Use whenever the user wants to poke at a HiWonder servo by hand rather than run the robot. Runs everything from the host via `mpremote`; does not modify files already on the board.
---

# HiWonder Servo — Manual Operations via PyBoard

For hands-on, interactive-style work with **one (or a few) HiWonder bus servos** on the bench: find its ID, read its state, nudge it to a position, check how hot it is.

**Ground rule: leave the board's filesystem alone.** Do not push, edit, or delete files on the PyBoard for manual servo work. Everything here runs through `mpremote ... exec` (code sent over the wire, nothing stored) or `mpremote ... run <local-file>` (a *host-side* file executed on the board without being written to it). The modules already on the board (`hiwonder_bus.py`, `hiwonder_packet.py`, `stm_uart_port.py`, `log.py`, `dump_mem.py`) are imported and used as-is.

---

## Fast path: `move_via_pyboard.py`

For the common operations — read, move, release — use the repo's host-side script rather than composing an `mpremote exec` snippet. It is faster to issue and less error-prone:

```bash
./move_via_pyboard.py -hw 42 -r             # read position, limits, load, temp, vin, offset
./move_via_pyboard.py -hw 42 -p 777         # move to an absolute position
./move_via_pyboard.py -hw 42 -a -30         # move to degrees from center (750)
./move_via_pyboard.py -hw 42 -p 777 --rel   # move there, then stop holding
./move_via_pyboard.py -hw 42 --rel          # stop holding (prevents overheating)
```

It already does the things the rest of this skill tells you to do by hand: finds the board and records it in `.pyboard_last_seen.json`, refuses an unknown board, prompts before moving on the Robot board (`-y` skips, needed when there is no tty), converts angles at 4.1667 counts/deg about center 750, warns when the target is outside the servo's `ANGLE_LIMIT`, and issues **one** smooth `MOVE_TIME_WRITE` with travel time scaled to the distance. `-n` resolves everything and stops short of moving; `-r` overrides any `-p`/`-a`/`--release`. Exit codes: 0 fine, 1 servo not found or aborted, 2 board problem.

**Use the recipes below for everything the script does not cover**: scanning the bus, changing servo IDs, angle offsets, writing angle/voltage/temperature limits, `MOVE_STOP`, synchronised multi-servo moves, packet-level debugging, and longer experiments.

---

## Wiring / connection facts

| Thing | Value |
|-------|-------|
| Host serial device | `/dev/ttyACM0` |
| HiWonder bus | **UART 4**, 115200 baud, half-duplex (`UART_Port` sets HDSEL) |
| Dynamixel bus (not this) | UART 2, 1 Mbaud — see the `ax-servo` skill |
| Default servo ID in test code | `1` |

`hiwonder/bench_servo.py` in this repo uses `UART_Port(2, 115200)` — that is an older/testbed wiring. Production (`numa/numa.py:134`) and `hiwonder/temp_rise_test.py` both use UART 4. **Use UART 4 unless the user says otherwise**; if reads time out, ask whether the servo is on UART 2.

### Module import path

`sys.path` on the board already contains `/flash`, and the hiwonder modules live there — so a plain `import hiwonder_bus` works. On a board where the modules are on the SD card instead, prepend `sys.path.insert(0, '/sd')` first. Check with:

```bash
mpremote connect /dev/ttyACM0 exec "import os,sys; print(sys.path); print(sorted(os.listdir('/flash')))"
```

---

## Safety first — check the board before moving anything

Run this before any command that actuates a servo:

```bash
mpremote connect /dev/ttyACM0 exec "import machine, ubinascii; print(ubinascii.hexlify(machine.unique_id()).decode())"
```

| ID | Board |
|----|-------|
| `3700530005504b4d52323420` | **Testbed** — free to experiment |
| `380046001951363039343332` | **Robot** — real hardware |

**On the Robot board, ask the user to confirm before any command that moves a servo or changes stored servo settings.** Read-only commands (`POS_READ`, `TEMP_READ`, `VIN_READ`, `ID_READ`, `*_READ`) are always fine.

Also confirm with the user before any command in the "persistent settings" section below — `ID_WRITE`, `ANGLE_OFFSET_WRITE`, `ANGLE_LIMIT_WRITE`, `VIN_LIMIT_WRITE`, `TEMP_MAX_LIMIT_WRITE` write to the servo's own flash and survive power cycles. Getting an ID wrong on a 14-servo robot means hunting it down by hand.

---

## The preamble

Every snippet below assumes this setup block. Keep `HW_ID` at the top so it is easy to change.

```python
from stm_uart_port import UART_Port
from hiwonder_bus import Bus, BusError
import hiwonder_packet as pkt

HW_ID = 1
bus = Bus(UART_Port(4, 115200))
```

To see the raw bytes on the wire while debugging, construct with
`Bus(UART_Port(4, 115200), show=Bus.SHOW_COMMANDS | Bus.SHOW_PACKETS)`.

---

## Units

- **Position**: the servos on this robot are **HiWonder HX-35HM**: `0`–`1500` counts over ~`360°`, so `0.24°` per count (`4.167` counts per degree) and **centre = `750`**. See `hx35hmpos()` in `numa/poses.py:251` and `HX_DEG_PER_COUNT` in `optimize_gait.py:47`.
  **Do not assume the LX-16A `0`–`1000` / `240°` spec.** Degrees-per-count is identical between the two (0.24°), so relative offsets look right either way — but the centre and the endpoints are not, and using 500 as "centre" puts the joint 60° out. If you are unsure which servo is on the bus, read `ANGLE_LIMIT_READ`: an untouched HX-35HM reports `0 .. 1500`.
- `POS_READ` returns a **signed** 16-bit value and can read slightly outside the nominal range.
- **Move time**: milliseconds, `0`–`30000`. `0` means "as fast as possible".
- **Temperature**: `°C`, one unsigned byte.
- **Voltage**: millivolts, unsigned 16-bit.
- All multi-byte parameters are **little-endian**.

---

## Read operations

### Scan the bus for servo IDs

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from hiwonder_bus import Bus
bus = Bus(UART_Port(4, 115200))
found = []
bus.scan(0, 32, dev_found=lambda b, i: found.append(i))
print('found:', found)
"
```

`ping()`/`scan()` use `ID_READ`, because HiWonder has no PING command. Caveat: a servo answers `ID_READ` even when addressed by the broadcast id (`0xFE` = 254), so **with more than one servo on the bus a scan can produce collisions and garbage**. For a single-servo bench setup it is reliable. Scanning stops before 254.

### Read position, temperature, voltage

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from hiwonder_bus import Bus
import hiwonder_packet as pkt
import struct
HW_ID = 1
bus = Bus(UART_Port(4, 115200))
pos  = struct.unpack('<h', bus.read(HW_ID, pkt.Command.POS_READ))[0]
temp = bus.read(HW_ID, pkt.Command.TEMP_READ)[0]
vin  = struct.unpack('<H', bus.read(HW_ID, pkt.Command.VIN_READ))[0]
print('pos {} ({:.1f} deg)  temp {}C  vin {} mV'.format(pos, pos * 0.24, temp, vin))
"
```

`bus.read(id, cmd)` returns the parameter bytes as a `bytearray`. Unpack per the table below.

### Poll position while a move runs

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from hiwonder_bus import Bus
import hiwonder_packet as pkt
import struct, utime
HW_ID = 1
bus = Bus(UART_Port(4, 115200))
for _ in range(20):
    print(struct.unpack('<h', bus.read(HW_ID, pkt.Command.POS_READ))[0])
    utime.sleep_ms(100)
"
```

---

## Move operations

Writes are **not acknowledged** by HiWonder servos — `bus.write()` sends and returns immediately, and always reports `ErrorCode.NONE`. To confirm a move actually happened, read the position back.

### Move to a position over a given time

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from hiwonder_bus import Bus
import hiwonder_packet as pkt
import struct, utime
HW_ID, POS, MS = 1, 500, 800
bus = Bus(UART_Port(4, 115200))
bus.write(HW_ID, pkt.Command.MOVE_TIME_WRITE, bytearray(struct.pack('<HH', POS, MS)))
utime.sleep_ms(MS + 100)
print('now at', struct.unpack('<h', bus.read(HW_ID, pkt.Command.POS_READ))[0])
"
```

`bytearray(struct.pack('<HH', pos, time_ms))` is the 4-byte payload: `pos_lo, pos_hi, time_lo, time_hi`.

### Prefer ONE command — the servo interpolates for you

`MOVE_TIME_WRITE` hands the servo a destination *and* a duration, and the servo generates its own smooth trajectory over that duration. **One command for the whole move is both the smoothest and the simplest option — make it the default.**

**Do not loop small `MOVE_TIME_WRITE` steps toward a target unless you need per-step verification.** Stepping produces visibly jerky motion: the servo accelerates, decelerates, stops and *holds* at every waypoint, then sits idle while you read the position back and evaluate the next step. At a 400 ms step plus a ~100 ms read-back that is roughly two stutters per second, and the operator will notice and ask about it.

Pick `MS` from the distance rather than reusing a fixed number:

| Want roughly | Use |
|---|---|
| 60 °/s | `MS = counts * 4` |
| 48 °/s | `MS = counts * 5` |
| 30 °/s | `MS = counts * 8` |

(counts x 0.24 = degrees; `MS` is capped at 30000.) A 395-count move (~95 deg) at `MS = 2000` runs ~47 deg/s and completes in one smooth sweep.

Stepping is justified only when a stall or obstruction partway through would matter — mounted hardware with unknown mechanics, or a joint you have not driven through that range before — because it lets you abort after one small increment instead of the full travel. When you do step, **say so and say why**, since the operator is paying for it in smoothness. Prefer the largest step that still gives useful early warning (50 counts / ~12 deg was workable) and drop back to single commands as soon as the range is known good.

A read-back that lands 1-2 counts off the commanded position is this servo's normal deadband, not accumulated stepping error; the miss direction is not consistent between moves.

### Stop an in-progress move

```python
bus.fill_and_write_packet(HW_ID, pkt.Command.MOVE_STOP)
```

`MOVE_STOP` takes no parameters, so call `fill_and_write_packet` directly (`bus.write` expects a data argument).

### Synchronised move across several servos

Prime each servo with `MOVE_TIME_WAIT_WRITE` (same 4-byte payload as `MOVE_TIME_WRITE`), then fire them together:

```python
for sid, pos in ((1, 300), (2, 700)):
    bus.write(sid, pkt.Command.MOVE_TIME_WAIT_WRITE, bytearray(struct.pack('<HH', pos, 500)))
bus.action()   # broadcasts MOVE_START
```

### Torque on/off (load/unload)

```python
bus.write(HW_ID, pkt.Command.LOAD_OR_UNLOAD_WRITE, bytearray([0]))  # 0 = unload (limp, back-drivable)
bus.write(HW_ID, pkt.Command.LOAD_OR_UNLOAD_WRITE, bytearray([1]))  # 1 = load (holding)
```

Unloading is the right move when the user wants to position the horn by hand, or to stop a servo that is straining. It is also the safe thing to leave a bench servo in when finished.

---

## Persistent settings — confirm with the user first

These change state stored in the servo itself.

```python
# Change servo ID (1 byte, 0-253). The servo answers on the new ID immediately.
bus.write(HW_ID, pkt.Command.ID_WRITE, bytearray([new_id]))

# Trim the zero point: adjust live (-125..125, ~ -30..30 deg), then save it.
bus.write(HW_ID, pkt.Command.ANGLE_OFFSET_ADJUST, bytearray([offset & 0xff]))  # signed byte
bus.fill_and_write_packet(HW_ID, pkt.Command.ANGLE_OFFSET_WRITE)               # persists it

# Travel limits (each 0-1500 on HX-35HM, min < max)
bus.write(HW_ID, pkt.Command.ANGLE_LIMIT_WRITE, bytearray(struct.pack('<HH', min_pos, max_pos)))

# Input voltage limits, millivolts
bus.write(HW_ID, pkt.Command.VIN_LIMIT_WRITE, bytearray(struct.pack('<HH', min_mv, max_mv)))

# Max temperature before the servo unloads itself (50-100 C)
bus.write(HW_ID, pkt.Command.TEMP_MAX_LIMIT_WRITE, bytearray([deg_c]))
```

`ANGLE_OFFSET_ADJUST` alone is volatile — it is lost on power-down unless followed by `ANGLE_OFFSET_WRITE`. That makes "adjust, check by eye, then save" a safe workflow: only the final `ANGLE_OFFSET_WRITE` is irreversible.

Note `hiwonder/hiwonder.py` in this repo is a small, partly-stale constants file (it mixes in Dynamixel register numbers). Prefer `hiwonder_packet.Command` as the source of truth for command numbers.

---

## Command reference

Commands come from `hiwonder_packet.Command`. "Params" is the payload passed to `bus.write` / returned by `bus.read`.

| Command | # | Params | Notes |
|---|---|---|---|
| `MOVE_TIME_WRITE` | 1 | `<HH` pos, ms | Move now |
| `MOVE_TIME_READ` | 2 | → `<HH` pos, ms | Last commanded move |
| `MOVE_TIME_WAIT_WRITE` | 7 | `<HH` pos, ms | Primes; fires on `MOVE_START` |
| `MOVE_TIME_WAIT_READ` | 8 | → `<HH` | Primed move |
| `MOVE_START` | 11 | none | `bus.action()` broadcasts this |
| `MOVE_STOP` | 12 | none | Halts current move |
| `ID_WRITE` | 13 | `B` id | Persistent |
| `ID_READ` | 14 | → `B` id | Answers to broadcast too; used for ping |
| `ANGLE_OFFSET_ADJUST` | 17 | `b` offset | Volatile until saved |
| `ANGLE_OFFSET_WRITE` | 18 | none | Persists the offset |
| `ANGLE_OFFSET_READ` | 19 | → `b` offset | |
| `ANGLE_LIMIT_WRITE` | 20 | `<HH` min, max | Persistent |
| `ANGLE_LIMIT_READ` | 21 | → `<HH` min, max | |
| `VIN_LIMIT_WRITE` | 22 | `<HH` min, max mV | Persistent |
| `VIN_LIMIT_READ` | 23 | → `<HH` min, max mV | |
| `TEMP_MAX_LIMIT_WRITE` | 24 | `B` °C | Persistent |
| `TEMP_MAX_LIMIT_READ` | 25 | → `B` °C | |
| `TEMP_READ` | 26 | → `B` °C | Current temperature |
| `VIN_READ` | 27 | → `<H` mV | Current bus voltage |
| `POS_READ` | 28 | → `<h` pos | **Signed**; may exceed the nominal 0–1500 |
| `OR_MOTOR_MODE_WRITE` | 29 | `B` mode, `B` 0, `<h` speed | mode 0 = servo, 1 = continuous motor |
| `OR_MOTOR_MODE_READ` | 30 | → mode, 0, `<h` speed | |
| `LOAD_OR_UNLOAD_WRITE` | 31 | `B` 0/1 | 0 = unload, 1 = load |
| `LOAD_OR_UNLOAD_READ` | 32 | → `B` 0/1 | |
| `LED_CTRL_WRITE` | 33 | `B` 0/1 | Verify polarity on the bench before relying on it |
| `LED_CTRL_READ` | 34 | → `B` | |
| `LED_ERROR_WRITE` | 35 | `B` mask | Which faults light the LED |
| `LED_ERROR_READ` | 36 | → `B` mask | |

Verified against hardware in this repo: `MOVE_TIME_WRITE`, `MOVE_STOP`, `TEMP_READ`, `POS_READ`, `ID_READ` (see `hiwonder/temp_rise_test.py`, `hiwonder/bench_servo.py`). The rest come from the protocol tables in `hiwonder_packet.py` — if one misbehaves, dump packets with `show=Bus.SHOW_PACKETS` rather than assuming the table is right.

---

## Longer experiments

When a task needs more than a few lines — a sweep, a soak test, a timing benchmark — write the script to the **scratchpad directory on the host** and run it without installing it on the board:

```bash
mpremote connect /dev/ttyACM0 run /path/to/scratchpad/servo_sweep.py
```

`run` streams the file to the board and executes it; nothing is written to the board's filesystem. Existing examples of this shape: `hiwonder/bench_servo.py` (throughput benchmark) and `hiwonder/temp_rise_test.py` (thermal soak). Note both of those start with `sys.path.insert(0, '/sd')` for a testbed board with an SD card — drop that line on a board whose modules are in `/flash`.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `BusError: Rcvd Status: Timeout` on every read | Wrong UART (try 2 instead of 4), servo unpowered, or wrong ID — scan first |
| `ENODEV` touching `/sd` | No SD card in this board; the modules are in `/flash` |
| Connection hangs | Board is running code — the user must Ctrl-C in their own terminal |
| Writes appear to do nothing | Expected: writes are unacknowledged. Read position back to confirm |
| Servo moves then goes limp | Hit its temperature or voltage limit — read `TEMP_READ` / `VIN_READ` |
| Garbage from `scan()` | More than one servo answering the broadcast `ID_READ` |

## Finishing up

After bench work that loaded the servo, leave it in a known state — unload it (`LOAD_OR_UNLOAD_WRITE` with `0`) unless the user wants it holding position, and report the final position and temperature.
