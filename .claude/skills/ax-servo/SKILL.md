---
name: ax-servo
description: Manual, one-off operations on a Dynamixel AX-series servo through the MicroPython PyBoard — ping/scan for IDs, read position/temperature/load, command a move, toggle torque and LED, read angle limits. Use whenever the user wants to poke at an AX/Dynamixel servo by hand rather than run the robot. Runs everything from the host via `mpremote`; does not modify files already on the board. For HiWonder bus servos use the hiwonder-servo skill instead.
---

# Dynamixel AX Servo — Manual Operations via PyBoard

For hands-on, interactive-style work with the **Dynamixel AX servos** on this robot: find which IDs are alive, read position/temperature, nudge a joint, flash an LED to identify a servo.

**Ground rule: leave the board's filesystem alone.** Do not push, edit, or delete files on the PyBoard for manual servo work. Everything here runs through `mpremote ... exec` (code sent over the wire, nothing stored) or `mpremote ... run <local-file>` (a *host-side* file executed on the board without being written to it). The modules already on the board (`bus.py`, `packet.py`, `ax.py`, `stm_uart_port.py`, `log.py`, `dump_mem.py`) are imported and used as-is.

---

## The one thing that will confuse you first

**Every write to these servos raises `BusError: Rcvd Status: Timeout`, even when the write succeeds.**

`numa/init.py:59` sets `RTN_LVL = 1` and applies it to all leg servos (via `servo_group.write_return_level`) and turret servos at init. This is a **deliberate project convention** — return level 1 means "reply to READ_DATA only, not to writes", which keeps the bus quiet during normal high-rate command operation.

The consequence for manual work: `bioloid3/bus.py:229` `write()` unconditionally calls `read_status_packet()` and raises when no status comes back. Reads are unaffected.

So **always wrap writes and verify by reading back**:

```python
def w(dev_id, offset, data):
    try:
        bus.write(dev_id, offset, data)
    except BusError:
        pass   # RETURN_LEVEL=1: writes are not acknowledged by design
```

Do not "fix" this by raising the return level — it would defeat the convention. Do not report a write as failed on the strength of the `BusError` alone; read the register back and say what it actually holds.

---

## Fast path: `move_via_pyboard.py`

For the common operations — read, move, release — use the repo's host-side script rather than composing an `mpremote exec` snippet. It is faster to issue and less error-prone:

```bash
./move_via_pyboard.py -ax 22 -r             # position, limits, torque, speed, temp, return level
./move_via_pyboard.py -ax 22 -p 619         # move to an absolute position
./move_via_pyboard.py -ax 22 -a -30         # move to degrees from center (512)
./move_via_pyboard.py -ax 22 -p 619 --rel   # move there, then torque off
./move_via_pyboard.py -ax 22 --rel          # torque off (prevents overheating)
```

It already handles the traps this skill warns about: writes are wrapped for `RETURN_LEVEL = 1` and verified by read-back, `MOVING_SPEED` is set to a sane default (200, ~133 deg/s) instead of leaving it at 0 = MAX, the target is checked against `CW`/`CCW_ANGLE_LIMIT` with a warning that holding against a clamp heats the servo, and the Robot board prompts before moving (`-y` skips). Angles convert at 3.4133 counts/deg about center 512, matching `ax12pos()` in `numa/poses.py`. `-n` is a dry run; `-r` overrides any `-p`/`-a`/`--release`. Exit codes: 0 fine, 1 servo not found or aborted, 2 board problem.

**Use the recipes below for everything the script does not cover**: scanning the bus, the LED, changing IDs, writing angle limits or return level, `sync_write` across several servos, packet-level debugging, and longer experiments.

---

## Wiring / connection facts

| Thing | Value |
|-------|-------|
| Host serial device | `/dev/ttyACM0` |
| Dynamixel bus | **UART 2**, 1 Mbaud, half-duplex (`UART_Port` sets HDSEL) |
| HiWonder bus (not this) | UART 4, 115200 baud |

Modules live in `/flash`, which is already on `sys.path`, so a plain `import bus` works. On a board where they are on an SD card instead, prepend `sys.path.insert(0, '/sd')`.

---

## Safety first — check the board before moving anything

```bash
mpremote connect /dev/ttyACM0 exec "import machine, ubinascii; print(ubinascii.hexlify(machine.unique_id()).decode())"
```

| ID | Board |
|----|-------|
| `3700530005504b4d52323420` | **Testbed** — free to experiment |
| `380046001951363039343332` | **Robot** — real hardware, assembled legs |

**On the Robot board, ask the user to confirm before any command that moves a servo.** These are leg and turret joints on an assembled machine: a single joint moving can shift the robot's weight or make it drop. Read-only commands and the LED are always fine.

Before commanding a move, always read **`CW_ANGLE_LIMIT_L` / `CCW_ANGLE_LIMIT_L`** — these servos have per-joint travel limits well inside the full range (servo 22 measured 279–852; others are tighter). A goal outside the limits is refused or clamped, and the value that looks like "center" may not be reachable.

---

## The preamble

```python
from stm_uart_port import UART_Port
from bus import Bus, BusError
import ax, struct, utime

bus = Bus(UART_Port(2, 1000000))

def w(dev_id, offset, data):
    try:
        bus.write(dev_id, offset, data)
    except BusError:
        pass   # RETURN_LEVEL=1
```

For raw packet visibility while debugging: `Bus(UART_Port(2, 1000000), show=Bus.SHOW_COMMANDS | Bus.SHOW_PACKETS)`.

---

## Units and gotchas

- **Position**: `0`–`1023` maps to `0`–`300°` (~`3.41` units per degree). **Center = `512`** (150°).
- **`MOVING_SPEED = 0` means *maximum speed*, not stopped.** It is the power-on default. Set a moderate value (e.g. `300`) before moving a joint by hand on assembled hardware, or it will snap. It is a RAM register, so it reverts to `0` on power cycle; `numa/init.py` sets its own speeds at startup.
- **Temperature**: `°C`, one unsigned byte at `PRESENT_TEMP` (43).
- All multi-byte values are **little-endian** (`struct.pack('<H', ...)`).
- `bus.read(dev_id, offset, num_bytes)` returns a `bytearray`. Note the signature differs from the HiWonder bus, which takes a command rather than an offset and length.

---

## Read operations

### Scan the bus

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from bus import Bus
bus = Bus(UART_Port(2, 1000000))
found = []
bus.scan(0, 64, dev_found=lambda b, i: found.append(i))
print('ids on bus:', found)
"
```

Unlike the HiWonder bus, Dynamixel has a real `PING` command, so scanning is reliable with many servos attached. `CLAUDE.md` describes 14 servos; scans during bench work have returned fewer, and **the roster has changed between consecutive scans** — if a servo is missing, suspect power or a bus connection before concluding it is dead, and say so rather than silently working with a short list.

### Read a servo's state

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from bus import Bus
import ax, struct
ID = 22
bus = Bus(UART_Port(2, 1000000))
cw  = struct.unpack('<H', bus.read(ID, ax.CW_ANGLE_LIMIT_L, 2))[0]
ccw = struct.unpack('<H', bus.read(ID, ax.CCW_ANGLE_LIMIT_L, 2))[0]
pos = struct.unpack('<H', bus.read(ID, ax.PRESENT_POSITION, 2))[0]
print('limits: {} .. {}'.format(cw, ccw))
print('pos: {} ({:.1f} deg)'.format(pos, pos * 300.0 / 1023))
print('torque_en:   ', bus.read(ID, ax.TORQUE_ENABLE, 1)[0])
print('moving_speed:', struct.unpack('<H', bus.read(ID, ax.MOVING_SPEED, 2))[0])
print('temp:        ', bus.read(ID, ax.PRESENT_TEMP, 1)[0])
"
```

---

## Move a servo to a position

Read the limits first (above), confirm the goal is inside them, then:

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from bus import Bus, BusError
import ax, struct, utime
ID, GOAL, SPEED = 22, 512, 300
bus = Bus(UART_Port(2, 1000000))
def w(off, data):
    try: bus.write(ID, off, data)
    except BusError: pass
print('pos before:', struct.unpack('<H', bus.read(ID, ax.PRESENT_POSITION, 2))[0])
w(ax.MOVING_SPEED, struct.pack('<H', SPEED))
utime.sleep_ms(25)
w(ax.TORQUE_ENABLE, bytearray([1]))
utime.sleep_ms(25)
w(ax.GOAL_POSITION, struct.pack('<H', GOAL))
utime.sleep_ms(1200)
print('pos after: ', struct.unpack('<H', bus.read(ID, ax.PRESENT_POSITION, 2))[0])
print('temp:      ', bus.read(ID, ax.PRESENT_TEMP, 1)[0])
"
```

AX servos close the loop precisely — expect the read-back to land exactly on the goal, unlike the HiWonder servos' one-count deadband. A read-back that is off by more than a count or two means the joint is obstructed, outside its limits, or torque is off.

### Torque on/off

```python
w(ID, ax.TORQUE_ENABLE, bytearray([1]))   # holding
w(ID, ax.TORQUE_ENABLE, bytearray([0]))   # limp, back-drivable
```

### LED — useful for identifying which physical servo an ID is

```python
LED = 25   # see note below before using ax.LED
w(ID, LED, bytearray([1]))   # on
w(ID, LED, bytearray([0]))   # off
print(bus.read(ID, LED, 1)[0])   # verify
```

Harmless on the Robot board — it actuates nothing. Remember to turn it back off.

**`ax.LED` may not exist on the board yet.** `LED = 25` was added to `numa/ax.py` in the repo, but the board carries its own copy in `/flash` — until someone redeploys, `ax.LED` raises `AttributeError` there. Use the literal `25`, or check first:

```bash
mpremote connect /dev/ttyACM0 exec "import ax; print(hasattr(ax, 'LED'))"
```

---

## Control table reference

Constants come from `numa/ax.py` (a partial table — only what the project uses). Addresses are protocol 1.0.

| Name | Addr | Bytes | Notes |
|---|---|---|---|
| `CW_ANGLE_LIMIT_L` | 6 | 2 | EEPROM, persistent |
| `CCW_ANGLE_LIMIT_L` | 8 | 2 | EEPROM, persistent |
| `RETURN_LEVEL` | 16 | 1 | **Project sets this to 1** — see top of this skill |
| `TORQUE_ENABLE` | 24 | 1 | 0 = limp, 1 = holding |
| `LED` | 25 | 1 | 0 = off, 1 = on |
| `GOAL_POSITION` | 30 | 2 | 0–1023, center 512 |
| `MOVING_SPEED` | 32 | 2 | **0 = maximum**, 1–1023 proportional |
| `BIOLOID_FRAME_LENGTH` | 33 | 1 | |
| `PRESENT_POSITION` | 36 | 2 | Read-only |
| `PRESENT_LOAD_L` | 40 | 2 | Read-only |
| `PRESENT_TEMP` | 43 | 1 | Read-only, °C |
| `READ_DATA` / `WRITE_DATA` | 2 / 3 | | Instruction bytes, not registers |
| `SYNC_WRITE` | 131 | | Instruction byte; see `bus.sync_write()` |

EEPROM registers (angle limits, return level, ID) survive power cycles — **confirm with the user before writing any of them.** RAM registers (torque, LED, goal, speed) reset at power-up.

`numa/ax.py` uses CRLF line endings; preserve them when editing it. Note the deploy staging copy at `micropy-to-upload/ax.py` is **generated** by `deploy_pyboard.py` — edit `numa/ax.py` and redeploy, never the staged copy.

### Writing to several servos at once

`bus.sync_write(dev_ids, offset, values)` broadcasts one packet to many servos — this is what `numa/servo_group.py` and `numa/init.py` use. It is a broadcast, so it is never acknowledged regardless of return level. Prefer it over a loop of individual writes when setting the same register across a group.

---

## Longer experiments

For sweeps, soak tests, or benchmarks, write the script to the **scratchpad directory on the host** and run it without installing it on the board:

```bash
mpremote connect /dev/ttyACM0 run /path/to/scratchpad/ax_sweep.py
```

`hiwonder/temp_rise_test.py` is an existing example driving both buses at once (AX on UART 2, HiWonder on UART 4). Note it starts with `sys.path.insert(0, '/sd')` for a testbed board with an SD card — drop that on a board whose modules are in `/flash`.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `BusError: Timeout` on a **write** | Expected — `RETURN_LEVEL = 1`. Wrap it and read back to confirm |
| `BusError: Timeout` on a **read** | Wrong ID (scan first), servo unpowered, or wrong UART (2, not 4) |
| Servo does not move, read-back unchanged | Torque disabled, goal outside angle limits, or joint obstructed |
| Servo snaps violently | `MOVING_SPEED` is 0 = maximum; set it before commanding a goal |
| IDs missing between scans | Suspect intermittent power or bus wiring — report it, do not assume the servo is gone |
| `ENODEV` touching `/sd` | No SD card in this board; modules are in `/flash` |
| Connection hangs | Board is running code — the user must Ctrl-C in their own terminal |

## Finishing up

Leave the servo in a known state and report it: final position, torque state, temperature. Turn off any LED you switched on for identification.
