---
name: feetech-servo
description: Manual, one-off operations on a Feetech STS-series serial bus servo (STS3215 / STS3235) through the MicroPython PyBoard — ping/scan for IDs, read position/speed/load/voltage/temperature, command a move, toggle torque, read angle limits, change ID. Use whenever the user wants to poke at a Feetech servo by hand rather than run the robot. Runs everything from the host via `mpremote`; does not modify files already on the board. For Dynamixel AX use ax-servo, for HiWonder use hiwonder-servo.
---

# Feetech STS Servo — Manual Operations via PyBoard

For hands-on work with **one (or a few) Feetech STS3215 / STS3235 servos** on the bench: find its ID, read its state, move it, check temperature and voltage.

**Ground rule: leave the board's filesystem alone.** Do not push, edit, or delete files on the PyBoard for manual servo work. Everything here runs through `mpremote ... exec` (code sent over the wire, nothing stored) or `mpremote ... run <local-file>` (a *host-side* file executed on the board without being written to it).

**Not hardware-verified.** This skill was written from the repo's Feetech code (`feetech/feetech.py`, `numa/servo_group.py`) and its byte-for-byte checks against the protocol manual, with no servo attached. Register addresses and the wire format are cross-checked (see `feetech/feetech.py`); the recipes below have not been run. If something misbehaves, dump packets with `show=Bus.SHOW_PACKETS` before assuming the table is wrong, and tell the user which parts you confirmed on real hardware.

---

## Two things that differ from the other servo skills

### 1. There is no fast-path script

`move_via_pyboard.py` handles `-hw` (HiWonder) and `-ax` (Dynamixel) only. It has no Feetech support, so everything below is composed by hand. (If the user asks to add `-ft`, that is a feature request for the script, not something to work around here.)

### 2. `feetech.py` is not on the board

`deploy_pyboard.py` `SOURCES` does not include `feetech/feetech.py`, so `import feetech` on the board raises `ImportError` (and `numa/servo_group.py` silently falls back to `ft = None`). Do **not** deploy it to work around this. Instead, use the generic Dynamixel `bus.Bus` that is already on the board — Feetech framing is **identical to Dynamixel protocol 1.0**, so `Bus.ping / read / write / sync_write` work unchanged — and use the literal register addresses in the table below.

Check first, in case the board differs:

```bash
mpremote connect /dev/ttyACM0 exec "import os; print(sorted(os.listdir('/flash')))"
```

If `feetech.py` *is* listed, `import feetech as ft` and `ft.Register.*` are usable; otherwise stay with the literals.

---

## Wiring / connection facts

| Thing | Value |
|-------|-------|
| Host serial device | `/dev/ttyACM0` |
| Feetech bus | **UART 2**, 1 Mbaud, half-duplex — the **same wire as the Dynamixel AX servos** (`numa/numa.py:192`, `PROTOCOL_FT: self.axbus`) |
| HiWonder bus (not this) | UART 4, 115200 — see the `hiwonder-servo` skill |

Feetech STS servos are factory-set to 1 Mbaud. If reads time out at 1 Mbaud, the servo's baud register (addr 6) may have been changed — ask the user rather than guessing rates.

**Shared bus with AX servos:** on the robot, Feetech and AX servos need **non-colliding IDs**, and both must agree on the response level (below). A ping to an ID gets answered by whichever family owns it, so a scan alone does not say which family a servo is. To tell them apart, read 5 bytes from address 0: an AX-12 puts its model number (12) in bytes 0-1; an STS puts firmware version in bytes 0-1 and the servo model in bytes 3-4. Treat that as a hint and say so.

**Bench setup:** a fresh STS servo is typically ID 1 from the factory, so **two new servos on one bus collide**. Connect them one at a time when assigning IDs.

Modules live in `/flash`, already on `sys.path`, so a plain `import bus` works. On a board where they are on an SD card instead, prepend `sys.path.insert(0, '/sd')`.

---

## Safety first — check the board before moving anything

```bash
mpremote connect /dev/ttyACM0 exec "import machine, ubinascii; print(ubinascii.hexlify(machine.unique_id()).decode())"
```

| ID | Board |
|----|-------|
| `3700530005504b4d52323420` | **Testbed** — free to experiment |
| `380046001951363039343332` | **Robot** — real hardware |

**On the Robot board, ask the user to confirm before any command that moves a servo or changes stored servo settings.** Read-only commands (`PING`, reads of any register) are always fine.

Also confirm before anything in the "persistent settings" section — ID, angle limits, response level, baud rate live in the servo's EEPROM and survive power cycles. **Never write `128` to `TORQUE_ENABLE`**: per `feetech/feetech.py` that runs the servo's *centering function* (redefines the neutral position), which is not a torque command.

---

## The response-level catch (why writes may "fail")

Feetech's `RESPONSE_STATUS_LEVEL` (addr 8) is the equivalent of the AX `RETURN_LEVEL`, with the same encoding: `0` = never reply, `1` = reply to reads only, `2` = reply to everything.

`numa/init.py:64` sets `RTN_LVL = RETURN_LEVEL_READ_ONLY` (1) on every leg servo, Feetech included, and the value is stored in the servo's EEPROM. So:

- A servo that has **been through robot init** (level 1) does not acknowledge writes. `bus.write()` always waits for a status packet, so it **raises `BusError: Timeout` even though the write succeeded**.
- A **factory-fresh** servo normally acknowledges writes (the factory level is expected to be 2, from general STS behavior rather than anything recorded in this repo), and then `bus.write()` returns normally. Read addr 8 to see which case you are in.

So always wrap writes and verify by reading back. Do not report a write as failed on the strength of the `BusError` alone, and do not "fix" it by changing the response level unless the user asks — on the shared robot bus AX and Feetech must agree, and mismatched acknowledgements collide and make reads time out.

---

## The preamble

```python
from stm_uart_port import UART_Port
from bus import Bus, BusError
import struct, utime

ID = 1
bus = Bus(UART_Port(2, 1000000))

# STS3215/STS3235 control table (feetech/feetech.py Register class)
ID_REG, BAUD, RESP_LEVEL = 5, 6, 8
MIN_LIM, MAX_LIM = 9, 11               # 2 bytes each, EEPROM
MAX_TEMP, MAX_VOLT, MIN_VOLT = 13, 14, 15
TORQUE_EN = 40                         # RAM
GOAL_POS, GOAL_TIME, GOAL_SPEED = 42, 44, 46   # RAM, 2 bytes each, contiguous
LOCK = 55                              # 0 = EEPROM writable, 1 = protected
PRESENT = 56                           # 8-byte block, see Units

def w(dev_id, addr, data):
    try:
        bus.write(dev_id, addr, bytearray(data))
    except BusError:
        pass   # response level 1: writes are not acknowledged

def u16(b):
    return b[0] | (b[1] << 8)          # STS is LOW byte first
```

For raw packet visibility while debugging: `Bus(UART_Port(2, 1000000), show=Bus.SHOW_COMMANDS | Bus.SHOW_PACKETS)`.

---

## Units and gotchas

- **Position**: `0`–`4095` over `360°` — **0.0879° per count, 11.38 counts per degree, centre = `2048`** (180°). Clockwise as the count rises. (`STS_CENTER` in `numa/poses.py`.) Do not use the AX (512, 300°) or HiWonder (750, 240°) figures.
- **Byte order**: two-byte values are **low byte first** on the STS (magnetic-encoder) series — `struct.pack('<H', ...)`. The older SCS potentiometer series is high-byte-first; that is not what this robot uses.
- **Present-state block**: one `bus.read(ID, 56, 8)` returns position(2), speed(2), load(2), voltage(1), temperature(1) in that order.
- **Speed** (`GOAL_SPEED`, present speed): steps per second. The manual's own example commands `1000` (~88°/s). **Set an explicit speed before moving a joint on assembled hardware** — do not rely on the register's power-up value, which is RAM and resets on power cycle. Whether speed `0` means "maximum" on this model is not confirmed in this repo; treat it as possibly maximum.
- **Voltage / load** scales and sign bits are per the model datasheet, not recorded in the repo. Report the raw values, and say the conversion (voltage is normally 0.1 V per count) is from the datasheet convention, not verified here.
- **Temperature**: `°C`, one byte.
- `bus.read(dev_id, addr, num_bytes)` returns a `bytearray`. Same signature as the AX bus, **not** the HiWonder one (which takes a command).
- Goal position outside `MIN_LIM`..`MAX_LIM` is limited by the servo. Read the limits before commanding a move.

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

`PING` is a real instruction, so scanning is reliable with many servos attached. Scanning stops before 254 (broadcast). On the robot's shared bus this lists AX **and** Feetech IDs together — see the wiring section for telling them apart.

### Read a servo's state

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from bus import Bus
bus = Bus(UART_Port(2, 1000000))
ID = 1
u16 = lambda b: b[0] | (b[1] << 8)
lim = bus.read(ID, 9, 4)
print('limits:        {} .. {}'.format(u16(lim[0:2]), u16(lim[2:4])))
print('resp level:    ', bus.read(ID, 8, 1)[0])
print('torque_en:     ', bus.read(ID, 40, 1)[0])
print('goal speed:    ', u16(bus.read(ID, 46, 2)))
p = bus.read(ID, 56, 8)
pos = u16(p[0:2])
print('pos:            {} ({:.1f} deg)'.format(pos, pos * 360.0 / 4096))
print('speed/load raw:', u16(p[2:4]), u16(p[4:6]))
print('voltage raw:   ', p[6], ' temp C:', p[7])
"
```

### Poll position while a move runs

```python
for _ in range(20):
    print(u16(bus.read(ID, PRESENT, 2)))
    utime.sleep_ms(100)
```

---

## Move a servo to a position

Read the limits first (above), confirm the goal is inside them, then set torque and write the whole 6-byte goal block — position, time, speed — in **one** command, exactly as the protocol manual's worked example does:

```bash
mpremote connect /dev/ttyACM0 exec "
from stm_uart_port import UART_Port
from bus import Bus, BusError
import struct, utime
ID, GOAL, SPEED = 1, 2048, 1000
bus = Bus(UART_Port(2, 1000000))
u16 = lambda b: b[0] | (b[1] << 8)
def w(addr, data):
    try: bus.write(ID, addr, bytearray(data))
    except BusError: pass   # response level 1: unacknowledged
print('pos before:', u16(bus.read(ID, 56, 2)))
w(40, [1])                                        # torque on
utime.sleep_ms(25)
w(42, struct.pack('<HHH', GOAL, 0, SPEED))       # goal pos, time 0, speed
utime.sleep_ms(1500)
p = bus.read(ID, 56, 8)
print('pos after: ', u16(p[0:2]), ' temp:', p[7])
"
```

Goal **time `0`** means "use the speed field", as in the manual's example. Pick `SPEED` from the distance: `counts / SPEED` seconds is roughly the sweep time, so a 1000-count (~88°) move at `SPEED = 1000` takes about a second. Size the `sleep_ms` to that plus margin.

**Prefer one command per move.** The servo runs its own trajectory to the goal; looping small position steps makes it accelerate, decelerate and hold at every waypoint, which looks jerky. Step in increments only when a stall partway through would matter (mounted hardware, unknown mechanics), and say so when you do.

A read-back off the goal by more than a few counts means torque is off, the goal was outside the angle limits, or the joint is obstructed.

### Torque on/off

```python
w(ID, TORQUE_EN, [1])   # holding
w(ID, TORQUE_EN, [0])   # limp, back-drivable
```

`TORQUE_ENABLE` is RAM, so no `LOCK` handling is needed and it resets to off at power-up. Unloading is the right move to position the horn by hand, or to stop a servo that is straining.

### Synchronised move across several servos

```python
ids = [1, 2, 3]
goals = [1800, 2048, 2300]
bus.sync_write(ids, GOAL_POS,
               [bytearray(struct.pack('<HHH', g, 0, 1000)) for g in goals])
```

`sync_write` is a broadcast, so it is never acknowledged. One packet carries **one address for every servo in it**, which is why `numa/servo_group.py` sends AX and Feetech as separate batches even on a shared bus — never mix the two families in one `sync_write`.

---

## Persistent settings — confirm with the user first

Everything at addresses below 40 is EEPROM. Writes to it need the **`LOCK` dance**: write `0` to `LOCK` (55), write the value, write `1` back to `LOCK`.

```python
w(ID, LOCK, [0])                                          # unlock
w(ID, MIN_LIM, struct.pack('<HH', min_pos, max_pos))      # angle limits, 9 and 11 are adjacent
w(ID, LOCK, [1])                                          # relock
```

`numa/servo_group.py` (`_ft_set_lock`, `write_angle_limits`, `write_return_level`) does the same bracket.

**Changing the ID** needs one extra care — the servo answers on the new ID immediately, so the relock must be addressed to the **new** ID:

```python
w(ID, LOCK, [0])
w(ID, ID_REG, [new_id])          # 0..253
w(new_id, LOCK, [1])             # NOT the old id
print(bus.read(new_id, ID_REG, 1)[0])   # verify
```

If the robot's shared bus is involved, check the new ID against the AX IDs (`11 21 31 41 12 22 32 42 13 23 33 43`, turret `51 52`) so it does not collide. Broadcasting an ID write (`0xFE`) changes **every** servo on the bus to that ID — the manual's own example does exactly this — so never do it with more than one servo connected.

Do not write `BAUD` (6) without a plan to reach the servo at the new rate afterwards; a servo at an unexpected baud looks dead.

---

## Register reference

From `feetech/feetech.py` `Register` (STS3215 / STS3235; cross-checked against the protocol manual, the STS3215 datasheet and `matthieuvigne/STS_servos`).

| Name | Addr | Bytes | Kind | Notes |
|---|---|---|---|---|
| `SERVO_MAJOR` / `MINOR` | 3 / 4 | 1 / 1 | EEPROM | Model identification |
| `ID` | 5 | 1 | EEPROM | 0–253; change with `LOCK` dance |
| `BAUD_RATE` | 6 | 1 | EEPROM | |
| `RESPONSE_STATUS_LEVEL` | 8 | 1 | EEPROM | 0 never / 1 reads only / 2 all. Robot uses 1 |
| `MIN_ANGLE_LIMIT` | 9 | 2 | EEPROM | |
| `MAX_ANGLE_LIMIT` | 11 | 2 | EEPROM | |
| `MAX_TEMPERATURE` | 13 | 1 | EEPROM | |
| `MAX_VOLTAGE` / `MIN_VOLTAGE` | 14 / 15 | 1 / 1 | EEPROM | |
| `MAX_TORQUE` | 16 | 2 | EEPROM | |
| `OPERATION_MODE` | 33 | 1 | EEPROM | `0` = position servo mode (what legs want) |
| `TORQUE_ENABLE` | 40 | 1 | RAM | 0 off, 1 on, **128 = centering function — do not write** |
| `GOAL_POSITION` | 42 | 2 | RAM | 0–4095, centre 2048 |
| `GOAL_TIME` | 44 | 2 | RAM | 0 = use speed |
| `GOAL_SPEED` | 46 | 2 | RAM | Steps/s |
| `TORQUE_LIMIT` | 48 | 2 | RAM | |
| `LOCK` | 55 | 1 | RAM | 0 = EEPROM writable, 1 = protected |
| `PRESENT_POSITION` | 56 | 2 | RAM, RO | |
| `PRESENT_SPEED` | 58 | 2 | RAM, RO | |
| `PRESENT_LOAD` | 60 | 2 | RAM, RO | |
| `PRESENT_VOLTAGE` | 62 | 1 | RAM, RO | |
| `PRESENT_TEMPERATURE` | 63 | 1 | RAM, RO | °C |
| `MOVING` | 66 | 1 | RAM, RO | |
| `PRESENT_CURRENT` | 69 | 2 | RAM, RO | |

Instruction bytes (Dynamixel-1.0 compatible plus one Feetech addition): `PING` 1, `READ` 2, `WRITE` 3, `REG_WRITE` 4, `ACTION` 5, `RESET` 6, `SYNC_READ` 0x82, `SYNC_WRITE` 0x83.

`SYNC_READ` (0x82) is Feetech-only and is **not implemented by the board's `bus.py`**. If the user wants it, build the packet by hand from `feetech.build_sync_read` and the byte format in `feetech/feetech.py`; individual `bus.read` calls cover manual work fine.

`RESET` (instruction 6) restores the servo's control table to factory values, including its ID and response level. Do not send it without explicit confirmation.

---

## Longer experiments

When a task needs more than a few lines — a sweep, a soak test, a timing benchmark — write the script to the **scratchpad directory on the host** and run it without installing it on the board:

```bash
mpremote connect /dev/ttyACM0 run /path/to/scratchpad/ft_sweep.py
```

`run` streams the file to the board and executes it; nothing is written to the board's filesystem. Drop any `sys.path.insert(0, '/sd')` on a board whose modules are in `/flash`.

To exercise the host-side protocol code with no hardware at all, the packet-level checks run on the PC: `python3 feetech/test_feetech.py` from the repo root.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `BusError: Timeout` on a **write** | Response level 1 (robot init sets it) — expected. Wrap it and read back to confirm |
| `BusError: Timeout` on a **read** | Wrong ID (scan first), servo unpowered, wrong UART (2, not 4), or non-default baud |
| `ImportError: no module named 'feetech'` | Expected — it is not deployed. Use `bus.Bus` and the literal addresses |
| Two new servos both answer ID 1 / garbage scan | Factory-default ID collision. Connect one at a time |
| EEPROM write reads back unchanged | `LOCK` was not cleared first |
| ID change "worked" but relock did nothing | Relock was sent to the old ID; address it to the new one |
| Servo does not move, read-back unchanged | Torque off, goal outside angle limits, or joint obstructed |
| Servo snaps violently | Speed left at its default — set `GOAL_SPEED` before the goal |
| AX servos start timing out after touching a Feetech | Mismatched response levels on the shared bus, or an ID collision |
| `ENODEV` touching `/sd` | No SD card in this board; modules are in `/flash` |
| Connection hangs | Board is running code — the user must Ctrl-C in their own terminal |

## Finishing up

Leave the servo in a known state and report it: final position, torque state, temperature. Unload it (`TORQUE_ENABLE` = 0) unless the user wants it holding, and tell the user which parts of this session were confirmed against a real servo.
