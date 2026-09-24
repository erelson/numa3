---
name: pyboard
description: Interact with the MicroPython PyBoard over USB — push/pull files with rshell, run code and capture output with mpremote, identify which board (Testbed vs Robot) is attached. Use for any file transfer, code execution, or verification on the board. For hands-on commanding of a single HiWonder bus servo, use the hiwonder-servo skill instead.
---

# PyBoard Skill

Use this skill whenever the user wants to interact with the MicroPython PyBoard connected via USB.

For manual, one-off operations on a **HiWonder bus servo** (move it, read its position/temp, scan for IDs, change its ID), use the `hiwonder-servo` skill instead — it builds on this one.

## Setup

- Device: `/dev/ttyACM0`
- Tool: `rshell -p /dev/ttyACM0`
- Board filesystems: `/pyboard/flash/` (main) and `/pyboard/sd/` (SD card)
- Project source dirs: `numa/` (main robot code) and `hiwonder/` (HiWonder servo code)

## What this skill does

When invoked, read the user's intent and perform the most appropriate action(s) below. Chain multiple actions when it makes sense (e.g. push then verify).

---

## Tools

- **rshell**: file management (push/pull/list). Each invocation reconnects — normal.
- **mpremote**: execute code and capture stdout. Use this for testing and verification.

Both use `/dev/ttyACM0`. SD card is `/sd/` on-board, `/pyboard/sd/` via rshell, `:` prefix via mpremote.

---

## Actions

### List files on board
```bash
rshell -p /dev/ttyACM0 ls /pyboard/sd/
rshell -p /dev/ttyACM0 ls /pyboard/flash/
```

### Push a file to the board
Push to SD (preferred for working files):
```bash
rshell -p /dev/ttyACM0 cp numa/numa.py /pyboard/sd/
```
Push to flash (for boot/system files):
```bash
rshell -p /dev/ttyACM0 cp numa/boot.py /pyboard/flash/
```

### Push multiple files to SD
```bash
rshell -p /dev/ttyACM0 cp numa/numa.py numa/IK.py numa/poses.py numa/commander.py /pyboard/sd/
```

### Push all hiwonder files to SD
```bash
rshell -p /dev/ttyACM0 cp hiwonder/hiwonder.py hiwonder/hiwonder_bus.py hiwonder/hiwonder_packet.py /pyboard/sd/
```

### Read a file from the board
```bash
mpremote connect /dev/ttyACM0 cat :main.py
```

### Run a snippet and capture output (for testing/verification)
```bash
mpremote connect /dev/ttyACM0 exec "print('hello')"
```
Multi-line:
```bash
mpremote connect /dev/ttyACM0 exec "
import os
print(os.listdir('/sd'))
"
```

### Run a local script on the board and capture output
```bash
mpremote connect /dev/ttyACM0 run test_script.py
```

### Import and test a module already on the board
```bash
mpremote connect /dev/ttyACM0 exec "
import hiwonder
# test code here
"
```

### Soft-reset the board
```bash
mpremote connect /dev/ttyACM0 reset
```

### Open interactive REPL (tell user to run this themselves)
Claude cannot hold an interactive session — tell the user to run in their own terminal:
```
mpremote connect /dev/ttyACM0
```
or
```
rshell -p /dev/ttyACM0 repl
```

---

## Iterative test workflow

1. Edit local file
2. Push with rshell: `rshell -p /dev/ttyACM0 cp <file> /pyboard/sd/`
3. Run test with mpremote and read captured stdout
4. Repeat

---

## Decision guide

- **Pushing numa.py / IK.py / poses.py / commander.py** → push to `/pyboard/sd/`
- **Pushing boot.py** → push to `/pyboard/flash/`
- **Hiwonder files** → push to `/pyboard/sd/`
- **New experimental scripts** → push to `/pyboard/sd/` unless user specifies otherwise
- **User wants to interactively debug** → tell them to open a terminal and run `mpremote connect /dev/ttyACM0`
- **Testing/verifying a change** → push file, then use `mpremote exec` or `mpremote run` to capture output

## Board identities

Read the attached board's ID before running any test or experimental code:
```bash
mpremote connect /dev/ttyACM0 exec "import machine, ubinascii; print(ubinascii.hexlify(machine.unique_id()).decode())"
```

| ID | Board |
|----|-------|
| `3700530005504b4d52323420` | **Testbed** (safe for experiments) |
| `380046001951363039343332` | **Robot** (actual robot hardware) |

**If the attached board is the Robot board, do NOT run test/experimental scripts on it.** Only push production code (numa.py, IK.py, etc.) or run safe read-only checks. Ask the user to confirm before doing anything that moves servos or actuates hardware on the Robot board.

---

## Notes

- Flash (`/flash/` on board) has `boot.py` and `main.py` which run at startup.
- If the board is busy (running code), connection may hang — user may need to Ctrl-C first in their terminal.
- mpremote uses `:` prefix for board paths (e.g. `:main.py` = `/main.py` on board's default fs).
