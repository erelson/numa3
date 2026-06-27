Overview
--------
(circa April 2024)

This folder contains pyboard scripts for a test setup where a pyboard drives a two-wheel base.

This was used for testing ROS/lidar/mapping/navigation.

Files:
- `MotorDriver.py` - PWM + direction motor driver class (per VNH5019 channel)
- `two_wheel.py` - `Hardware` class mapping cmd_vel to left/right wheel speeds
- `main.py` - pyboard entry point; reads cmd_vel packets over UART from the Pi
- `build_map.launch` - ROS launch for slam_karto mapping (remaps `scan` -> `base_scan`)


Raspberry Pi software (ROS side)
--------------------------------
(via claude)
The Pi runs ROS and is the only ROS machine in this setup (the PyBoard speaks the raw
serial packet protocol below, not ROS). Based on `build_map.launch` here and the
installed-package list in `../numapi/deblist.txt`, the Pi-side stack was roughly:

- **ROS distro**: ROS1, tested with **Kinetic on Ubuntu 16.04** (per the top-level
  `CLAUDE.md`); `deblist.txt` shows `ros-kinetic-slam-karto`, with `open-karto` /
  `gmapping` also installed under Melodic/Noetic from later experiments.
- **`roscore`** plus a **2D lidar driver** publishing `LaserScan` on `base_scan`
  (the project's scanner is an LDLidar 2D unit).
- **Mapping**: `slam_karto` via `build_map.launch` — it subscribes to `scan`, remapped
  to `base_scan`, and builds the occupancy map. `deblist.txt` indicates gmapping and
  `laser-scan-matcher` were also available/tried as alternatives.
- **`cmd_vel_relay`** (from `../numapi/numapi_to_pyboard`) — subscribes to `/cmd_vel`
  and forwards `linear.x` / `angular.z` to the PyBoard over serial using the packet
  format below. The `/cmd_vel` source during testing was typically
  `teleop_twist_keyboard` (also in `deblist.txt`).
- Map saving via `map_saver` (`ros-noetic-map-saver` in `deblist.txt`).

Note: only `build_map.launch` and the relay node are certain from the files in this
repo; the lidar driver and teleop source are inferred from the package list and the
project's stated hardware, so exact node/driver names may differ from what was run.


Wiring / Connections
--------------------
(via claude; I haven't diagrammed this test bot in my usual SVG)
Three components: **Raspberry Pi** (runs ROS) -> **PyBoard** (MicroPython) -> **motor
driver(s)** (VNH5019-style, with direction A/B, PWM, and current-sense pins) -> two DC
gearmotors. Pin names below are PyBoard board labels (e.g. `X5`, `Y9`).

### Raspberry Pi <-> PyBoard
Serial link on the pyboard's `UART(4)` at 115200 baud (`main.py`).

| Signal | PyBoard pin (UART4) | Pi side |
|--------|---------------------|---------|
| PyBoard TX -> Pi RX | `X1` (TX) | Pi UART RX |
| Pi TX -> PyBoard RX | `X2` (RX) | Pi UART TX |
| Ground               | `GND`     | Pi GND  |

Packet format (`main.py` / matches `cmd_vel_relay.py`): 2-byte header `\xAB\xCD`,
two little-endian float32 (`linear_x`, `angular_z`), 1-byte checksum (`sum(bytes) % 256`).

### PyBoard <-> Motor drivers
From `two_wheel.py` `Hardware.__init__`. Each motor uses two direction pins, one PWM
pin, and a current-sense (CS) pin. The left motor is run reversed in software
(`leftMotor.run_reversed()`).

| Motor | Dir A (INA) | Dir B (INB) | PWM | Current sense (CS) |
|-------|-------------|-------------|-----|--------------------|
| Right | `X5`        | `X8`        | `X6` | `X7`              |
| Left  | `Y9`        | `Y12`       | `Y10`| `Y11`             |

PWM timer/channel mapping (`MotorDriver.py` `PWM_PINS`):

| PWM pin | Timer | Channel |
|---------|-------|---------|
| `X6`    | 2     | 1       |
| `Y10`   | 2     | 4       |

PWM runs at 10 kHz (`Timer.init(freq=10000)`) — needs 5 kHz+ for the VNH5019 current
sense to read correctly.

Notes:
- CS pins are read via ADC in `MotorDriver`, but per the comment in `two_wheel.py` the
  current-sense pins were **not actually wired** in this test setup.
- Direction logic (`MotorDriver.direct_set_speed`): forward = Dir A high / Dir B low;
  reverse = Dir A low / Dir B high; stop = both low (coast). Speed input range -127..127
  is scaled to 0..100% duty cycle via `max_speed` (default 127).
- `right_scale` (~0.92) compensates for the right motor running faster than the left at
  matched commands (no-load, good ~0.3-0.5 range).
