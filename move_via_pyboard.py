#!/usr/bin/env python3
"""Move a single servo on the Numa robot, via the MicroPython PyBoard.

Host-side helper for manual / bench servo moves. Nothing is written to the
PyBoard filesystem: the move runs through `mpremote ... exec`, importing the
modules already present on the board.

Examples:
    ./move_via_pyboard.py -hw 42 -p 777    # HiWonder id 42 -> position 777
    ./move_via_pyboard.py -hw 42 -a 0      # HiWonder id 42 -> 0 deg from center
    ./move_via_pyboard.py -ax 22 -p 619    # Dynamixel id 22 -> position 619
    ./move_via_pyboard.py -ax 22 -a -30    # Dynamixel id 22 -> -30 deg from center
    ./move_via_pyboard.py -hw 42 -r        # just read id 42's state and exit
                                           # (-r wins over any -p/-a/--release)
    ./move_via_pyboard.py -hw 42 --rel     # stop holding position (avoid overheating)
    ./move_via_pyboard.py -j 12 -J 45      # leg 1 femur -> JOINT angle 45 deg
    ./move_via_pyboard.py -j 42 -J 0       # leg 4 femur -> joint zero (trim check)
    ./move_via_pyboard.py -j 22 -r         # -j resolves bus+id, works with -p/-a/-r too

Joint mode (-j/-J):
    -j takes a joint id (<leg><joint>, e.g. 12 = leg 1 femur; 1=coax 2=femur
    3=tibia) and resolves the bus and servo id from numa/servo_inventory.py.
    -J then commands a JOINT angle in the robot's kinematic convention, routed
    through numa/poses.py LegDef, so the servo type, that unit's trim_deg, the
    bracket aoffset and the leg/joint direction signs all apply -- the same
    conversion the robot itself uses. Contrast -a, which is raw degrees from the
    servo's electrical center with none of that applied.
    The conversion reads this working tree, not the board, so you can check a
    config before deploying it. A target outside the joint's configured limits
    is flagged.
    ./move_via_pyboard.py -ax 22 -p 619 --rel   # move there, then stop holding

Buses (see numa/numa.py):
    HiWonder  HX-35HM  UART 4 @ 115200   center 750, 4.1667 counts/deg (0-1500)
              (--uart N overrides the HiWonder UART for bench wiring tests.)
    Dynamixel AX-12    UART 2 @ 1000000  center 512, 3.4133 counts/deg (0-1023)

Counts-per-degree match hx35hmpos() / ax12pos() in numa/poses.py so that
positions agree with the robot code. This script rounds where poses.py
truncates, so a converted angle can differ by one count.

Board detection: the attached board's unique id is read on every run and
recorded in a flag file (default .pyboard_last_seen.json) along with the
device and timestamp. A move on the Robot board asks for confirmation unless
-y is given; the Testbed board never asks.
"""

import argparse
import glob
import json
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FLAG_FILE = os.path.join(REPO, ".pyboard_last_seen.json")

BOARDS = {
    "3700530005504b4d52323420": "testbed",
    "380046001951363039343332": "robot",
}

# bus name -> (uart, baud, center, counts_per_degree, nominal_max)
BUSES = {
    "hw": (4, 115200, 750, 750.0 / 180.0, 1500),
    "ax": (2, 1000000, 512, 512.0 / 150.0, 1023),
}


def find_device(explicit=None):
    """Return the serial device for the PyBoard, or None."""
    if explicit:
        return explicit if os.path.exists(explicit) else None
    found = sorted(glob.glob("/dev/ttyACM*"))
    return found[0] if found else None


def run_remote(device, code, timeout=120):
    """Run code on the board via mpremote. Returns (returncode, output)."""
    try:
        proc = subprocess.run(
            ["mpremote", "connect", device, "exec", code],
            capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        return 127, "mpremote not found on PATH"
    except subprocess.TimeoutExpired:
        return 124, "timed out after {}s talking to {}".format(timeout, device)
    return proc.returncode, (proc.stdout + proc.stderr).strip()


def get_board_id(device):
    """Read the board's unique id, or None if it could not be read."""
    rc, out = run_remote(
        device,
        "import machine, ubinascii;"
        "print(ubinascii.hexlify(machine.unique_id()).decode())",
        timeout=30)
    if rc != 0:
        return None
    for line in out.splitlines():
        line = line.strip()
        if len(line) == 24 and all(c in "0123456789abcdef" for c in line):
            return line
    return None


def load_flag(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (IOError, ValueError):
        return None


def save_flag(path, board_id, board_name, device):
    data = {
        "board_id": board_id,
        "board_name": board_name,
        "device": device,
        "last_seen": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    try:
        with open(path, "w") as fh:
            json.dump(data, fh, indent=2)
            fh.write("\n")
    except IOError as exc:
        print("warning: could not write flag file {}: {}".format(path, exc),
              file=sys.stderr)
    return data


def angle_to_pos(bus, angle):
    _uart, _baud, center, per_deg, _max = BUSES[bus]
    return int(round(center + angle * per_deg))


# servo kind (poses/servo_inventory naming) -> bus name used above
KIND_TO_BUS = {"ax12": "ax", "ax12a": "ax", "hx-35hm": "hw"}


def _load_robot_config():
    """Import the repo's poses + servo_inventory.

    Host-side on purpose: joint angles convert using THIS working tree's
    config, so a value can be checked before it is deployed to the board.
    """
    path = os.path.join(REPO, "numa")
    if path not in sys.path:
        sys.path.insert(0, path)
    import poses
    import servo_inventory
    return poses, servo_inventory


def resolve_joint(joint_id):
    """Map a joint id (e.g. 12 = leg 1 femur) to (bus, servo_id, kind)."""
    _poses, inv = _load_robot_config()
    try:
        kind = inv.joint_kind(joint_id)
    except KeyError:
        raise ValueError(
            "joint {} is not assigned in numa/servo_inventory.py".format(joint_id))
    if kind not in KIND_TO_BUS:
        raise ValueError("unknown servo kind {!r} at joint {}".format(kind, joint_id))
    return KIND_TO_BUS[kind], inv.joint_bus_id(joint_id), kind


def joint_angle_to_pos(joint_id, joint_angle):
    """Convert a robot joint angle (deg) to a servo position via LegDef.

    Servo type, the unit's trim_deg, the bracket aoffset and the leg/joint
    direction signs all apply -- the same path the robot uses.

    Returns (pos, kind, trim_deg, limit_lo, limit_hi).
    """
    poses, inv = _load_robot_config()
    leg_num, joint = divmod(joint_id, 10)
    if not (1 <= leg_num <= 4 and 1 <= joint <= 3):
        raise ValueError(
            "joint angles need a leg joint 11..43 (got {}); the turret has no "
            "LegDef -- use -p/-a for ids 51/52".format(joint_id))
    legs = poses.gen_numa2_legs(inv.leg_servo_types(), inv.leg_servo_trims())
    leg = legs[leg_num]  # legs[0] is the LegGeom
    angles = [0.0, 0.0, 0.0]
    angles[joint - 1] = joint_angle
    pos = leg.get_pos_from_angle(*angles)[joint - 1]
    lo = (leg.s1min, leg.s2min, leg.s3min)[joint - 1]
    hi = (leg.s1max, leg.s2max, leg.s3max)[joint - 1]
    return pos, leg.servo_types[joint - 1], leg.servo_trims[joint - 1], lo, hi


HW_CODE = """
from stm_uart_port import UART_Port
from hiwonder_bus import Bus, BusError
import hiwonder_packet as pkt
import struct, utime
SID = %(sid)d
POS = %(pos)d
REQ_MS = %(ms)d
RELEASE = %(release)d
bus = Bus(UART_Port(%(uart)d, 115200))
if not bus.ping(SID):
    print('ERROR: no HiWonder servo answering id', SID)
else:
    lo, hi = struct.unpack('<HH', bus.read(SID, pkt.Command.ANGLE_LIMIT_READ))
    p0 = struct.unpack('<h', bus.read(SID, pkt.Command.POS_READ))[0]
    print('limits: {} .. {}'.format(lo, hi))
    print('before: {}  ({:+.2f} deg from center 750)'.format(p0, (p0 - 750) * 0.24))
    if POS < lo or POS > hi:
        print('WARNING: target {} is outside limits {}..{}; the servo will clamp'.format(POS, lo, hi))
        print('WARNING: holding against a clamp heats the servo - do not leave it there')
    d = abs(POS - p0)
    ms = REQ_MS if REQ_MS > 0 else max(300, d * 5)
    if ms > 30000:
        ms = 30000
    print('moving {} counts ({:.1f} deg) over {} ms'.format(d, d * 0.24, ms))
    bus.write(SID, pkt.Command.MOVE_TIME_WRITE, bytearray(struct.pack('<HH', POS, ms)))
    utime.sleep_ms(ms + 400)
    p = struct.unpack('<h', bus.read(SID, pkt.Command.POS_READ))[0]
    print('after:  {}  ({:+.2f} deg from center 750)'.format(p, (p - 750) * 0.24))
    if abs(p - POS) > 3:
        print('NOTE: settled {} counts from target (clamp, stall, or deadband)'.format(abs(p - POS)))
    print('load: {}  temp: {}C  vin: {} mV'.format(
          bus.read(SID, pkt.Command.LOAD_OR_UNLOAD_READ)[0],
          bus.read(SID, pkt.Command.TEMP_READ)[0],
          struct.unpack('<H', bus.read(SID, pkt.Command.VIN_READ))[0]))
    if RELEASE:
        bus.write(SID, pkt.Command.LOAD_OR_UNLOAD_WRITE, bytearray([0]))
        utime.sleep_ms(300)
        print('released: load now {}'.format(bus.read(SID, pkt.Command.LOAD_OR_UNLOAD_READ)[0]))
"""

AX_CODE = """
from stm_uart_port import UART_Port
from bus import Bus, BusError
import ax, struct, utime
SID = %(sid)d
POS = %(pos)d
SPEED = %(speed)d
RELEASE = %(release)d
bus = Bus(UART_Port(2, 1000000))
def w(off, data):
    # RETURN_LEVEL=1 project-wide (numa/init.py): writes are never acknowledged
    try:
        bus.write(SID, off, data)
    except BusError:
        pass
if not bus.ping(SID):
    print('ERROR: no Dynamixel servo answering id', SID)
else:
    cw = struct.unpack('<H', bus.read(SID, ax.CW_ANGLE_LIMIT_L, 2))[0]
    ccw = struct.unpack('<H', bus.read(SID, ax.CCW_ANGLE_LIMIT_L, 2))[0]
    p0 = struct.unpack('<H', bus.read(SID, ax.PRESENT_POSITION, 2))[0]
    print('limits: {} .. {}'.format(cw, ccw))
    print('before: {}  ({:+.2f} deg from center 512)'.format(p0, (p0 - 512) / 3.41333))
    if POS < cw or POS > ccw:
        print('WARNING: target {} is outside limits {}..{}; the servo will clamp'.format(POS, cw, ccw))
        print('WARNING: holding against a clamp heats the servo - do not leave it there')
    d = abs(POS - p0)
    w(ax.MOVING_SPEED, struct.pack('<H', SPEED))
    utime.sleep_ms(25)
    w(ax.TORQUE_ENABLE, bytearray([1]))
    utime.sleep_ms(25)
    deg = d / 3.41333
    est = int(deg / (max(1, SPEED) * 0.666) * 1000) + 700
    print('moving {} counts ({:.1f} deg) at speed {} (~{} ms)'.format(d, deg, SPEED, est))
    w(ax.GOAL_POSITION, struct.pack('<H', POS))
    utime.sleep_ms(est)
    p = struct.unpack('<H', bus.read(SID, ax.PRESENT_POSITION, 2))[0]
    print('after:  {}  ({:+.2f} deg from center 512)'.format(p, (p - 512) / 3.41333))
    if abs(p - POS) > 3:
        print('NOTE: settled {} counts from target (clamp, stall, or deadband)'.format(abs(p - POS)))
    print('torque: {}  temp: {}C'.format(
          bus.read(SID, ax.TORQUE_ENABLE, 1)[0],
          bus.read(SID, ax.PRESENT_TEMP, 1)[0]))
    if RELEASE:
        w(ax.TORQUE_ENABLE, bytearray([0]))
        utime.sleep_ms(300)
        print('released: torque now {}'.format(bus.read(SID, ax.TORQUE_ENABLE, 1)[0]))
"""


HW_READ_CODE = """
from stm_uart_port import UART_Port
from hiwonder_bus import Bus, BusError
import hiwonder_packet as pkt
import struct
SID = %(sid)d
bus = Bus(UART_Port(%(uart)d, 115200))
if not bus.ping(SID):
    print('ERROR: no HiWonder servo answering id', SID)
else:
    lo, hi = struct.unpack('<HH', bus.read(SID, pkt.Command.ANGLE_LIMIT_READ))
    p = struct.unpack('<h', bus.read(SID, pkt.Command.POS_READ))[0]
    print('position: {}  ({:+.2f} deg from center 750)'.format(p, (p - 750) * 0.24))
    print('limits:   {} .. {}'.format(lo, hi))
    print('load:     {}'.format(bus.read(SID, pkt.Command.LOAD_OR_UNLOAD_READ)[0]))
    print('temp:     {} C'.format(bus.read(SID, pkt.Command.TEMP_READ)[0]))
    print('vin:      {} mV'.format(struct.unpack('<H', bus.read(SID, pkt.Command.VIN_READ))[0]))
    print('offset:   {}'.format(struct.unpack('<b', bus.read(SID, pkt.Command.ANGLE_OFFSET_READ))[0]))
"""

AX_READ_CODE = """
from stm_uart_port import UART_Port
from bus import Bus, BusError
import ax, struct
SID = %(sid)d
bus = Bus(UART_Port(2, 1000000))
if not bus.ping(SID):
    print('ERROR: no Dynamixel servo answering id', SID)
else:
    cw = struct.unpack('<H', bus.read(SID, ax.CW_ANGLE_LIMIT_L, 2))[0]
    ccw = struct.unpack('<H', bus.read(SID, ax.CCW_ANGLE_LIMIT_L, 2))[0]
    p = struct.unpack('<H', bus.read(SID, ax.PRESENT_POSITION, 2))[0]
    print('position: {}  ({:+.2f} deg from center 512)'.format(p, (p - 512) / 3.41333))
    print('limits:   {} .. {}'.format(cw, ccw))
    print('torque:   {}'.format(bus.read(SID, ax.TORQUE_ENABLE, 1)[0]))
    print('speed:    {}  (0 means MAX)'.format(struct.unpack('<H', bus.read(SID, ax.MOVING_SPEED, 2))[0]))
    print('temp:     {} C'.format(bus.read(SID, ax.PRESENT_TEMP, 1)[0]))
    # addr 42 = PRESENT_VOLTAGE (0.1 V units). Literal, not ax.PRESENT_VOLTAGE:
    # the constant is new in numa/ax.py and the board's copy may predate it.
    print('voltage:  {:.1f} V'.format(bus.read(SID, 42, 1)[0] / 10.0))
    print('rtn_lvl:  {}  (1 = writes unacknowledged, project default)'.format(
          bus.read(SID, ax.RETURN_LEVEL, 1)[0]))
"""

HW_RELEASE_CODE = """
from stm_uart_port import UART_Port
from hiwonder_bus import Bus, BusError
import hiwonder_packet as pkt
import struct, utime
SID = %(sid)d
bus = Bus(UART_Port(%(uart)d, 115200))
if not bus.ping(SID):
    print('ERROR: no HiWonder servo answering id', SID)
else:
    p0 = struct.unpack('<h', bus.read(SID, pkt.Command.POS_READ))[0]
    print('before: position {}  load {}  temp {} C'.format(
          p0, bus.read(SID, pkt.Command.LOAD_OR_UNLOAD_READ)[0],
          bus.read(SID, pkt.Command.TEMP_READ)[0]))
    bus.write(SID, pkt.Command.LOAD_OR_UNLOAD_WRITE, bytearray([0]))
    utime.sleep_ms(300)
    p = struct.unpack('<h', bus.read(SID, pkt.Command.POS_READ))[0]
    load = bus.read(SID, pkt.Command.LOAD_OR_UNLOAD_READ)[0]
    print('after:  position {}  load {}  temp {} C'.format(
          p, load, bus.read(SID, pkt.Command.TEMP_READ)[0]))
    if load != 0:
        print('WARNING: servo still reports load={}'.format(load))
    if abs(p - p0) > 3:
        print('NOTE: joint moved {} counts on release - it was holding against a force'.format(abs(p - p0)))
"""

AX_RELEASE_CODE = """
from stm_uart_port import UART_Port
from bus import Bus, BusError
import ax, struct, utime
SID = %(sid)d
bus = Bus(UART_Port(2, 1000000))
def w(off, data):
    # RETURN_LEVEL=1 project-wide (numa/init.py): writes are never acknowledged
    try:
        bus.write(SID, off, data)
    except BusError:
        pass
if not bus.ping(SID):
    print('ERROR: no Dynamixel servo answering id', SID)
else:
    p0 = struct.unpack('<H', bus.read(SID, ax.PRESENT_POSITION, 2))[0]
    print('before: position {}  torque {}  temp {} C'.format(
          p0, bus.read(SID, ax.TORQUE_ENABLE, 1)[0],
          bus.read(SID, ax.PRESENT_TEMP, 1)[0]))
    w(ax.TORQUE_ENABLE, bytearray([0]))
    utime.sleep_ms(300)
    p = struct.unpack('<H', bus.read(SID, ax.PRESENT_POSITION, 2))[0]
    torque = bus.read(SID, ax.TORQUE_ENABLE, 1)[0]
    print('after:  position {}  torque {}  temp {} C'.format(
          p, torque, bus.read(SID, ax.PRESENT_TEMP, 1)[0]))
    if torque != 0:
        print('WARNING: servo still reports torque={}'.format(torque))
    if abs(p - p0) > 3:
        print('NOTE: joint moved {} counts on release - it was holding against a force'.format(abs(p - p0)))
"""

def main():
    ap = argparse.ArgumentParser(
        description="Move one servo via the PyBoard.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="examples:\n"
               "  %(prog)s -hw 42 -p 777\n"
               "  %(prog)s -hw 42 -a 0\n"
               "  %(prog)s -ax 22 -p 619\n"
               "  %(prog)s -ax 22 -a -30\n"
               "  %(prog)s -hw 42 -r\n"
               "  %(prog)s -hw 42 --rel\n"
               "  %(prog)s -j 12 -J 45      (leg 1 femur -> joint angle 45 deg)\n"
               "  %(prog)s -j 42 -J 0       (leg 4 femur -> joint zero)\n")
    which = ap.add_mutually_exclusive_group(required=True)
    which.add_argument("-hw", type=int, metavar="ID",
                       help="HiWonder servo id (UART 4)")
    which.add_argument("-ax", type=int, metavar="ID",
                       help="Dynamixel AX servo id (UART 2)")
    which.add_argument("-j", "--joint", type=int, metavar="JOINT",
                       help="joint id from servo_inventory (<leg><joint>, e.g. "
                            "12 = leg 1 femur); resolves bus + servo id")
    target = ap.add_mutually_exclusive_group(required=False)
    target.add_argument("-p", "--position", type=int, metavar="POS",
                        help="absolute servo position in counts")
    target.add_argument("-a", "--angle", type=float, metavar="DEG",
                        help="degrees from the servo's electrical center (raw; "
                             "no trim/aoffset/sign applied)")
    target.add_argument("-J", "--joint-angle", type=float, metavar="DEG",
                        dest="joint_angle",
                        help="JOINT angle in the robot's kinematic convention "
                             "(requires -j). Converted via LegDef, so servo "
                             "type, trim, bracket aoffset and direction signs "
                             "all apply.")
    ap.add_argument("-r", "--read", action="store_true",
                    help="read and print the servo's current state, then exit. "
                         "Overrides everything: any -p/-a/--release is ignored.")
    ap.add_argument("--release", "--rel", action="store_true", dest="release",
                    help="stop the motor holding position (HiWonder: unload; "
                         "AX: torque off). Use alone, or after -p/-a to move "
                         "then let go. A loaded joint may sag or drop.")
    ap.add_argument("--time", type=int, default=0, metavar="MS",
                    help="HiWonder travel time in ms (default: counts*5, ~48 deg/s)")
    ap.add_argument("--speed", type=int, default=200, metavar="N",
                    help="AX MOVING_SPEED, 1-1023 (default 200, ~133 deg/s; 0 means MAX)")
    ap.add_argument("--uart", type=int, default=BUSES["hw"][0], metavar="N",
                    help="UART for the HiWonder bus (default: %(default)s). "
                         "Ignored for AX, which is UART 2.")
    ap.add_argument("-d", "--device", metavar="DEV",
                    help="serial device (default: first /dev/ttyACM*)")
    ap.add_argument("--flag-file", default=DEFAULT_FLAG_FILE, metavar="PATH",
                    help="where to record the last-seen board (default: %(default)s)")
    ap.add_argument("-y", "--yes", action="store_true",
                    help="skip the Robot-board confirmation prompt")
    ap.add_argument("-n", "--dry-run", action="store_true",
                    help="resolve everything and print the target, but do not move")
    args = ap.parse_args()

    if not (args.read or args.release or args.position is not None
            or args.angle is not None or args.joint_angle is not None):
        ap.error("one of -p/-a/-J/-r/--release is required")
    if args.joint_angle is not None and args.joint is None:
        ap.error("-J/--joint-angle requires -j/--joint (it needs the leg+joint)")

    if args.joint is not None:
        try:
            bus, sid, _kind = resolve_joint(args.joint)
        except Exception as exc:
            print("ERROR: could not resolve joint {}: {}".format(args.joint, exc),
                  file=sys.stderr)
            return 2
    else:
        bus = "hw" if args.hw is not None else "ax"
        sid = args.hw if bus == "hw" else args.ax
    _uart, _baud, center, per_deg, nominal = BUSES[bus]

    limits = None
    if args.read:
        pos = None
        how = "read current state"
    elif (args.position is None and args.angle is None
            and args.joint_angle is None):
        pos = None
        how = "release (stop holding position)"
    elif args.position is not None:
        pos = args.position
        how = "position {}".format(pos)
    elif args.joint_angle is not None:
        try:
            pos, kind, trim, lo, hi = joint_angle_to_pos(args.joint, args.joint_angle)
        except Exception as exc:
            print("ERROR: {}".format(exc), file=sys.stderr)
            return 2
        limits = (lo, hi)
        how = ("joint angle {:+g} deg -> position {}  [{}, trim {:+g} deg, "
               "config limits {}..{}]".format(
                   args.joint_angle, pos, kind, trim, lo, hi))
    else:
        pos = angle_to_pos(bus, args.angle)
        how = "{:+g} deg from center {} -> position {}".format(args.angle, center, pos)

    if pos is not None and limits is not None and not (limits[0] <= pos <= limits[1]):
        print("warning: {} is outside this joint's configured limits {}..{} "
              "(poses.BRACKET_GEOM + trim); the servo will clamp"
              .format(pos, limits[0], limits[1]), file=sys.stderr)

    if pos is not None:
        if pos < 0:
            ap.error("target position {} is negative".format(pos))
        if pos > nominal:
            print("warning: target {} is above the nominal {} range 0..{}"
                  .format(pos, bus, nominal), file=sys.stderr)

    device = find_device(args.device)
    if device is None:
        prev = load_flag(args.flag_file)
        print("ERROR: no PyBoard found" +
              ("" if args.device is None else " at {}".format(args.device)),
              file=sys.stderr)
        if prev:
            print("       last seen: {} ({}) on {} at {}".format(
                  prev.get("board_name"), prev.get("board_id"),
                  prev.get("device"), prev.get("last_seen")), file=sys.stderr)
        else:
            print("       no previous board recorded in {}".format(args.flag_file),
                  file=sys.stderr)
        return 2

    board_id = get_board_id(device)
    if board_id is None:
        print("ERROR: found {} but could not read a board id "
              "(board busy? try Ctrl-C in your own REPL)".format(device),
              file=sys.stderr)
        return 2
    board_name = BOARDS.get(board_id, "unknown")

    prev = load_flag(args.flag_file)
    if prev and prev.get("board_id") and prev["board_id"] != board_id:
        print("note: board changed since last run ({} -> {})".format(
              prev.get("board_name"), board_name))
    save_flag(args.flag_file, board_id, board_name, device)

    print("board:  {} ({}) on {}".format(board_name, board_id, device))
    if args.release and pos is not None:
        how += ", then release"
    label = "{} servo {}".format(bus, sid)
    if bus == "hw" and args.uart != BUSES["hw"][0]:
        label += " (UART {})".format(args.uart)
    if args.joint is not None:
        label = "joint {} ({})".format(args.joint, label)
    print("target: {} -> {}".format(label, how))

    if args.read:
        ignored = []
        if args.position is not None:
            ignored.append("-p")
        if args.angle is not None:
            ignored.append("-a")
        if args.joint_angle is not None:
            ignored.append("-J")
        if args.release:
            ignored.append("--release")
        if ignored:
            print("note: -r/--read overrides; ignoring {}".format(", ".join(ignored)))

    if board_name == "unknown":
        print("ERROR: unrecognized board id {}; refusing to move a servo "
              "on an unknown board".format(board_id), file=sys.stderr)
        return 2

    if args.dry_run:
        print("dry run: not {}".format(
              "reading" if args.read else
              "releasing" if pos is None else "moving"))
        return 0

    if args.read:
        # Reads are harmless on any board, so no confirmation prompt.
        code = (HW_READ_CODE if bus == "hw" else AX_READ_CODE) % {
            "sid": sid, "uart": args.uart}
        rc, out = run_remote(device, code, timeout=60)
        if out:
            print(out)
        if rc != 0:
            print("ERROR: mpremote exited {}".format(rc), file=sys.stderr)
            return rc
        return 1 if "ERROR:" in out else 0

    if pos is None:
        action = "Release (stop holding) {}".format(label)
    elif args.release:
        action = "Move {} to {}, then release".format(label, pos)
    else:
        action = "Move {} to {}".format(label, pos)

    if board_name == "robot" and not args.yes:
        if not sys.stdin.isatty():
            print("ERROR: Robot board and no tty to confirm on; pass -y to proceed",
                  file=sys.stderr)
            return 2
        reply = input("This is the ROBOT board. {}? [y/N] ".format(action))
        if reply.strip().lower() not in ("y", "yes"):
            print("aborted")
            return 1

    if pos is None:
        code = (HW_RELEASE_CODE if bus == "hw" else AX_RELEASE_CODE) % {
            "sid": sid, "uart": args.uart}
    elif bus == "hw":
        code = HW_CODE % {"sid": sid, "uart": args.uart, "pos": pos,
                          "ms": max(0, args.time),
                          "release": int(args.release)}
    else:
        code = AX_CODE % {"sid": sid, "pos": pos,
                          "speed": max(0, min(1023, args.speed)),
                          "release": int(args.release)}

    rc, out = run_remote(device, code, timeout=180)
    if out:
        print(out)
    if rc != 0:
        print("ERROR: mpremote exited {}".format(rc), file=sys.stderr)
        return rc
    return 1 if "ERROR:" in out else 0


if __name__ == "__main__":
    sys.exit(main())
