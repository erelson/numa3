#!/usr/bin/env python3
"""Restart the robot program on the PyBoard, without power-cycling.

A soft reset (Ctrl-D from the FRIENDLY repl) makes the board re-run boot.py and
then its main script -- i.e. the numa main loop starts over from scratch.

    ./restart_robot.py              # soft reset, then stream output
    ./restart_robot.py -w 30        # ...streaming for 30 s
    ./restart_robot.py --no-watch   # trigger and exit immediately
    ./restart_robot.py --hard       # machine.reset() instead (full MCU reset)
    ./restart_robot.py --stop       # just interrupt the running program
    ./restart_robot.py -n           # dry run: resolve everything, send nothing

Why not `mpremote soft-reset`: mpremote talks to the board through the RAW
repl, and MicroPython deliberately skips main.py after a raw-repl soft reset.
That resets the interpreter but never restarts the robot program. So this
script drives the serial port itself and sends, in order:

    Ctrl-C  interrupt whatever is running
    Ctrl-B  leave the raw repl if some tool left us in it (-> friendly repl)
    Ctrl-D  soft reset: re-runs boot.py, then main script

Soft reset keeps the USB serial connection alive on the PyBoard, so output can
be streamed straight through. A --hard reset re-enumerates USB, so the port
drops and the script waits for it to come back.

SAFETY: restarting runs the robot. g8Stand drives every leg to the standing
pose as the first thing it does. On the Robot board this asks for confirmation
unless -y is given. Detaching from --watch does NOT stop the robot; use --stop
for that.
"""

import argparse
import sys
import time

import serial

from move_via_pyboard import BOARDS, find_device, get_board_id

CTRL_C = b"\x03"
CTRL_B = b"\x02"
CTRL_D = b"\x04"


def open_port(device):
    return serial.Serial(device, 115200, timeout=0.1, dsrdtr=False, rtscts=False)


def send_restart(port, settle=0.15):
    """Interrupt, drop to the friendly repl, then soft reset."""
    port.write(CTRL_C)
    port.flush()
    time.sleep(settle)
    port.write(CTRL_C)
    port.flush()
    time.sleep(settle)
    port.write(CTRL_B)   # raw repl -> friendly repl (harmless if already there)
    port.flush()
    time.sleep(settle)
    port.reset_input_buffer()
    port.write(CTRL_D)   # soft reset: boot.py, then the main script
    port.flush()


def send_stop(port, settle=0.15):
    """Interrupt the running program, leaving the board at the repl."""
    for _ in range(2):
        port.write(CTRL_C)
        port.flush()
        time.sleep(settle)


def stream(port, seconds):
    """Echo board output until `seconds` elapse or the user interrupts."""
    end = time.time() + seconds
    try:
        while time.time() < end:
            data = port.read(512)
            if data:
                sys.stdout.write(data.decode("utf-8", "replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n[detached -- the robot is STILL RUNNING; "
              "use --stop to halt it]")
        return
    print("\n[watch window ended after {:g}s -- the robot is still running]"
          .format(seconds))


def wait_for_device(explicit, timeout=15.0):
    """After a hard reset the USB port re-enumerates; wait for it to return."""
    end = time.time() + timeout
    while time.time() < end:
        device = find_device(explicit)
        if device:
            time.sleep(0.5)  # let the CDC settle before opening
            return device
        time.sleep(0.25)
    return None


def main():
    ap = argparse.ArgumentParser(
        description="Restart the robot program on the PyBoard.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="examples:\n"
               "  %(prog)s\n"
               "  %(prog)s -w 30\n"
               "  %(prog)s --hard\n"
               "  %(prog)s --stop\n")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--hard", action="store_true",
                      help="machine.reset() (full MCU reset, USB re-enumerates) "
                           "instead of a soft reset")
    mode.add_argument("--stop", action="store_true",
                      help="just interrupt the running program; do not restart")
    ap.add_argument("-w", "--watch", type=float, default=10.0, metavar="SEC",
                    help="stream board output for this long (default %(default)s)")
    ap.add_argument("--no-watch", action="store_true",
                    help="do not stream output; return immediately")
    ap.add_argument("-d", "--device", metavar="DEV",
                    help="serial device (default: first /dev/ttyACM*)")
    ap.add_argument("-y", "--yes", action="store_true",
                    help="skip the Robot-board confirmation prompt")
    ap.add_argument("-n", "--dry-run", action="store_true",
                    help="resolve everything and print the plan, but send nothing")
    args = ap.parse_args()

    device = find_device(args.device)
    if device is None:
        print("ERROR: no PyBoard found" +
              ("" if args.device is None else " at {}".format(args.device)),
              file=sys.stderr)
        return 2

    board_id = get_board_id(device)
    if board_id is None:
        print("ERROR: found {} but could not read a board id "
              "(board busy? try --stop first)".format(device), file=sys.stderr)
        return 2
    board_name = BOARDS.get(board_id, "unknown")
    if board_name == "unknown":
        print("ERROR: unrecognized board id {}; refusing to act on an unknown "
              "board".format(board_id), file=sys.stderr)
        return 2

    if args.stop:
        action = "Interrupt (stop) the running program"
    elif args.hard:
        action = "HARD reset (machine.reset()) and restart the robot program"
    else:
        action = "Soft reset and restart the robot program"
    print("board:  {} ({}) on {}".format(board_name, board_id, device))
    print("action: {}".format(action))
    if not args.stop:
        print("        the robot will run: g8Stand drives every leg to the "
              "standing pose")

    if args.dry_run:
        print("dry run: nothing sent")
        return 0

    if board_name == "robot" and not args.stop and not args.yes:
        if not sys.stdin.isatty():
            print("ERROR: Robot board and no tty to confirm on; pass -y",
                  file=sys.stderr)
            return 2
        reply = input("This is the ROBOT board. {}? [y/N] ".format(action))
        if reply.strip().lower() not in ("y", "yes"):
            print("aborted")
            return 1

    if args.hard:
        # Done over mpremote: the port disappears as USB re-enumerates.
        import subprocess
        subprocess.run(["mpremote", "connect", device, "exec",
                        "import machine; machine.reset()"],
                       capture_output=True, timeout=30)
        print("hard reset sent; waiting for USB to come back...")
        device = wait_for_device(args.device)
        if device is None:
            print("ERROR: device did not reappear after the hard reset",
                  file=sys.stderr)
            return 2
        print("device back at {}".format(device))
        if args.no_watch:
            return 0
        with open_port(device) as port:
            stream(port, args.watch)
        return 0

    with open_port(device) as port:
        if args.stop:
            send_stop(port)
            print("interrupt sent; board should be at the repl")
            return 0
        send_restart(port)
        if args.no_watch:
            print("soft reset sent")
            return 0
        stream(port, args.watch)
    return 0


if __name__ == "__main__":
    sys.exit(main())
