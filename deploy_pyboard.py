#!/usr/bin/env python3
"""Minify and deploy Numa's PyBoard code. Port of numa2's create_upload_files.py.

Stages minified copies of the robot sources into micropy-to-upload/, detects
which files actually changed (md5), and optionally writes the changed files
to an attached PyBoard via rshell, verifying each transfer by reading it back
(transfers silently truncate when the board's flash is full - see 2025-09
robot board incident).

Minification uses python-minifier configured to keep the output readable:
names are preserved; only comments, docstrings, blank lines, and indentation
are stripped. --readable does a plain comment/blank-line strip instead, and
--full disables minification entirely.

The bioloid3 modules (bus.py, stm_uart_port.py, ...) come from the bioloid3/
git submodule (github.com/erelson/bioloid3, branch erelson_fixes_1); run
`git submodule update --init` after cloning to populate it.

boot.py is NOT deployed by default: the robot board's boot.py currently
selects numa2.py via pyb.main(), and replacing it switches the robot to the
new codebase. Use --include-boot deliberately.

Usage:
    python3 deploy_pyboard.py              # stage + report only
    python3 deploy_pyboard.py --write      # stage, then write changes to board
"""
import argparse
import hashlib
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.abspath(__file__))

SOURCES = [
    "numa/numa.py",
    "numa/IK.py",
    "numa/poses.py",
    "numa/init.py",
    "numa/commander.py",
    "numa/helpers.py",
    "numa/ax.py",
    "numa/MotorDriver.py",
    "numa/servo_group.py",
    "hiwonder/hiwonder_bus.py",
    "hiwonder/hiwonder_packet.py",
    # bioloid3 submodule (github.com/erelson/bioloid3, branch erelson_fixes_1)
    "bioloid3/bus.py",
    "bioloid3/packet.py",
    "bioloid3/stm_uart_port.py",
    "bioloid3/log.py",
    "bioloid3/dump_mem.py",
]
BOOT_FILE = "numa/boot.py"

STAGING = os.path.join(REPO, "micropy-to-upload")

# PyBoard v1.1 internal flash: 190 blocks of 512 bytes (~95KB). FAT allocates
# whole blocks, so each file's cost rounds up to a block multiple.
BLOCK_SIZE = 512
FLASH_BLOCKS = 190

BOARD_IDS = {
    "3700530005504b4d52323420": "Testbed",
    "380046001951363039343332": "Robot",
}

DEVICE = "/dev/ttyACM0"


def minify_source(source, mode):
    if mode == "full":
        return source
    if mode == "readable":
        # Drop blank lines and whole-line comments only (inline '#' may live
        # inside string literals, so those lines are left alone)
        lines = [ln for ln in source.splitlines(True)
                 if ln.strip() and not ln.lstrip().startswith('#')]
        return "".join(lines)
    import python_minifier
    return python_minifier.minify(
        source,
        rename_locals=False,
        rename_globals=False,
        hoist_literals=False,
        remove_literal_statements=True,  # docstrings
        remove_asserts=False,            # numa.py's board-ID guardrail is an assert
    )


def md5(path):
    if not os.path.isfile(path):
        return None
    with open(path, 'rb') as f:
        return hashlib.md5(f.read()).hexdigest()


def blocks(n_bytes):
    return (n_bytes + BLOCK_SIZE - 1) // BLOCK_SIZE


def stage(sources, mode, force):
    """Minify sources into STAGING. Returns (all_staged, changed) filename lists."""
    os.makedirs(STAGING, exist_ok=True)
    staged, changed = [], []
    total_bytes = 0
    print("%-24s %8s %7s" % ("file", "bytes", "blocks"))
    for rel in sources:
        src = os.path.join(REPO, rel)
        out = os.path.join(STAGING, os.path.basename(rel))
        with open(src) as f:
            source = f.read()
        result = minify_source(source, mode)
        compile(result, out, 'exec')  # syntax check before it can reach the board
        old_hash = md5(out)
        with open(out, 'w') as f:
            f.write(result)
        size = os.stat(out).st_size
        total_bytes += blocks(size) * BLOCK_SIZE
        staged.append(out)
        if force or old_hash != md5(out):
            changed.append(out)
        print("%-24s %8d %7d%s" % (os.path.basename(rel), size, blocks(size),
                                   "  (changed)" if out in changed else ""))
    print("Total: %d bytes in %d blocks (board flash: %d blocks / %d bytes)"
          % (total_bytes, total_bytes // BLOCK_SIZE,
             FLASH_BLOCKS, FLASH_BLOCKS * BLOCK_SIZE))
    return staged, changed


def mpremote_exec(code):
    out = subprocess.run(
        ["mpremote", "connect", DEVICE, "exec", code],
        capture_output=True, text=True, timeout=30)
    if out.returncode != 0:
        raise RuntimeError("mpremote failed: " + out.stderr.strip())
    return out.stdout


def board_identity():
    out = mpremote_exec(
        "import machine, ubinascii;"
        "print(ubinascii.hexlify(machine.unique_id()).decode())")
    board_id = out.strip()
    return board_id, BOARD_IDS.get(board_id)


def board_read_file(board_path):
    out = subprocess.run(
        ["mpremote", "connect", DEVICE, "cat", ":" + board_path],
        capture_output=True, text=True, timeout=30)
    if out.returncode != 0:
        return None
    # serial console adds \r to every line
    return out.stdout.replace('\r\n', '\n')


def write_to_board(changed, dest):
    board_id, name = board_identity()
    if name is None:
        sys.exit("Unknown board ID %s - aborting. Add it to BOARD_IDS if this "
                 "board is yours." % board_id)
    print("\nAttached board: %s (%s)" % (name, board_id))

    free = int(mpremote_exec(
        "import os; st = os.statvfs('%s'); print(st[0]*st[3])" % dest).strip())
    need = sum(blocks(os.stat(f).st_size) * BLOCK_SIZE for f in changed)
    print("Board %s free: %d bytes; upload needs up to %d bytes" % (dest, free, need))
    if need > free:
        sys.exit("Not enough free space (writes would truncate silently). "
                 "Free up flash or deploy fewer files.")

    rshell_dest = "/pyboard" + dest + "/"
    cmd = ["rshell", "-p", DEVICE, "cp"] + changed + [rshell_dest]
    subprocess.run(cmd, check=True, capture_output=True, timeout=120)

    failures = []
    for f in changed:
        with open(f) as fh:
            local = fh.read()
        remote = board_read_file(dest + "/" + os.path.basename(f))
        status = "OK" if remote == local else "MISMATCH"
        if status != "OK":
            failures.append(f)
        print("  verify %-24s %s" % (os.path.basename(f), status))
    if failures:
        sys.exit("Verification failed for: %s - board copy is not trustworthy."
                 % ", ".join(os.path.basename(f) for f in failures))
    print("Wrote and verified %d file(s)." % len(changed))


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("-f", "--full", action="store_true",
                   help="don't minify staged files")
    p.add_argument("-r", "--readable", action="store_true",
                   help="only strip blank lines and comment lines")
    p.add_argument("--force", action="store_true",
                   help="treat all files as changed")
    p.add_argument("-w", "--write", action="store_true",
                   help="write changed files to the attached PyBoard")
    p.add_argument("--dest", choices=("/flash", "/sd"), default="/flash",
                   help="target filesystem on the board (default /flash)")
    p.add_argument("--include-boot", action="store_true",
                   help="also deploy boot.py (changes which main program runs!)")
    args = p.parse_args()

    mode = "full" if args.full else ("readable" if args.readable else "minify")
    sources = SOURCES + ([BOOT_FILE] if args.include_boot else [])
    staged, changed = stage(sources, mode, args.force)

    if not args.write:
        print("\nDry run (no --write). Files that would upload:",
              ", ".join(os.path.basename(f) for f in changed) or "none")
        return
    if not changed:
        print("\nNothing changed; nothing to write.")
        return
    write_to_board(changed, args.dest)


if __name__ == "__main__":
    main()
