#!/usr/bin/env python3
"""Minify and deploy Numa's PyBoard code. Port of numa2's create_upload_files.py.

Stages minified copies of the robot sources into micropy-to-upload/, detects
which files actually changed by comparing SHA-256 hashes against the files ON
THE BOARD (hashed board-side), and optionally writes the changed files to the
attached PyBoard via rshell, verifying each transfer by re-hashing it on the
board (transfers silently truncate when the board's flash is full - see
2025-09 robot board incident).

A dry run (no --write) leaves micropy-to-upload/ untouched. If no board is
reachable, a dry run falls back to comparing against the previous staging run.

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
import time

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
    "numa/servo_inventory.py",
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

# Rough rshell upload timing, only used for the progress note. rshell costs a
# few seconds per file regardless of size (a flat 120 s was not enough for a
# 17-file deploy); the per-KB term is a guess. The measured time is printed
# after each write - tune these against it.
RSHELL_BASE_S = 5
RSHELL_PER_FILE_S = 7
RSHELL_PER_KB_S = 0.05


# MicroPython's inline assembler parses one instruction per statement, so
# minification (which joins statements with ';') turns an @micropython.asm_thumb
# body into "SyntaxError: expecting an assembler instruction" at import time.
# viper/native are ordinary Python syntax, but they are performance-critical
# enough that shipping them verbatim is the safer default too.
NO_MINIFY_MARKERS = ("@micropython.asm_thumb",
                     "@micropython.viper",
                     "@micropython.native")


def minify_source(source, mode):
    if mode == "full":
        return source
    if any(marker in source for marker in NO_MINIFY_MARKERS):
        return source  # ships verbatim; see NO_MINIFY_MARKERS
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


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def file_sha256(path):
    if not os.path.isfile(path):
        return None
    with open(path, 'rb') as f:
        return sha256(f.read())


def blocks(n_bytes):
    return (n_bytes + BLOCK_SIZE - 1) // BLOCK_SIZE


def stage(sources, mode, force, board_info, commit):
    """Minify sources; returns the staged paths that differ from the baseline.

    The baseline is the file's hash on the board when board_info ({basename:
    (size, sha256)}) is given, else the previous staging run's copy. Staged
    files are only (over)written in STAGING when commit is true, so a dry run
    doesn't consume the change it reports.
    """
    if commit:
        os.makedirs(STAGING, exist_ok=True)
    changed = []
    total_bytes = 0
    print("%-24s %8s %7s" % ("file", "bytes", "blocks"))
    for rel in sources:
        src = os.path.join(REPO, rel)
        base = os.path.basename(rel)
        out = os.path.join(STAGING, base)
        with open(src) as f:
            source = f.read()
        result = minify_source(source, mode)
        compile(result, out, 'exec')  # syntax check before it can reach the board
        payload = result.encode('utf-8')
        if board_info is not None:
            old_hash = board_info[base][1] if base in board_info else None
        else:
            old_hash = file_sha256(out)
        if commit:
            with open(out, 'wb') as f:
                f.write(payload)
        size = len(payload)
        total_bytes += blocks(size) * BLOCK_SIZE
        note = ""
        if force or old_hash != sha256(payload):
            changed.append(out)
            note = "  (changed)"
            if board_info is not None and base not in board_info:
                note = "  (changed, not on board)"
        print("%-24s %8d %7d%s" % (base, size, blocks(size), note))
    print("Total: %d bytes in %d blocks (board flash available: %d blocks / %d bytes)"
          % (total_bytes, total_bytes // BLOCK_SIZE,
             FLASH_BLOCKS, FLASH_BLOCKS * BLOCK_SIZE))
    return changed


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


# Runs on the board. Hashes the raw file bytes in small chunks (no RAM spike,
# no serial newline translation) and prints "name size sha256" per file that
# exists. SHA-256 is always built into the STM32 port; MD5/SHA-1 are optional.
BOARD_INFO_CODE = (
    "import os, ubinascii\n"
    "try: import hashlib\n"
    "except ImportError: import uhashlib as hashlib\n"
    "for f in %r:\n"
    "    try:\n"
    "        p = '%s/' + f\n"
    "        h = hashlib.sha256()\n"
    "        with open(p, 'rb') as fh:\n"
    "            while True:\n"
    "                b = fh.read(512)\n"
    "                if not b: break\n"
    "                h.update(b)\n"
    "        print(f, os.stat(p)[6], ubinascii.hexlify(h.digest()).decode())\n"
    "    except OSError: pass\n"
)


def board_file_info(dest, names):
    """{basename: (size_in_bytes, sha256_hex)} for names that exist on the board."""
    listing = mpremote_exec(BOARD_INFO_CODE % (list(names), dest))
    info = {}
    for line in listing.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1].isdigit():
            info[parts[0]] = (int(parts[1]), parts[2])
    return info


def write_to_board(changed, dest, board_info):
    free = int(mpremote_exec(
        "import os; st = os.statvfs('%s'); print(st[0]*st[3])" % dest).strip())

    # Replacing a file frees its old blocks before the new content is written,
    # so the space actually required is the GROWTH, not the full payload. Sum
    # only the positive per-file deltas: that is the worst case regardless of
    # the order rshell writes them in (all growing files first).
    growth = net = 0
    for f in changed:
        base = os.path.basename(f)
        new_b = blocks(os.stat(f).st_size)
        old_b = blocks(board_info[base][0]) if base in board_info else 0
        growth += max(0, new_b - old_b) * BLOCK_SIZE
        net += (new_b - old_b) * BLOCK_SIZE
    print("Board %s free: %d bytes" % (dest, free))
    print("Upload: %d bytes of new blocks, %+d bytes net after reclaiming "
          "replaced files" % (growth, net))
    if growth > free:
        sys.exit("Not enough free space: needs %d bytes of new blocks but only "
                 "%d free (writes would truncate silently). Free up flash or "
                 "deploy fewer files." % (growth, free))

    total_kb = sum(os.stat(f).st_size for f in changed) / 1024.0
    estimate = (RSHELL_BASE_S + RSHELL_PER_FILE_S * len(changed)
                + RSHELL_PER_KB_S * total_kb)
    print("\nUploading %d file(s), %.1f KB via rshell: this is slow, expect "
          "roughly %d s (plus a few s to verify).\nDon't interrupt it - an "
          "aborted copy can leave a truncated file on the board."
          % (len(changed), total_kb, 5 * round(estimate / 5.0)))
    started = time.time()
    rshell_dest = "/pyboard" + dest + "/"
    cmd = ["rshell", "-p", DEVICE, "cp"] + changed + [rshell_dest]
    # Scale with the file count: rshell takes several seconds per file, and a
    # timeout here kills the copy mid-write, leaving a TRUNCATED file on the
    # board (observed with a flat 120 s on a 17-file --force deploy).
    subprocess.run(cmd, check=True, capture_output=True,
                   timeout=max(180, 30 * len(changed)))
    print("Upload took %.0f s (estimated %.0f s)." % (time.time() - started,
                                                       estimate))

    # Re-hash on the board so the check covers what actually landed in flash
    after = board_file_info(dest, [os.path.basename(f) for f in changed])
    failures = []
    for f in changed:
        base = os.path.basename(f)
        if base not in after:
            status = "MISSING"
        elif after[base][1] != file_sha256(f):
            status = "MISMATCH"
        else:
            status = "OK"
        if status != "OK":
            failures.append(f)
        print("  verify %-24s %s" % (base, status))
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
                   help="treat all files as changed (normally unneeded: files "
                        "are compared against the board's copies)")
    p.add_argument("-w", "--write", action="store_true",
                   help="write changed files to the attached PyBoard")
    p.add_argument("--dest", choices=("/flash", "/sd"), default="/flash",
                   help="target filesystem on the board (default /flash)")
    p.add_argument("--include-boot", action="store_true",
                   help="also deploy boot.py (changes which main program runs!)")
    args = p.parse_args()

    mode = "full" if args.full else ("readable" if args.readable else "minify")
    sources = SOURCES + ([BOOT_FILE] if args.include_boot else [])

    # Baseline for change detection: the files on the board itself.
    board_info = None
    try:
        board_id, name = board_identity()
        if args.write and name is None:
            sys.exit("Unknown board ID %s - aborting. Add it to BOARD_IDS if "
                     "this board is yours." % board_id)
        print("Attached board: %s (%s)\n" % (name or "UNKNOWN", board_id))
        board_info = board_file_info(
            args.dest, [os.path.basename(s) for s in sources])
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as e:
        if args.write:
            sys.exit("Cannot read the attached board: %s" % e)
        print("Board not reachable (%s);\ncomparing against the previous "
              "staging run instead.\n" % e)

    changed = stage(sources, mode, args.force, board_info, commit=args.write)

    if not args.write:
        print("\nDry run (no --write). Files that would upload:",
              ", ".join(os.path.basename(f) for f in changed) or "none")
        return
    if not changed:
        print("\nBoard is up to date; nothing to write.")
        return
    write_to_board(changed, args.dest, board_info)


if __name__ == "__main__":
    main()
