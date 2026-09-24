"""PC test for ServoGroup bus/protocol routing.

Run with:  python3 test_servo_group.py

Regression guard for a real failure: ServoGroup used to test `kind == 'ax'`,
but callers pass the pos/center_lookup keys from poses.py ('ax12', 'ax12a',
'hx-35hm'). 'ax12' != 'ax', so every AX servo fell into the HiWonder branch and
got a HiWonder-shaped command sent over the Dynamixel bus via bus.write(),
which waits for a status packet that never arrives (RETURN_LEVEL=1) -- crashing
startup in g8Stand with "BusError: Rcvd Status: Timeout".

So these tests use the REAL kind strings, never a short bus name.
"""
import sys
import os
_HERE = os.path.dirname(os.path.abspath(__file__))
for _sib in ("hiwonder", "feetech"):
    sys.path.insert(0, os.path.join(_HERE, os.pardir, _sib))
import servo_group as SG


class FakeBus:
    """Records calls so we can assert which protocol each servo received."""
    def __init__(self, name):
        self.name = name
        self.writes = []       # individual writes (HiWonder style)
        self.sync_writes = []  # batched writes (AX style)
        self.sync_writes_full = []  # (ids, offset, values) for detail checks

    def write(self, dev_id, cmd, data):
        self.writes.append((dev_id, cmd))

    def sync_write(self, ids, offset, values):
        self.sync_writes.append((list(ids), offset))
        self.sync_writes_full.append((list(ids), offset,
                                      [bytes(v) for v in values]))


def build():
    """A realistic mixed group: AX coax/tibia, one HiWonder femur."""
    ax_bus, hw_bus = FakeBus("ax"), FakeBus("hw")
    servos = [SG.Servo(11, "ax12", ax_bus),
              SG.Servo(22, "ax12a", ax_bus),   # variant must route as AX too
              SG.Servo(42, "hx-35hm", hw_bus),
              SG.Servo(13, "ax12", ax_bus)]
    return SG.ServoGroup(servos), ax_bus, hw_bus


def test_kind_classification():
    grp, _, _ = build()
    assert [s.protocol for s in grp.servos] == [
        SG.PROTOCOL_AX, SG.PROTOCOL_AX, SG.PROTOCOL_HW, SG.PROTOCOL_AX]


def test_unknown_kind_fails_fast():
    # An unregistered kind must raise at construction, not be silently lumped
    # in with AX (which is what a "not is_hw" style test would do).
    try:
        SG.Servo(99, "some-new-servo", FakeBus("x"))
    except ValueError as exc:
        assert "KIND_PROTOCOL" in str(exc), exc
    else:
        assert False, "unknown kind should have raised"


def test_third_protocol_is_not_treated_as_ax():
    # Register a hypothetical third protocol: it must be excluded from the AX
    # batch rather than silently joining it.
    SG.KIND_PROTOCOL["zz-future"] = "zz"
    try:
        ax_bus, other = FakeBus("ax"), FakeBus("zz")
        grp = SG.ServoGroup([SG.Servo(11, "ax12", ax_bus),
                             SG.Servo(77, "zz-future", other)])
        grp.write_positions([100, 200])
        assert ax_bus.sync_writes[0][0] == [11], ax_bus.sync_writes
        assert other.writes == [] and other.sync_writes == []
    finally:
        del SG.KIND_PROTOCOL["zz-future"]


def test_positions_route_by_protocol():
    grp, ax_bus, hw_bus = build()
    grp.write_positions([100, 200, 871, 400], move_ms=36)
    # AX servos batched into ONE sync_write, never individual writes
    assert ax_bus.writes == [], ax_bus.writes
    assert len(ax_bus.sync_writes) == 1
    assert ax_bus.sync_writes[0][0] == [11, 22, 13]
    # HiWonder servo gets its own write, and nothing on the AX bus
    assert [d for d, _ in hw_bus.writes] == [42]
    assert hw_bus.sync_writes == []


def test_include_hw_false_skips_hiwonder_only():
    grp, ax_bus, hw_bus = build()
    grp.write_positions([100, 200, 871, 400], move_ms=36, include_hw=False)
    assert len(ax_bus.sync_writes) == 1      # AX still written every pass
    assert hw_bus.writes == []               # HiWonder skipped


def test_torque_routes_by_protocol():
    grp, ax_bus, hw_bus = build()
    grp.write_torque(True)
    assert ax_bus.writes == []
    assert ax_bus.sync_writes[0][0] == [11, 22, 13]
    assert [d for d, _ in hw_bus.writes] == [42]


def test_angle_limits_route_by_protocol():
    grp, ax_bus, hw_bus = build()
    grp.write_angle_limits([[1, 2], [3, 4], [5, 6], [7, 8]])
    assert ax_bus.writes == []
    assert len(ax_bus.sync_writes) == 2      # CW then CCW
    assert all(sw[0] == [11, 22, 13] for sw in ax_bus.sync_writes)
    assert [d for d, _ in hw_bus.writes] == [42]


def test_ax_only_helpers_skip_hiwonder():
    grp, ax_bus, hw_bus = build()
    grp.write_speed(300)
    grp.write_return_level(1)
    assert [sw[0] for sw in ax_bus.sync_writes] == [[11, 22, 13], [11, 22, 13]]
    assert hw_bus.writes == [] and hw_bus.sync_writes == []


def test_ax_bus_lookup_finds_ax_not_hw():
    grp, ax_bus, hw_bus = build()
    assert grp._ax_bus() is ax_bus


def test_feetech_kinds_classify():
    assert SG.protocol_for("sts3215") == SG.PROTOCOL_FT
    assert SG.protocol_for("sts3235") == SG.PROTOCOL_FT


def build_shared():
    """AX and Feetech on ONE bus object (same wire), HiWonder on its own."""
    shared, hw_bus = FakeBus("shared"), FakeBus("hw")
    return SG.ServoGroup([SG.Servo(11, "ax12", shared),
                          SG.Servo(12, "sts3215", shared),
                          SG.Servo(13, "ax12", shared),
                          SG.Servo(42, "hx-35hm", hw_bus)]), shared, hw_bus


def test_ax_and_feetech_get_separate_sync_writes():
    # A sync_write carries ONE address for every servo in it, and the goal
    # position register differs (AX 30 vs Feetech 42), so even on a shared bus
    # the two families need one batch each -- never a combined packet.
    import ax as axreg, feetech as ft
    grp, shared, hw_bus = build_shared()
    grp.write_positions([500, 2048, 500, 871], move_ms=36)
    batches = {off: ids for ids, off, _ in
               [(s[0], s[1], s[2]) for s in shared.sync_writes_full]}
    assert batches[axreg.GOAL_POSITION] == [11, 13]
    assert batches[ft.Register.GOAL_POSITION] == [12]
    assert shared.writes == []          # nothing sent per-servo on that bus
    assert [d for d, _ in hw_bus.writes] == [42]


def test_feetech_speed_and_torque_addresses():
    import ax as axreg, feetech as ft
    grp, shared, _ = build_shared()
    grp.write_speed(300)
    offs = [off for _, off, _ in shared.sync_writes_full]
    assert axreg.MOVING_SPEED in offs and ft.Register.GOAL_SPEED in offs
    shared.sync_writes_full = []
    grp.write_torque(True)
    offs = [off for _, off, _ in shared.sync_writes_full]
    assert axreg.TORQUE_ENABLE in offs and ft.Register.TORQUE_ENABLE in offs


def test_feetech_angle_limits_bracketed_by_lock():
    # MIN_ANGLE_LIMIT(9)/MAX_ANGLE_LIMIT(11) are adjacent EEPROM registers, so
    # one 4-byte sync_write covers both -- but LOCK must be cleared first.
    import feetech as ft
    grp, shared, _ = build_shared()
    grp.write_angle_limits([[100, 900], [200, 3900], [100, 900], [1, 2]])
    ft_seq = [(off, vals) for ids, off, vals in shared.sync_writes_full
              if ids == [12]]
    assert [off for off, _ in ft_seq] == [ft.Register.LOCK,
                                          ft.Register.MIN_ANGLE_LIMIT,
                                          ft.Register.LOCK]
    assert ft_seq[0][1][0] == b"\x00"       # unlocked
    assert ft_seq[2][1][0] == b"\x01"       # relocked
    assert ft_seq[1][1][0] == b"\xc8\x00\x3c\x0f"   # 200, 3900 little-endian


def test_return_level_uses_same_value_for_ax_and_feetech():
    # Same encoding on both, so one call configures the shared bus coherently.
    import ax as axreg, feetech as ft
    grp, shared, _ = build_shared()
    grp.write_return_level(SG.RETURN_LEVEL_READ_ONLY)
    seq = [(ids, off, vals) for ids, off, vals in shared.sync_writes_full]
    ax_w = [(o, v) for i, o, v in seq if i == [11, 13]]
    ft_w = [(o, v) for i, o, v in seq if i == [12]]
    assert ax_w[0][0] == axreg.RETURN_LEVEL
    assert ax_w[0][1][0] == bytes([SG.RETURN_LEVEL_READ_ONLY])
    # Feetech register is EEPROM -> LOCK, write, LOCK
    assert [o for o, _ in ft_w] == [ft.Register.LOCK,
                                    ft.Register.RESPONSE_STATUS_LEVEL,
                                    ft.Register.LOCK]
    assert ft_w[1][1][0] == bytes([SG.RETURN_LEVEL_READ_ONLY])


def test_hiwonder_needs_no_return_level_write():
    # HiWonder has no such register; it is hard-wired to read-only replies, so
    # nothing is sent to it and the request is silently satisfied.
    grp, _, hw_bus = build_shared()
    grp.write_return_level(SG.RETURN_LEVEL_READ_ONLY)
    assert hw_bus.writes == [] and hw_bus.sync_writes == []


def test_hiwonder_warns_on_unachievable_return_level():
    # Asking for a level HiWonder cannot honor must not pass silently.
    import io, contextlib
    grp, _, hw_bus = build_shared()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        grp.write_return_level(SG.RETURN_LEVEL_ALL)
    out = buf.getvalue()
    assert "HiWonder" in out and "42" in out, out
    assert hw_bus.writes == []


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print("\nall %d tests passed" % len(tests))


if __name__ == "__main__":
    main()
