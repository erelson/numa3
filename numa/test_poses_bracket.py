"""PC test for BRACKET_GEOM per-(role, type) geometry.

Run with:  python3 test_poses_bracket.py

Locks the AX-12 bracket values (the historical constants) and the measured
HiWonder femur bracket, plus the usable joint-angle range those values imply.

The range check is the important one: min/max are in the SERVO frame, but a
joint angle carries an extra factor of `jointsign`, so for the femur
(jointsign = -1) they map to the OPPOSITE ends in joint-angle space:

    jointsign = -1:  a ranges over [aoffset - max, aoffset - min]
    jointsign = +1:  a ranges over [min - aoffset, max - aoffset]

`min` therefore bounds how far the femur can FOLD and `max` how far it can
EXTEND -- the reverse of what the names suggest. This test pins that mapping so
a future edit cannot silently swap them.
"""
import poses


def usable_angle_range(role, kind):
    """Joint-angle range a joint can be commanded over, from its bracket."""
    b = poses.bracket_geom(role, kind)
    if b["jointsign"] < 0:
        return (b["aoffset"] - b["max"], b["aoffset"] - b["min"])
    return (b["min"] - b["aoffset"], b["max"] - b["aoffset"])


def test_ax_bracket_values():
    coax = poses.bracket_geom("coax", "ax12")
    femur = poses.bracket_geom("femur", "ax12")
    tibia = poses.bracket_geom("tibia", "ax12")
    assert (coax["aoffset"], coax["min"], coax["max"]) == (45.0, -10, 95)
    assert (femur["aoffset"], femur["min"], femur["max"], femur["jointsign"]) == \
        (31.54, -68, 100, -1)
    assert (tibia["aoffset"], tibia["min"], tibia["max"], tibia["jointsign"]) == \
        (31.54 - 5.63, -140, 10, 1)


def test_ax12a_shares_ax12_bracket():
    assert poses.bracket_geom("femur", "ax12a") is poses.bracket_geom("femur", "ax12")


def test_hw_femur_bracket_values():
    # Measured 2026-09 on the HiWonder femur bracket:
    #   aoffset 0.0  -> the servo sits at its electrical center at joint angle 0
    #   min -90      -> bench-measured mechanical stop at a2 = +90 (max fold)
    #   max 75       -> extension end; NOT yet measured, and unused because no
    #                   pose commands a negative a2
    hw = poses.bracket_geom("femur", "hx-35hm")
    assert (hw["aoffset"], hw["min"], hw["max"], hw["jointsign"]) == \
        (0.0, -90, 75, -1), hw


def test_hw_femur_usable_range_covers_poses():
    # min bounds the FOLD end, max the EXTEND end (see module docstring).
    lo, hi = usable_angle_range("femur", "hx-35hm")
    assert (lo, hi) == (-75.0, 90.0), (lo, hi)
    # Everything the robot actually commands must fit, with the fold end being
    # the tight one: g8Stand/g8FeetDown use a2=45, g8Crouch a2=85 (kept short of
    # the 90 deg mechanical stop), and the walk cycle peaks near 78 deg.
    for a2 in (45, 85, 78):
        assert lo <= a2 <= hi, (a2, lo, hi)


def test_ax_femur_usable_range():
    # Same inversion applies to the AX femur; guards the shared formula.
    assert usable_angle_range("femur", "ax12") == (31.54 - 100, 31.54 + 68)
    # Tibia has jointsign +1, so its min/max map the direct way.
    lo, hi = usable_angle_range("tibia", "ax12")
    assert (round(lo, 2), round(hi, 2)) == (-165.91, -15.91), (lo, hi)
    assert lo <= -155 <= hi and lo <= -125 <= hi  # g8Crouch / g8Stand a3


def test_legdef_uses_bracket_direction():
    # femur/tibia joint signs come from the bracket, not a shared LegGeom field.
    _, l1, _, _, _ = poses.gen_numa2_legs()
    assert l1.joint2sign == poses.bracket_geom("femur", "ax12")["jointsign"]
    assert l1.joint3sign == poses.bracket_geom("tibia", "ax12")["jointsign"]


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print("\nall %d tests passed" % len(tests))


if __name__ == "__main__":
    main()
