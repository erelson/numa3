"""PC test for LegDef trim_deg support (step 3 of the integration plan).

Run with:  python3 test_poses_trim.py

Checks (1) no trim leaves everything unchanged, and (2) a trim shifts the
joint zero AND both travel limits by pos(trim) counts, in the servo frame
(independent of the leg/joint sign), while leaving the commanded delta per
degree unchanged.
"""
import poses


def _leg4(types, trims=None):
    # leg4 femur (joint id 42) is HiWonder in the real config.
    _, _, _, _, l4 = poses.gen_numa2_legs(types, trims)
    return l4


def test_default_trim_zero():
    l4 = _leg4({42: "hx-35hm"})
    assert l4.servo_trims == [0.0, 0.0, 0.0], l4.servo_trims


def test_trim_shifts_center_and_limits_equally():
    # Uses HW42's real trim: big enough to be a meaningful shift, but small
    # enough that the trimmed lower limit stays >= 0 (see the clamp test below,
    # which is why a large negative trim is deliberately not used here).
    trim_deg = 29.04
    base = _leg4({42: "hx-35hm"})
    trimmed = _leg4({42: "hx-35hm"}, {42: trim_deg})
    expected = int(trim_deg / 180.0 * 750)  # HiWonder scale
    assert expected == 121, expected
    assert trimmed.s2_center - base.s2_center == expected
    assert trimmed.s2min - base.s2min == expected
    assert trimmed.s2max - base.s2max == expected
    assert trimmed.s2min > 0 and base.s2min > 0  # no clamping in play
    # Other joints untouched
    assert trimmed.s1_center == base.s1_center
    assert trimmed.s3_center == base.s3_center


def test_large_negative_trim_clamps_lower_limit():
    """A trim large enough to push the lower limit below 0 gets clamped.

    LegDef pins s*min at 0 (a servo position cannot be negative), so for such a
    trim the center and upper limit still shift by pos(trim) but the lower limit
    does not. Documented so the asymmetry is known behavior, not a surprise.
    """
    base = _leg4({42: "hx-35hm"})
    trimmed = _leg4({42: "hx-35hm"}, {42: -95.0})
    expected = int(-95.0 / 180.0 * 750)
    assert expected == -395, expected
    assert trimmed.s2_center - base.s2_center == expected
    assert trimmed.s2max - base.s2max == expected
    # Lower limit would have gone negative, so it is clamped to 0 instead.
    assert trimmed.s2min == 0
    assert trimmed.s2min - base.s2min > expected


def test_trim_not_through_sign():
    # Same trim on femurs whose sign differs (leg1 s2_sign=-1, leg2 s2_sign=+1)
    # still shifts the center the same direction (servo frame).
    types = {12: "hx-35hm", 22: "hx-35hm"}
    _, l1t, l2t, _, _ = poses.gen_numa2_legs(types, {12: 10.0, 22: 10.0})
    _, l1b, l2b, _, _ = poses.gen_numa2_legs(types)
    shift = int(10.0 / 180.0 * 750)
    assert l1t.s2_center - l1b.s2_center == shift
    assert l2t.s2_center - l2b.s2_center == shift  # same sign despite mirror


def test_trim_shifts_commanded_positions_uniformly():
    base = _leg4({42: "hx-35hm"})
    trimmed = _leg4({42: "hx-35hm"}, {42: -95.0})
    shift = int(-95.0 / 180.0 * 750)
    for a2 in (-30, 0, 45, 90):
        db = base.get_pos_from_angle(0, a2, 0)[1]
        dt = trimmed.get_pos_from_angle(0, a2, 0)[1]
        assert dt - db == shift, (a2, dt - db)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print("\nall %d tests passed" % len(tests))


if __name__ == "__main__":
    main()
