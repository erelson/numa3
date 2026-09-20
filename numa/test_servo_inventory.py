"""PC test for servo_inventory helpers (step 1 of the integration plan).

Plain-assert test, no framework. Run with:  python3 test_servo_inventory.py
Verifies the derivation helpers produce the intended joint->kind/trim maps and
that the two data tables are internally consistent.
"""
import servo_inventory as inv


def test_data_consistency():
    labels = [s["label"] for s in inv.PHYSICAL_SERVOS]
    assert len(labels) == len(set(labels)), "duplicate servo labels"
    assigned = list(inv.JOINT_ASSIGNMENTS.values())
    assert all(l in labels for l in assigned), "assignment references unknown label"
    assert len(assigned) == len(set(assigned)), "a servo is assigned to two joints"


def test_counts():
    by_kind = {}
    for s in inv.PHYSICAL_SERVOS:
        by_kind[s["kind"]] = by_kind.get(s["kind"], 0) + 1
    assert by_kind == {inv.KIND_AX12: 14, inv.KIND_HX35HM: 4}, by_kind
    spares = set(s["label"] for s in inv.PHYSICAL_SERVOS) - set(inv.JOINT_ASSIGNMENTS.values())
    assert spares == {"AX12", "AX22", "AX32", "AX42"}, spares


def test_joint_kind():
    # Femurs are HiWonder; everything else is AX-12.
    for femur in (12, 22, 32, 42):
        assert inv.joint_kind(femur) == inv.KIND_HX35HM, femur
    for other in (11, 21, 31, 41, 13, 23, 33, 43, 51, 52):
        assert inv.joint_kind(other) == inv.KIND_AX12, other


def test_joint_trim_resolves_through_tables():
    # Deliberately does NOT pin trim values: those are calibration data and are
    # expected to change. What matters is that the lookup resolves correctly
    # through JOINT_ASSIGNMENTS -> PHYSICAL_SERVOS, and that the values are
    # plausible degrees (a trim entered in counts would blow the range check).
    by_label = {s["label"]: s for s in inv.PHYSICAL_SERVOS}
    for joint_id, label in inv.JOINT_ASSIGNMENTS.items():
        trim = inv.joint_trim(joint_id)
        assert trim == by_label[label]["trim_deg"], (joint_id, label)
        assert isinstance(trim, float), (joint_id, trim)
        assert -180.0 < trim < 180.0, (joint_id, trim)


def test_joint_bus_id():
    for j in inv.JOINT_ASSIGNMENTS:
        assert inv.joint_bus_id(j) == j


def test_leg_servo_maps():
    types = inv.leg_servo_types()
    trims = inv.leg_servo_trims()
    expected_joints = {11, 21, 31, 41, 12, 22, 32, 42, 13, 23, 33, 43}
    assert set(types) == expected_joints, set(types)
    assert set(trims) == expected_joints, set(trims)
    assert 51 not in types and 52 not in types, "turret must be excluded"
    assert types[12] == inv.KIND_HX35HM and types[11] == inv.KIND_AX12
    # Values themselves are calibration data; just check the map agrees with
    # the per-joint accessors it is built from.
    for joint_id in expected_joints:
        assert trims[joint_id] == inv.joint_trim(joint_id), joint_id
        assert types[joint_id] == inv.joint_kind(joint_id), joint_id


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print("\nall %d tests passed" % len(tests))


if __name__ == "__main__":
    main()
