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


def test_joint_trim():
    assert inv.joint_trim(42) == -95.0
    for j in (12, 22, 32):
        assert inv.joint_trim(j) == 0.0, j
    assert inv.joint_trim(11) == 0.0


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
    assert trims[42] == -95.0 and trims[13] == 0.0


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print("\nall %d tests passed" % len(tests))


if __name__ == "__main__":
    main()
