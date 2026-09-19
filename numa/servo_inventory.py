"""Physical servo inventory and joint assignments for Numa 3.

DATA ONLY - nothing imports this yet. It is the intended future source of
truth for which physical servo (and therefore which servo *type*) sits at
each joint, so that a swap is a one-line edit here rather than scattered
changes across numa.py / poses.py / init.py.

Two independent tables:

  PHYSICAL_SERVOS   the servos that physically exist (owned hardware),
                    keyed by a stable label that never changes even when a
                    unit is moved between joints. Carries the servo kind and
                    a per-unit trim (mounting/horn offset, degrees) that
                    belongs to the physical unit, not the joint.

  JOINT_ASSIGNMENTS which physical servo is currently installed at each
                    joint. Joint ids follow the leg/turret numbering used in
                    numa.py: <leg><joint>, joints 1=coax 2=femur 3=tibia,
                    plus turret 51=pan 52=tilt. A servo's operating bus id,
                    when installed, equals its joint id.

Initial configuration (2026-09):
  - 14x AX-12 and 4x HiWonder HX-35HM owned.
  - HiWonders assigned to the four femur joints: 12, 22, 32, 42.
  - AX-12 assigned to the 4 coax, 4 tibia, and 2 turret joints (10 joints).
  - That leaves 4 AX-12 unassigned (the former femur units AX12/AX22/AX32/AX42,
    marked "unused"), available as spares.

Buses (for reference; derivable from kind): AX-12 on the Dynamixel bus
(UART2), HiWonder on the HX bus (UART4).
"""

# Servo kind strings match poses.LegGeom.pos_lookup / center_lookup keys.
KIND_AX12 = "ax12"
KIND_HX35HM = "hx-35hm"


# Owned physical servos. label is stable; id-when-installed comes from the
# joint assignment below. trim_deg is a per-unit offset, 0.0 until measured.
PHYSICAL_SERVOS = [
    # 14x AX-12
    {"label": "AX11", "kind": KIND_AX12, "trim_deg": 0.0, "notes": ""},
    {"label": "AX12", "kind": KIND_AX12, "trim_deg": 0.0, "notes": "unused"},
    {"label": "AX13", "kind": KIND_AX12, "trim_deg": 0.0, "notes": ""},
    {"label": "AX21", "kind": KIND_AX12, "trim_deg": 0.0, "notes": ""},
    {"label": "AX22", "kind": KIND_AX12, "trim_deg": 0.0, "notes": "unused"},
    {"label": "AX23", "kind": KIND_AX12, "trim_deg": 0.0, "notes": ""},
    {"label": "AX31", "kind": KIND_AX12, "trim_deg": 0.0, "notes": ""},
    {"label": "AX32", "kind": KIND_AX12, "trim_deg": 0.0, "notes": "unused"},
    {"label": "AX33", "kind": KIND_AX12, "trim_deg": 0.0, "notes": ""},
    {"label": "AX41", "kind": KIND_AX12, "trim_deg": 0.0, "notes": ""},
    {"label": "AX42", "kind": KIND_AX12, "trim_deg": 0.0, "notes": "unused"},
    {"label": "AX43", "kind": KIND_AX12, "trim_deg": 0.0, "notes": ""},
    {"label": "AX51", "kind": KIND_AX12, "trim_deg": 0.0, "notes": "turret"},
    {"label": "AX52", "kind": KIND_AX12, "trim_deg": 0.0, "notes": "turret"},
    # 4x HiWonder HX-35HM
    {"label": "HW12", "kind": KIND_HX35HM, "trim_deg": 0.0, # TODO
     "notes": "id 12"},
    {"label": "HW22", "kind": KIND_HX35HM, "trim_deg": 0.0, # TODO
     "notes": "id 22"},
    {"label": "HW32", "kind": KIND_HX35HM, "trim_deg": 0.0, # TODO
     "notes": "id 32"},
    {"label": "HW42", "kind": KIND_HX35HM, "trim_deg": -95.0,  # Special case, servo horn wasn't centered properly I think; 9-16-2026...:
     "notes": "id 42"},
]


# joint id -> physical servo label. Unassigned joints or spare servos simply
# do not appear. Bus id of the installed servo == the joint id.
JOINT_ASSIGNMENTS = {
    # Coax (joint 1): AX-12
    11: "AX11",
    21: "AX21",
    31: "AX31",
    41: "AX41",
    # Femur (joint 2): HiWonder
    12: "HW12",
    22: "HW22",
    32: "HW32",
    42: "HW42",
    # Tibia (joint 3): AX-12
    13: "AX13",
    23: "AX23",
    33: "AX33",
    43: "AX43",
    # Turret: AX-12
    51: "AX51",  # pan
    52: "AX52",  # tilt
}


# ---------------------------------------------------------------------------
# Derivation helpers (pure; safe to import on PC and PyBoard).
# Nothing in the robot code consumes these yet - see the integration plan.
# ---------------------------------------------------------------------------

# label -> physical servo record, for O(1) lookup.
_BY_LABEL = {s["label"]: s for s in PHYSICAL_SERVOS}


# NOTE: joint_id as used below is an integer servo ID

def _servo_at(joint_id):
    """Return the physical servo record installed at joint_id (raises KeyError
    if the joint is unassigned or the label is unknown)."""
    return _BY_LABEL[JOINT_ASSIGNMENTS[joint_id]]


def joint_kind(joint_id):
    """Servo kind ('ax12' / 'hx-35hm') installed at joint_id."""
    return _servo_at(joint_id)["kind"]


def joint_trim(joint_id):
    """Per-unit trim (signed degrees) of the servo installed at joint_id."""
    return _servo_at(joint_id)["trim_deg"]


def joint_bus_id(joint_id):
    """Bus id the installed servo answers to. Equal to the joint id today."""
    return joint_id


def _is_leg_joint(joint_id):
    # Leg joints have a leg number 1-4 in the tens place; turret is 5x.
    return 1 <= joint_id // 10 <= 4


def leg_servo_types():
    """{joint_id: kind} for the 12 leg joints (11-43), turret excluded."""
    return {j: joint_kind(j) for j in JOINT_ASSIGNMENTS if _is_leg_joint(j)}


def leg_servo_trims():
    """{joint_id: trim_deg} for the 12 leg joints (11-43), turret excluded."""
    return {j: joint_trim(j) for j in JOINT_ASSIGNMENTS if _is_leg_joint(j)}
