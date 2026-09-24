import sys
from math import pi
sysname = sys.platform
if sysname == 'linux' or sysname == 'win32':
    # Mocks for non-pyboard use
    import time
    def sleep_ms(x): return time.sleep(x/1e3)
elif sysname == 'pyboard':
    from utime import sleep_ms


RAD_TO_ANGLE = 180./pi

# Servo center positions (counts), the single source of truth for "straight".
# AX-12/AX-12A: 1023-count range -> center 512 (~511.5). HiWonder HX-35HM:
# 1500-count range -> center 750. Imported by init.py/numa.py for the turret.
AX_CENTER = 512
HX35HM_CENTER = 750
# Feetech STS3215 / STS3235: 12-bit magnetic encoder, 0..4095 over 360 deg,
# neutral 2048 = 180 deg (datasheet 7-6..7-8). Same for both models.
STS_CENTER = 2048

# Per-(role, servo type) bracket geometry. A joint's resting angle offset (deg),
# travel limits (deg from the servo's electrical center, sign per s*_sign), and
# motion direction depend on the mounting bracket, which changes with the servo
# type. AX-12 entries are the values Numa2/3 has used to date. The HiWonder femur
# entry is a placeholder equal to the AX femur until the real bracket is measured
# (integration plan step 6); it is unused while the femurs are forced to AX.
ROLE_COAX, ROLE_FEMUR, ROLE_TIBIA = "coax", "femur", "tibia"
_JOINT_ROLE = {1: ROLE_COAX, 2: ROLE_FEMUR, 3: ROLE_TIBIA}

BRACKET_GEOM = {
    (ROLE_COAX,  "ax12"):    {"aoffset": 45.0,         "min": -10,  "max": 95,  "jointsign": 1},
    (ROLE_FEMUR, "ax12"):    {"aoffset": 31.54,        "min": -68,  "max": 100, "jointsign": -1},
    (ROLE_TIBIA, "ax12"):    {"aoffset": 31.54 - 5.63, "min": -140, "max": 10,  "jointsign": 1},  # off_b - off_h
    (ROLE_FEMUR, "hx-35hm"): {"aoffset": 0.0,        "min": -90,  "max": 75, "jointsign": -1},  # partiall tested at least
}


def bracket_geom(role, kind):
    """Bracket geometry dict for a joint role and servo kind.
    'ax12a' shares the 'ax12' bracket."""
    if kind == "ax12a":
        kind = "ax12"
    try:
        return BRACKET_GEOM[(role, kind)]
    except KeyError:
        # A servo kind can be registered in pos_lookup/center_lookup (so the
        # protocol and scale are known) while its bracket has not been measured
        # yet -- e.g. the Feetech STS parts. Say so instead of raising a bare
        # KeyError from deep inside LegDef.
        raise KeyError(
            "no bracket geometry for role {!r} with servo kind {!r}; measure "
            "the aoffset/min/max/jointsign for that mounting and add it to "
            "poses.BRACKET_GEOM".format(role, kind))

# NOTE: All g8 pose functions should return a wait time in ms

# Send neutral standing positions to all servos.
def g8Stand(gait, leg_servos):
    a2 = 45
    a3 = -125

    gait.s11pos, gait.s12pos, gait.s13pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3)
    gait.s21pos, gait.s22pos, gait.s23pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3)
    gait.s31pos, gait.s32pos, gait.s33pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3)
    gait.s41pos, gait.s42pos, gait.s43pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3)

    leg_servos.write_positions(
               (gait.s11pos, gait.s21pos, gait.s31pos, gait.s41pos,
                gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos))
    return 1000

# Send standing positions to all servos. BUT don't rotate legs to center position
def g8FeetDown(gait, leg_servos):
    a2 = 45
    a3 = -125
    _, gait.s12pos, gait.s13pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3)
    _, gait.s22pos, gait.s23pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3)
    _, gait.s32pos, gait.s33pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3)
    _, gait.s42pos, gait.s43pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3)
    # Don't send positions to coax servos (indices 0-3)
    leg_servos.write_positions(
               (gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos),
               indices=range(4, 12))
    return 1000

# Send standing positions to all servos.
def g8Flop(gait, leg_servos):
    # TODO unused; TODO define updated angles
    a2 = 45
    a3 = -125
    # TODO I didn't change these yet
    gait.s11pos, gait.s12pos, gait.s13pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3)
    gait.s21pos, gait.s22pos, gait.s23pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3)
    gait.s31pos, gait.s32pos, gait.s33pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3)
    gait.s41pos, gait.s42pos, gait.s43pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3)

    leg_servos.write_positions(
               (gait.s11pos, gait.s21pos, gait.s31pos, gait.s41pos,
                gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos))
    return 1000

# Lower feet to ground regardless of shoulder servo position, then cut torque to prevent overheating
def g8Crouch(gait, leg_servos):
    # angles are leg angles
    a2 = 85  # changed for HW servos; used to be 90 with AX servos
    a3 = -155
    # Don't send positions to coax servos (indices 0-3)
    _, gait.s12pos, gait.s13pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3)
    _, gait.s22pos, gait.s23pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3)
    _, gait.s32pos, gait.s33pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3)
    _, gait.s42pos, gait.s43pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3)

    leg_servos.write_positions(
               (gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos),
               indices=range(4, 12))

    # Let the servos move, then disable torque to femur servos (indices 4-7)
    sleep_ms(400)
    leg_servos.write_torque(False, indices=range(4, 8))

    return 1000

#  a: default leg position (fixed)
#  b: walk vector (from controller)
#  c: trav offset vector (dynamic for each leg)
# Add a + c to get leg vector and determine leg length
#
#        c      _ b   
#       /       /|    
#      a\      /    / 
#        \    /    /  
#         \ _____ /   
#          |     |    
#          |numa |    counter clockwise is positive direction
#          |_____|    
#         /       \   
#        /         \  
#       /           \ 

#                          //\3                           
#          3-4-2 = alph2  // \\            (3)            
#          3-2-4 = alph3 //   \\                          
#                        4\__  \\__     4                 
#                            \__\==      \                
#               3               2      ___\2              
#              /\                     H  alph1            
#             /  \                                        
#            /    \          alph2 +/- alph1 = 3-4-horiz  
#           /      \                                      
#          /        \                                     
#         /          \                                    
#        /            \                                   
#     4 /              \                                  
#      |                \        |                        
#      |                 \_______|1        ___            
#      |                  2      |_______    |            
#      |                                     |- bodyH     
#      |5      TODO mirror this drawing    __|            
#
#
#      |________legLen__aka L0___|

def gen_numa2_legs(leg_servo_types=None, leg_servo_trims=None):
    """Generate leg geometry and leg definitions for Numa 2/3.

    leg_servo_types: optional flat dict {joint_id: kind}, joint_id being the
        <leg><joint> id used elsewhere (e.g. 12 = leg 1 femur). kind is a
        pos/center_lookup key such as 'ax12' or 'hx-35hm'. Unlisted joints
        default to 'ax12'.
    leg_servo_trims: optional flat dict {joint_id: trim_deg} (signed degrees),
        the per-unit mounting/horn offset. Unlisted joints default to 0.0.

    These map directly onto servo_inventory.leg_servo_types() /
    leg_servo_trims().
    """
    if leg_servo_types is None:
        leg_servo_types = {}
    if leg_servo_trims is None:
        leg_servo_trims = {}

    def _leg_overrides(leg_num):
        # Translate the flat {joint_id: ...} maps into LegDef's per-leg dict of
        # 'servoX_type'/'servoX_trim' keys for X in 1..3.
        d = {}
        for joint in (1, 2, 3):
            jid = leg_num * 10 + joint
            if jid in leg_servo_types:
                d["servo{0}_type".format(joint)] = leg_servo_types[jid]
            if jid in leg_servo_trims:
                d["servo{0}_trim".format(joint)] = leg_servo_trims[jid]
        return d
# 4\ __^__ /3
#   |     |
#   |numa2|
#   |_____|
# 1/       \2
    stance = 5  # degrees; see README
    # Shared geometry (not per servo type). Per-(role, type) aoffset, travel
    # limits, and joint direction live in module-level BRACKET_GEOM instead.
    offsets_dict = {
            "a1stance": stance,        # degrees
            "a1stance_rear": -10,      # degrees
            "L0": 130, # mm; aka legLen
            "L12": 58,
            "L23": 61, # 61 is with HW/AX servo combo on Numa3; #65, #63,
            "L34": 130, #67,
            "L45": 5,  # This isn't used in numa2's case
            }
    leg_model = LegGeom(offsets_dict)

    # Per servo joint signs? what about "legsign"?
    # Given how I alternated the below due to how I assembled the legs in the past,
    # I could possibly have defined a single "legsign" per leg, and adjusted the sign of the second angle elsewhere.
    # leg_geom, s1_sign, s2_sign, s3_sign, front_leg=True):
    leg1 = LegDef(leg_model, _leg_overrides(1),  1, -1,  1)
    leg2 = LegDef(leg_model, _leg_overrides(2), -1,  1, -1)
    leg3 = LegDef(leg_model, _leg_overrides(3),  1, -1,  1, front_leg=True)
    leg4 = LegDef(leg_model, _leg_overrides(4), -1,  1, -1, front_leg=True)

    return leg_model, leg1, leg2, leg3, leg4


class LegGeom(object):
    """

    Conventions:
    - See ascii drawings in IK.py
    - Can support 3 or 4 servos per leg
    - Can optionally specify servo type. Default is AX-12A

    Each servo has an offset defined by

    """

    def __init__(self, offsets_dict):

        self.L0 =  offsets_dict.pop("L0")
        self.L12 = offsets_dict.pop("L12")
        self.L23 = offsets_dict.pop("L23")
        self.L34 = offsets_dict.pop("L34")
        self.L45 = offsets_dict.pop("L45")

        # Stance is offset from default 45 degree leg direction. Positive stance puts
        # forward legs more forward and rear legs more rearward
        self.a1stance = offsets_dict.pop("a1stance")
        # Optional rear stance lets front legs and back legs have separate stance angle
        self.a1stance_rear = offsets_dict.pop("a1stance_rear", self.a1stance)

        # Per-joint resting offset (aoffset), travel limits, and joint direction
        # are per (role, servo type) -- see module-level BRACKET_GEOM, consulted
        # by LegDef.

        self.pos_lookup = {"ax12": self.ax12pos,
                           "ax12a": self.ax12pos,
                           "hx-35hm": self.hx35hmpos,
                           "sts3215": self.stspos,
                           "sts3235": self.stspos,
        }
        # Simple center values (position) for servo types
        self.center_lookup = {"ax12": AX_CENTER,
                              "ax12a": AX_CENTER,
                              "hx-35hm": HX35HM_CENTER,
                              "sts3215": STS_CENTER,
                              "sts3235": STS_CENTER,
        }

    def ax12pos(self, angle):
        """Return an angle converted from degrees into integer position values for the servo
        
        Note: Generally combined with an offset representing the servo's center position
        """
        return int(angle/150.0 * 512)  # Degrees -> servo position

    def hx35hmpos(self, angle):
        """Return an angle converted from degrees into integer position values for the servo

        Hiwonder HX-35HM servos.

        Roughly 360 degree range of motion over 1500 position values
        
        Note: Generally combined with an offset representing the servo's center position
        """
        return int(angle/180.0 * 750)  # Degrees -> servo position (360 deg = 1500 positions)

    def stspos(self, angle):
        """Return an angle converted from degrees into integer position values.

        Feetech STS3215 / STS3235: 12-bit magnetic encoder, 0..4095 spanning
        360 degrees, so 2048 counts per 180 degrees (0.088 deg/count).

        Note: Generally combined with an offset representing the servo's center position
        """
        return int(angle/180.0 * 2048)  # Degrees -> servo position (360 deg = 4096 positions)


class LegDef(object):
    """A representation of the servos in a leg that can be generically given
    a set of joint angles and will generate corresponding servo positions,
    accounting for orientation, etc.
    """

    def __init__(self, leg_geom, offsets_dict, s1_sign, s2_sign, s3_sign, front_leg=True):
        """
        offsets_dict : dict
            Per-joint config. Optional keys 'servoX_type' (default 'ax12') and
            'servoX_trim' (signed degrees, default 0.0), for X in 1..3.
        """
        self.leg_geom = leg_geom
        self.s1_sign = s1_sign
        self.s2_sign = s2_sign
        self.s3_sign = s3_sign

        # degrees
        a1_stance_offset = self.leg_geom.a1stance if front_leg else self.leg_geom.a1stance_rear

        # Set per-joint position functions and center values based on servo type
        _t1 = offsets_dict.pop("servo1_type", "ax12")
        _t2 = offsets_dict.pop("servo2_type", "ax12")
        _t3 = offsets_dict.pop("servo3_type", "ax12")
        self.servo_types = [_t1, _t2, _t3]
        self.pos1 = leg_geom.pos_lookup[_t1]
        self.pos2 = leg_geom.pos_lookup[_t2]
        self.pos3 = leg_geom.pos_lookup[_t3]
        # Servo center positions
        c1 = leg_geom.center_lookup[_t1]
        c2 = leg_geom.center_lookup[_t2]
        c3 = leg_geom.center_lookup[_t3]

        # Per-unit trim (signed degrees): a mounting/horn offset in the servo's
        # own frame, applied as an effective-center shift (NOT through the leg or
        # joint sign). Folding it into c* means BOTH the joint zero (s*_center)
        # and the range-of-motion limits below move with it. Default 0.0 -> no
        # change.
        _tr1 = offsets_dict.pop("servo1_trim", 0.0)
        _tr2 = offsets_dict.pop("servo2_trim", 0.0)
        _tr3 = offsets_dict.pop("servo3_trim", 0.0)
        self.servo_trims = [_tr1, _tr2, _tr3]
        c1 = c1 + self.pos1(_tr1)
        c2 = c2 + self.pos2(_tr2)
        c3 = c3 + self.pos3(_tr3)

        # Bracket geometry per (joint role, servo type): resting aoffset, travel
        # limits, and motion direction. Coax has no separate joint sign (its
        # direction is carried by s1_sign), so b1["jointsign"] is unused.
        b1 = bracket_geom(ROLE_COAX, _t1)
        b2 = bracket_geom(ROLE_FEMUR, _t2)
        b3 = bracket_geom(ROLE_TIBIA, _t3)
        self.joint2sign = b2["jointsign"]
        self.joint3sign = b3["jointsign"]

        # Servo range-of-motion limits
        s1lims = [c1 + self.s1_sign * self.pos1(b1["max"]), c1 + self.s1_sign * self.pos1(b1["min"])]
        s2lims = [c2 + self.s2_sign * self.pos2(b2["max"]), c2 + self.s2_sign * self.pos2(b2["min"])]
        s3lims = [c3 + self.s3_sign * self.pos3(b3["max"]), c3 + self.s3_sign * self.pos3(b3["min"])]
        s1lims.sort()
        s2lims.sort()
        s3lims.sort()
        self.s1min, self.s1max = s1lims
        self.s2min, self.s2max = s2lims
        self.s3min, self.s3max = s3lims
        self.s1min = self.s1min if self.s1min >= 0 else 0
        self.s2min = self.s2min if self.s2min >= 0 else 0
        self.s3min = self.s3min if self.s3min >= 0 else 0

        # Convert offsets in degrees to servo values
        self.s1_center_angle = self.s1_sign * (b1["aoffset"] + a1_stance_offset)
        self.s1_center_radians = self.s1_center_angle / RAD_TO_ANGLE
        self.s1_center = c1 + self.pos1(self.s1_center_angle)
        self.s2_center = c2 + self.pos2(self.s2_sign * b2["aoffset"])
        self.s3_center = c3 + self.pos3(self.s3_sign * b3["aoffset"])

    def get_pos_from_angle(self, a1, a2, a3):
        # Angles are in degrees. Returns list of servo positions
        return [
                self.s1_center + self.pos1(a1),  # Remember to supply offset from center, not absolute angle
                self.s2_center + self.pos2(self.s2_sign * self.joint2sign * a2),
                self.s3_center + self.pos3(self.s3_sign * self.joint3sign * a3),
        ]

    def get_pos_from_radians(self, a1, a2, a3):
        # Current convention is we convert all angles from radians to degrees
        return [
                self.s1_center + self.pos1(RAD_TO_ANGLE * a1),
                self.s2_center + self.pos2(RAD_TO_ANGLE * self.s2_sign * self.joint2sign * a2),
                self.s3_center + self.pos3(RAD_TO_ANGLE * self.s3_sign * self.joint3sign * a3),
        ]
