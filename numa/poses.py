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

# NOTE: All g8 pose functions should return a wait time in ms

# Send neutral standing positions to all servos.
def g8Stand(gait, leg_servos):
    a2 = 45
    a3 = -125
    a4 = 0

    gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3, a4)
    gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3, a4)
    gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3, a4)
    gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3, a4)

    leg_servos.write_positions(
               (gait.s11pos, gait.s21pos, gait.s31pos, gait.s41pos,
                gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos))
    return 1000

# Send standing positions to all servos. BUT don't rotate legs to center position
def g8FeetDown(gait, leg_servos):
    a2 = 45
    a3 = -125
    a4 = 0
    _, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3, a4)
    # Don't send positions to coax servos (indices 0-3)
    leg_servos.write_positions(
               (gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos),
               indices=range(4, 16))
    return 1000

# Send standing positions to all servos.
def g8Flop(gait, leg_servos):
    # TODO unused; TODO define updated angles
    a2 = 45
    a3 = -125
    a4 = 0
    # TODO I didn't change these yet
    gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3, a4)
    gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3, a4)
    gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3, a4)
    gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3, a4)

    leg_servos.write_positions(
               (gait.s11pos, gait.s21pos, gait.s31pos, gait.s41pos,
                gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos))
    return 1000

# Lower feet to ground regardless of shoulder servo position, then cut torque to prevent overheating
def g8Crouch(gait, leg_servos):
    # angles are leg angles
    a2 = 90
    a3 = -155
    a4 = 0
    # Don't send positions to coax servos (indices 0-3)
    _, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3, a4)

    leg_servos.write_positions(
               (gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos),
               indices=range(4, 16))

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

def gen_numa2_legs(leg_servo_types=None):
    """Generate leg geometry and leg definitions for Numa 2/3.

    leg_servo_types: optional dict mapping leg number (1-4) to a dict of
        servo type overrides, e.g.:
        {1: {'servo1_type': 'hx-35hm'}, 2: {'servo1_type': 'hx-35hm'}}
        Any unspecified joints default to 'ax12'.
    """
    if leg_servo_types is None:
        leg_servo_types = {}
# 4\ __^__ /3
#   |     |
#   |numa2|
#   |_____|
# 1/       \2
    stance = 5  # degrees; see README
    offsets_dict = {
            # Offsets are in degrees
            "aoffset1": 45.0,  # this one is special
            "aoffset2": 31.54,
            "aoffset3": 31.54 - 5.63, # off_b - off_h
            "a1stance": stance,
            "a1stance_rear": -10,  # degrees
            "L0": 130, # mm; aka legLen
            "L12": 58,
            "L23": 65, #63,
            "L34": 130, #67,
            "L45": 5,  # This isn't used in numa2's case
            # mins/max are in degrees from actual servo center (not joint center!)
            "max1": 95,
            "min1": -10,
            "max2": 100,
            "min2": -68,
            "max3": 10, #90,
            "min3": -140, #-20,
            #
            "joint2sign": -1,
            "joint3sign": 1,
            }
    leg_model = LegGeom(offsets_dict)

    # leg_geom, s1_sign, s2_sign, s3_sign, s4_sign=None, front_leg=True):
    leg1 = LegDef(leg_model, dict(leg_servo_types.get(1, {})),  1, -1,  1)
    leg2 = LegDef(leg_model, dict(leg_servo_types.get(2, {})), -1,  1, -1)
    leg3 = LegDef(leg_model, dict(leg_servo_types.get(3, {})),  1, -1,  1, front_leg=True)
    leg4 = LegDef(leg_model, dict(leg_servo_types.get(4, {})), -1,  1, -1, front_leg=True)

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

        # Offsets are specified in degrees
        self.aoffset1 = offsets_dict.pop("aoffset1")
        self.aoffset2 = offsets_dict.pop("aoffset2")
        self.aoffset3 = offsets_dict.pop("aoffset3")
        self.aoffset4 = offsets_dict.pop("aoffset4", 0)

        # +1 if servo is on non-moving side of joint, -1 if servo is on moving side.
        self.joint2sign = offsets_dict.pop("joint2sign", 1)
        self.joint3sign = offsets_dict.pop("joint3sign", 1)
        self.joint4sign = offsets_dict.pop("joint4sign", 1)

        # Stance is offset from default 45 degree leg direction. Positive stance puts
        # forward legs more forward and rear legs more rearward
        self.a1stance = offsets_dict.pop("a1stance")
        # Optional rear stance lets front legs and back legs have separate stance angle
        self.a1stance_rear = offsets_dict.pop("a1stance_rear", self.a1stance)

        # Joint max/min angles
        # TODO(enhancement): genericize for other servo types
        self.max_angle = {}
        self.min_angle = {}
        for n in range(1,5):
            self.max_angle[n] = offsets_dict.pop("max{0}".format(n), 150)
            self.min_angle[n] = offsets_dict.pop("min{0}".format(n), -150)

        self.pos_lookup = {"ax12": self.ax12pos,
                           "ax12a": self.ax12pos,
                           "hx-35hm": self.hx35hmpos,
        }
        self.center_lookup = {"ax12": 512,
                              "ax12a": 512,
                              "hx-35hm": 750,
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


class LegDef(object):
    """A representation of the servos in a leg that can be generically given
    a set of joint angles and will generate corresponding servo positions,
    accounting for orientation, etc.
    """

    def __init__(self, leg_geom, offsets_dict, s1_sign, s2_sign, s3_sign, s4_sign=1, front_leg=True):
        """
        offsets_dict : dict
            Dictionary with keys 'servoX_type'.
        """
        self.leg_geom = leg_geom
        self.s1_sign = s1_sign
        self.s2_sign = s2_sign
        self.s3_sign = s3_sign
        self.s4_sign = s4_sign

        # degrees
        a1_stance_offset = self.leg_geom.a1stance if front_leg else self.leg_geom.a1stance_rear

        # Set per-joint position functions and center values based on servo type
        _t1 = offsets_dict.pop("servo1_type", "ax12")
        _t2 = offsets_dict.pop("servo2_type", "ax12")
        _t3 = offsets_dict.pop("servo3_type", "ax12")
        _t4 = offsets_dict.pop("servo4_type", "ax12")
        self.servo_types = [_t1, _t2, _t3, _t4]
        self.pos1 = leg_geom.pos_lookup[_t1]
        self.pos2 = leg_geom.pos_lookup[_t2]
        self.pos3 = leg_geom.pos_lookup[_t3]
        self.pos4 = leg_geom.pos_lookup[_t4]
        c1 = leg_geom.center_lookup[_t1]
        c2 = leg_geom.center_lookup[_t2]
        c3 = leg_geom.center_lookup[_t3]
        c4 = leg_geom.center_lookup[_t4]

        s1lims = [c1 + self.s1_sign * self.pos1(leg_geom.max_angle[1]), c1 + self.s1_sign * self.pos1(leg_geom.min_angle[1])]
        s2lims = [c2 + self.s2_sign * self.pos2(leg_geom.max_angle[2]), c2 + self.s2_sign * self.pos2(leg_geom.min_angle[2])]
        s3lims = [c3 + self.s3_sign * self.pos3(leg_geom.max_angle[3]), c3 + self.s3_sign * self.pos3(leg_geom.min_angle[3])]
        s4lims = [c4 + self.s4_sign * self.pos4(leg_geom.max_angle[4]), c4 + self.s4_sign * self.pos4(leg_geom.min_angle[4])]
        s1lims.sort()
        s2lims.sort()
        s3lims.sort()
        s4lims.sort()
        self.s1min, self.s1max = s1lims
        self.s2min, self.s2max = s2lims
        self.s3min, self.s3max = s3lims
        self.s4min, self.s4max = s4lims
        self.s1min = self.s1min if self.s1min >= 0 else 0
        self.s2min = self.s2min if self.s2min >= 0 else 0
        self.s3min = self.s3min if self.s3min >= 0 else 0
        self.s4min = self.s4min if self.s4min >= 0 else 0

        # Convert offsets in degrees to servo values
        self.s1_center_angle = self.s1_sign * (leg_geom.aoffset1 + a1_stance_offset)
        self.s1_center_radians = self.s1_center_angle / RAD_TO_ANGLE
        self.s1_center = c1 + self.pos1(self.s1_center_angle)
        self.s2_center = c2 + self.pos2(self.s2_sign * leg_geom.aoffset2)
        self.s3_center = c3 + self.pos3(self.s3_sign * leg_geom.aoffset3)
        if leg_geom.aoffset4:
            self.s4_center = c4 + self.pos4(self.s4_sign * leg_geom.aoffset4)
        else:
            self.s4_center = c4

    def get_pos_from_angle(self, a1, a2, a3, a4=None):
        # Angles are in degrees. Returns list of servo positions
        positions = [
                self.s1_center + self.pos1(a1),  # Remember to supply offset from center, not absolute angle
                self.s2_center + self.pos2(self.s2_sign * self.leg_geom.joint2sign * a2),
                self.s3_center + self.pos3(self.s3_sign * self.leg_geom.joint3sign * a3),
        ]

        if a4 and self.s4_sign:
            positions.append(self.s4_center + self.pos4(
                self.s4_sign * self.leg_geom.joint4sign * a4))
        else:
            positions.append(self.s4_center)

        return positions

    def get_pos_from_radians(self, a1, a2, a3, a4=None):
        # Current convention is we convert all angles from radians to degrees
        positions = [
                self.s1_center + self.pos1(RAD_TO_ANGLE * a1),
                self.s2_center + self.pos2(RAD_TO_ANGLE * self.s2_sign * self.leg_geom.joint2sign * a2),
                self.s3_center + self.pos3(RAD_TO_ANGLE * self.s3_sign * self.leg_geom.joint3sign * a3),
        ]

        if a4 and self.leg_geom.aoffset4:
            positions.append(self.s4_center + self.pos4(
                RAD_TO_ANGLE * self.s4_sign * self.leg_geom.joint4sign * a4))
        else:
            positions.append(self.s4_center)

        return positions
