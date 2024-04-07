from math import sqrt, pi
from IK import Gaits, calc_foot_h
from poses import gen_numa2_legs, LegGeom, LegDef
import unittest


RAD_TO_ANGLE = 180./pi

# Simple "unit leg" model a la poses.gen_numa2_legs()
def gen_simple_legs():
# 4\ __^__ /3
#   |     | 
#   |     | 
#   |_____| 
# 1/       \2
    stance = 0  # degrees; see README
    offsets_dict = {
            # Offsets are in degrees
            #"aoffset1": 45.0, # this one is special
            #"aoffset2": 31.54,
            #"aoffset3": 31.54 - 5.63, # off_b - off_h
            "aoffset1": 0, #45.0, # this one is special
            "aoffset2": 0, #31.54,
            "aoffset3": 0, #31.54 - 5.63, # off_b - off_h
            "a1stance": stance,
            "a1stance_rear": 0,
            "L0": 2,  # ???  But does this affect things?
            "L12": 1,
            "L23": sqrt(2),
            "L34": sqrt(2.),
            "L45": 0,
        #print(self.sqL34, sqL24, self.sqL23, self.L34, L24)
        # TODO why is sqL24 gigantic?
            # mins/max are in degrees from actual servo center (not joint center!)
            # TODO ugh what do I do here? 4-5-2024
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
    leg1 = LegDef(leg_model, offsets_dict,  1, -1,  1)
    leg2 = LegDef(leg_model, offsets_dict, -1,  1, -1)
    leg3 = LegDef(leg_model, offsets_dict,  1, -1,  1, front_leg=True)
    leg4 = LegDef(leg_model, offsets_dict, -1,  1, -1, front_leg=True)

    return leg_model, leg1, leg2, leg3, leg4

# TODO: Why did I make this class?
class TestGait(Gaits):
    def __init__(self):
        pass

class TestGenericLegKinem(unittest.TestCase):
    # genericLegKinem solves for angles given a footH and legLen

    def test_genericLegKinem(self):
        # Points 0,0, 2,2, 3,1
        g = TestGait()
        g.L12 = 1
        g.bodyH = 1.
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.L23 = 2*sqrt(2)
        g.sqL23 = 8
        g.L34 = sqrt(2.)
        g.sqL34 = 2.

        a2, a3, _ = g.genericLegKinem(footH=2, legLen=g.L12+3)

        self.assertAlmostEqual(pi/4, a2)
        self.assertAlmostEqual(-pi/2, a3)

    def test_genericLegKinem2(self):
        # Points 0,0, 1,1, 2,0
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 1.
        g.L23 = sqrt(2.)
        g.sqL23 = 2.
        g.L34 = sqrt(2.)
        g.sqL34 = 2.

        a2, a3, _ = g.genericLegKinem(footH=1, legLen=g.L12+2)

        self.assertAlmostEqual(pi/4, a2)
        self.assertAlmostEqual(-pi/2, a3)

    def test_genericLegKinem3(self):
        # Points 0,0, 1,1, 2,1
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 1.
        g.L23 = sqrt(2.)
        g.sqL23 = 2.
        g.L34 = 1.
        g.sqL34 = 1.

        a2, a3, _ = g.genericLegKinem(footH=2, legLen=g.L12+2)

        self.assertAlmostEqual(pi/4, a2)
        self.assertAlmostEqual(-pi/4, a3)

# TODO we don't handle the leg being below horizontal yet
    def test_genericLegKinem4(self):
        # Points 0,0, 1,-1, 1,-2
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 2.
        g.L23 = sqrt(2.)
        g.sqL23 = 2.
        g.L34 = 1.
        g.sqL34 = 1.

        a2, a3, _ = g.genericLegKinem(footH=0, legLen=g.L12+1)
        print(g.v23, g.v34)

        self.assertAlmostEqual(-pi/4, a2)
        self.assertAlmostEqual(-pi/4, a3)

    def test_genericLegKinem5(self):
        # Points 0,0, 1,1, 1,-1
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 1.
        g.L23 = sqrt(2.)
        g.sqL23 = 2.
        g.L34 = 2.
        g.sqL34 = 4.

        a2, a3, _ = g.genericLegKinem(footH=0, legLen=g.L12+1)
        print(g.v23, g.v34)

        self.assertAlmostEqual(pi/4, a2)
        self.assertAlmostEqual(-3*pi/4, a3)

    def test_genericLegKinem6(self):
        # Points 0,0, 1,0, 2,-1
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 1.
        g.L23 = 1.
        g.sqL23 = 1.
        g.L34 = sqrt(2.)
        g.sqL34 = 2.

        a2, a3, _ = g.genericLegKinem(footH=0, legLen=g.L12+2)
        print(g.v23, g.v34)

        self.assertAlmostEqual(0, a2)
        self.assertAlmostEqual(-pi/4, a3)



class TestCalcFootH(unittest.TestCase):
    def test_calc_foot_h(self):
        foot_h_max = 1
        time_down_frac = 0.1
        half_loopLen = 50 # our `now` is between 0 and this
        trans_frac = 0.1
        h_frac = 0.2

        # zero; on ground for first half
        footh = calc_foot_h(0, foot_h_max, time_down_frac, half_loopLen, trans_frac, h_frac)
        self.assertEqual(footh, 0)
        # still on ground
        footh = calc_foot_h(half_loopLen, foot_h_max, time_down_frac, half_loopLen, trans_frac, h_frac)
        self.assertEqual(footh, 0)
        # going up slowly
        # going up quickly
        # steady at max height
        footh = calc_foot_h(1.5*half_loopLen, foot_h_max, time_down_frac, half_loopLen, trans_frac, h_frac)
        self.assertEqual(footh, foot_h_max)
        # going down quickly
        # going down slowly
        # idle on ground
        footh = calc_foot_h(2*half_loopLen, foot_h_max, time_down_frac, half_loopLen, trans_frac, h_frac)
        self.assertEqual(footh, 0)
        # again

# Commented out 4-5-2024
#class TestDoLegKinem(unittest.TestCase):
#    # doLegKinem ... solves for angles given a footH and offset vector(?) of the foot from neutral position
#
#    def test_do_leg_kinem(self):
#        #foot_h_max = 1
#        #time_down_frac = 0.1
#        #loopLen = 100 # our `now` is between 0 and this
#        #trans_frac = 0.1
#        #h_frac = 0.2
#
#        # Points 0,0, 2,2, 3,1
#        leg_geom, leg1, leg2, leg3, leg4 = gen_numa2_legs()
#        g = Gaits(leg_geom, leg1, leg2, leg3, leg4)
#        #g = TestGait()
#        #g.L12 = 1
#        #g.bodyH = 1.
#        #g.L45 = 0
#        #g.v12 = [g.L12, 0]
#        #g.L23 = 2*sqrt(2)
#        #g.sqL23 = 8
#        #g.L34 = sqrt(2.)
#        #g.sqL34 = 2.
#
#        #n1, n2, n3, n4 = 1
#        # l1a2 = leg 1, servo angle 2
#        # trav cdir/sdir are zero at the neutral leg position
#        l1a2, l1a3, _ = g.doLegKinem(trav_cdir=0, trav_sdir=0, footH=0)
#        l2a2, l2a3, _ = g.doLegKinem(trav_cdir=0, trav_sdir=0, footH=0)
#
#        #self.assertAlmostEqual(0, g.footH13)
#        #self.assertAlmostEqual(0, g.footH24)
#        #print(leg1.s2_center, g.s12pos, leg1.s2_sign * leg_geom.aoffset2)
#        #print(leg2.s2_center, g.s22pos, leg2.s2_sign * leg_geom.aoffset2)
#        #print(leg1.s2_center, g.s12pos, leg1.s2_sign * leg_geom.aoffset2)
#        #print(leg2.s2_center, g.s22pos, leg2.s2_sign * leg_geom.aoffset2)
#        #self.assertAlmostEqual(l1a2 * RAD_TO_ANGLE, leg1.s2_sign * leg_geom.aoffset2)
#        #self.assertAlmostEqual(l2a2 * RAD_TO_ANGLE, leg2.s2_sign * leg_geom.aoffset2)
#        print(l1a2, l1a3)
#        print(l2a2, l2a3)
#        # ??? vs ??? aoffset2
#        # Assert that the position angle for the second servo from doLegKinem
#        # is equivalent to the offset angle for servo 2 (times +/-1 depending
#        # on the leg)
#        self.assertAlmostEqual(l1a2 * RAD_TO_ANGLE, leg1.s2_sign * leg_geom.aoffset2)
#        self.assertAlmostEqual(l2a2 * RAD_TO_ANGLE, leg2.s2_sign * leg_geom.aoffset2)
#        #self.assertAlmostEqual(leg1.s2_center, g.s12pos)
#        #self.assertAlmostEqual(leg2.s2_center, g.s22pos)

class TestWalkCode(unittest.TestCase):
    def test_trav_calculation(self):
        foot_h_max = 1
        time_down_frac = 0.1
        loopLen = 100 # our `now` is between 0 and this
        trans_frac = 0.1
        h_frac = 0.2
        travRate = 0.5  # ??? Was 10, probably way too big

        # We define a leg with a valid orientation where the joints are at points 0,0, 1,1, 2,0 ... are set by gen_simple_legs()
        leg_geom, leg1, leg2, leg3, leg4 = gen_simple_legs()
        g = Gaits(leg_geom, leg1, leg2, leg3, leg4, bodyH=1)
        g.initTrig()

        # numa.get_now(ms = 0)
        n1, n2, n3, n4 = loopLen/2, 0, loopLen/2, 0  # From plots # TODO what part of the gait is this?
        ang_dir = 0  # forward
        g.walk_code(loopLen, loopLen/2, travRate, n1, n2, n3, n4, ang_dir)

        self.assertAlmostEqual(0, g.footH13)
        self.assertAlmostEqual(0, g.footH24)

#    def test_walk_code(self):
#        foot_h_max = 1
#        time_down_frac = 0.1
#        loopLen = 100 # our `now` is between 0 and this
#        trans_frac = 0.1
#        h_frac = 0.2
#
#        # Points 0,0, 2,2, 3,1
#        g = TestGait()
#        g.L12 = 1
#        g.bodyH = 1.
#        g.L45 = 0
#        g.v12 = [g.L12, 0]
#        g.L23 = 2*sqrt(2)
#        g.sqL23 = 8
#        g.L34 = sqrt(2.)
#        g.sqL34 = 2.
#
#        n1, n2, n3, n4 = 1
#        a2, a3, _ = g.walk_code(loopLen, loopLen/2, travRate, n1, n2, n3, n4)
#
#        self.assertAlmostEqual(0, g.footH13)
#        self.assertAlmostEqual(0, g.footH24)
