import struct
import sys
sysname = sys.platform
if sysname == 'linux' or sysname == 'win32':
    # Mocks for non-pyboard use
    import time
    def sleep_ms(x): return time.sleep(x/1e3)
elif sysname == 'pyboard':
    from utime import sleep_ms

import ax
from poses import AX_CENTER
from servo_group import RETURN_LEVEL_READ_ONLY


# See WalkingOmni.nb
#Center angles for coax servo of each leg.plus test offsets.
#Measured from 0deg = front of bot  --------- this doesn't look right.
#/But are the

# Servo position limits, range from 0 to 1023
# From AX12 manual: CW Angle Limit <= Goal Position <= CCW
PAN_CENTER = AX_CENTER + 153


def initServoLims(leg_servos, axbus, turret_ids, gaits):
    l1, l2, l3, l4 = gaits.leg1, gaits.leg2, gaits.leg3, gaits.leg4
    # Limits in leg_ids order: coax x4, femur x4, tibia x4
    leg_lims = [
        [l1.s1min, l1.s1max], [l2.s1min, l2.s1max], [l3.s1min, l3.s1max], [l4.s1min, l4.s1max],
        [l1.s2min, l1.s2max], [l2.s2min, l2.s2max], [l3.s2min, l3.s2max], [l4.s2min, l4.s2max],
        [l1.s3min, l1.s3max], [l2.s3min, l2.s3max], [l3.s3min, l3.s3max], [l4.s3min, l4.s3max],
    ]
    turret_lims = [
        [PAN_CENTER - 4 * (52+30), PAN_CENTER + 4 * (52+30)],  # 51
        [AX_CENTER - 4 * 31,       AX_CENTER + 4 * 65],         # 52
    ]
    leg_servos.write_angle_limits(leg_lims)
    sleep_ms(25)
    axbus.sync_write(turret_ids, ax.CW_ANGLE_LIMIT_L,  [struct.pack('<H', lim[0]) for lim in turret_lims])
    sleep_ms(25)
    axbus.sync_write(turret_ids, ax.CCW_ANGLE_LIMIT_L, [struct.pack('<H', lim[1]) for lim in turret_lims])
    sleep_ms(25)
    leg_servos.write_torque(True)
    axbus.sync_write(turret_ids, ax.TORQUE_ENABLE, [bytearray([1]) for _ in turret_ids])
    sleep_ms(25)


COAX_SPEED = 200
SERVO_SPEED = 300
TURRET_SERVO_SPEED = 200
def myServoSpeeds(leg_servos, axbus, turret_ids):
    leg_servos.write_speed(SERVO_SPEED)
    sleep_ms(25)
    leg_servos.write_speed(COAX_SPEED, indices=range(4, 8))
    sleep_ms(25)
    axbus.sync_write(turret_ids, ax.MOVING_SPEED,
                     [struct.pack('<H', TURRET_SERVO_SPEED) for _ in turret_ids])


# Reply to reads only, so writes are never acknowledged. Applied to every leg
# servo that has the register: AX (RETURN_LEVEL) and Feetech
# (RESPONSE_STATUS_LEVEL) share this encoding; HiWonder is fixed here already.
RTN_LVL = RETURN_LEVEL_READ_ONLY
def myServoReturnLevels(leg_servos, axbus, turret_ids):
    leg_servos.write_return_level(RTN_LVL)
    axbus.sync_write(turret_ids, ax.RETURN_LEVEL,
                     [bytearray([RTN_LVL]) for _ in turret_ids])
    sleep_ms(25)
