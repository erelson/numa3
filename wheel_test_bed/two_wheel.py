from pyb import Pin
from MotorDriver import MotorDriver

instantiated = False  # crude singleton implementation because I'm too lazy to look up one

class Hardware():
    def __init__(self):
        # Note: cs pin is not wired...
        # Account for difference in right/left motor performance.
        self.right_scale = 0.9175  # Good from 0.3 to 0.5 at least (no load). Falls out of sync at 0.2
        self.left_scale = 1.0

        global instantiated
        if not instantiated:
            self.rightMotor = MotorDriver(Pin.board.X5, Pin.board.X8, Pin.board.X6, cs=Pin.board.X7)
            self.leftMotor = MotorDriver(Pin.board.Y9, Pin.board.Y12, Pin.board.Y10, cs=Pin.board.Y11)
            instantiated = True
            # TODO: do we need to do run_forward/reversed on either motor?
            self.leftMotor.run_reversed()
        else:
            print("WARNING already set up the hardware before")
            # TODO For ad-hoc testing, could we access the instantiated r/l motor instances? should we?
    
    def stop_wheels(self):
        self.rightMotor.direct_set_speed(0)  # 127 max
        self.leftMotor.direct_set_speed(0)  # 127 max


    def forward_wheels(self):
        self.rightMotor.direct_set_speed(50)  # 127 max
        self.leftMotor.direct_set_speed(50)  # 127 max

    def backward_wheels(self):
        self.rightMotor.direct_set_speed(-50)  # 127 max
        self.leftMotor.direct_set_speed(-50)  # 127 max

    def set_velocity(self, linear_x, angular_z):
        
        # Linear_x is typically in meters/second
        # angular_z is in radians/s?

        # Scale factors (adjust these based on your robot's specifics)
        MAX = 70
        linear_scale = 50  # Max linear speed scale
        angular_scale = 25  # Max angular speed difference scale

        # Calculate wheel speeds
        right_speed = linear_x * linear_scale * self.right_scale + angular_z * angular_scale
        left_speed = linear_x * linear_scale * self.left_scale - angular_z * angular_scale

        # Limit speeds to maximum values (assuming 127 is max speed)
        right_speed = max(min(right_speed, 127), -127)
        left_speed = max(min(left_speed, 127), -127)

        # Set motor speeds
        self.rightMotor.direct_set_speed(right_speed)
        self.leftMotor.direct_set_speed(left_speed)   

        # Set timer that will trigger `stop_wheels` upon expiration.
        # On the RPi, we expect keyboard teleop to publich cmd_vel at a rate of ?? Hz

