#!/usr/bin/env python
import socket
import rospy
from geometry_msgs.msg import Twist
import serial
import struct
from time import sleep

hostname = socket.gethostname()
if hostname.startswith("orange"): # orange pi zero 3; other boards may not be the same uart #
    SERIAL_PORT_PATH = "/dev/ttyAS5"  # Uart5; enabled in orangepi-config
else:  # Rpi
    SERIAL_PORT_PATH = "/dev/serial0"
BAUD = 115200

HEADER = b'\xAB\xCD'  # Header bytes

def cmd_vel_callback(msg):
    # Extract linear and angular velocities
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    # Prepare the packet (header, 2 floats for data, and a checksum)
    packet = HEADER + struct.pack('ff', linear_x, angular_z)
    checksum = sum(bytearray(packet)) % 256
    print(checksum)
    packet += struct.pack('B', checksum)

    # Send the packet over UART
    ser.write(packet)

def main():
    # Initialize ROS node
    rospy.init_node('cmd_vel_to_uart')

    # Open serial port
    # TODO retry on failure
    global ser
    ser = serial.Serial(SERIAL_PORT_PATH, BAUD, timeout=1)
    rospy.loginfo("Started")

    # Subscribe to cmd_vel
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    # Keep the program alive
    #rospy.spin()
    #return
    try:
        rospy.spin()
    # TODO: Not clear that these exceptions are triggering on ctrl+c
    except KeyboardInterrupt:
        # Close the serial port when the node is stopped
        ser.close()
        rospy.loginfo("Cleaned up " + SERIAL_PORT_PATH)
        sleep(0.5)
        #raise
    except Exception:
        ser.close()
        rospy.loginfo("Cleaned up " + SERIAL_PORT_PATH)
        sleep(0.5)
        #raise

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Bye")
        pass

