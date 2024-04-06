#from sys import stdin
from two_wheel import Hardware

from pyb import Pin, UART, ADC
from time import sleep
import struct

hw = Hardware()
phase = 0
#start = stdin.readline()
#while True:
#    if phase == 0:
#        hw.forward_wheels()
#        pass
#    elif phase == 1 or phase == 3:
#        hw.stop_wheels()
#        pass
#    elif phase == 2:
#        hw.backward_wheels()
#        pass
#    phase += 1
#    phase %= 4
#    start = stdin.readline()


HEADER = b'\xAB\xCD'  # Header bytes

def read_cmd_vel(uart):
    # Define the expected packet size (2 bytes for header, 2 floats for data, 1 byte for checksum)
    PACKET_SIZE = 2 + 8 + 1


    # Look for the header
    header_buffer = b'\x00\x00'
    while True:
        #header = serial_port.read(2)
        byte = uart.read(1)
        if byte is None:
            continue
        header_buffer = header_buffer[1:] + byte
        if header_buffer is None:
            print("what the heck")
            continue
        if header_buffer == HEADER:
            break
        sleep(0.001)

    # Read the rest of the packet
    while uart.any() < PACKET_SIZE - 2:
        sleep(0.001)
    #print("Got enough bytes for a packet!")
    #packet = uart.read(PACKET_SIZE - 2)
    packet = HEADER + uart.read(PACKET_SIZE - 2)
    if packet is None:
        print("empty rest of the packet; what the heck")
        return None

    # Verify packet size
    if len(packet) != PACKET_SIZE:# - 2:
        return None

    # Unpack the data and the checksum
    #linear_x, angular_z, checksum_received = struct.unpack('2sffB', packet)
    linear_x, angular_z, checksum_received = struct.unpack('ffB', packet[2:])

    # Calculate checksum
    checksum_calculated = sum(packet[:-1]) % 256

    # Validate checksum
    if checksum_received != checksum_calculated:
        print("Bad checksum: ", checksum_received, packet[-1], checksum_calculated, packet, ["{}".format(x) for x in packet])
        return None

    # Empty buffer
    uart.read(uart.any())

    # Return the extracted values
    return linear_x, angular_z

# Example usage
#serial_port = serial.Serial('/dev/ttyS0', 115200)
print("Starting!")
rpibus = UART(4, 115200)  # 38400  - what I used in prior testing
while True:
    cmd_vel = read_cmd_vel(rpibus)#serial_port)
    if cmd_vel:
        linear_x, angular_z = cmd_vel
        print("Got:", linear_x, angular_z)
        hw.set_velocity(linear_x, angular_z)
        # Set motor values or perform other actions based on cmd_vel
    
        # TODO update drive speed, etc.
