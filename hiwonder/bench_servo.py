import sys
sys.path.insert(0, '/sd')
from stm_uart_port import UART_Port
from hiwonder_bus import Bus
import hiwonder_packet as packet
import utime

port = UART_Port(2, 115200)
bus = Bus(port)
N = 100

# Write benchmark — alternate positions so servo is actually commanded to move
t0 = utime.ticks_us()
for i in range(N):
    pos = 0 if i % 2 == 0 else 500
    data = bytearray([pos & 0xff, pos >> 8, 200, 0])  # 200ms move time
    bus.write(1, packet.Command.MOVE_TIME_WRITE, data)
write_us = utime.ticks_diff(utime.ticks_us(), t0)
write_per_cmd = write_us / N
print("Write: {:.1f} us/cmd, {:.0f} cmd/s".format(write_per_cmd, N * 1e6 / write_us))

# Read benchmark
t0 = utime.ticks_us()
for i in range(N):
    bus.read(1, packet.Command.POS_READ)
read_us = utime.ticks_diff(utime.ticks_us(), t0)
read_per_cmd = read_us / N
print("Read:  {:.1f} us/cmd, {:.0f} cmd/s".format(read_per_cmd, N * 1e6 / read_us))

# Projections for N servos (write + one read per cycle)
print()
print("Projected max update rate (write-all + read-all per cycle):")
for n in [1, 4, 8, 14]:
    cycle_us = n * (write_per_cmd + read_per_cmd)
    hz = 1e6 / cycle_us
    print("  {:>2} servos: {:.0f} us/cycle -> {:.1f} Hz".format(n, cycle_us, hz))

print()
print("Projected max update rate (write-only, no reads):")
for n in [1, 4, 8, 14]:
    cycle_us = n * write_per_cmd
    hz = 1e6 / cycle_us
    print("  {:>2} servos: {:.0f} us/cycle -> {:.1f} Hz".format(n, cycle_us, hz))
