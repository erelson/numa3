"""
Measures how long a HiWonder and/or Dynamixel servo takes to rise by
TEMP_RISE_TARGET degrees C while moving back and forth continuously.

Wiring:
  HiWonder servo  -> UART 4 (half-duplex), 115200 baud
  Dynamixel servo -> UART 2 (half-duplex), 1 Mbaud

Run with:
    mpremote connect /dev/ttyACM0 run hiwonder/temp_rise_test.py
"""

import sys
sys.path.insert(0, '/sd')
import struct
import utime

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
TEMP_RISE_TARGET  = 2     # degrees C rise to declare done
MOVE_PERIOD_MS    = 600   # ms between position commands
TEMP_SAMPLE_MS    = 4000  # ms between temperature reads

ENABLE_HIWONDER   = True
ENABLE_DYNAMIXEL  = True

HW_ID             = 1
HW_POS_A          = 50    # 0–1000 (full range = 0–240 deg)
HW_POS_B          = 950
HW_MOVE_TIME_MS   = 500   # servo travel time per move

AX_ID             = 52
AX_POS_A          = 420   # within ID 52 limits (387–771)
AX_POS_B          = 740
AX_MOVING_SPEED   = 300   # 0=max, 1–1023 proportional
# ---------------------------------------------------------------------------

from stm_uart_port import UART_Port

if ENABLE_HIWONDER:
    from hiwonder_bus import Bus as HWBus
    import hiwonder_packet as hw_pkt

    hw_port = UART_Port(4, 115200)
    hw_bus  = HWBus(hw_port)

    def hw_move(pos):
        data = bytearray([pos & 0xff, (pos >> 8) & 0xff,
                          HW_MOVE_TIME_MS & 0xff, (HW_MOVE_TIME_MS >> 8) & 0xff])
        hw_bus.write(HW_ID, hw_pkt.Command.MOVE_TIME_WRITE, data)

    def hw_read_temp():
        return hw_bus.read(HW_ID, hw_pkt.Command.TEMP_READ)[0]

    def hw_stop():
        hw_bus.fill_and_write_packet(HW_ID, hw_pkt.Command.MOVE_STOP)

if ENABLE_DYNAMIXEL:
    from bus import Bus as AXBus
    import ax

    ax_port = UART_Port(2, 1000000)
    ax_bus  = AXBus(ax_port)
    ax_bus.sync_write([AX_ID], ax.MOVING_SPEED, [struct.pack('<H', AX_MOVING_SPEED)])
    utime.sleep_ms(25)
    ax_bus.sync_write([AX_ID], ax.TORQUE_ENABLE, [bytearray([1])])
    utime.sleep_ms(25)

    def ax_move(pos):
        ax_bus.sync_write([AX_ID], ax.GOAL_POSITION, [struct.pack('<H', pos)])

    def ax_read_temp():
        return ax_bus.read(AX_ID, ax.PRESENT_TEMP, 1)[0]

    def ax_stop():
        ax_bus.sync_write([AX_ID], ax.TORQUE_ENABLE, [bytearray([0])])

# --- Read starting temperatures and print config ---
hw_start = hw_read_temp() if ENABLE_HIWONDER else None
ax_start = ax_read_temp() if ENABLE_DYNAMIXEL else None

if ENABLE_HIWONDER:
    print("HiWonder  ID={} UART4: start temp {}C".format(HW_ID, hw_start))
if ENABLE_DYNAMIXEL:
    print("Dynamixel ID={} UART2: start temp {}C".format(AX_ID, ax_start))
print("Target: +{}C  move_period={}ms  sample_period={}s".format(
    TEMP_RISE_TARGET, MOVE_PERIOD_MS, TEMP_SAMPLE_MS // 1000))
print()

# --- Main loop ---
start_time     = utime.ticks_ms()
last_move      = start_time
last_temp_read = start_time
target_idx     = 0
hw_done        = not ENABLE_HIWONDER
ax_done        = not ENABLE_DYNAMIXEL

if ENABLE_HIWONDER:
    hw_move(HW_POS_A)
if ENABLE_DYNAMIXEL:
    ax_move(AX_POS_A)

while not (hw_done and ax_done):
    now = utime.ticks_ms()

    if utime.ticks_diff(now, last_move) >= MOVE_PERIOD_MS:
        target_idx = 1 - target_idx
        if not hw_done:
            hw_move([HW_POS_A, HW_POS_B][target_idx])
        if not ax_done:
            ax_move([AX_POS_A, AX_POS_B][target_idx])
        last_move = now

    if utime.ticks_diff(now, last_temp_read) >= TEMP_SAMPLE_MS:
        elapsed_s = utime.ticks_diff(now, start_time) / 1000.0

        if not hw_done:
            t = hw_read_temp()
            rise = t - hw_start
            print("t={:.0f}s  HiWonder  {}C  rise={}C".format(elapsed_s, t, rise))
            if rise >= TEMP_RISE_TARGET:
                print("  -> HiWonder done: +{}C in {:.0f}s".format(rise, elapsed_s))
                hw_stop()
                hw_done = True

        if not ax_done:
            t = ax_read_temp()
            rise = t - ax_start
            print("t={:.0f}s  Dynamixel {}C  rise={}C".format(elapsed_s, t, rise))
            if rise >= TEMP_RISE_TARGET:
                print("  -> Dynamixel done: +{}C in {:.0f}s".format(rise, elapsed_s))
                ax_stop()
                ax_done = True

        last_temp_read = now

    utime.sleep_ms(20)

print("Done.")
