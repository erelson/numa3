"""Tests for the Feetech protocol module.

Run with:  python3 test_feetech.py

Every instruction packet is asserted against the worked example in
"Communication Protocol User Manual-EN(191218-0923).pdf" (repo root), quoted in
each test, so the implementation is pinned to the document rather than to my
reading of it.
"""
import feetech as ft


def hexs(b):
    return " ".join("%02X" % x for x in b)


def test_ping_example_1():
    # 1.3.1 Example 1, id 1: FF FF 01 02 01 FB
    assert hexs(ft.build_ping(0x01)) == "FF FF 01 02 01 FB"


def test_read_example_2():
    # 1.3.2 Example 2: read 2 bytes at 0x38 from id 1
    assert hexs(ft.build_read(0x01, ft.Register.PRESENT_POSITION, 2)) == \
        "FF FF 01 04 02 38 02 BE"


def test_write_id_example_3():
    # 1.3.3 Example 3: set id to 1 by writing 1 to address 5, via broadcast
    assert hexs(ft.build_write(ft.BROADCAST_ID, ft.Register.ID, [0x01])) == \
        "FF FF FE 04 03 05 01 F4"


def test_write_goal_block_example_4():
    # 1.3.3 Example 4: id 1 -> position 2048, time 0, speed 1000 at 0x2A
    data = (ft.pack_u16(2048) + ft.pack_u16(0) + ft.pack_u16(1000))
    assert hexs(ft.build_write(0x01, ft.Register.GOAL_POSITION, data)) == \
        "FF FF 01 09 03 2A 00 08 00 00 E8 03 D5"


def test_action_example_6():
    # 1.3.5 Example 6: broadcast ACTION
    assert hexs(ft.build_action()) == "FF FF FE 02 05 FA"


def test_sync_write_example_7():
    # 1.3.6 Example 7: ids 1-4, addr 0x2A, position 2048 / time 0 / speed 1000
    block = ft.pack_u16(2048) + ft.pack_u16(0) + ft.pack_u16(1000)
    packet = ft.build_sync_write(ft.Register.GOAL_POSITION,
                                 [(i, block) for i in (1, 2, 3, 4)])
    assert hexs(packet) == (
        "FF FF FE 20 83 2A 06 "
        "01 00 08 00 00 E8 03 "
        "02 00 08 00 00 E8 03 "
        "03 00 08 00 00 E8 03 "
        "04 00 08 00 00 E8 03 58")
    # manual: Length = (L+1)*N + 4
    assert packet[3] == (6 + 1) * 4 + 4 == 0x20


def test_sync_read_example_8():
    # 1.3.7 Example 8: ids 1-2, 8 bytes from 0x38
    packet = ft.build_sync_read(ft.Register.PRESENT_POSITION, 8, [1, 2])
    assert hexs(packet) == "FF FF FE 06 82 38 08 01 02 36"
    assert packet[3] == 2 + 4  # manual: Length = N + 4


def test_reset_example():
    # 1.3.8: reset id 0 -> FF FF 00 02 06 F7 (table); id 1 -> ...06 F6 (text)
    assert hexs(ft.build_reset(0x00)) == "FF FF 00 02 06 F7"
    assert hexs(ft.build_reset(0x01)) == "FF FF 01 02 06 F6"


def test_response_parsing_example_2():
    # 1.3.2 Example 2 reply: position 0x0518 = 1304, low byte first
    dev_id, error, params = ft.parse_response(
        bytes([0xFF, 0xFF, 0x01, 0x04, 0x00, 0x18, 0x05, 0xDD]))
    assert (dev_id, error) == (1, 0)
    assert ft.unpack_u16(params) == 0x0518 == 1304


def test_response_parsing_sync_read_example_8():
    # 1.3.7 Example 8 replies, 8 params each
    for raw, expect_id in (
            (bytes([0xFF, 0xFF, 0x01, 0x0A, 0x00,
                    0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x79, 0x1E, 0x55]), 1),
            (bytes([0xFF, 0xFF, 0x02, 0x0A, 0x00,
                    0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x77, 0x23, 0x53]), 2)):
        dev_id, error, params = ft.parse_response(raw)
        assert (dev_id, error, len(params)) == (expect_id, 0, 8)
    # id1 position 0x0800 = 2048; id2 position 0x07FF = 2047
    assert ft.unpack_u16(b"\x00\x08") == 2048
    assert ft.unpack_u16(b"\xff\x07") == 2047


def test_byte_order_matters():
    # Manual 1.0: potentiometer series is high byte first, magnetic encoder low.
    assert ft.pack_u16(0x0518, ft.BYTE_ORDER_LOW_FIRST) == [0x18, 0x05]
    assert ft.pack_u16(0x0518, ft.BYTE_ORDER_HIGH_FIRST) == [0x05, 0x18]
    for order in (ft.BYTE_ORDER_LOW_FIRST, ft.BYTE_ORDER_HIGH_FIRST):
        packed = ft.pack_u16(1304, order)
        assert ft.unpack_u16(packed, order) == 1304, order


def test_checksum_rule():
    # Manual 1.1: Checksum = ~(ID + Length + Instruction + params), low byte
    assert ft.checksum([0x01, 0x02, 0x01]) == 0xFB
    assert ft.checksum([0xFE, 0x04, 0x03, 0x05, 0x01]) == 0xF4


def test_sync_write_rejects_ragged_blocks():
    # Manual 1.3.6: the data length must be the same for every servo.
    try:
        ft.build_sync_write(0x2A, [(1, [0, 1]), (2, [0, 1, 2])])
    except ValueError as exc:
        assert "same" in str(exc) or "bytes" in str(exc), exc
    else:
        assert False, "ragged sync write should have raised"


def test_response_rejects_bad_checksum():
    bad = bytes([0xFF, 0xFF, 0x01, 0x04, 0x00, 0x18, 0x05, 0x00])
    try:
        ft.parse_response(bad)
    except ValueError as exc:
        assert "checksum" in str(exc), exc
    else:
        assert False, "bad checksum should have raised"


def test_sts_control_table():
    # Addresses corroborated by the protocol manual's examples, FEETECH's
    # STS3215 spec sheet, and matthieuvigne/STS_servos.
    R = ft.Register
    assert (R.ID, R.BAUD_RATE, R.RESPONSE_STATUS_LEVEL) == (5, 6, 8)
    assert (R.MIN_ANGLE_LIMIT, R.MAX_ANGLE_LIMIT) == (9, 11)
    assert (R.TORQUE_ENABLE, R.GOAL_POSITION, R.GOAL_TIME, R.GOAL_SPEED) == \
        (40, 42, 44, 46)
    assert (R.LOCK, R.PRESENT_POSITION) == (55, 56)
    # The manual's 6-byte goal block and 8-byte present block must be contiguous
    assert R.GOAL_TIME == R.GOAL_POSITION + 2
    assert R.GOAL_SPEED == R.GOAL_TIME + 2
    assert (R.PRESENT_SPEED, R.PRESENT_LOAD) == (58, 60)
    assert (R.PRESENT_VOLTAGE, R.PRESENT_TEMPERATURE) == (62, 63)
    # example 8 read 8 bytes from PRESENT_POSITION and got exactly this span
    assert R.PRESENT_TEMPERATURE - R.PRESENT_POSITION + 1 == 8


def test_sts_position_scale():
    # Datasheet 7-6..7-8: 2048 = 180 deg, 360 deg over 4096, 0.088 deg/count
    assert ft.POSITION_RESOLUTION == 4096
    assert ft.POSITION_CENTER == 2048
    assert abs(ft.DEGREES_PER_COUNT - 0.088) < 0.0005
    assert ft.POSITION_CENTER * ft.DEGREES_PER_COUNT == 180.0
    # 12-bit magnetic encoder series -> low byte first
    assert ft.STS_BYTE_ORDER == ft.BYTE_ORDER_LOW_FIRST


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print("\nall %d tests passed" % len(tests))


if __name__ == "__main__":
    main()
