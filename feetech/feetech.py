"""Feetech serial bus servo protocol (SCS / STS series).

Implemented from "Communication Protocol User Manual-EN(191218-0923).pdf" in
the repo root. Every packet builder here is checked byte-for-byte against the
manual's worked examples in feetech/test_feetech.py.

Framing is IDENTICAL to Dynamixel protocol 1.0:

    FF FF | ID | Length | Instruction | Param1..ParamN | Checksum
    Length   = N + 2
    Checksum = ~(ID + Length + Instruction + Param1..ParamN) & 0xFF

so bioloid3's Bus already carries PING / READ / WRITE / REG_WRITE / ACTION /
SYNC_WRITE / RESET unchanged -- no second bus implementation is needed, which
matters because the PyBoard's flash is nearly full. What this module adds is
the Feetech-specific parts: the SYNC READ instruction (0x82, which Dynamixel
1.0 does not have), the series-dependent byte order, and the register
addresses the manual documents.

BYTE ORDER (manual section 1.0): two-byte values are sent
  * high byte first on the POTENTIOMETER series (SCS), and
  * low byte first on the MAGNETIC ENCODER series (STS),
so it must be chosen per servo model -- see BYTE_ORDER_* below. The manual's
own examples (positions, speeds) are all low-byte-first.

CONTROL TABLE: the manual is protocol-only and states that the memory table is
per model ("please refer to the memory table of the specific model"). Only the
addresses its examples actually exercise are defined below. Anything else
(torque enable, angle limits, ...) needs that model's memory table.
"""

HEADER = b'\xff\xff'
BROADCAST_ID = 0xFE   # all servos act; only PING replies (and not on a multi-servo bus)
MAX_ID = 0xFD         # ids 0x00..0xFD are addressable

# Two-byte value order; see the module docstring.
BYTE_ORDER_LOW_FIRST = 'low'    # magnetic encoder series (STS)
BYTE_ORDER_HIGH_FIRST = 'high'  # potentiometer series (SCS)


class Instruction:
    """Instruction bytes (manual section 1.3)."""
    PING = 0x01         # param len 0
    READ = 0x02         # param len 2   (addr, count)
    WRITE = 0x03        # param len >=1 (addr, data...)
    REG_WRITE = 0x04    # param len >=2 (addr, data...), applied on ACTION
    ACTION = 0x05       # param len 0
    RESET = 0x06        # param len 0   (control table -> factory values)
    SYNC_READ = 0x82    # param len >=3 (addr, count, id...)
    SYNC_WRITE = 0x83   # param len >=2 (addr, len, [id, data...]...)


class Register:
    """Register addresses the manual's examples document.

    These are the only addresses the protocol manual pins down; the rest of the
    control table is model-specific. Do not guess the others -- read them from
    the model's memory table.
    """
    ID = 0x05                 # example 3: write 1 to address 5 sets the id
    GOAL_POSITION = 0x2A      # example 4/7: 6-byte block = position, time, speed
    PRESENT_POSITION = 0x38   # example 2/8: 8-byte block = position, speed,
                              #              load, voltage, temperature


def checksum(body):
    """Checksum over ID, Length, Instruction and parameters (manual 1.1)."""
    return (~sum(body)) & 0xFF


def build_packet(dev_id, instruction, params=()):
    """Build an instruction packet. Returns a bytes object."""
    params = list(params)
    body = [dev_id, len(params) + 2, instruction] + params
    body.append(checksum(body))
    return bytes([0xFF, 0xFF] + body)


def pack_u16(value, byte_order=BYTE_ORDER_LOW_FIRST):
    """Split a 16-bit value into the two parameter bytes, in bus order."""
    value &= 0xFFFF
    lo, hi = value & 0xFF, (value >> 8) & 0xFF
    return [lo, hi] if byte_order == BYTE_ORDER_LOW_FIRST else [hi, lo]


def unpack_u16(data, byte_order=BYTE_ORDER_LOW_FIRST):
    """Combine two parameter bytes, in bus order, into a 16-bit value."""
    a, b = data[0], data[1]
    return (a | (b << 8)) if byte_order == BYTE_ORDER_LOW_FIRST else ((a << 8) | b)


def build_read(dev_id, addr, count):
    """READ DATA: fetch `count` bytes starting at `addr`."""
    return build_packet(dev_id, Instruction.READ, [addr, count])


def build_write(dev_id, addr, data):
    """WRITE DATA: write `data` (an iterable of bytes) starting at `addr`."""
    return build_packet(dev_id, Instruction.WRITE, [addr] + list(data))


def build_reg_write(dev_id, addr, data):
    """REG WRITE: stage a write; applied when ACTION arrives."""
    return build_packet(dev_id, Instruction.REG_WRITE, [addr] + list(data))


def build_action(dev_id=BROADCAST_ID):
    """ACTION: apply staged REG WRITEs. Broadcast so servos move together."""
    return build_packet(dev_id, Instruction.ACTION)


def build_ping(dev_id):
    return build_packet(dev_id, Instruction.PING)


def build_reset(dev_id):
    """RESET: restore the control table to factory values."""
    return build_packet(dev_id, Instruction.RESET)


def build_sync_write(addr, id_data_pairs):
    """SYNC WRITE: one packet writing the same register block on many servos.

    id_data_pairs is [(id, data), ...]; every data block must be the same
    length (a protocol requirement, manual 1.3.6). No response is returned,
    since the packet is addressed to the broadcast id.
    """
    pairs = list(id_data_pairs)
    if not pairs:
        raise ValueError("sync write needs at least one servo")
    data_len = len(pairs[0][1])
    params = [addr, data_len]
    for dev_id, data in pairs:
        if len(data) != data_len:
            raise ValueError(
                "sync write data blocks must all be {} bytes; id {} has {}"
                .format(data_len, dev_id, len(data)))
        params.append(dev_id)
        params.extend(data)
    return build_packet(BROADCAST_ID, Instruction.SYNC_WRITE, params)


def build_sync_read(addr, count, ids):
    """SYNC READ: ask several servos for the same register block.

    Each servo replies with its own ordinary response packet, in the id order
    given here (manual 1.3.7). Not all serial bus servos support it.
    """
    ids = list(ids)
    if not ids:
        raise ValueError("sync read needs at least one servo")
    return build_packet(BROADCAST_ID, Instruction.SYNC_READ,
                        [addr, count] + ids)


def parse_response(data):
    """Parse a response packet (manual 1.2).

    Returns (dev_id, error, params) or raises ValueError. `error` is 0 when the
    servo reports no problem; the meaning of individual bits is model-specific.
    """
    if len(data) < 6:
        raise ValueError("response too short: {} bytes".format(len(data)))
    if data[0] != 0xFF or data[1] != 0xFF:
        raise ValueError("bad header")
    dev_id, length, error = data[2], data[3], data[4]
    total = 4 + length           # header(2) + id + length + (length) more bytes
    if len(data) < total:
        raise ValueError(
            "truncated response: need {} bytes, have {}".format(total, len(data)))
    body = list(data[2:total - 1])
    if checksum(body) != data[total - 1]:
        raise ValueError("checksum mismatch")
    return dev_id, error, bytes(data[5:total - 1])
