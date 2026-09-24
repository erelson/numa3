import struct
import ax

try:
    import hiwonder_packet as hw_pkt
except ImportError:
    hw_pkt = None

try:
    import feetech as ft
except ImportError:
    ft = None


# Servo kind (a pos/center_lookup key from poses.py) -> wire protocol.
# The single source of truth for which protocol a servo speaks: ServoGroup
# dispatches on it and numa.py picks the bus with it. Adding a servo type means
# adding it here AND giving it a branch in each ServoGroup method below; an
# unregistered kind raises at construction rather than being silently misrouted.
# Return-level encoding, identical for AX (RETURN_LEVEL, addr 16) and Feetech
# (RESPONSE_STATUS_LEVEL, addr 8), so the same value means the same thing on a
# shared bus. HiWonder has no such register: its protocol never acknowledges a
# write and always answers a read, which is READ_ONLY behavior, permanently.
RETURN_LEVEL_NONE = 0       # never reply
RETURN_LEVEL_READ_ONLY = 1  # reply to reads only (what this project uses)
RETURN_LEVEL_ALL = 2        # reply to every command, writes included

PROTOCOL_AX = 'ax'   # Dynamixel AX-12/AX-12A
PROTOCOL_HW = 'hw'   # HiWonder HX-35HM; one packet per servo, no sync-write
PROTOCOL_FT = 'ft'   # Feetech STS serial bus servos
KIND_PROTOCOL = {
    'ax12': PROTOCOL_AX,
    'ax12a': PROTOCOL_AX,
    'hx-35hm': PROTOCOL_HW,
    'sts3215': PROTOCOL_FT,
    'sts3235': PROTOCOL_FT,   # same control table and scale as the 3215
}


def protocol_for(kind):
    """Wire protocol for a servo kind. Raises ValueError on unknown kinds."""
    try:
        return KIND_PROTOCOL[kind]
    except KeyError:
        raise ValueError(
            "unknown servo kind {!r}; add it to servo_group.KIND_PROTOCOL "
            "(and give it a branch in the ServoGroup methods)".format(kind))


class Servo:
    """A single servo on a bus."""
    def __init__(self, servo_id, kind, bus):
        self.id = servo_id
        self.kind = kind
        self.protocol = protocol_for(kind)
        self.bus = bus


class ServoGroup:
    """Ordered collection of servos across one or more buses.

    Servo order matches the position list ordering used throughout the
    codebase (leg_ids order: coax x4, femur x4, tibia x4, foot x4).
    """

    def __init__(self, servos):
        self.servos = servos

    def _select(self, indices):
        """Return (index, servo) pairs for the given indices, or all if None."""
        if indices is None:
            return list(enumerate(self.servos))
        return [(i, self.servos[i]) for i in indices]

    def _bus_for_protocol(self, protocol):
        for s in self.servos:
            if s.protocol == protocol:
                return s.bus
        return None

    def _ax_bus(self):
        return self._bus_for_protocol(PROTOCOL_AX)

    def _ft_bus(self):
        return self._bus_for_protocol(PROTOCOL_FT)

    def _ft_set_lock(self, bus, ids, locked):
        """Feetech guards its EEPROM with a LOCK register.

        The angle limits and response level live in EEPROM, so LOCK must be
        cleared before writing them and is restored afterwards (the protocol
        manual notes an unlocked value is otherwise not retained / is exposed
        to accidental writes).
        """
        bus.sync_write(ids, ft.Register.LOCK,
                       [bytearray([1 if locked else 0]) for _ in ids])

    def write_positions(self, positions, move_ms=500, indices=None,
                        include_hw=True):
        """Send goal positions to servos.

        AX servos are batched into a single sync_write. HiWonder servos
        receive individual MOVE_TIME_WRITE commands. move_ms sets the
        travel duration for HiWonder servos and is ignored for AX.
        indices, if given, selects a subset of servos (matched to positions).

        Feetech servos get their own sync_write: the framing is Dynamixel
        compatible, but the goal-position register sits at a different address
        (42 vs 30), and a sync_write carries ONE address for every servo in the
        packet - so the two families need one batch each, even when they share
        a bus. Speed is preset via write_speed, as for AX.

        include_hw=False skips the HiWonder servos entirely, writing only the
        batched ones. Each HiWonder command is its own packet (~1.1 ms), so a
        caller on a tight loop can write them every Nth pass with a travel
        time that spans the gap and let the servo interpolate between updates.
        """
        if indices is not None:
            pairs = [(self.servos[i], pos) for i, pos in zip(indices, positions)]
        else:
            pairs = list(zip(self.servos, positions))
        ax_ids, ax_vals = [], []
        ft_ids, ft_vals = [], []
        t = int(move_ms)
        for servo, pos in pairs:
            p = int(pos)
            if servo.protocol == PROTOCOL_AX:
                ax_ids.append(servo.id)
                ax_vals.append(struct.pack('<H', p))
            elif servo.protocol == PROTOCOL_FT:
                # STS is little-endian like AX (12-bit magnetic encoder series)
                ft_ids.append(servo.id)
                ft_vals.append(struct.pack('<H', p))
            elif servo.protocol == PROTOCOL_HW:
                if include_hw:
                    data = bytearray([p & 0xff, (p >> 8) & 0xff,
                                       t & 0xff, (t >> 8) & 0xff])
                    servo.bus.write(servo.id, hw_pkt.Command.MOVE_TIME_WRITE, data)
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.GOAL_POSITION, ax_vals)
        if ft_ids:
            self._ft_bus().sync_write(ft_ids, ft.Register.GOAL_POSITION, ft_vals)

    def write_torque(self, enable, indices=None):
        """Enable or disable torque, optionally on a subset of servos."""
        selected = self._select(indices)
        ax_ids, ft_ids = [], []
        val = 1 if enable else 0
        for _, servo in selected:
            if servo.protocol == PROTOCOL_AX:
                ax_ids.append(servo.id)
            elif servo.protocol == PROTOCOL_FT:
                ft_ids.append(servo.id)
            elif servo.protocol == PROTOCOL_HW:
                servo.bus.write(servo.id, hw_pkt.Command.LOAD_OR_UNLOAD_WRITE,
                                bytearray([val]))
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.TORQUE_ENABLE,
                                      [bytearray([val]) for _ in ax_ids])
        if ft_ids:
            # RAM register, so no LOCK dance needed.
            self._ft_bus().sync_write(ft_ids, ft.Register.TORQUE_ENABLE,
                                      [bytearray([val]) for _ in ft_ids])

    def write_speed(self, speed, indices=None):
        """Set moving speed for AX and Feetech.

        HiWonder has no speed register; its speed comes from the move_ms travel
        time in write_positions, so HiWonder servos are skipped here.
        """
        selected = self._select(indices)
        ax_ids = [s.id for _, s in selected if s.protocol == PROTOCOL_AX]
        ft_ids = [s.id for _, s in selected if s.protocol == PROTOCOL_FT]
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.MOVING_SPEED,
                                      [struct.pack('<H', speed) for _ in ax_ids])
        if ft_ids:
            self._ft_bus().sync_write(ft_ids, ft.Register.GOAL_SPEED,
                                      [struct.pack('<H', speed) for _ in ft_ids])

    def write_angle_limits(self, limits, indices=None):
        """Set angle limits. limits is a list of [min, max] pairs, one per selected servo.

        AX: two sync_writes (CW then CCW). HiWonder: individual
        ANGLE_LIMIT_WRITE. Feetech: one sync_write, since MIN_ANGLE_LIMIT(9)
        and MAX_ANGLE_LIMIT(11) are adjacent 2-byte registers -- but they are
        EEPROM, so the write is bracketed by clearing and restoring LOCK.
        """
        selected = self._select(indices)
        ax_ids, ax_cw, ax_ccw = [], [], []
        ft_ids, ft_vals = [], []
        for (_, servo), lim in zip(selected, limits):
            if servo.protocol == PROTOCOL_AX:
                ax_ids.append(servo.id)
                ax_cw.append(struct.pack('<H', lim[0]))
                ax_ccw.append(struct.pack('<H', lim[1]))
            elif servo.protocol == PROTOCOL_FT:
                ft_ids.append(servo.id)
                ft_vals.append(struct.pack('<HH', lim[0], lim[1]))
            elif servo.protocol == PROTOCOL_HW:
                data = bytearray([lim[0] & 0xff, (lim[0] >> 8) & 0xff,
                                   lim[1] & 0xff, (lim[1] >> 8) & 0xff])
                servo.bus.write(servo.id, hw_pkt.Command.ANGLE_LIMIT_WRITE, data)
        if ax_ids:
            bus = self._ax_bus()
            bus.sync_write(ax_ids, ax.CW_ANGLE_LIMIT_L, ax_cw)
            bus.sync_write(ax_ids, ax.CCW_ANGLE_LIMIT_L, ax_ccw)
        if ft_ids:
            bus = self._ft_bus()
            self._ft_set_lock(bus, ft_ids, False)
            bus.sync_write(ft_ids, ft.Register.MIN_ANGLE_LIMIT, ft_vals)
            self._ft_set_lock(bus, ft_ids, True)

    def write_return_level(self, level):
        """Set when a servo replies (AX RETURN_LEVEL / Feetech RESPONSE_STATUS_LEVEL).

        Both use the same encoding (see RETURN_LEVEL_* above), so one value
        covers the whole group. This matters most on a SHARED bus: if AX and
        Feetech disagree about whether writes are acknowledged, the extra
        replies collide with what the bus expects and reads time out. Feetech's
        copy is in EEPROM, so it is bracketed by the LOCK dance.

        HiWonder needs no write: it is hard-wired to RETURN_LEVEL_READ_ONLY
        behavior. Asking for any other level therefore cannot be honored by
        those servos, so say so rather than leaving a silent asymmetry.
        """
        ax_ids = [s.id for s in self.servos if s.protocol == PROTOCOL_AX]
        ft_ids = [s.id for s in self.servos if s.protocol == PROTOCOL_FT]
        hw_ids = [s.id for s in self.servos if s.protocol == PROTOCOL_HW]
        if hw_ids and level != RETURN_LEVEL_READ_ONLY:
            print("warning: HiWonder servos", hw_ids,
                  "are fixed at return level", RETURN_LEVEL_READ_ONLY,
                  "- cannot set", level)
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.RETURN_LEVEL,
                                      [bytearray([level]) for _ in ax_ids])
        if ft_ids:
            bus = self._ft_bus()
            self._ft_set_lock(bus, ft_ids, False)
            bus.sync_write(ft_ids, ft.Register.RESPONSE_STATUS_LEVEL,
                           [bytearray([level]) for _ in ft_ids])
            self._ft_set_lock(bus, ft_ids, True)
