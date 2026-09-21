import struct
import ax

try:
    import hiwonder_packet as hw_pkt
except ImportError:
    hw_pkt = None


# Servo kind (a pos/center_lookup key from poses.py) -> wire protocol.
# The single source of truth for which protocol a servo speaks: ServoGroup
# dispatches on it and numa.py picks the bus with it. Adding a servo type means
# adding it here AND giving it a branch in each ServoGroup method below; an
# unregistered kind raises at construction rather than being silently misrouted.
PROTOCOL_AX = 'ax'
PROTOCOL_HW = 'hw'
KIND_PROTOCOL = {
    'ax12': PROTOCOL_AX,
    'ax12a': PROTOCOL_AX,
    'hx-35hm': PROTOCOL_HW,
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

    def _ax_bus(self):
        for s in self.servos:
            if s.protocol == PROTOCOL_AX:
                return s.bus
        return None

    def write_positions(self, positions, move_ms=500, indices=None,
                        include_hw=True):
        """Send goal positions to servos.

        AX servos are batched into a single sync_write. HiWonder servos
        receive individual MOVE_TIME_WRITE commands. move_ms sets the
        travel duration for HiWonder servos and is ignored for AX.
        indices, if given, selects a subset of servos (matched to positions).

        include_hw=False skips the HiWonder servos entirely, writing only the
        AX batch. Each HiWonder command is its own packet (~1.1 ms), so a
        caller on a tight loop can write them every Nth pass with a travel
        time that spans the gap and let the servo interpolate between updates.
        """
        if indices is not None:
            pairs = [(self.servos[i], pos) for i, pos in zip(indices, positions)]
        else:
            pairs = list(zip(self.servos, positions))
        ax_ids, ax_vals = [], []
        t = int(move_ms)
        for servo, pos in pairs:
            p = int(pos)
            if servo.protocol == PROTOCOL_AX:
                ax_ids.append(servo.id)
                ax_vals.append(struct.pack('<H', p))
            elif servo.protocol == PROTOCOL_HW:
                if include_hw:
                    data = bytearray([p & 0xff, (p >> 8) & 0xff,
                                       t & 0xff, (t >> 8) & 0xff])
                    servo.bus.write(servo.id, hw_pkt.Command.MOVE_TIME_WRITE, data)
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.GOAL_POSITION, ax_vals)

    def write_torque(self, enable, indices=None):
        """Enable or disable torque, optionally on a subset of servos."""
        selected = self._select(indices)
        ax_ids = []
        val = 1 if enable else 0
        for _, servo in selected:
            if servo.protocol == PROTOCOL_AX:
                ax_ids.append(servo.id)
            elif servo.protocol == PROTOCOL_HW:
                servo.bus.write(servo.id, hw_pkt.Command.LOAD_OR_UNLOAD_WRITE,
                                bytearray([val]))
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.TORQUE_ENABLE,
                                      [bytearray([val]) for _ in ax_ids])

    def write_speed(self, speed, indices=None):
        """Set moving speed. AX only — HiWonder speed is set via move_ms in write_positions."""
        selected = self._select(indices)
        ax_ids = [s.id for _, s in selected if s.protocol == PROTOCOL_AX]
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.MOVING_SPEED,
                                      [struct.pack('<H', speed) for _ in ax_ids])

    def write_angle_limits(self, limits, indices=None):
        """Set angle limits. limits is a list of [min, max] pairs, one per selected servo.

        AX: two sync_writes (CW then CCW). HiWonder: individual ANGLE_LIMIT_WRITE.
        """
        selected = self._select(indices)
        ax_ids, ax_cw, ax_ccw = [], [], []
        for (_, servo), lim in zip(selected, limits):
            if servo.protocol == PROTOCOL_AX:
                ax_ids.append(servo.id)
                ax_cw.append(struct.pack('<H', lim[0]))
                ax_ccw.append(struct.pack('<H', lim[1]))
            elif servo.protocol == PROTOCOL_HW:
                data = bytearray([lim[0] & 0xff, (lim[0] >> 8) & 0xff,
                                   lim[1] & 0xff, (lim[1] >> 8) & 0xff])
                servo.bus.write(servo.id, hw_pkt.Command.ANGLE_LIMIT_WRITE, data)
        if ax_ids:
            bus = self._ax_bus()
            bus.sync_write(ax_ids, ax.CW_ANGLE_LIMIT_L, ax_cw)
            bus.sync_write(ax_ids, ax.CCW_ANGLE_LIMIT_L, ax_ccw)

    def write_return_level(self, level):
        """Set return level. AX only."""
        ax_ids = [s.id for s in self.servos if s.protocol == PROTOCOL_AX]
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.RETURN_LEVEL,
                                      [bytearray([level]) for _ in ax_ids])
