import struct
import ax

try:
    import hiwonder_packet as hw_pkt
except ImportError:
    hw_pkt = None


class Servo:
    """A single servo on a bus."""
    def __init__(self, servo_id, kind, bus):
        self.id = servo_id
        self.kind = kind   # 'ax' or 'hw'
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
            if s.kind == 'ax':
                return s.bus
        return None

    def write_positions(self, positions, move_ms=500):
        """Send goal positions to all servos.

        AX servos are batched into a single sync_write. HiWonder servos
        receive individual MOVE_TIME_WRITE commands. move_ms sets the
        travel duration for HiWonder servos and is ignored for AX.
        """
        ax_ids, ax_vals = [], []
        t = int(move_ms)
        for servo, pos in zip(self.servos, positions):
            p = int(pos)
            if servo.kind == 'ax':
                ax_ids.append(servo.id)
                ax_vals.append(struct.pack('<H', p))
            else:
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
            if servo.kind == 'ax':
                ax_ids.append(servo.id)
            else:
                servo.bus.write(servo.id, hw_pkt.Command.LOAD_OR_UNLOAD_WRITE,
                                bytearray([val]))
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.TORQUE_ENABLE,
                                      [bytearray([val]) for _ in ax_ids])

    def write_speed(self, speed, indices=None):
        """Set moving speed. AX only — HiWonder speed is set via move_ms in write_positions."""
        selected = self._select(indices)
        ax_ids = [s.id for _, s in selected if s.kind == 'ax']
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
            if servo.kind == 'ax':
                ax_ids.append(servo.id)
                ax_cw.append(struct.pack('<H', lim[0]))
                ax_ccw.append(struct.pack('<H', lim[1]))
            else:
                data = bytearray([lim[0] & 0xff, (lim[0] >> 8) & 0xff,
                                   lim[1] & 0xff, (lim[1] >> 8) & 0xff])
                servo.bus.write(servo.id, hw_pkt.Command.ANGLE_LIMIT_WRITE, data)
        if ax_ids:
            bus = self._ax_bus()
            bus.sync_write(ax_ids, ax.CW_ANGLE_LIMIT_L, ax_cw)
            bus.sync_write(ax_ids, ax.CCW_ANGLE_LIMIT_L, ax_ccw)

    def write_return_level(self, level):
        """Set return level. AX only."""
        ax_ids = [s.id for s in self.servos if s.kind == 'ax']
        if ax_ids:
            self._ax_bus().sync_write(ax_ids, ax.RETURN_LEVEL,
                                      [bytearray([level]) for _ in ax_ids])
