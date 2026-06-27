from log import log
import hiwonder_packet as packet
from dump_mem import dump_mem


class BusError(Exception):
    """Exception which is raised when a non-successful status packet is received."""

    def __init__(self, error_code, *args, **kwargs):
        super(BusError, self).__init__(self, *args, **kwargs)
        self.error_code = error_code

    def get_error_code(self):
        """Retrieves the error code associated with the exception."""
        return self.error_code

    def __str__(self):
        return "Rcvd Status: " + str(packet.ErrorCode(self.error_code))


class Bus:
    """Knows the commands used to talk to HiWonder serial bus servos.

    Adapted from a Dynamixel/bioloid bus driver. Notable HiWonder differences:
    - Packets are framed with a 0x55 0x55 header (not 0xFF 0xFF).
    - Writes are not acknowledged, so write() does not read a status packet.
    - Status packets echo the request's command byte rather than returning
      Dynamixel-style error/status bits.
    - There is no sync-write equivalent.
    """

    SHOW_NONE       = 0
    SHOW_COMMANDS   = (1 << 0)
    SHOW_PACKETS    = (1 << 1)

    def __init__(self, serial_port, show=SHOW_NONE):
        self.serial_port = serial_port
        self.show = show

    def fill_and_write_packet(self, dev_id, cmd, data=None):
        """Allocates and fills a packet. data should be a bytearray of data
        to include in the packet, or None if no data should be included.
        """
        packet_len = 6
        if data is not None:
            packet_len += len(data)
        pkt_bytes = bytearray(packet_len)
        pkt_bytes[0] = 0x55
        pkt_bytes[1] = 0x55
        pkt_bytes[2] = dev_id
        pkt_bytes[3] = 3       # base length for len + id + cmd
        pkt_bytes[4] = cmd
        if data is not None:
            pkt_bytes[3] += len(data)  # update the length
            pkt_bytes[5:packet_len - 1] = data
        # Checksum: bitwise-NOT of the sum of the id byte through the last data byte
        pkt_bytes[-1] = ~sum(pkt_bytes[2:-1]) & 0xff
        if self.show & Bus.SHOW_PACKETS:
            dump_mem(pkt_bytes, prefix='  W', show_ascii=True, log=log)
        self.serial_port.write_packet(pkt_bytes)

    def read(self, dev_id, cmd):
        """Sends a READ request and returns the data read.

        Raises a BusError if any errors occur.
        """
        self.send_read(dev_id, cmd)
        pkt = self.read_status_packet()
        return pkt.params()

    def read_status_packet(self):
        """Reads a status packet and returns it.

        Raises a BusError if an error occurs.
        """
        pkt = packet.Packet()
        while True:
            byte = self.serial_port.read_byte()
            if byte is None:
                raise BusError(packet.ErrorCode.TIMEOUT)
            err = pkt.process_byte(byte)
            if err != packet.ErrorCode.NOT_DONE:
                break
        if err != packet.ErrorCode.NONE:
            raise BusError(err)
        if self.show & Bus.SHOW_COMMANDS:
            log('Rcvd Status: {}'.format(packet.ErrorCode(err)))
        if self.show & Bus.SHOW_PACKETS:
            dump_mem(pkt.pkt_bytes, prefix='  R', show_ascii=True, log=log)
        # HiWonder echoes the request's command byte in the status packet rather
        # than returning error bits, so there is no error code to validate here.
        return pkt

    def ping(self, dev_id):
        """Returns True if a servo with dev_id responds.

        HiWonder has no dedicated PING command; ID_READ (14) is used instead.
        A servo replies to ID_READ with its own id and, uniquely among the read
        commands, answers even when addressed by the broadcast id.
        """
        try:
            self.send_read(dev_id, packet.Command.ID_READ)
            self.read_status_packet()
        except BusError:
            return False
        return True

    def scan(self, start_id=0, num_ids=32, dev_found=None, dev_missing=None):
        """Scans the bus, calling dev_found(self, dev_id) for each device that
        responds and dev_missing(self, dev_id) for each that does not.

        Returns True if any devices were found.
        """
        end_id = start_id + num_ids - 1
        if end_id >= packet.Id.BROADCAST:
            end_id = packet.Id.BROADCAST - 1
        some_dev_found = False
        for dev_id in range(start_id, end_id + 1):
            if self.ping(dev_id):
                some_dev_found = True
                if dev_found:
                    dev_found(self, dev_id)
            elif dev_missing:
                dev_missing(self, dev_id)
        return some_dev_found

    def send_read(self, dev_id, cmd):
        """Sends a READ command request to the device."""
        if self.show & Bus.SHOW_COMMANDS:
            log('Sending READ to ID {} CMD 0x{:02x}'.format(dev_id, cmd))
        self.fill_and_write_packet(dev_id, cmd)

    def send_write(self, dev_id, cmd, data):
        if self.show & Bus.SHOW_COMMANDS:
            log('Sending {} to ID {} data len {}'.format(cmd, dev_id, len(data)))
        self.fill_and_write_packet(dev_id, cmd, data)

    def write(self, dev_id, cmd, data):
        self.send_write(dev_id, cmd, data)
        # HiWonder does not acknowledge writes (reading a status packet here just
        # times out with a BusError), so there is nothing to read back.
        return packet.ErrorCode.NONE

    def action(self):
        """Broadcasts MOVE_START, triggering moves that were primed on each
        servo with MOVE_TIME_WAIT_WRITE so they begin simultaneously.

        This is HiWonder's nearest analog to the Dynamixel ACTION broadcast;
        unlike ACTION it only triggers pending moves, not arbitrary deferred
        writes (HiWonder has no general REG_WRITE).
        """
        if self.show & Bus.SHOW_COMMANDS:
            log('Broadcasting MOVE_START')
        self.fill_and_write_packet(packet.Id.BROADCAST, packet.Command.MOVE_START)
