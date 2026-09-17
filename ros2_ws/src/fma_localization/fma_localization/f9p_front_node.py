import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64
import serial


UBX_SYNC = b'\xb5\x62'
UBX_CLASS_NAV = 0x01
UBX_ID_RELPOSNED = 0x3C


def verify_nmea_checksum(sentence):
    if not sentence.startswith('$') or '*' not in sentence:
        return False

    body, checksum_text = sentence[1:].rsplit('*', 1)
    checksum = 0
    for char in body:
        checksum ^= ord(char)

    try:
        expected = int(checksum_text[:2], 16)
    except (ValueError, IndexError):
        return False

    return checksum == expected


def nmea_coordinate(value, hemisphere):
    raw = float(value)
    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    result = degrees + minutes / 60.0

    if hemisphere in ('S', 'W'):
        result = -result
    elif hemisphere not in ('N', 'E'):
        raise ValueError('invalid hemisphere')

    return result


def parse_gga(sentence):
    if not verify_nmea_checksum(sentence):
        raise ValueError('invalid NMEA checksum')

    fields = sentence.split(',')

    if fields[0] not in ('$GNGGA', '$GPGGA'):
        raise ValueError('not GGA')

    if len(fields) < 15:
        raise ValueError('short GGA')

    quality = int(fields[6] or '0')
    if quality <= 0:
        raise ValueError('no GNSS fix')

    latitude = nmea_coordinate(fields[2], fields[3])
    longitude = nmea_coordinate(fields[4], fields[5])

    altitude_msl = float(fields[9]) if fields[9] else math.nan
    geoid_sep = float(fields[11]) if fields[11] else math.nan

    if math.isfinite(altitude_msl) and math.isfinite(geoid_sep):
        altitude = altitude_msl + geoid_sep
    else:
        altitude = altitude_msl

    return latitude, longitude, altitude, quality


def ubx_checksum(data):
    ck_a = 0
    ck_b = 0

    for value in data:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF

    return ck_a, ck_b


def parse_relposned(payload):
    if len(payload) < 64:
        raise ValueError('short NAV-RELPOSNED payload')

    rel_length_cm = struct.unpack_from('<i', payload, 20)[0]
    rel_heading_raw = struct.unpack_from('<i', payload, 24)[0]
    rel_length_hp = struct.unpack_from('<b', payload, 35)[0]
    acc_length_raw = struct.unpack_from('<I', payload, 48)[0]
    acc_heading_raw = struct.unpack_from('<I', payload, 52)[0]
    flags = struct.unpack_from('<I', payload, 60)[0]

    rel_pos_valid = bool(flags & (1 << 2))
    carr_soln = (flags >> 3) & 0x03
    heading_valid = bool(flags & (1 << 8))

    # cm + 0.1 mm high-precision component
    baseline_m = rel_length_cm * 0.01 + rel_length_hp * 0.0001
    heading_deg = (rel_heading_raw * 1e-5) % 360.0

    # UBX accuracies: length 0.1 mm, heading 1e-5 deg.
    baseline_accuracy_m = acc_length_raw * 0.0001
    heading_accuracy_deg = acc_heading_raw * 1e-5

    return {
        'heading_deg': heading_deg,
        'baseline_m': baseline_m,
        'baseline_accuracy_m': baseline_accuracy_m,
        'heading_accuracy_deg': heading_accuracy_deg,
        'rel_pos_valid': rel_pos_valid,
        'heading_valid': heading_valid,
        'rtk_fixed': carr_soln == 2,
        'flags': flags,
    }


class F9PFrontNode(Node):
    def __init__(self):
        super().__init__('f9p_front')

        self.declare_parameter(
            'front_port',
            '/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'gps_front')
        self.declare_parameter('baseline_m', 0.92)
        self.declare_parameter('baseline_tolerance_m', 0.05)
        self.declare_parameter('max_heading_accuracy_deg', 1.0)

        port = self.get_parameter('front_port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.expected_baseline_m = float(
            self.get_parameter('baseline_m').value)
        self.baseline_tolerance_m = float(
            self.get_parameter('baseline_tolerance_m').value)
        self.max_heading_accuracy_deg = float(
            self.get_parameter('max_heading_accuracy_deg').value)

        self.fix_publisher = self.create_publisher(
            NavSatFix, '/gps/fix', qos_profile_sensor_data)

        self.heading_publisher = self.create_publisher(
            Float64, '/gps/heading', qos_profile_sensor_data)

        self.fixed_publisher = self.create_publisher(
            Bool, '/gps/rtk_fixed', qos_profile_sensor_data)
        self.baseline_publisher = self.create_publisher(
            Float64, '/gps/baseline_m', qos_profile_sensor_data)
        self.heading_accuracy_publisher = self.create_publisher(
            Float64, '/gps/heading_accuracy_deg', qos_profile_sensor_data)
        self.healthy_publisher = self.create_publisher(
            Bool, '/gps/healthy', qos_profile_sensor_data)

        self.buffer = bytearray()

        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0,
        )

        self.get_logger().info(f'Front F9P opened: {port}')

        self.timer = self.create_timer(0.02, self.poll)

    def poll(self):
        try:
            waiting = self.serial.in_waiting
            if waiting <= 0:
                return

            self.buffer.extend(self.serial.read(waiting))
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(f'GPS serial error: {error}')
            return

        self.process_buffer()

    def process_buffer(self):
        while self.buffer:
            nmea_pos = self.buffer.find(b'$')
            ubx_pos = self.buffer.find(UBX_SYNC)

            candidates = [p for p in (nmea_pos, ubx_pos) if p >= 0]

            if not candidates:
                if len(self.buffer) > 8192:
                    del self.buffer[:-64]
                return

            start = min(candidates)

            if start > 0:
                del self.buffer[:start]

            if self.buffer.startswith(b'$'):
                if not self.process_nmea():
                    return
                continue

            if self.buffer.startswith(UBX_SYNC):
                if not self.process_ubx():
                    return
                continue

            del self.buffer[0]

    def process_nmea(self):
        end = self.buffer.find(b'\n')

        if end < 0:
            return False

        raw = bytes(self.buffer[:end + 1])
        del self.buffer[:end + 1]

        try:
            sentence = raw.decode('ascii').strip()
        except UnicodeDecodeError:
            return True

        if not (
            sentence.startswith('$GNGGA,') or
            sentence.startswith('$GPGGA,')
        ):
            return True

        try:
            latitude, longitude, altitude, _quality = parse_gga(sentence)
        except ValueError:
            return True

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_publisher.publish(msg)
        return True

    def process_ubx(self):
        if len(self.buffer) < 6:
            return False

        msg_class = self.buffer[2]
        msg_id = self.buffer[3]
        payload_length = struct.unpack_from('<H', self.buffer, 4)[0]

        if payload_length > 4096:
            del self.buffer[0]
            return True

        packet_length = 6 + payload_length + 2

        if len(self.buffer) < packet_length:
            return False

        packet = bytes(self.buffer[:packet_length])

        expected_a, expected_b = ubx_checksum(
            packet[2:6 + payload_length])

        if packet[-2] != expected_a or packet[-1] != expected_b:
            del self.buffer[0]
            return True

        del self.buffer[:packet_length]

        if msg_class != UBX_CLASS_NAV or msg_id != UBX_ID_RELPOSNED:
            return True

        payload = packet[6:6 + payload_length]

        try:
            status = parse_relposned(payload)
        except ValueError:
            return True

        fixed_msg = Bool()
        fixed_msg.data = status['rtk_fixed']
        self.fixed_publisher.publish(fixed_msg)

        baseline_msg = Float64()
        baseline_msg.data = status['baseline_m']
        self.baseline_publisher.publish(baseline_msg)

        accuracy_msg = Float64()
        accuracy_msg.data = status['heading_accuracy_deg']
        self.heading_accuracy_publisher.publish(accuracy_msg)

        healthy = (
            status['rel_pos_valid']
            and status['heading_valid']
            and status['rtk_fixed']
            and abs(
                status['baseline_m'] - self.expected_baseline_m
            ) <= self.baseline_tolerance_m
            and status['heading_accuracy_deg']
            <= self.max_heading_accuracy_deg
        )

        healthy_msg = Bool()
        healthy_msg.data = healthy
        self.healthy_publisher.publish(healthy_msg)

        if status['rel_pos_valid'] and status['heading_valid']:
            heading_msg = Float64()
            heading_msg.data = status['heading_deg']
            self.heading_publisher.publish(heading_msg)

        return True

    def destroy_node(self):
        try:
            self.serial.close()
        except (serial.SerialException, OSError):
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = F9PFrontNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError) as error:
        rclpy.logging.get_logger('f9p_front').error(
            f'Unable to open front F9P: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
