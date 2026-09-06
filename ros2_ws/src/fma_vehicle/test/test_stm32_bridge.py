"""Hardware-free protocol and safety tests; no physical serial port is opened."""
import math
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from fma_interfaces.msg import DriveCommand
from fma_vehicle.stm32_bridge_node import (
    STM32BridgeNode, parse_telemetry, speed_to_drive, steering_to_adc)


class ProtocolTests(unittest.TestCase):
    def test_parser(self):
        self.assertEqual(parse_telemetry('ENC=-123 SPEED=456mm/s STEER=2182 DRIVE=1'),
                         (-123, 0.456, 2182, 1))
        for line in ('FORWARD', 'ENC=1 SPEED=2 STEER=3 DRIVE=1',
                     'ENC=1 SPEED=2mm/s STEER=65536 DRIVE=1',
                     'ENC=1 SPEED=2mm/s STEER=3 DRIVE=3',
                     'ENC=2147483648 SPEED=2mm/s STEER=3 DRIVE=1'):
            with self.assertRaises(ValueError):
                parse_telemetry(line)

    def test_speed(self):
        for speed, expected in ((1., b'W'), (.2, b'W'), (-1., b'S'), (.01, b'X')):
            self.assertEqual(speed_to_drive(speed, .01), expected)
        for speed in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                speed_to_drive(speed, .01)

    def test_steering(self):
        # Synthetic endpoints for unit tests only, not vehicle calibration.
        def adc(angle, enabled=True, right=-.4, left=.6):
            return steering_to_adc(angle, enabled, right, left, 50, 2182, 4040, .001)
        self.assertEqual(adc(0, False, math.nan, math.nan), 2182)
        for args in ((.1, False), (0, True, math.nan, math.nan)):
            with self.assertRaises(ValueError):
                adc(*args)
        self.assertEqual(adc(-.2), 1116)
        self.assertEqual(adc(.3), 3111)
        self.assertEqual(adc(-2), 50)
        self.assertEqual(adc(2), 4040)


class NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.port = MagicMock()
        self.port.write.side_effect = lambda packet: len(packet)
        self.port.in_waiting = 0
        self.port.read.return_value = b''
        self.mock_serial = patch('fma_vehicle.stm32_bridge_node.serial.Serial',
                                 return_value=self.port)
        self.mock_serial.start()
        self.clock = patch('fma_vehicle.stm32_bridge_node.time.monotonic', return_value=100.)
        self.now = self.clock.start()
        self.node = STM32BridgeNode()

    def tearDown(self):
        self.node.destroy_node()
        self.clock.stop()
        self.mock_serial.stop()

    def command(self, speed=1., angle=0., emergency=False):
        msg = DriveCommand()
        msg.speed_mps, msg.steering_angle_rad, msg.emergency_stop = speed, angle, emergency
        self.node.on_command(msg)

    def advance(self, now):
        self.now.return_value = now
        self.node.poll()

    def test_tx_spacing_emergency_and_timeout(self):
        self.command()
        self.assertEqual(self.port.write.call_args.args, (b'W',))
        self.advance(100.01)
        self.assertEqual(self.port.write.call_count, 1)
        self.advance(100.03)
        self.assertEqual(self.port.write.call_args.args, (b'T2182',))
        self.command(emergency=True)
        self.advance(100.06)
        self.assertEqual(self.port.write.call_args.args, (b'X',))
        self.advance(100.3)
        self.assertEqual(self.port.write.call_args.args, (b'X',))
        self.command()
        self.advance(100.51)
        self.advance(100.81)
        self.assertEqual(self.port.write.call_args.args, (b'X',))
        self.assertTrue(all(c.args[0] in (b'W', b'X', b'T2182')
                            for c in self.port.write.call_args_list))

    def test_uncalibrated_stop(self):
        self.command(angle=.1)
        self.assertEqual(self.port.write.call_args.args, (b'X',))
        self.advance(100.11)
        self.assertEqual(self.port.write.call_count, 1)

    def test_receive_only_and_fragmented_rx(self):
        self.node.config['receive_only'] = True
        self.node.publisher = MagicMock()
        self.command()
        self.port.in_waiting = 100
        self.port.read.return_value = b'FORWARD\r\nENC=-123 SPEED=456mm/s STE'
        self.advance(100.1)
        self.node.publisher.publish.assert_not_called()
        self.port.read.return_value = b'ER=2182 DRIVE=1\r\n'
        self.advance(100.2)
        msg = self.node.publisher.publish.call_args.args[0]
        self.assertEqual(msg.encoder_count, -123)
        self.assertAlmostEqual(msg.speed_mps, .456, places=6)
        self.assertEqual((msg.steering_adc, msg.drive_state, msg.header.frame_id), (2182, 1, ''))
        self.node.close_serial()
        self.port.write.assert_not_called()

    def test_disconnect(self):
        self.port.read.side_effect = OSError('unplugged')
        self.advance(100.)
        self.assertFalse(self.node.connected)
        self.assertIsNone(self.node.serial)
        self.port.close.assert_called_once()

    def test_open_failure(self):
        self.node.destroy_node()
        with patch('fma_vehicle.stm32_bridge_node.serial.Serial', side_effect=OSError('missing')):
            self.node = STM32BridgeNode()
        self.assertFalse(self.node.connected)
        self.node.poll()


if __name__ == '__main__':
    unittest.main()
