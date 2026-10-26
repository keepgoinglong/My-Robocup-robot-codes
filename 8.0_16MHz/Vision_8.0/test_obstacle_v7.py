import struct
import unittest

from lane_protocol import crc16, encode_obstacle
from obstacle_tracker import ObstacleTracker


class ObstacleV7Tests(unittest.TestCase):
    def test_protocol_and_angle_sign(self):
        packet=encode_obstacle(dict(present=True,angle=-12.3),17)
        self.assertEqual(packet[:4],bytes((0xaa,0x58,17,1)))
        self.assertEqual(struct.unpack('<h',packet[4:6])[0],-123)
        self.assertEqual(struct.unpack('<H',packet[6:])[0],crc16(packet[:6]))

    def test_three_frame_confirmation_and_clear(self):
        tracker=ObstacleTracker(confirm=3,clear=3)
        hit=dict(angle=10.0,line=(0,0,300,50),score=300)
        self.assertFalse(tracker.update(hit)['present'])
        self.assertFalse(tracker.update(hit)['present'])
        self.assertTrue(tracker.update(hit)['present'])
        self.assertTrue(tracker.update(None)['present'])
        self.assertTrue(tracker.update(None)['present'])
        self.assertFalse(tracker.update(None)['present'])


if __name__=='__main__':unittest.main()
