import unittest,struct
from angle_tracker import AngleTracker
from lane_protocol import encode_angle,crc16
class AngleTests(unittest.TestCase):
    def test_threshold_confirm_hysteresis_reverse_and_loss(self):
        tr=AngleTracker(tau=.00001)
        for i in range(4):self.assertFalse(tr.update(8,i*.1)['active'])
        self.assertFalse(tr.update(9,.5)['active'])
        self.assertFalse(tr.update(9,.6)['active'])
        self.assertTrue(tr.update(9,.71)['active'])
        self.assertTrue(tr.update(5,.85)['active'])
        self.assertFalse(tr.update(4,1.0)['active'])
        self.assertFalse(tr.update(-10,1.15)['active'])
        self.assertTrue(tr.update(-10,1.36)['active'])
        self.assertFalse(tr.update(None,1.4)['valid'])
        self.assertFalse(tr.update(-10,1.5)['active'])
    def test_gap_cancels_confirmation(self):
        tr=AngleTracker()
        tr.update(12,0)
        self.assertFalse(tr.update(12,.3)['active'])
    def test_packet_sign_status_crc(self):
        for valid,active,angle,status in [(True,True,-12.3,2),(True,False,5,1),(False,False,0,0)]:
            p=encode_angle(dict(valid=valid,active=active,angle=angle),256)
            self.assertEqual(len(p),8)
            self.assertEqual(p[:4],bytes([170,86,0,status]))
            self.assertEqual(struct.unpack('<h',p[4:6])[0],-123 if active else 0)
            self.assertEqual(struct.unpack('<H',p[6:])[0],crc16(p[:6]))
