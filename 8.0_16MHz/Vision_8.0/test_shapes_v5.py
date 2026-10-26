"""图形拒识、空间连续确认、冷却和串口报文回归。"""
import unittest
import cv2
import numpy as np
from shape_tracker import CardRecognizer,ShapeEvents,candidate_ok
from replay_shapes import candidates
from lane_protocol import encode,crc16

class Shape5Tests(unittest.TestCase):
    def setUp(self):self.recognizer=CardRecognizer()
    def detect(self,g):return self.recognizer.detect(lambda x,y:int(g[y,x]),candidates(g))
    def test_blank_and_empty_frame_are_not_shapes(self):
        for gray in (40,90,150,230):
            g=np.full((480,640),gray,np.uint8)
            self.assertIsNone(self.detect(g))
            cv2.rectangle(g,(240,170),(380,310),20,4)
            self.assertIsNone(self.detect(g))
    def test_filled_card_and_lane_are_not_shapes(self):
        g=np.full((480,640),190,np.uint8)
        cv2.rectangle(g,(240,170),(380,310),30,-1)
        self.assertIsNone(self.detect(g))
        g[:]=190
        cv2.line(g,(50,440),(170,126),30,9)
        cv2.line(g,(570,440),(460,126),30,9)
        self.assertIsNone(self.detect(g))
    def test_noise_is_not_shape(self):
        g=np.random.default_rng(5).integers(40,200,(480,640),dtype=np.uint8)
        self.assertIsNone(self.detect(g))
    def test_candidates_are_allowed_across_the_full_frame(self):
        self.assertTrue(candidate_ok((0,0,80,80)))
        self.assertTrue(candidate_ok((560,400,80,80)))
    def test_each_shape_is_sent_only_once_per_run(self):
        e=ShapeEvents();d=dict(shape=2,box=(200,180,120,120))
        self.assertEqual(e.update(d,0),0)
        self.assertEqual(e.update(d,.1),2)
        self.assertEqual(e.update(d,12),0) # 尚未真正发送，不能重复入队
        e.mark_sent(2,12)
        self.assertEqual(e.update(d,12.1),0)
        self.assertEqual(e.update(d,21.9),0)
        self.assertEqual(e.update(d,22.1),0)
        other=dict(shape=3,box=(200,180,120,120))
        self.assertEqual(e.update(other,22.2),0)
        self.assertEqual(e.update(other,22.3),3)
    def test_missing_or_different_card_breaks_confirmation(self):
        e=ShapeEvents();d=dict(shape=1,box=(200,180,80,80))
        e.update(d,0);e.update(None,.1)
        self.assertEqual(e.update(d,.2),0)
        far=dict(shape=1,box=(420,180,80,80))
        self.assertEqual(e.update(far,.3),0)
        self.assertEqual(e.update(far,.4),1)
    def test_shape_packet_preserves_id_even_when_lane_lost(self):
        packet=encode(dict(valid=False,stop=True,state='LOST'),255,6)
        self.assertEqual(packet[:5],bytes((170,85,255,0,6)))
        self.assertEqual(int.from_bytes(packet[5:],'little'),crc16(packet[:5]))

if __name__=='__main__':unittest.main()
