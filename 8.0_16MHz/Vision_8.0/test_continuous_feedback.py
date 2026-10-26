"""Regression: missing/busy/fault feedback must not block a new left request."""
import unittest
from motion_feedback import MotionFeedback
from lane_protocol import encode_angle

class ContinuousFeedbackTest(unittest.TestCase):
    def test_left_command_survives_feedback_wait_and_fault(self):
        for fault in (False,True):
            f=MotionFeedback(enabled=True,continuous_control=True)
            f.fault=fault;f.pending=10;f.sent_at=0;f.ready=False
            lane=dict(valid=True,state='LEFT')
            command=dict(valid=True,active=True,direction=-1,angle=0,status=3,reason='POSITION')
            got_lane,got=f.gate(lane,command,20)
            self.assertEqual(got,command)
            self.assertEqual(got_lane['state'],'LEFT')
            self.assertEqual(encode_angle(got,11)[3],3)

    def test_no_pending_transaction_blocks_following_direction(self):
        f=MotionFeedback(enabled=True,continuous_control=True)
        f.sent(dict(active=True),1,0)
        self.assertIsNone(f.pending)
        command=dict(active=True,status=4,direction=1,angle=0)
        self.assertEqual(f.gate(dict(valid=True),command,1)[1]['status'],4)

if __name__=='__main__':unittest.main()
