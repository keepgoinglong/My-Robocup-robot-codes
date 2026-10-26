import unittest
from boundary_tracker import BoundaryTracker,resolve_control
from angle_tracker import AngleTracker
from lane_protocol import encode,encode_angle,crc16
from motion_feedback import MotionFeedback

class BoundaryControlTests(unittest.TestCase):
    def track(self,runs,seconds=1):
        t=BoundaryTracker();a=AngleTracker()
        for n in range(int(seconds*25)+1):
            r=t.update(None,160,120,n*.04,run_provider=runs)
            r,c=resolve_control(r,a.update(t.raw_image_angle,n*.04))
        return t,r,c
    def test_dim_background_and_blank_frames(self):
        for bg,ink in [(130,65),(255,50)]:
            tr=BoundaryTracker()
            r=tr.update(lambda x,y:ink if 20<=x<25 or 135<=x<140 else bg,160,120,0)
            self.assertEqual(r['mode'],'BOTH')
        for value in (0,130,255):
            self.assertFalse(BoundaryTracker().update(lambda x,y:value,160,120,0)['valid'])
    def test_single_left_and_right_start_without_history(self):
        for x,direction,mode in [(55,1,'SINGLE_LEFT'),(105,-1,'SINGLE_RIGHT')]:
            t,r,c=self.track(lambda y:[x])
            self.assertEqual(r['mode'],mode)
            self.assertEqual(c['direction'],direction)
            self.assertEqual(c['reason'],'POSITION')
            self.assertIsNone(t.raw_image_angle)
            self.assertEqual(c['status'],4 if direction==1 else 3)
    def test_single_central_unknown_not_forced(self):
        t,r,c=self.track(lambda y:[80])
        self.assertEqual(r['mode'],'UNCERTAIN')
        self.assertFalse(c['active'])
    def test_far_single_edge_not_automatically_escape(self):
        t,r,c=self.track(lambda y:[10])
        self.assertEqual(r['mode'],'SINGLE_LEFT')
        self.assertFalse(c['active'])
    def test_short_marker_does_not_become_boundary(self):
        t,r,c=self.track(lambda y:[65,95] if 50<=y<=75 else [])
        self.assertFalse(r['valid'])
    def test_lateral_dashes_do_not_replace_two_boundaries(self):
        t,r,c=self.track(lambda y:[20,125]+([5] if y%3==0 else []))
        self.assertEqual(r['mode'],'BOTH')
        self.assertFalse(c['active'])
    def test_confirmed_curve_beats_opposite_position_shift_with_two_edges(self):
        lane=dict(valid=True,stop=False,state='RIGHT',mode='BOTH',position_direction=1,
                  curve_direction=-1,remembered_angle=-20)
        r,c=resolve_control(lane,dict(valid=True,active=True,direction=-1,angle=-20))
        self.assertEqual(r['state'],'LEFT')
        self.assertEqual(c['direction'],-1)
        self.assertEqual(encode(r,1)[3],2)
        self.assertEqual(c['reason'],'CURVE')

    def test_single_edge_position_escape_still_has_precedence(self):
        lane=dict(valid=True,stop=False,state='RIGHT',mode='SINGLE_LEFT',position_direction=1)
        r,c=resolve_control(lane,dict(valid=True,active=True,direction=-1,angle=-20))
        self.assertEqual(r['state'],'RIGHT')
        self.assertEqual(c['direction'],1)
        self.assertEqual(c['reason'],'POSITION')
    def test_loss_is_invalid_not_stale_angle(self):
        t=BoundaryTracker();t.update(None,160,120,0,run_provider=lambda y:[20,140])
        r=t.update(None,160,120,.1,run_provider=lambda y:[])
        self.assertFalse(r['valid']);self.assertIsNone(t.raw_image_angle)
    def test_gap_resets_single_confirmation(self):
        t=BoundaryTracker()
        for i in range(4):t.update(None,160,120,i*.1,run_provider=lambda y:[55])
        t.update(None,160,120,.35,run_provider=lambda y:[])
        r=t.update(None,160,120,.4,run_provider=lambda y:[55])
        self.assertEqual(r['position_direction'],0)
    def test_straight_perspective_heading_is_not_a_turn(self):
        a=AngleTracker()
        lane=dict(valid=True,stop=False,state='RIGHT',position_direction=1)
        for i in range(10):
            _,c=resolve_control(lane,a.update(-15,i*.04))
            self.assertEqual(c['direction'],1)
            a.reset()
        lane['position_direction']=0
        _,c=resolve_control(lane,a.update(-15,.4));self.assertFalse(c['active'])
        for i in range(11,18):_,c=resolve_control(lane,a.update(-15,i*.04))
        self.assertEqual(c['direction'],0)
        self.assertEqual(c['reason'],'STRAIGHT')

    def test_quadratic_left_curve_requires_confirmation(self):
        tracker=BoundaryTracker(counterclockwise=True)
        def curved(y):
            t=(y/120-.49)/.33
            return [int(-3*t*t+12*t+20),int(-3*t*t+12*t+140)]
        lane=None
        for i in range(8):
            lane=tracker.update(None,160,120,i*.04,run_provider=curved)
        self.assertEqual(lane['road_shape'],'LEFT_CURVE')
        self.assertEqual(lane['curve_direction'],-1)

    def test_diagonal_straight_boundaries_remain_straight(self):
        tracker=BoundaryTracker(counterclockwise=True)
        def straight(y):
            t=(y/120-.49)/.33
            return [int(12*t+20),int(12*t+140)]
        lane=None
        for i in range(12):
            lane=tracker.update(None,160,120,i*.04,run_provider=straight)
        lane,control=resolve_control(lane,AngleTracker().update(tracker.raw_image_angle,.5))
        self.assertEqual(lane['road_shape'],'STRAIGHT')
        self.assertEqual(control['direction'],0)

    def test_confirmed_left_is_held_across_single_right_boundary(self):
        tracker=BoundaryTracker(counterclockwise=True,turn_memory_seconds=7.0)
        tracker.remember_turn(-1,0.0)
        lane=tracker.update(None,160,120,.1,run_provider=lambda y:[130])
        lane,control=resolve_control(lane,dict(valid=False,active=False,direction=0,angle=0.0))
        self.assertEqual(lane['mode'],'SINGLE_RIGHT')
        self.assertEqual(lane['state'],'LEFT')
        self.assertEqual(control['direction'],-1)
        self.assertEqual(control['reason'],'TURN_MEMORY')

        lane=tracker.update(None,160,120,7.1,run_provider=lambda y:[130])
        lane,control=resolve_control(lane,dict(valid=False,active=False,direction=0,angle=0.0))
        self.assertEqual(lane['state'],'STRAIGHT')
        self.assertFalse(control['active'])

    def test_turn_memory_beats_inactive_single_edge_estimate(self):
        lane=dict(valid=True,stop=False,state='STRAIGHT',mode='SINGLE_RIGHT',
                  position_direction=0,remembered_direction=-1,remembered_angle=-18)
        angle=dict(valid=True,active=False,direction=0,angle=.2)
        lane,control=resolve_control(lane,angle)
        self.assertEqual(lane['state'],'LEFT')
        self.assertEqual(control['reason'],'TURN_MEMORY')
        self.assertEqual(control['angle'],-18)
        self.assertEqual(control['status'],3)
        packet=encode_angle(control,7)
        self.assertEqual(packet[3],3)
        self.assertEqual(int.from_bytes(packet[4:6],'little',signed=True),-180)

    def test_curved_single_right_edge_refreshes_left_with_angle(self):
        tracker=BoundaryTracker(counterclockwise=True)
        def curved(y):
            t=(y/120-.49)/.33
            return [int(-5*t*t+25*t+105)]
        lane=None
        for i in range(8):
            lane=tracker.update(None,160,120,i*.1,run_provider=curved)
        lane,control=resolve_control(lane,dict(valid=False,active=False,direction=0,angle=0.0))
        self.assertEqual(lane['mode'],'SINGLE_RIGHT')
        self.assertEqual(control['direction'],-1)
        self.assertEqual(control['angle'],lane['remembered_angle'])
        self.assertEqual(control['status'],3)
        packet=encode_angle(control,9)
        self.assertEqual(packet[3],3)
        self.assertEqual(int.from_bytes(packet[4:6],'little',signed=True),round(lane['remembered_angle']*10))

    def test_curved_single_left_edge_starts_left_turn(self):
        tracker=BoundaryTracker(counterclockwise=True)
        def curved(y):
            t=(y/120-.49)/.33
            return [int(-5*t*t+5*t+25)]
        lane=None
        for i in range(8):
            lane=tracker.update(None,160,120,i*.04,run_provider=curved)
        lane,control=resolve_control(lane,dict(valid=False,active=False,direction=0,angle=0.0))
        self.assertEqual(lane['mode'],'SINGLE_LEFT')
        self.assertEqual(control['direction'],-1)
        self.assertEqual(control['reason'],'CURVE')

    def test_straight_single_right_edge_does_not_fabricate_left_curve(self):
        tracker=BoundaryTracker(counterclockwise=True)
        def perspective_only(y):
            t=(y/120-.49)/.33
            return [int(6*t+120)]
        lane=None
        for i in range(8):
            lane=tracker.update(None,160,120,i*.1,run_provider=perspective_only)
        lane,control=resolve_control(lane,dict(valid=False,active=False,direction=0,angle=0.0))
        self.assertEqual(lane['mode'],'SINGLE_RIGHT')
        self.assertFalse(control['active'])

    def test_turn_memory_never_overrides_loss_or_unknown_edge(self):
        tracker=BoundaryTracker(counterclockwise=True)
        tracker.remember_turn(-1,0.0)
        for runs in (lambda y:[],lambda y:[80]):
            tracker=BoundaryTracker(counterclockwise=True)
            tracker.remember_turn(-1,0.0)
            lane=tracker.update(None,160,120,.1,run_provider=runs)
            lane,control=resolve_control(lane,dict(valid=False,active=False,direction=0,angle=0.0))
            self.assertFalse(control['active'])

    def test_confirmed_curve_is_held_during_short_loss(self):
        tracker=BoundaryTracker(counterclockwise=True,turn_memory_seconds=7.0)
        tracker.curve_active=True
        tracker.curve_time=0.0
        tracker.turn_memory_angle=-18.0
        lane=tracker.update(None,160,120,.1,run_provider=lambda y:[])
        lane,control=resolve_control(lane,dict(valid=False,active=False,direction=0,angle=0.0))
        self.assertTrue(lane['valid'])
        self.assertEqual(lane['state'],'LEFT')
        self.assertEqual(control['reason'],'LOSS_HOLD')
        self.assertEqual(control['angle'],-18.0)

class FeedbackTests(unittest.TestCase):
    def packet(self,seq,state):
        b=bytes([170,87,seq,state]);c=crc16(b);return b+bytes([c&255,c>>8])
    def test_split_noise_crc_wrong_seq_and_one_step_gate(self):
        f=MotionFeedback(True)
        p=self.packet(0,0);f.feed(b'noise'+p[:3],0);self.assertFalse(f.ready)
        f.feed(p[3:],.1);self.assertTrue(f.ready)
        control=dict(valid=True,active=True,direction=1,angle=9,status=2)
        lane=dict(valid=True,state='RIGHT')
        f.sent(control,42,.2)
        r,c=f.gate(lane,control,.3);self.assertEqual(c['status'],5);self.assertFalse(c['active'])
        f.feed(self.packet(41,2),.4);self.assertEqual(f.pending,42)
        bad=bytearray(self.packet(42,2));bad[-1]^=1
        f.feed(bad,.5);self.assertEqual(f.pending,42)
        f.feed(self.packet(42,1)+self.packet(42,2),.6)
        self.assertIsNone(f.pending);self.assertTrue(f.reset_heading)
        self.assertEqual(f.gate(lane,control,.7)[1]['status'],5)
        self.assertEqual(f.gate(lane,control,.8)[1]['status'],2)
        self.assertEqual(f.gate(lane,control,2)[1]['status'],2)
    def test_fresh_ready_clears_old_fault_when_idle(self):
        f=MotionFeedback(True);f.feed(self.packet(0,0),0)
        f.sent(dict(active=True),2,.1);f.feed(self.packet(2,3),.2);f.feed(self.packet(0,0),.3)
        self.assertTrue(f.ready)
        self.assertFalse(f.fault)
    def test_unmatched_feedback_does_not_replace_startup_handshake(self):
        f=MotionFeedback(True)
        f.feed(self.packet(77,1),.1)
        self.assertEqual(f.link_status(.2),'WAIT')
        self.assertEqual(f.valid_packets,1)
        self.assertIsNone(f.last_rx)
        lane,control=f.gate(dict(valid=True,state='LEFT'),dict(active=True),.2)
        self.assertEqual(control['status'],5)
        self.assertEqual(f.link_status(3.1),'FAULT')
    def test_one_shot_handshake_stays_ok_and_action_timeout_faults(self):
        f=MotionFeedback(True);f.feed(self.packet(0,0),.1)
        self.assertEqual(f.link_status(30),'OK')
        f.sent(dict(active=True),9,30)
        f.gate(dict(valid=True),dict(active=True),42.1)
        self.assertEqual(f.link_status(42.1),'FAULT')
    def test_disabled_feedback_never_fabricates_wait(self):
        f=MotionFeedback(False);r,c=f.gate(dict(valid=True),dict(active=True),99)
        self.assertTrue(c['active'])
