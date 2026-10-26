import math
import struct
import unittest

from line_tracker import LaneTracker
from lane_protocol import crc16, encode


def synthetic(tracker, stamp, a=0, b=0, offset=0, missing=False, occluded=False):
    def runs(y):
        if missing:
            return []
        t = (y/120 - 0.49)/0.33
        center = 80 * (1 + a*t*t+b*t+offset)
        half = 40 + t*9
        if occluded and 45 < y < 65:
            return []
        return [center-half, center+half]
    return tracker.update(None, 160, 120, stamp, run_provider=runs)


class TrackerTests(unittest.TestCase):
    def test_straight_with_periodic_translation_and_roll(self):
        tracker = LaneTracker()
        for n in range(600):
            t = n/60
            result = synthetic(tracker, t, b=.22*math.sin(t*2*math.pi*2),
                               offset=.20*math.sin(t*2*math.pi*2))
            self.assertTrue(result['valid'])
            self.assertEqual(result['state'], 'STRAIGHT')

    def test_both_turn_directions(self):
        for sign, name in [(1, 'RIGHT'), (-1, 'LEFT')]:
            tracker = LaneTracker(counterclockwise=False)
            for n in range(120):
                result = synthetic(tracker, n/60, a=sign*.07, b=-sign*.23)
            self.assertEqual(result['state'], name)
            self.assertGreater(sign*result['steer'], 0)

    def test_transient_does_not_trigger_turn(self):
        tracker = LaneTracker()
        for n in range(120):
            pulse = 30 <= n < 40
            result = synthetic(tracker, n/60, a=.08 if pulse else 0, b=-.3 if pulse else 0)
            self.assertEqual(result['state'], 'STRAIGHT')

    def test_loss_stops_immediately_and_recovers(self):
        tracker = LaneTracker()
        synthetic(tracker, 0)
        first = synthetic(tracker, .02, missing=True)
        self.assertTrue(first['stop'])
        self.assertEqual(first['steer'], 0)
        self.assertEqual(synthetic(tracker, .3, missing=True)['state'], 'LOST')
        self.assertEqual(synthetic(tracker, .4)['state'], 'STRAIGHT')

    def test_some_occluded_rows(self):
        self.assertTrue(synthetic(LaneTracker(), 0, occluded=True)['valid'])

    def test_blank_and_full_black_are_invalid(self):
        for gray in [0, 255]:
            result = LaneTracker().update(lambda x, y: gray, 160, 120, 0)
            self.assertFalse(result['valid'])

    def test_pixel_scan_matches_native_contract(self):
        tracker = LaneTracker()
        get_pixel = lambda x, y: 0 if 30 <= x < 34 or 125 <= x < 129 else 255
        self.assertEqual(tracker._runs(get_pixel, 50, 160), [31.5, 126.5])
        a = tracker.update(get_pixel, 160, 120, 0)
        b = LaneTracker().update(None, 160, 120, 0, run_provider=lambda y: [31.5, 126.5])
        self.assertEqual(a, b)

    def test_sustained_offset_commands_turn(self):
        for sign, name in [(1, 'RIGHT'), (-1, 'LEFT')]:
            tracker = LaneTracker(counterclockwise=False)
            for n in range(90):
                result = synthetic(tracker, n/60, offset=sign*.22)
            self.assertEqual(result['state'], name)

    def test_ccw_ignores_right_curve_and_small_right_offset(self):
        for a, b, offset in [(.07, -.23, 0), (0, 0, .22)]:
            tracker = LaneTracker()
            for n in range(120):
                result = synthetic(tracker, n/60, a=a, b=b, offset=offset)
                self.assertNotEqual(result['state'], 'RIGHT')

    def test_ccw_right_recovery_then_release(self):
        tracker = LaneTracker()
        for n in range(120):
            result = synthetic(tracker, n/60, offset=.35)
        self.assertEqual(result['state'], 'RIGHT')
        for n in range(120, 300):
            result = synthetic(tracker, n/60)
        self.assertEqual(result['state'], 'STRAIGHT')

    def test_binary_matches_grayscale(self):
        gray = lambda x, y: 70 if 30 <= x < 34 or 125 <= x < 129 else 190
        binary = lambda x, y: 0 if gray(x, y) <= 70 else 255
        a = LaneTracker(threshold=70).update(gray, 160, 120, 0)
        b = LaneTracker(threshold=0).update(binary, 160, 120, 0)
        self.assertEqual(a, b)

    def test_left_boundary_loss_is_bounded_and_never_turns_right(self):
        tracker = LaneTracker()
        for n in range(120):
            result = synthetic(tracker, n/60, a=-.07, b=.23)
        self.assertEqual(result['state'], 'LEFT')
        last_full = tracker.last_full_time
        right_only = lambda y: [tracker.full_rows[y][1]] if y in tracker.full_rows else []
        for n in range(1, 40):
            r = tracker.update(None, 160, 120, last_full+n/60, run_provider=right_only)
            self.assertTrue(r['valid'])
            self.assertTrue(r['degraded'])
            self.assertEqual(r['state'], 'LEFT')
        r = tracker.update(None, 160, 120, last_full+0.9, run_provider=right_only)
        self.assertTrue(r['valid'])
        self.assertTrue(r['degraded'])
        self.assertEqual(r['state'],'LEFT')
        self.assertFalse(tracker.update(None,160,120,last_full+1.0,
                                        run_provider=lambda y:[])['valid'])

    def test_single_edge_from_straight_is_degraded_not_a_turn(self):
        tracker = LaneTracker()
        synthetic(tracker, 0)
        result = tracker.update(None, 160, 120, .02, run_provider=lambda y: [120])
        self.assertTrue(result['valid'])
        self.assertTrue(result['degraded'])
        self.assertEqual(result['state'],'STRAIGHT')

    def test_near_occlusion_bridges_four_pairs_but_not_blank(self):
        tracker=LaneTracker()
        synthetic(tracker,0)
        base=tracker.full_rows.copy()
        four=set(sorted(base)[-4:])
        partial=lambda y:list(base[y]) if y in four else []
        held=tracker.update(None,160,120,.20,run_provider=partial)
        self.assertTrue(held['valid'])
        self.assertTrue(held['degraded'])
        self.assertFalse(tracker.update(None,160,120,.21,
                                        run_provider=lambda y:[])['valid'])

    def test_crc_and_discrete_command_payload(self):
        self.assertEqual(crc16(b'123456789'), 0x29b1)
        r = dict(valid=True, stop=False, state='LEFT', offset=-.25,
                 heading=-.2, steer=-.5, confidence=.8)
        packet = encode(r, 257, 3)
        self.assertEqual(len(packet), 7)
        self.assertEqual(packet[:5], b'\xaa\x55\x01\x02\x03')
        self.assertEqual(struct.unpack('<H', packet[5:])[0], crc16(packet[:5]))
        r['valid'] = False
        self.assertEqual(encode(r, 0)[3], 0)
        self.assertEqual(encode(r, 2, 4)[4], 4)
        self.assertEqual(encode(r, 3, 5)[4], 5)
        self.assertEqual(encode(r, 4, 6)[4], 6)


if __name__ == '__main__':
    unittest.main()
