import unittest

from boundary_tracker import BoundaryTracker, resolve_control
from lane_protocol import encode_angle, crc16


class RightRecoveryV7Tests(unittest.TestCase):
    def sample(self, tracker, timestamp, left, right=145):
        return tracker.update(
            None, 160, 120, timestamp,
            run_provider=lambda y: [left] + ([] if right is None else [right]))

    def test_early_confirm_and_hysteresis(self):
        tracker = BoundaryTracker()
        for timestamp in (0, .04, .08, .12, .16):
            lane = self.sample(tracker, timestamp, 38)
        self.assertTrue(lane['wide_road'])
        self.assertEqual(self.sample(tracker, .241, 38)['position_direction'], 1)
        self.assertEqual(self.sample(tracker, .28, 31)['position_direction'], 1)
        self.assertEqual(self.sample(tracker, .32, 28)['position_direction'], 0)

    def test_mode_change_returns_to_conservative_single_edge(self):
        tracker = BoundaryTracker()
        for timestamp in (0, .04, .08, .12, .16):
            self.sample(tracker, timestamp, 40)
        self.sample(tracker, .20, 40, None)
        lane = self.sample(tracker, .321, 40, None)
        self.assertFalse(lane['aggressive_right'])
        self.assertEqual(lane['position_direction'], 0)

    def test_narrow_road_keeps_original_right_threshold(self):
        tracker = BoundaryTracker()
        for timestamp in (0, .04, .08, .12, .16):
            lane = self.sample(tracker, timestamp, 38, 108)
        self.assertFalse(lane['wide_road'])
        self.assertTrue(tracker.narrow_road_latched)
        self.assertEqual(self.sample(tracker, .40, 38, 108)['position_direction'], 0)

    def test_single_edge_never_uses_wide_road_aggressive_right(self):
        tracker = BoundaryTracker()
        for timestamp in (0, .04, .08, .12, .16):
            lane = self.sample(tracker, timestamp, 40, 145)
        self.assertTrue(lane['wide_road'])
        lane = self.sample(tracker, .20, 40, None)
        self.assertFalse(lane['aggressive_right'])
        self.assertEqual(lane['position_direction'], 0)

    def test_single_transient_does_not_turn(self):
        tracker = BoundaryTracker()
        self.assertEqual(self.sample(tracker, 0, 40, None)['position_direction'], 0)
        self.assertEqual(self.sample(tracker, .08, 40, None)['position_direction'], 0)
        self.assertEqual(self.sample(tracker, .10, 15, None)['position_direction'], 0)

    def test_urgent_right_beats_curve(self):
        lane = dict(valid=True, stop=False, mode='BOTH', position_direction=1,
                    curve_direction=-1, position_error_deg=12, urgent_right=True)
        _, command = resolve_control(lane, dict(valid=True, angle=-20))
        self.assertEqual(command['status'], 4)
        packet = encode_angle(command, 255)
        self.assertEqual(int.from_bytes(packet[4:6], 'little', signed=True), 120)
        self.assertEqual(int.from_bytes(packet[-2:], 'little'), crc16(packet[:-2]))


if __name__ == '__main__':
    unittest.main()
