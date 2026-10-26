import json
import re
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


class ObstacleActionTests(unittest.TestCase):
    def test_generated_tables_preserve_json(self):
        text=(ROOT/'Inc/obstacle_action_frames.h').read_text(encoding='utf-8')
        for filename,prefix,count in (("my_actionsfinalshort3.json","k_small",6),
                                      ("my_actions strideover4.json","k_over",16)):
            data=json.loads((ROOT/'Actions'/filename).read_text(encoding='utf-8-sig'))
            self.assertEqual(len(data['frames']),count)
            block=text.split('static const ActionFrame '+prefix+'_frames',1)[1].split('};',1)[0]
            for frame in data['frames']:
                positions=', '.join(str(frame['positions'][str(i)]) for i in range(1,15))
                self.assertIn('{{'+positions+'}',block)
                self.assertIn('%dU, %dU, %dU, %dU' %
                              (frame['duration_ms'],frame['hold_ms'],frame['speed'],frame['acc']),block)

    def test_normal_walk_remains_default(self):
        motion=(ROOT/'Src/robot_motion.c').read_text(encoding='utf-8')
        self.assertIn('s_motion = k_action_frames[frame];',motion)
        self.assertRegex(motion,r'if \(s_obstacle_small && frame >= ACTION_LOOP_FIRST\)')
        self.assertIn('if (s_frame == 13U || s_frame == 17U)',motion)
        self.assertIn('(void)BeginFrame(ACTION_STEP4_STABLE, now);',motion)

    def test_ultrasonic_uses_strict_consecutive_confirmation(self):
        control=(ROOT/'Src/obstacle_control.c').read_text(encoding='utf-8')
        config=(ROOT/'Inc/robot_config.h').read_text(encoding='utf-8')
        self.assertIn('#define OBSTACLE_APPROACH_CONFIRM_SAMPLES  8U',config)
        self.assertIn('#define OBSTACLE_OVER_CONFIRM_SAMPLES      5U',config)
        self.assertIn('s_near_count < OBSTACLE_APPROACH_CONFIRM_SAMPLES',control)
        self.assertIn('s_over_count < OBSTACLE_OVER_CONFIRM_SAMPLES',control)
        self.assertIn('s_over_count >= OBSTACLE_OVER_CONFIRM_SAMPLES',control)
        self.assertIn('if (distance < APPROACH_ENTER_MM)',control)
        self.assertNotIn('sample_window',control)
        self.assertIn('if (result != HC_SR04_OK) {',control)
        self.assertIn('s_near_count = 0U;',control)
        self.assertIn('s_over_count = 0U;',control)

    def test_small_step_latches_until_obstacle_over(self):
        control=(ROOT/'Src/obstacle_control.c').read_text(encoding='utf-8')
        motion=(ROOT/'Src/robot_motion.c').read_text(encoding='utf-8')
        self.assertNotIn('Robot_SetObstacleApproach(0U, 0)',control)
        self.assertNotIn('APPROACH_EXIT_MM',control)
        self.assertIn('if (!s_obstacle_small_requested && !s_obstacle_small)',motion)
        self.assertIn('s_obstacle_small = 0U; s_obstacle_small_requested = 0U;',motion)

    def test_small_step_visual_angle_is_divided_by_one_point_five(self):
        config=(ROOT/'Inc/robot_config.h').read_text(encoding='utf-8')
        motion=(ROOT/'Src/robot_motion.c').read_text(encoding='utf-8')
        self.assertIn('#define OBSTACLE_ANGLE_DIVISOR_X10      15',config)
        self.assertIn('/ OBSTACLE_ANGLE_DIVISOR_X10',motion)

    def test_host_left_curve_angle_is_divided_by_four(self):
        config=(ROOT/'Inc/robot_config.h').read_text(encoding='utf-8')
        motion=(ROOT/'Src/robot_motion.c').read_text(encoding='utf-8')
        self.assertIn('#define LEFT_CURVE_ANGLE_DIVISOR          4',config)
        self.assertIn('/ (10 * LEFT_CURVE_ANGLE_DIVISOR)',motion)


if __name__=='__main__':unittest.main()
