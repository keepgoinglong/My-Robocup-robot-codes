"""Validate this action format and generate the C table. Does not edit the source JSON."""
import hashlib
import json
from action_model import DATA
from fddebug_reference.servo_core import validate, frame_duration, servo_timing
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def generate():
    source = ROOT / 'Actions/my_actionsfinalorigin4.json'
    data = validate(DATA)
    assert data['period_ms'] == 10
    assert len(data['frames']) == 18
    lines = ['/* Frame-6 left-foot-lift start pose and six-frame loop from my_actionsfinalorigin4; old startup removed. */',
             '/* SHA256: ' + hashlib.sha256(source.read_bytes()).hexdigest() + ' */',
             '#ifndef ACTION_FRAMES_H', '#define ACTION_FRAMES_H',
             '#define ACTION_FRAME_COUNT 18U', '#define ACTION_LOOP_FIRST 10U',
             '/* my_actionsfinalorigin4原文件：第2/5帧是双脚着地稳定点，第6帧（左脚离地）是直走起始动作。 */',
             '#define ACTION_STEP3_STABLE 11U',
             '#define ACTION_STEP4_STABLE 15U',
             '#define ACTION_WALK_START_FRAME 17U',
             '#define ACTION_WALK_START_NEXT ACTION_LOOP_FIRST',
             '#define ACTION_FINAL_LANDING ACTION_STEP4_STABLE',
             'typedef struct {',
             '  uint16_t position[ROBOT_SERVO_COUNT];',
             '  uint32_t duration_ms, hold_ms;',
             '  uint16_t speed;',
             '  uint8_t acc, wait_reached, landing;',
             '} ActionFrame;',
             'static const uint16_t k_action_limits[ROBOT_SERVO_COUNT][2] = {']
    for i in range(1, 15):
        lo, hi = data['limits'][str(i)]
        assert 0 <= lo <= hi <= 4095
        lines.append(f'  {{{lo}, {hi}}}, /* ID{i} */')
    lines += ['};', 'static const ActionFrame k_action_frames[ACTION_FRAME_COUNT] = {']
    for index, frame in enumerate(data['frames']):
        assert not frame.get('samples') and not frame.get('limits')
        assert set(frame['positions']) == {str(i) for i in range(1, 15)}
        assert (frame['name'].endswith('落地')) == (index in (11, 15))
        pos = [frame['positions'][str(i)] for i in range(1, 15)]
        for i, p in enumerate(pos, 1):
            assert type(p) is int and data['limits'][str(i)][0] <= p <= data['limits'][str(i)][1]
        lines += [f"  /* {index}: {frame['name']} */",
                  '  {{' + ', '.join(map(str, pos)) + '}, ' +
                  f"{frame_duration(frame)}U, {frame['hold_ms']}U, {frame['speed']}U, {frame['acc']}U, " +
                  f"{int(frame['wait_reached'])}U, {int(index in (11,15))}U" + '},']
    lines += ['};']
    for name,key in [('k_action_delay','delay_ms'),('k_action_joint_duration','duration_ms')]:
        lines += ['static const uint32_t '+name+'[ACTION_FRAME_COUNT][ROBOT_SERVO_COUNT] = {']
        for frame in data['frames']:
            lines += ['{'+', '.join(str(servo_timing(frame,i)[0 if key == 'delay_ms' else 1])+'U' for i in range(1,15))+'},']
        lines += ['};']
    lines += ['#endif','']
    (ROOT / 'Inc/action_frames.h').write_text('\n'.join(lines), encoding='utf-8')
    print('Generated 18 frames, 14 IDs; source positions and limits preserved.')

if __name__ == '__main__':
    generate()
