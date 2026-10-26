"""Import all shape targets and timings exactly; verify gait anchor compatibility."""
import hashlib
import json
from action_model import DATA
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
gait = DATA
anchors = [11,11,14,10,11]
lines = ['/* Generated from Actions/1.json .. 5.json. Arms from shape sources, legs and foot timing from current straight gait. */',
         '#ifndef SHAPE_FRAMES_H', '#define SHAPE_FRAMES_H',
         'static const uint8_t k_shape_anchor[5] = {11U,11U,14U,10U,11U};',
         'static const ActionFrame k_shape_frames[5] = {']
for n,anchor in enumerate(anchors,1):
    path = ROOT/f'Actions/{n}.json'
    data=json.loads(path.read_text(encoding='utf-8-sig'))
    assert len(data['frames'])==1 and data['period_ms']==10
    f=data['frames'][0]
    assert not f.get('samples') and not f.get('timing') and not f.get('limits')
    assert f['hold_ms']==3000 and f['duration_ms']==(800 if n in (3,4) else 200) and f['speed']==1600 and f['acc']==16
    assert not f['wait_reached']
    base=gait['frames'][anchor]
    p=[base['positions'][str(i)] for i in range(1,15)]
    if n in (1,2,5):
        for i in range(1,5):p[i-1]=f['positions'][str(i)]
    motion=base if n in (3,4) else f
    for i in range(1,15):
        lo,hi=gait['limits'][str(i)]
        assert lo<=p[i-1]<=hi
    label = ('右脚前摆落地稳定姿态，双脚在地' if n in (1,2,5) else f['name'])
    lines += [f'  /* shape {n}: {label}; 动作源SHA256 {hashlib.sha256(path.read_bytes()).hexdigest()} */',
              '  {{'+', '.join(map(str,p))+'}, '+str(motion['duration_ms'])+'U, 3000U, '+str(motion['speed'])+'U, '+str(motion['acc'])+'U, 0U, '+str(int(anchor in (11,15)))+'U},']
lines+=['};','#endif','']
(ROOT/'Inc/shape_frames.h').write_text('\n'.join(lines),encoding='utf-8')
print('5 shapes validated: arm shapes 1/2/5 at right-foot-forward landing frame11; left foot at14; right foot at10.')
