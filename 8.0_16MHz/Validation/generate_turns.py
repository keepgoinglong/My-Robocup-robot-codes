from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parents[1]
gait=json.loads((ROOT/'Actions/my_actions4.json').read_text(encoding='utf-8-sig'))
lines=['/* Generated from TURNLEFT/RIGHT.json; all targets and timings preserved. */','#ifndef TURN_FRAMES_H','#define TURN_FRAMES_H','static const ActionFrame k_turn_frames[2][8] = {']
for name in ['TURNLEFT','TURNRIGHT']:
 p=ROOT/f'Actions/{name}.json';data=json.loads(p.read_text(encoding='utf-8-sig'))
 assert data['period_ms']==10 and len(data['frames'])==8
 lines+=['/* '+name+' SHA256 '+hashlib.sha256(p.read_bytes()).hexdigest()+' */','{']
 for index,f in enumerate(data['frames']):
  assert not f.get('samples') and not f.get('timing') and not f.get('limits') and not f['wait_reached']
  assert f['duration_ms']>0 and f['hold_ms']>=0 and f['speed']>0
  pos=[f['positions'][str(i)] for i in range(1,15)]
  for i,v in enumerate(pos,1):
   assert data['limits'][str(i)][0]<=v<=data['limits'][str(i)][1]
   assert gait['limits'][str(i)][0]<=v<=gait['limits'][str(i)][1], (name,index,i,v)
  lines+=['{{'+', '.join(map(str,pos))+'}, '+f"{f['duration_ms']}U, {f['hold_ms']}U, {f['speed']}U, {f['acc']}U, 0U, {int(index in (1,5))}U"+'},']
 lines+=['},']
lines+=['};']
p=ROOT/'Actions/TURNLEFT1.json'
data=json.loads(p.read_text(encoding='utf-8-sig'))
assert data['period_ms']==10 and len(data['frames'])==8
lines+=['/* Default left: TURNLEFT1 SHA256 '+hashlib.sha256(p.read_bytes()).hexdigest()+' */','static const ActionFrame k_default_left_frames[8] = {']
for index,f in enumerate(data['frames']):
 assert not f.get('samples') and not f.get('timing') and not f.get('limits') and not f['wait_reached']
 pos=[f['positions'][str(i)] for i in range(1,15)]
 for i,v in enumerate(pos,1):assert gait['limits'][str(i)][0]<=v<=gait['limits'][str(i)][1]
 lines+=['{{'+', '.join(map(str,pos))+'}, '+f"{f['duration_ms']}U, {f['hold_ms']}U, {f['speed']}U, {f['acc']}U, 0U, {int(index in (1,5))}U"+'},']
lines+=['};','/* Frozen reference for legacy turn deltas; never subtract the new gait here. */','static const uint16_t k_turn_reference[8][ROBOT_SERVO_COUNT] = {']
for f in gait['frames'][10:18]:lines+=['{'+', '.join(str(f['positions'][str(i)]) for i in range(1,15))+'},']
lines+=['};','#endif','']
(ROOT/'Inc/turn_frames.h').write_text('\n'.join(lines),encoding='utf-8')
print('Validated both 8-frame turn sequences against source and existing robot limits.')
