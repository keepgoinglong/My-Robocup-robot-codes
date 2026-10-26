"""Keep original startup slots; map six new loop frames without adding physical frames."""
from pathlib import Path
import json,copy
ROOT=Path(__file__).resolve().parents[1]
STARTUP=json.loads((ROOT/'Actions/my_actions4.json').read_text(encoding='utf-8-sig'))
SOURCE=json.loads((ROOT/'Actions/my_actionsfinalorigin4.json').read_text(encoding='utf-8-sig'))
assert len(SOURCE['frames'])==6 and SOURCE['period_ms']==10
LOOP=[10,11,13,14,15,17]
DATA=copy.deepcopy(STARTUP);DATA['limits']=SOURCE['limits']
for i,slot in enumerate(LOOP):DATA['frames'][slot]=copy.deepcopy(SOURCE['frames'][i])
for slot in (12,16):
 DATA['frames'][slot]=copy.deepcopy(DATA['frames'][slot-1])
 DATA['frames'][slot].update(name='Unused straight slot; legacy turn only',duration_ms=0,hold_ms=0)

# 起始动作采用原第6帧（左脚离地），保留该帧全部参数；随后从第1帧循环。
DATA['frames'][0]=copy.deepcopy(DATA['frames'][17])
for slot in range(1,10):
 DATA['frames'][slot]=copy.deepcopy(DATA['frames'][0])
 DATA['frames'][slot].update(name='Unused removed startup',duration_ms=0,hold_ms=0,wait_reached=False)
