"""fddebug rule boundaries and actual ARM wait/timing regression."""
import copy,struct
from fddebug_reference.servo_core import validate,frame_duration
from action_model import SOURCE
from test_firmware import Firmware,TARGETS
for speed,acc,ok in [(1,0,True),(32766,254,True),(0,0,False),(32767,0,False),(1,255,False)]:
    d=copy.deepcopy(SOURCE);d['frames'][0].update(speed=speed,acc=acc)
    try: validate(d);accepted=True
    except ValueError: accepted=False
    assert accepted==ok,(speed,acc)
f=copy.deepcopy(SOURCE['frames'][0]);f['timing']={'14':{'delay_ms':15,'duration_ms':0},'13':{'delay_ms':25,'duration_ms':1000}}
assert frame_duration(f)==1025
# 当前第6帧起始动作不要求wait_reached；20/21边界在再次起步的姿态校验中检查。
for error,expected in [(20,1),(21,0)]:
    b=Firmware();b.stand();b.feedback_follows=False
    b.measured=[v+error for v in TARGETS[0]]
    assert b.call('Robot_StartActions',0,0)==expected
b=Firmware();b.stand();b.trace.clear()
# Exercise an off-grid delay and zero duration using actual compiled C tables.
b.uc.mem_write(b.symbols['k_action_delay']+(10*14+13)*4,struct.pack('<I',15))
b.uc.mem_write(b.symbols['k_action_joint_duration']+(10*14+13)*4,struct.pack('<I',0))
assert b.call('Robot_StartActions',0,0)==1
begin=b.tick
for _ in range(160):b.step(1)
packets=[r for r in b.trace if r['frame']==10 and 14 in r['ids']]
assert len(packets)==1 and packets[0]['t']-begin==15
r=packets[0];assert r['sent'][r['ids'].index(14)]==TARGETS[10][13]
print('PASS: speed/ACC limits; max joint end; 20/21-count start-pose boundary; 15 ms delay and zero-duration endpoint.')
