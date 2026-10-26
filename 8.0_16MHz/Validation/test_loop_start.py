from test_feedback import Board,control
from test_firmware import TARGETS,FRAMES
for slot in [10,11,13,14,15,17]:
 b=Board();b.stand();assert b.measured==TARGETS[15]==TARGETS[0]
 b.call('HandleCommand',ord('g'));b.control=True;b.step()
 b.until(lambda:b.value('s_frame')==slot)
 b.call('HandleCommand',ord('s'));b.until(lambda:b.value('s_state')==2)
 assert b.measured==TARGETS[15] and b.call('VisionControl_IsEnabled')==0
 b.call('HandleCommand',ord('g'));assert b.value('s_frame')==17 and b.call('VisionControl_IsEnabled')==1
b=Board();b.start();start=len(b.trace)
b.until(lambda:sum(r['frame']==17 and r['t']==r['begin'] for r in b.trace)>=4)
rows=[r for r in b.trace if r['t']==r['begin'] and r['frame']!=0]
assert [r['frame'] for r in rows[:18]]==[17,10,11,13,14,15]*3
for r in b.trace:
 if r['frame']!=0 and r['t']-r['begin']==r['duration']:assert r['pos']==TARGETS[r['frame']]
for status,angle in [(2,200),(2,-200),(3,0),(3,900),(3,-900),(4,0)]:
 b=Board();b.start();b.feed(control(20,status,angle));b.step();b.done(20)
 assert b.value('s_state')==3
for shape in range(1,7):
 b=Board();b.start();assert b.call('Robot_RequestShape',shape)==1
 b.until(lambda:b.value('s_completed_shapes',4)==1,25000)
 assert b.value('s_state')==3
# s while left correction active returns exact base pose, without leaving hip offset.
b=Board();b.start();b.feed(control(21,3));b.step();b.until(lambda:b.value('s_control_phase')==2 and b.value('s_frame')==15)
b.call('HandleCommand',ord('s'));b.until(lambda:b.value('s_state')==2);assert b.measured==TARGETS[15]
print('PASS actual ARM: left-front start; no startup steps; 3 exact loops; s from all6 frames; g resumes at17; all correction modes; all6 shapes; s clears turn hip offset.')
