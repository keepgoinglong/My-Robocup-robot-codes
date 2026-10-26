"""ARM replay: shapes retain steering hips and full hold; mocked servos."""
import struct
from test_feedback import Board,control
from test_vision import packet
from test_firmware import TARGETS

def pose(b,name):return list(struct.unpack('<14H',b.uc.mem_read(b.symbols[name],28)))
for direction,angle in [(3,0),(4,50)]:
 for shape in range(1,6):
  b=Board();b.stand();b.control=True;assert b.call('VisionControl_StartWalk')==1
  b.feed(control(1,direction,angle));b.step()
  # Establish a complete turned step before sending the independent shape event.
  for _ in range(300):b.step()
  b.feed(packet(2,0,shape));b.step()
  b.until(lambda:b.value('s_shape_phase')!=0,6000)
  phase=b.value('s_shape_phase');start=b.tick
  if phase==4:expected=pose(b,'s_now')[1:3]
  else:
   f=b.value('s_frame');expected=TARGETS[f][1:3].copy();hip=1 if f<14 else 0
   expected[hip]=2107 if direction==3 else expected[hip]-82
  seen=set();hold_start=None
  while not b.value('s_completed_shapes',4):
   phase=b.value('s_shape_phase');seen.add(phase)
   if phase in (1,2,3):
    assert pose(b,'s_motion')[1:3]==expected,(direction,shape,phase,pose(b,'s_motion'),expected)
    if phase in (1,3) and b.value('s_stage')==2 and hold_start is None:hold_start=b.tick
   assert b.value('s_state')!=4
   assert b.tick-start<12000
   b.step()
  assert hold_start is not None and b.tick-hold_start>=3000
  assert b.value('s_control_kind')==direction
  f=b.value('s_frame');hip=2 if f<14 else 1
  target=2107 if direction==3 else TARGETS[f][hip]-82
  assert pose(b,'s_motion')[hip]==target
  print('PASS shape',shape,'direction',direction,'hips',expected,'hold>=3000; resumes steering')
