"""Actual compiled ARM code, mocked UART and servo bus; no physical dynamics."""
import struct,json,hashlib
from test_feedback import Board,control
from test_firmware import TARGETS,FRAMES,ROOT,IMAGE

def signed(b,name):
 return struct.unpack('<h',b.uc.mem_read(b.symbols[name],2))[0]
results=[]
for angle,counts in [(-900,62),(0,62),(10,66),(50,82),(95,100),(150,100),(900,100)]:
 b=Board();b.stand();b.control=True;assert b.call('VisionControl_StartWalk')==1;b.feed(control(1,4,angle));b.step()
 assert signed(b,'s_requested_correction')==-counts
 # Request during a frame must not rewrite the current interpolation endpoint.
 before=bytes(b.uc.mem_read(b.symbols['s_motion'],28))
 b.feed(control(2,4,angle));b.step()
 assert bytes(b.uc.mem_read(b.symbols['s_motion'],28))==before
 start=b.tick;b.trace.clear()
 while b.tick-start<6000:
  b.step(10)
  assert b.value('s_state')==3
 endpoints=[r for r in b.trace if r['t']-r['begin']==r['duration'] and r['begin']>start]
 assert len(endpoints)>=6
 for r in endpoints:
  frame=r['frame'];hip=2 if frame<14 else 1
  expected=TARGETS[frame].copy();expected[hip]-=counts
  assert r['pos']==expected,(angle,frame,r['pos'],expected)
  assert (r['duration'],r['speed'],r['acc'])==(FRAMES[frame]['duration_ms'],FRAMES[frame]['speed'],FRAMES[frame]['acc'])
 # Latest straight packet releases correction while BUSY/DONE may be active.
 b.feed(control(3,1,0));b.step();assert signed(b,'s_requested_correction')==0
 b.trace.clear();start=b.tick
 while b.tick-start<3000:b.step(10)
 for r in b.trace:
  if r['t']-r['begin']==r['duration'] and r['begin']>start:assert r['pos']==TARGETS[r['frame']]
 results.append({'angle_deci':angle,'right_counts':counts,'endpoints':len(endpoints)})
# Invalid CRC/range must leave prior accepted right correction intact; WAIT/loss too.
b=Board();b.stand();b.control=True;assert b.call('VisionControl_StartWalk')==1;b.feed(control(1,4,50));b.step()
for payload in [control(2,4,901),control(3,0,0),control(4,5,0),control(5,4,0)[:-1]+b'\x00']:
 b.feed(payload);b.step();assert signed(b,'s_requested_correction')==-82
# Manual stop still lands and holds.
b.call('VisionControl_SetEnabled',0);b.call('Robot_StopMarch');b.until(lambda:b.value('s_state')==5,6000)
(ROOT/'Validation/right_recovery_5_3_results.json').write_text(json.dumps({'sha256':hashlib.sha256(IMAGE.read_bytes()).hexdigest(),'cases':results,'scope':'ARM with mocked buses, not physical robot'},indent=2),encoding='utf-8')
print('PASS: 7 right amplitudes; actual UART CRC/range/loss; six-frame endpoints; original timing/speed/ACC; straight release; manual landing stop')
