"""Execute current ARM firmware; UART4 TX/RX and servo hardware are mocked."""
import struct,binascii,json,hashlib,sys
from test_vision import System,packet,ROOT,IMAGE,TARGETS,FRAMES
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R1,UC_ARM_REG_R2

def control(seq,status=1,angle=0):
 b=bytes([170,86,seq,status])+struct.pack('<h',angle)
 return b+struct.pack('<H',binascii.crc_hqx(b,65535))

class Board(System):
 def __init__(self):
  super().__init__();self.feedback=[];self.tx_pending=False;self.tx_fail=False
  address=self.symbols['HAL_UART_Transmit_IT']&~1
  self.uc.hook_add(UC_HOOK_CODE,self.transmit,begin=address,end=address)
  self.uart=0x20017000
  self.call('Vision_Init',self.uart)
 def transmit(self,*args):
  if self.tx_fail:self.result(1);return
  p=self.uc.reg_read(UC_ARM_REG_R1);n=self.uc.reg_read(UC_ARM_REG_R2)
  raw=bytes(self.uc.mem_read(p,n));assert n==6 and raw[:2]==b'\xaa\x57'
  assert binascii.crc_hqx(raw[:4],65535)==int.from_bytes(raw[4:],'little')
  self.feedback.append(dict(t=self.tick,seq=raw[2],state=raw[3],raw=raw,frame=self.value('s_frame'),phase=self.value('s_control_phase')))
  self.tx_pending=True;self.result(0)
 def write(self,*args):
  super().write(*args)
  self.trace[-1]['control']=self.value('s_control_phase')
  self.trace[-1]['kind']=self.value('s_control_kind')
 def step(self,ms=10):
  if self.tx_pending:
   self.call('HAL_UART_TxCpltCallback',self.uart);self.tx_pending=False
  super().step(ms)
 def start(self):
  self.stand();self.control=True;assert self.call('VisionControl_StartWalk')==1;self.step()
  assert self.feedback[-1]['state']==0
 def done(self,seq):
  self.until(lambda:len([x for x in self.feedback if x['seq']==seq and x['state']==2])==3,40000)
  self.step()
 def ready(self):self.until(lambda:self.value('s_control_gate')==1,2000)

RESULTS=[]
def passed(s):RESULTS.append(s);print('PASS',s)
def left_target(frame, offset=42):
 p=TARGETS[frame].copy()
 p[1 if frame<14 else 2]=2048+offset
 return p

def run():
 b=Board();b.uc.mem_write(0x20018000,b'123456789');assert b.call('Vision_CRC16',0x20018000,9)==0x29b1
 b.start();b.feed(b'\x00\xaa'+control(255,3)[:3]);b.feed(control(255,3)[3:]+packet(255,2,0)+control(0,5));b.step()
 assert b.value('s_feedback_state')==1 and b.value('s_feedback_seq')==255
 for i in range(40):b.feed(control(i,4));b.step()
 b.done(255)
 assert all(x['seq']==255 for x in b.feedback if x['state'] in (1,2))
 dones=[x for x in b.feedback if x['state']==2];assert [dones[i+1]['t']-dones[i]['t'] for i in range(2)]==[50,50]
 assert all(x['frame'] in (13,17) and x['phase']==4 for x in dones)
 rows=[r for r in b.trace if r['control']==2 and r['t']==r['begin']]
 assert [r['frame'] for r in rows]==[13,14,15]
 left=json.loads((ROOT/'Actions/TURNLEFT1.json').read_text(encoding='utf-8-sig'))['frames']
 for r in [r for r in b.trace if r['control']==2 and r['t']-r['begin']==r['duration']]:
  src=FRAMES[r['frame']];assert r['pos']==left_target(r['frame'])
  assert (r['duration'],r['speed'],r['acc'])==(src['duration_ms'],src['speed'],src['acc'])
 b.ready();b.feed(control(255,3));b.step();assert b.value('s_feedback_state')==0
 b.until(lambda:b.value('s_frame')==14)
 b.feed(control(0,3));b.step();b.done(0)
 rows2=[r for r in b.trace if r['control']==2 and r['t']==r['begin']]
 assert [r['frame'] for r in rows2[-3:]]==[17,10,11]
 for r in [r for r in b.trace if r['control']==2 and r['t']-r['begin']==r['duration']]:
  assert r['pos']==left_target(r['frame'])
 passed('CRC29B1; mixed split packets; first request retained through WAIT; busy packets discarded; seq255->0; all6 hip-only left poses; DONE three times50ms')
 # Angle-bearing left packets must only reduce/increase left amplitude, never reverse.
 for value,offset in [(0,42),(10,37),(-10,47),(50,17),(-50,67),(900,12),(-900,72)]:
  b=Board();b.start();b.feed(control(12,3,value));b.step();b.done(12)
  rows=[r for r in b.trace if r['control']==2 and r['t']-r['begin']==r['duration']]
  assert len(rows)==3
  for row in rows:
   assert row['pos']==left_target(row['frame'],offset)
   assert row['pos'][1 if row['frame']<14 else 2]>2048
 passed('Left zero/sign/saturation: 2090 default, 2060..2120, 5 counts/degree, no right reversal; other joints unchanged')
 # Invalid values and CRC errors neither execute nor alter the accepted transaction.
 b=Board();b.start();bad=bytearray(control(4,3));bad[-1]^=1
 b.feed(bad+control(5,6)+control(6,2,901)+control(7,4,1));b.step()
 assert b.value('s_feedback_state')==0 and b.status()['crc']>=1 and b.status()['fields']==3
 b.feed(b'\xaa\x56\x00'+control(8,4));b.step();assert b.value('s_feedback_seq')==8
 b.done(8);passed('CRC/unknown-status/out-of-range rejection and embedded-header recovery; status4 angle0 executes right correction')
 for status,value in [(2,-200),(2,200),(2,0),(2,900),(2,-900),(4,0)]:
  b=Board();b.start();b.feed(control(1,status,value));b.step()
  b.until(lambda:b.value('s_control_phase')==2)
  b.feed(control(2,0)+control(3,5)+packet(1,2,0))
  b.done(1)
  endpoints=[r for r in b.trace if r['control']==2 and r['t']-r['begin']==r['duration']]
  assert len(endpoints)==3
  divisor=1
  import re
  divisor=int(re.search(r'#define LINE_ANGLE_DIVISOR\s+(\d+)',(ROOT/'Inc/robot_config.h').read_text(encoding='utf-8')).group(1))
  c=-62 if status==4 else (-1 if value>0 else 1)*int(abs(value)*4096/(3600*divisor)+.5)
  for row in endpoints:
   expected=TARGETS[row['frame']].copy()
   expected[1 if row['frame']<14 else 2]+=c
   assert row['pos']==expected
  count=len([r for r in b.trace if r['control']==2 and r['t']==r['begin']])
  for _ in range(600):b.step()
  assert len([r for r in b.trace if r['control']==2 and r['t']==r['begin']])==count and b.value('s_state')==3
 passed('Current divisor, angle signs/extremes; non-hip joints equal straight gait; status0/5/loss never repeats correction or stops walking; AA55 cannot turn')
 # Verify receiver never cancels shape events, even through transactional motion.
 for shape in range(1,7):
  b=Board();b.start();b.feed(control(1,3));b.step();b.until(lambda:b.value('s_control_phase')==2)
  b.feed(packet(10,2,shape)+control(2,5));b.done(1)
  b.until(lambda:b.value('s_completed_shapes',4)==1,25000)
  if shape<=5:
   rows=[r for r in b.trace if r['shape']==shape and r['phase'] in (1,3)]
   later=[r for r in b.trace if r['t']>rows[-1]['t']]
   assert later[0]['t']-rows[-1]['t']==3000
  for _ in range(1000):b.step()
  b.feed(packet(11,1,shape));b.until(lambda:b.value('s_completed_shapes',4)==2,25000)
 passed('All6 shapes survive busy/WAIT/loss; full holds; same shape with new sequence after10s executes again')
 # Shape already running holds the queued correction until done, with BUSY heartbeat.
 b=Board();b.start();b.feed(packet(1,1,4));b.until(lambda:b.value('s_active_shape')==4)
 b.feed(control(20,3));b.step();assert b.value('s_control_phase')==1
 b.until(lambda:b.value('s_completed_shapes',4)==1);b.done(20)
 passed('Request waits behind active shape with BUSY heartbeats, never shortens display')
 for mode in ('stop','off','write','stall'):
  b=Board();b.start();b.feed(control(25,3));b.step();b.until(lambda:b.value('s_control_phase')==2)
  if mode=='stop':b.call('VisionControl_SetEnabled',0)
  elif mode=='off':b.call('Robot_EmergencyOff')
  elif mode=='write':b.write_fail=True
  else:b.tick+=300;b.call('Robot_Task')
  b.until(lambda:any(x['state']==3 and x['seq']==25 for x in b.feedback),10000)
  assert not any(x['state']==2 for x in b.feedback)
 passed('Manual stop/emergency, scheduler stall and write failure still report FAULT, never DONE')
 b=Board();b.start();b.feed(control(25,3));b.step();b.until(lambda:b.value('s_control_phase')==2)
 b.read_fail=True;b.feedback_follows=False;b.measured[5]+=100
 b.done(25)
 starts=[r for r in b.trace if r['t']==r['begin'] and r['frame']>=10]
 data=FRAMES
 for x,y in zip(starts,starts[1:]):
  src=data[x['frame']]
  assert y['t']-x['t']==src['duration_ms']+src['hold_ms']
 assert b.value('s_state')==3 and not any(x['state']==3 for x in b.feedback)
 passed('Correction DONE means trajectory complete; no servo arrival reads or added waits; all frame transitions retain original timing despite feedback mismatch')
 b=Board();b.start();b.tx_fail=True;b.feed(control(26,3));b.step()
 b.until(lambda:b.value('s_control_phase')==4,20000)
 assert not any(x['state']==2 for x in b.feedback)
 b.feed(control(27,4));b.step();assert b.value('s_feedback_seq')==26
 b.tx_fail=False;b.done(26)
 passed('Failed TX is nonblocking and retried, no next transaction before completion feedback')
 # Real vision-side feedback parser compatibility, including no-feedback watchdog.
 sys.path.insert(0,str(ROOT.parent/'1.0/Vision'))
 from motion_feedback import MotionFeedback
 v=MotionFeedback(True);b=Board();b.start()
 v.feed(b.feedback[-1]['raw'],b.tick/1000);assert v.ready
 v.sent(dict(active=True),40,b.tick/1000);b.feed(control(40,3));b.step();b.done(40)
 for x in b.feedback[1:]:v.feed(x['raw'],x['t']/1000)
 assert v.pending is None and v.ready and v.reset_heading
 lane=dict(valid=True,state='LEFT');req=dict(active=True,direction=-1,angle=0,status=3)
 _,out=v.gate(lane,req,b.tick/1000+2);assert out['status']==5
 passed('Actual vision MotionFeedback accepts matching AA57 and waits on absent/stale feedback')
 b=Board();b.start()
 raw=bytes(b.uc.mem_read(b.symbols['s_tx_packet'],6))
 assert b.call('Vision_SendFeedback',77,3)==0
 assert bytes(b.uc.mem_read(b.symbols['s_tx_packet'],6))==raw
 b.feed(packet(1,2,0))
 for _ in range(1000):b.step()
 assert not any(r['control']==2 for r in b.trace)
 assert not any(x['state'] in (1,2) for x in b.feedback)
 b.feed(control(10,3));b.step();b.done(10)
 for _ in range(1100):b.step()
 before=len([x for x in b.feedback if x['state']==2])
 b.feed(control(10,4));b.step()
 b.until(lambda:len([x for x in b.feedback if x['state']==2])==before+3,15000)
 b=Board();b.tick=0xffffe000;b.start();b.feed(control(0,3));b.step();b.done(0)
 assert b.tick>0xffffffff
 passed('AA55-only never starts turns; TX busy buffer immutable; same seq reusable after short-term window; tick wrap completes')

 (ROOT/'Validation/feedback_test_results.json').write_text(json.dumps({'image_sha256':hashlib.sha256(IMAGE.read_bytes()).hexdigest(),'passed':RESULTS,'scope':'Actual ARM code; mocked UART/servo; no mechanical or physical contact validation'},indent=2),encoding='utf-8')
if __name__=='__main__':run()
