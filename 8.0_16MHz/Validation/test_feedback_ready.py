from test_feedback import Board,control
from test_firmware import ROOT,IMAGE,FRAMES
import hashlib,json,struct,binascii
results=[]
def check(msg):results.append(msg);print('PASS',msg)
def settled(b):return b.value('s_state') in (2,5) and b.call('Robot_GetActiveShape')==0
def ready():
 b=Board();b.stand();b.control=True;b.step();assert b.feedback[-1]['state']==0;return b
# Independently check every emitted binary variant against Python CCITT implementation.
b=Board()
for seq in (0,6,255):
 for state in range(4):
  assert b.call('Vision_SendFeedback',seq,state)==1
  raw=b.feedback[-1]['raw'];assert raw[:4]==bytes([170,87,seq,state])
  assert raw[-2:]==binascii.crc_hqx(raw[:4],65535).to_bytes(2,'little')
  b.call('HAL_UART_TxCpltCallback',b.uart);b.tx_pending=False
assert bytes.fromhex('aa57000043dd')==bytes([170,87,0,0])+binascii.crc_hqx(bytes([170,87,0,0]),65535).to_bytes(2,'little')
assert bytes.fromhex('aa570600e577')==bytes([170,87,6,0])+binascii.crc_hqx(bytes([170,87,6,0]),65535).to_bytes(2,'little')
check('READY/BUSY/DONE/FAULT 6-byte binary, matching sequence, CCITT-FALSE and both supplied READY vectors')
b=Board();b.control=True
for _ in range(60):b.step()
assert not b.feedback # no torque-off READY even if test mode was historically enabled
b.stand();b.step()
for _ in range(65):b.step()
rows=[r for r in b.feedback if r['state']==0];assert len(rows)>=4 and all(r['seq']==0 for r in rows)
assert all(y['t']-x['t']==200 for x,y in zip(rows,rows[1:]))
check('No false READY at unprepared boot; periodic READY every200ms after start pose')
for status,angle in [(1,0),(2,200),(2,-200),(3,0),(3,900),(3,-900),(4,0)]:
 b=ready();b.feed(control(6,status,angle));b.step();assert b.feedback[-1]['state']==1 and b.feedback[-1]['seq']==6
 b.feed(control(7,5));b.done(6);assert settled(b)
 dones=[r for r in b.feedback if r['state']==2];assert len(dones)==3 and all(r['seq']==6 for r in dones)
 assert not any(r['state']==3 for r in b.feedback)
 for _ in range(80):b.step()
 assert settled(b) and b.feedback[-1]['state']==0 and b.feedback[-1]['seq']==0
 assert all(r['frame'] in (11,15) for r in dones)
 # unchanged original durations/holds for every executed non-startup frame
 starts=[r for r in b.trace if r['t']==r['begin'] and r['frame']!=0]
 for x,y in zip(starts,starts[1:]):
  f=FRAMES[x['frame']];assert y['t']-x['t']==f['duration_ms']+f['hold_ms']
 b.feed(control(8,3));b.step();b.done(8);assert settled(b)
check('All correction modes: BUSY sameSEQ -> landed DONE sameSEQ -> periodic READY; WAIT does not restart; next request accepted; unchanged frame timing')
for cmd in ('s','h','x'):
 b=ready();b.feed(control(10,3));b.step();b.until(lambda:b.value('s_control_phase')==2)
 b.call('HandleCommand',ord(cmd))
 if cmd!='x':b.until(lambda:settled(b))
 for _ in range(60):b.step()
 assert not any(r['state'] in (2,3) for r in b.feedback)
 if cmd!='x':assert b.feedback[-1]['state']==0
 else:assert b.value('s_state')==0
 assert b.call('VisionControl_IsEnabled')==0
check('Manual s/h: no falseFAULT/no fabricatedDONE, landed READY even with manual lock; x torqueOFF does not claim READY')
for fault in ('write','stall','read'):
 b=ready()
 if fault=='read':b.read_fail=True
 b.feed(control(11,3));b.step()
 if fault!='read':
  b.until(lambda:b.value('s_control_phase')==2)
  if fault=='write':b.write_fail=True;b.step()
  else:b.step(300)
 b.step();assert b.value('s_state')==4 and b.feedback[-1]['state']==3
 assert b.feedback[-1]['seq']==11
 assert not any(r['state']==2 for r in b.feedback)
 b.write_fail=False;b.read_fail=False
 b.call('HandleCommand',ord('s'));b.until(lambda:settled(b));b.step()
 assert b.feedback[-1]['state']==0
check('Real write/scheduler/start-scan safety failures reportFAULT with requestSEQ; manual recovery to start pose restoresREADY')
b=ready();b.tx_fail=True;b.feed(control(15,3));b.step()
b.until(lambda:settled(b));b.step();assert b.value('s_feedback_state')==2
b.tx_fail=False;b.done(15);assert settled(b)
check('TX failure retries without blocking movement; static TX buffer and DONE ordering retained')
# Straight request takes original gait to next planned landing before READY.
b=ready();b.feed(control(40,1));b.step();assert b.value('s_state')==3
n=len(b.feedback)
b.done(40);assert settled(b)
assert not any(r['state']==0 for r in b.feedback[n:])
for _ in range(25):b.step()
assert b.feedback[-1]['state']==0
# Shape events still complete; no READY during actual gesture motion/holds.
from test_vision import packet
for shape in range(1,7):
 b=ready();b.feed(packet(1,1,shape)+control(41,3));b.step()
 b.until(lambda:b.value('s_completed_shapes',4)==1,30000)
 assert b.value('s_state')!=4
check('Straight gait emitsREADY only at planned landing; all6 independent shape events complete')
# Reproduce frame11 restart failure: feedback pause must not use manual 14-servo scan.
b=ready();b.feed(control(50,1));b.step();b.done(50)
for _ in range(25):b.step()
assert b.call('Robot_IsFeedbackPaused')==1
b.read_fail=True;b.feedback_follows=False;b.measured[5]+=100
b.feed(control(51,3));b.step();b.done(51)
assert settled(b) and not any(r['state']==3 for r in b.feedback)
check('Feedback pause resumes existing trajectory despite unavailable position reads; no repeated manual-start scan or falseFAULT')
old=json.loads((ROOT/'Validation/feedback_ready_data_hashes.json').read_text())
for path,digest in old.items():assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest,path
check('All source action JSON, generated gait/shape tables and tuning config byte-identical')
(ROOT/'Validation/feedback_ready_test_results.json').write_text(json.dumps(dict(image_sha256=hashlib.sha256(IMAGE.read_bytes()).hexdigest(),passed=results,scope='Actual ARM with mocked UART/servos; no hardware run'),indent=2),encoding='utf-8')
