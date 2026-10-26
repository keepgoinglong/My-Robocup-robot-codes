"""Actual ARM instruction replay; hardware and contact are mocked."""
import copy,hashlib,json
from test_feedback import Board as BaseBoard,control
from test_vision import packet
from test_firmware import ROOT,IMAGE,FRAMES,TARGETS

class Board(BaseBoard):
    def step(self,ms=10):
        self.call('SG90_Task',(self.tick+ms)&0xffffffff)
        super().step(ms)
    def start(self):
        self.stand();self.control=True
        self.feed(packet(0,0,0));self.feed(control(0,5));self.step()
        self.call('HandleCommand',ord('g'))
        assert self.call('VisionControl_IsEnabled')==1
        self.step()

raw=(ROOT/'Actions/my_actions8_original.json').read_bytes()
source=json.loads(raw.decode('utf-8-sig'))
expected=copy.deepcopy(source)
for index in (1,4):expected['frames'][index]['hold_ms']=300
assert json.loads((ROOT/'Actions/my_actions8.json').read_text(encoding='utf-8-sig'))==expected
slots=[17,10,11,13,14,15]
for i,slot in enumerate(slots):assert FRAMES[slot]==expected['frames'][i]
assert TARGETS[0]==TARGETS[15]
b=Board();b.start()
b.until(lambda:len([r for r in b.trace if r['frame']>=10 and r['t']==r['begin']])>=19)
starts=[r for r in b.trace if r['frame']>=10 and r['t']==r['begin']]
assert [r['frame'] for r in starts[:18]]==slots*3
assert starts[6]['t']-starts[0]['t']==2400
origin=None;key=None
for r in b.trace:
    f=FRAMES[r['frame']]
    assert r['duration']==f['duration_ms']
    assert (r['speed'],r['acc'])==(f['speed'],f['acc'])
    new=(r['frame'],r['begin'])
    if new!=key:origin=r['pos'];key=new
    elapsed=r['t']-r['begin']
    if r['frame'] in (10,14,11,15):
        for sid in range(1,15):
            t=f.get('timing',{}).get(str(sid),{})
            delay=t.get('delay_ms',0);duration=t.get('duration_ms',f['duration_ms'])
            x=min(max(elapsed-delay,0)/duration,1);q=x*x*x*(10+x*(-15+6*x))
            target=origin[sid-1]+(f['positions'][str(sid)]-origin[sid-1])*q
            assert abs(r['pos'][sid-1]-target)<=0.501
    if elapsed==r['duration']:assert r['pos']==TARGETS[r['frame']]
for a,z in zip(starts,starts[1:]):
    f=FRAMES[a['frame']];assert z['t']-a['t']==f['duration_ms']+f['hold_ms']

# Every landing-dependent shape is checked at both landing poses.
for shape in (1,2,5,6):
    for slot,expected_landing in ((17,11),(13,15)):
        b=Board();b.start();b.until(lambda:b.value('s_frame')==slot)
        assert b.call('Robot_RequestShape',shape)==1
        b.until(lambda:b.value('s_shape_phase') in (1,5))
        assert b.value('s_frame')==expected_landing
        assert b.measured[4:]==TARGETS[expected_landing][4:]
        if shape!=6:
            b.until(lambda:b.value('s_stage')==2)
            t=b.value('s_stage_begin',4)
            b.until(lambda:b.value('s_shape_phase')!=1)
            assert b.tick-t==3000
        else:
            t=b.tick
            b.until(lambda:b.value('s_shape_phase')!=5)
            assert b.tick-t==3400  # 3s shake plus existing400ms centering
        b.until(lambda:b.value('s_completed_shapes',4)==1)
        assert all(r['pos'][4:]==TARGETS[expected_landing][4:] for r in b.trace if r['phase'] in (1,2,5))

for shape in (3,4):
    b=Board();b.start();b.call('Robot_RequestShape',shape)
    b.until(lambda:b.value('s_shape_phase')==3)
    assert b.value('s_frame')==(14 if shape==3 else 10)
    b.until(lambda:b.value('s_stage')==2);t=b.value('s_stage_begin',4)
    b.until(lambda:b.value('s_shape_phase')!=3);assert b.tick-t==3000

for status,angle in ((1,0),(2,200),(2,-200),(3,0),(3,900),(3,-900),(4,0)):
    b=Board();b.start();b.feed(control(40,status,angle));b.step();b.done(40)
    assert b.value('s_state')==3
for slot in slots:
    for cmd in ('s','h'):
        b=Board();b.start();b.until(lambda:b.value('s_frame')==slot)
        b.call('HandleCommand',ord(cmd));b.until(lambda:b.value('s_state') in (2,5))
        assert b.value('s_frame') in (11,15)
        assert b.measured==TARGETS[b.value('s_frame')]
        if cmd=='s':assert b.measured==TARGETS[15]
        assert b.call('VisionControl_IsEnabled')==0

report=dict(image_sha256=hashlib.sha256(IMAGE.read_bytes()).hexdigest(),source_sha256=hashlib.sha256(raw).hexdigest(),
            loop_ms=2400,passed=['Only frames2/5 holds changed200->300; sixth-frame start; three1..6 loops',
            'All positions/speeds/ACC/durations/holds; rightID9 delay200/duration100; leftID7 delay250/duration50',
            'All landing joints restored to original quintic curves; cubic experiment removed',
            'Shapes1/2/5/6 use both landing frames; fixed legs; original holds',
            'Shapes3/4 swing holds; AA56 modes; six-phase s/h stop; g auto vision'],
            scope='ARM instruction replay with mocked clock/UART/servo/PWM; no physical clearance/stability validation')
(ROOT/'Validation/actions8_test_results.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('PASS actual ARM: swing holds300ms; 2400ms loop; sixth-frame start; all shapes and both landings; control/stop.')
