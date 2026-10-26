"""Replay actual ARM image against the supplied fddebug; no physical hardware."""
import sys,json,hashlib
from pathlib import Path
DEBUGGER=Path(r"D:/Desktop/fddebug (2)/fddebug")
sys.path.insert(0,str(DEBUGGER))
from servo_core import curve,servo_timing
from test_firmware import Firmware,ROOT,IMAGE,FRAMES
results=[]
for step,profile in ((1,0),(7,0),(1,1),(7,1)):
    b=Firmware();b.stand();b.trace.clear()
    assert b.call('Robot_StartActions',profile,0)==1
    begin=b.tick
    while b.tick-begin < 6000:
        b.step(step)
        assert b.value('s_state') != 5
    key=None;count=0;first=[]
    for r in b.trace:
        new=(r['frame'],r['begin'])
        if new!=key:
            key=new;start={i+1:p for i,p in enumerate(r['pos'])};pending=set(range(1,15));first.append(r)
        f=FRAMES[r['frame']];elapsed=r['t']-r['begin']
        expected={i:p for i,p in curve(f,start,elapsed).items() if i in pending}
        assert r['ids']==list(expected),(step,new,elapsed,r['ids'],list(expected))
        assert r['sent']==list(expected.values()),(step,new,elapsed,r['sent'],expected)
        assert (r['speed'],r['acc'])==(f['speed'],f['acc'])
        pending.difference_update(i for i in expected if elapsed>=sum(servo_timing(f,i)))
        count+=1
    assert [r['frame'] for r in first[:12]]==[10,11,13,14,15,17]*2
    if step==1:
        assert first[6]['t']-first[0]['t']==2700
        for r in b.trace: assert (r['t']-r['begin'])%10==0
    results.append({'poll_ms':step,'profile':profile,'packets_compared':count})
(ROOT/'Validation/fddebug_5_1_results.json').write_text(json.dumps({'image_sha256':hashlib.sha256(IMAGE.read_bytes()).hexdigest(),'reference':str(DEBUGGER/'servo_core.py'),'results':results,'scope':'Actual ARM replay with mocked servo bus; physical timing/stability unverified'},indent=2),encoding='utf-8')
print('PASS: exact fddebug target positions, ID subsets, speed/ACC; 2700 ms loop; 1 ms and 7 ms polling.',results)
