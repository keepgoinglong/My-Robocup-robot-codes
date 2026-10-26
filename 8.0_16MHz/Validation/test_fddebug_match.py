"""Compare emitted ARM subsets directly against the user's fddebug curve function."""
import sys,json,hashlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'fddebug'))
from servo_core import curve,servo_timing
from test_actions11 import Board,ROOT,FRAMES,IMAGE
b=Board();b.start()
b.until(lambda:len([r for r in b.trace if r['frame']==17 and r['t']==r['begin']])>=4)
key=None
for r in b.trace:
    if r['frame']<10:continue
    new=(r['frame'],r['begin'])
    if new!=key:
        key=new;start={i+1:p for i,p in enumerate(r['pos'])};pending=set(range(1,15))
    f=FRAMES[r['frame']];elapsed=r['t']-r['begin']
    expected={i:p for i,p in curve(f,start,elapsed).items() if i in pending}
    assert r['ids']==list(expected), (new,elapsed,r['ids'],list(expected))
    assert all(abs(p-expected[i])<=1 for i,p in zip(r['ids'],r['sent']))
    pending.difference_update(i for i in expected if elapsed>=sum(servo_timing(f,i)))
    if r['frame']==17 and elapsed==10:assert r['pos']!=[f['positions'][str(i)] for i in range(1,15)]
(ROOT/'Validation/fddebug_match_results.json').write_text(json.dumps(dict(
    image_sha256=hashlib.sha256(IMAGE.read_bytes()).hexdigest(),
    passed=['Every transmitted ID subset and target matches actual fddebug curve/pending algorithm',
            'Delayed IDs absent before start; completed IDs not resent; first step interpolates'],
    scope='Ideal10ms mocked ARM schedule; rounding difference<=1 count; real UART timing and mechanics unverified'),indent=2))
print('PASS: actual fddebug function matches emitted ARM IDs/positions at every frame tick.')
