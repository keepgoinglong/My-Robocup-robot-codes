from pathlib import Path
import hashlib,json
from action_model import DATA,LOOP,ROOT
frames=DATA['frames']
sequence=[0,17,10,11,13,14,15,17]
pairs=list(zip(sequence,sequence[1:]))
rows=[]
for a,b in pairs:
 f=frames[b]
 delta=[f['positions'][str(i)]-frames[a]['positions'][str(i)] for i in range(1,15)]
 rows.append(dict(from_frame=a,to_frame=b,name=f['name'],max_delta=max(map(abs,delta)),duration_ms=f['duration_ms'],hold_ms=f['hold_ms'],timing=f.get('timing',{})))
cycle=sum(frames[i]['duration_ms']+frames[i]['hold_ms'] for i in LOOP)
assert cycle==2800
report=dict(source_sha256=hashlib.sha256((ROOT/'Actions/my_actions6.json').read_bytes()).hexdigest(),loop_slots=LOOP,original_cycle_ms=cycle,transitions=rows,limitations='Command timing only; physical tracking, balance and speed have not been measured.')
(ROOT/'Validation/trajectory_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
lines=['# 2.1 my_actions6 直走时序核算','','起步帧0采用左前落地姿势；旧起步1～9已移除。走路从17衔接10、11、13、14、15并循环；所有转向仅改变髋。','','| 过渡 | 动作 | 最大位移/计数 | 运动/ms | 停留/ms |','|---|---|---:|---:|---:|']
for r in rows:lines.append(f"| {r['from_frame']}→{r['to_frame']} | {r['name']} | {r['max_delta']} | {r['duration_ms']} | {r['hold_ms']} |")
lines+=['','直走循环2800ms/两步，名义1.4秒/步，不含图形动作及调度开销。','右脚前摆ID9延迟30ms，再用470ms完成插值；其余关节按该帧600ms执行。','保留JSON停留；纠偏完成不额外等到位、稳定或观察。图形保持仍为三秒。','以上核对指令轨迹和时序，不能证明实机稳定性、触地或实际移动速度。']
(ROOT/'关键帧与时序核算.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print('Audit: 6 straight frames, 2800 ms cycle.')
