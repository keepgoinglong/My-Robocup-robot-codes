"""PC回放：共用板端分类器/事件状态机，OpenCV只模拟原生连通域。"""
from pathlib import Path
import argparse,json,time
import cv2
from shape_tracker import CardRecognizer,ShapeEvents,CANDIDATE_THRESHOLDS,candidate_ok

def candidates(gray):
    boxes=[]
    for threshold in CANDIDATE_THRESHOLDS:
        _,_,stats,_=cv2.connectedComponentsWithStats((gray<=threshold).astype('uint8'),8)
        for x,y,w,h,area in stats[1:]:
            box=tuple(map(int,(x,y,w,h)))
            if area>=65 and w*h>=1400 and candidate_ok(box):boxes.append((box,threshold))
    return boxes

def replay(path,out):
    cap=cv2.VideoCapture(str(path));fps=cap.get(5)
    if not cap.isOpened():raise ValueError(str(path))
    recognizer=CardRecognizer();events=ShapeEvents();index=0;records=[];emitted=[];cost=[]
    while True:
        ok,frame=cap.read()
        if not ok:break
        now=index/fps;index+=1
        if (index-1)%3:continue
        gray=cv2.cvtColor(cv2.resize(frame,(640,480)),cv2.COLOR_BGR2GRAY)
        start=time.perf_counter();d=recognizer.detect(lambda x,y:int(gray[y,x]),candidates(gray))
        cost.append((time.perf_counter()-start)*1000)
        event=events.update(d,now)
        top=max(recognizer.last_candidates,key=lambda r:r['score'],default=None)
        records.append(dict(time=now,detection=d,best=top))
        if event:
            events.mark_sent(event,now);emitted.append(dict(shape=event,time=now,score=d['score']))
            x,y,w,h=d['box'];cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(frame,'V5 SHAPE %d %.2fs score %.2f'%(event,now,d['score']),(12,150),0,.65,(0,255,0),2)
            cv2.imencode('.jpg',frame)[1].tofile(str(out/(path.stem+'_event.jpg')))
    cap.release()
    report=dict(video=str(path),frames=index,events=emitted,samples=records,pc_mean_ms=sum(cost)/max(1,len(cost)),note='PC replay; native CanMV blobs, board timing and UART require device validation')
    (out/(path.stem+'.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(path.name,'events',emitted,'detections',sum(r['detection'] is not None for r in records),'/',len(records))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('videos',nargs='+');p.add_argument('--output',default='validation');args=p.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    for video in args.videos:replay(Path(video),out)
