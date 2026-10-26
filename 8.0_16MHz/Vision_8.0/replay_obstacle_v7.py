"""PC reference replay for the three supplied videos; not a K230D FPS claim."""
import cv2
import json
import math
import statistics
from pathlib import Path

VIDEOS={
    "front":Path(r"D:\Desktop\2026 cobocup 人形机器人\视频\障碍\正对2.mp4"),
    "left":Path(r"D:\Desktop\2026 cobocup 人形机器人\视频\障碍\偏左.mp4"),
    "right":Path(r"D:\Desktop\2026 cobocup 人形机器人\视频\障碍\偏右.mp4"),
}


def replay(path):
    cap=cv2.VideoCapture(str(path));total=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));angles=[]
    for index in range(total):
        ok,frame=cap.read()
        if not ok:break
        if index<total*.6:continue
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);height,width=gray.shape
        roi=gray[int(height*.38):int(height*.92),int(width*.05):int(width*.95)]
        edge=cv2.Canny(roi,35,100)
        lines=cv2.HoughLinesP(edge,1,math.pi/180,55,
                              minLineLength=int(width*.32),maxLineGap=int(width*.08))
        best=None
        if lines is not None:
            for x1,y1,x2,y2 in lines.reshape(-1,4):
                dx=int(x2)-int(x1);dy=int(y2)-int(y1)
                angle=math.degrees(math.atan2(dy,dx))
                if angle>90:angle-=180
                if angle<-90:angle+=180
                length=math.hypot(dx,dy)
                if abs(angle)<=35 and (best is None or length>best[0]):best=(length,angle)
        if best:angles.append(best[1])
    cap.release()
    return dict(frames=total,detections=len(angles),median_deg=round(statistics.median(angles),2),
                min_deg=round(min(angles),2),max_deg=round(max(angles),2))


if __name__=="__main__":
    result={name:replay(path) for name,path in VIDEOS.items()}
    out=Path(__file__).with_name("validation")/"obstacle_video_replay.json"
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False,indent=2))
