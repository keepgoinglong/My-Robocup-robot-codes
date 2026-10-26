"""Independent long-bar detector. It never changes lane or shape results."""
import math


class ObstacleTracker:
    def __init__(self, confirm=3, clear=3):
        self.confirm=confirm;self.clear=clear
        self.hits=0;self.misses=0;self.present=False;self.angle=0.0

    def detect(self,img):
        # Work on the original VGA grayscale frame. Road borders are near vertical;
        # cards/cross shapes are too short. The obstacle must be a long near-horizontal bar.
        lines=img.find_lines(roi=(32,180,576,260),threshold=1500,
                             theta_margin=20,rho_margin=20)
        best=None
        for line in lines:
            dx=line.x2()-line.x1();dy=line.y2()-line.y1()
            length=(dx*dx+dy*dy)**0.5
            if length<260 or abs(dx)<1:continue
            angle=math.degrees(math.atan2(dy,dx))
            if angle>90:angle-=180
            if angle<-90:angle+=180
            if abs(angle)>35:continue
            middle=(line.y1()+line.y2())*.5
            if middle<190 or middle>430:continue
            score=length-abs(angle)*2
            if best is None or score>best[0]:best=(score,angle,(line.x1(),line.y1(),line.x2(),line.y2()))
        return None if best is None else dict(angle=best[1],line=best[2],score=best[0])

    def update(self,detection):
        if detection is None:
            self.hits=0;self.misses+=1
            if self.misses>=self.clear:self.present=False;self.angle=0.0
        else:
            self.misses=0;self.hits+=1
            # Low-pass only the bar angle; no data is fed into shape recognition.
            self.angle=detection['angle'] if self.hits==1 else self.angle*.6+detection['angle']*.4
            if self.hits>=self.confirm:self.present=True
        return dict(present=self.present,angle=self.angle if self.present else 0.0,
                    detection=detection)
