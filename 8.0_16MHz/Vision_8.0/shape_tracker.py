"""Vision5.0：外框校正 + 内部图形模板；仅用 MicroPython 标准语法。
检测像素必须来自未画标记的灰度图。OpenCV 仅用于 PC 候选框回放。
"""
import math

SHAPE_NAMES = ('none', 'circle', 'star', 'square', 'diamond', 'plus', 'triangle')
# 图形与巡线阈值独立；多档灰度覆盖暗场和反光，最终使用卡片局部对比度。
CANDIDATE_THRESHOLDS = (55, 70, 85, 100, 115, 130)
GRID = 29


def candidate_ok(box, width=640, height=480):
    x,y,w,h = box
    return (40 <= w <= 280 and 38 <= h <= 280 and .55 < w/h < 1.8
            and x >= 0 and y >= 0 and x+w <= width and y+h <= height)


def board_candidates(img):
    boxes=[]
    for threshold in CANDIDATE_THRESHOLDS:
        # 全画面搜索：图形一进入视野就参与确认，不等到画面中间。
        blobs=img.find_blobs([(0,threshold)],roi=(0,0,640,480),
                             x_stride=2,y_stride=2,pixels_threshold=65,
                             area_threshold=1400,merge=False)
        for blob in blobs:
            box=blob.rect()
            if candidate_ok(box): boxes.append((box,threshold))
    return boxes


def _point_segment(x,y,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
    return math.sqrt((x-a[0]-t*dx)**2+(y-a[1]-t*dy)**2)


def _vertices(sid,r,phase=0.):
    if sid==2:
        return [(.5+ (r if i%2==0 else r*.43)*math.cos(-math.pi/2+phase+i*math.pi/5),
                 .5+ (r if i%2==0 else r*.43)*math.sin(-math.pi/2+phase+i*math.pi/5)) for i in range(10)]
    if sid==3:return [(.5-r,.5-r),(.5+r,.5-r),(.5+r,.5+r),(.5-r,.5+r)]
    if sid==4:return [(.5,.5-r),(.5+r,.5),(.5,.5+r),(.5-r,.5)]
    if sid==6:return [(.5,.5-r),(.5+r,.5+r),(.5-r,.5+r)]
    return []


def make_templates():
    result=[]
    for sid in range(1,7):
        for radius in (.32,.36,.40):
            # 五角星旋转72度后重合；四个18度相位覆盖卡片正放、倒放和横放。
            phases=(0.,math.pi/10,math.pi/5,math.pi*3/10) if sid==2 else (0.,)
            for phase in phases:
                vertices=_vertices(sid,radius,phase)
                pts=[]
                for y in range(GRID):
                    v=y/(GRID-1)
                    for x in range(GRID):
                        u=x/(GRID-1)
                        if not (.10<u<.90 and .10<v<.90):continue
                        if sid==1:dist=abs(math.sqrt((u-.5)**2+(v-.5)**2)-radius)
                        elif sid==5:
                            dist=min(_point_segment(u,v,(.5-radius,.5),(.5+radius,.5)),
                                     _point_segment(u,v,(.5,.5-radius),(.5,.5+radius)))
                        else:dist=min(_point_segment(u,v,vertices[i-1],vertices[i]) for i in range(len(vertices)))
                        if dist<.024:pts.append(y*GRID+x)
                near=set()
                for p in pts:
                    x,y=p%GRID,p//GRID
                    for dy in (-1,0,1):
                        for dx in (-1,0,1):near.add((y+dy)*GRID+x+dx)
                result.append((sid,pts,near))
    return result


class CardRecognizer:
    def __init__(self):
        self.templates=make_templates()
        self.last_candidates=[]

    def classify(self,get_pixel,box,threshold):
        x,y,w,h=box
        # 外框是连通域的四个极值角；排除内图案不会改变极值。
        corners=[None]*4;ext=[1e9,-1e9,-1e9,1e9]
        for yy in range(y,y+h,2):
            for xx in range(x,x+w,2):
                if get_pixel(xx,yy)>threshold:continue
                a,b=xx+yy,xx-yy
                if a<ext[0]:ext[0]=a;corners[0]=(xx,yy)
                if b>ext[1]:ext[1]=b;corners[1]=(xx,yy)
                if a>ext[2]:ext[2]=a;corners[2]=(xx,yy)
                if b<ext[3]:ext[3]=b;corners[3]=(xx,yy)
        if None in corners:return None
        tl,tr,br,bl=corners
        edges=[math.sqrt((corners[i][0]-corners[i-1][0])**2+(corners[i][1]-corners[i-1][1])**2) for i in range(4)]
        if min(edges)<28 or max(edges)/min(edges)>1.65:return None
        # 四角双线性映射，补偿平面卡片旋转与中等透视。
        values=[]
        for gy in range(GRID):
            v=gy/(GRID-1)
            for gx in range(GRID):
                u=gx/(GRID-1)
                px=(1-v)*((1-u)*tl[0]+u*tr[0])+v*((1-u)*bl[0]+u*br[0])
                py=(1-v)*((1-u)*tl[1]+u*tr[1])+v*((1-u)*bl[1]+u*br[1])
                values.append(get_pixel(int(px+.5),int(py+.5)))
        ordered=sorted(values)
        dark,light=ordered[len(ordered)//10],ordered[len(ordered)*7//10]
        if light-dark<16:return None
        cut=dark+(light-dark)*.48
        # 外框四条边分别验真，机械结构、单独图案不能冒充完整卡片。
        sides=[]
        for side in range(4):
            hits=0
            for t in range(3,GRID-3):
                if side==0:ids=[d*GRID+t for d in range(3)]
                elif side==1:ids=[(GRID-1-d)*GRID+t for d in range(3)]
                elif side==2:ids=[t*GRID+d for d in range(3)]
                else:ids=[t*GRID+GRID-1-d for d in range(3)]
                hits+=min(values[p] for p in ids)<cut
            sides.append(hits/(GRID-6))
        if min(sides)<.60:return None
        observed=set(gy*GRID+gx for gy in range(3,GRID-3) for gx in range(3,GRID-3)
                     if values[gy*GRID+gx]<cut)
        if not 12<=len(observed)<=240:return None
        expanded=set()
        for p in observed:
            for dy in (-1,0,1):
                for dx in (-1,0,1):expanded.add(p+dy*GRID+dx)
        scores={}
        for sid,pts,near in self.templates:
            recall=sum(p in expanded for p in pts)/len(pts)
            precision=sum(p in near for p in observed)/len(observed)
            score=2*recall*precision/max(.001,recall+precision)
            scores[sid]=max(scores.get(sid,0),score)
        rank=sorted(scores,key=lambda sid:scores[sid],reverse=True)
        sid=rank[0];score=scores[sid];margin=score-scores[rank[1]]
        return dict(shape=sid,score=score,margin=margin,box=box,corners=corners,
                    accepted=score>=.82 and margin>=.065,scores=scores,contrast=light-dark)

    def detect(self,get_pixel,candidates):
        found=[]
        for box,threshold in candidates:
            result=self.classify(get_pixel,box,threshold)
            if result is not None:found.append(result)
        self.last_candidates=found
        accepted=[r for r in found if r['accepted']]
        if not accepted:return None
        return max(accepted,key=lambda r:r['score']+r['margin']*.3)


class ShapeEvents:
    """同一空间卡片连续确认；每个图形ID在本次运行中最多发送一次。"""
    def __init__(self):
        self.sent=set();self.queued=set()
        self.pending_id=0;self.hits=0;self.last_seen=None;self.box=None
        self.current=None

    def update(self,detection,now):
        self.current=detection
        if detection is None:
            self.pending_id=0;self.hits=0
            return 0
        sid=detection['shape'];box=detection['box']
        same=(sid==self.pending_id and self.last_seen is not None and now-self.last_seen<.65)
        if same and self.box is not None:
            same=abs(box[0]-self.box[0])+abs(box[1]-self.box[1])<max(box[2],box[3])*.8
        self.hits=self.hits+1 if same else 1
        self.pending_id=sid;self.last_seen=now;self.box=box
        if self.hits<2 or sid in self.queued or sid in self.sent:return 0
        self.queued.add(sid)
        return sid

    def mark_sent(self,sid,now):
        self.sent.add(sid);self.queued.discard(sid)
