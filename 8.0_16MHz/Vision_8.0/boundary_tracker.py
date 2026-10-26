"""Sparse continuous boundaries and one authoritative steering decision.
No image-wide processing. Reuses eleven rows; raw angle is image geometry.
"""
import math
from line_tracker import quadratic

# 直道右纠偏参数：左边线横坐标/画宽；更早介入，退回退出线才撤销。
RIGHT_ENTER = .22
RIGHT_EXIT = .18
RIGHT_URGENT = .38
RIGHT_CONFIRM = .08
RIGHT_SINGLE_CONFIRM = .12
WIDE_ROAD_MIN = .58
WIDE_ROAD_HISTORY_SECONDS = 1.0
WIDE_ROAD_MIN_SAMPLES = 5
NARROW_ROAD_CONFIRM_SECONDS = .45

class BoundaryTracker:
    def __init__(self, threshold=70, **kwargs):
        self.threshold=threshold
        self.counterclockwise=kwargs.get('counterclockwise',True)
        self.turn_memory_seconds=kwargs.get('turn_memory_seconds',7.0)
        self.rows=[]
        self.raw_image_angle=None
        self.last_time=None
        self.last_edges={}
        self.width_fit=None
        self.width_time=None
        self.last_full_time=None
        self.full_rows={}
        self.offset=self.heading=self.curvature=0.0
        self.state='LOST'
        self.mode='LOST'
        self.degraded=False
        self.pending=0
        self.pending_since=None
        self.position_direction=0
        self.edge_side=None
        self.edge_time=None
        self.threshold_used=threshold
        self.turn_memory_direction=0
        self.turn_memory_time=None
        self.turn_memory_angle=0.0
        # Road-shape classification is deliberately separate from steering.
        # Camera perspective makes even a straight lane look diagonally left.
        self.curve_active=False
        self.curve_pending=None
        self.curve_pending_since=None
        self.curve_time=None
        self.road_width_history=[]
        self.wide_road=False
        self.road_width_classified=False
        self.narrow_road_latched=False
        self.narrow_width_since=None

    def remember_turn(self,direction,timestamp,angle=None):
        """Remember only an independently confirmed turn, never memory itself."""
        if direction and (not self.counterclockwise or direction<0):
            self.turn_memory_direction=direction
            self.turn_memory_time=timestamp
            if angle is not None and abs(angle)>=4:
                self.turn_memory_angle=angle

    def _update_road_shape(self,evidence,timestamp,angle=None):
        """Confirm -1=left curve or 0=straight; None means insufficient view."""
        if evidence is None:
            self.curve_pending=None
            self.curve_pending_since=None
            return
        if evidence!=self.curve_pending:
            self.curve_pending=evidence
            self.curve_pending_since=timestamp
            return
        confirm=.16 if evidence<0 else .30
        if timestamp-self.curve_pending_since<confirm:return
        if evidence<0:
            self.curve_active=True
            self.curve_time=timestamp
            self.remember_turn(-1,timestamp,angle)
        else:
            self.curve_active=False
            self.curve_time=None
            self.turn_memory_direction=0
            self.turn_memory_time=None

    def _scan(self,get_pixel,y,width):
        values=[get_pixel(x,y) for x in range(width)]
        # A few samples estimate background without sorting the entire row.
        sample=sorted(values[::8])
        bg=sample[int(len(sample)*.7)]
        cutoff=0 if self.threshold==0 else min(110,max(35,int(bg*.65)))
        self.threshold_used=cutoff
        runs=[];start=None
        for x in range(width+1):
            dark=x<width and values[x]<=cutoff
            if dark and start is None:start=x
            elif not dark and start is not None:
                size=x-start
                if 2<=size<=width*.18:runs.append(((start+x-1)*.5,size))
                start=None
        return runs

    def _chains(self, rows, width):
        chains=[]
        for y,runs in rows:
            used=set()
            # Existing long paths get first choice; line-width continuity also matters.
            chains.sort(key=len,reverse=True)
            for chain in chains:
                py,px,pw=chain[-1]
                if y-py>18:continue
                slope=0
                if len(chain)>1:
                    y0,x0,_=chain[-2];slope=(px-x0)/(py-y0)
                expected=px+max(-1.2,min(1.2,slope))*(y-py)
                options=[(abs(x-expected)+abs(w-pw)*.4,i) for i,(x,w) in enumerate(runs)
                         if i not in used and abs(x-expected)<width*.12]
                if options:
                    _,i=min(options);x,w=runs[i];chain.append((y,x,w));used.add(i)
            for i,(x,w) in enumerate(runs):
                if i not in used and len(chains)<16:chains.append([(y,x,w)])
            chains=[c for c in chains if y-c[-1][0]<=18]
        strong=[]
        for chain in chains:
            if len(chain)<7 or chain[-1][0]-chain[0][0]<42:continue
            fit=quadratic([((y/120-.49)/.33,x) for y,x,_ in chain])
            if fit is None:continue
            a,b,c=fit
            error=sum(abs(x-(a*((y/120-.49)/.33)**2+b*((y/120-.49)/.33)+c)) for y,x,_ in chain)/len(chain)
            if error>2.5:continue
            strong.append((chain,fit,error))
        strong.sort(key=lambda e:(-len(e[0]),e[2]))
        return strong[:4]

    def update(self,get_pixel,width,height,timestamp,run_provider=None):
        if self.last_time is not None and timestamp<=self.last_time:raise ValueError('timestamps must increase')
        dt=0 if self.last_time is None else timestamp-self.last_time
        self.last_time=timestamp
        if dt>.25:
            self.pending=0;self.pending_since=timestamp;self.position_direction=0
        previous_mode=self.mode
        self.rows=[];self.raw_image_angle=None
        ys=[int(height*(.24+i*.056)) for i in range(11)]
        sampled=[(y,[(x,3) for x in run_provider(y)] if run_provider else self._scan(get_pixel,y,width)) for y in ys]
        edges=self._chains(sampled,width)
        def at(f,t):return f[0]*t*t+f[1]*t+f[2]
        paired=None
        for i,e in enumerate(edges):
            for other in edges[i+1:]:
                l,r=sorted((e,other),key=lambda z:at(z[1],.8))
                widths=[at(r[1],t)-at(l[1],t) for t in [-.5,0,.8]]
                if min(widths)<width*.30 or max(widths)>width*1.4:continue
                # Perspective normally widens toward feet; avoid internal marker pairs.
                if widths[-1]<widths[0]*.7:continue
                score=len(l[0])+len(r[0])+sum(widths)/width
                if paired is None or score>paired[0]:paired=(score,l,r)
        fit=None;near_left=near_right=None;confidence=0;source='NONE'
        single_curve_direction=0
        if paired:
            _,l,r=paired
            fit=tuple((a+b)*.5 for a,b in zip(l[1],r[1]))
            self.width_fit=tuple(b-a for a,b in zip(l[1],r[1]))
            self.width_time=timestamp
            self.last_full_time=timestamp
            self.last_edges={'LEFT':(l[1],timestamp),'RIGHT':(r[1],timestamp)}
            self.mode='BOTH';self.degraded=False
            near_left=at(l[1],.8);near_right=at(r[1],.8)
            near_width=near_right-near_left
            self.road_width_history=[(t,w) for t,w in self.road_width_history
                                     if timestamp-t<=WIDE_ROAD_HISTORY_SECONDS]
            self.road_width_history.append((timestamp,near_width))
            if not self.road_width_classified and len(self.road_width_history)>=WIDE_ROAD_MIN_SAMPLES:
                ordered=sorted(w for _,w in self.road_width_history)
                # 下四分位可排除摆动中偶发的宽配对；窄道不能套用直道早右纠。
                self.wide_road=ordered[len(ordered)//4]>=width*WIDE_ROAD_MIN
                self.narrow_road_latched=not self.wide_road
                self.road_width_classified=True
            elif self.road_width_classified and not self.narrow_road_latched:
                # 宽道进入窄道时要持续变窄才切换；一次摆动或错误配对不能误锁窄道。
                if near_width<width*WIDE_ROAD_MIN:
                    if self.narrow_width_since is None:self.narrow_width_since=timestamp
                    elif timestamp-self.narrow_width_since>=NARROW_ROAD_CONFIRM_SECONDS:
                        self.narrow_road_latched=True
                        self.wide_road=False
                else:
                    self.narrow_width_since=None
            confidence=min(len(l[0]),len(r[0]))/11
            self.rows=[(y,at(l[1],(y/height-.49)/.33),at(r[1],(y/height-.49)/.33)) for y in ys]
            self.full_rows={y:(l,r) for y,l,r in self.rows}
            source='PAIRED'
        elif edges:
            edge,ef,error=edges[0];near=at(ef,.8)
            # Dashes are supporting evidence only, outside the candidate on its left.
            dash_rows=sum(any(4<at(ef,(y/height-.49)/.33)-x<width*.28 and w<=4 for x,w in runs) for y,runs in sampled)
            side=None
            old=[]
            for name,(old_fit,stamp) in self.last_edges.items():
                if timestamp-stamp<.6:old.append((abs(near-at(old_fit,.8)),name))
            if old and min(old)[0]<width*.20:side=min(old)[1]
            if side is None and self.edge_time is not None and timestamp-self.edge_time<.3:
                if abs(near-self.edge_near)<width*.15:side=self.edge_side
            if side is None:
                if 2<=dash_rows<=8:side='LEFT'
                elif near<width*.40:side='LEFT'
                elif near>width*.60:side='RIGHT'
            self.degraded=True
            self.mode='SINGLE_'+side if side else 'UNCERTAIN'
            confidence=min(.65,len(edge)/11*.65) if side else .15
            if side:
                self.edge_side,self.edge_time,self.edge_near=side,timestamp,near
                self.last_edges[side]=(ef,timestamp)
                if side=='LEFT':near_left=near
                else:near_right=near
                # Width is used only while recent; never report an old angle as measured.
                if self.width_time is not None and timestamp-self.width_time<=.8:
                    sign=.5 if side=='LEFT' else -.5
                    fit=tuple(a+sign*b for a,b in zip(ef,self.width_fit))
                    source='ESTIMATED'
                # On this counter-clockwise course either edge can prove a
                # left curve. Thresholds differ because the inner/left edge is
                # seen earlier. Recorded straight left edges have negative b;
                # recorded curved left edges have a<=-3 and positive b.
                a,b,c=ef
                left_curve=(side=='RIGHT' and a<=-2 and b>=15) or \
                           (side=='LEFT' and a<=-3 and b>=2)
                if self.counterclockwise and left_curve:
                    single_curve_direction=-1
                    single_angle=math.degrees(math.atan(-b/(.33*height)))
                    self.remember_turn(-1,timestamp,
                                       self.turn_memory_angle or single_angle)
                self.rows=[(y,x,x) for y,x,w in edge]
        else:
            self.mode='LOST';self.degraded=True
        valid=self.mode not in ('LOST','UNCERTAIN')
        if fit:
            a,b,c=fit
            self.raw_image_angle=math.degrees(math.atan(-b/(.33*height)))
            alpha=1 if dt<=0 or dt>.25 else 1-math.exp(-dt/.35)
            offset=(at(fit,.8)-width*.5)/(width*.5)
            self.offset+=alpha*(offset-self.offset)
            self.heading=-b/(width*.5)
            self.curvature=a/(width*.5)
            # Thresholds measured from two all-straight and two all-left-curve
            # recordings. Both terms are needed: curvature rejects perspective,
            # while heading rejects small quadratic fit noise on straight edges.
            if self.curvature<=-.018 and self.heading<=-.09:
                self._update_road_shape(-1,timestamp,self.raw_image_angle)
            # Leave a confirmed curve only when the fitted edge itself becomes
            # nearly straight. Heading alone is perspective-dependent.
            elif self.curvature>=-.012:
                self._update_road_shape(0,timestamp,self.raw_image_angle)
            else:
                self._update_road_shape(None,timestamp)
        elif single_curve_direction:
            self._update_road_shape(-1,timestamp,self.turn_memory_angle)
        else:
            self._update_road_shape(None,timestamp)
        # Position escape has precedence over image-heading correction.
        aggressive_right=self.wide_road and self.mode=='BOTH'
        right_enter=RIGHT_ENTER if aggressive_right else .34
        right_exit=RIGHT_EXIT if aggressive_right else .28
        wanted=0
        if valid:
            if self.mode=='SINGLE_LEFT' and near_left>width*(right_exit if self.position_direction==1 else right_enter):wanted=1
            elif self.mode=='SINGLE_RIGHT' and near_right<width*.72:wanted=-1
            elif self.mode=='BOTH':
                if near_left>width*(right_exit if self.position_direction==1 else right_enter):wanted=1
                elif near_right<width*.66:wanted=-1
                elif self.position_direction==1 and near_left>width*right_exit:wanted=1
                elif self.position_direction==-1 and near_right<width*.72:wanted=-1
        # 同方向单双边切换不重置右纠确认；丢线/反向仍重置。
        if self.mode!=previous_mode and not (aggressive_right and wanted==1 and self.pending==1):
            self.position_direction=0
            self.pending=0
            self.pending_since=timestamp
        if wanted!=self.pending:
            self.pending=wanted;self.pending_since=timestamp
        if not valid or not wanted:self.position_direction=0
        elif wanted==1:
            confirm=RIGHT_CONFIRM if aggressive_right else (.4 if self.degraded else .2)
            if timestamp-self.pending_since>=confirm:self.position_direction=1
        elif timestamp-self.pending_since>=(.4 if self.degraded else .2):self.position_direction=wanted
        urgent_right=bool(aggressive_right and valid and near_left is not None and near_left>width*RIGHT_URGENT
                          and self.position_direction==1)
        remembered_direction=0
        if (valid and self.mode=='SINGLE_RIGHT' and
                self.turn_memory_direction<0 and self.turn_memory_time is not None and
                timestamp-self.turn_memory_time<=self.turn_memory_seconds):
            remembered_direction=self.turn_memory_direction
        curve_direction=0
        if (self.curve_active and self.curve_time is not None and
                timestamp-self.curve_time<=self.turn_memory_seconds):
            curve_direction=-1
        self.state=('RIGHT' if self.position_direction>0 else 'LEFT') if self.position_direction else 'STRAIGHT' if valid else 'LOST'
        # 正误差=靠左，要减少左转/向右纠偏；负误差=靠右，要增加左转。
        # 边线位置换算为控制等效角，不是相机朝向角。安全区内误差为0。
        position_error_deg=0.0
        if valid:
            if self.mode in ('BOTH','SINGLE_LEFT') and near_left>width*right_exit:
                position_error_deg=min(15.0,(near_left/width-right_exit)*60.0)
            elif self.mode in ('BOTH','SINGLE_RIGHT') and near_right<width*.72:
                position_error_deg=max(-15.0,(near_right/width-.72)*60.0)
        return dict(valid=valid,stop=not valid,state=self.state,confidence=confidence,
                    position_error_deg=position_error_deg,urgent_right=urgent_right,
                    wide_road=self.wide_road,aggressive_right=aggressive_right,
                    degraded=self.degraded,offset=self.offset,heading=self.heading,curvature=self.curvature,
                    steer=self.position_direction,mode=self.mode,angle_source=source,
                    position_direction=self.position_direction,
                    remembered_direction=remembered_direction,
                    remembered_angle=self.turn_memory_angle,
                    single_curve_direction=single_curve_direction,
                    curve_direction=curve_direction,
                    road_shape='LEFT_CURVE' if curve_direction else 'STRAIGHT')


def resolve_control(lane,angle):
    """Both outgoing packets and the display use this same decision."""
    out=dict(angle)
    position=lane.get('position_direction',0)
    if not lane['valid'] and lane.get('curve_direction',0):
        # A short occlusion inside an already confirmed left curve must not
        # cancel the turn. This cannot start a turn from a straight/lost frame.
        direction=lane['curve_direction']
        remembered_angle=lane.get('remembered_angle',0.0)
        out.update(valid=True,active=True,direction=direction,angle=remembered_angle,
                   status=3,
                   reason='LOSS_HOLD')
    elif not lane['valid']:
        out.update(valid=False,active=False,direction=0,angle=0.0,status=0,reason='NO_EDGE')
    elif position>0 and lane.get('urgent_right',False):
        # 已确认贴近左边线：双边可见时也优先右避边。
        out.update(valid=True,active=True,direction=1,angle=0.0,status=4,reason='POSITION')
    elif position and lane.get('mode')!='BOTH':
        # With only one edge visible, escaping the nearby boundary is safest.
        out.update(valid=True,active=True,direction=position,angle=0.0,
                   status=4 if position>0 else 3,reason='POSITION')
    elif lane.get('curve_direction',0):
        direction=lane['curve_direction']
        curve_angle=angle.get('angle',0.0) if angle.get('valid') else lane.get('remembered_angle',0.0)
        out.update(valid=True,active=True,direction=direction,angle=curve_angle,
                   status=3,reason='CURVE')
    elif position:
        out.update(valid=True,active=True,direction=position,angle=0.0,
                   status=4 if position>0 else 3,reason='POSITION')
    elif lane.get('remembered_direction',0):
        direction=lane['remembered_direction']
        remembered_angle=lane.get('remembered_angle',0.0)
        out.update(valid=True,active=True,direction=direction,angle=remembered_angle,
                   status=3,
                   reason='TURN_MEMORY')
    else:
        # A straight road uses only lateral position correction. Its perspective
        # angle must never be converted into a turn command.
        out.update(valid=True,active=False,direction=0,angle=0.0,status=1,reason='STRAIGHT')
    # 弯道、转弯记忆和短时丢线必须保留真实转角；只有位置避边使用位置误差角。
    if out.get('reason')=='POSITION' and out['status'] in (3,4) and 'position_error_deg' in lane:
        out['angle']=lane['position_error_deg'] if out['status']==3 else max(0.0,lane['position_error_deg'])
    lane=dict(lane)
    if out.get('reason')=='LOSS_HOLD':
        lane['valid']=True;lane['stop']=False
    lane['state']=('RIGHT' if out['direction']>0 else 'LEFT') if out['active'] else 'STRAIGHT' if lane['valid'] else 'LOST'
    return lane,out
