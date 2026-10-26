"""Image heading, positive = lane right / rightward correction, not body yaw."""
import math

class AngleTracker:
    def __init__(self, enter=8.0, leave=4.0, confirm=.2, tau=.35, zero=0.0):
        self.enter, self.leave, self.confirm = enter, leave, confirm
        self.tau, self.zero = tau, zero
        self.reset()

    def reset(self):
        self.history = []
        self.filtered = self.last = None
        self.active = self.pending = 0
        self.since = None

    def update(self, raw, timestamp):
        if raw is None or not math.isfinite(raw):
            self.reset()
            return dict(valid=False, active=False, angle=0.0, direction=0)
        if self.last is not None and timestamp-self.last > .25:
            self.reset()
        self.history = [(t,v) for t,v in self.history if timestamp-t <= .1]
        self.history.append((timestamp,raw-self.zero))
        values = sorted(v for _,v in self.history)
        n=len(values)
        median=(values[(n-1)//2]+values[n//2])*.5
        if self.filtered is None:
            self.filtered=median
        else:
            self.filtered += (1-math.exp(-(timestamp-self.last)/self.tau))*(median-self.filtered)
        self.last=timestamp
        angle=self.filtered
        direction=1 if angle>0 else -1
        if self.active and (abs(angle)<=self.leave or direction!=self.active):
            self.active=0
        candidate=direction if abs(angle)>self.enter else 0
        if self.active:
            self.pending=0
            self.since=None
        elif not candidate:
            self.pending=0
            self.since=None
        elif candidate!=self.pending:
            self.pending=candidate
            self.since=timestamp
        elif timestamp-self.since>=self.confirm:
            self.active=candidate
            self.pending=0
            self.since=None
        return dict(valid=True, active=bool(self.active), angle=angle, direction=self.active)
