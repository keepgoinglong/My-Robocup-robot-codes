"""Small scanline lane tracker. Core uses only Python's standard library.

Input: upright grayscale frame exposing get_pixel(x, y), width, height.
Output signs: positive means right. No actuator commands are sent here.
"""
import math


def quadratic(points):
    """Fit x = a*t*t + b*t + c using a tiny pivoted linear solve."""
    sums = [sum(t ** k for t, x in points) for k in range(5)]
    rhs = [sum(x * t ** k for t, x in points) for k in range(3)]
    m = [[sums[i + j] for j in range(3)] + [rhs[i]] for i in range(3)]
    for i in range(3):
        pivot = max(range(i, 3), key=lambda j: abs(m[j][i]))
        m[i], m[pivot] = m[pivot], m[i]
        if abs(m[i][i]) < 1e-9:
            return None
        scale = m[i][i]
        m[i] = [v / scale for v in m[i]]
        for j in range(3):
            if j != i:
                scale = m[j][i]
                m[j] = [v - scale * w for v, w in zip(m[j], m[i])]
    return m[2][3], m[1][3], m[0][3]


class LaneTracker:
    def __init__(self, threshold=90, filter_seconds=0.65,
                 turn_enter=0.14, turn_exit=0.075, confirm_seconds=0.45,
                 offset_enter=0.18, offset_exit=0.10, counterclockwise=True,
                 right_recover_enter=0.30, right_recover_exit=0.14):
        self.threshold = threshold
        self.filter_seconds = filter_seconds
        self.turn_enter = turn_enter
        self.turn_exit = turn_exit
        self.confirm_seconds = confirm_seconds
        self.offset_enter = offset_enter
        self.offset_exit = offset_exit
        self.counterclockwise = counterclockwise
        self.right_recover_enter = right_recover_enter
        self.right_recover_exit = right_recover_exit
        self.last_time = None
        self.last_valid = None
        self.offset = self.heading = self.curvature = 0.0
        self.state = "LOST"
        self.pending = None
        self.pending_since = None
        self.raw_image_angle = None
        self.rows = []
        self.full_rows = {}
        self.last_full_time = None
        self.single_boundary_seconds = 0.8
        self.degraded = False

    def _runs(self, get_pixel, y, width):
        runs = []
        start = None
        for x in range(width + 1):
            dark = x < width and get_pixel(x, y) <= self.threshold
            if dark and start is None:
                start = x
            elif not dark and start is not None:
                # Ignore tiny printed ticks and broad transverse obstacles.
                if max(2, width // 100) <= x - start <= width * 0.18:
                    runs.append((start + x - 1) * 0.5)
                start = None
        return runs

    def update(self, get_pixel, width, height, timestamp, run_provider=None):
        if self.last_time is not None and timestamp <= self.last_time:
            raise ValueError("timestamps must increase")
        dt = 0.0 if self.last_time is None else timestamp - self.last_time
        self.last_time = timestamp
        self.raw_image_angle = None
        self.rows = []
        self.degraded = False
        scan_ys = [int(height * (0.82 - i * 0.066)) for i in range(10, -1, -1)]
        runs_by_y = {}
        for y in scan_ys:
            runs_by_y[y] = (run_provider(y) if run_provider else
                            self._runs(get_pixel, y, width))
        # A tight bend can leave only one physical boundary in frame for
        # seconds. Seven smooth single-edge rows are far harder for an inner
        # marker to imitate than a one-row timeout bridge.
        single_mode = (self.last_full_time is not None and
                       sum(len(runs_by_y[y]) == 1 and y in self.full_rows
                           for y in scan_ys) >= 7)
        previous = None
        # Bottom 17% excludes feet / overlay in the supplied recording.
        # Start above the feet/near marker. A false bottom-row pair must not
        # anchor the rest of the image to an interior printed shape.
        for y in scan_ys:
            runs = runs_by_y[y]
            if len(runs) > 12:  # Bound pair enumeration on textured/noisy frames.
                continue
            candidates = []
            for li, left in enumerate(runs):
                for right in runs[li + 1:]:
                    span = right - left
                    # At the closest part of a bend both boundaries can sit on
                    # the image edges, spanning almost the complete grid.
                    if not width * 0.24 < span < width * 1.01:
                        continue
                    center = (left + right) / 2
                    if previous is None:
                        cost = abs(center - width / 2) - span * 0.1
                    else:
                        pc, ps = previous
                        if abs(center - pc) > width * 0.14 or abs(span - ps) > width * 0.22:
                            continue
                        cost = abs(center - pc) + abs(span - ps) * 0.4
                    if (self.last_full_time is not None and timestamp-self.last_full_time < .3
                            and y in self.full_rows):
                        old_left, old_right = self.full_rows[y]
                        # Printed marker edges are inside the lane and should
                        # not replace the established lane width.
                        cost += abs(span-(old_right-old_left))*.8
                        cost += abs(center-(old_left+old_right)/2)*.1
                    candidates.append((cost, left, right))
            if candidates:
                _, left, right = min(candidates)
                previous = ((left + right) / 2, right - left)
                self.rows.append((y, left, right))
            elif single_mode and y in self.full_rows and len(runs) == 1:
                old_left, old_right = self.full_rows[y]
                edge = runs[0]
                span = old_right-old_left
                if abs(edge-old_right) <= abs(edge-old_left):
                    right, left = edge, edge-span
                else:
                    left, right = edge, edge+span
                center = (left+right)/2
                if previous is None or abs(center-previous[0]) <= width*.18:
                    previous = (center, span)
                    self.rows.append((y, left, right))
                    self.degraded = True
        points = [((y / height - 0.49) / 0.33, (l + r) / width - 1)
                  for y, l, r in self.rows]
        fit = quadratic(points) if len(points) >= 7 else None
        if fit:
            a, b, c = fit
            points = [(t, x) for t, x in points if abs(x - (a*t*t+b*t+c)) < 0.045]
            fit = quadratic(points) if len(points) >= 7 else None
        valid = fit is not None and max(t for t, x in points) - min(t for t, x in points) > 1.3
        if valid:
            a, b, c = fit
            residual = sum(abs(x - (a*t*t+b*t+c)) for t, x in points) / len(points)
            valid = residual < 0.025
        if not valid:
            self.pending = self.pending_since = None
            # A near marker or a foot can hide the lower scan rows briefly.
            # Preserve an already established lane only while at least four
            # mutually consistent pairs remain and the last full solution is
            # very recent. Blank/true loss still stops immediately.
            if self.last_full_time is None and len(self.rows) >= 3:
                # At startup an extreme curve may expose only the nearest
                # paired rows. Keep a low-confidence straight suggestion until
                # enough vertical coverage exists for the normal fit.
                self.last_valid = timestamp
                self.degraded = True
                self.state = "STRAIGHT"
                return self._result(True, min(.12, len(self.rows)/11*.20))
            bridge = (1000000 if len(self.rows) >= 4 else
                      1.20 if len(self.rows) >= 3 else
                      .50 if len(self.rows) >= 2 else 0)
            if (bridge and self.last_full_time is not None and
                    timestamp-self.last_full_time <= bridge):
                self.last_valid = timestamp
                self.degraded = True
                return self._result(True, min(.25, len(self.rows)/11*.25))
            if self.last_valid is None or timestamp - self.last_valid >= 0.25:
                self.state = "LOST"
            return self._result(False, 0.0)
        if self.degraded:
            # Bounded hold of an ALREADY confirmed left turn. Do not learn
            # widths, refresh the full-frame timer, or infer a right correction.
            self.last_valid = timestamp
            self.pending = self.pending_since = None
            return self._result(True, min(0.35, len(points)/11*0.35))
        self.raw_image_angle = math.degrees(math.atan(-b*width/(2*.33*height)))
        self.full_rows = {y: (left, right) for y, left, right in self.rows}
        self.last_full_time = timestamp
        reset = self.last_valid is None or timestamp - self.last_valid >= 0.25
        self.last_valid = timestamp
        alpha = 1.0 if reset else 1 - math.exp(-dt / self.filter_seconds)
        # Fit's linear term describes near-to-far direction; curvature alone
        # cannot distinguish walking sway and a turn in this camera geometry.
        self.offset += alpha * (a * 0.64 + b * 0.8 + c - self.offset)
        self.heading += alpha * (-b - self.heading)
        self.curvature += alpha * (a - self.curvature)
        if reset:
            self.state = "STRAIGHT"
            self.pending = self.pending_since = None
        direction = "RIGHT" if self.heading > 0 else "LEFT"
        turn_evidence = abs(self.heading) > self.turn_enter and self.heading * self.curvature > 0
        desired = direction if turn_evidence else self.state
        if not turn_evidence and abs(self.offset) > self.offset_enter:
            desired = "RIGHT" if self.offset > 0 else "LEFT"
        if abs(self.heading) < self.turn_exit and abs(self.offset) < self.offset_exit:
            desired = "STRAIGHT"
        if self.counterclockwise:
            # Positive offset: lane is to the right, robot too far LEFT.
            # Rightward curvature alone never triggers a right turn on this track.
            if self.offset > self.right_recover_enter:
                desired = "RIGHT"
            elif self.state == "RIGHT" and self.offset > self.right_recover_exit:
                desired = "RIGHT"
            elif desired == "RIGHT":
                desired = "LEFT" if (turn_evidence and direction == "LEFT") or self.offset < -self.offset_enter else "STRAIGHT"
        if desired != self.state:
            if self.pending != desired:
                self.pending, self.pending_since = desired, timestamp
            elif timestamp - self.pending_since >= self.confirm_seconds:
                self.state = desired
                self.pending = self.pending_since = None
        else:
            self.pending = self.pending_since = None
        return self._result(True, len(points) / 11 * max(0, 1 - residual / 0.025))

    def _result(self, valid, confidence):
        # Invalid frames ALWAYS disable the motion suggestion, even during grace.
        steer = max(-1.0, min(1.0, self.offset * 0.55 + self.heading * 1.2)) if valid else 0.0
        return dict(valid=valid, state=self.state, confidence=confidence,
                    degraded=self.degraded,
                    offset=self.offset, heading=self.heading, curvature=self.curvature,
                    steer=steer, stop=not valid)
