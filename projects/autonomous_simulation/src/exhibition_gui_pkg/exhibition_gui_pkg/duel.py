"""Comparable race records and visitor steering, independent of ROS/Qt."""
from dataclasses import dataclass, field
from exhibition_gui_pkg.experience import LapProgress

RACE_PWM = int(255 * 1.2345 / 1.82)
DUEL_DIFFICULTIES = {
    'beginner': {
        'label': '입문', 'description': 'AI가 천천히 달리고 장애물이 1대예요.',
        'ai_speed_ratio': .70, 'obstacles': (True, False, False),
    },
    'intermediate': {
        'label': '도전', 'description': 'AI 속도와 장애물 수가 균형 잡힌 코스예요.',
        'ai_speed_ratio': .85, 'obstacles': (True, True, False),
    },
    'expert': {
        'label': '고급', 'description': 'AI가 빠르게 달리고 장애물이 3대예요.',
        'ai_speed_ratio': 1.00, 'obstacles': (True, True, True),
    },
}
# Keep the historical course as the default for standalone race mode and tests.
RACE_OBSTACLES = DUEL_DIFFICULTIES['intermediate']['obstacles']


def throttle_pwm(throttle, brake=False):
    """Use the same speed cap as AI; LT always wins over RT."""
    return 0 if brake else round(RACE_PWM * max(0.0, min(1.0, throttle)))


@dataclass
class RunRecord:
    total: float
    start: float
    progress: LapProgress = field(init=False)
    emergency_stops: int = 0
    recoveries: int = 0
    finished_sec: float = None
    ended_sec: float = None
    valid: bool = True
    error: str = ''
    trace: list = field(default_factory=list)
    events: list = field(default_factory=list)
    sample_at: float = -1.0
    risk_since: float = None
    clear_since: float = None
    risk_counted: bool = False

    def __post_init__(self):
        self.progress = LapProgress(self.total)
        self.progress.rebase(self.start)

    def observe(self, route, pose, elapsed, on_road=True):
        if self.ended_sec is not None or not self.valid:
            return
        if on_road:
            self.progress.update(route)
        if elapsed - self.sample_at >= .1:
            self.trace.append([round(pose[0], 3), round(pose[1], 3)])
            self.sample_at = elapsed
        if self.progress.best >= self.total - .45 and elapsed >= 8:
            self.finish(elapsed)

    def emergency(self, active, elapsed, pose):
        if self.ended_sec is not None or not self.valid:
            return
        if active:
            self.clear_since = None
            if self.risk_since is None:
                self.risk_since = elapsed
            if not self.risk_counted and elapsed - self.risk_since >= .12:
                self.emergency_stops += 1
                self.risk_counted = True
                self.event('brake', elapsed, pose)
        else:
            if self.clear_since is None:
                self.clear_since = elapsed
            if elapsed - self.clear_since >= .5:
                self.risk_since = None
                self.risk_counted = False

    def recover(self, route, pose, elapsed):
        if self.ended_sec is not None or not self.valid:
            return
        self.recoveries += 1
        self.event('recovery', elapsed, pose)
        self.trace.append(None)
        self.progress.rebase(route)

    def event(self, kind, elapsed, pose):
        self.events.append(dict(kind=kind, sec=round(elapsed, 2),
                                pose=list(pose[:2]) if pose else None))

    def finish(self, elapsed):
        if self.finished_sec is None:
            self.finished_sec = round(elapsed, 2)
            self.end(elapsed)

    def end(self, elapsed):
        if self.ended_sec is None:
            self.ended_sec = round(elapsed, 2)

    def snapshot(self, elapsed):
        return dict(progress=round(min(100, self.progress.best / self.total * 100), 1),
                    emergency_stops=self.emergency_stops, recoveries=self.recoveries,
                    finished=self.finished_sec is not None, ended=self.ended_sec is not None,
                    elapsed=self.ended_sec if self.ended_sec is not None else round(elapsed, 2),
                    finish_sec=self.finished_sec, valid=self.valid, error=self.error,
                    trace=list(self.trace), events=list(self.events))


class VisitorSteering:
    def __init__(self):
        self.value = 0.0

    def update(self, target, elapsed):
        # Keyboard taps build steering progressively; releasing recentres it.
        # The limit applies equally to either direction, never to the AI result.
        rate = 36.0 if abs(target) < .01 else 24.0
        step = rate * max(0.0, min(elapsed, .05))
        self.value += max(-step, min(step, target - self.value))
        return round(self.value)
