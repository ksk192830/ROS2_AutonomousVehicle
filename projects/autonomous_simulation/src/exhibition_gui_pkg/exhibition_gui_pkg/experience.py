"""Visitor-paced learning flow and lap accounting, independent of ROS and Qt."""

from dataclasses import dataclass, field
import uuid


STAGES = {
    'IDLE': '체험 준비', 'EXPLORE': '대회장 둘러보기', 'INTRO': '자율주행 원리 살펴보기',
    'DEMO': 'AI 운전 관찰', 'PRACTICE': '조작 익히기', 'DIFFICULTY': '난이도 고르기',
    'CHALLENGE': '한 바퀴 도전', 'RESULTS': '나의 체험 결과',
}
LESSONS = (
    ('카메라', '카메라로 차 앞의 도로를 봐요',
     '차 앞의 카메라 영상에서 길을 찾기 시작해요. 전체 트랙 지도는 체험자가 차의 위치를 확인하는 화면이에요.',
     '다음: 달릴 공간 찾기'),
    ('달릴 공간', '차가 달릴 공간을 색으로 표시해요',
     '파랑과 분홍은 AI가 찾은 서로 다른 주행 공간이에요. 이렇게 사진의 부분을 종류별로 나누는 기술을 Segmentation이라고 해요.',
     '다음: 위에서 본 도로'),
    ('위에서 본 도로', '도로를 위에서 내려다본 모습으로 바꿔요',
     '먼 길이 좁아 보이는 사진을 위에서 본 모습으로 펼쳐요. 차선 사이의 위치를 비교하기 쉬워져요. 이 모습을 BEV라고 해요.',
     '다음: 가운데 선 만들기'),
    ('가운데 선', '차가 따라갈 가운데 선을 만들어요',
     '흰 점은 달릴 공간의 양쪽 끝이에요. 그 사이의 중간점을 이어 초록 선을 만들어요. 점선은 부족한 정보를 보완한 구간이에요.',
     '다음: 핸들 방향 정하기'),
    ('핸들 방향', '가운데 선에 맞춰 핸들 방향을 정해요',
     '차가 천천히 움직이면서 초록 선을 따라 핸들을 돌려요. 다음 단계에서는 장애물을 놓거나 차선을 바꿔 볼 수 있어요.',
     'AI 주행 체험으로'),
)
PRACTICE_TASKS = (('왼쪽 조향', '오른쪽 조향'), ('가속',), ('제동',))

# Both monitors share titles; the main screen adds purpose and observation.
LESSON_GUIDES = (
    (LESSONS[0][1],
     '방금 살펴본 차량의 앞쪽 카메라 한 대가 보는 화면이에요.\n이 영상에서 달릴 곳을 찾고, 길의 모양을 바꿔 보고, 핸들 방향을 정해요.',
     '흰 차선과 커브를 찾아보세요',
     '커브에 가까워질수록 화면 속 흰 차선이 어떻게 휘는지 보세요.'),
    (LESSONS[1][1],
     'AI가 사진의 작은 점마다 종류를 판단해요. 같은 종류를 같은 색으로 표시하면 달릴 공간을 구분할 수 있어요.',
     '같은 색으로 묶인 부분을 보세요',
     '파랑과 분홍은 서로 다른 주행 공간이에요. 흰 차선 사이를 채운 색이 어떻게 나뉘는지 살펴보세요.'),
    (LESSONS[2][1],
     '사진에서는 먼 도로가 좁아 보여요.\n위에서 본 모습으로 펼치면, 차가 지나갈 공간의 양쪽 끝과 가운데를 비교하기 쉬워져요.',
     '좁아 보이던 먼 길이 어떻게 바뀌나요?',
     '노란 네 점을 기준으로 도로를 펼쳐요. 앞쪽 카메라로 찍은 사진을 계산해서 바꾼 모습이에요.'),
    (LESSONS[3][1],
     '차선 한가운데를 따라가려면 기준이 될 선이 필요해요.\n달릴 공간의 양쪽 끝에서 같은 거리인 점들을 찾아 초록 선으로 이어요.',
     '흰 점 사이에서 초록 선을 찾아보세요',
     '흰 점은 달릴 공간의 양쪽 끝, 초록 선은 차가 따라갈 가운데예요. 점선은 부족한 정보를 보완한 구간이에요.'),
    (LESSONS[4][1],
     '가운데 선을 따라가려면 핸들을 맞춰 돌려야 해요.\nAI는 차가 가운데에서 벗어난 정도와 앞쪽 길이 휘는 방향을 함께 보고 계산해요.',
     '초록 선과 핸들 방향을 비교해 보세요',
     '초록 선이 휘는 쪽으로 핸들도 돌아가는지 보세요.'),
)

LESSON_TERMS = ('', '영역 나누기 · Segmentation', '위에서 본 모습 · BEV', '', '')


@dataclass
class ExperienceSession:
    phase: str = 'IDLE'
    started_at: float = 0.0
    stage_started_at: float = 0.0
    session_id: str = ''
    results: list = field(default_factory=list)
    error: str = ''
    lesson: int = 0
    practice_step: int = 0
    attempt: int = 0
    answer: str = ''
    difficulty: str = 'beginner'

    @property
    def label(self):
        return STAGES[self.phase]

    def go(self, phase, now):
        if phase not in STAGES:
            raise ValueError(f'Unknown experience phase: {phase}')
        self.phase = phase
        self.stage_started_at = now

    def start(self, now):
        self.started_at = now
        self.session_id = uuid.uuid4().hex[:8]
        self.results = []
        self.error = self.answer = ''
        self.lesson = self.practice_step = self.attempt = 0
        self.difficulty = 'beginner'
        self.go('EXPLORE', now)

    def clear(self, now):
        self.go('IDLE', now)
        self.started_at = 0.0
        self.session_id = ''
        self.results = []
        self.error = self.answer = ''
        self.lesson = self.practice_step = self.attempt = 0
        self.difficulty = 'beginner'

    def begin_attempt(self, now):
        self.attempt += 1
        self.error = ''
        self.go('CHALLENGE', now)

    def record(self, **result):
        if self.phase == 'CHALLENGE' and self.attempt and not any(
                item['attempt'] == self.attempt for item in self.results):
            self.results.append({'attempt': self.attempt, **result})

    def snapshot(self, now):
        return {
            'id': self.session_id, 'phase': self.phase, 'label': self.label,
            'attempt': self.attempt, 'remaining_sec': None, 'visitor_paced': True,
            'elapsed_sec': round(max(0.0, now - self.started_at), 1) if self.session_id else 0.0,
            'lesson': self.lesson, 'practice_step': self.practice_step,
            'results': list(self.results), 'error': self.error, 'answer': self.answer,
            'difficulty': self.difficulty,
        }


def practice_complete(step, tasks):
    return set(PRACTICE_TASKS[step]).issubset(tasks)


def result_text(results):
    if not results:
        return '도로를 보고, 차선을 찾고, 방향을 정하는 과정을 살펴봤어요.'
    latest = results[-1]
    if not latest.get('valid', True):
        return '연결 문제로 이번 기록은 비교에서 제외했어요. 준비되면 다시 도전할 수 있어요.'
    if not latest.get('started', latest.get('progress_percent', 0) > 0):
        return '아직 출발하지 못했어요. 조작 연습에서 가속과 제동을 다시 익혀 보세요.'
    if latest.get('completed'):
        text = f"한 바퀴 완주!  {latest['elapsed_sec']:.1f}초\n도로로 돌아온 횟수  {latest['offtracks']}회"
    else:
        text = f"도로의 {latest['progress_percent']:.0f}%까지 달렸어요.\n도로로 돌아온 횟수  {latest['offtracks']}회"
    completed = [item for item in results if item.get('valid', True) and item.get('completed')]
    if latest.get('completed') and len(completed) >= 2:
        change = completed[-2]['elapsed_sec'] - completed[-1]['elapsed_sec']
        text += '\n' + ('이전과 같은 기록이에요.' if abs(change) < 0.05 else
                       f'이전 완주보다 {abs(change):.1f}초 ' + ('빨라졌어요!' if change > 0 else '더 걸렸어요.'))
    return text


@dataclass
class LapProgress:
    total: float
    last: float = 0.0
    signed_distance: float = 0.0
    best: float = 0.0

    def update(self, route_s, max_step=3.0):
        delta = (route_s - self.last + self.total / 2) % self.total - self.total / 2
        self.last = route_s
        if abs(delta) <= max_step:
            # Reverse travel must be repaid before advancing the best result.
            self.signed_distance += delta
            self.best = max(self.best, self.signed_distance)
        return self.best

    def rebase(self, route_s):
        """A recovery teleport never counts as driving progress."""
        self.last = route_s
