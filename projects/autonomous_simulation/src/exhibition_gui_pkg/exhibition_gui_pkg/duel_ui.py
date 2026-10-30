"""Live comparison and an honest, spatial result instead of an opaque score."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPixmap, QTransform
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar, QPushButton, QGridLayout

HUMAN, AI, TEXT, MUTED = '#ffd166', '#25d8d0', '#f4f8ff', '#9bb0c7'
FONT = 'Noto Sans CJK KR'


def text(value, size=16, color=TEXT):
    widget = QLabel(value)
    widget.setWordWrap(True)
    widget.setFont(QFont(FONT, size))
    widget.setProperty('visitorFontSize', size)
    widget.setStyleSheet(f'color: {color}; background: transparent;')
    return widget


def duration(record):
    if not record or not record.get('valid', True): return '기록 제외'
    if record.get('ended') and not record.get('finished'): return '미완주'
    if not record.get('finished'): return '주행 중'
    seconds = record['finish_sec']
    if seconds < 60: return f'{seconds:.1f}초'
    return f'{int(seconds // 60)}분 {seconds % 60:04.1f}초'


class DuelMap(QWidget):
    def __init__(self, trails=False):
        super().__init__()
        self.trails = trails
        self.status = {}
        self.setMinimumSize(180, 140)
        path = Path(get_package_share_directory('simulation_pkg')) / 'models/race_track/materials/textures/track.png'
        self.track = QPixmap(str(path)).transformed(QTransform().rotate(180))

    def refresh(self, status):
        self.status = status
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        width = min(self.width() - 10, (self.height() - 10) * 4/3)
        area = QRectF((self.width()-width)/2, (self.height()-width*.75)/2, width, width*.75)
        p.setOpacity(.62)
        p.drawPixmap(area, self.track, QRectF(self.track.rect()))
        p.setOpacity(1)
        def point(pose):
            return QPointF(area.left()+(.5-pose[1]/16)*width, area.top()+(.5-pose[0]/12)*area.height())
        duel = self.status.get('duel', {})
        p.setClipRect(area)
        controls = self.status.get('demo_controls', {})
        for i, pose in enumerate(controls.get('positions', [(3.4,-5.1),(4.9,-.9),(3.6,2.5)])):
            if not controls.get('obstacles', [True]*3)[i]: continue
            at = point(pose)
            p.setPen(QPen(QColor('#ff887b'), 2)); p.setBrush(QColor('#251c26'))
            p.drawEllipse(at, 9, 9)
            p.setFont(QFont(FONT, 9, QFont.Bold))
            p.drawText(QRectF(at.x()-9, at.y()-9, 18, 18), Qt.AlignCenter, str(i+1))
        for name, color, pose in [('ai', AI, duel.get('ai_pose')), ('player', HUMAN, self.status.get('player_pose'))]:
            record = duel.get(name) or {}
            if self.trails:
                path = QPainterPath(); connected = False
                for sample in record.get('trace', []):
                    if sample is None: connected = False; continue
                    if connected: path.lineTo(point(sample))
                    else: path.moveTo(point(sample)); connected = True
                p.setPen(QPen(QColor(color), 2.5)); p.setBrush(Qt.NoBrush); p.drawPath(path)
                for event in record.get('events', []):
                    if not event.get('pose'): continue
                    at = point(event['pose'])
                    p.setPen(QPen(QColor(color), 2)); p.setBrush(QColor('#111a28'))
                    p.drawEllipse(at, 6, 6)
            if pose:
                at = point(pose)
                p.setPen(QPen(QColor('#06101e'), 2)); p.setBrush(QColor(color))
                p.drawEllipse(at, 7 if name == 'player' else 10, 7 if name == 'player' else 10)
        p.end()


class DuelPanel(QWidget):
    def __init__(self, send, results=False):
        super().__init__()
        self.results = results
        root = QVBoxLayout(self); root.setContentsMargins(0,0,0,0); root.setSpacing(12)
        self.root_layout = root
        if results:
            columns = QHBoxLayout()
            root.addLayout(columns, 1)
            summary = QWidget(); root = QVBoxLayout(summary)
            root.setContentsMargins(0,0,20,0)
            columns.addWidget(summary, 4)
        self.heading = text('같은 코스 · 같은 출발', 20)
        root.addWidget(self.heading)
        row = QHBoxLayout(); row.addWidget(text('● 나', 17, HUMAN)); row.addWidget(text('● AI', 17, AI)); root.addLayout(row)
        self.bars = {}
        for key, color in [('player', HUMAN), ('ai', AI)]:
            bar = QProgressBar(); bar.setRange(0,1000); bar.setMinimumHeight(29)
            fill = 'rgba(255,209,102,80)' if key == 'player' else 'rgba(37,216,208,80)'
            bar.setFont(QFont(FONT,12)); bar.setStyleSheet(f'QProgressBar {{background:#12293d; color:{TEXT}; border:0; border-radius:5px; text-align:center}} QProgressBar::chunk {{background:{fill}; border-radius:5px}}')
            root.addWidget(bar); self.bars[key] = bar
        self.metrics = QGridLayout(); self.metrics.setHorizontalSpacing(16); self.metrics.setVerticalSpacing(10)
        for col, (title,color) in enumerate([('비교할 기록',MUTED),('나',HUMAN),('AI',AI)]):
            self.metrics.addWidget(text(title,14,color),0,col)
        self.values = {}
        for row, (key, name) in enumerate([('emergency_stops','안전장치 제동'),('recoveries','도로 복귀'),('finish_sec','완주 시간')],1):
            self.metrics.addWidget(text(name,14),row,0)
            for col, racer in enumerate(['player','ai'],1):
                cell = text('—',15,HUMAN if racer=='player' else AI)
                self.values[racer,key] = cell; self.metrics.addWidget(cell,row,col)
        root.addLayout(self.metrics)
        self.map = DuelMap(trails=results)
        if results:
            columns.addWidget(self.map, 6)
            root.addStretch(1)
        else:
            root.addWidget(self.map,1)
        self.note = text('',13,MUTED); root.addWidget(self.note)
        self.recover = QPushButton('막혔어요 · 도로로 돌아가기')
        self.recover.setMinimumHeight(48); self.recover.clicked.connect(lambda: send('duel_recover'))
        self.recover.setStyleSheet('QPushButton {color:#ffd166; background:#29313d; border:1px solid #ffd166; border-radius:8px; padding:8px;}')
        root.addWidget(self.recover); self.recover.hide()
        if not results:
            self.pedal = QProgressBar()
            self.pedal.setRange(0,100); self.pedal.setMinimumHeight(30)
            self.pedal.setStyleSheet('QProgressBar {background:#12293d; color:#ffd166; border:0; border-radius:5px; text-align:center} QProgressBar::chunk {background:#655d36; border-radius:5px}')
            root.addWidget(self.pedal)
            self.end_run = QPushButton('여기까지 결과 보기')
            self.end_run.setMinimumHeight(40)
            self.end_run.clicked.connect(lambda: send('duel_finish'))
            self.end_run.setStyleSheet('QPushButton {color:#9bb0c7; background:transparent; border:0; padding:8px;}')
            root.addWidget(self.end_run)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        compact = self.window().height() < 850
        self.root_layout.setSpacing(7 if compact else 12)
        self.metrics.setVerticalSpacing(5 if compact else 10)
        self.map.setMinimumHeight(80 if compact else 140)
        self.map.setVisible(self.results or not compact)
        for widget in self.findChildren(QLabel):
            base = widget.property('visitorFontSize')
            if base:
                font = widget.font(); font.setPointSize(max(10, round(base * (.8 if compact else 1))))
                widget.setFont(font)
                widget.setMinimumHeight(widget.fontMetrics().height() + 3)

    def refresh(self, status):
        duel = status.get('duel', {})
        records = [duel.get(key) or {} for key in ('player','ai')]
        for racer, record in zip(('player','ai'), records):
            percent = 100 if record.get('finished') else record.get('progress',0)
            self.bars[racer].setValue(round(percent*10))
            self.bars[racer].setFormat(('나' if racer=='player' else 'AI') + (' · 완주' if record.get('finished') else f' · {percent:.0f}%'))
            for key in ('emergency_stops','recoveries','finish_sec'):
                self.values[racer,key].setText(duration(record) if key=='finish_sec' else f'{record.get(key,0)}회')
        self.map.refresh(status)
        self.recover.setVisible(not self.results and duel.get('recovery_available',False))
        controller = status.get('input_mode') == 'controller'
        self.recover.setText(('X  ' if controller else '') + '막혔어요 · 도로로 돌아가기')
        if not self.results:
            self.end_run.setText(('Y  ' if controller else '') + '여기까지 결과 보기')
            self.pedal.setValue(round(status.get('manual_throttle',0)*100))
            self.pedal.setFormat('LT 브레이크 우선' if controller and status.get('manual_brake',0) >= .03 else
                                'RT 가속 입력  %p%' if controller else '↑ 가속 입력  %p%')
        if self.results:
            self.heading.setText('달린 길과 기록을 비교해 보세요')
            self.note.setText(duel.get('error') or ('AI가 아직 달리고 있어요. 기록이 들어오면 여기에 표시돼요.' if not records[1].get('finished') else
                '선은 실제 이동 경로, 동그라미는 안전장치 제동·도로 복귀 지점이에요.'))
        else:
            controls = '왼쪽 스틱 핸들\nRT 누를수록 가속 · LT 브레이크' if controller else '← → 핸들 · ↑ 가속 · ↓ 브레이크'
            self.note.setText(duel.get('error') or f'{controls}\n\n청록색 AI와는 부딪히지 않아요.')
