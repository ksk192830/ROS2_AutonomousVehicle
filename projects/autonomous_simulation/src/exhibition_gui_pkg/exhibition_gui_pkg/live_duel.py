"""Run a real camera-driven opponent alongside the participant."""
import json
import math
import time
from pathlib import Path

from gazebo_msgs.msg import ModelStates, ContactsState
from gazebo_msgs.srv import SetEntityState
from interfaces_pkg.msg import MotionCommand, LaneInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from exhibition_gui_pkg.duel import DUEL_DIFFICULTIES, RACE_OBSTACLES, RACE_PWM, RunRecord, VisitorSteering, throttle_pwm
from exhibition_gui_pkg.scene_safety import ObstacleSafety, ContactConfirmation, read_scene

AI_ENTITY = 'ai_rival'
PARK = (32.0, 30.0, 0.0)
START = (-1.15, -6.80, 0.0)


class LiveDuel:
    def __init__(self, owner):
        self.owner = owner
        self.pose = None
        self.pose_at = 0.0
        self.speed = 0.0
        self.autonomy = MotionCommand()
        self.autonomy_at = 0.0
        self.lane_valid = False
        self.geometry = {}
        self.geometry_at = 0.0
        self.guard = ObstacleSafety()
        self.contact = ContactConfirmation(AI_ENTITY)
        self.preparing = False
        self.countdown_armed = False
        self.prepared_at = 0.0
        self.reset_at = 0.0
        self.started = False
        self.reset_pending = False
        self.reset_generation = 0
        self.hold_until = 0.0
        self.last_recovery = 0.0
        self.outside_since = None
        self.command_at = 0.0
        self.started_sim = 0.0
        self.paused_sim = None
        self.player = self.ai = None
        self.steering = VisitorSteering()
        self.control_at = time.monotonic()
        self.error = ''
        self.saved = False
        self.difficulty = 'beginner'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.motion = owner.create_publisher(MotionCommand, '/ai/motion_command', 10)
        self.command = owner.create_publisher(String, '/ai/exhibition/command', 10)
        self.safety = owner.create_publisher(String, '/ai/exhibition/safety', qos)
        owner.create_subscription(MotionCommand, '/ai/autonomy/motion_command', self.on_autonomy, 1)
        owner.create_subscription(String, '/ai/cam0/lane_geometry', self.on_geometry, 1)
        owner.create_subscription(LaneInfo, '/ai/cam0/lane_info', lambda m: setattr(self, 'lane_valid', m.lane_num in (1, 2)), 1)
        owner.create_subscription(ModelStates, '/model_states', self.on_scene, sensor)
        owner.create_subscription(ContactsState, '/ai/exhibition/contacts', self.on_contact,
                                  QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        owner.create_timer(.02, self.tick)

    @property
    def elapsed(self):
        clock = self.paused_sim if self.paused_sim is not None else self.owner.sim_time
        return max(0.0, clock - self.started_sim) if self.started else 0.0

    @property
    def available(self):
        return self.pose is not None and time.monotonic() - self.pose_at < .5 and self.motion.get_subscription_count() > 0

    @property
    def profile(self):
        # The intermediate fallback preserves the original course in isolated checks.
        return DUEL_DIFFICULTIES.get(getattr(self, 'difficulty', 'intermediate'), DUEL_DIFFICULTIES['intermediate'])

    @property
    def obstacles(self):
        return self.profile['obstacles']

    def configure_difficulty(self, difficulty):
        if difficulty not in DUEL_DIFFICULTIES or self.started or self.preparing:
            return False
        self.difficulty = difficulty
        return True

    @property
    def start_ready(self):
        state = self.owner.obstacle_state
        return (self.available and not self.reset_pending and self.lane_valid and
                time.monotonic() - self.autonomy_at < .5 and
                tuple(state.get('obstacles', [])) == self.obstacles and
                not state.get('clearing', True) and not any(state.get('pending', [True]*3)))

    def prepare(self):
        self.countdown_armed = False
        self.prepared_at = time.monotonic()
        self.preparing, self.started, self.saved = True, False, False
        self.error = ''
        self.player = RunRecord(self.owner.path_total, self.owner._route_at(*START[:2]))
        self.ai = RunRecord(self.owner.path_total, self.owner._route_at(*START[:2]))
        self.steering = VisitorSteering()
        self.outside_since = None
        self.command.publish(String(data='demo_reset'))
        self.reset_ai(START)

    def park(self):
        self.preparing = self.started = False
        self.motion.publish(MotionCommand())
        if self.available:
            self.reset_ai(PARK)

    def start(self):
        self.preparing = False
        self.started = True
        self.started_sim = self.owner.sim_time
        self.paused_sim = None
        self.command.publish(String(data='demo_reset'))

    def reset_ai(self, pose):
        self.reset_generation += 1
        generation = self.reset_generation
        self.reset_pending = True
        self.reset_at = time.monotonic()
        self.motion.publish(MotionCommand())
        self.contact.reset(round(self.owner.sim_time * 1e9))
        request = SetEntityState.Request()
        request.state = self.owner._entity_state(AI_ENTITY, *pose)
        future = self.owner.state_client.call_async(request)
        def done(future):
            if generation != self.reset_generation: return
            self.reset_pending = False
            try:
                if not future.result().success: raise RuntimeError('AI 차량 위치 초기화 실패')
            except Exception as exc:
                self.fail(str(exc))
            self.hold_until = time.monotonic() + .7
            self.autonomy_at = 0.0
        future.add_done_callback(done)

    def on_autonomy(self, message):
        self.autonomy, self.autonomy_at = message, time.monotonic()

    def on_geometry(self, message):
        try:
            self.geometry = json.loads(message.data)
            self.geometry_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_scene(self, message):
        pose, obstacles, velocity = read_scene(message, AI_ENTITY)
        if pose is None: return
        self.pose, self.pose_at = pose, time.monotonic()
        self.speed = math.hypot(*velocity[:2])
        self.guard.update(pose, obstacles, velocity, self.pose_at)
        if not self.started or not self.ai or self.ai.finished_sec is not None or self.reset_pending:
            return
        self.ai.observe(self.owner._route_at(*pose[:2]), pose, self.elapsed,
                        on_road=self.owner.track.contains_world_point(*pose[:2]))
        self.owner.ai_progress_s = self.ai.progress.best
        if self.ai.finished_sec is not None:
            self.motion.publish(MotionCommand())
            self.save()
            return
        if self.owner.track.contains_world_point(*pose[:2]):
            self.outside_since = None
        elif self.outside_since is None:
            self.outside_since = self.pose_at
        elif self.pose_at - self.outside_since > .5:
            self.recover()

    def on_contact(self, message):
        if (self.started and self.ai and self.ai.finished_sec is None and
                not self.reset_pending and time.monotonic() >= self.hold_until and
                self.contact.observe(message, time.monotonic())):
            self.recover()

    def recover(self):
        now = time.monotonic()
        if self.reset_pending or now - self.last_recovery < 1.5: return
        self.last_recovery = now
        route = self.owner._route_at(*self.pose[:2])
        chosen = None
        for increment in range(int(self.owner.path_total * 2)):
            candidate = self.owner._pose_at_distance((route + increment * .5) % self.owner.path_total)
            if self.owner.track.contains_world_point(*candidate[:2]) and all(
                math.hypot(candidate[0]-p[0], candidate[1]-p[1]) > 3 for p in self.guard.obstacles.values()):
                chosen = candidate; break
        if chosen is None:
            self.fail('AI 차량이 돌아갈 공간을 찾지 못했어요'); return
        self.ai.recover(self.owner._route_at(*chosen[:2]), self.pose, self.elapsed)
        self.command.publish(String(data='demo_reset'))
        self.reset_ai(chosen)
        self.outside_since = None

    def tick(self):
        now = time.monotonic()
        if self.started:
            if self.owner.state == 'PAUSED' and self.paused_sim is None:
                self.paused_sim = self.owner.sim_time
            elif self.owner.state != 'PAUSED' and self.paused_sim is not None:
                self.started_sim += self.owner.sim_time - self.paused_sim
                self.paused_sim = None
        if self.reset_pending and now - self.reset_at > 5:
            self.reset_generation += 1
            self.reset_pending = False
            self.fail('AI 차량 초기화가 지연되었어요')
        safety = self.guard.status(now)
        self.safety.publish(String(data=json.dumps(safety)))
        if self.preparing and now - self.command_at > .5:
            self.command_at = now
            state = self.owner.obstacle_state
            if not state.get('clearing', True) and self.owner.player_pose and math.hypot(
                    self.owner.player_pose[0]-START[0], self.owner.player_pose[1]-START[1]) < .5:
                for i, enabled in enumerate(state.get('obstacles', [])):
                    if enabled != self.obstacles[i] and not state.get('pending', [True]*3)[i]:
                        self.owner.exhibition_command_pub.publish(String(data=f'obstacle_toggle_{i+1}'))
        if not self.started or not self.ai or self.ai.finished_sec is not None or not self.ai.valid:
            self.motion.publish(MotionCommand()); return
        if self.owner.state in ('PAUSED', 'ERROR') or self.reset_pending or now < self.hold_until:
            self.motion.publish(MotionCommand()); return
        if now - self.pose_at > 1.0 or now - self.autonomy_at > 1.0:
            self.fail('AI 연결이 끊겨 비교 기록을 제외했어요'); return
        risk = safety['stop'] and safety['reason'] != 'position'
        self.ai.emergency(risk, self.elapsed, self.pose)
        if safety['stop'] or now < self.contact.pending_until or now - self.autonomy_at > .5:
            self.motion.publish(MotionCommand()); return
        command = MotionCommand()
        command.steering = self.autonomy.steering
        speed_cap = round(RACE_PWM * self.profile['ai_speed_ratio'])
        command.left_speed = min(speed_cap, int(self.autonomy.left_speed * 1.2345 / 1.82))
        command.right_speed = min(speed_cap, int(self.autonomy.right_speed * 1.2345 / 1.82))
        self.motion.publish(command)

    def observe_player(self):
        if self.started and self.player and self.owner.state == 'RUNNING' and self.owner.player_pose:
            self.player.observe(self.owner._route_at(*self.owner.player_pose[:2]), self.owner.player_pose, self.elapsed,
                                on_road=self.owner.track.contains_world_point(*self.owner.player_pose[:2]))

    def player_risk(self, active):
        if self.started and self.player and self.owner.state == 'RUNNING':
            braking = (self.owner._trigger(5) >= .03 if self.owner.input_mode == 'controller'
                       else self.owner.keyboard['brake'])
            self.player.emergency(active and not braking, self.elapsed, self.owner.player_pose)

    def player_recovery(self, target):
        if self.started and self.player:
            self.player.recover(self.owner._route_at(*target[:2]), self.owner.player_pose, self.elapsed)
            self.steering.value = 0.0

    def player_command(self, direction, throttle, brake):
        now = time.monotonic()
        value = self.steering.update(direction * 10, now - self.control_at)
        self.control_at = now
        command = MotionCommand()
        command.steering = value
        command.left_speed = command.right_speed = throttle_pwm(throttle, brake)
        return command

    def player_finished(self, complete=True):
        if self.player:
            if complete:
                self.player.finish(self.elapsed)
            else:
                self.player.end(self.elapsed)
            self.save()

    def fail(self, reason):
        self.error = reason
        self.motion.publish(MotionCommand())
        if self.ai:
            self.ai.valid, self.ai.error = False, reason
        self.save()

    def snapshot(self):
        payload = dict(enabled=True, available=self.available, preparing=self.preparing,
                       autonomy_steering=int(self.autonomy.steering),
                       autonomy_fresh=time.monotonic()-self.autonomy_at < .5,
                       lane_valid=self.lane_valid,
                       lane_geometry=self.geometry if time.monotonic()-self.geometry_at < 1 else {},
                       start_ready=self.start_ready,
                       recovery_available=(self.started and self.owner.state == 'RUNNING' and
                           self.owner.player_speed_mps < .08 and self.owner.hazard_since is not None),
                       started=self.started, error=self.error, ai_pose=self.pose,
                       ai_speed=round(self.speed, 3), auto_speed=False,
                       difficulty=self.difficulty, difficulty_label=self.profile['label'],
                       obstacle_count=sum(self.obstacles), ai_speed_ratio=self.profile['ai_speed_ratio'],
                       player=None, ai=None)
        if self.player and self.ai:
            payload.update(player=self.player.snapshot(self.elapsed), ai=self.ai.snapshot(self.elapsed))
        return payload

    def save(self):
        if (self.saved or not self.player or self.player.ended_sec is None or
                not self.ai or (self.ai.finished_sec is None and self.ai.valid)):
            return
        directory = Path.home() / '.local/state/e2e-exhibition/duels'
        try:
            directory.mkdir(parents=True, exist_ok=True)
            filename = f'{self.owner.session.session_id}-{self.owner.session.attempt}.json'
            (directory / filename).write_text(json.dumps(self.snapshot(), ensure_ascii=False, indent=2))
            self.saved = True
        except OSError as exc:
            self.owner.get_logger().warning(f'Comparison record could not be saved: {exc}')
