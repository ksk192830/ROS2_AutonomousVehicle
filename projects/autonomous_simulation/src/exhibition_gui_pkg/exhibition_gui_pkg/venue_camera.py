"""Keep one inspection camera, streaming only while the visitor explores."""
import math
import time

from gazebo_msgs.srv import SpawnEntity, SetEntityState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_prefix
from pathlib import Path
from sensor_msgs.msg import Image

from exhibition_gui_pkg.display_camera import LatestFrameConverter
from exhibition_gui_pkg.venue_view import ENTITY, TOPIC, VenueView, VenueBoundary, camera_sdf


class VenueCamera:
    def __init__(self, owner):
        self.owner = owner
        self.boundary = VenueBoundary(owner.track.normalized_contours[0]) if owner.track.available else None
        self.view = VenueView(boundary=self.boundary)
        self.active = self.exists = False
        self.operation = self.pose_request = None
        self.operation_at = self.last_tick = time.monotonic()
        self.started_at = 0.
        self.subscription = None
        self.error = ''
        self.actual_pose = None
        self.sent_pose = None
        self.image = None
        self.sequence = 0
        self.image_at = 0.
        self.frame_pose = self.frame_car = None
        self.converter = LatestFrameConverter(
            lambda packet: (owner._convert_display_image(packet[0]), packet[1], packet[2]),
            owner.display_frame_notifier.ready.emit)
        owner.create_timer(1/120, self.tick)

    @property
    def ready(self):
        return self.active and self.exists and time.monotonic()-self.image_at < 1.5

    def activate(self, active):
        if self.active == active:
            return
        self.active = active
        self.error = ''
        if active:
            self.view = VenueView(boundary=self.boundary)
            self.started_at = time.monotonic()
            self.image = None
            self.image_at = 0.
            self.sequence = (self.converter.latest or (0,))[0]
            self.sent_pose = None
            self.actual_pose = None
            self.subscription = self.owner.create_subscription(Image, TOPIC, self.on_image,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        elif self.subscription:
            self.owner.destroy_subscription(self.subscription)
            self.subscription = None

    def on_image(self, msg):
        self.converter.submit((msg, self.actual_pose or self.view.pose, self.owner.player_pose))

    def focus(self, vehicle=False):
        if self.active:
            self.error = ''
            self.started_at = time.monotonic()
            self.view.focus(self.owner.player_pose if vehicle else None)

    def state(self):
        result = SetEntityState.Request()
        result.state.name = ENTITY
        result.state.reference_frame = 'world'
        result.state.pose.position.x, result.state.pose.position.y, result.state.pose.position.z = self.view.pose[:3]
        sp, cp = math.sin(self.view.pose[3]/2), math.cos(self.view.pose[3]/2)
        sy, cy = math.sin(self.view.yaw/2), math.cos(self.view.yaw/2)
        result.state.pose.orientation.x = -sp*sy
        result.state.pose.orientation.y = sp*cy
        result.state.pose.orientation.z = cp*sy
        result.state.pose.orientation.w = cp*cy
        return result

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        self.collect_display_frame()
        if self.operation:
            if not self.operation.done():
                if now-self.operation_at > 8:
                    self.error = '둘러보기 화면 연결이 지연되고 있어요. B로 돌아가 다시 시작해 주세요.'
                return
            future = self.operation
            self.operation = None
            try:
                result = future.result()
                if not result.success:
                    raise RuntimeError(result.status_message)
                self.exists = True
                self.sent_pose = None
                self.error = ''
                self.started_at = now
            except Exception:
                self.error = '둘러보기 화면을 준비하지 못했어요. X를 눌러 다시 시도해 주세요.'
                return
        if not self.active:
            return
        if not self.exists:
            if self.error:
                return
            if self.owner.spawn_client.service_is_ready():
                plugin = Path(get_package_prefix('exhibition_sim_plugins'))/'lib/libexhibition_camera_rate.so'
                request = SpawnEntity.Request(name=ENTITY, xml=camera_sdf(str(plugin)), reference_frame='world')
                request.initial_pose = self.state().state.pose
                self.operation = self.owner.spawn_client.call_async(request)
                self.operation_at = now
            return
        if now-self.started_at > 8 and not self.ready:
            # A temporary image gap must not stop navigation or frame recovery.
            self.error = '둘러보기 영상을 다시 연결하고 있어요.'
        right = forward = turn = tilt = 0.
        if self.owner.state == 'EXPLORE':
            if self.owner.input_mode == 'controller' and self.owner.controller_connected:
                axes = self.owner.joy_axes
                right = axes[0] if axes else 0.
                forward = -axes[1] if len(axes) > 1 else 0.
                turn = axes[3] if len(axes) > 3 else 0.
                tilt = axes[4] if len(axes) > 4 else 0.
                if len(axes)>7:
                    right = axes[6] if abs(axes[6])>.5 else right
                    forward = -axes[7] if abs(axes[7])>.5 else forward
            elif self.owner.input_mode == 'keyboard':
                keys = self.owner.keyboard
                right = float(keys['right'])-float(keys['left'])
                forward = float(keys['throttle'])-float(keys['brake'])
                turn = float(keys.get('view_right', False))-float(keys.get('view_left', False))
                tilt = float(keys.get('view_down', False))-float(keys.get('view_up', False))
        self.view.move(right, forward, dt, turn, tilt)
        if self.pose_request:
            if not self.pose_request.done():
                return
            try:
                if not self.pose_request.result().success:
                    self.error = '시점 이동 연결을 확인해 주세요. X로 다시 시도할 수 있어요.'
            except Exception:
                self.error = '시점 이동 연결을 확인해 주세요. X로 다시 시도할 수 있어요.'
            self.pose_request = None
        pose = self.view.pose
        if self.sent_pose is None or math.dist(pose, self.sent_pose) > .003:
            self.pose_request = self.owner.state_client.call_async(self.state())
            self.sent_pose = pose

    def collect_display_frame(self):
        latest = self.converter.latest
        if self.active and latest and latest[0] != self.sequence:
            self.sequence, frame, self.image_at = latest
            image, self.frame_pose, self.frame_car = frame
            self.image = (self.sequence, image)
            if time.monotonic()-self.image_at < 1.5:
                self.error = ''

    def snapshot(self):
        return dict(ready=self.ready, error=self.error, pose=list(self.actual_pose or self.view.pose),
                    active=self.active, image_fresh=self.ready)

    def close(self):
        self.converter.close()
