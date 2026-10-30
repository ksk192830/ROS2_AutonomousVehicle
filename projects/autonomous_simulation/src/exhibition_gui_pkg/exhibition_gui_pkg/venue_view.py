"""Ground-parallel visitor camera movement, independent of ROS and Qt."""
from dataclasses import dataclass
import math

ENTITY = 'exhibition_venue_view'
TOPIC = '/exhibition/venue/image_raw'
HEIGHT = 4.2
PITCH = .68
PITCH_LIMITS = (-.55, 1.35)
FOV = 1.25
HOME = (-4.0, 0.0)
X_LIMITS = (-5.8, 5.8)
Y_LIMITS = (-7.8, 7.8)


class VenueBoundary:
    """Filled outer track outline: the infield is also available to explore."""
    def __init__(self, normalized_contour):
        import numpy as np
        points = normalized_contour.reshape(-1,2)
        self.contour = np.column_stack(((.5-points[:,1])*12, (.5-points[:,0])*16)).astype(np.float32)

    def contains(self, x, y):
        import cv2
        # Vehicle bounds extend .25 m into grass; keep the viewpoint another
        # .15 m inside the painted outer edge.
        return cv2.pointPolygonTest(self.contour, (float(x),float(y)), True) >= .4

    def restrict(self, start, end):
        if self.contains(*end):
            return end
        low, high = 0., 1.
        for _ in range(20):
            t = (low+high)/2
            if self.contains(start[0]+(end[0]-start[0])*t, start[1]+(end[1]-start[1])*t):
                low = t
            else:
                high = t
        return (start[0]+(end[0]-start[0])*low, start[1]+(end[1]-start[1])*low)


@dataclass
class VenueView:
    x: float = HOME[0]
    y: float = HOME[1]
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0
    pitch: float = PITCH
    pitch_rate: float = 0.0
    target: tuple = None
    target_yaw: float = 0.
    boundary: object = None

    def focus(self, car=None):
        self.target_yaw = math.atan2(car[1], car[0]) if car else 0.
        distance = (HEIGHT-.35)/math.tan(PITCH)
        self.target = ((car[0]-distance*math.cos(self.target_yaw),
                        car[1]-distance*math.sin(self.target_yaw)) if car else HOME)
        self.target = (max(X_LIMITS[0], min(X_LIMITS[1], self.target[0])),
                       max(Y_LIMITS[0], min(Y_LIMITS[1], self.target[1])))
        if self.boundary:
            self.target = self.boundary.restrict(HOME, self.target)

    @property
    def pose(self):
        return (self.x, self.y, HEIGHT, self.pitch, self.yaw)

    def move(self, right, forward, dt, turn=0., tilt=0.):
        dt = max(0, min(.08, dt))
        def axis(value):
            return math.copysign(max(0., (min(1., abs(value))-.14)/.86), value)
        right, forward, turn, tilt = axis(right), axis(forward), axis(turn), axis(tilt)
        norm = max(1., math.hypot(right, forward))
        if right or forward or turn or tilt:
            self.target = None
        blend = 1-math.exp(-10*dt)
        angle = math.atan2(math.sin(self.target_yaw-self.yaw), math.cos(self.target_yaw-self.yaw))
        target_turn = max(-1.3, min(1.3, angle*3)) if self.target else -turn*1.1
        self.yaw_rate += (target_turn-self.yaw_rate)*blend
        self.yaw = math.atan2(math.sin(self.yaw+self.yaw_rate*dt), math.cos(self.yaw+self.yaw_rate*dt))
        target_tilt = max(-.95, min(.95, (PITCH-self.pitch)*3)) if self.target else tilt*.95
        self.pitch_rate += (target_tilt-self.pitch_rate)*blend
        self.pitch = max(PITCH_LIMITS[0], min(PITCH_LIMITS[1], self.pitch+self.pitch_rate*dt))
        if self.target:
            dx, dy = self.target[0]-self.x, self.target[1]-self.y
            length = math.hypot(dx, dy)
            speed = min(4.0, length*3)
            tx, ty = (speed*dx/length, speed*dy/length) if length > .01 else (0., 0.)
        else:
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            tx, ty = 3*(forward*cy+right*sy)/norm, 3*(forward*sy-right*cy)/norm
        self.vx += (tx-self.vx)*blend
        self.vy += (ty-self.vy)*blend
        target = (max(X_LIMITS[0], min(X_LIMITS[1], self.x+self.vx*dt)),
                  max(Y_LIMITS[0], min(Y_LIMITS[1], self.y+self.vy*dt)))
        if self.boundary:
            position = self.boundary.restrict((self.x,self.y), target)
            position = self.boundary.restrict(position, (target[0], position[1]))
            target = self.boundary.restrict(position, (position[0], target[1]))
        self.x, self.y = target


def project(point, pose, aspect=16/9):
    dx, dy, dz = point[0]-pose[0], point[1]-pose[1], point[2]-pose[2]
    yaw = pose[4] if len(pose)>4 else 0.
    dx, dy = math.cos(yaw)*dx+math.sin(yaw)*dy, -math.sin(yaw)*dx+math.cos(yaw)*dy
    cp, sp = math.cos(pose[3]), math.sin(pose[3])
    depth = cp*dx-sp*dz
    if depth <= .1:
        return None
    u = .5-dy/(2*depth*math.tan(FOV/2))
    v = .5-(sp*dx+cp*dz)*aspect/(2*depth*math.tan(FOV/2))
    return (u,v) if 0<u<1 and 0<v<1 else None


def camera_sdf(rate_plugin='libexhibition_camera_rate.so'):
    return f'''<sdf version="1.6"><model name="{ENTITY}">
      <static>true</static><link name="camera_link"><gravity>false</gravity>
        <sensor name="venue" type="camera"><always_on>false</always_on><update_rate>120</update_rate>
          <camera><horizontal_fov>{FOV}</horizontal_fov>
            <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>60</far></clip>
          </camera>
          <plugin name="venue_camera" filename="libgazebo_ros_camera.so">
            <ros><namespace>/exhibition/venue</namespace>
              <remapping>venue/image_raw:={TOPIC}</remapping>
              <remapping>venue/camera_info:=/exhibition/venue/camera_info</remapping>
            </ros><camera_name>venue</camera_name><frame_name>venue_camera_link</frame_name>
          </plugin>
          <plugin name="exhibition_venue_rate" filename="{rate_plugin}">
            <rate_topic>/exhibition/camera_rate/venue</rate_topic><initial_rate>120</initial_rate>
          </plugin>
        </sensor>
      </link></model></sdf>'''
