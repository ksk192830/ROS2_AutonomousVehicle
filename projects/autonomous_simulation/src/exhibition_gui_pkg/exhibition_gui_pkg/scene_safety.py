"""Exhibition safety envelope, separate from the camera's driving decisions."""
import math


def body_center(pose):
    x, y, yaw = pose
    # Both exhibition cars use the rear of the 0.99 x 0.55 m chassis as origin.
    return x + .495 * math.cos(yaw), y + .495 * math.sin(yaw)


def bodies_overlap(first, second, padding=0.0):
    """Separating-axis test for the actual oriented chassis footprints."""
    a, b = body_center(first), body_center(second)
    axes = [(math.cos(p[2]), math.sin(p[2])) for p in (first, second)]
    sides = [(-axis[1], axis[0]) for axis in axes]
    delta = (b[0] - a[0], b[1] - a[1])
    for test in (*axes, *sides):
        distance = abs(delta[0] * test[0] + delta[1] * test[1])
        radius = sum((.495 + padding) * abs(axis[0]*test[0] + axis[1]*test[1]) +
                     (.275 + padding) * abs(side[0]*test[0] + side[1]*test[1])
                     for axis, side in zip(axes, sides))
        if distance > radius:
            return False
    return True


class ObstacleSafety:
    def __init__(self):
        self.pose = None
        self.obstacles = {}
        self.velocity = (0.0, 0.0, 0.0)
        self.received_at = 0.0
        self.side_blocked = False

    def update(self, pose, obstacles, velocity, received_at):
        self.pose, self.obstacles = pose, dict(obstacles)
        self.velocity, self.received_at = velocity, received_at

    def lane_change_blocked(self, direction, full_manoeuvre=False):
        """Check the swept body along a proposed lane change.

        A car beside our rear quarter need not prevent an automatic manoeuvre:
        the front can move into the free road ahead while the rear passes it.
        Manual requests also check the destination of the full manoeuvre and
        retain the stricter side-by-side interlock in status().
        """
        if not self.obstacles:
            return False
        speed = math.hypot(*self.velocity[:2])
        curvature = max(-.35, min(.35, self.velocity[2] / max(.35, speed)))
        x, y, yaw = self.pose
        distances = (0, .15, .3, .45, .6, .8, 1.0, 1.2, 1.4)
        if full_manoeuvre:
            # A visitor can ask to return to a lane that still contains the
            # car we just avoided. Check the destination, not only our side.
            distances += (1.6, 1.9, 2.2, 2.5, 2.8, 3.2)
        for distance in distances:
            progress = distance / 3.2
            offset = direction * 1.15 * (3*progress**2 - 2*progress**3)
            slope = direction * 1.15 * 6*progress*(1-progress) / 3.2
            angle = curvature * distance
            if abs(curvature) < 1e-5:
                forward, side = distance, 0.0
            else:
                forward = math.sin(angle) / curvature
                side = (1-math.cos(angle)) / curvature
            forward -= offset * math.sin(angle)
            side += offset * math.cos(angle)
            predicted = (x + forward*math.cos(yaw) - side*math.sin(yaw),
                         y + forward*math.sin(yaw) + side*math.cos(yaw),
                         yaw + angle + math.atan(slope))
            if any(bodies_overlap(predicted, p, padding=.04) for p in self.obstacles.values()):
                return True
        return False

    def status(self, now):
        if self.pose is None or now - self.received_at > .5:
            return dict(lane_blocked=True, reason='position', stop=True, overlap=False,
                        auto_lane1_blocked=True, auto_lane2_blocked=True,
                        manual_lane1_blocked=True, manual_lane2_blocked=True)
        center = body_center(self.pose)
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        # Lock while the bodies overlap along the direction of travel. A large
        # fixed radius also locked cars that had already passed, leaving no
        # useful gap in which to avoid the next obstacle.
        clearance = .14 if self.side_blocked else .08
        lateral = 2.3 if self.side_blocked else 2.2
        blocked = False
        for other in self.obstacles.values():
            point = body_center(other)
            dx, dy = point[0] - center[0], point[1] - center[1]
            relative_yaw = other[2] - self.pose[2]
            longitudinal = (.495 + .495 * abs(math.cos(relative_yaw)) +
                            .275 * abs(math.sin(relative_yaw)) + clearance)
            if abs(dx*c + dy*s) <= longitudinal and abs(-dx*s + dy*c) <= lateral:
                blocked = True
        self.side_blocked = blocked
        overlap = any(bodies_overlap(self.pose, p) for p in self.obstacles.values())
        # Keep a forward envelope after braking to avoid stop/restart oscillation.
        speed = max(.85, math.hypot(*self.velocity[:2]))
        omega = self.velocity[2]
        danger = overlap
        # Measured braking coast is about 0.45 m at exhibition speed. Include
        # command latency; the old 0.4 s lookahead stopped too late after a
        # visitor steered towards an occupied lane.
        for elapsed in (0.0, .1, .2, .3, .4, .5, .6, .7):
            yaw = self.pose[2] + omega * elapsed
            mid = self.pose[2] + omega * elapsed * .5
            predicted = (self.pose[0] + speed*elapsed*math.cos(mid),
                         self.pose[1] + speed*elapsed*math.sin(mid), yaw)
            if any(bodies_overlap(predicted, p, padding=.04) for p in self.obstacles.values()):
                danger = True
                break
        return dict(lane_blocked=blocked, reason='obstacle' if blocked else '',
                    stop=danger, overlap=overlap,
                    auto_lane1_blocked=self.lane_change_blocked(1),
                    auto_lane2_blocked=self.lane_change_blocked(-1),
                    manual_lane1_blocked=blocked or self.lane_change_blocked(1, True),
                    manual_lane2_blocked=blocked or self.lane_change_blocked(-1, True))


def read_scene(message, vehicle='ego_vehicle'):
    """Only spawned obstacles count; empty spawn locations never lock lanes."""
    pose, velocity, obstacles = None, (0.0, 0.0, 0.0), {}
    for index, name in enumerate(message.name):
        if name != vehicle and name not in ('obstacle1', 'obstacle2', 'obstacle3'):
            continue
        value = message.pose[index]
        q = value.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        location = (float(value.position.x), float(value.position.y), yaw)
        if not all(math.isfinite(v) for v in location):
            continue
        if name == vehicle:
            pose = location
            twist = message.twist[index]
            velocity = (float(twist.linear.x), float(twist.linear.y), float(twist.angular.z))
        else:
            obstacles[name] = location
    return pose, obstacles, velocity


class ContactConfirmation:
    """Confirm physical body contact, never distance or predicted overlap."""
    GROUND = {'ground', 'ground_plane', 'grass_ground', 'marble_floor', 'race_track'}

    def __init__(self, vehicle='ego_vehicle'):
        self.vehicle = vehicle
        self.minimum_stamp = 0
        self.reset()

    def reset(self, minimum_stamp=None):
        if minimum_stamp is not None:
            self.minimum_stamp = minimum_stamp
        self.last_pair = None
        self.last_stamp = 0
        self.last_at = 0.0
        self.pending_until = 0.0

    def observe(self, message, now):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= self.minimum_stamp:
            return False
        pairs = []
        strong_impact = False
        for contact in message.states:
            names = (contact.collision1_name, contact.collision2_name)
            if not any(name.startswith(self.vehicle + '::chassis::') for name in names):
                continue
            other = [name for name in names if not name.startswith(self.vehicle + '::')]
            if not other or other[0].split('::')[0] in self.GROUND:
                continue
            force = contact.total_wrench.force
            strength = math.sqrt(force.x**2 + force.y**2 + force.z**2)
            depth = max(contact.depths, default=0.0)
            # A hard impact may separate the bodies within one sensor frame.
            # Require BOTH penetration and load for that single-frame case.
            strong_impact |= depth >= .005 and strength >= 30.0
            # Ignore numerical touching with no appreciable penetration/load.
            if depth > .001 or strength > 5.0:
                # Gazebo may report wheel/body collision shapes in a different
                # order on adjacent frames. Confirm contact with the same car,
                # rather than requiring the exact same pair of sub-shapes.
                pairs.append(other[0].split('::')[0])
        if not pairs:
            self.reset()
            return False
        pair = self.last_pair if self.last_pair in pairs else pairs[0]
        confirmed = (pair == self.last_pair and
                     15_000_000 <= stamp - self.last_stamp <= 150_000_000 and
                     now - self.last_at <= .2)
        if stamp > self.last_stamp:
            self.last_pair, self.last_stamp, self.last_at = pair, stamp, now
            self.pending_until = now + .15
        return strong_impact or confirmed
