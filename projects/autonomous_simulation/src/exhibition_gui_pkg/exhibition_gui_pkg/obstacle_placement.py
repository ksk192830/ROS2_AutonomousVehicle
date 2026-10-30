"""Shared spawn positions and proximity rules for UI and actual spawning."""
import math

# Exhibition positions selected from the original mission cases. Keep enough
# space to finish passing one car before changing lanes for the next. Randomly
# combining the old cases could leave only 2.4 m between consecutive cars.
OBSTACLES = {
    1: ('obstacle1', (('fixed', (3.4, -5.1, 0.0, 0.9)),)),
    2: (
        'obstacle2',
        (
            ('case1', (4.9, -1.4, 0.0, 1.7)),
        ),
    ),
    3: (
        'obstacle3',
        (
            ('case3', (3.6, 3.0, 0.0, 1.7)),
        ),
    ),
}

# Distances are between model origins in world metres. Both vehicle models
# extend roughly one metre forward from their rear origin. Include clearance
# for those bodies and for the car moving between a click and service handling.
# Creating a car also costs spawn + camera inference time before avoidance
# starts. 2.5 m prevented instant overlap but did not leave room to go around.
SPAWN_BLOCK_DISTANCE_M = 3.5
SPAWN_RELEASE_DISTANCE_M = 3.8
POSE_MAX_AGE_SEC = 0.5


class ObstacleSpawnGuard:
    def __init__(self):
        self.nearby = set()

    def reasons(self, pose, received_at, now):
        if (pose is None or len(pose) < 2 or
                not all(math.isfinite(value) for value in pose[:2]) or
                now - received_at > POSE_MAX_AGE_SEC):
            return ['position'] * len(OBSTACLES)
        result = []
        for index, (_, cases) in OBSTACLES.items():
            # Check EVERY candidate: random placement must never pick a nearby
            # case even when the location shown on the small map looks distant.
            distance = min(math.hypot(pose[0] - values[0], pose[1] - values[1])
                           for _, values in cases)
            limit = SPAWN_RELEASE_DISTANCE_M if index in self.nearby else SPAWN_BLOCK_DISTANCE_M
            if distance <= limit:
                self.nearby.add(index)
                result.append('nearby')
            else:
                self.nearby.discard(index)
                result.append('')
        return result
