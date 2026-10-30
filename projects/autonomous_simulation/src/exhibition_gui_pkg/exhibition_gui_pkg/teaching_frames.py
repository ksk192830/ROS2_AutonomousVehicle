"""Same-exposure camera/segmentation/BEV snapshots for the moving lessons."""


class TeachingFrames:
    def __init__(self, capacity=16):
        self.capacity = capacity
        self.pending = {}
        self.latest = None

    def add(self, stamp, key, image, pixels, received_at):
        if self.latest is not None and stamp < self.latest[0] - 1_000_000_000:
            self.pending.clear()
            self.latest = None
        bucket = self.pending.setdefault(stamp, {})
        bucket[key] = (image, pixels)
        if all(name in bucket for name in ('camera', 'segmentation', 'bev')):
            if self.latest is None or stamp > self.latest[0]:
                self.latest = (stamp, {name: value[0] for name, value in bucket.items()},
                               bucket['segmentation'][1], received_at)
            del self.pending[stamp]
        while len(self.pending) > self.capacity:
            del self.pending[min(self.pending)]
