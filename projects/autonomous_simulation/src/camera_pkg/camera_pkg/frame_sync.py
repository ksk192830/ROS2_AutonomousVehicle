"""Bounded exact-source pairing without reusing detections on newer images."""
from collections import OrderedDict


def stamp_key(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class FramePairs:
    def __init__(self, max_age=.8, capacity=40):
        self.max_age = max_age
        self.capacity = capacity
        self.images = OrderedDict()
        self.detections = OrderedDict()
        self.last_image_stamp = None
        self.last_processed = None
        self.generation = 0

    def add(self, message, now, image=False):
        key = stamp_key(message)
        if image:
            if self.last_image_stamp is not None and key < self.last_image_stamp - 500_000_000:
                self.images.clear()
                self.detections.clear()
                self.last_processed = None
                self.generation += 1
            self.last_image_stamp = key
        if self.last_processed is not None and key <= self.last_processed:
            return None
        own = self.images if image else self.detections
        own[key] = (now, message)
        for cache in (self.images, self.detections):
            for old in list(cache):
                if now - cache[old][0] > self.max_age:
                    del cache[old]
            while len(cache) > self.capacity:
                cache.popitem(last=False)
        if key not in self.images or key not in self.detections:
            return None
        received, source = self.images.pop(key)
        _, result = self.detections.pop(key)
        self.last_processed = key
        return source, result, received
