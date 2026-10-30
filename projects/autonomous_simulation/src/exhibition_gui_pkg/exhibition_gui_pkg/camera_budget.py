"""Spend rendering work on the camera visible in the current visitor step."""
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64


def camera_rates(state, phase, preparing=False):
    touring = state == 'EXPLORE'
    race = phase in ('CHALLENGE', 'RESULTS') or preparing
    return {'front': 20. if state == 'READY' or touring else 120.,
            'overview': 0. if state == 'READY' or touring else 120.,
            'opponent': 20. if race else 0.,
            'venue': 120. if touring else 0.}


class CameraBudget:
    def __init__(self, node):
        self.node = node
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publishers = {key: node.create_publisher(Float64, f'/exhibition/camera_rate/{key}', qos)
                           for key in ('front', 'overview', 'opponent', 'venue')}
        self.rates = None

    def update(self):
        rates = camera_rates(self.node.state, self.node.session.phase,
                             bool(self.node.duel and self.node.duel.preparing))
        if rates == self.rates:
            return
        self.rates = rates
        for key, rate in rates.items():
            self.publishers[key].publish(Float64(data=rate))
