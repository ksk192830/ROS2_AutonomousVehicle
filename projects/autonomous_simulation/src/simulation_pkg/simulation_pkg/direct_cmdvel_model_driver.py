import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState, ModelStates


class DirectCmdVelModelDriver(Node):
    def __init__(self):
        super().__init__('direct_cmdvel_model_driver')

        self.declare_parameter('cmd_topic', '/cmd_vel_direct')
        self.declare_parameter('entity_name', '')  # empty => auto-detect mercedes*
        self.declare_parameter('reference_frame', 'world')

        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.entity_name = self.get_parameter('entity_name').get_parameter_value().string_value.strip()
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value

        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 10)
        self.sub_states = self.create_subscription(ModelStates, '/gazebo/model_states', self.states_cb, 10)

        self.latest_states = None
        self.resolved_entity = None
        self.last_warn_ns = 0

        self.get_logger().info(
            f'Direct model driver listening on {self.cmd_topic} '
            f'(entity_name={self.entity_name or "<auto>"})')

    def states_cb(self, msg: ModelStates):
        self.latest_states = msg

        if self.entity_name:
            self.resolved_entity = self.entity_name
            return

        # auto-detect: prefer mercedes* spawned model name
        candidates = [n for n in msg.name if 'mercedes' in n.lower()]
        if candidates:
            self.resolved_entity = candidates[0]
        elif msg.name:
            # fallback to first non-world model
            self.resolved_entity = next((n for n in msg.name if n != 'ground_plane'), msg.name[0])

    def _find_entity_pose(self, name: str):
        if self.latest_states is None:
            return None
        try:
            idx = self.latest_states.name.index(name)
            return self.latest_states.pose[idx]
        except ValueError:
            return None

    def cmd_cb(self, msg: Twist):
        if not self.client.service_is_ready():
            self.client.wait_for_service(timeout_sec=0.5)
            if not self.client.service_is_ready():
                return

        name = self.resolved_entity or self.entity_name
        if not name:
            self._throttled_warn('Entity not resolved yet. Waiting for /gazebo/model_states...')
            return

        pose = self._find_entity_pose(name)
        if pose is None:
            self._throttled_warn(f'Entity [{name}] not found in /gazebo/model_states')
            return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.reference_frame = self.reference_frame
        req.state.pose = pose  # keep current pose/orientation, only update twist

        req.state.twist.linear.x = float(msg.linear.x)
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = float(msg.angular.z)

        fut = self.client.call_async(req)
        fut.add_done_callback(self._service_done)

    def _service_done(self, fut):
        try:
            res = fut.result()
            if res is None or (hasattr(res, 'success') and not res.success):
                status = getattr(res, 'status_message', 'unknown failure') if res else 'no response'
                self._throttled_warn(f'set_entity_state failed: {status}')
        except Exception as e:
            self._throttled_warn(f'set_entity_state exception: {e}')

    def _throttled_warn(self, msg: str, sec: float = 2.0):
        now = self.get_clock().now().nanoseconds
        if now - self.last_warn_ns > int(sec * 1e9):
            self.get_logger().warn(msg)
            self.last_warn_ns = now


def main(args=None):
    rclpy.init(args=args)
    node = DirectCmdVelModelDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
