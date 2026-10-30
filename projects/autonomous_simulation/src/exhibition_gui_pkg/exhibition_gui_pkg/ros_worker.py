"""Keep ROS callbacks from competing with Qt paint events on the UI thread."""
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException


class RosWorker:
    def __init__(self, node, on_exit):
        self.node = node
        self.on_exit = on_exit
        self.running = True
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.run, name='exhibition-ros', daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        try:
            while self.running and rclpy.ok(context=self.node.context):
                self.executor.spin_once(timeout_sec=.05)
        except ExternalShutdownException:
            pass
        except Exception as error:
            self.node.get_logger().error(f'Exhibition callback failed: {error!r}')
        finally:
            if self.running:
                self.on_exit()

    def close(self):
        self.running = False
        self.executor.wake()
        self.thread.join()
        self.executor.shutdown()
        self.executor.remove_node(self.node)
