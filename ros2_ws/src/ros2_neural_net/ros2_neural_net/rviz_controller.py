import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class RvizController(Node):
    def __init__(self):
        super().__init__('rviz_controller')
        self.subscription = self.create_subscription(
            String,
            'rviz_instructions',
            self.listener_callback,
            10)
        self.process = None

    def listener_callback(self, msg):
        instruction = msg.data
        self.get_logger().info(f'Received instruction: "{instruction}"')
        if instruction == 'start':
            if self.process is None or self.process.poll() is not None:
                self.process = subprocess.Popen(['ros2', 'launch', 'ros2_neural_net', 'rviz_launch.py'])
                self.get_logger().info('Launched rviz_launch.py')
            else:
                self.get_logger().info('rviz_launch.py is already running.')
        elif instruction == 'stop':
            if self.process and self.process.poll() is None:
                self.process.terminate()
                self.get_logger().info('Terminated rviz_launch.py')
            else:
                self.get_logger().info('rviz_launch.py is not running.')

def main(args=None):
    rclpy.init(args=args)
    node = RvizController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.process and node.process.poll() is None:
            node.process.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()