import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import math

class RvizController(Node):
    def __init__(self):
        super().__init__('rviz_controller')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.1, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.sub = self.create_subscription(Twist, 'nn_cmd', self.cmd_callback, 10)
        self.current_cmd = Twist()
        self.get_logger().info("rviz_controller started and waiting for commands.")

    def cmd_callback(self, msg):
        self.current_cmd = msg

    def timer_callback(self):
        # Integrate motion
        dt = 0.1
        self.x += self.current_cmd.linear.x * dt
        self.y += self.current_cmd.linear.y * dt
        self.yaw += self.current_cmd.angular.z * dt

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RvizController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
