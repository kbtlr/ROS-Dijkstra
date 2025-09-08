import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NeuralNetController(Node):
    def __init__(self):
        super().__init__('ros_dijkstra')
        self.pub = self.create_publisher(Twist, 'nn_cmd', 10)
        self.create_timer(0.5, self.timer_callback)
        self.toggle = True
        self.get_logger().info("ros_dijkstra started.")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2 if self.toggle else 0.0
        msg.angular.z = 0.5 if self.toggle else -0.5
        self.pub.publish(msg)
        self.get_logger().info(f"Published cmd: vx={msg.linear.x}, wz={msg.angular.z}")
        self.toggle = not self.toggle

def main(args=None):
    rclpy.init(args=args)
    node = NeuralNetController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
