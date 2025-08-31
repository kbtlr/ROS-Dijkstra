import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import math

print("💡 Running rviz_controller.py file!")

class MoverNode(Node):
    def __init__(self):
        super().__init__('rviz_controller')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0.0
        self.get_logger().info("MoverNode started.")

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.position.x = 2.0 * math.cos(self.counter)
        marker.pose.position.y = 2.0 * math.sin(self.counter)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        self.publisher.publish(marker)
        self.get_logger().info(f"Published Marker at position x={marker.pose.position.x:.2f}, y={marker.pose.position.y:.2f}")
        self.counter += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = MoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
