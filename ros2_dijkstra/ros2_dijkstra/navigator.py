import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

from ros2_dijkstra.utils import quaternion_to_yaw, angle_diff
from ros2_dijkstra.dijkstra import dijkstra


class DijkstraNavigator(Node):
    def __init__(self):
        super().__init__('dijkstra_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.map = self.load_fake_map()
        self.map_height = len(self.map)
        self.map_width = len(self.map[0])

        self.robot_pose = None
        self.goal = None
        self.path = []
        self.current_index = 0

        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Dijkstra navigator ready.")

    def load_fake_map(self):
        return [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
            [0, 1, 0, 0, 0, 1, 1, 0, 1, 0],
            [0, 1, 1, 1, 1, 1, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        ]

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_pose = (x, y, yaw)
        self.get_logger().debug(f"odom_callback: Robot pose updated: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    def goal_callback(self, msg):
        gx = int(round(msg.pose.position.x))
        gy = int(round(msg.pose.position.y))
        self.goal = (gx, gy)
        self.get_logger().info(f"Received goal: {self.goal}")

        if self.robot_pose:
            rx = int(round(self.robot_pose[0]))
            ry = int(round(self.robot_pose[1]))
            self.get_logger().info(f"Robot grid position: ({rx}, {ry})")

            if not self.is_free(rx, ry):
                self.get_logger().warn(f"Start position ({rx}, {ry}) is not free.")
                return
            if not self.is_free(gx, gy):
                self.get_logger().warn(f"Goal position ({gx}, {gy}) is not free.")
                return

            self.path = dijkstra(self.map, (rx, ry), self.goal)
            self.current_index = 0

            if self.path:
                self.get_logger().info(f"Path planned with {len(self.path)} steps.")
            else:
                self.get_logger().warn("No path found.")

    def is_free(self, x, y):
        if 0 <= y < self.map_height and 0 <= x < self.map_width:
            return self.map[y][x] == 0
        return False

    def timer_callback(self):
        if not self.path or self.robot_pose is None:
            return

        if self.current_index >= len(self.path):
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Goal reached.")
            return

        rx, ry, ryaw = self.robot_pose
        tx, ty = self.path[self.current_index]
        dx = tx - rx
        dy = ty - ry
        distance = math.sqrt(dx**2 + dy**2)
        path_angle = math.atan2(dy, dx)
        heading_error = angle_diff(path_angle, ryaw)

        cmd = Twist()
        cmd.angular.z = 1.5 * heading_error

        if abs(heading_error) < 0.6:
            cmd.linear.x = min(0.6, distance)
        else:
            cmd.linear.x = 0.0

        self.get_logger().info(
            f"timer_callback: "
            f"Robot=({rx:.2f},{ry:.2f},{ryaw:.2f}), "
            f"Target=({tx},{ty}), "
            f"Dist={distance:.2f}, "
            f"Heading error={heading_error:.2f}, "
            f"Cmd linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}"
        )

        self.cmd_pub.publish(cmd)

        if distance < 0.2:
            self.current_index += 1
            return

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

