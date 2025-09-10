import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def push(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def pop(self):
        return heapq.heappop(self.elements)[1]

    def empty(self):
        return not self.elements

class DijkstraNavigator(Node):
    def __init__(self):
        super().__init__('dijkstra_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.map = self.load_fake_map()
        self.robot_pose = None
        self.goal = None
        self.path = []
        self.current_index = 0

        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ Dijkstra navigator ready.")

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
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)
        self.robot_pose = (x, y, yaw)

    def goal_callback(self, msg):
        gx = int(round(msg.pose.position.x))
        gy = int(round(msg.pose.position.y))
        self.goal = (gx, gy)
        self.get_logger().info(f"📍 Received goal: {self.goal}")

        if self.robot_pose:
            rx = int(round(self.robot_pose[0]))
            ry = int(round(self.robot_pose[1]))
            self.get_logger().info(f"🤖 Robot grid position: ({rx}, {ry})")

            if not self.is_free(rx, ry):
                self.get_logger().warn(f"❌ Start position ({rx}, {ry}) is not free.")
                return
            if not self.is_free(gx, gy):
                self.get_logger().warn(f"❌ Goal position ({gx}, {gy}) is not free.")
                return

            self.path = self.dijkstra(self.map, (rx, ry), self.goal)
            self.current_index = 0

            if self.path:
                self.get_logger().info(f"✅ Path planned with {len(self.path)} steps.")
            else:
                self.get_logger().warn("❌ No path found.")

    def is_free(self, x, y):
        # Access map[y][x], not map[x][y]
        if 0 <= y < len(self.map) and 0 <= x < len(self.map[0]):
            value = self.map[y][x]
            self.get_logger().info(f"Checking cell ({x}, {y}) = {value}")
            return value == 0
        return False

    def quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def get_neighbors(self, current):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= ny < len(self.map) and 0 <= nx < len(self.map[0]):
                if self.map[ny][nx] == 0:
                    neighbors.append((nx, ny))
        self.get_logger().debug(f"Neighbors of {current}: {neighbors}")
        return neighbors

    def movement_cost(self, a, b):
        return 1

    def reconstruct_path(self, came_from, start, goal):
        path = [goal]
        while goal != start:
            goal = came_from[goal]
            path.append(goal)
        path.reverse()
        return path

    def dijkstra(self, map_grid, start, goal):
        frontier = PriorityQueue()
        frontier.push(start, 0)
        came_from = {}
        cost_so_far = {start: 0}

        while not frontier.empty():
            current = frontier.pop()
            if current == goal:
                break

            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.movement_cost(current, neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    frontier.push(neighbor, new_cost)
                    came_from[neighbor] = current

        if goal in came_from:
            return self.reconstruct_path(came_from, start, goal)
        else:
            return []

    def timer_callback(self):
        if not self.path or self.robot_pose is None:
            return

        if self.current_index >= len(self.path):
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🎯 Goal reached.")
            return

        rx, ry, ryaw = self.robot_pose
        tx, ty = self.path[self.current_index]
        dx = tx - rx
        dy = ty - ry
        distance = math.sqrt(dx**2 + dy**2)
        path_angle = math.atan2(dy, dx)
        heading_error = self.angle_diff(path_angle, ryaw)

        cmd = Twist()
        cmd.angular.z = 1.5 * heading_error

        if abs(heading_error) < 0.3:
            cmd.linear.x = min(0.3, distance)

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"➡ Moving to {self.path[self.current_index]} | "
            f"vx: {cmd.linear.x:.2f}, wz: {cmd.angular.z:.2f}"
        )

        if distance < 0.2:
            self.current_index += 1

    def angle_diff(self, target, source):
        a = target - source
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
