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

class Dijkstra(Node):
    def __init__(self):
        super().__init__('ros_dijkstra')

        # Publisher for velocity commands
        self.pub = self.create_publisher(Twist, 'nn_cmd', 10)

        # Subscribers for robot pose and goal pose
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Fake 2D occupancy grid (0 = free, 1 = obstacle)
        self.map = self.load_fake_map()

        # Robot and goal state variables
        self.robot_pose = None  # (x, y, yaw)
        self.goal = None        # (x, y) grid coordinates
        self.path = []
        self.current_index = 0

        # Timer to update robot commands at 10Hz
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("ros_dijkstra node with dynamic goal and odom started.")

    def load_fake_map(self):
        # 0 = free, 1 = obstacle
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
        # Extract robot's current position (x,y) and yaw from Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)
        self.robot_pose = (x, y, yaw)

    def goal_callback(self, msg):
        # Convert PoseStamped goal to integer grid cell coords
        gx = int(round(msg.pose.position.x))
        gy = int(round(msg.pose.position.y))
        self.goal = (gx, gy)
        self.get_logger().info(f"Received new goal: {self.goal}")

        if self.robot_pose is not None:
            rx = int(round(self.robot_pose[0]))
            ry = int(round(self.robot_pose[1]))
            self.path = self.dijkstra(self.map, (rx, ry), self.goal)
            self.current_index = 0
            self.get_logger().info(f"Planned path with {len(self.path)} waypoints.")

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw angle (radians)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_neighbors(self, current):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < len(self.map) and 0 <= ny < len(self.map[0]):
                if self.map[nx][ny] == 0:
                    neighbors.append((nx, ny))
        return neighbors

    def movement_cost(self, a, b):
        return 1

    def reconstruct_path(self, came_from, start, goal):
        path = [goal]
        current = goal
        while current != start:
            current = came_from[current]
            path.append(current)
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
            self.get_logger().warn("No path found!")
            return []

    def timer_callback(self):
        if not self.path or self.robot_pose is None:
            # No path or robot pose yet
            return

        if self.current_index >= len(self.path):
            self.stop_robot()
            self.get_logger().info("Reached goal.")
            return

        rx, ry, ryaw = self.robot_pose
        nx, ny = self.path[self.current_index]
        nx_m = float(nx)
        ny_m = float(ny)

        dx = nx_m - rx
        dy = ny_m - ry
        distance = math.sqrt(dx**2 + dy**2)
        path_angle = math.atan2(dy, dx)
        heading_error = self.angle_diff(path_angle, ryaw)

        cmd = Twist()
        cmd.angular.z = 1.5 * heading_error  # P gain

        # Move forward only if facing roughly toward target waypoint
        if abs(heading_error) < 0.3:
            cmd.linear.x = min(0.3, distance)

        self.pub.publish(cmd)
        self.get_logger().info(f"Moving to waypoint {self.path[self.current_index]} | vx={cmd.linear.x:.2f} wz={cmd.angular.z:.2f}")

        if distance < 0.2:
            self.current_index += 1

    def stop_robot(self):
        self.pub.publish(Twist())

    def angle_diff(self, target, source):
        a = target - source
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = Dijkstra()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

