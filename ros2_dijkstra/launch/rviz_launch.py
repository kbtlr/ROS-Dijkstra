from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    print("=== Launching Dijkstra Navigator with full TF, URDF, and RViz setup ===")

    pkg_dir = get_package_share_directory('ros2_dijkstra')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Publishes joint states (required by robot_state_publisher)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        # Publishes robot state and TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        # Simulates odometry and moves base_link using cmd_vel
        Node(
            package='ros2_dijkstra',
            executable='rviz_controller',
            name='rviz_controller',
            output='screen'
        ),
        # Runs the Dijkstra planner and publishes cmd_vel
        Node(
            package='ros2_dijkstra',
            executable='ros_dijkstra',
            name='ros_dijkstra',
            output='screen'
        ),
        # Static transform between map and odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])
