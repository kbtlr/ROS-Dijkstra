from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    print("=== Launching full system with robot_state_publisher, rviz_controller, ros_neural_net, and rviz2 ===")

    urdf_path = os.path.join(
        get_package_share_directory('ros2_neural_net'),
        'urdf',
        'robot.urdf'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('ros2_neural_net'),
        'rviz',
        'config.rviz'
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='ros2_neural_net',
            executable='rviz_controller',  # console_script
            name='rviz_controller',
            output='screen'
        ),
        Node(
            package='ros2_neural_net',
            executable='ros_neural_net',  # console_script
            name='ros_neural_net',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
