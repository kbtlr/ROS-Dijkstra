from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    print("=== Launching full system with static_tf_pub, rviz_controller, and rviz2 ===")
    rviz_config_path = os.path.join(
        get_package_share_directory('ros2_neural_net'),
        'rviz',
        'config.rviz'
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',  # ensure this is set
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),
        Node(
            package='ros2_neural_net',
            executable='rviz_controller',
            name='rviz_controller',
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