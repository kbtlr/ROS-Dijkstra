from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments expected from rviz_controller.py
    arg1 = DeclareLaunchArgument(
        'config_file',
        default_value='default.rviz',
        description='RViz config file'
    )
    arg2 = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        arg1,
        arg2,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('config_file')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])