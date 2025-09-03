from setuptools import setup
from glob import glob
import os

package_name = 'ros2_neural_net'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='someone',
    maintainer_email='someone@someplace.com',
    description='ROS 2 neural net integration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ros_neural_net = ros2_neural_net.ros_neural_net:main',
            'rviz_controller = ros2_neural_net.rviz_controller:main',
        ],
    },
)

