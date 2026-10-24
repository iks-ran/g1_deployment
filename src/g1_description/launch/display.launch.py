"""Offline URDF check: robot_state_publisher + joint_state_publisher_gui + RViz (no robot needed)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('g1_description'), 'urdf', 'g1.urdf')
    rviz = os.path.join(get_package_share_directory('g1_bringup'), 'rviz', 'g1.rviz')
    with open(urdf) as f:
        robot_description = f.read()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz, '-f', 'pelvis']),
    ])
