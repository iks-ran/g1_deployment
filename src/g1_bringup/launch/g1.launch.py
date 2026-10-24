"""G1 bringup: robot_state_publisher + state bridge (TF/joints/lidar/IMU) in one component container, + RViz.

Requires the unitree_ros2 environment (source ./setup.zsh) so that the unitree_hg / unitree_go messages
and the CycloneDDS interface config are available. Works the same on a replayed bag (scripts/play_bag.zsh).
Both nodes share the process `g1_container`. No intra-process comms: Humble allows it only for volatile topics,
and robot_state_publisher's /tf_static and /robot_description are transient local (its constructor throws).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('g1_description'), 'urdf', 'g1.urdf')
    rviz_cfg = os.path.join(get_package_share_directory('g1_bringup'), 'rviz', 'g1.rviz')
    with open(urdf) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        ComposableNodeContainer(
            name='g1_container', namespace='', package='rclcpp_components', executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(package='robot_state_publisher', plugin='robot_state_publisher::RobotStatePublisher',
                               name='robot_state_publisher',
                               parameters=[{'robot_description': robot_description}]),
                ComposableNode(package='g1_bringup', plugin='g1_bringup::StateBridge', name='g1_state_bridge'),
            ]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg, '-f', 'odom'],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
