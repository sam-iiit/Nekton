import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_sim_bringup = get_package_share_directory('sim_bringup')
    bridge_config_path = os.path.join(pkg_sim_bringup, 'config','odom_bridge.yml')

    # Bridge
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_odom_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    return LaunchDescription([
        odom_bridge
    ])
