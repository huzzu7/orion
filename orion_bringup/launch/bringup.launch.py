import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


import xacro


def generate_launch_description():
    
    bringup_pkg = get_package_share_directory('orion_bringup')
    drive_pkg = get_package_share_directory('drive')
    
    
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                bringup_pkg,'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    drive_control = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                            drive_pkg,'launch','drive_controllers.launch.py')])
    )
    
    return LaunchDescription([
        rsp,
        drive_control
    ])