import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='orion_simulation'
    pkg_path = os.path.join(get_package_share_directory('orion_simulation'))
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join
            (get_package_share_directory('ros_gz_sim'), 'launch', 'ign_gazebo.launch.py')),
            launch_arguments={'gz_args': world_file}.items()  
            )
    
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'orion'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo_launch,
        rviz_launch,
        spawn_robot,
    ])
