import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction


import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('orion_simulation'))
    xacro_file = os.path.join(pkg_path, 'description/urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    controller_params_file = os.path.join(get_package_share_directory("orion_simulation"),'config','diff_drive_controller.yaml')
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    bridge_params = os.path.join(get_package_share_directory("orion_simulation"),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )


    # Create a robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml(),
                     'use_sim_time': use_sim_time}]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [f'-r {world_file}'],
                              'on_exit_shutdown': 'true',
                              'params_file': controller_params_file}.items()
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
        arguments=['-topic', 'robot_description','-name', 'my_bot','-z', '0.1'],
        output='screen')

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        robot_state_publisher,
        joint_state_publisher,
        gazebo_launch,
        rviz_launch,
        spawn_robot,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
    ])