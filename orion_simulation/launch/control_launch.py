import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_path = get_package_share_directory('orion_simulation')
    xacro_file = os.path.join(pkg_path, 'description/urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    controller_params_file = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')

    # Process xacro to get URDF XML string
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml, 'use_sim_time': use_sim_time}]
    )

    # joint_state_publisher node (optional, only if needed)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    # Launch Gazebo (ros_gz_sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}', 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot in gazebo from robot_description topic
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.1'],
        output='screen'
    )

    # Controller manager node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_xml},
            controller_params_file
        ],
        output='screen'
    )

    # Load and activate diff_drive_controller after delay (wait for controller_manager to be ready)
    load_diff_drive_controller = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
                output='screen'
            )
        ]
    )

    # Optionally RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        gazebo_launch,
        spawn_robot_node,
        controller_manager_node,
        load_diff_drive_controller,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
