import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    share_dir = get_package_share_directory('robot_pkg')
    parent_dir = os.path.dirname(share_dir)
    world_dir = get_package_share_directory('gz_worlds')

    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''), ':', parent_dir]
    )

    urdf_path = os.path.join(share_dir, 'urdf', 'robot_description.urdf')
    robot_urdf = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    default_world_path = os.path.join(world_dir, 'worlds', 'marsyard2022.sdf')
    bridge_config = os.path.join(share_dir, 'config', 'bridge.yaml')
    pointcloud_to_laserscan_config = os.path.join(share_dir, 'config', 'pointcloud_to_laserscan.yaml')
    ekf_config_path = os.path.join(share_dir, 'config', 'dual_ekf.yaml')
    
    
    spawn_delay = LaunchConfiguration('spawn_delay')
    world_file = LaunchConfiguration('world_file')

    declare_spawn_delay = DeclareLaunchArgument(
        'spawn_delay',
        default_value='5.0',
        description='Seconds to wait after Gazebo starts before spawning the robot'
    )

    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=default_world_path,
        description='Full path to the SDF world file to load in Gazebo'
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}, {'use_sim_time': True}],
        output='screen'
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "my_robot", "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "1.85"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    spawn_after_gazebo_ready = TimerAction(
        period=spawn_delay,
        actions=[spawn_robot]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_steering_controller"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_velocity_controller"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox'), 
            ('scan', '/scan')       
        ],
        parameters=[pointcloud_to_laserscan_config, {'use_sim_time': True}]
    )

    controller_node = Node(
        package='robot_pkg',
        executable='controller.py',
        name='controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    odom_pub_node = Node(
        package='robot_pkg',
        executable='odom.py',
        name='controller_odom',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ekf_node_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    
    # Step A: When Spawn exits -> Start Bridge and Joint State Broadcaster
    start_bridge_and_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot, 
            on_exit=[bridge_node, jsb_spawner]
        )
    )

    # Step B: When Joint State Broadcaster exits -> Start Steering Controller
    start_steering_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[steering_spawner]
        )
    )

    # Step C: When Steering Controller exits -> Start Velocity Controller
    start_velocity_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=steering_spawner,
            on_exit=[velocity_spawner]
        )
    )

    # Step D: When Velocity Controller exits -> Start all custom/autonomy nodes
    start_custom_nodes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velocity_spawner,
            on_exit=[pointcloud_to_laserscan_node, controller_node, odom_pub_node, ekf_node_odom]
        )
    )

    return LaunchDescription([
        set_gz_path,
        declare_spawn_delay,
        declare_world_file,
        robot_state_publisher_node,
        gz_sim_launch,
        spawn_after_gazebo_ready,
        start_bridge_and_jsb,
        start_steering_controller,
        start_velocity_controller,
        start_custom_nodes,
    ])