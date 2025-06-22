import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Declare arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the share directory for your rom_ur_sim package
    rom_ur_sim_pkg_dir = get_package_share_directory('rom_ur_sim')
    
    # Define the path to your robot's URDF/xacro file
    xacro_file_path = os.path.join(
        rom_ur_sim_pkg_dir,
        'urdf',
        'ur5_camera.urdf.xacro'
    )

    # Define the path to your controller configuration file
    controllers_yaml_path = os.path.join(
        rom_ur_sim_pkg_dir,
        'config',
        'ur5_controllers.yaml'
    )

    # Define the path to your custom Gazebo world file
    world_file_path = os.path.join(
        rom_ur_sim_pkg_dir,
        'worlds',
        'arm_on_the_table.sdf'
    )

    # Use xacro to process the URDF file
    # This reads the xacro file and converts it into a standard URDF string
    robot_description_content = xacro.process_file(xacro_file_path).toxml()

    # Robot State Publisher Node
    # This node reads the robot description and publishes the robot's state to TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content
        }]
    )

    # Include Gazebo Ignition launch file
    # This launches the Gazebo simulation environment with your custom world
    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': world_file_path, # Now uses your custom world file
            'on_exit_shutdown': 'true', # Ensures Gazebo shuts down when this launch file exits
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Gazebo Spawn Entity Node
    # This node spawns your robot into the Gazebo simulation
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'ur5_camera', # Name of your robot in Gazebo (changed to reflect UR5)
            '-topic', '/robot_description', # Topic where robot_state_publisher publishes the URDF
            '-x', '0.0', # Initial X position
            '-y', '0.0', # Initial Y position
            '-z', '0.0', # Initial Z position
        ]
    )

    # ROS2 Control Node (controller_manager)
    # This node manages and loads the controllers for your robot.
    # It requires the robot description and the controllers YAML file.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description_content},
            controllers_yaml_path
        ],
        output="screen",
    )

    # Joint State Broadcaster Spawner
    # This spawner loads and starts the 'joint_state_broadcaster' which publishes
    # the current state of the robot's joints.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Joint Trajectory Controller Spawner
    # This spawner loads and starts the 'joint_trajectory_controller'.
    # This controller typically takes joint commands and moves the robot to desired positions.
    # Make sure this controller is defined in your ur5_controllers.yaml
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Register event handler to run spawners after ros2_control_node has started
    # This ensures the controller manager is ready before attempting to spawn controllers.
    delay_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                ros2_control_node,
                joint_state_broadcaster_spawner,
                joint_trajectory_controller_spawner,
            ],
        )
    )


    # Return the LaunchDescription with all defined nodes
    return LaunchDescription([
        declare_use_sim_time_arg,
        gazebo_launch_include, # Now uses IncludeLaunchDescription
        robot_state_publisher_node,
        spawn_entity_node,
        delay_after_spawn_entity, # This will launch controllers after the robot is spawned
    ])

