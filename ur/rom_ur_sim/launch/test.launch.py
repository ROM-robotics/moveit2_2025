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
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the share directory for your rom_ur_sim package
    # This assumes 'rom_ur_sim' is correctly installed in your ROS 2 workspace.
    rom_ur_sim_pkg_dir = get_package_share_directory('rom_ur_sim')
    
    # Define the path to your robot's URDF/xacro file
    # Ensure this path is correct relative to your 'rom_ur_sim' package.
    xacro_file_path = os.path.join(
        rom_ur_sim_pkg_dir,
        'urdf',
        'ur5_camera_gripper.urdf.xacro'
    )

    # Define the path to your controller configuration file
    # This YAML file should define the 'joint_state_broadcaster', 'joint_trajectory_controller',
    # and 'gripper_position_controller'.
    controllers_yaml_path = os.path.join(
        rom_ur_sim_pkg_dir,
        'config',
        'ur5_controllers_gripper.yaml'
    )

    # Use xacro to process the URDF file
    # This reads the xacro file and converts it into a standard URDF string that
    # can be used by ROS 2 nodes.
    robot_description_content = xacro.process_file(xacro_file_path).toxml()
    
    # Set up MoveIt 2 configuration using MoveItConfigsBuilder
    # This utility simplifies the process of loading MoveIt 2 parameters.
    # Ensure 'rom_ur5_moveit_config' package is set up for your robot and contains
    # the necessary SRDF, kinematics, and controller files.
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="rom_ur5_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro") # Your robot's URDF/xacro
        .robot_description_semantic(file_path="config/ur.srdf") # Your robot's SRDF
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # MoveIt's controllers
        .robot_description_kinematics(file_path="config/kinematics.yaml") # Kinematics solvers
        .planning_scene_monitor(
            publish_robot_description= True, 
            publish_robot_description_semantic=True, 
            publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"] # Planning algorithms
        )
        .to_moveit_configs()
    )
    
    # Robot State Publisher Node
    # This node reads the robot description and publishes the robot's state (joint positions, links) to TF.
    # It's essential for visualization and for other ROS components that need to know the robot's structure.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content, # Pass the processed URDF content
        }]
    )

    # Include Gazebo Ignition launch file for Jazzy/Harmonic
    # 'gz_sim.launch.py' is provided by the 'ros_gz_sim' package and starts the Gazebo simulation.
    # '-r empty.sdf' starts Gazebo with a basic empty world. You can replace 'empty.sdf'
    # with your custom world file if needed.
    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf', # Arguments passed directly to the Gazebo simulator executable
            'on_exit_shutdown': 'true', # Ensures Gazebo process shuts down when this launch file exits
            'use_sim_time': LaunchConfiguration('use_sim_time'), # Use simulation time for Gazebo
        }.items()
    )

    # Gazebo Spawn Entity Node
    # This node uses 'ros_gz_sim' to spawn your robot model into the running Gazebo simulation.
    # It subscribes to the '/robot_description' topic, which 'robot_state_publisher' publishes to.
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'ur5_camera', # Name your robot will have in Gazebo
            '-topic', '/robot_description', # Topic from which to get the robot's URDF
            '-x', '0.0', # Initial X position in Gazebo
            '-y', '0.0', # Initial Y position in Gazebo
            '-z', '0.0', # Initial Z position in Gazebo
        ]
    )

    # RViz2 Node
    # RViz2 is the primary visualization tool in ROS 2. This node launches RViz2 with a pre-defined
    # configuration for MoveIt 2, allowing you to visualize the robot, planning scene, and motion plans.
    rviz_config_path = os.path.join(
        get_package_share_directory("rom_ur5_moveit_config"),
        "config",
        "moveit.rviz", # Path to your RViz configuration file
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # Pass use_sim_time to RViz2 for proper time synchronization
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ROS2 Control Node (controller_manager)
    # This node acts as the central manager for all hardware interfaces and controllers.
    # It reads the 'robot_description' to understand the robot's joints and interfaces,
    # and the 'controllers_yaml_path' to load controller configurations.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description_content}, # Robot URDF content
            controllers_yaml_path # Path to your robot's controller configurations
        ],
        output="screen",
    )

    # Joint State Broadcaster Spawner
    # This spawner loads and starts the 'joint_state_broadcaster' controller.
    # This controller publishes the current state of all robot joints to the '/joint_states' topic,
    # which is consumed by 'robot_state_publisher' and RViz2.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Joint Trajectory Controller Spawner
    # This spawner loads and starts the 'joint_trajectory_controller'.
    # This controller is typically used for controlling the robot's arm joints by taking
    # joint trajectory commands (e.g., from MoveIt 2).
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Gripper Position Controller Spawner
    # This spawner loads and starts a dedicated controller for the gripper.
    # The name 'gripper_position_controller' should match a controller defined in your
    # 'ur5_controllers_gripper.yaml' file.
    gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Gazebo-ROS 2 Bridge for Clock
    # This node bridges the Gazebo simulation clock to the ROS 2 clock.
    # This helps ensure consistent time across the simulation and ROS 2 nodes.
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # Gazebo-ROS 2 Bridge for Camera Image
    # This node bridges the camera's raw image topic from Gazebo to ROS 2.
    # The topic names here depend on how your camera is defined in the URDF/Gazebo model.
    # Adjust '/camera/image_raw' if your camera publishes to a different topic.
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'], 
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}] # Ensure camera bridge uses sim time
    )

    # MoveIt 2 MoveGroup Node
    # The 'move_group' node is the central component of MoveIt 2. It integrates
    # planning, execution, and monitoring of robot movements.
    # It uses the MoveItConfigsBuilder output for its parameters.
    use_sim_time_param = {"use_sim_time": LaunchConfiguration('use_sim_time')}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time_param) # Ensure use_sim_time is passed to move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Register event handler to run spawners after ros2_control_node has started
    # This ensures that the controller manager is fully initialized and ready
    # before attempting to load and start controllers.
    delay_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node, # Wait for the robot to be spawned in Gazebo
            on_exit=[
                ros2_control_node, # Then start the controller manager
                joint_state_broadcaster_spawner, # Then spawn individual controllers
                joint_trajectory_controller_spawner,
                gripper_position_controller_spawner,
            ],
        )
    )

    # Return the LaunchDescription with all defined nodes and event handlers
    return LaunchDescription([
        declare_use_sim_time_arg, # Declare the use_sim_time argument first
        gazebo_launch_include, # Start Gazebo simulation
        robot_state_publisher_node, # Start robot state publisher
        spawn_entity_node, # Spawn the robot into Gazebo
        rviz_node, # Launch RViz2
        move_group_node, # Launch MoveIt's move_group node
        gz_sim_bridge, # Bridge Gazebo clock
        camera_bridge, # Bridge camera images
        delay_after_spawn_entity, # This will launch controllers after the robot is spawned
    ])
