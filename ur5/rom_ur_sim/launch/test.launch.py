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
    rom_ur_sim_pkg_dir = get_package_share_directory('rom_ur_sim')
    
    #add moveit config
    joint_controllers_file = os.path.join(
        get_package_share_directory('rom_ur_sim'), 'config', 'ur5_controllers.yaml'
    )
    #add moveit config
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="rom_ur5_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Extract the parameters from moveit_config.to_dict() and flatten them if necessary
    # The 'planning_pipelines' parameter expects a list of strings directly
    moveit_cpp_parameters = moveit_config.to_dict()
    
    # Ensure 'planning_pipelines' is directly available at the top level
    # MoveItConfigsBuilder already handles this correctly for the most part,
    # but explicitly extracting it and making it a direct parameter ensures it.
    planning_pipelines_param = moveit_cpp_parameters.pop("planning_pipelines")
    
    moveit_cpp_node = Node(
        name="moveit_cpp",
        package="rom_ur_sim",
        executable="arm_control_from_ui",
        output="both",
        parameters=[
            moveit_cpp_parameters, # All other MoveIt configurations
            {"use_sim_time": True},
            {"planning_pipelines": planning_pipelines_param}, # Explicitly pass planning_pipelines
        ],
    )

    # Return the LaunchDescription with all defined nodes
    return LaunchDescription([
        declare_use_sim_time_arg,
        moveit_cpp_node,
    ])