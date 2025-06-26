#camera_test
----------------------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
             package='ros2_opencv',
             executable='img_publisher', #from setup.py
             name='image_publisher',
             output='screen'
       ),

        Node(
            package='ros2_opencv',
            executable='img_subscriber',
            name='image_subscriber',
            output='screen'
       ),
])
