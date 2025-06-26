
#camera_launch.py
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
            executable='gesture_publisher', #'img_subscriber',
            name='gesture_publisher', #'image_subscriber',
            output='screen'
       ),
])


