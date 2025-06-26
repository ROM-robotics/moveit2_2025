from setuptools import find_packages, setup

package_name = 'ros2_opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ros2_opencv/launch',['launch/camera_launch.py']),
        ('share/ros2_opencv/launch',['launch/camera_test.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hnin21',
    maintainer_email='hnin21@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
      'img_publisher = ros2_opencv.webcam_pub:main',
      'img_subscriber = ros2_opencv.webcam_sub:main',
      'gesture_publisher = ros2_opencv.gesture_publisher:main',
      'gesture_control = ros2_opencv.gesture_control:main',
      
    ],
},
)
