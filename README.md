# moveit2_2025
```
ros2 launch rom_ur_sim spawn_ur5_camera_gripper_moveit_table_yolo.launch.py 
# or
tmuxinator

```

#### Create python env for yolo
```
create env
pip install numpy==1.24.4
pip install PyQt5
```

### .bashrc
```
source /home/mr_robot/devel_ws/install/setup.bash
source /home/mr_robot/moveit_ws/install/setup.bash
source /home/mr_robot/ros2_ws/install/setup.bash
#source /home/mr_robot/test_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export ROS_DOMAIN_ID=69

alias bb='colcon build && source install/setup.bash'
alias delete_workspace='rm -rf build install log; echo "Done!"'
alias bb_save='colcon build --executor sequential --parallel-workers 4'
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/mr_robot/devel_ws/src/moveit2_2025/ur/
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/mr_robot/devel_ws/src/moveit2_2025/ur/

```
