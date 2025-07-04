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

#### camera position
```
    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="0.2 0.6 1.2" rpy="0 1.570796326 0" />
    </joint>
```
