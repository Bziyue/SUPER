# super_planner

```bash
sudo apt-get install ros-humble-pcl-* ros-humble-tf2-ros ros-humble-geometry-msgs ros-humble-nav-msgs
```


```bash
source /opt/ros/humble/setup.bash
cd ${PATH-TO-WS}
colcon build --packages-up-to super_planner
source install/local_setup.bash
```

Launch examples:

```bash
source /opt/ros/humble/setup.bash
cd ${PATH-TO-WS}
source install/local_setup.bash

ros2 launch mission_planner banchmark_high_speed.launch.py
ros2 launch mission_planner benchmark_dense.launch.py
ros2 launch mission_planner click_demo.launch.py
```
