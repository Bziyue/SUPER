# super_planner

```bash
sudo apt-get install ros-noetic-mavros* ros-noetic-octomap* ros-noetic-plotjuggler* ros-noetic-joy 
sudo apt-get install libdw-dev
```


```bash
source /opt/ros/noetic/setup.bash
cd ${PATH-TO-WS}
catkin_make --pkg super_planner
source devel/setup.bash
```

Launch examples:

```bash
source /opt/ros/noetic/setup.bash
cd ${PATH-TO-WS}
source devel/setup.bash

roslaunch mission_planner benchmark_high_speed.launch
roslaunch mission_planner benchmark_dense.launch
roslaunch mission_planner click_demo.launch
roslaunch super_planner rviz.launch
```
