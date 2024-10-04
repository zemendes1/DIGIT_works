# Instructions

### Launch RViz for visualization.
``` bash
cd
cd Desktop/RosBag
source /opt/ros/humble/local_setup.bash
ros2 run rviz2 rviz2 -d ROS_Workspace/default.rviz
```

### Now, we will run our quality of service analyzer package
``` bash
cd
cd Desktop/RosBag/ROS_Workspace
source /opt/ros/humble/local_setup.bash
colcon build
source install/setup.bash  
ros2 run data_quality tf_listener
```

### Next, we will play back a recorded ROS bag file to simulate real-time data.
``` bash
cd
cd Desktop/RosBag
source /opt/ros/humble/local_setup.bash
ros2 bag play bag_011024
```



