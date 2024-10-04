# Instructions

### Launch RViz for visualization.
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/
mamba activate ros_env
ros2 run rviz2 rviz2 -d ROS_Workspace/default.rviz
```

### Now, we will run our quality of service analyzer package
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/ROS_Workspace/
mamba activate ros_env
colcon build
source install/local_setup.sh 
ros2 run data_quality tf_listener
```

### Next, we will play back a recorded ROS bag file to simulate real-time data.
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/
mamba activate ros_env
ros2 bag play bag_011024
```



