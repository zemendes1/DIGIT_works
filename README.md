# Instructions for macOS

### Launch RViz for visualization.
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/
mamba activate ros_env
ros2 run rviz2 rviz2 -d ROS_Workspace/default.rviz
```

### To run the quality of service analyzer package between all markers and camera
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/ROS_Workspace/
mamba activate ros_env
colcon build
source install/local_setup.sh 
ros2 run data_quality tf_listener.launch.py bag_name:=bag_name
```

### To run the quality of service analyzer package between marker_111, marker_222 and marker_0
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/ROS_Workspace/
mamba activate ros_env
colcon build
source install/local_setup.sh 
ros2 launch data_quality tf_listener_marker_0.launch.py bag_name:=bag_name
```

### Next, we will play back a recorded ROS bag file to simulate real-time data.
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/
mamba activate ros_env
ros2 bag play bag_name
```

### To check the frames and their transformations
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/
mamba activate ros_env
ros2 run tf2_tools view_frames -o frames
```

### Also, example Checking the transformation between two frames
``` bash
ros2 run tf2_ros tf2_echo marker_111 marker_0
```