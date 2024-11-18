# Instructions for macOS

### Launch RViz for visualization.
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/
mamba activate ros_env
ros2 run rviz2 rviz2 -d ROS_Workspace/default.rviz
```

### To run the tf service analyzer package
``` bash
cd
cd Documents/GitKraken/DIGIT_works/RosBag/ROS_Workspace/
mamba activate ros_env
colcon build
source install/local_setup.sh 
ros2 launch data_quality tf_listener.launch.py
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

### Watch the camera feed
``` bash
mamba activate ros_env
ros2 run rqt_image_view rqt_image_view --ros-args --remap image:=/anafi/camera/image
```