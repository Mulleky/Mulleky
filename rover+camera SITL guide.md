# Guide on how to run the rover + RGB camera SITL sim

1.- Run the "make" command in one terminal
```
make px4_sitl gz_r1_rover_camera
```

2.- Open a new terminal and run the gz-ros bridge so that ros can see the camera topics
```
ros2 run ros_gz_bridge parameter_bridge \
/world/rover/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image \
/world/rover/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

3.- Open a new terminal and run the launch file (from the ros2_ws path)
```
ros2 launch control rover_camera.bringup.launch.py
```

## Things to correct
* When RVIZ2 is launched the link for the camera joint don't work
