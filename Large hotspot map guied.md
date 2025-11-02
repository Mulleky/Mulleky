# How to run the Large Hotspot SITL sim

The large hotspot sim consists of a gazebo + RVIZ2 + ROS2 SITL simulation that generates a large temperature field 
(75x75) divided as a 3x3 grid. Five of the nine available regions will be randomly assigned a hotspot field 
from the following: 
- Radial field 
- Compressed in the x-direction
- Compressed in the y-direction
- Compressed in the x-direction and tilted
- Compressed in the y-direction and tilted
The remaining 4 regions will have a base temperature + noise

## How to run the SITL sim 
Do the first three steps from the README.md file as described there:
```
make px4_sitl gz_r1_rover
```
```
MicroXRCEAgent udp4 -p 8888
```
```
ros2 launch control rover_bringup.launch.py 
```



