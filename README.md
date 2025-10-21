# How to run the aquatttic SITL sim 
Step 1
Go to PX4 path in terminal 1 and then paste this
```
make px4_sitl gz_r1_rover
```
Step 2
Open terminal 2 and paste this (no specific path required)
```
MicroXRCEAgent udp4 -p 8888
```
Step 3
Open terminal 3 and paste this
```
ros2 launch control rover_bringup.launch.py
```
