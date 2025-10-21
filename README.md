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
To launch the "rover_bringup.launch.py" file you need a new terminal
If you launch the terminal from an already existing one (e.g. terminal 2), then you first need to go to go to home directory, source the bash file, and launch the python file. This is done as follows
```
carlos@carlos-Lenovo-Slim-Pro-7-14ARP8:~/PX4-Autopilot$ cd
carlos@carlos-Lenovo-Slim-Pro-7-14ARP8:~$ source .bashrc
carlos@carlos-Lenovo-Slim-Pro-7-14ARP8:~$ ros2 run control position_controller.py 
```
