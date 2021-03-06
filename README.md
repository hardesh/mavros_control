# mavros_control

PX4 Firmware (https://github.com/PX4/Firmware) provides one of the most accurate quadrotor simulation platforms.

For the installation of PX4 firmware follow instructions provided by [GAAS](https://github.com/generalized-intelligence/GAAS/blob/master/Setup.md).


Getting started guides are available on: https://gaas.gitbook.io/guide/


#### Installation instructions for ROS melodic:
```bash
cd catkin_ws/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.9.0

cd ..
git clone --recursive https://github.com/PX4/sitl_gazebo.git
cd ..
catkin build
```
Install the python dependencied mentioned on: 
1. https://github.com/PX4/sitl_gazebo
2. https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html 


--------------- 

To use this repo:
```bash
cd catkin_ws/src
git clone https://github.com/hardesh/mavros_control.git 
cd ..
catkin build
source devel/setup.bash
```

After this you should be able to use the package.

[dronecontroller.py](https://github.com/hardesh/mavros_control/blob/master/scripts/dronecontroller.py) is a generalised class for quadrotors. You can use this class to control your drone using MAVROS.

Example:
```bash
# Using the takeoff script
roslaunch px4 mavros_posix_sitl.launch
rosrun mavros_control takeoff.py
```
