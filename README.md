# MB_1r2t Driver
ROS1 driver for MB_1r2t
![alt text](doc/mb_1r2t.jpg)

## How to install
I have currently only tested the driver with Ubuntu 20.04 using ROS1 Noetic

1. Install ros and create a workspace: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

2. Clone and build package:

``` 
$ cd ~/catkin_ws/src
$ git clone https://github.com/Vidicon/mb_1r2t_ros.git
$ cd ~/catkin_ws/
$ catkin_make
```

## How to run the driver
1. Connect the lidar using a TTL serial to usb adapter

2. find connected port:
```
$ ls /dev/tty*
```
you would expect to see `/dev/ttyUSB0` or `/dev/ttyACM0` in the list

3. Launch the launch file:
```
$ roslaunch mb_1r2t_ros view.launch port:=/dev/ttyUSB0
```
You should see rviz with the lidar data:

![alt text](doc/rviz_view.png)



