# espeleo_decawave
Positioning System based on Decawave's DWM1001 Ultra Wide Band transceivers. This is a ROS packaged adapted from <a href="https://github.com/verlab/ros_decawave">Verlab-ros_Decawave</a>

## Installation

### Dependencies

#### Python
```
$ pip install serial
$ pip install struct
```

#### ROS 

```
$ sudo apt install ros-kinetic-hector-trajectory-server
```

### Building Package

Install this package on your catking workspace:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ITVRoC/espeleo_decawave.git
$ cd ..
$ catkin_make ## or catkin build
```

## Tutorial

### Initial Setup

Install the Android application located at <a href="https://github.com/verlab/ros_decawave/tree/master/docs/DWM1001_DWM1001-DEV_MDEK1001_Sources_and_Docs_v8/Android%20Application">docs/DWM1001_DWM1001-DEV_MDEK1001_Sources_and_Docs_v8/Android Application/</a> in some device.

Setup the tag and the anchors using the Android application, setting the anchor position in relation with the world frame.
ps.: It's necessary to power on the anchors and tags for visualization and setup on the app.


![App Description](https://github.com/ITVRoC/espeleo_decawave/blob/master/media/app_decawave2.jpg)

Connected the tag using an USB cable to PC.

### Package Setup

After setup the anchors positions, it is necessary configure some parameters for better execution of the ROS package.
In the configuration file <a href="https://github.com/ITVRoC/espeleo_decawave/blob/master/config/parameters.yaml">config/parameters.yaml</a>, it is possible setup some information about the "tag" connected on the PC.

Parameter   |  Description
------------- | -------------
port      | Serial port in which the tag was connected. Ex.: /dev/ttyS0
tag_name  | Id reference of the tag. Ex.: DW802F
rate      | Publication frequency of the position topic in Hz
    
    
 ### Launch
 
 After all configuration, run the launch:
 ```
 $ roslaunch ros_decawave decawave_driver.launch
 ```
 
## Extra Information
 
### Topics
```
$ rostopic list
/decawave/tag_pose
/decawave/tag_status
/tf 
```

![Rviz Visualization](https://github.com/ITVRoC/espeleo_decawave/blob/master/media/ros_decawave_rviz.png)



