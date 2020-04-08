# EspeleoRobo localization
----------------------
This repository must contain the files to perform the localization of the robot.




## Nodes

- `pose_constructor.cpp` This node is a simple pose constructor, it captures position of GPS and orientation from IMU and merges in a single topic. OBS: This conde is outdated. Use `state_estimator_espeleo.cpp` instead.

- `state_estimator_espeleo.cpp` This nodes performs a EKF filter for the pose estimation of the espeleorobo. It uses GPS position information, gyroscopic data from IMU, and commanded forward speed. The EKF filter is implemented in a class in the files `EKF_espeleo.cpp` and `EKF_espeleo.h`. The filter considers the initial position of the robot as the zero position. The estimated yaw angle is with respect to the East direction.





## How to interact

The following topics compose the message flow of this package:

**Topics:**
- `/fix`  (message type:`sensor_msgs/NavSatFix`): Topic where the filter gets position information.

- `/imu/raw`  (message type:`sensor_msgs/Imu`): Topic where the filter gets angular velocity information.

- `/cmd_vel`  (message type:`geometry_msgs/Twist`): Topic where the filter gets forward linear velocity information.

- `/espeleo/pose`  (message type:`geometry_msgs/Pose`): Topic in which the estimated pose of the robot is published.

- `/states_filter`  (message type:`std_msgs/Float32MultiArray`): Topic That contains a vector with the estate estimates of the EKF filter.

- `/visualization_marker_espeleo_EKF`  (message type:`visualization_msgs/Marker`): Topic with the pose estimated by the filter used by rviz.



**Services:**
- ``





## TO DO

- Insert the parameters of the EKF filter in a yaml file. They are currently in the cpp code.

- Generalize the position initialization of the filter to consider global UTM localization.

- Incorporate a good outlier rejection filter
