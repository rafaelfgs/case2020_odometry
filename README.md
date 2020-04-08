# odom_ufmg_case

Repository with the necessary files to generate different types of odometry from bags recorded in UFMG with EspeleoRobô.

The recorded bags were recorded on February 2020. The output of this repository is the publication of six different localization techniques: wheel, wheel+imu, visual, lidar, uwb and uwb+imu.


## Correcting the bags

First, it is necessary to correct the bag's content by running the node *correct_node*, which is represented by the file *necessary_files/scripts/correct_bag_ufmg.py*. It is used to create another bag with the correct configurations for the topics used in odometry generation:

* */cmd_vel (geometry_msgs/Twist)*
* */decawave/tag_pose (espeleo_decawave/Tag)*
* */deviceX/get_joint_state (sensor_msgs/JointState)*
* */imu/data (sensor_msgs/Imu)*
* */laser_odom_to_init (nav_msgs/Odometry)*
* */integrated_to_init (nav_msgs/Odometry)*
* */os1_cloud_node/points (sensor_msgs/PointCloud2)*
* */robot_odom (nav_msgs/Odometry)*
* */realsense/gyro/sample (nav_msgs/Odometry)*
* */realsense/accel/sample (nav_msgs/Odometry)*
* */realsense/odom/sample (nav_msgs/Odometry)*
* */tf (tf2_msgs/TFMessage)*

Part of these topics were not recorded in some the bags and must be run offline, when possible. This node just republish many of these topics, except for:
*/decawave/tag_pose* - added an Odometry message
*/imu/data* - added a TF message
*/robot_odom* - fixed its covariance
*/realsense/gyro/sample* and */realsense/accel/sample* - merged into an Imu message
*/realsense/odom/sample* - used to create a Clock message
*/tf* - used to generate a Pose Message for estimation (UWB+IMU).

The main parameters of this node can be edited in the file, where:

* *input_file*: represents the complete name for the input bags.
* *output_file*: represents the name for the output bag.


## Odometry

The process for each odometry is presented in th following.


### Wheel Odometry

Up to three nodes are required to generate wheel odometry:

* *joints_node*: When the joints states were not recorded, it publishes */deviceX/get_joint_state* from */cmd_vel*.

* *odom_node*: When the wheel odometry was not computed online, it publishes the topic */robot_odom* from */deviceX/get_joint_state*.

* *output_wheel_node*: Changes */robot_odom* to desired frames, publishing it in */wheel/odom*.


### Wheel-EKF Odometry

Two nodes are required for wheel-EKF odometry:

* *robot_pose_ekf*: Uses Extended Kalman Filter for combining */robot_odom* and */imu/data* into the topic */robot_pose_ekf/odom_combined*.

* *output_wheel_ekf_node*: Changes */robot_pose_ekf/odom_combined* to desired frames, publishing it in */wheel_ekf/odom*.


### Visual Odometry

As visual odometry has always been computed online, only one node is required:

* *output_visual_node*: Changes */realsense/odom/sample* to desired frames, publishing it in */visual/odom*.


### LiDAR Odometry

For LiDAR odometry, one package and two nodes are required:

* *lego_loam*:  Using */os1_cloud_node/points* (and */imu/data*), this package publishes a simple LiDAR Odometry in */laser_odom_to_init* and a combined LiDAR+IMU Odometry in */integrated_to_init*

* *output_lidar_noimu_node*: Changes */laser_odom_to_init* to desired frames, publishing it in */lidar_noimu/odom*.

* *output_lidar_node*: Changes */integrated_to_init* to desired frames, publishing it in */lidar/odom*.


### UWB Localization Method

The only node required for localization method using UWB:

* *output_uwb_node*: Changes */decawave/odom* (created in *correct_node*) to desired frames, publishing it in */uwb/odom*.


### UWB-EKF Odometry

Two nodes are required for UWB-EKF odometry:

* *state_estimator_decawave*: Uses Extended Kalman Filter for combining linear velocity from */cmd_vel*, position from */decawave/pose* and gyroscope from */imu/data*, publishing it in */espeleo/pose*.

* *output_uwb_ekf_node*: Changes */espeleo/pose* to desired frames, publishing it in */uwb_ekf/odom*.


### Rviz View

Finally, for visualization of the whole process, the node *rviz_node* opens RViz with a correct configuration for topics and view.


## Building

The commands for built the package are simple:

```bash
git clone https://github.com/rafaelfgs/odom_ufmg_case.git
cd ..
catkin build
source devel/setup.bash
```

Note: *catkin build* can be replaced by *catkin_make*.


## Running

First, it is necessary to correct the bags, running the first node with the command:

```bash
rosrun required_files odom_ufmg_case.py
```

To simplify the command for the cited nodes, a *.launch* file can be used for running all of them (except for *odom_ufmg_case*), through the commands:

```bash
rosbag play PATH_TO_FILE.bag
roslaunch required_files odom_ufmg_case.launch 
```

### Parameters of *.launch* file

As many bags do not hold some topics, four boolean arguments were created to select the necessary nodes of the repository:

*joints*: true for bags that contain */deviceX/get_joint_state* (it is false only in corridor experiments)
*robot_odom*: true for bags that contain */robot_odom* (it is true only in hall experiments)
*loam*: true for bags that contain */laser_odom_to_init* and */integrated_to_init* (it is true only in hall experiments)
*estimation*: true for bags that contain */espeleo/pose* (it is true only in hall experiments)

Another important parameter is the transform from world to the robot initial pose. The vector with position and quaternion of each experiment are:

*[0.0 0.0 0.0 0.0 0.0 0.0174524 0.9998477]*: for the experiment in the closed path
*[1.0 1.08 0.0 0.0 0.0 0.0 1.0]*: for the experiment in the corridor
*[0.0 0.0 0.0 0.0 0.0 0.0 1.0]*: for the experiment in the hall

The sensors' transforms and the name of the sensors' topics are also presented as argument, and can be modified as desired.
