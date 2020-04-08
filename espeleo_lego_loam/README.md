# Espeleo LeGO-LOAM
<p style='text-align: justify;'> 
This repository contains code to a LiDAR SLAM, called espeleo_lego_loam, which is a modification of the LeGO-LOAM ROS package (https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.) adapted for the EspeleoRobô. The system takes in point cloud from an Ouster OS1-16 Lidar and optional IMU data as inputs. It outputs a 6D pose estimation in real-time. 

Below you can see the SLAM result for an experiment carried out at Veloso Mine. 
 </p>
  
<p align='center'>
<img src="/ReadMe/mapa-videos-espeleo.gif" alt="center" width="700"/>
</p>

<p style='text-align: justify;'> 
Some images of the experiments carried out with espeleo_lego_loam.
</p>

Veloso Mine:

<p align='center'>
    <img src="/ReadMe/mapa_vista_interna.png" alt="center" width="250"/>
    <img src="/ReadMe/image_top_vision.png" alt="center" width="250"/>
</p>

Laboratory experiments at UFMG: 

<p align='center'>
(Auditorium hall, Engineering building corridors, Short corridor)
</p>

<p align='center'>
    <img src="/ReadMe/auditorio_perspectiva_novo.png" alt="center" width="200"/>
    <img src="/ReadMe/volta_corredor_mapa_novo.jpg" alt="center" width="256"/>
    <img src="/ReadMe/corredor_vista_superior.png" alt="center" width="95.5"/>
</p>

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with indigo and kinetic)
- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)

```
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.0-alpha2/
  mkdir build && cd build
  cmake ..
  sudo make install
```

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/ITVRoC/espeleo_lego_loam.git
cd ..
catkin_make -j1
```
<p style='text-align: justify;'> 
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.
</p>

## The system

<p style='text-align: justify;'> 
This approach uses point cloud segmentation to filter out the noise, adjusts the distortion using IMU and extracts planar and edge features. The proposal uses a two-step optimization method to determine the components of a homogeneous transformation H ∈ SE(3) between two consecutive scans.

The figure below shows the overview of the \texttt{LeGO-LOAM} algorithm, subdivided into five modules: Segmentation, Feature Extraction, LiDAR Odometry, LiDAR Mapping, and Integration Transform.
</p>

<p align='center'>
    <img src="/ReadMe/scheme_lidar.png" alt="drawing" width="400" />
</p>

<p style='text-align: justify;'> 
Segmentation projects the point cloud into the image range for segmentation in clusters. Feature Extraction obtains planar and edge points of the point cloud adjusted by the IMU. In LiDAR Odometry, a Levenberg-Marquardt optimization is applied into two consecutive scans to determine linear and angular displacements of the robot. LiDAR Mapping records points on a point cloud map, and another Levenberg-Marquardt optimization computes the device pose that is saved in a graph. Integration Transform merges the poses returned by the LiDAR-odometry with the LiDAR-mapping module, and the algorithm outputs the final pose estimation.
</p>

## Parameters

<p style='text-align: justify;'> 
This package has some configurable parameters, which can be found in the folder:
</p>

```
/EspeleoLeGOLOAM/Config/map_parameters.yaml
```
<p style='text-align: justify;'> 
The parameters of this package are:
</p>

- Input Topic:

> - `pointCloud2In`: Point cloud topic;
> - `imuTopicIn`: Imu topic.


- Frames:

> - `init`: Name of the initial frame of LiDAR odometry;
> - `odom`: Name of the odom frame of LiDAR odometry;
> - `imu_odom`: Name of the odom frame with imu of LiDAR odometry;
> - `map`: Name of the map frame;
> - `base`: Name of the base link.

- Folder to save PCD point cloud:

> - `fileDirectoryName`: Folder path to save the map point clouds.

- Loop Closure settings:

>  - `enableLoopClosure`: Enable or disable de loop closure;
>  - `keyframeRadiuns`: 2n + 1 number of history keyframes will be fused into a submap for loop closure;
>  - `keyframeNum`: Number of history keyframes will be fused into a submap for loop closure;
> - `keyframeScore`: The smaller the better alignment.
 
- Mapping settings:

> - `segmentThetaValue`: Decrese this value may improve accuracy;
> - `nearestFeatureSearchSqDistValue`: Text;
> - `surroundingKeyframeSearchRadiusValue`: Key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled);
> - `surroundingKeyframeSearchNumValue`: Submap size (when loop closure enabled);
> - `leafsizemapValue`: Leaf size for KDtree map;
> - `globalMapVisualizationSearchRadiusValue`: Key frames with in n meters will be visualized.

To set the LiDAR parameters go to the folder in:

```
/EspeleoLeGOLOAM/include/utility_espeleo.h
```

The package was set for the Ouster LiDAR  OS1-16, as shown below: 

```
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 512; 
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 33.2/float(N_SCAN-1);
extern const float ang_bottom = 16.6+0.1;
extern const int groundScanInd = 7;
```

After changing the LiDAR parameters, you need to compile the workspace.

## Run the package

To run online experiments:

1. Run the package of LiDAR:
```
rroslaunch ouster_ros os1.launch
```
2. Run the launch file:
```
roslaunch espeleo_lego_loam espeleo_lego_loam.launch
```

To run offline in bag files:

1. Run the launch file:
```
roslaunch espeleo_lego_loam espeleo_lego_loam.launch
```

2. Play existing bag files:
```
rosbag play *.bag --clock --topic /laser_cloud_surround /imu/data
```
## Report a Bug

To report a bug, send email to gilmarpcjunior@yahoo.com.br.

