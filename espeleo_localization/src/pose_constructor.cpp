#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "gps_utm.cpp"
#include <visualization_msgs/Marker.h>


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <string>

#include <ros/package.h>




double pos[3], quat[4], yaw;

int count_gps, count_imu;
double cx_gps, cy_gps, center_yaw;


void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps)
{

     // Transform to UTM reference system
     double northing, easting;
     char zone;
     LLtoUTM(gps->latitude, gps->longitude,  northing, easting , &zone);


     if (count_gps < 120){
       count_gps++;
       cx_gps += easting/120.0;
       cy_gps += northing/120.0;
       printf("count:%d\n", count_gps);
     }
     else{
       double alpha = 0.3;                                   //Beguinning of the trajectory (Dijkstra)
       pos[0] = (1-alpha)*pos[0] + alpha*(easting-cx_gps       -7.823163*0); //(-8.9399 * 0)
       pos[1] = (1-alpha)*pos[1] + alpha*(northing-cy_gps      +2.010794*0); //(+3.0343 * 0)
       pos[2] = 0.0;
     }


}


void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{

    quat[0] = imu->orientation.x;
    quat[1] = imu->orientation.y;
    quat[2] = imu->orientation.z;
    quat[3] = imu->orientation.w;

    double r, p, y;
    tf::Matrix3x3 rotMatrix(tf::Quaternion(quat[0],quat[1],quat[2],quat[3]));

    // Get roll, pitch and yaw
    rotMatrix.getRPY(r, p, y);
    yaw = y;


    //I think that it is not necessary. But it can be done to ensure that yaw=0 means robot pointing East
    if (count_imu < 120){
      count_imu++;
      center_yaw += yaw/120.0;
    }
    else{
      yaw = y - center_yaw;
    }


}



// Main
int main(int argc, char **argv) {

  ros::init(argc, argv, "pose_constructor");
  ros::NodeHandle nh;

  double freq = 20.0;


  ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, gps_callback);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, imu_callback);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/espeleo/pose_gps_imu", 1);
  ros::Publisher rviz_pose_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker_espeleo_estimation", 1);
  ros::Rate loop_rate(freq);


  int i = 0;
  count_gps = 0;
  count_imu = 0;
  cx_gps = 0; cy_gps = 0; center_yaw = 0;

  geometry_msgs::Pose espeleo_pose;

  visualization_msgs::Marker espeleo_marker;


  int state = 0; // 0 - take off; 1 - follow vector field


  string resultsFile;
  nh.param<std::string>( "/espeleo/experimentResultsFile", resultsFile, "/home/adrianomcr/results.txt" );
  printf("\33[43mCORRIGIR O LUGAR DESSE ARQUIVO 'results.txt'.\33[0m");

  FILE *f;
  f = fopen(resultsFile.c_str(), "w");
  if (f==NULL){
    //ROS_ERROR("Can not open file 'results.txt'.");
    printf("\33[41mCan not open file 'results.txt'.\33[0m");
    return -1;
  }


  while (ros::ok())
  {

    // Read the callbacks
    ros::spinOnce();


    //cout << i << endl;
    i++;


    fprintf(f,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",pos[0],pos[1],pos[2],quat[0],quat[1],quat[2],quat[3],yaw);


    // Uptade the rost variable to be publishe
    espeleo_pose.position.x = pos[0];
    espeleo_pose.position.y = pos[1];
    espeleo_pose.position.z = pos[2];
    espeleo_pose.orientation.x = quat[0];
    espeleo_pose.orientation.y = quat[1];
    espeleo_pose.orientation.z = quat[2];
    espeleo_pose.orientation.w = quat[3];

    // Publish rateThrust command
    pose_pub.publish(espeleo_pose);



    espeleo_marker.header.frame_id = "/world";
    espeleo_marker.header.stamp = ros::Time::now();
    espeleo_marker.id = 0;
    espeleo_marker.type = espeleo_marker.CUBE;
    espeleo_marker.action = espeleo_marker.ADD;
    espeleo_marker.scale.x = 0.50;
    espeleo_marker.scale.y = 0.30;
    espeleo_marker.scale.z = 0.12;
    espeleo_marker.color.a = 0.3;
    espeleo_marker.color.r = 0.9;
    espeleo_marker.color.g = 0.9;
    espeleo_marker.color.b = 0.0;
    espeleo_marker.pose.position.x = pos[0];
    espeleo_marker.pose.position.y = pos[1];
    espeleo_marker.pose.position.z = pos[2];
    //quaternio = [esp_q[0], esp_q[1], esp_q[2], esp_q[3]]
    espeleo_marker.pose.orientation.x = quat[0];
    espeleo_marker.pose.orientation.y = quat[1];
    espeleo_marker.pose.orientation.z = quat[2];
    espeleo_marker.pose.orientation.w = quat[3];

    rviz_pose_pub.publish(espeleo_marker);



    // Sleep program
    loop_rate.sleep();
  }



  //Terminate if ros is not ok
  if(!ros::ok()){
    // Close results file
    fclose(f);
  }



}
