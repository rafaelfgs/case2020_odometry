// kalman filter
// ROS stuffs
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>

#include <espeleo_decawave/Tag.h>

//Eigen stuff
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
// manage files
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/stat.h> // commands
#include <stdio.h>

//Our libs
#include "EKF_espeleo.h"

#define PI 3.1415926535


#define ESPELEO_ERROR_ODOM 0.7 //1.0


/*
Universidade Federal de Minas Gerais (UFMG) - 2020
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
*/


using namespace std;

// Variáveis globais
Eigen::VectorXd u_m(10,1);
bool new_u_m;
Eigen::VectorXd y_gps_position(2,1);
Eigen::VectorXd y_odom_vel(1,1);
bool new_y_gps_position;
bool new_y_odom_vel;
//bool new_gps_orientation
Eigen::VectorXd y_imu_orientation(4,1);
Eigen::VectorXd y_gps_velocity(3,1);
bool new_y_gps_velocity;
int cont_gps = 0;
int cont_vel_x = 0;
Eigen::VectorXd  y_scan(2,1);
Eigen::VectorXd  y_scan_global(2,1);
bool new_y_scan;
Eigen::VectorXd  y_alt(1,1);
bool new_y_alt;


bool filter_init = false;


// callbacks for IMU data
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){

  u_m << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
         1*msg->angular_velocity.x, 1*msg->angular_velocity.y, 1*msg->angular_velocity.z,
         msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;

  new_u_m = true;

}







int count_gps, count_imu;
double cx_gps, cy_gps;

double last_decawave_msg[3] = {0.0, 0.0, 0.0};
double decawave_qf_threshold = 0.7;

void decawave_callback(const espeleo_decawave::Tag::ConstPtr& msg){

  if (msg->x==last_decawave_msg[0] && msg->y==last_decawave_msg[1] && msg->z==last_decawave_msg[2]){
    printf("\33[93mWarning: Repeated decawave message %d!\33[m\n",msg->header.seq);
  }
  else if(msg->qf<decawave_qf_threshold){
    printf("\33[93mWarning: Ignoring low qf decawave message  %d!\33[m\n",msg->header.seq);
  }
  else{
    filter_init = true;
    y_gps_position << msg->x, msg->y;
    new_y_gps_position = true;
    cont_gps++;
    last_decawave_msg[0] = msg->x; last_decawave_msg[1] = msg->y; last_decawave_msg[2] = msg->z;
  }




  //
  // if(msg->qf < decawave_qf_threshold){
  //   printf("\33[93mWarning: Ignoring low qf decawave message  %d!\33[m\n",msg->header.seq);
  // }
  // else if (msg->x==last_decawave_msg[0] && msg->y==last_decawave_msg[1] && msg->z==last_decawave_msg[2]){
  //   printf("\33[93mWarning: Repeated decawave message %d!\33[m\n",msg->header.seq);
  // }
  // else{
  //   last_decawave_msg[0] = msg->x; last_decawave_msg[1] = msg->y; last_decawave_msg[2] = msg->z;
  //   filter_init = true;
  //   new_y_gps_position = true;
  //   y_gps_position << msg->x, msg->y;
  //   cont_gps++;
  // }



}









void cmdvel_callback(const geometry_msgs::Twist::ConstPtr& msg){

  y_odom_vel << msg->linear.x*ESPELEO_ERROR_ODOM;
  new_y_odom_vel = true;
  cont_vel_x++;

}





// Rotation Matrix body to inertial
Eigen::MatrixXd R_bw(const Eigen::VectorXd &u){

  // matriz de rotacao
  double e_w, e_x, e_y, e_z;
  e_w = u[0]; e_x = u[1]; e_y = u[2]; e_z = u[3];

  // Create a rotation matrix from body- to inertial-frame
  Eigen::MatrixXd R_bw(3,3); // Rotation matrix body-frame to navigation-frame
  R_bw << pow(e_x,2)+pow(e_w,2)-pow(e_y,2)-pow(e_z,2), -2*e_z*e_w+2*e_y*e_x, 2*e_y*e_w +2*e_z*e_x,
  2*e_x*e_y+ 2*e_w*e_z, pow(e_y,2)+pow(e_w,2)-pow(e_x,2)-pow(e_z,2), 2*e_z*e_y-2*e_x*e_w,
  2*e_x*e_z-2*e_w*e_y, 2*e_y*e_z+2*e_w*e_x, pow(e_z,2)+pow(e_w,2)-pow(e_x,2)-pow(e_y,2);

  return R_bw;
}


#define N_STATES 4

Eigen::VectorXd EKF_states_0(N_STATES);
Eigen::MatrixXd EKF_H(3,N_STATES);
Eigen::MatrixXd EKF_Q(N_STATES,N_STATES);
Eigen::MatrixXd EKF_Q_bar(6,6);
Eigen::MatrixXd EKF_R(3,3);
Eigen::MatrixXd EKF_P(N_STATES,N_STATES);



void load_EKF_parameters(ros::NodeHandle nh){

  std::vector<double> temp_vector;

  try{

    nh.getParam("/state_estimator_decawave/qf_threshold", decawave_qf_threshold);

    nh.getParam("/state_estimator_decawave/states_0", temp_vector);
    EKF_states_0 = VectorXd::Map(temp_vector.data(), temp_vector.size());

    nh.getParam("/state_estimator_decawave/H", temp_vector);
    EKF_H = Eigen::Map<Eigen::Matrix<double, N_STATES, 3> >(temp_vector.data()).transpose();

    nh.getParam("/state_estimator_decawave/Q", temp_vector);
    EKF_Q = Eigen::Map<Eigen::Matrix<double, N_STATES, N_STATES> >(temp_vector.data()).transpose();

    nh.getParam("/state_estimator_decawave/Q_bar", temp_vector);
    EKF_Q_bar = Eigen::Map<Eigen::Matrix<double, 6, 6> >(temp_vector.data()).transpose();

    nh.getParam("/state_estimator_decawave/R", temp_vector);
    EKF_R = Eigen::Map<Eigen::Matrix<double, 3, 3> >(temp_vector.data()).transpose();

    nh.getParam("/state_estimator_decawave/P", temp_vector);
    EKF_P = Eigen::Map<Eigen::Matrix<double, N_STATES, N_STATES> >(temp_vector.data()).transpose();


    printf("\33[92m\nThe following EKF parameters were loaded:\33[0m\n\n");
    cout << "decawave_qf_threshold:\n" << decawave_qf_threshold << endl << endl;
    cout << "EKF_states_0:\n" << EKF_states_0 << endl << endl;
    cout << "EKF_Q:\n" << EKF_Q << endl << endl;
    cout << "EKF_Q_bar:\n" << EKF_Q_bar << endl << endl;
    cout << "EKF_R:\n" << EKF_R << endl << endl;
    cout << "EKF_P:\n" << EKF_P << endl << endl;


  }catch(...){

    printf("\33[41mError when trying to read EKF parameters\33[0m");

  }



}





int main(int argc, char *argv[])
{

  //Initialize node
  ros::init(argc, argv, "state_estimator_decawave");

  // Get handle
  ros::NodeHandle nh_;
  // Handle for getting parameters
  ros::NodeHandle nh_2("state_estimator_decawave");

  //Define frequency
  ros::Rate loop_rate(25.0*1.05);

  // Callbacks
  ros::Subscriber sub_imu = nh_.subscribe("/imu/data", 1, &imu_callback);
  ros::Subscriber sub_gps = nh_.subscribe("/decawave/tag_pose", 1, &decawave_callback);
  ros::Subscriber sub_cmdvel = nh_.subscribe("/cmd_vel", 1, &cmdvel_callback);

  //Publishers
  ros::Publisher states_pub = nh_.advertise<std_msgs::Float32MultiArray>("/states_filter", 50);
  ros::Publisher pose_pub = nh_.advertise<geometry_msgs::Pose>("/espeleo/pose", 1);
  ros::Publisher rviz_pose_pub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker_espeleo_EKF", 1);

  //Used messages
  geometry_msgs::Pose espeleo_pose;
  visualization_msgs::Marker espeleo_marker;

  //Initialize some global variables
  count_gps = 0;
  count_imu = 0;
  cont_vel_x = 0;
  cx_gps = 0; cy_gps = 0;


  //Initialize more variables
  new_u_m = false; //imu
  new_y_gps_position = false;
  new_y_odom_vel = false;
  new_y_gps_velocity = false;
  new_y_scan = false;
  new_y_alt = false;
  y_odom_vel << 0.0;

  //Declare more variables
  double dt = 0.0;
  double t_current = ros::Time::now().toSec();
  double t_previous = ros::Time::now().toSec();
  double t_init = ros::Time::now().toSec();
  double time_log = 0.0;
  int gps_step = 1; // vary the GPS frequency
  int vel_x_step = 1; // vary the


  //Read parameters of the EKF filter
  load_EKF_parameters(nh_2);

  // EKF filter object
  VectorXd initial_pos(2);
  initial_pos << 0.0,0.0;  // x, y, z
  double frequency = 25.0;
  double init_angle = 0.0;
  EKF_Filter *Filter;
  Filter = new EKF_Filter(EKF_states_0, frequency, EKF_H, EKF_Q, EKF_Q_bar, EKF_R, EKF_P);
  Eigen::VectorXd states(4);
  //Local states variable
  states << 0.0, 0.0, 0.0, 0.0;










// c_pos = Eigen::Map<Eigen::Matrix<double, 8, 3> >(c_pos_aux.data()).transpose();

  // ================================ Create log =================================
  std::string log_path;
  std::string log_path_1;
  std::string log_path_2;
  std::string log_path_3;
  std::string log_path_4;
  if (nh_2.getParam ("/state_estimator_decawave/log_path", log_path)){
    cout << "\33[92mSetting log path from parameter ...\33[0m" << endl;
    log_path_1 = log_path+"state_log.txt";
    log_path_2 = log_path+"gps_position.txt";
    log_path_3 = log_path+"imu_log.txt";
  }
  else{
    cout << "\33[93mWarning: Path for log files not found, using defalut:\33[0m" << endl;
    cout << "/home/adrianomcr/ROS_projects/vale_ws/src/espeleo_localization/log_files/" << endl;
    cout << "\33[41mIt may cause an error!\33[0m" << endl;
    log_path_1 = "/home/adrianomcr/ROS_projects/vale_ws/src/espeleo_localization/log_files/state_log.txt";
    log_path_2 = "/home/adrianomcr/ROS_projects/vale_ws/src/espeleo_localization/log_files/gps_position.txt";
    log_path_3 = "/home/adrianomcr/ROS_projects/vale_ws/src/espeleo_localization/log_files/imu_log.txt";
  }


  FILE *state_log;
  FILE *gps_position_log;
  FILE *imu_log;

  try{
    state_log = fopen(log_path_1.c_str(),"w");
    gps_position_log = fopen(log_path_2.c_str(),"w");
    imu_log = fopen(log_path_3.c_str(),"w");

  } catch(...){
    cout << "\33[41mError when oppening the log files\33[0m" << endl;
  }




  //Main loop
  while (ros::ok()){


    // =====================================
    // Perform a prediction step with the IMU information
    // =====================================
    if(new_u_m == true){
      new_u_m = false;

      //calcula dt
      t_current = ros::Time::now().toSec();
      dt = t_current - t_previous;

      t_previous = t_current;

      if (filter_init==true){

        //Update the IMU data in the filter object
        double Ts = dt;
        Filter->callback_imu(u_m.block(3,0,3,1), u_m.block(0,0,3,1), Ts); //(gyro,accel)

        //Perform a prediction step in the filter
        Filter->prediction();

        // Get the current states of the filter
        states = Filter->get_states();


        //Save data to log files
        time_log = ros::Time::now().toSec() - t_init;
        fprintf(imu_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",u_m(0),u_m(1),u_m(2),u_m(3),u_m(4),u_m(5),u_m(6),u_m(7),u_m(8),u_m(9),time_log);
        fflush(imu_log);

        fprintf(state_log,"%f\t%f\t%f\t%f\t%f\n",states(0),states(1),states(2),states(3),time_log);
        fflush(state_log);

      }
    }




    // =====================================
    // Update filter with GPS position information
    // =====================================
    if(new_y_gps_position == true){
      new_y_gps_position = false;

      if (cont_gps == gps_step){

        cont_gps = 0;
        if (filter_init==true){

          //Call the update
          Filter->callback_position(y_gps_position);

          //Save data to log files
          time_log = ros::Time::now().toSec() - t_init;
          fprintf(gps_position_log,"%f\t%f\t%f\n",y_gps_position(0),y_gps_position(1),time_log);

        }
      }
    }





    // =====================================================================
    // Update filter with forward velocity information comming from odometry
    // =====================================================================
    if(new_y_odom_vel == true){
      new_y_odom_vel = false;

      if (cont_vel_x == vel_x_step){
        cont_vel_x = 0;
        if (filter_init==true){
          //Call the update
          Filter->callback_vel_x(y_odom_vel);
        }
      }
    }








    if(filter_init){
      // ----------  ----------  ---------- ----------  ----------
      //Publish the pose estimation of the robot

      // Uptade the rost variable to be publishe
      espeleo_pose.position.x = states(0);
      espeleo_pose.position.y = states(1);
      espeleo_pose.position.z = 0.0;
      espeleo_pose.orientation.x = 0.0;
      espeleo_pose.orientation.y = 0.0;
      espeleo_pose.orientation.z = sin(states(2)/2.0);
      espeleo_pose.orientation.w = cos(states(2)/2.0);

      // Publish the Pose message
      pose_pub.publish(espeleo_pose);


      // ----------  ----------  ---------- ----------  ----------
      //Publish a marker with the pose of the estimation
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
      espeleo_marker.color.b = 0.9;
      espeleo_marker.pose.position.x = states(0);
      espeleo_marker.pose.position.y = states(1);
      espeleo_marker.pose.position.z = 0.0;
      espeleo_marker.pose.orientation.x = 0.0;
      espeleo_marker.pose.orientation.y = 0.0;
      espeleo_marker.pose.orientation.z = sin(states(2)/2.0);
      espeleo_marker.pose.orientation.w = cos(states(2)/2.0);

      rviz_pose_pub.publish(espeleo_marker);
      // ----------  ----------  ---------- ----------  ----------

      //Publish a transform between the world frame and the filter estimation
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(states(0), states(1), 0.0) );
      tf::Quaternion q;
      q.setRPY(0, 0, states(2));
      transform.setRotation(q);
      //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "estimation"));
    }






    //Check callbacks and wait
    ros::spinOnce();
    loop_rate.sleep();


    //Publish states in a vector
    std_msgs::Float32MultiArray vector_states;
    for (int k = 0; k<4; k++){
      vector_states.data.push_back(states(k));
    }
    states_pub.publish(vector_states);



  }//end while

  //Close log files
  fclose(state_log);
  fclose(gps_position_log);
  fclose(imu_log);
}
