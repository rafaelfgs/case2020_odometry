#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>



using namespace std;


//void Calc_trajectory(ros::Publisher *pub_trajectory, longitudinal_control::Trajectory *msg){

//}




//Main
int main(int argc, char **argv){
	ros::init(argc, argv, "teste");
	ros::NodeHandle nh;
	//ros::Publisher pub_trajectory = nh.advertise<longitudinal_control::Trajectory>("trajectory_point", 1);
	ros::Rate loop_rate(10);
	//Calc_trajectory(&pub_trajectory, &msg);	
	sleep(5);
	double t = 0.0;
	int taux = 0;
	float xf,x0,tf,a0,a1,a2,a3;
	double start_time = ros::Time::now().toSec();
	while (ros::ok()){
		nh.getParam("decawave/decawave_driver/rate", taux);
		cout<<taux<<endl;
		loop_rate.sleep();
	}
}
