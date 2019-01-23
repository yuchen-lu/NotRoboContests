#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>



#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <chrono>



using namespace std;

double angular, linear;
//odom variables
double posX, posY, yaw;
double pi=3.1416;



void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//fill with your code
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y; 
	yaw = tf::getYaw(msg->pose.pose.orientation);  // convert queration to euler angles

	ROS_INFO(“Position: (%f, %f) Orientation: %f rad or %f degrees.”, posX, posY, yaw, yaw*180/pi); // print pos and orientation for testing
}










int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");   //init a node called image_listener
	ros::NodeHandle nh; //
	teleController eStop;


	//create a subscriber named bumper_sub; subscribe to topic named bumper; size of msg quene; ROS will call  bumperCallback() function whenever a new message arrives 
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	image_transport::ImageTransport it(nh); //new


	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);


 	//create a publisher named vel_pub; msg type is Twist; will send msg thru topic teleop;  size of the message queue is 1
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); 

	angular = 0.0;
	linear = 0.0;

	geometry_msgs::Twist vel; //define a msg, type is Twist, name is vel

	std::chrono::time_point<std::chrono::system_clock> start;
	start  = std::chrono::system_clock::now(); //start timer
	uint64_t secondsElapsed = 0; //the timer just started, we know less than 480, no neded to change




	while(ros::ok() && secondsElapsed <=489){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code

  		vel.angular.z = angular; 
  		vel.linear.x = linear;

  		vel_pub.publish(vel);   // vel_pub node publishes vel msg


		secondsElapsed = std::chrono::duration_cast<srd::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}
