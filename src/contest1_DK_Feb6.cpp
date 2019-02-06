#include <ros/console.h>
#include <ros/ros.h>
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

// global variables declarations
double angular = 0.0, linear = 0.0;
//odom variables
double posX=0, posY=0, yaw=0;
double pi=3.1416;
//bumper variables
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

//LaserScan variables
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 20;
int laserIndex;

bool leftTurn, rightTurn;
int dist_to_left, dist_to_right;

double left_most_dist, right_most_dist;
double mid_left_dist, mid_right_dist;
bool step1 = 1;// if step 1 is 1, run step 1; if step 1 is 0, run step 2
//sint dist_to_left, dist_to_right;
//double slam_map [][];
//double raw_map [][];
int bigturn_counter = 0;
// callback functions defined below
// each func receive a msg param with a specific variable type
// use global variable to share info back to main func (i.e. int main())



void left_or_right(double laserRange, double left_most_dist, double right_most_dist, double mid_left_dist, double mid_right_dist, bool step1) {
// decide left or right turn
	// step 1
	if (step1 == 1){
	
		if (laserRange <= 1){
			if (mid_left_dist >= mid_right_dist && (mid_left_dist - mid_right_dist >0.1)) {
				leftTurn = 1;
			}
			if (mid_left_dist < mid_right_dist && (mid_left_dist - mid_right_dist >0.1)) {
				rightTurn = 1;
			}
		}
	}

	// step 2
	else {
		// TBD !!!!

		if (laserRange <= 1){	
			if (left_most_dist >= right_most_dist && (left_most_dist - right_most_dist >0.1)) {
				leftTurn = 1;
			}
			if (left_most_dist < right_most_dist && (right_most_dist - left_most_dist >0.1)) {
				rightTurn = 1;
			}


		
		}

	}


}

void turn_or_straight (double laserRange, bool step1){

//ROS_INFO("inside turn_or_straight !!\n");

	// if step 1, go around 
	if (step1 == 1){
		//ROS_INFO("inside step1 turn or straight !!\n");
		if (laserRange <=0.7) {
		//ROS_INFO("TOO CLOSE !!\n");

		angular = pi/3;
		linear = 0.0;
		bigturn_counter++; 

		}

		if(laserRange > 1){
		//	ROS_INFO("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS \n");
			angular = 0;
			linear = 0.1;
		}

		if(rightTurn == 1 && laserRange > 0.7){
		//	ROS_INFO("RRRRRRRRRRRRRRRRRRRRRRRRRRRR \n");
			angular= -pi/6;
			linear=0.0;
		}


		if(leftTurn == 1 && laserRange > 0.7){
			angular= pi/6;
		//	ROS_INFO("LLLLLLLLLLLLLLLLLLLLLLLLLLLLL \n");

			linear = 0.0;
		}

	//	ROS_INFO("INside: angular is : %f, linear is : %f", angular, linear);
	}


	// if step 2, decide what to response when see leftTurn = 1 or RightTurn = 1
	if (step1 == 0){
		
		// TBD !!!!!
	}

}





void bumperCallback(const kobuki_msgs::BumperEvent msg){

if (msg.bumper == 0)
	bumperLeft = !bumperLeft;
else if (msg.bumper == 1)
	bumperCenter = !bumperCenter;
else if (msg.bumper == 2)
	bumperRight = !bumperRight;
}


// LaserScan msg: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
// Inside the laserCallback function calculate the size of the msg->ranges array. Also calculate the desired field of
// view. This parameter will be used to reduce the number of indices in the array that we will consider when
// finding a distance measurement.

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);
	ROS_INFO("Size of laser scan array: %i and size of offset: %i \n", laserSize, laserOffset);// Print the size of the array and the offset for testing purposes.
	laserRange = 11;
	// Below: distance measurement by searching the “ranges” array
	// done by finding the smallest distance in the desired field of view, in meters
	// The desired field of view is defined by a negative and positive laseroffset from the heading angle of the robot.
	//  If the desired field of view > sensor's, use sensor's instead
// array index 0~639  0 320 639
	
/*
	left_most_dist = msg->ranges[laserSize/2 - laserOffset+2]; // index only works for desiredAngle = 10  **********
	right_most_dist = msg->ranges[laserSize/2 + laserOffset-2];

*/






// Jan 31 added: 
//ranges index fron turtlebot view: [600][599][][][499]][][][][1][0]

										//^         ^
										//|			|
										//step2		step1
										//left_most	 mid_left

/* trial 1: with average
	left_most_dist = (msg->ranges[laserSize/2 + laserOffset-2]+msg->ranges[laserSize/2 + laserOffset-3]+msg->ranges[laserSize/2 + laserOffset-4])/3;
	right_most_dist = (msg->ranges[laserSize/2 - laserOffset+2]+msg->ranges[laserSize/2 - laserOffset+3]+msg->ranges[laserSize/2 - laserOffset+4])/3;

*/

// trial 2: without avgerage

	left_most_dist = msg->ranges[laserSize/2 + laserOffset-2]; // step 2
	right_most_dist = msg->ranges[laserSize/2 - laserOffset+2]; // step 2

	mid_left_dist = msg->ranges[laserSize/2 + laserOffset-2 - 200]; // step 1, needs tuning
 	mid_right_dist = msg->ranges[laserSize/2 - laserOffset+2 + 200]; // step 1, needs tuning
	

// if step 2, use left_most and right_most to set laserRange
	if (step1 == 0){
		if (left_most_dist >= right_most_dist){
			laserRange = right_most_dist;
		}
		else 
			laserRange = left_most_dist;
}

	// if step 1, use mid_left and mid_right to set laserRange
	if (step1 == 1){

		if (mid_left_dist >= mid_right_dist){
			laserRange = mid_right_dist;
		}
		else 
			laserRange = mid_left_dist;
	}

	if (laserRange == 11){ 
		laserRange = 0;
	}

}







//Odom Tutorial: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y; 
	yaw = tf::getYaw(msg->pose.pose.orientation);  // convert queration to euler angles

	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi); // print pos and orientation for testing
}



int main(int argc, char **argv)
{
	std::cout<<"enter main cout"<<std::endl;
	ROS_INFO ("entered main");
	ros::init(argc, argv, "image_listener");   //init a node called image_listener
	ros::NodeHandle nh; //
	teleController eStop;


	//create a subscriber named map_sub, subscrite to topic named map within gmapping, 
//	ros::Subscriber map_sub = nh.subscribe("map", 10, &mapCallback);


	//create a subscriber named bumper_sub; subscribe to topic named bumper; size of msg quene; ROS will call  bumperCallback() function whenever a new message arrives 
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	//image_transport::ImageTransport it(nh); //new

	//create a subscriber named bumper_sub; subscribe to topic named bumper; size of msg quene; ROS will call  bumperCallback() function whenever a new message arrives 
	// note: odom topic defined in turtlebot
	ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback); 

 	//create a publisher named vel_pub; msg type is Twist; will send msg thru topic teleop;  size of the message queue is 1
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); 

//	double angular = 0;
//	double linear = 0;

	geometry_msgs::Twist vel; //define a msg, type is Twist, name is vel

	std::chrono::time_point<std::chrono::system_clock> start;
	start  = std::chrono::system_clock::now(); //start timer
	uint64_t secondsElapsed = 0; //the timer just started, we know less than 480, no neded to change



	while(ros::ok() && secondsElapsed <=480 ){
	//	ROS_INFO("Position: (%f, %f) Orientation: %f laseRange: %f .", posX, posY, yaw*180/pi, laserRange); // print pos and orientation for testing

		ros::spinOnce();  // invoke all callback funcs and publish msgs
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();


		// decide when to enter step2
		if (bigturn_counter >2000){  // needs tuning
			step1 = 0;
		}

		// set Leftturn ad rightturn to 1 
		left_or_right(laserRange, left_most_dist, right_most_dist,mid_left_dist, mid_right_dist, step1);

		// response to different Leftturn and rightturn 
		turn_or_straight (laserRange, step1);

	// note: -ve angular speed -> cw


		ROS_INFO("mid_left_dist: %f, mid_right_dist %f \n", mid_left_dist, mid_right_dist);
		//ROS_INFO("angular is : %f, linear is : %f", angular, linear);
  		vel.angular.z = angular; 
  		vel.linear.x = linear;

  		vel_pub.publish(vel);   // vel_pub node publishes vel msg
		leftTurn = 0;
		rightTurn = 0;

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}