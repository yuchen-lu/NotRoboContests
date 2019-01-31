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
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
int laserIndex;
double laserFront;


bool leftTurn, rightTurn;


int dist_to_left, dist_to_right;
//double slam_map [][];
//double raw_map [][];

// callback functions defined below
// each func receive a msg param with a specific variable type
// use global variable to share info back to main func (i.e. int main())

void bumperCallback(const kobuki_msgs::BumperEvent msg){

if (msg.bumper == 0)
	bumperLeft = !bumperLeft;
else if (msg.bumper == 1)
	bumperCenter = !bumperCenter;
else if (msg.bumper == 2)
	bumperRight = !bumperRight;
}


// if the shortest distance happened on the left, turn right; otherwise turn left
// result change leftTurn

/*
void TurnDirecion(int laserIndex, int laserSize,  int laserOffset){
	ROS_INFO("laserIndex = : %i laserSize = : %i", "lazerOffset = %i \n", laserIndex, laserSize, laserOffset);

	if (laserIndex - laserSize/2 - laserOffset != laserIndex-laserSize/2 + laserOffset){   
		if (laserIndex - laserSize/2 - laserOffset > laserIndex-laserSize/2 + laserOffset)
			leftTurn = !leftTurn;
		else
			rightTurn = !rightTurn;	
	}
	
}

*/

// LaserScan msg: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
// Inside the laserCallback function calculate the size of the msg->ranges array. Also calculate the desired field of
// view. This parameter will be used to reduce the number of indices in the array that we will consider when
// finding a distance measurement.

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);
	//ROS_INFO("Size of laser scan array: %i and size of offset: %i \n", laserSize, laserOffset);// Print the size of the array and the offset for testing purposes.

	// Below: distance measurement by searching the “ranges” array
	// done by finding the smallest distance in the desired field of view, in meters
	// The desired field of view is defined by a negative and positive laseroffset from the heading angle of the robot.
	//  If the desired field of view > sensor's, use sensor's instead
// array index 0~639  0 320 639
	laserRange = 11;
	if (desiredAngle*pi/180 <msg->angle_max && -desiredAngle*pi/180 >msg->angle_min){
		for (int i =laserSize/2 - laserOffset; i<laserSize/2 + laserOffset; i++){
			if (laserRange > msg->ranges[i]){
				laserRange = msg->ranges[i];
				laserIndex = i;
			}
		}
		
		
	}
	else
	{
		for ( int i =0; i < laserSize; i++){
			if (laserRange>msg->ranges[i]){
				laserRange = msg ->ranges[i];
				laserIndex = i;
			}
		}
		
	}
	if (laserRange == 11){ 
		laserRange = 0;
	}

	//TurnDirecion(laserIndex, laserSize, laserOffset);

	laserFront = msg->ranges[320]; // detects any object right in front of the turtlebot


}



//Odom Tutorial: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y; 
	yaw = tf::getYaw(msg->pose.pose.orientation);  // convert queration to euler angles

	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi); // print pos and orientation for testing
}



/*
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

	map_width = msg->info.width;
	map_height = msg->info.height;
	//convert map info to 2d array
	int row=0, col=0;
	while (row <= map_width>) {
	 	while (col <=map_height) 
		 slam_map[row][col] = raw_map[row + col * map_width];

	 }


}

*/



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

	double angular = 0;
	double linear = 0;

	geometry_msgs::Twist vel; //define a msg, type is Twist, name is vel

	std::chrono::time_point<std::chrono::system_clock> start;
	start  = std::chrono::system_clock::now(); //start timer
	uint64_t secondsElapsed = 0; //the timer just started, we know less than 480, no neded to change

	int walldetector = 0, i=0; // define variables for wall detection
	double laserOld = 0, laserNew = 0;
	double yawZero;
	double yawInitial = yaw;

	while(ros::ok() && secondsElapsed <=480 ){
	//	ROS_INFO("Position: (%f, %f) Orientation: %f laseRange: %f .", posX, posY, yaw*180/pi, laserRange); // print pos and orientation for testing

		ros::spinOnce();  // invoke all callback funcs and publish msgs
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code

/*
		if (posX < 0.5 && yaw < pi/12 &&!bumperRight && !bumperLeft && laserRange > 0.7)
		{
			angular = 0.0;
			linear = 0.2;
		}
		else if (posX > 0.4 && yaw < pi/2 &&!bumperRight && !bumperLeft && !bumperCenter && laserRange > 0.5)
		{
			angular = pi/6; 
			linear = 0.0;
		}

		else if (laserRange > 1.0 && !bumperRight && !bumperCenter && !bumperLeft)
		{
			if (yaw < 17*pi/36 || posX > 0.6)
			{
				angular = pi/12;
				linear = 0.1;
			}
			else if (yaw > 19*pi/36 || posX < 0.4)
			{
				angular = -pi/12;
				linear = 0.1;
			}
			else
			{
				angular = 0;
				linear = 0.1;
			}
		}
		else
		{
			angular = 0.0;
			linear =0.1;
		}*/

	// 	ROS_INFO("Orientation: %f laseRange: %f .\n",  yaw*180/pi, laserRange); // print pos and orientation for testing


		/*if (laserRange <= 1){
			dist_to_left = laserIndex - laserSize/2 + laserOffset;
			dist_to_right = laserSize/2 + laserOffset-laserIndex;
			ROS_INFO("laser index : %i \n" ,laserIndex);
			ROS_INFO("laser index dis to left: %i, laserindex dis to right %i \n", dist_to_left, dist_to_right);
			if (dist_to_left > dist_to_right) 
					leftTurn = 1;
			else if (dist_to_left <= dist_to_right)
					rightTurn = 1;		
		}
		// note: -ve angular speed -> cw

		if(laserRange > 1){
		//	ROS_INFO("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS \n");
			angular = 0;
			linear = 0.1;
		}
		if(rightTurn == 1){
		//	ROS_INFO("RRRRRRRRRRRRRRRRRRRRRRRRRRRR \n");
			angular= -pi/6;
			linear=0.0;
		}

		if(leftTurn == 1){
			angular= pi/6;
		//	ROS_INFO("LLLLLLLLLLLLLLLLLLLLLLLLLLLLL \n");

			linear = 0.0;
		}*/

		while (walldetector == 0) {

			
			while (i>= 0) { // loop if turtlebot is not // to wall
				//ROS_INFO ("entered loop");
				
				ros::spinOnce(); // invoke all callback funcs and publish msgs
				//std::cout<<laserRange;

				if (laserRange <= 1 && i == 0){
					dist_to_left = laserIndex - laserSize/2 + laserOffset;
					dist_to_right = laserSize/2 + laserOffset-laserIndex;
					//ROS_INFO("laser index : %i \n" ,laserIndex);
					//ROS_INFO("laser index dis to left: %i, laserindex dis to right %i \n", dist_to_left, dist_to_right);
					if (dist_to_left > dist_to_right) {
							leftTurn = 1;
							rightTurn = 0;
					}

					else if (dist_to_left <= dist_to_right) {
							rightTurn = 1;
							leftTurn = 0;
					}		
				}

				if (laserRange >1 && i==0) { // keep straight when no wall detected
					angular = 0;
					linear = 0.2;
				}

				else if (laserRange <= 1 && leftTurn == 1 && i==0){ // if wall on the left side of turtlebot, turn 90 deg to keep wall on the right
					
					//usleep(1000000);
					//ROS_INFO ("entered loop");
					
					while (abs(yawInitial-yaw) <= 0.4*pi) {
						angular = pi/6;
						linear = 0;

						vel.angular.z = angular; 
						vel.linear.x = linear;
						vel_pub.publish(vel);
						
						ros::spinOnce();
					}
					usleep(2000000);
					break;
				} 

				else{
					laserNew = laserFront; // store the laserFront distance in array
					//std::cout<<leftTurn;

					if (i <5){
						//ROS_INFO ("entered turn");
						angular = -pi/20;
						linear = 0;
						laserOld = laserNew;
						i ++;
					}
					else{
						if (laserNew <= laserOld){ // if laserfront distance increases, keep turning left
							//ROS_INFO ("entered turn");
							angular = -pi/10;
							linear = 0;
							laserOld = laserNew;
						}
						if (laserNew > laserOld) { // until laserfront reading start to decreases, turtlebot is now perpendicular to wall
							yawZero = yaw;
							walldetector = 1;
							i=-1;
							usleep(1000000);
							
							break;
						}
					}
				}

				vel.angular.z = angular; 
				vel.linear.x = linear;

				vel_pub.publish(vel);

			}

		}

		if (walldetector ==1) {

			//usleep(1000000);
			//ROS_INFO ("entered 90turn");
			
			angular = -pi/6;
			linear = 0;

			if (abs(yaw-yawZero)<= pi/2-pi/20) {
				//ROS_INFO (yaw);
				//ROS_INFO("\n");
				//std::cout<<yaw;
				//std::cout<<"\n";
				vel.angular.z = angular; 
				vel.linear.x = linear;
				vel_pub.publish(vel);
						
				ros::spinOnce();
				continue;
			}
			walldetector ++;
			
		}

		/*if (walldetector !=0 && walldetector !=1) {
			
			if (laserRange <= 1){
			dist_to_left = laserIndex - laserSize/2 + laserOffset;
			dist_to_right = laserSize/2 + laserOffset-laserIndex;
			ROS_INFO("laser index : %i \n" ,laserIndex);
			ROS_INFO("laser index dis to left: %i, laserindex dis to right %i \n", dist_to_left, dist_to_right);
			if (dist_to_left > dist_to_right) 
					leftTurn = 1;
			else if (dist_to_left <= dist_to_right)
					rightTurn = 1;		
			}
			// note: -ve angular speed -> cw

			if(laserRange > 1){
			//	ROS_INFO("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS \n");
				angular = 0;
				linear = 0.1;
			}
			if(rightTurn == 1){
			//	ROS_INFO("RRRRRRRRRRRRRRRRRRRRRRRRRRRR \n");
				angular= -pi/6;
				linear=0.0;
			}

			if(leftTurn == 1){
				angular= pi/6;
			//	ROS_INFO("LLLLLLLLLLLLLLLLLLLLLLLLLLLLL \n");

				linear = 0.0;
			}
		}

	
  		vel.angular.z = angular; 
  		vel.linear.x = linear;

  		vel_pub.publish(vel);   // vel_pub node publishes vel msg*/
		leftTurn = 0;
		rightTurn = 0;

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}

