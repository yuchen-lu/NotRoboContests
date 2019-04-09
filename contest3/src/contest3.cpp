#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;

using namespace std;

//follower variables
geometry_msgs::Twist follow_cmd;
int world_state;

//Image variables
bool imageMatched = false;
bool oldImgMatch = false;

//bumper variables
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

//cliff variables
bool cliffLeft = 0, cliffCenter = 0, cliffRight = 0;

// global variables declarations
double angular = 0.0, linear = 0.0;

//odom variables
double posX=0, posY=0, yaw=0;
double pi=3.1416;

//follower callback
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

//bumper callback
void bumperCB(const kobuki_msgs::BumperEvent msg){
    if (msg.bumper == 0)
		bumperLeft = !bumperLeft;
	else if (msg.bumper == 1)
		bumperCenter = !bumperCenter;
	else if (msg.bumper == 2)
		bumperRight = !bumperRight;
}

//Image callback
void imageCB(const std_msgs::Int8& msg){
	oldImgMatch = imageMatched;
	if(msg.data == 1){
		imageMatched = true;
	}
	else{
		imageMatched = false;
	}
}

//cliff callback
void cliffCB(const kobuki_msgs::CliffEvent msg){
	if (msg.sensor == 0){
		if (msg.state==1) cliffLeft = 1;
		else if (msg.state==0) cliffLeft = 0;
	}
	else if (msg.sensor == 1){
		if (msg.state==1) cliffCenter = 1;
		else if (msg.state==0) cliffCenter = 0;
	}
	else if (msg.sensor == 2){
		if (msg.state==1) cliffRight = 1;
		else if (msg.state==0) cliffRight = 0;
	}
}

void backup() { // backup if hit bumper
	ros::NodeHandle nh;
	//ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback); 
	
 	//create a publisher named vel_pub; msg type is Twist; will send msg thru topic teleop;  size of the message queue is 1
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); 

	//geometry_msgs::Twist vel;
	geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

	ros::Time start = ros::Time::now();
	int bump_counter =0;
	ros::Duration diff = ros::Time::now() - start;
	double diff_sec= diff.toSec();

	while(diff_sec < 2){ // backup for 1 sec

		cout<<"backup\n";
		
		ros::spinOnce();
		angular = 0;
		linear = -0.1;

		cmd->angular.z = angular; 
		cmd->linear.x = linear;
		vel_pub.publish(cmd);

		diff = ros::Time::now() - start;
		diff_sec= diff.toSec();
	}

	bumperCenter = 0;
	bumperLeft = 0;
	bumperRight = 0;

	angular = 0;
	linear = 0;
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber cliff = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);
	ros::Subscriber imageMatch = nh.subscribe("img_contest3/imagetrans",1,&imageCB);

	//imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	ros::Time start;
	ros::Duration diff;
	double diff_sec;

	int lost_counter= 0;

	

	Mat sad = imread("/home/turtlebot/catkin_ws/src/NotRoboContests/contest3/sounds/sad.jpg");
	Mat anger = imread("/home/turtlebot/catkin_ws/src/NotRoboContests/contest3/sounds/Anger.png");
	Mat fear = imread("/home/turtlebot/catkin_ws/src/NotRoboContests/contest3/sounds/Fear.png");
	Mat infatuated = imread("/home/turtlebot/catkin_ws/src/NotRoboContests/contest3/sounds/tom_in_love.png");


	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................


		//----------------change world state according to emotional sensor feedback------------the order follows the priority of a real human
		
		if(bumperCenter || bumperLeft || bumperRight){  
			world_state=2; // hit object, case 2
		}
		else if(cliffCenter && cliffLeft && cliffRight){
			world_state=3;  //lifed up, case 3
		}
		else if(imageMatched == true && imageMatched != oldImgMatch){
			world_state=4;  //see the picture of another turtlebot, case 4
		}
		else if(follow_cmd.angular.z == 0 &&  follow_cmd.linear.x == 0){
			lost_counter ++;
			if (lost_counter == 1) {
				start = ros::Time::now();
			} else {
				diff = ros::Time::now() - start;
				diff_sec = diff.toSec();

				cout<< diff_sec <<"\n";

				if (diff_sec > 3) {
					world_state = 1;
					lost_counter = 0;
				}
			}

			 //lose track of object for 3 sec, case 1
		}


		//------------------------Take actions according to world state--------------------------------
		if(world_state == 0){	 // object detected, continue following
			vel_pub.publish(follow_cmd);
			if(follow_cmd.angular.z != 0 &&  follow_cmd.linear.x != 0) {
				lost_counter = 0; //follower founr clear lost state counter
			}
		}
		else if(world_state == 1){   //object lost, play confused sound
		//	follow_cmd.angular.x=0;
			vel_pub.publish(follow_cmd);  // stop the robotjpg

			namedWindow("sad", WINDOW_NORMAL);
			setWindowProperty("sad", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
			waitKey(50);
			imshow("sad", sad);
			waitKey(50);

			sc.playWave(path_to_sounds + "sad.wav");
			cout<<"object lost\n";
			ros::Duration(10.0).sleep();
			destroyAllWindows();
			waitKey(50);	
		
		}
		else if(world_state == 2){    //hit object, play sad (cry) sound, back up 0.5m

			lost_counter = 0;

			namedWindow("anger", WINDOW_NORMAL);
			setWindowProperty("anger", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
			waitKey(50);
			imshow("anger", anger);
			waitKey(50);

			sc.playWave(path_to_sounds + "angry.wav");
			cout<<"hit\n";
			
			backup();
			ros::Duration(2.0).sleep();
			destroyAllWindows();
			waitKey(50);	
		}
		else if(world_state==3){   //lifed up, play scared sound

			lost_counter = 0;

			namedWindow("fear", WINDOW_NORMAL);
			setWindowProperty("fear", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
			waitKey(50);
			imshow("fear", fear);
			waitKey(50);
			cout<<"lift\n";	

			sc.playWave(path_to_sounds + "fear.wav");
			ros::Duration(3.0).sleep();
			sc.playWave(path_to_sounds + "fear.wav");
			ros::Duration(3.0).sleep();
			
			destroyAllWindows();
			waitKey(50);
		}
		else if(world_state==4){   //see the picture of another turtlebot, play infatuated sound

			lost_counter = 0;

			namedWindow("infatuated", WINDOW_NORMAL);
			setWindowProperty("infatuated", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
			waitKey(50);
			imshow("infatuated", infatuated);
			waitKey(50);

			cout<<"saw picture!\n";
			sc.playWave(path_to_sounds + "wow.wav");
			ros::Duration(3.0).sleep();
			sc.playWave(path_to_sounds + "wistle.wav");
			ros::Duration(5.0).sleep();

			destroyAllWindows();
			waitKey(50);
			
		}

		world_state=0;  //----------------------continue follower function after action---------------------------
	}

	return 0;
}
