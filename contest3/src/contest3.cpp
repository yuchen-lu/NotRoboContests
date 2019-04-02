#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
    //Fill with code
	//change world_state for emotion indication
	//backup a little bit???
}

//-------------------------------------------------------------

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

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//----------------change world state according to emotional sensor feedback------------

		if(world_state == 0){
			//fill with your code
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			
			//-----------------------***Note*** the sc.playWave() function does NOT stop the running of the main program!!!--------
			// -------------- Need add finite sleep() when playing soundtrack----------then call sc.stopWave()-------------------
			
		}
		world_state=0;  //----------------------continue follower function after action---------------------------
	}

	return 0;
}
