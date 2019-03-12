#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose); // subscribe a class fnc
    // Initialize box coordinates and templates
    Boxes boxes;
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: "
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    //**** your code here

    // init navi
    Navigation navigation;

    float offset = 0.75;
    float x_pose, y_pose, orientation;
    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        // ROS_INFO("X :%f, Y: %f,Z: %f",robotPose.x, robotPose.y, robotPose.phi );
        // ROS_INFO("X_box1 :%f, Y_box1: %f,Z_box2: %f",boxes.coords[0][0], boxes.coords[0][1], boxes.coords[0][2] );
        // x_pose = robotPose.x;
        // y_pose = robotPose.y;
        // orientation = robotPose.phi;
        // ROS_INFO("Xstart :%f, Ystart: %f,Zstart: %f",x_pose, y_pose, orientation);
        // navigation.moveToGoal(boxes.coords[0][0]+offset*cos(boxes.coords[0][2]), boxes.coords[0][1]+offset*cos(boxes.coords[0][2]),boxes.coords[0][2]);
        // ROS_INFO("FINISHED 1");
        // ros::spinOnce();

        // ROS_INFO("X :%f, Y: %f,Z: %f",x_pose, y_pose, orientation );
        // navigation.moveToGoal(boxes.coords[1][0]+offset*cos(boxes.coords[1][2]), boxes.coords[1][1]+offset*cos(boxes.coords[1][2]),boxes.coords[1][2]);
        // ROS_INFO("FINISHED 2");
        // ros::spinOnce();

        // ROS_INFO("X :%f, Y: %f,Z: %f",x_pose, y_pose, orientation );
        // navigation.moveToGoal(boxes.coords[2][0]+offset*cos(boxes.coords[2][2]), boxes.coords[2][1]+offset*cos(boxes.coords[2][2]),boxes.coords[2][2]);
        // ROS_INFO("FINISHED 3");
        // navigation.moveToGoal(boxes.coords[1][0], boxes.coords[1][1], boxes.coords[1][2]);
        // ROS_INFO("X :%f, Y: %f,Z: %f",robotPose.x, robotPose.y, robotPose.phi );
        // ROS_INFO("X_box2 :%f, Y_box2: %f,Z_box2: %f",boxes.coords[1][0], boxes.coords[1][1], boxes.coords[1][2] );
        // navigation.moveToGoal(boxes.coords[2][0], boxes.coords[2][1], boxes.coords[2][2]);
        // navigation.moveToGoal(boxes.coords[3][0], boxes.coords[3][1], boxes.coords[3][2]);
        // navigation.moveToGoal(boxes.coords[4][0], boxes.coords[4][1], boxes.coords[4][2]);

        int match_id = imagePipeline.getTemplateID(boxes);
        ROS_INFO("match id is : %d\n", match_id);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
