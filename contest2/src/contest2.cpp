#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

float BoxCoord[5][3];

int FindTarget(float x, float y) {
    
    float d[5];

    for(int i = 0; i < 5; i++) {
        d[i] = sqrt((x - BoxCoord[i][0]) * (x - BoxCoord[i][0])  + (y - BoxCoord[i][1]) * (y - BoxCoord[i][1]));
    }

    int min = 0;

    for(int k = 1; k < 5; k++) {
        if (d[k] < d[min]){
            min = k;
        }
    }

    return min;
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
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
    // ImagePipeline imagePipeline(n);
    // Execute strategy.

    for(int j = 0; j < boxes.coords.size(); ++j) {
        BoxCoord[j][0] = boxes.coords[j][0];
        BoxCoord[j][1] = boxes.coords[j][1];
        BoxCoord[j][2] = boxes.coords[j][2];
    }
 
    Navigation navigation;

    while(ros::ok()) {
        ros::spinOnce();

        do {
            float posX = robotPose.x;
            float posY = robotPose.y;

            std::cout << posX;
            std::cout << "\n";
            std::cout << posY;
            std::cout << "\n";
            std::cout << "\n";

            int target = FindTarget(posX, posY);

            std::cout << target;
            std::cout << "\n";
            std::cout << "\n";

            double goalX = BoxCoord[target][0]+0.6*cos(BoxCoord[target][2]);
            double goalY = BoxCoord[target][1]+0.6*sin(BoxCoord[target][2]);

            std::cout << goalX;
            std::cout << "\n";
            std::cout << goalY;
            std::cout << "\n";
            std::cout << "\n";

            Navigation::moveToGoal(goalX, goalY,  BoxCoord[target][2]);
        }
        while (false);

        //imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
