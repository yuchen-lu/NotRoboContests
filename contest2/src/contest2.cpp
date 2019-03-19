#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <fstream>

using namespace std;

float BoxCoord[5][3];
double pi = 3.14;
int FootPrint[5] = {0, 0, 0, 0, 0};
float d[5];

int FindTarget(float x, float y) {

    for(int i = 0; i < 5; i++) {
        if (FootPrint[i]==0) {
            d[i] = sqrt((x - BoxCoord[i][0]) * (x - BoxCoord[i][0])  + (y - BoxCoord[i][1]) * (y - BoxCoord[i][1]));
        } else {
            d[i] = 100;
        }
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
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    for(int j = 0; j < boxes.coords.size(); ++j) {
        BoxCoord[j][0] = boxes.coords[j][0];
        BoxCoord[j][1] = boxes.coords[j][1];
        BoxCoord[j][2] = boxes.coords[j][2];
    }
 
    Navigation navigation;

    int counter = 0;

    ofstream myfile;
    myfile.open ("contest2Result.txt");
    myfile<<"Output format: template number\tX\tY\tPhi\n\n";

    while(ros::ok()) {
        ros::spinOnce();

        float InitialPosX;
        float InitialPosY;
        float InitialPosZ;

        float posX = robotPose.x;
        float posY = robotPose.y;

        int target = FindTarget(posX, posY);
        FootPrint[target] = 1;

        if (counter == 0) {
            InitialPosX = robotPose.x;
            InitialPosY = robotPose.y;
            InitialPosZ = robotPose.phi;

        } else if (counter == 5) {
            Navigation::moveToGoal(InitialPosX, InitialPosY, InitialPosZ);
            std::cout << "Original location reached \n";
            break;
        }

        if (d[target] != 10) {

            counter ++;

            double goalX = BoxCoord[target][0]+0.5*cos(BoxCoord[target][2]);
            double goalY = BoxCoord[target][1]+0.5*sin(BoxCoord[target][2]);
            double goalZ = BoxCoord[target][2];

            if (goalZ >= 0) {
                goalZ -= pi;
            } else {
                goalZ += pi;
            }

            Navigation::moveToGoal(goalX, goalY, goalZ);

            ros::spinOnce();
            usleep(1000);

            if(imagePipeline.getTemplateID(boxes) == -2){   //safety check for core dump (return -2 from imagePipeline)   
                int originalGoalZ = goalZ;
                if (goalZ >= 0) {
                    do{
                        cout<<"relocate for core dump \n"; 
                        goalZ = goalZ - pi/36;
                        cout<<goalZ<<"\n";
                        Navigation::moveToGoal(goalX, goalY, goalZ);
                        ros::spinOnce();
                        usleep(1000);
                    }while(imagePipeline.getTemplateID(boxes) == -2 && abs(originalGoalZ-goalZ)<=1);
                } 
                else {
                    do{
                        cout<<"relocate for core dump \n"; 
                        goalZ = goalZ + pi/36;
                        cout<<goalZ<<"\n";
                        Navigation::moveToGoal(goalX, goalY, goalZ);
                        ros::spinOnce();
                        usleep(1000);
                    }while(imagePipeline.getTemplateID(boxes) == -2 && abs(originalGoalZ-goalZ)<=1);
                }           
            }

            do {   //Safety check for invalid image (return -1 from imagePipeline)

            }while (imagePipeline.getTemplateID(boxes) == -1);


            int Match1 = imagePipeline.getTemplateID(boxes);  //take three measurements to confirm they match
            int Match2 = imagePipeline.getTemplateID(boxes);
            int Match3 = imagePipeline.getTemplateID(boxes);

            if (Match1 == Match2 && Match1 == Match3) {
                if (Match1 != 0) {
                    std::cout << "Match with Template " << Match1  <<"\n\n"; 
                    myfile<<"Template "<<Match1<<"\t"<<BoxCoord[target][0]<<"\t"<<BoxCoord[target][1]<<"\t"<<BoxCoord[target][2]<<"\n";
                }
                else {
                    std::cout << "No match, blank pic detected \n\n";
                    myfile<<"no match \t"<<BoxCoord[target][0]<<"\t"<<BoxCoord[target][1]<<"\t"<<BoxCoord[target][2]<<"\n";
                }
            }
            else {  
                // back up a bit 
                goalX = BoxCoord[target][0]+0.6*cos(BoxCoord[target][2]);
                goalY = BoxCoord[target][1]+0.6*sin(BoxCoord[target][2]);
                goalZ = BoxCoord[target][2];
                Navigation::moveToGoal(goalX, goalY, goalZ);
                ros::spinOnce();
                // read image again

                Match1 = imagePipeline.getTemplateID(boxes);  //Re-take three measurements to confirm they match
                Match2 = imagePipeline.getTemplateID(boxes);
                Match3 = imagePipeline.getTemplateID(boxes);

                if (Match1 == Match2 && Match1 == Match3) {
                    if (Match1 != 0) {
                        std::cout << "Match with Template " << Match1 <<"\n\n";
                        myfile<<"Template "<<Match1<<"\t"<<BoxCoord[target][0]<<"\t"<<BoxCoord[target][1]<<"\t"<<BoxCoord[target][2]<<"\n"; 
                    }
                    else {
                        std::cout << "No match, blank pic detected \n\n";
                        myfile<<"no match \t"<<BoxCoord[target][0]<<"\t"<<BoxCoord[target][1]<<"\t"<<BoxCoord[target][2]<<"\n";
                    }
                } else {
                    std::cout << "ERROR \n";  //Posible solution: navigate robot to the next position and come back to this location later
                    myfile<<"Error \t"<<BoxCoord[target][0]<<"\t"<<BoxCoord[target][1]<<"\t"<<BoxCoord[target][2]<<"\n";
                }
            }
        }
        

        ros::Duration(0.01).sleep();
    }

    myfile.close();
    return 0;
}
