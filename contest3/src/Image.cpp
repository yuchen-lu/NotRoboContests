#include <imagePipeline.h>
#include<iostream>
#include<stdio.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <ros/ros.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <std_msgs/Int8.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/image" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

bool isValid;
Mat img;

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
    cout<<"img pipe\n";
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cout<<"try\n";
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();box
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        cout<<"catch\n";
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int main(int argc, char **argv) {

    ros::init(argc,argv,"OCV");
    ros::NodeHandle n;
    ros::Publisher img_pub = n.advertise<std_msgs::Int8>("img_contest3", 1);

    std_msgs::Int8 banana;

    int template_id = 0;

    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.box
        template_id = 0;
        while (ros::ok()) {
            // ---------------------------------------------Read in the tempate object image-------------------------------------------------//
            Mat img_object = imread("/home/ziqi/catkin_ws/src/mie443_contest3/bot.png", IMREAD_GRAYSCALE);
            Mat img_scene = img;

            if( !img_scene.data )  //If camera data is missing
            {
                std::cout<< " --(!) Error reading img_scene " << std::endl; 
                template_id = -1; 
            }
            if( !img_object.data )  //If file template file cannot be loaded
            { 
                std::cout<< " --(!) Error reading img_object " << std::endl;
                template_id = -1;
            }

            if (template_id != -1) {

                //----------------------------------Detect the keypoints and calculate descriptors using SURF Detector---------------------------//
                int minHessian = 400;
                Ptr<SURF> detector = SURF::create(minHessian);
                std::vector<KeyPoint> keypoints_object, keypoints_scene;
                Mat descriptors_object, descriptors_scene;
                detector->detectAndCompute(img_object, Mat(), keypoints_object,descriptors_object);
                detector->detectAndCompute(img_scene, Mat(), keypoints_scene,descriptors_scene);

                //-----------------------------------------Feature matching using FLANN matcher---------------------------------------------------//
                FlannBasedMatcher matcher;
                std::vector< DMatch > matches;
                matcher.match( descriptors_object, descriptors_scene, matches );

                //-------------------Determine quality of matchers: Quick calculation of max and min distanes between keypoints-------------------//
                double max_dist = 0; double min_dist = 100;

                for(int i = 0; i < descriptors_object.rows; i++){
                    double dist = matches[i].distance;
                    if( dist < min_dist ) min_dist = dist;
                    if( dist > max_dist ) max_dist = dist;
                }

                //--------------------Outlier Rejection: Draw only "good" matches (i.e. whose distance is less than 3*min_dist )------------------//
                std::vector< DMatch > good_matches;

                for( int i = 0; i < descriptors_object.rows; i++ ){
                    if( matches[i].distance < 4*min_dist ){
                        good_matches.push_back(matches[i]);
                    }
                }
                
                if(good_matches.size()<4){            //return -2 if core dumped. Move robot and re-run imagepipeline
                    std::cout<<"error: core dumped(good match)                    ";
                    template_id = 0;
                }
            
                Mat img_matches;
                drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,good_matches, img_matches,
                            Scalar::all(-1), Scalar::all(-1),std:: vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                //--------------------------------------------Getting corresponding matched keypoints-----------------------------------------------//
                //Localize the object
                std::vector<Point2f> obj;
                std::vector<Point2f> scene;

                for( int i = 0; i < good_matches.size(); i++ )
                {
                    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
                }

                //----------------------------------------------------Object/Scene Transformation----------------------------------------------------//
                Mat H = findHomography( obj, scene, RANSAC );
                if(H.rows<3 || H.cols<3){           //return -2 if core dumped
                    std::cout<<"error: core dumped(H)                     ";
                    template_id = 0;
                }

                //Get the corners from the image_1 ( the object to be "detected" )
                std::vector<Point2f> obj_corners(4);
                obj_corners[0] = cvPoint(0,0); 
                obj_corners[1] = cvPoint( img_object.cols, 0 );
                obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); 
                obj_corners[3] = cvPoint( 0, img_object.rows );
                std::vector<Point2f> scene_corners(4);
                perspectiveTransform( obj_corners, scene_corners, H);

                //------------------------------------Print the bounded object in the scene (draw lines)---------------------------------------------//
                line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0),
                    scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
                line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0),
                    scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
                line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0),
                    scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
                line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0),
                    scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

                //-----------------------------------------------------check best match-------------------------------------------------------------//
                double Top,Right,Bottom,Left,leftDif,topDif,rightDif,bottomDif;

                Left=abs(scene_corners[0].x-scene_corners[1].x);  //Calculate side length
                Top=abs(scene_corners[1].y-scene_corners[2].y);
                Right=abs(scene_corners[2].x-scene_corners[3].x);
                Bottom=abs(scene_corners[3].y-scene_corners[0].y);

                leftDif=abs(scene_corners[0].y-scene_corners[1].y);
                topDif=abs(scene_corners[1].x-scene_corners[2].x);
                rightDif=abs(scene_corners[2].y-scene_corners[3].y);
                bottomDif=abs(scene_corners[3].x-scene_corners[0].x);




                if(Left>30 && Top>30 && Right>30 && Bottom>30){ //side length meet requirement
                    if(leftDif<80 && topDif<80 && rightDif<80 && bottomDif<80){  //Overall shape meet requirement
                        template_id = 1;
                        std::cout<<template_id<<"\n";
                    }   
                    else{  //Overall shape does not meet requirement
                        template_id = 0;
                    }
                }
                else{   //Side length not meet requirement 
                    template_id = 0;
                }
                
                cv::waitKey(200);
                banana.data = template_id;
                img_pub.publish(banana);
            }
        }
    }
    return 0;
}