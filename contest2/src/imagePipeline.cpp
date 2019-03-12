#include <imagePipeline.h>
#include<iostream>
#include<stdio.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
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
        for(int tempNumber=0;tempNumber<=2;tempNumber++){
            // ---------------------------------------------Read in the tempate object image-------------------------------------------------//
            Mat img_object = boxes.templates[tempNumber];    //need to change to boxes.templates[i], i=0,1,2, return template_id = i !!!!!!!!!!!!!!!!
            Mat img_scene = img;

            if( !img_scene.data )  //If camera data is missing
            { std::cout<< " --(!) Error reading img_scene " << std::endl; return -1; }
            if( !img_object.data )  //If file template file cannot be loaded
            { std::cout<< " --(!) Error reading img_object " << std::endl; return -1; }

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
                    good_matches.push_back( matches[i]);
                }
            }

            //code onward is not necessary for matchmaking
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

            //----------------------------------------------------object/Scene Transformation----------------------------------------------------//
            Mat H = findHomography( obj, scene, RANSAC );

            //-- Get the corners from the image_1 ( the object to be "detected" )
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
            std::cout<<scene_corners[0]<<"\t"<<scene_corners[1]<<"\t"<<scene_corners[2]<<"\t"<<scene_corners[3]<<"\n";

            // check best match??
            double Top,Right,Bottom,Left,leftDif,topDif,rightDif,bottomDif;

            Left=abs(scene_corners[0].x-scene_corners[1].x);  //Calculate side length
            Top=abs(scene_corners[1].y-scene_corners[2].y);
            Right=abs(scene_corners[2].x-scene_corners[3].x);
            Bottom=abs(scene_corners[3].y-scene_corners[0].y);

            leftDif=abs(scene_corners[0].y-scene_corners[1].y);
            topDif=abs(scene_corners[1].x-scene_corners[2].x);
            rightDif=abs(scene_corners[2].y-scene_corners[3].y);
            bottomDif=abs(scene_corners[3].x-scene_corners[0].x);


            if(Left>30 && Top>30 && Right>30 && Bottom>30){ 
                if(leftDif<20 && topDif<20 && rightDif<20 && bottomDif<20){
                    template_id = tempNumber;
                }   
            }
            else{
                std::cout<<"Does not match template "<<tempNumber<<"\n";
            }
            
            cv::imshow("good match & object detection", img_matches);
            cv::imshow("view", img);
            cv::waitKey(1000);
        }
    }
    return template_id;
}