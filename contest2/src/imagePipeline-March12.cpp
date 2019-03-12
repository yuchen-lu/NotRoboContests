#include <imagePipeline.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;



#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"



/** @function readme */
void readme()
{ std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }



// change from knect to webcam
//#define IMAGE_TOPIC "camera/image"


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
    if (!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if (img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates



        // load object image, read scene image
        //Mat img_object =
        Mat img_scene = img; // read img from video stream))
        Mat img_object = boxes.templates[1];
        imshow("img_object", img_object);
//        cv::waitKey(10000);


        if (!img_object.data || !img_scene.data) {
            std::cout << " --(!) Error reading images " << std::endl;
            return -1;
        }



        //-- Step 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
        int minHessian = 400;
        Ptr <SURF> detector = SURF::create(minHessian);
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;
        detector->detectAndCompute(img_object, Mat(), keypoints_object,
                                   descriptors_object);
        detector->detectAndCompute(img_scene, Mat(), keypoints_scene,
                                   descriptors_scene);

        //-- Step 3: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector<DMatch> matches;
        matcher.match(descriptors_object, descriptors_scene, matches);
        double max_dist = 0;
        double min_dist = 100;



        //-- Quick calculation of max and min distances between keypoints
        for (int i = 0; i < descriptors_object.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        printf("-- Max dist : %f \n", max_dist);
        printf("-- Min dist : %f \n", min_dist);


        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector<DMatch> good_matches;
        for (int i = 0; i < descriptors_object.rows; i++) {
            if (matches[i].distance < 3 * min_dist) { good_matches.push_back(matches[i]); }
        }

        Mat img_matches;

        drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for (int i = 0; i < good_matches.size(); i++) {
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
            scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
        }



        // Object/scene transform
        // using locations of matching keypts, transformation best maps the keypints is calculayted using findHomo
        // the transformation is applied using perspectiveTransfrom to a rect


        Mat H = findHomography(obj, scene, RANSAC);
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(img_object.cols, 0);
        obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
        obj_corners[3] = cvPoint(0, img_object.rows);
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform(obj_corners, scene_corners, H);


        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0),
             scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0),
             scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0),
             scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0),
             scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

        int good_match_number = good_match.size();

        cv::imshow("view", img);
        cv::imshow("good match & object detection", img_matches);
        cv::waitKey(10);
        return template_id;
    }
}


