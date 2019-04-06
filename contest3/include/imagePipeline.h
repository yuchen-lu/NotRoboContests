#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>

class ImagePipeline {
    private:
        
        image_transport::Subscriber sub;
    public:
        ImagePipeline(ros::NodeHandle& n);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	
        
};
