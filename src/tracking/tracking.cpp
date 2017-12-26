#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// #include "cmt_tracking/CMT.h"
// #include "cmt_tracking/gui.h"
// #include "cmt_tracking/getopt.h"

#define IMAGE_OUTPUT_FILTERED "/camera/image_processed"

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

void trackCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    // obatianing the image from the camera topic
    try{
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    cv::Mat in_img = cv_ptr->image;
    // init tracking and publish the bounding box
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_preprocessor");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // init publisher

    // init subscriber
    image_transport::Subscriber raw_img_sub = it.subscribe(IMAGE_OUTPUT_FILTERED, 2, trackCallback);
    ros::spin();
    return 0;
}
