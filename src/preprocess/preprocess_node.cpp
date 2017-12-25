#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "filters/whitebalance.h"
#include "filters/median_subtract.h"
#include "filters/bilateral_filter.h"

#define IMAGE_INPUT_RAW "/camera/image_raw"
#define IMAGE_OUTPUT_FILTERED "/camera/image_processed"

image_transport::Publisher filter_img_pub;

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    // obatianing the image from the camera topic
    try{
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    // processing the image using white balancing filter
    cv::Mat processed_img = cv_ptr->image;
    processed_img = whiteBalance(processed_img);

    // pubslishing the image after converting back to sensor_msgs.image format
    sensor_msgs::ImagePtr msg_out;
    msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", processed_img).toImageMsg();
    filter_img_pub.publish(msg_out);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_preprocessor");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // init publisher
    //@TODO correct of images to be stored in the topic
    filter_img_pub = it.advertise( IMAGE_OUTPUT_FILTERED, 2);

    // init subscriber
    image_transport::Subscriber raw_img_sub = it.subscribe(IMAGE_INPUT_RAW, 2, imageCallback);
    ros::spin();
    return 0;
}
