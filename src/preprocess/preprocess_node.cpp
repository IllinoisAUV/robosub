#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "median_subtract.h"
#include "whitebalance.h"

using cv::Mat;

image_transport::Publisher pub;
image_transport::Subscriber sub;

void process(Mat &img) {
  // img is in BGR
  // Do any processing and place the result in img

  cvtColor(img, img, CV_BGR2HSV);
  whiteBalance(img);
  subtractMedian(img);
  cvtColor(img, img, CV_HSV2BGR);
}

void callback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  process(cv_ptr->image);
  pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_preprocessor");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  sub = it.subscribe("input", 1, callback);
  pub = it.advertise("output", 1);

  ros::spin();
  return 0;
}
