#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include "robosub/VisualTarget.h"

using cv::Mat;

ros::Publisher pub;
image_transport::Subscriber sub;

robosub::VisualTarget process(Mat &img) {
  // img is in BGR
  // Do any processing and place the result in img
  cvtColor(img, img, CV_BGR2HSV);
  cvtColor(img, img, CV_HSV2BGR);

  // hard-coded for now, will change later
  robosub::VisualTarget target;

  target.x = 10;
  target.y = 50;

  return target;
}

void callback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  robosub::VisualTarget target_ctr;
  target_ctr = process(cv_ptr->image);
  pub.publish(target_ctr);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_processor");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  sub = it.subscribe("videofile/image_raw/", 1, callback);
  pub = nh.advertise<robosub::VisualTarget>("target", 1);

  ros::spin();
  return 0;
}
