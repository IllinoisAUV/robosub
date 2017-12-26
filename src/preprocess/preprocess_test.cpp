#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "filters/whitebalance.h"
#include "filters/median_subtract.h"

using namespace cv;

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_preprocessor");
  std::string imageName("");
  if(argc > 1) {
    imageName = argv[1];
  }

  Mat image;
  image = imread(imageName.c_str(), IMREAD_COLOR);

  if(image.empty()) {
    std::cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }

  Mat whiteBalanced = whiteBalance(image);
  Mat median = subtractMedian(whiteBalanced);

  namedWindow("Original", WINDOW_AUTOSIZE);
  imshow("Original", image);
  namedWindow("Color Balanced", WINDOW_AUTOSIZE);
  imshow("Color Balanced", whiteBalanced);
  namedWindow("median subtraction", WINDOW_AUTOSIZE);
  imshow("median subtraction", median);

  Mat colored;
  cvtColor(median, colored, CV_BGR2HSV);
  std::vector<Mat> channels;
  split(median, channels);
  namedWindow("B", WINDOW_AUTOSIZE);
  imshow("B", channels[0]);
  namedWindow("G", WINDOW_AUTOSIZE);
  imshow("G", channels[1]);
  namedWindow("R", WINDOW_AUTOSIZE);
  imshow("R", channels[2]);


  waitKey(0);
  return 0;
}
