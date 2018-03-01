#include "preprocess/whitebalance.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
// #include <opencv2/xphoto/white_balance.hpp>
//
using namespace cv;

void whiteBalance(Mat &img) {
  /* Mat img = src.clone(); */
  /* auto balancer = xphoto::createGrayworldWB(); */
  /* balancer->balanceWhite(img, result); */
  std::vector<Mat> channels;
  split(img, channels);
  normalize(channels[0], channels[0], 0, 255, NORM_MINMAX);
  normalize(channels[1], channels[1], 0, 255, NORM_MINMAX);
  normalize(channels[2], channels[2], 0, 255, NORM_MINMAX);
  /* equalizeHist(channels[0], channels[0]); */
  /* equalizeHist(channels[1], channels[1]); */
  /* equalizeHist(channels[2], channels[2]); */
  merge(channels, img);

  cvtColor(img, img, CV_BGR2HSV);

  split(img, channels);
  /* normalize(channels[0], channels[0], 0, 255, NORM_MINMAX); */
  normalize(channels[1], channels[1], 0, 255, NORM_MINMAX);
  normalize(channels[2], channels[2], 0, 255, NORM_MINMAX);
  merge(channels, img);
  cvtColor(img, img, CV_HSV2BGR);
}
