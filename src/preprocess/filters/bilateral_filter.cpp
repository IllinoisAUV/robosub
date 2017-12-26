#include "preprocess/filters/bilateral_filter.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

using namespace cv;

Mat bilateral_filter(Mat &src) {
  Mat img = src.clone();
  Mat bi_img;
  cv::bilateralFilter ( src, bi_img, 10, 50, 50 );
  return img;
}
