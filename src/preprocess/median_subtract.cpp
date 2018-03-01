#include <opencv2/core/core.hpp>

using namespace cv;

void subtractMedian(Mat &src) {
  /* std::vector<Mat> channels; */
  /* split(src, channels); */

  Scalar m = mean(src);
  /* Scalar b = mean(channels[0]); */
  /* Scalar g = mean(channels[1]); */
  /* Scalar r = mean(channels[2]); */

  /* Mat res(src.rows, src.cols, CV_8UC3); */
  /* res = m; */
  src -= m;
}
