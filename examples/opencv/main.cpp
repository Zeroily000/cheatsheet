#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

int main() {
  static char constexpr filename[]{"data/lena.png"};
  cv::Mat const image{cv::imread(filename, cv::IMREAD_COLOR)};

  cv::imshow("Display", image);
  cv::waitKey(0);

  return 0;
}
