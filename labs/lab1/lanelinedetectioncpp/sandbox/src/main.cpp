#include <opencv2/opencv.hpp>
#include <iostream>
#include "overloader.hpp"
#include "cv_helper.hpp"
#include "signal_proc.hpp"

int main() {
  std::vector<cv::String> image_paths;
  cv::glob("../images/test_images/*.jpg", image_paths);
  std::vector<cv::Mat> axs(image_paths.size()*2);
  cv::Mat img, gray;
  for (unsigned int i = 0; i < image_paths.size(); i++) {
    img = cv::imread(image_paths[i]);
    //std::cout << "img.type(): " << img.type() << std::endl;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    //std::cout << "nonzero at (50,100): " << static_cast<int>(gray.at<uint8_t>(100,50)) << std::endl;
    axs[i] = std::move(gray);

    axs[i+image_paths.size()] = std::move(img);
  }
  cv::Mat ploty;
  linspace(0, 719, 720, ploty);
  std::cout << "shape(ploty): " << shape(ploty) << std::endl;
  std::cout << "ploty: " << ploty << std::endl;
  subplot("title", axs, 8, 2);

}
