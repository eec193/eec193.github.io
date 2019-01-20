#include <opencv2/opencv.hpp>
#include <iostream>
#include <matplotlibcpp.h>
#include "overloader.hpp"
#include "cv_helper.hpp"
#include "thresholds.hpp"
#include "window_search.hpp"
#include "signal_proc.hpp"
#include "lane_line.hpp"

namespace plt = matplotlibcpp;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: ./window_search_tester <path_to_warped_images>\\*.png> \n";
    return 1;
  }
  std::vector<cv::String> image_paths;
  cv::glob(argv[1], image_paths);
  //cv::glob("../images/warped_images/*.png", image_paths);
  cv::Mat warped_img, binary_warped, histogram, window_img, final_result_img;

  warped_img = cv::imread(image_paths[2]);
  apply_thresholds(warped_img, binary_warped, 15, 60, 255, 60, 255, 0.0, 0.8, 140, 255, 120, 255, 205, 255, 160, 255);
  //apply_thresholds(warped_img, binary_warped, 15, 60, 255, 60, 255, 0.0, 0.8, 120, 255, 120, 255, 205, 255, 150, 255);
  cv::cvtColor(binary_warped, window_img, cv::COLOR_GRAY2BGR);
  std::vector<int> peaks;
  //std::cout << "peaks: " << peaks << std::endl;
  //std::vector<double> histogram_vis;
  //histogram.copyTo(histogram_vis);
  //plt::plot(histogram_vis);
  //plt::show();
  double xm_per_pix = 3.7/700.;
  double ym_per_pix = 30./720.;
  std::vector<cv::Scalar> colors = {cv::Scalar(0,0,255), cv::Scalar(255,0,0)};
  int order = 2; // polynomial order for lane lines
  cv::Mat ploty;
  linspace(0, binary_warped.rows-1, binary_warped.rows, ploty);

  // number of windows to use
  int n_windows = 9;
  // set window height
  int step = 1;
  // set window width to +/- margin
  int margin = 150;
  // minimum number of pixels required to re-center window
  int minpix = 50;

  int thickness1, thickness2;
  thickness1 = std::round(binary_warped.cols/256);
  thickness1 = (thickness1 > 1) ? thickness1 : 1;
  thickness2 = std::round(binary_warped.cols/50);
  thickness2 = (thickness2 > 1) ? thickness2 : 1;

  int num_lane_lines = 2;
  cv::String path;
  std::vector<cv::Mat> axs(image_paths.size()*2);
  std::vector<std::unique_ptr<LaneLine>> lane_lines;
  for (int j = 0; j < num_lane_lines; j++) {
    lane_lines.push_back(std::unique_ptr<LaneLine> (new LaneLine(xm_per_pix, ym_per_pix, order, ploty, colors[j])));
  }
  for (unsigned int i = 0; i < image_paths.size(); i++) {
    path = image_paths[i];
    warped_img = cv::imread(path);
    apply_thresholds(warped_img, binary_warped, 15, 60, 255, 60, 255, 0.0, 0.8, 140, 255, 120, 255, 205, 255, 160, 255);
    cv::cvtColor(binary_warped, window_img, cv::COLOR_GRAY2BGR);
    peaks = findPeaks(binary_warped, histogram);
    window_search(binary_warped, window_img, lane_lines, peaks, n_windows, margin, minpix, step);
    final_result_img = cv::Mat::zeros(window_img.rows, window_img.cols, window_img.type());
    for (int j = 0; j < num_lane_lines; j++) {
      lane_lines[j]->drawLine(window_img, final_result_img, margin, thickness1, thickness2);
    }
    axs[i] = std::move(window_img);
    axs[i+image_paths.size()] = std::move(final_result_img);
  }
  subplot("title", axs, 8, 2);
}
