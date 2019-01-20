#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <utility>
#include "thresholds.hpp"
#include "signal_proc.hpp"
#include "window_search.hpp"
#include "lane_line.hpp"
#include "cv_helper.hpp"
#include "overloader.hpp"

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: ./images_main <yaml_file> <path_to_test_images>\\*.png \n";
    return 1;
  }

  std::vector<cv::String> image_paths;
  cv::glob(argv[2], image_paths);

  cv::FileStorage file(argv[1], cv::FileStorage::READ);
  cv::Mat dist, mtx;
  std::vector<cv::Point2f> src_points, dst_points;

  file["distortion coefficients"] >> dist;
  file["camera matrix"] >> mtx;
  file["source points"] >> src_points;
  file["destination points"] >> dst_points;
  cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);
  cv::Mat Minv = cv::getPerspectiveTransform(dst_points, src_points);

  double xm_per_pix = 3.7/700.;
  double ym_per_pix = 30./720.;
  cv::Mat ploty;
  //int frame_width = 1280;
  int frame_height = 720;
  linspace(0, frame_height-1, frame_height, ploty);
  std::vector<cv::Scalar> colors = {cv::Scalar(0,0,255), cv::Scalar(255,0,0)};
  int order = 2;

  std::vector<std::unique_ptr<LaneLine>> lane_lines;
  for (int j = 0; j < 2; j++) {
    lane_lines.push_back(std::unique_ptr<LaneLine> (new LaneLine(xm_per_pix, ym_per_pix, order, ploty, colors[j])));
  }
  std::vector<std::pair<double, double>> waypoints;
  cv::Mat img, result;
  std::vector<cv::Mat> axs(image_paths.size());
  for (unsigned int i = 0; i < image_paths.size(); i++) {
    img = cv::imread(image_paths[i]);
    waypoints = detect_lane_lines(img, result, lane_lines, dist, mtx, M, Minv,
    xm_per_pix, ym_per_pix);
    std::cout << "waypoinfts: " << waypoints << std::endl;
    axs[i] = std::move(result);
  }
  subplot("result", axs, 4, 2);
  return 0;
}
