#include <opencv2/opencv.hpp>
#include <iostream>
#include "thresholds.hpp"
#include "window_search.hpp"
#include "lane_line.hpp"
#include "signal_proc.hpp"
#include  "cv_helper.hpp"
#include "timer.hpp"

LaneLine::LaneLine(double xm_per_pix, double ym_per_pix, int order, const cv::Mat& ploty_pix, const cv::Scalar color) {
  this->xm_per_pix = xm_per_pix;
  this->ym_per_pix = ym_per_pix;
  this->order = order;
  this->ploty_pix = ploty_pix; // NOTE: this makes a reference, not a copy
  this->ploty_meters = this->ploty_pix * this->ym_per_pix;
  this->color = color;
}

LaneLine::~LaneLine() {}

void LaneLine::fit(cv::Mat& line_inds_y, cv::Mat& line_inds_x) {

  line_inds_y.convertTo(line_inds_y, CV_64F);
  line_inds_x.convertTo(line_inds_x, CV_64F);

  try {
    if ((line_inds_y.rows > 0) && (line_inds_x.rows > 0)) {
      this->line_inds_y = line_inds_y;
      this->line_inds_x = line_inds_x;
      // fit best-fit line
      polyFit(line_inds_y, line_inds_x, this->current_fit_pix, this->order);
      polyFit(line_inds_y * this->ym_per_pix, line_inds_x * this->xm_per_pix,
      this->current_fit_meters, this->order);
      // evaluate best-fit line
      polyVal(this->ploty_pix, this->fitx_pix, this->current_fit_pix);
      polyVal(this->ploty_meters, this->fitx_meters, this->current_fit_meters);
    }
  }
  catch(const std::exception& e) {
    std::cerr << "Caught exception in LaneLine::fit(): " << e.what() << std::endl;
    std::exit(1);
  }
}

void LaneLine::drawLine(cv::Mat& window_img, cv::Mat& final_result_img, int margin, int thickness1, int thickness2) {
  cv::Mat pts1, pts2, left_line, right_line, poly_img;
  std::vector<cv::Mat> left_merge = {this->fitx_pix-margin, this->ploty_pix};
  cv::merge(left_merge, left_line);
  std::vector<cv::Mat> right_merge = {this->fitx_pix+margin, this->ploty_pix};
  cv::merge(right_merge, right_line);
  cv::flip(right_line, right_line, 0);
  cv::vconcat(left_line, right_line, pts2);
  pts2.convertTo(pts2, CV_32S);
  std::array<cv::Mat,1> poly = { pts2 };
  poly_img = cv::Mat::zeros(window_img.rows, window_img.cols, window_img.type());
  cv::fillPoly(poly_img, poly, cv::Scalar(0,255,0));
  cv::hconcat(this->fitx_pix, this->ploty_pix, pts1);
  pts1.convertTo(pts1, CV_32S);
  cv::polylines(window_img, pts1, 0, cv::Scalar(0,255,255), thickness1);
  cv::addWeighted(window_img, 1.0, poly_img, 0.3, 0.0, window_img);
  cv::polylines(final_result_img, pts1, 0, this->color, thickness2);
}

std::vector<std::pair<double, double>> detect_lane_lines(cv::Mat& src, cv::Mat& dst, std::vector<std::unique_ptr<LaneLine>>& lane_lines, cv::Mat& dist, cv::Mat& mtx, cv::Mat& M, cv::Mat& Minv, double xm_per_pix, double ym_per_pix) {
  std::vector<std::pair<double, double>> waypoints;
  std::vector<int> peaks;

  // number of windows to use
  int n_windows = 9;
  // set window height
  int step = 1;
  // set window width to +/- margin
  int margin = 150;
  // minimum number of pixels required to re-center window
  int minpix = 50;
  int radius;
  int num_waypoints = 8;

  int thickness1, thickness2;
  Timer time;
  cv::Mat undistorted_img, binary_warped, warped_img, window_img, overlay_img, histogram;
  cv::Mat img = src;
  time.reset();
  cv::undistort(img, undistorted_img, mtx, dist, mtx);
  std::cout << "undistort time: " << time.time() << std::endl;
  time.reset();
  cv::warpPerspective(undistorted_img, warped_img, M, img.size());
  std::cout <<"birds-eye view time: " << time.time() << std::endl;
  time.reset();
  apply_thresholds(warped_img, binary_warped, 3, 60, 255, 60, 255, 0.0, 0.8, 140, 255, 120, 255, 205, 255, 160, 255);
  std::cout << "thresholds time: " << time.time() << std::endl;
  cv::cvtColor(binary_warped, window_img, cv::COLOR_GRAY2BGR);
  time.reset();
  peaks = findPeaks(binary_warped, histogram);
  std::cout << "peaks time: " << time.time() << std::endl;
  time.reset();
  window_search(binary_warped, window_img, lane_lines, peaks, n_windows, margin, minpix, step);
  std::cout << "windows time: " << time.time() << std::endl;
  overlay_img = cv::Mat::zeros(window_img.rows, window_img.cols, window_img.type());
  thickness1 = std::round(binary_warped.cols/256);
  thickness1 = (thickness1 > 1) ? thickness1 : 1;
  thickness2 = std::round(binary_warped.cols/50);
  thickness2 = (thickness2 > 1) ? thickness2 : 1;
  radius = std::round(binary_warped.cols/(num_waypoints*10));
  radius = (radius > 1) ? radius : 1;
  time.reset();
  #pragma omp parallel for
  for (unsigned int j = 0; j < lane_lines.size(); j++) {
    lane_lines[j]->drawLine(window_img, overlay_img, margin, thickness1, thickness2);
  }
  std::cout << "draw lines time: " << time.time() << std::endl;
  time.reset();
  waypoints = generate_waypoints(overlay_img, lane_lines, 0.05, 0.05, num_waypoints, radius, xm_per_pix, ym_per_pix);
  std::cout << "waypoints time: " << time.time() << std::endl;
  cv::warpPerspective(overlay_img, overlay_img, Minv, overlay_img.size());
  cv::addWeighted(undistorted_img, 1, overlay_img, 0.5, 0, undistorted_img);
  dst = undistorted_img;
  return waypoints;
}
