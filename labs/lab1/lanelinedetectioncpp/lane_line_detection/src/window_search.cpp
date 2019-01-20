#include <opencv2/opencv.hpp>
#include <iostream>
#include "window_search.hpp"
#include "cv_helper.hpp"

void window_search(cv::Mat& binary_warped, cv::Mat& window_img, std::vector<std::unique_ptr<LaneLine>>& lane_lines, std::vector<int>& peaks, int n_windows, int margin, int minpix, int step) {
  int x_current;
  int window_height = binary_warped.rows/n_windows;
  int win_y_low, win_y_high, win_x_low, win_x_high;
  //binary_warped(cv::Range(0,binary_warped.rows),
  //cv::Range(binary_warped.cols/2,binary_warped.cols)).setTo(0);
  #pragma omp parallel for
  for (unsigned int i = 0; i < lane_lines.size(); i++) {
    cv::Mat good_inds, good_inds_x, good_inds_y, line_inds_x, line_inds_y;
    x_current = peaks[i];
    #pragma omp parallel for
    for (int window = 0; window < (n_windows * step - (step - 1)); window++) {
      // update position of windows, making sure it is within bounds of image
      win_y_low = binary_warped.rows - (window/step+1)*window_height;
      win_y_low = (win_y_low > 0) ? win_y_low : 0;
      win_y_high = binary_warped.rows - (window/step)*window_height;
      win_y_high= (win_y_high > 0) ? win_y_high : 0;
      win_x_low = x_current - margin;
      win_x_low = (win_x_low > 0) ? win_x_low : 0;
      win_x_high = x_current + margin;
      win_x_high = (win_x_high < binary_warped.cols) ? win_x_high : binary_warped.cols-1;

      try {
        // draw window
        cv::rectangle(window_img, cv::Point(win_x_low, win_y_low),
        cv::Point(win_x_high, win_y_high), cv::Scalar(0,255,0), 2);
        // color in pixels inside of window
        window_img(cv::Range(win_y_low,win_y_high),
        cv::Range(win_x_low,win_x_high)).setTo(lane_lines[i]->color,
        binary_warped(cv::Range(win_y_low,win_y_high),
        cv::Range(win_x_low,win_x_high)) != 0);
        // find non-zero pixel locations
        cv::findNonZero(binary_warped(cv::Range(win_y_low,win_y_high),
        cv::Range(win_x_low,win_x_high)), good_inds);
      }
      catch (const std::exception& e) {
        std::cerr << "Tried to access something out of range in window search: "
        << e.what();
        std::exit(1);
      }

      if (good_inds.rows > 0) {
        cv::extractChannel(good_inds, good_inds_x, 0);
        cv::extractChannel(good_inds, good_inds_y, 1);
        good_inds_x += win_x_low;
        good_inds_y += win_y_low;
        line_inds_x.push_back(good_inds_x);
        line_inds_y.push_back(good_inds_y);
        if (good_inds_x.rows > minpix) {
          x_current = std::round(cv::mean(good_inds_x)[0]);
        }
      }
    }
    lane_lines[i]->fit(line_inds_y, line_inds_x);
  }
}

std::vector<std::pair<double, double>> generate_waypoints(cv::Mat& overlay_img, std::vector<std::unique_ptr<LaneLine>>& lane_lines, double start_fraction, double stop_fraction, int num_waypoints, int radius, double xm_per_pix, double ym_per_pix) {
  // NOTE: This function assumes that the ploty for all lane lines is the same
  if (lane_lines.size() != 2) {
    std::stringstream ss;
    ss << "There can only be two lane lines.\n";
    throw std::invalid_argument(ss.str());
  }
  std::vector<std::pair<double, double>> waypoints(num_waypoints);
  std::array<cv::Scalar, 6> colors = {cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255), cv::Scalar(255,255,0), cv::Scalar(255,0,255), cv::Scalar(0,255,255)};
  cv::Mat center_line_pix = (lane_lines[1]->fitx_pix - lane_lines[0]->fitx_pix) / 2 + lane_lines[0]->fitx_pix;
  cv::Mat center_line_meters = (lane_lines[1]->fitx_meters - lane_lines[0]->fitx_meters) / 2 + lane_lines[0]->fitx_meters;
  int start = (1-start_fraction)*(lane_lines[0]->ploty_pix.rows-1);
  int stop = stop_fraction*(lane_lines[0]->ploty_pix.rows-1);
  double step = (stop - start) / (num_waypoints - 1);
  int x_pix, y_pix;
  double x_meters, y_meters;
  int index;
  int count = 0;
  for (double i = start; i > stop; i += step) {
    index = std::round(i);
    x_pix = center_line_pix.row(index).at<double>(0);
    y_pix = lane_lines[0]->ploty_pix.row(index).at<double>(0);
    cv::circle(overlay_img, cv::Point2i(x_pix, y_pix), radius,
    colors[count % colors.size()], -1);
    x_meters = center_line_meters.row(index).at<double>(0);
    y_meters = lane_lines[0]->ploty_meters.row(index).at<double>(0);
    x_meters -= overlay_img.cols*xm_per_pix/2;
    y_meters = overlay_img.rows*ym_per_pix - y_meters;
    //waypoints.push_back(std::make_pair(x_meters, y_meters));
    waypoints[count++] = std::make_pair(x_meters, y_meters);
  }

  return waypoints;
}
