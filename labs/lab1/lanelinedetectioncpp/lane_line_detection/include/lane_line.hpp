#ifndef LANE_LINE_HPP
#define LANE_LINE_HPP
#include <utility>

class LaneLine {
 public:
  // Constructor
  LaneLine(double xm_per_pix, double ym_per_pix, int order, const cv::Mat& ploty_pix, const cv::Scalar color);
  // Destructor
  ~LaneLine();
  // Updates best-fit line of lane line
  void fit(cv::Mat& line_inds_x, cv::Mat& line_inds_y);
  // Draws lane line on image with windows and on a blank final resulting image
  void drawLine(cv::Mat& window_img, cv::Mat& final_result_img, int margin, int thickness1, int thickness2);
  // all x locations of nonzero pixels for lane line
  cv::Mat line_inds_x;
  // all y locations of nonzero pixels for lane line
  cv::Mat line_inds_y;
  // y values over which to plot lane line in pixels
  cv::Mat ploty_pix;
  // y values over which to plot lane line in meters
  cv::Mat ploty_meters;
  // x values of current best-fit line in pixels
  cv::Mat fitx_pix;
  // x values of current best-fit line in meters
  cv::Mat fitx_meters;
  // color of lane lines pixels
  cv::Scalar color;
 private:
  // current best-fit coefficients for lane line in pixels
  cv::Mat current_fit_pix;
  // currrent best-fit coefficients for lane line in meters
  cv::Mat current_fit_meters;
  // conversion factor for pixels to meters along x-axis
  double xm_per_pix;
  // conversion factor for pixels to meters along y-axis
  double ym_per_pix;
  // order of polynomial to be used for best-fit line
  int order;

};

std::vector<std::pair<double, double>> detect_lane_lines(cv::Mat& src, cv::Mat& dst, std::vector<std::unique_ptr<LaneLine>>& lane_lines, cv::Mat& dist, cv::Mat& mtx, cv::Mat& M, cv::Mat& Minv, double xm_per_pix, double ym_per_pix);

#endif /* LANE_LINE_HPP */
