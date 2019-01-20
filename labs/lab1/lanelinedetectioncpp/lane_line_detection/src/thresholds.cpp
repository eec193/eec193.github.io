#include "thresholds.hpp"
#include "cv_helper.hpp"

void abs_sobel_thresh(cv::Mat& src, cv::Mat& dst, char orient, uint8_t sobel_kernel=7, uint8_t thresh_min=40, uint8_t thresh_max=100) {
  cv::Mat gray, sobel;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

  if (orient == 'x') {
    cv::Sobel(gray, sobel, CV_32F, 1, 0, sobel_kernel);
  }
  else if (orient == 'y') {
    cv::Sobel(gray, sobel, CV_32F, 0, 1, sobel_kernel);
  }
  else {
    std::stringstream ss;
    ss << orient << "is invalid for argument orient. Must be either 'x' or 'y'.\n";
    throw std::runtime_error(ss.str());
  }

  sobel = cv::abs(sobel);
  double min, max;
  cv::minMaxLoc(sobel, &min, &max);
  sobel = (sobel*255)/max;
  sobel.convertTo(sobel, CV_8U);

  //cv::Mat img_min, img_max;
  //img_min = abs_sobel > thresh_min;
  //img_max = abs_sobel < thresh_max;
  //dst = img_min & img_max;
  dst = (sobel > thresh_min) & (sobel < thresh_max);
  //cv::threshold(abs_sobel, img_min, thresh_min, 255, cv::THRESH_BINARY);
  //cv::threshold(abs_sobel, img_max, thresh_max, 255, cv::THRESH_BINARY_INV);
  //cv::bitwise_and(img_min, img_max, dst);
}

void sobel_mag_dir_thresh(cv::Mat& src, cv::Mat& mag, cv::Mat& dir, uint8_t sobel_kernel, uint8_t mag_thresh_min=40, uint8_t mag_thresh_max=100, float dir_thresh_min=0.7, float dir_thresh_max=1.3) {
  cv::Mat gray, sobelx, sobely;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  // take derivatives with respect to x and y
  cv::Sobel(gray, sobelx, CV_64F, 1, 0, sobel_kernel);
  cv::Sobel(gray, sobely, CV_64F, 0, 1, sobel_kernel);

  sobelx = cv::abs(sobelx);
  sobely = cv::abs(sobely);

  cv::cartToPolar(sobelx, sobely, mag, dir);
  double min, max;
  cv::minMaxLoc(mag, &min, &max);
  mag = (mag*255)/max;
  mag.convertTo(mag, CV_8U);
  mag = (mag > mag_thresh_min) & (mag < mag_thresh_max);
  dir = (dir > dir_thresh_min) & (dir < dir_thresh_max);
}

void apply_thresholds(cv::Mat& src, cv::Mat& dst, uint8_t sobel_kernel, uint8_t gradx_thresh_low, uint8_t gradx_thresh_high, uint8_t mag_thresh_low, uint8_t mag_thresh_high, float dir_thresh_low, float dir_thresh_high, uint8_t hls_s_thresh_low, uint8_t hls_s_thresh_high, uint8_t hls_l_thresh_low, uint8_t hls_l_thresh_high, uint8_t hls_l_thresh2_low, uint8_t hls_l_thresh2_high, uint8_t lab_b_thresh_low, uint8_t lab_b_thresh_high) {

  cv::Mat sobelx_img, sobel_mag_img, sobel_dir_img, hls_src, lab_src, hls_dst, hls_dst2, lab_dst;

  abs_sobel_thresh(src, sobelx_img, 'x', sobel_kernel, gradx_thresh_low, gradx_thresh_high);
  sobel_mag_dir_thresh(src, sobel_mag_img, sobel_dir_img, sobel_kernel, mag_thresh_low, mag_thresh_high, dir_thresh_low, dir_thresh_high);

  cv::cvtColor(src, hls_src, cv::COLOR_BGR2HLS);
  cv::cvtColor(src, lab_src, cv::COLOR_BGR2Lab);

  cv::inRange(hls_src, cv::Scalar(0,hls_l_thresh_low,hls_s_thresh_low), cv::Scalar(255,hls_l_thresh_high,hls_s_thresh_high), hls_dst);

  cv::inRange(hls_src, cv::Scalar(0,hls_l_thresh2_low,0), cv::Scalar(255,hls_l_thresh2_high,255), hls_dst2);

  cv::inRange(lab_src, cv::Scalar(0,0,lab_b_thresh_low), cv::Scalar(255,255,lab_b_thresh_high), lab_dst);

  //dst = (sobelx_img & sobel_mag_img & sobel_dir_img) | (hls_dst | hls_dst2 | lab_dst);
  dst = hls_dst | hls_dst2 | lab_dst;

}
