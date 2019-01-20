#ifndef THRESHOLDS_HPP
#define THRESHOLDS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>

// applies sobel algorithm with respect to x or y and applies thresholds
void abs_sobel_thresh(cv::Mat& src, cv::Mat& dst, char orient, uint8_t sobel_kernel, uint8_t thresh_min, uint8_t thresh_max);

// uses sobel algorithm with respect to x and y to find direction of gradient and applies thresholds
// NOTE: thresh_min and thresh_max are in radians (0 - pi)
void sobel_mag_dir_thresh(cv::Mat& src, cv::Mat& mag, cv::Mat& angle, uint8_t sobel_kernel, uint8_t mag_thresh_min, uint8_t mag_thresh_max, float dir_thresh_min, float dir_thresh_max);

void apply_thresholds(cv::Mat& src, cv::Mat& dst, uint8_t sobel_kernel, uint8_t gradx_thresh_low, uint8_t gradx_thresh_high, uint8_t mag_thresh_low, uint8_t mag_thresh_high, float dir_thresh_low, float dir_thresh_high, uint8_t hls_s_thresh_low, uint8_t hls_s_thresh_high, uint8_t hls_l_thresh_low, uint8_t hls_l_thresh_high, uint8_t hls_l_thresh2_low, uint8_t hls_l_thresh2_high, uint8_t lab_b_thresh_low, uint8_t lab_b_thresh_high);


















#endif /* THRESHOLDS_HPP */
