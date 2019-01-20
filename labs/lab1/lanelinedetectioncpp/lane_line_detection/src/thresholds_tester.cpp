#include <opencv2/opencv.hpp>
#include <iostream>
#include "cv_helper.hpp"
#include "overloader.hpp"
#include "thresholds.hpp"

int main() {
  std::vector<cv::String> image_paths;
  cv::glob("../images/warped_images/*.png", image_paths);

  cv::String path;
  cv::Mat warped_img, hls_warped_img, lab_warped_img, sobelx_img, sobely_img, sobel_mag_img, sobel_dir_img, grad_thresh_img, color_thresh_img, l_thresh_img, s_thresh_img, l_thresh_img2, b_thresh_img, result, dst;
  std::vector<cv::Mat> sobel_imgs(image_paths.size()*5);
  for (unsigned int i = 0; i < image_paths.size(); i++) {
    path = image_paths[i];
    warped_img = cv::imread(path);
    abs_sobel_thresh(warped_img, sobelx_img, 'x', 15, 40, 255);
    abs_sobel_thresh(warped_img, sobely_img, 'y', 7, 10, 40);
    sobel_mag_dir_thresh(warped_img, sobel_mag_img, sobel_dir_img, 15, 40, 255, 0.0, 0.8);
    //grad_thresh_img = (sobel_mag_img & sobel_dir_img) | (sobelx_img & sobely_img);
    grad_thresh_img = sobel_mag_img & sobel_dir_img & sobelx_img;
    cv::cvtColor(warped_img, hls_warped_img, cv::COLOR_BGR2HLS);
    cv::cvtColor(warped_img, lab_warped_img, cv::COLOR_BGR2Lab);
    cv::inRange(hls_warped_img, cv::Scalar(0,120,0), cv::Scalar(255,255,255), l_thresh_img);
    cv::inRange(hls_warped_img, cv::Scalar(0,0,140), cv::Scalar(255,255,255), s_thresh_img);
    cv::inRange(hls_warped_img, cv::Scalar(0,205,0), cv::Scalar(255,255,255), l_thresh_img2);
    cv::inRange(lab_warped_img, cv::Scalar(0,0,160), cv::Scalar(255,255,255), b_thresh_img);
    color_thresh_img = (l_thresh_img & s_thresh_img) | b_thresh_img | l_thresh_img2;
    result = grad_thresh_img | color_thresh_img;
    apply_thresholds(warped_img, dst, 15, 60, 255, 60, 255, 0.0, 0.8, 140, 255, 120, 255, 205, 255, 160, 255);
    sobel_imgs[i] = std::move(warped_img);
    sobel_imgs[i+image_paths.size()] = std::move(grad_thresh_img);
    sobel_imgs[i+2*image_paths.size()] = std::move(color_thresh_img);
    sobel_imgs[i+3*image_paths.size()] = std::move(result);
    sobel_imgs[i+4*image_paths.size()] = std::move(dst);
  }
  subplot("title", sobel_imgs, 8, 5);
}
