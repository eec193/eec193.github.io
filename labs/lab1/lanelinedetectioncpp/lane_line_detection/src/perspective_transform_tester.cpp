#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <array>
#include "overloader.hpp"
#include "cv_helper.hpp"

void draw_viewing_window(cv::Mat& dst, std::vector<cv::Point2f>& points, cv::Scalar color, int thickness);

int main(int argc, char* argv[]) {

  if (argc != 3) {
    std::cerr << "Usage: ./perspective_transform_tester <yaml_file> <path/to/images/\\*.png> \n";
    return 1;
  }

  cv::FileStorage file(argv[1], cv::FileStorage::READ);
  cv::Mat dist, mtx;
  std::vector<cv::Point2f> src_points, dst_points;

  file["distortion coefficients"] >> dist;
  file["camera matrix"] >> mtx;
  file["source points"] >> src_points;
  file["destination points"] >> dst_points;

  std::vector<cv::String> image_paths;
  cv::glob(argv[2], image_paths);

  std::string save_path = "../images/warped_images/";
  if (!boost::filesystem::exists(save_path)) {
    boost::filesystem::create_directory(save_path);
  }

  std::stringstream ss; // for saving images with unique names

  cv::Mat img, undistorted_img, src_img, dst_img, warped_img;
  std::vector<cv::Mat> images, undistorted_imgs;
  for (cv::String path : image_paths) {
    images.push_back(cv::imread(path));
  }

  for (cv::Mat img : images) {
    cv::undistort(img, undistorted_img, mtx, dist, mtx);
    undistorted_imgs.push_back(std::move(undistorted_img));
  }

  std::vector<cv::Mat> src_imgs;
  for (cv::Mat img : undistorted_imgs) {
    src_img = img.clone();
    draw_viewing_window(src_img, src_points, cv::Scalar(0,255,0), 5);
    src_imgs.push_back(src_img);
  }
  subplot("source points", src_imgs, 4, 2);

  cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);
  std::vector<cv::Mat> warped_imgs, dst_imgs;
  for (unsigned int i = 0; i < undistorted_imgs.size(); i++) {
    undistorted_img = undistorted_imgs[i];
    cv::warpPerspective(undistorted_img, warped_img, M, img.size());
    ss.str(std::string());
    ss << save_path << "warped" << i << ".png";
    cv::imwrite(ss.str(), warped_img);
    dst_img = warped_img.clone();
    draw_viewing_window(dst_img, dst_points, cv::Scalar(0,255,0), 5);
    warped_imgs.push_back(std::move(warped_img));
    dst_imgs.push_back(dst_img);
  }

  subplot("destination points", dst_imgs, 4, 2);
  subplot("warped images", warped_imgs, 4, 2);

  return 0;
}

void draw_viewing_window(cv::Mat& dst, std::vector<cv::Point2f>& points, cv::Scalar color, int thickness) {
  if (points.size() != 4) {
    std::stringstream ss;
    ss << "Invalid number of points. There must be exactly 4 points.\n";
    throw std::runtime_error(ss.str());
  }
  cv::line(dst, points[0], points[1], color, thickness);
  cv::line(dst, points[1], points[2], color, thickness);
  cv::line(dst, points[2], points[3], color, thickness);
  cv::line(dst, points[3], points[0], color, thickness);
}
