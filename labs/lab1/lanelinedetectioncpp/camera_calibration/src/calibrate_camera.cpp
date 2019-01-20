#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include "overloader.hpp"
#include "cv_helper.hpp"

int main(int argc, char* argv[]) {

  if (argc != 3) {
    std::cerr << "Usage: ./calibrate_camera <yaml_file> <path/to/images/\\*.png> \n";
    return 1;
  }
  // distortion coefficients and camera matrix
  cv::Mat dist, mtx;
  // Reading source and destination points to save original yaml file state
  std::vector<cv::Point2f> src_points, dst_points;
  {
    cv::FileStorage file(argv[1], cv::FileStorage::READ);
    file["distortion coefficients"] >> dist;
    file["camera matrix"] >> mtx;
    file["source points"] >> src_points;
    file["destination points"] >> dst_points;
    file.release();
  }

  std::stringstream ss;
  std::vector<cv::String> image_paths;
  cv::glob(argv[2], image_paths);
  std::cout << image_paths << std::endl;

  unsigned int nx = 10;
  unsigned int ny = 7;
  cv::Size boardSize(nx, ny);

  std::vector<cv::Point2f> imgp;
  std::vector<cv::Point3f> objp;
  for (float i = 0; i < ny; i++) {
    for (float j = 0; j < nx; j++) {
      objp.push_back(cv::Point3f(static_cast<float>(i), static_cast<float>(j), 0.0));
    }
  }
  std::cout << objp << std::endl;

  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  cv::Mat image, gray, drawn_corners_img;
  bool ret;
  std::string save_path = "../drawn_corners/";
  if (!boost::filesystem::exists(save_path)) {
    boost::filesystem::create_directory(save_path);
  }
  for (unsigned int i = 0; i < image_paths.size(); i++) {
    image = cv::imread(image_paths[i]);
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    ret = cv::findChessboardCorners(gray, boardSize, imgp);
    if (ret) {
      img_points.push_back(imgp);
      obj_points.push_back(objp);
      drawn_corners_img = image.clone();
      cv::drawChessboardCorners(drawn_corners_img, boardSize, imgp, ret);
      ss.str(std::string());
      ss << save_path << i << ".png";
      cv::imwrite(ss.str(), drawn_corners_img);
    }
  }

  std::vector<cv::Mat> rvecs, tvecs;

  cv::calibrateCamera(obj_points, img_points, image.size(), mtx, dist, rvecs, tvecs);

  std::cout << "dist: " << dist << std::endl;
  std::cout << "mtx: " << mtx << std::endl;

  cv::FileStorage file(argv[1], cv::FileStorage::WRITE);
  file << "distortion coefficients" << dist;
  file << "camera matrix" << mtx;
  file << "source points" << src_points;
  file << "destination points" << dst_points;
  file.release();

}
