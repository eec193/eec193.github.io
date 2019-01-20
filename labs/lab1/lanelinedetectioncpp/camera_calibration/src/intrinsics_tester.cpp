#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include "overloader.hpp"
#include "cv_helper.hpp"

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: ./intrinsics_tester <yaml_file> <path/to/images/\\*.png> \n";
    return 1;
  }
  cv::FileStorage file(argv[1], cv::FileStorage::READ);
  cv::Mat dist, mtx;
  file["distortion coefficients"] >> dist;
  file["camera matrix"] >> mtx;

  std::cout << "dist: " << dist << std::endl;
  std::cout << "mtx: " << mtx << std::endl;

  std::vector<cv::String> image_paths;
  cv::glob(argv[2], image_paths);

  // generate non-repeating random indexes for displaying undistortion results on random images
  std::vector<unsigned int> random_indexes(image_paths.size());
  std::iota(random_indexes.begin(), random_indexes.end(), 0);
  std::random_device random_dev;
  std::mt19937 generator(random_dev());
  std::shuffle(random_indexes.begin(), random_indexes.end(), generator);

  unsigned int num_images = 8;
  std::vector<cv::Mat> images;
  cv::Mat image, undistorted_img;
  for (unsigned int i = 0; i < num_images; i++) {
    image = cv::imread(image_paths[random_indexes[i]]);
    cv::undistort(image, undistorted_img, mtx, dist, mtx);
    images.push_back(std::move(image));
    images.push_back(std::move(undistorted_img));
  }
  subplot("window", images, 4, 4);
}
