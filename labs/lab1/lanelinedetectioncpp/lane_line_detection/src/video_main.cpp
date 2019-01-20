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
#include "timer.hpp"

// trims leading and trailing white spaces from string
std::string trim(const std::string& str,
                 const std::string& whitespace=" ");

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: ./video_main <yaml_file> <path_to_input_video> <path_to_output_video>.mp4 \n";
    return 1;
  }
  if (boost::filesystem::exists(argv[3])) {
    std::cout << argv[3] << " already exists. Are you sure you want to overwrite?"
    << std::endl;
    std::string user_input = "";
    while (user_input != "y" && user_input != "Y" && user_input != "n" &&
    user_input != "N") {
      std::cout  << "[y/n] ";
      std::getline(std::cin, user_input);
      user_input = trim(user_input);
      if (user_input == "y" || user_input == "Y") {
        std::cout << "Overwriting " << argv[3] << "...\n";
      }
      else if (user_input == "n" || user_input == "N") {
        std::cout << "Canceling program.\n";
        std::exit(0);
      }
      else {
        std::cout << "Invalid command.\n";
      }
    }
  }
  cv::FileStorage file(argv[1], cv::FileStorage::READ);
  cv::Mat dist, mtx;
  std::vector<cv::Point2f> src_points, dst_points;

  file["distortion coefficients"] >> dist;
  file["camera matrix"] >> mtx;
  file["source points"] >> src_points;
  file["destination points"] >> dst_points;
  cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);
  cv::Mat Minv = cv::getPerspectiveTransform(dst_points, src_points);

  //std::vector<cv::String> image_paths;
  //cv::glob(argv[2], image_paths);

  double xm_per_pix = 3.7/700.;
  double ym_per_pix = 30./720.;
  cv::Mat ploty;
  int frame_width = 1280;
  int frame_height = 720;
  linspace(0, frame_height-1, frame_height, ploty);
  std::vector<cv::Scalar> colors = {cv::Scalar(0,0,255), cv::Scalar(255,0,0)};
  int order = 2;

  std::vector<std::unique_ptr<LaneLine>> lane_lines;
  for (int j = 0; j < 2; j++) {
    lane_lines.push_back(std::unique_ptr<LaneLine> (new LaneLine(xm_per_pix, ym_per_pix, order, ploty, colors[j])));
  }
  std::vector<std::pair<double, double>> waypoints;
  cv::Mat frame, result;

  cv::VideoCapture cap(argv[2]);
  int length = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
  cv::VideoWriter video(argv[3],CV_FOURCC('M','P','4','V'),30, Size(frame_width,frame_height));
  // Check if camera opened successfully
  if(!cap.isOpened()){
    std::cout << "Error opening video stream or file" << std::endl;
    return 1;
  }
  int count = 1;
  Timer time;
  double t;
  while(1){
    // Capture frame-by-frame
    cap >> frame;

    // If the frame is empty, break immediately
    if (frame.empty())
      break;
    time.reset();
    waypoints = detect_lane_lines(frame, result, lane_lines, dist, mtx, M, Minv,
    xm_per_pix, ym_per_pix);
    t = time.time();
    std::cout << "total time: " << t << " | fps: " << 1.0/t << std::endl;

    video.write(result);
    std::cout << "[" << count << "/" << length << "]\n";
    count += 1;
    cv::imshow("result", result);

    // Press ESC on keyboard to exit
    char c = static_cast<char>(cv::waitKey(25));
    if(c==27)
      break;
  }
  cap.release();
  video.release();
}

std::string trim(const std::string& str,
                 const std::string& whitespace)
{
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return "";
    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;
    return str.substr(strBegin, strRange);
}
