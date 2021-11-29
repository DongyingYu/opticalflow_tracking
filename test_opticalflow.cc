/***
 * @file: test_opticalflow.cc
 * @author: Dongying (yudong2817@sina.com)
 * @brief:
 * @version: 0.1
 * @date:  Do not edit
 * @copyright: Copyright (c) 2021
 */

#include <chrono>
#include "opticalflow.h"

int main(int argc, char **argv) {
  std::string video_file = "/home/ipsg/dataset_temp/78_cut.mp4";
  int skip_frames = 0;

  if (argc != 3) {
    std::cout << "[INFO]: Usage: ./test_SSIM  /video_file  video_file number "
              << std::endl;
    return 1;
  }
  video_file = argv[1];
  skip_frames = atoi(argv[2]);
  std::cout << "[INFO]: The video file is :  " << video_file << std::endl;
  std::cout << "[INFO]: The number of skip frames:   " << skip_frames << std::endl;
  // 选择以灰度格式加载，显示时再转回去
  // cv::Mat img1 = cv::imread(argv[1], 0);
  // cv::Mat img2 = cv::imread(argv[2], 0);

  bool capture_status = true;
  cv::VideoCapture capture;
  {
    capture.open(video_file);
    if (!capture.isOpened()) capture_status = false;
  }

  while (!capture_status) {
    std::cout << "[WARNING]: Could not open the input video: " << video_file
              << std::endl;
    capture.open(video_file);
    std::cout << "[INFO]: Reconnect to the video: " << video_file << std::endl;
    if (capture.isOpened()) capture_status = true;
    usleep(500000);
  }

  Opticalflow::Ptr current_opticalflow;
  Opticalflow::Ptr last_opticalflow;

  cv::Mat img;
  while (skip_frames-- > 0) {
    capture >> img;
  }

  int cnt = 0;
  for (int cnt = 0;; ++cnt) {
    capture >> img;
    cv::resize(img, img, {0, 0}, 0.5, 0.5);
    Opticalflow::Ptr opticalflow = std::make_shared<Opticalflow>(img);

    {
      last_opticalflow = current_opticalflow;
      current_opticalflow = opticalflow;
    }
    if (!last_opticalflow) {
      continue;
    }

    std::vector<cv::Point2f> point1;
    std::vector<cv::Point2f> point2;
    std::vector<uchar> status;
    std::vector<float> error;

    last_opticalflow->matchWithflow(current_opticalflow, point1, point2, status,
                                    error);
    std::cout << "[INFO]: The cnt of frame:  " << cnt << std::endl;
    cv::waitKey();
  }

  // Opticalflow::Ptr opticalflow1 = std::make_shared<Opticalflow>(img1);
  // Opticalflow::Ptr opticalflow2 = std::make_shared<Opticalflow>(img2);

  // std::vector<cv::Point2f> point1;
  // std::vector<cv::Point2f> point2;
  // std::vector<uchar> status;
  // std::vector<float> error;

  // opticalflow1->matchWithflow(opticalflow2, point1, point2, status, error);

  // cv::waitKey();

  return 0;
}