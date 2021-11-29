/***
 * @file:  opticalflow.h
 * @author: Dongying (yudong2817@sina.com)
 * @brief:
 * @version:  0.1
 * @date:  2020-04-21
 * @copyright: Copyright (c) 2021
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class Opticalflow {
 public:
  using Ptr = std::shared_ptr<Opticalflow>;

  Opticalflow(const cv::Mat &img);

  ~Opticalflow();

  void matchWithflow(const Opticalflow::Ptr frame,
                     std::vector<cv::Point2f> &pt1,
                     std::vector<cv::Point2f> &pt2, std::vector<uchar> &status,
                     std::vector<float> &error);

  static cv::Ptr<cv::GFTTDetector> detector_;

  cv::Mat getPose();

  void setPose(const cv::Mat &R, const cv::Mat &t);

  size_t getOpticalflowFrameId() const;

 private:
  void init();

 private:
  cv::Mat img_;
  std::vector<cv::KeyPoint> keypoints_;

  std::mutex mutex_pose_;
  cv::Mat Tcw_ = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat Rcw_ = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat tcw_ = cv::Mat::zeros(3, 1, CV_64F);

  size_t opticalflow_frame_id_;
  static size_t total_opticalflow_frame_cnt_;
};
