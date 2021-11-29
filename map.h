/***
 * @file:  map.h
 * @author: Dongying (yudong2817@sina.com)
 * @brief:
 * @version:  0.1
 * @date:  2021-04-22
 * @copyright: Copyright (c) 2021
 */

#pragma once

#include <mutex>
#include <numeric>  // for std::accumulate
#include <vector>
#include "opticalflow.h"
#include "mappoint.h"

class Map {
 public:
  using Ptr = std::shared_ptr<Map>;

  Map();

  ~Map();

  bool initialize(const Opticalflow::Ptr &opticalflow_frame1,
                  const Opticalflow::Ptr &opticalflow_frame2);

  void insertRecentOpticalflowFrame(const Opticalflow::Ptr &opticalflow_frame);

 private:
  int checkRtn(const cv::Mat &R, const cv::Mat &t, const cv::Mat &n,
               const cv::Mat &K, std::vector<cv::Point2f> points1,
               std::vector<cv::Point2f> points2, std::vector<uchar> &mask,
               std::vector<cv::Point3f> &mappoints);

  void triangulate(const cv::Point2f &pt1, const cv::Point2f &pt2,
                   const cv::Mat &P1, cv::Mat &P2, cv::Mat &x3D);

  cv::Point2f project(const cv::Mat &x3D, const cv::Mat K);

  float squareUvError(const cv::Point2f &uv_error);

  void insertMapPoint(const Mappoint::Ptr &mp);
  // 初始化时，重投影地图点时，允许的误差最大值的平方(ORBSLAM2值为4)
  double square_projection_error_threshold_ = 10;
  // 初始化时，至少应当有100个地图点
  int x3D_inliers_threshold_ = 50;

 private:
  std::mutex mutex_recent_opticalflow_frames_;
  std::map<size_t, Opticalflow::Ptr> recent_opticalflow_frames_;
  int max_recent_opticalflow_frames_ = 50;

  std::mutex mutex_mappoints_;
  std::map<size_t,Mappoint::Ptr> mappoints_;
};