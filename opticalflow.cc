/***
 * @file:  opticalflow.cc
 * @author: Dongying (yudong2817@sina.com)
 * @brief:
 * @version:  0.1
 * @date:  2021-04-21
 * @copyright: Copyright (c) 2021
 */
#include "opticalflow.h"

Opticalflow::Opticalflow(const cv::Mat &img) : img_(img.clone()) { init(); }

Opticalflow::~Opticalflow() {}

void Opticalflow::init() {
  assert(!img_.empty());

  opticalflow_frame_id_ = Opticalflow::total_opticalflow_frame_cnt_++;

  detector_->detect(img_, keypoints_);
}
// 匹配之后将筛选后两帧图像对应的Point输出
void Opticalflow::matchWithflow(const Opticalflow::Ptr frame,
                                std::vector<cv::Point2f> &point1,
                                std::vector<cv::Point2f> &point2,
                                std::vector<uchar> &status,
                                std::vector<float> &error) {
  std::vector<cv::Point2f> pt1, pt2;
  std::list<cv::Point2f> keypoints;
  for (const auto &kp : keypoints_) {
    pt1.push_back(kp.pt);
    keypoints.push_back(kp.pt);
  }
  std::cout << "[INFO]: The size of pt1 :  " << pt1.size() << std::endl;
  cv::calcOpticalFlowPyrLK(img_, frame->img_, pt1, pt2, status, error);

  int i = 0;
  for (auto iter = keypoints.begin(); iter != keypoints.end(); i++) {
    if (status[i] == 0) {
      iter = keypoints.erase(iter);
      continue;
    }
    point2.push_back(pt2[i]);
    iter++;
  }

  std::cout << "[INFO]: The size of point2 :  " << point2.size() << std::endl;
  // 该步操作可能存在问题，后续需要进行测试
  for (const auto &k : keypoints) {
    point1.push_back(k);
  }

  std::cout << "[INFO]: The size of point1 :  " << point1.size() << std::endl;
  cv::Mat image_test = frame->img_.clone();
  cv::Mat img2_CV = image_test.clone();
  // cv::cvtColor(image_test, img2_CV, CV_GRAY2BGR);
  for (int i = 0; i < pt2.size(); i++) {
    if (status[i]) {
      cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
    }
  }
  cv::imshow("Opticalflow_result", img2_CV);
}

cv::Mat Opticalflow::getPose() {
  std::unique_lock<std::mutex> lock(mutex_pose_);
  return Tcw_;
}

void Opticalflow::setPose(const cv::Mat &R, const cv::Mat &t) {
  std::unique_lock<std::mutex> lock(mutex_pose_);
  R.copyTo(Rcw_);
  t.copyTo(tcw_);
  Tcw_ = cv::Mat::zeros(4, 4, CV_64F);
  Rcw_.copyTo(Tcw_.rowRange(0, 3).colRange(0, 3));
  tcw_.copyTo(Tcw_.rowRange(0, 3).col(3));
}

size_t Opticalflow::getOpticalflowFrameId() const {
  return opticalflow_frame_id_;
}

size_t Opticalflow::total_opticalflow_frame_cnt_ = 0;

// keypoints, GFTT
// 默认使用的是harris角点
cv::Ptr<cv::GFTTDetector> Opticalflow::detector_ =
    cv::GFTTDetector::create(500, 0.01, 20);  // maximum 500 keypointss