/***
 * @file:  map.cc
 * @author: Dongying (yudong2817@sina.com)
 * @brief:
 * @version:  0.1
 * @date:  2021-04-22
 * @copyright: Copyright (c) 2021
 */
#include "map.h"
#include "mappoint.h"

Map::Map() {}

Map::~Map() {}

bool Map::initialize(const Opticalflow::Ptr &opticalflow_frame1,
                     const Opticalflow::Ptr &opticalflow_frame2) {
  std::cout << "[INFO]: Trying to initialize a map. " << std::endl;
  assert(1);
  cv::Mat K =
      (cv::Mat_<double>(3, 3) << 2394.3172302171947, 0.0, 1242.8769964884666,
       0.0, 2648.8056802299297, 737.2368181585232, 0.0, 0.0, 1.0);

  std::vector<cv::Point2f> points1, points2;
  std::vector<uchar> status;
  sts::vector<float> error;
  // 这里得到的匹配点，其存储序列是互相对应的
  opticalflow_frame1->matchWithflow(opticalflow_frame2, points1, points2,
                                    status, error);

  std::vector<uchar> ransac_status;
  if (points1.size() == 0 || points2.size() == 0) return false;
  cv::Mat H = cv::findHomography(points1, points2, ransac_status, cv::RANSAC);

  std::cout << "[INFO]: The value of H :  " << H << std::endl;
  int h_inliers = std::accumulate(ransac_status.begin(), ransac_status.end(), 0,
                                  [](int c1, int c2) { return c1 + c2; });

  std::cout << "[INFO]: Find H inliers: " << h_inliers << std::endl;
  if (h_inliers == 0) return false;

  // 利用单应矩阵计算R、t，挑选出正确的R t,初始化地图点
  std::vector<cv::Mat> Rs, ts, normals;
  cv::decomposeHomographyMat(H, K, Rs, ts, normals);

  cv::Mat R_h, t_h;
  int inliers = 0;
  std::vector<uchar> mask;
  // 地图点先暂时存储
  std::vector<Mappoint::Ptr> mappoints;
  for (int i = 0; i < Rs.size(); ++i) {
    std::vector<uchar> mask_tmp = ransac_status;
    std::vector<Mappoint::Ptr> mps_tmp;
    int inliers_tmp = checkRtn(Rs[i], ts[i], normals[i], K, points1, points1,
                               mask_tmp, mps_tmp);
    if (inliers_tmp > inliers) {
      inliers = inliers_tmp;
      R_h = Rs[i];
      t_h = ts[i];
      mask = mask_tmp;
      mappoints = mps_tmp;
    }
  }

  if (inliers == 0) {
    return false;
  }
  if (inliers < x3D_inliers_threshold_) {
    std::cout << "[WARNING]: Too few mappoint iliers. " << std::endl;
  }

  std::cout << "[INFO]: Recover Rt, mappoint inliers: " << inliers << std::endl;
  std::cout << "[INFO]: R_h: " << std::endl << R_h << std::endl;
  std::cout << "[INFO]: t: " << t_h.t() << std::endl;

  cv::Mat Tcw = opticalflow_frame1->getPose();
  cv::Mat R1, t1;
  Tcw.rowRange(0, 3).colRange(0, 3).copyTo(R1);
  Tcw.rowRange(0, 3).col(3).copyTo(t1);
  opticalflow_frame1->setPose(R_h * R1, R_h * t1 + t_h);

  insertRecentOpticalflowFrame(opticalflow_frame1);
  insertRecentOpticalflowFrame(opticalflow_frame2);

  // 如何对光流特征点确定其再图像帧中id及唯一对应的地图点
  int mp_idx = 0;
  for (int i = 0; i < mask.size(); ++i) {
    if (!mask[i]) {
      continue;
    }
  }

  for (const auto &mp : mappoints) {
    insertMapPoint(mp);
  }
}

void Map::insertRecentOpticalflowFrame(
    const Opticalflow::Ptr &opticalflow_frame) {
  std::unique_lock<std::mutex> lock(mutex_recent_opticalflow_frames_);
  recent_opticalflow_frames_[opticalflow_frame->getOpticalflowFrameId()] =
      opticalflow_frame;
  while (recent_opticalflow_frames_.size() > max_recent_opticalflow_frames_) {
    recent_opticalflow_frames_.erase(recent_opticalflow_frames_.begin());
  }
}

int Map::checkRtn(const cv::Mat &R, const cv::Mat &t, const cv::Mat &n,
                  const cv::Mat &K, std::vector<cv::Point2f> points1,
                  std::vector<cv::Point2f> points2, std::vector<uchar> &mask,
                  std::vector<Mappoint::Ptr> &mappoints) {
  // 因相机是俯视地面，法向量必须大致验z轴的（z轴分量绝对值最大）
  if (std::fabs(n.at<double>(2, 0)) <= std::fabs(n.at<double>(0, 0)) ||
      std::fabs(n.at<double>(2, 0)) <= std::fabs(n.at<double>(1, 0))) {
  }

  // 计算地图点，地图点在两个坐标系下的z值必须都为正
  // 在相机位置1参考系中，两相机光心
  cv::Mat O1 = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat O2 = -R.t() * t;

  // 相机1的重投影矩阵 K[I|0]
  cv::Mat P1(3, 4, CV_64F, cv::Scalar(0));
  K.copyTo(P1.rowRange(0, 3).colRange(0, 3));

  // 相机2的重投影矩阵 K[R|t]
  cv::Mat P2(3, 4, CV_64F);
  R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
  t.copyTo(P2.rowRange(0, 3).col(0));
  P2 = K * P2;

  std::vector<double> e1s, e2s, cos_thetas;

  int x3D_cnt = 0;
  mappoints.clear();
  for (int i = 0; i < mask.size(); i++) {
    // 如果不是计算H矩阵的内点，则不参与计算
    if (!mask[i]) {
      continue;
    }
    mask[i] = '\0';

    // 空间点在相机位置1参考系中的坐标
    auto pt1 = points1[i];
    auto pt2 = points2[j];
    cv::Mat x3D_C1;

    // 再有了地图点及两个重投影矩阵之后可，进行三角化
    triangulate(pt1, pt2, P1, P2, x3D_C1);

    // 空间点在相机位置2参考系中的坐标
    cv::Mat x3D_C2 = R * x3D_C1 + t;

    // 判断是否是有效的地图点
    // isfinite()用以检测一个值是否为有限值，若为有限值则返回1，否则返回0
    // 仅有效地图点才可以插入到地图点中
    if (!(std::isfinite(x3D_C1.at<double>(0)) &&
          std::isfinite(x3D_C1.at<double>(1)) &&
          std::isfinite(x3D_C1.at<double>(2)) && x3D_C1.at<double>(2) > 0 &&
          x3D_C2.at<double>(2) > 0)) {
      std::cout << "[WARNING]: invalid x3D " << i << ": in C1,  " << x3D_C1.t()
                << " in C2,  " << x3D_C2.t() << std::endl;
      continue;
    }
    cv::Mat N1 = x3D_C1 - O1;
    cv::Mat N2 = x3D_C2 - O2;
    double cos_theta = N1.dot(N1) / (cv::norm(N1) * cv::norm(N2));
    cos_thetas.emplace_back(cos_theta);

    // 计算空间点的投影误差，误差平方值应当在允许范围内
    auto proj_pt1 = project(x3D_C1, K);
    double e1 = squareUvError(pt1 - proj_pt1);
    auto proj_pt2 = project(x3D_C2, K);
    double e2 = squareUvError(pt2 - proj_pt2);

    e1s.emplace_back(e1);
    e2s.emplace_back(e2);

    if (e1 > square_projection_error_threshold_ ||
        e2 > square_projection_error_threshold_) {
      continue;
    }

    mappoints.emplace_back(x3D_C1.at<double>(0), x3D_C1.at<double>(1),
                           x3D_C1.at<double>(2));
    x3D_cnt++;
    mask[j] = '\1';
  }
  return x3D_cnt；
}

void Map::triangulate(const cv::Point2f &pt1, const cv::Point2f &pt2,
                      const cv::Mat &P1, cv::Mat &P2, cv::Mat &x3D) {
  // ORB-SLAM2中也是使用这种三角化方法
  cv::Mat A(4, 4, CV_64F);

  A.row(0) = pt1.x * P1.row(2) - P1.row(0);
  A.row(1) = pt1.y * P1.row(2) - P1.row(1);
  A.row(2) = pt2.x * P2.row(2) - P2.row(0);
  A.row(3) = pt2.y * P2.row(2) - P2.row(1);

  cv::Mat u, w, vt;
  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  x3D = vt.row(3).t();
  x3D = x3D.rowRange(0, 3) / x3D.at<double>(3);
}

cv::Point2f Map::project(const cv::Mat &x3D, const cv::Mat K) {
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1.1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1.2);
  double X = x3D.at<double>(0);
  double Y = x3D.at<double>(1);
  double Z = x3D.at<double>(2);
  double x = fx * X / Z + cx;
  double y = fy * y / Z + cy;
  return cv::Point2f(x, y);
}

float Map::squareUvError(const cv::Point2f &uv_error) {
  return uv_error.x * uv_error.x + uv_error.y * uv_error.y;
}

void Map::insertMapPoint(const Mappoint::Ptr &mp) {
  std::unique_lock<std::mutex> lock(mutex_mappoints_);
  mappoints_[mp->getId()] = mp;
}