/***
 * @file:  mappoint.h
 * @author: Dongying (yudong2817@sina.com)
 * @brief:
 * @version:  0.1
 * @date:  2021.04.23
 * @copyright: Copyright (c) 2021
 */

#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

class Mappoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::shared_ptr<Mappoint>;

  Mappoint(float x, float y, float z);

  ~Mappoint();

  size_t getId() const;

 private:
  std::mutex mutex_;
  float x_;
  float y_;
  float z_；

      size_t mp_id_;
  static size_t total_mp_cnt_;
};