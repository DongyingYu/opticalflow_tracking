/***
 * @file:  tracking.h
 * @author: Dongying (yudong2817@sina.com)
 * @brief:
 * @version:  0.1
 * @date:  2021.04.23
 * @copyright: Copyright (c) 2021
 */
#pragma once
#include <iostream>
#include <memory>
#include <vector>
#include "opticalflow.h"

class Tracking {
 public:
  using Ptr = std::shared_ptr<Tracking>;

  Tracking();
  ~Tracking();

 private:
};