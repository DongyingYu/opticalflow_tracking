/***
 * @file:  mappoint.cc
 * @author: Dongying (yudong2817@sina.com)
 * @brief:
 * @version:  0.1
 * @date:  2021.04.23
 * @copyright: Copyright (c) 2021
 */
#include "mappoint.h"

Mappoint::Mappoint(float x, float y, float z) : x_(x), y_(y), z_(z) {
  mp_id_ = total_mp_cnt_++;
}

Mappoint::~Mappoint() {}

size_t Mappoint::getId() const { return mp_id_; };

size_t Mappoint::total_mp_cnt_ = 0;