/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file speed_data.cc
 **/

#include "speed_data.h"

#include <algorithm>
#include <utility>
#include <glog/logging.h>

#include "common/linear_interpolation.h"

namespace apollo {
namespace planning {

SpeedData::SpeedData(std::vector<CommonPoint> speed_points)
    : speed_vector_(std::move(speed_points)) {}

void SpeedData::AppendSpeedPoint(const double t, const double s, const double v, const double a,
                                 const double da) {
  if (!speed_vector_.empty()) {
    CHECK(speed_vector_.back().x() < t);
  }
  speed_vector_.emplace_back(t, s, v, a, da);
}

const std::vector<CommonPoint>& SpeedData::speed_vector() const { return speed_vector_; }

void SpeedData::set_speed_vector(std::vector<CommonPoint> speed_points) {
  speed_vector_ = std::move(speed_points);
}

bool SpeedData::EvaluateByTime(const double t, CommonPoint* const speed_point) const {
  if (speed_vector_.size() < 2) {
    return false;
  }
  if (!(speed_vector_.front().x() < t + 1.0e-6 && t - 1.0e-6 < speed_vector_.back().x())) {
    return false;
  }

  auto comp = [](const CommonPoint& sp, const double t) { return sp.x() < t; };

  auto it_lower = std::lower_bound(speed_vector_.begin(), speed_vector_.end(), t, comp);
  if (it_lower == speed_vector_.end()) {
    *speed_point = speed_vector_.back();
  } else if (it_lower == speed_vector_.begin()) {
    *speed_point = speed_vector_.front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.x();
    double t1 = p1.x();

    double s = common::math::lerp(p0.y(), t0, p1.y(), t1, t);
    double v = common::math::lerp(p0.y_dot(), t0, p1.y_dot(), t1, t);
    double a = common::math::lerp(p0.y_ddot(), t0, p1.y_ddot(), t1, t);
    double j = common::math::lerp(p0.y_dddot(), t0, p1.y_dddot(), t1, t);

    *speed_point = CommonPoint(t, s, v, a, j);
  }
  return true;
}

double SpeedData::TotalTime() const {
  if (speed_vector_.empty()) {
    return 0.0;
  }
  return speed_vector_.back().x() - speed_vector_.front().x();
}

void SpeedData::Clear() { speed_vector_.clear(); }

// std::string SpeedData::DebugString() const {
//   const auto limit =
//       std::min(speed_vector_.size(),
//                static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
//   return apollo::common::util::StrCat(
//       "[\n",
//       apollo::common::util::PrintDebugStringIter(
//           speed_vector_.begin(), speed_vector_.begin() + limit, ",\n"),
//       "]\n");
// }

}  // namespace planning
}  // namespace apollo
