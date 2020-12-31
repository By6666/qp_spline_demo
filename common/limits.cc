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
 * @file limits.cc
 **/

#include "limits.h"

#include <algorithm>
#include <glog/logging.h>

#include "log.h"

namespace apollo {
namespace planning {

void Limits::AppendLimit(const double x, const double l, const double u) {
  if (!limit_points_.empty()) {
    DCHECK_GE(x, limit_points_.back().x());
  }
  limit_points_.emplace_back(x, l, u);
}
const std::vector<LimitPoint>& Limits::limit_points() const { return limit_points_; }

std::pair<double, double> Limits::GetLimitByX(const double x) const {
  DCHECK_GE(limit_points_.size(), 2);
  DCHECK_GE(x, limit_points_.front().x());

  auto compare_x = [](const LimitPoint& limit_point, const double x) {
    return limit_point.x() < x;
  };

  auto it_lower = std::lower_bound(limit_points_.begin(), limit_points_.end(), x, compare_x);

  if (it_lower == limit_points_.end()) {
    return std::pair<double, double>{(it_lower - 1)->l(), (it_lower - 1)->u()};
  }
  return std::pair<double, double>{(it_lower)->l(), (it_lower)->u()};
}

void Limits::Clear() { limit_points_.clear(); }

}  // namespace planning
}  // namespace apollo
