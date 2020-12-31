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
 * @file limits.h
 */

#ifndef COMMON_LIMITS_H_
#define COMMON_LIMITS_H_

#include <utility>
#include <vector>

#include "limit_point.h"

namespace apollo {
namespace planning {

class Limits {
 public:
  Limits() = default;

  void AppendLimit(const double x, const double l, const double u);

  const std::vector<LimitPoint>& limit_points() const;

  std::pair<double, double> GetLimitByX(const double x) const;

  void Clear();

 private:
  std::vector<LimitPoint> limit_points_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_SPEED_LIMIT_H_
