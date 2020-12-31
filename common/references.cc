/**
 * @file references.cc
 **/

#include "references.h"

#include <algorithm>
#include <glog/logging.h>

#include "log.h"

namespace apollo {
namespace planning {

void References::AppendReference(const double x, const double ref) {
  if (!reference_points_.empty()) {
    DCHECK_GE(x, reference_points_.back().x());
  }
  reference_points_.emplace_back(x, ref);
}
const std::vector<ReferencePoint>& References::reference_points() const {
  return reference_points_;
}

double References::GetReferenceByX(const double x) const {
  DCHECK_GE(reference_points_.size(), 2);
  DCHECK_GE(x, reference_points_.front().x());

  auto compare_x = [](const ReferencePoint& ref_point, const double x) {
    return ref_point.x() < x;
  };

  auto it_lower =
      std::lower_bound(reference_points_.begin(), reference_points_.end(), x, compare_x);

  if (it_lower == reference_points_.end()) {
    return (it_lower - 1)->ref();
  }
  return it_lower->ref();
}

void References::Clear() { reference_points_.clear(); }

}  // namespace planning
}  // namespace apollo
