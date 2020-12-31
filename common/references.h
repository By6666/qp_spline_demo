/**
 * @file references.h
 */

#ifndef COMMON_REFERENCES_H_
#define COMMON_REFERENCES_H_

#include <utility>
#include <vector>

#include "reference_point.h"

namespace apollo {
namespace planning {

class References {
 public:
  References() = default;

  void AppendReference(const double x, const double ref);

  const std::vector<ReferencePoint>& reference_points() const;

  double GetReferenceByX(const double x) const;

  void Clear();

 private:
  std::vector<ReferencePoint> reference_points_;
};

}  // namespace planning
}  // namespace apollo

#endif  // COMMON_REFERENCES_H_
