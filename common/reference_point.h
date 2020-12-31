/**
 * @file reference_point.h
 */

#ifndef COMMON_REFERENCE_POINT_H
#define COMMON_REFERENCE_POINT_H

/**
 * @brief common ref for path and speed
 * x is independent variable
 * ref is reference value on point x
 */

class ReferencePoint {
 public:
  ReferencePoint() = default;

  ReferencePoint(const double x, const double ref) : x_(x), ref_(ref) {}

  void setX(const double x) { x_ = x; }
  void setReference(const double ref) { ref_ = ref; }

  double x() const { return x_; }
  double ref() const { return ref_; }

 private:
  double x_;
  double ref_;
};

#endif // COMMON_REFERENCE_POINT_H
