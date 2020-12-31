/**
 * @file limit_point.h
 */

#ifndef COMMON_LIMIT_POINT_H
#define COMMON_LIMIT_POINT_H

/**
 * @brief common limit for path and speed
 * x is independent variable
 * l is lower bound on point x
 * u is upper bound on point x
 */

class LimitPoint {
 public:
  LimitPoint() = default;

  // only use to second derivative
  LimitPoint(const double x, const double l, const double u) : x_(x), l_(l), u_(u) {}

  void setX(const double x) { x_ = x; }
  void setLowerBound(const double l) { l_ = l; }
  void setUpperBound(const double u) { u_ = u_; }

  double x() const { return x_; }
  double l() const { return l_; }
  double u() const { return u_; }

 private:
  double x_;
  double l_;
  double u_;
};

#endif // COMMON_LIMIT_POINT_H
