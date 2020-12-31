/**
 * @file point.h
 **/

#ifndef COMMON_POINT_H
#define COMMON_POINT_H

/**
 * @brief common point for path and speed
 * x is independent variable
 * y is f(x)
 * y_dot is f'(x)
 * y_ddot id f''(x)
 * y_dddot is f'''(x)
 **/

class CommonPoint {
 public:
  CommonPoint() = default;

  // only use to second derivative
  CommonPoint(const double x, const double y, const double y_dot, const double y_ddot)
      : x_(x), y_(y), y_dot_(y_dot), y_ddot_(y_ddot), y_dddot_(0.0) {}

  CommonPoint(const double x, const double y, const double y_dot, const double y_ddot,
              const double y_dddot)
      : x_(x), y_(y), y_dot_(y_dot), y_ddot_(y_ddot), y_dddot_(y_dddot) {}

  void setX(const double x) { x_ = x; }
  void setY(const double y) { y_ = y; }
  void setYdot(const double ydot) { y_dot_ = ydot; }
  void setYddot(const double yddot) { y_ddot_ = yddot; }
  void setYdddot(const double ydddot) { y_dddot_ = ydddot; }

  double x() const { return x_; }
  double y() const { return y_; }
  double y_dot() const { return y_dot_; }
  double y_ddot() const { return y_ddot_; }
  double y_dddot() const { return y_dddot_; }

 private:
  double x_;
  double y_;
  double y_dot_;
  double y_ddot_;
  double y_dddot_;
};

#endif //COMMON_POINT_H
