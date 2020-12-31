/**
 * for qp spline config and param
 **/

/**
 * @file : qp_spline_gflags.h
 **/

#ifndef COMMON_QP_SPLINE_GFLAGS_H_
#define COMMON_QP_SPLINE_GFLAGS_H_

#include <gflags/gflags.h>

// qp spline speed
DECLARE_double(total_time);
DECLARE_double(total_length);
DECLARE_uint32(spline_order);
DECLARE_uint32(number_of_anchor_t);
DECLARE_double(trajectory_time_min_interval);

DECLARE_double(accel_kernel_weight);
DECLARE_double(jerk_kernel_weight);
DECLARE_double(cruise_kernel_weight);
DECLARE_double(speed_kernel_weight);
DECLARE_double(init_acc_target_weight);
DECLARE_double(regularization_weight);

#endif  // COMMON_QP_SPLINE_GFLAGS_H_
