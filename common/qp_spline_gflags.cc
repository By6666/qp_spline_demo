/**
 * @file : qp_spline_gflags.h
 **/

#include "qp_spline_gflags.h"

// qp spline speed
DEFINE_double(total_time, 8.0, "speed planning total time");
DEFINE_double(total_lenght, 250.0, "speed planning path total length");
DEFINE_uint32(number_of_anchor_t, 4, "number of t anchor points");
DEFINE_uint32(spline_order, 5, "set qp spline order");
DEFINE_double(trajectory_time_min_interval, 0.02,
              "(seconds) Trajectory time interval when publish. The is the min value.");

// weight
DEFINE_double(accel_kernel_weight, 1e3, "acc kernel weight");
DEFINE_double(jerk_kernel_weight, 1e5, "jerl kernel weight");
DEFINE_double(cruise_kernel_weight, 3.2, "cruise kernel weight");
DEFINE_double(speed_kernel_weight, 10.0, "speed kernel weight");
DEFINE_double(init_acc_target_weight, 5e4, "init point acc target weight");
DEFINE_double(regularization_weight, 0.1, "qp problem regularization weight");
