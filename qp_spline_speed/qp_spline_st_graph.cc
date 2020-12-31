/**
 * modify only base on apollo auto
 **/

/**
 * @file
 **/

#include "qp_spline_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "common/log.h"
#include "common/qp_spline_gflags.h"

namespace apollo {
namespace planning {

QpSplineStGraph::QpSplineStGraph() { Init(); }

void QpSplineStGraph::Init() {
  t_knots_resolution_ = FLAGS_total_time / FLAGS_number_of_anchor_t;

  // init knots
  double curr_t = 0.0;
  uint32_t num_spline = FLAGS_number_of_anchor_t - 1;
  for (uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.push_back(curr_t);
    curr_t += t_knots_resolution_;
  }

  uint32_t num_evaluated_t = 10 * num_spline + 1;

  // init evaluated t positions
  curr_t = 0;
  t_evaluated_resolution_ = FLAGS_total_time / (num_evaluated_t - 1);
  for (uint32_t i = 0; i < num_evaluated_t; ++i) {
    t_evaluated_.push_back(curr_t);
    curr_t += t_evaluated_resolution_;
  }
}

void QpSplineStGraph::SetCondition(const CommonPoint& init_point, const Limits& s_limits,
                                   const Limits& v_limits,
                                   const std::pair<double, double>& a_limits,
                                   const References& s_refs, const References& v_refs) {
  // initial status
  init_point_ = init_point;

  // limits
  s_limits_ = s_limits;
  v_limits_ = v_limits;
  a_limits_ = a_limits;

  // references
  s_refs_ = s_refs;
  v_refs_ = v_refs;
}

bool QpSplineStGraph::Solve(SpeedData* const speed_data) {
  // Reset spline generator
  spline_generator_->Reset(t_knots_, FLAGS_spline_order);

  // Add constraint
  if (!AddConstraint()) {
    AERROR << "Add constraint failed!";
    return false;
  }

  // Add kernel
  if (!AddKernel()) {
    AERROR << "Add kernel failed!";
    return false;
  }

  // Solve problem
  if (!Solve()) {
    AERROR << "Solve qp problem failed!";
    return false;
  }

  // extract output
  speed_data->Clear();
  const Spline1d& spline = spline_generator_->spline();

  const double t_output_resolution = FLAGS_trajectory_time_min_interval;
  double time = 0.0;
  while (time < FLAGS_total_time + t_output_resolution) {
    double s = spline(time);
    double v = std::max(0.0, spline.Derivative(time));
    double a = spline.SecondOrderDerivative(time);
    double da = spline.ThirdOrderDerivative(time);
    speed_data->AppendSpeedPoint(s, time, v, a, da);
    time += t_output_resolution;
  }

  AERROR << "Speed qp spline successful!!";

  return true;
}

// bool QpSplineStGraph::Search(const StGraphData& st_graph_data,
//                              const std::pair<double, double>& accel_bound,
//                              const SpeedData& reference_speed_data, SpeedData* const speed_data)
//                              {
//   cruise_.clear();
//   speed_ref_ = st_graph_data.cruise_speed();
//   reference_dp_speed_points_ = speed_data->speed_vector();

//   init_point_ = st_graph_data.init_point();
//   ADEBUG << "init point:" << init_point_.DebugString();

//   // reset spline generator
//   spline_generator_->Reset(t_knots_, FLAGS_spline_order);

//   if (!AddConstraint(st_graph_data.init_point(), st_graph_data.speed_limit(),
//                      st_graph_data.st_boundaries(), accel_bound)
//            .ok()) {
//     const std::string msg = "Add constraint failed!";
//     AERROR << msg;
//     return bool(ErrorCode::PLANNING_ERROR, msg);
//   }

//   if (!AddKernel(st_graph_data.init_point(), st_graph_data.st_boundaries(),
//                  st_graph_data.speed_ref(), st_graph_data.speed_limit())
//            .ok()) {
//     const std::string msg = "Add kernel failed!";
//     AERROR << msg;
//     return bool(ErrorCode::PLANNING_ERROR, msg);
//   }

//   if (!Solve().ok()) {
//     const std::string msg = "Solve qp problem failed!";
//     AERROR << msg;
//     return bool(ErrorCode::PLANNING_ERROR, msg);
//   }

//   // extract output
//   speed_data->Clear();
//   const Spline1d& spline = spline_generator_->spline();

//   const double t_output_resolution = FLAGS_trajectory_time_min_interval;
//   double time = 0.0;
//   while (time < FLAGS_total_time + t_output_resolution) {
//     double s = spline(time);
//     double v = std::max(0.0, spline.Derivative(time));
//     double a = spline.SecondOrderDerivative(time);
//     double da = spline.ThirdOrderDerivative(time);
//     speed_data->AppendSpeedPoint(s, time, v, a, da);
//     time += t_output_resolution;
//   }

//   return bool ::OK();
// }

bool QpSplineStGraph::AddConstraint() {
  Spline1dConstraint* constraint = spline_generator_->mutable_spline_constraint();

  // Initial point s constraint
  if (!constraint->AddPointConstraint(0.0, 0.0)) {
    AERROR << "Add st start point constraint failed";
    return false;
  }

  // Initial point s_dot(v) constraint
  if (!constraint->AddPointDerivativeConstraint(0.0, init_point_.y_dot())) {
    AERROR << "Add st start point velocity constraint failed!";
    return false;
  }

  // monotone constraint
  if (!constraint->AddMonotoneInequalityConstraint(t_evaluated_)) {
    AERROR << "Add monotone inequality constraint failed!";
    return false;
  }

  // smoothness constraint
  if (!constraint->AddThirdDerivativeSmoothConstraint()) {
    AERROR << "Add smoothness joint constraint failed!";
    return false;
  }

  // boundary constraint
  std::vector<double> s_lower_bound;
  std::vector<double> s_upper_bound;
  CommonGetConstraintForSolve(s_limits_, &s_lower_bound, &s_upper_bound);

  if (!constraint->AddBoundary(t_evaluated_, s_lower_bound, s_upper_bound)) {
    AERROR << "Fail to apply distance constraints.";
    return false;
  }

  // speed constraint
  std::vector<double> speed_lower_bound;
  std::vector<double> speed_upper_bound;
  CommonGetConstraintForSolve(v_limits_, &speed_lower_bound, &speed_upper_bound);

  if (!constraint->AddDerivativeBoundary(t_evaluated_, speed_lower_bound, speed_upper_bound)) {
    AERROR << "Fail to apply speed constraints.";
    return false;
  }

  // acceleration constraint
  std::vector<double> accel_lower_bound(t_evaluated_.size(), a_limits_.first);
  std::vector<double> accel_upper_bound(t_evaluated_.size(), a_limits_.second);

  if (!constraint->AddSecondDerivativeBoundary(t_evaluated_, accel_lower_bound,
                                               accel_upper_bound)) {
    AERROR << "Fail to apply acceleration constraints.";
    return false;
  }

  AERROR << "Add constraint sucessful!";
  return true;
}

void QpSplineStGraph::CommonGetConstraintForSolve(const Limits& limits,
                                                  std::vector<double>* const lower_bounds,
                                                  std::vector<double>* const upper_bounds) {
  CHECK_NOTNULL(lower_bounds);
  CHECK_NOTNULL(upper_bounds);

  for (const double curr_t : t_evaluated_) {
    auto limit = limits.GetLimitByX(curr_t);
    lower_bounds->push_back(limit.first);
    upper_bounds->push_back(limit.second);
    ADEBUG << "Add constraint by time: [t, l, u] => [" << curr_t << ", " << limit.first << ", "
           << limit.second << "]";
  }

  CHECK_EQ(t_evaluated_.size(), lower_bounds->size());
  CHECK_EQ(t_evaluated_.size(), upper_bounds->size());
}

void QpSplineStGraph::CommonGetReferenceForSolve(const References& refs,
                                                 std::vector<double>* const x_coord,
                                                 std::vector<double>* const x_refs) {
  CHECK_NOTNULL(x_coord);
  CHECK_NOTNULL(x_refs);

  for (const auto& ref : s_refs_.reference_points()) {
    x_coord->push_back(ref.x());
    x_refs->push_back(ref.ref());
    ADEBUG << "Add reference by time: [t, ref] => [" << ref.x() << ", " << ref.ref() << "]";
  }

  CHECK_EQ(x_coord->size(), x_refs->size());
}

bool QpSplineStGraph::AddKernel() {
  Spline1dKernel* spline_kernel = spline_generator_->mutable_spline_kernel();

  if (FLAGS_accel_kernel_weight > 0) {
    spline_kernel->AddSecondOrderDerivativeMatrix(FLAGS_accel_kernel_weight);
  }

  if (FLAGS_jerk_kernel_weight > 0) {
    spline_kernel->AddThirdOrderDerivativeMatrix(FLAGS_jerk_kernel_weight);
  }

  if (FLAGS_cruise_kernel_weight > 0 && s_refs_.reference_points().size() > 0) {
    if (!AddCruiseReferenceLineKernel(FLAGS_cruise_kernel_weight)) {
      AERROR << "Add cruise reference failed!";
      return false;
    }
  }

  if (FLAGS_speed_kernel_weight > 0 && v_refs_.reference_points().size() > 0) {
    if (!AddSpeedRefKernel(FLAGS_speed_kernel_weight)) {
      AERROR << "Add speed reference failed!";
      return false;
    }
  }

  if (spline_kernel->AddReferenceKernelToMainKernel() == false) {
    AERROR << "Add reference kernel to main kernel failed!";
    return false;
  }

  // init point a continuous kernel
  (*spline_kernel->mutable_kernel_matrix())(2, 2) += 2.0 * 4.0 * FLAGS_init_acc_target_weight;
  (*spline_kernel->mutable_offset())(2, 0) +=
      -4.0 * init_point_.y_ddot() * FLAGS_init_acc_target_weight;

  spline_kernel->AddRegularization(FLAGS_regularization_weight);

  return true;
}

bool QpSplineStGraph::Solve() { return spline_generator_->Solve(); }

bool QpSplineStGraph::AddCruiseReferenceLineKernel(const double weight) {
  auto* spline_kernel = spline_generator_->mutable_spline_kernel();

  std::vector<double> t_coord;
  std::vector<double> s_refs;

  CommonGetReferenceForSolve(s_refs_, &t_coord, &s_refs);

  spline_kernel->AddReferenceLineKernelMatrix(t_coord, s_refs,
                                              weight * FLAGS_total_time / t_evaluated_.size());

  return true;
}

bool QpSplineStGraph::AddSpeedRefKernel(const double weight) {
  auto* spline_kernel = spline_generator_->mutable_spline_kernel();
  // // double dist_ref = FLAGS_total_length;
  // std::vector<SpeedData> speed_ref_with_time;
  // std::vector<double> time;
  // std::vector<double> speed;
  // for (const auto& speed : speed_ref) {
  //   speed_ref_with_time.emplace_back(SpeedData(EstimateSpeedRefByInitSpeed(init_point, speed)));
  // }
  // LinearFillingInsideSpeedRef(speed_ref_with_time, &time, &speed);
  // DCHECK_EQ(time.size(), speed.size());

  std::vector<double> t_coord;
  std::vector<double> v_refs;

  CommonGetReferenceForSolve(v_refs_, &t_coord, &v_refs);

  spline_kernel->AddReferenceSpeedKernelMatrix(t_coord, v_refs,
                                               weight * FLAGS_total_time / t_coord.size());

  return true;
}

// bool QpSplineStGraph::AddDpStReferenceKernel(const double weight) const {
//   std::vector<double> t_pos;
//   std::vector<double> s_pos;
//   for (auto point : reference_dp_speed_points_) {
//     t_pos.push_back(point.t());
//     s_pos.push_back(point.s());
//   }
//   auto* spline_kernel = spline_generator_->mutable_spline_kernel();
//   if (!t_pos.empty()) {
//     spline_kernel->AddReferenceLineKernelMatrix(t_pos, s_pos,
//                                                 weight * FLAGS_total_time / t_pos.size());
//   }
//   return true;
// }

// bool QpSplineStGraph::GetSConstraintByTime(const double time double* const s_upper_bound,
//                                            double* const s_lower_bound) const {
//   *s_upper_bound = FLAGS_total_length;

//   for (const StBoundary* boundary : boundaries) {
//     double s_upper = 0.0;
//     double s_lower = 0.0;

//     if (!boundary->GetUnblockSRange(time, &s_upper, &s_lower)) {
//       continue;
//     }

//     if (boundary->boundary_type() == StBoundary::BoundaryType::STOP ||
//         boundary->boundary_type() == StBoundary::BoundaryType::FOLLOW ||
//         boundary->boundary_type() == StBoundary::BoundaryType::YIELD) {
//       *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
//     } else if (boundary->boundary_type() == StBoundary::BoundaryType::OVERTAKE) {
//       *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
//     } else {
//       AWARN << "Unhandled boundary type: " << StBoundary::TypeName(boundary->boundary_type());
//     }
//   }

//   return bool ::OK();
// }

// const SpeedData QpSplineStGraph::GetHistorySpeed() const {
//   const auto* last_frame = FrameHistory::instance()->Latest();
//   if (!last_frame) {
//     AWARN << "last frame is empty";
//     return SpeedData();
//   }
//   const ReferenceLineInfo* last_reference_line_info = last_frame->DriveReferenceLineInfo();
//   if (!last_reference_line_info) {
//     ADEBUG << "last reference line info is empty";
//     return SpeedData();
//   }
//   return last_reference_line_info->speed_data();
// }

// std::vector<common::SpeedPoint> QpSplineStGraph::EstimateSpeedRefByInitSpeed(
//     const common::TrajectoryPoint& init_point, const SpeedData& speed_ref) const {
//   std::vector<common::SpeedPoint> speed_ref_with_time = speed_ref.speed_vector();
//   for (size_t i = 0; i < speed_ref_with_time.size(); i++) {
//     speed_ref_with_time.at(i).set_t(speed_ref_with_time.at(i).s() / init_point.v());
//   }
//   return speed_ref_with_time;
// }

// void QpSplineStGraph::LinearFillingInsideSpeedRef(const std::vector<SpeedData>& speed_ref,
//                                                   std::vector<double>* time,
//                                                   std::vector<double>* v_ref) const {
//   for (uint32_t i = 0; i < t_evaluated_.size(); ++i) {
//     double min_v = std::numeric_limits<double>::infinity();
//     for (const auto& speed : speed_ref) {
//       if (speed.speed_vector().size() == 0) {
//         continue;
//       }
//       if (speed.speed_vector().size() == 1) {
//         if (i == t_evaluated_.size() - 1) {
//           continue;
//         }
//         if (t_evaluated_[i] < speed.speed_vector()[0].t() &&
//             t_evaluated_[i + 1] > speed.speed_vector()[0].t()) {
//           min_v = std::fmin(min_v, speed.speed_vector()[0].v());
//         }
//         continue;
//       }
//       for (uint32_t j = 0; j < speed.speed_vector().size() - 1; ++j) {
//         if (t_evaluated_[i] > speed.speed_vector()[j].t() &&
//             t_evaluated_[i] < speed.speed_vector()[j + 1].t()) {
//           common::SpeedPoint ref_p;
//           if (!speed.EvaluateByTime(t_evaluated_[i], &ref_p)) {
//             AERROR << "Failed to get target v when t = " << t_evaluated_[i];
//             continue;
//           }
//           min_v = std::fmin(ref_p.v(), min_v);
//         }
//       }
//     }
//     if (min_v < cruise_speed_[i]) {
//       time->emplace_back(t_evaluated_[i]);
//       v_ref->emplace_back(min_v);
//     }
//   }
// }

// bool QpSplineStGraph::EstimateSpeedUpperBound(const common::TrajectoryPoint& init_point,
//                                               const SpeedLimit& speed_limit,
//                                               std::vector<double>* speed_upper_bound) const {
//   DCHECK(speed_upper_bound != nullptr) << "speed_upper_bound cannot be null";

//   speed_upper_bound->clear();

//   // use v to estimate position: not accurate, but feasible in cyclic
//   // processing. We can do the following process multiple times and use
//   // previous cycle's results for better estimation.
//   const double v = init_point.v();
//   auto last_speed_data = GetHistorySpeed();

//   speed_upper_bound->push_back(std::max(v, speed_limit.speed_limit_points().front().second));

//   if (static_cast<double>(t_evaluated_.size() + speed_limit.speed_limit_points().size()) <
//       t_evaluated_.size() *
//           std::log(static_cast<double>(speed_limit.speed_limit_points().size()))) {
//     uint32_t i = 1;
//     uint32_t j = 0;
//     while (i < t_evaluated_.size() && j + 1 < speed_limit.speed_limit_points().size()) {
//       double distance = v * t_evaluated_[i];
//       if (!last_speed_data.Empty() && distance < last_speed_data.speed_vector().back().s()) {
//         SpeedPoint p;
//         last_speed_data.EvaluateByTime(t_evaluated_[i], &p);
//         distance = p.s();
//       }
//       constexpr double kDistanceEpsilon = 1e-6;
//       if (fabs(distance - speed_limit.speed_limit_points()[j].first) < kDistanceEpsilon) {
//         speed_upper_bound->push_back(speed_limit.speed_limit_points()[j].second);
//         ++i;
//       } else if (distance < speed_limit.speed_limit_points()[j].first) {
//         ++i;
//       } else if (distance <= speed_limit.speed_limit_points()[j + 1].first) {
//         speed_upper_bound->push_back(speed_limit.GetSpeedLimitByS(distance));
//         ++i;
//       } else {
//         ++j;
//       }
//     }

//     for (uint32_t k = speed_upper_bound->size(); k < t_evaluated_.size(); ++k) {
//       speed_upper_bound->push_back(FLAGS_planning_upper_speed_limit);
//       ADEBUG << "speed upper bound:" << speed_upper_bound->back();
//     }
//   } else {
//     auto cmp = [](const std::pair<double, double>& p1, const double s) { return p1.first < s; };

//     const auto& speed_limit_points = speed_limit.speed_limit_points();
//     for (size_t i = 1; i < t_evaluated_.size(); ++i) {
//       double s = v * t_evaluated_[i];
//       if (!last_speed_data.Empty() && s < last_speed_data.speed_vector().back().s()) {
//         SpeedPoint p;
//         last_speed_data.EvaluateByTime(t_evaluated_[i], &p);
//         s = p.s();
//       }

//       // NOTICE: we are using binary search here based on two assumptions:
//       // (1) The s in speed_limit_points increase monotonically.
//       // (2) The evaluated_t_.size() << number of speed_limit_points.size()
//       //
//       // If either of the two assumption is failed, a new algorithm must be
//       // used to replace the binary search.

//       const auto& it =
//           std::lower_bound(speed_limit_points.begin(), speed_limit_points.end(), s, cmp);
//       if (it != speed_limit_points.end()) {
//         speed_upper_bound->push_back(it->second);
//       } else {
//         speed_upper_bound->push_back(speed_limit_points.back().second);
//       }
//     }
//   }

//   if (is_change_lane_) {
//     for (uint32_t k = 0; k < t_evaluated_.size(); ++k) {
//       speed_upper_bound->at(k) *= (1.0 + FLAGS_change_lane_speed_relax_percentage);
//     }
//   }

//   const double kTimeBuffer = 1.0;
//   const double kSpeedBuffer = 0.1;
//   for (uint32_t k = 0; k < t_evaluated_.size() && t_evaluated_[k] < kTimeBuffer; ++k) {
//     speed_upper_bound->at(k) = std::fmax(init_point_.v() - kSpeedBuffer,
//     speed_upper_bound->at(k));
//   }

//   return true;
// }

}  // namespace planning
}  // namespace apollo
