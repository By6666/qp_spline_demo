/**
 * modify only base on apollo auto
 **/

/**
 * @file qp_spline_st_graph.h
 **/

#ifndef PLANNING_QP_SPLINE_SPEED_QP_SPLINE_ST_GRAPH_H_
#define PLANNING_QP_SPLINE_SPEED_QP_SPLINE_ST_GRAPH_H_

#include <memory>
#include <utility>
#include <vector>

#include "math_spline/spline_1d_generator.h"
#include "common/references.h"
#include "common/limits.h"
#include "common/point.h"
#include "speed_data.h"

namespace apollo {
namespace planning {

class QpSplineStGraph {
 public:
  QpSplineStGraph();

  // bool Search(const StGraphData& st_graph_data, const std::pair<double, double>& accel_bound,
  //             const SpeedData& reference_speed_data, SpeedData* const speed_data);

  void SetCondition(const CommonPoint& init_point, const Limits& s_limits, const Limits& v_limits,
                    const std::pair<double, double>& a_limits, const References& s_refs,
                    const References& v_refs);

  bool Solve(SpeedData* const speed_data);

 private:
  void Init();

  // Add st graph constraint
  bool AddConstraint();

  // Add objective function
  bool AddKernel();

  // solve
  bool Solve();

  // Get constraint
  void CommonGetConstraintForSolve(const Limits& limits, std::vector<double>* const lower_bounds,
                                   std::vector<double>* const upper_bounds);

  // Get reference
  void CommonGetReferenceForSolve(const References& refs, std::vector<double>* const x_coord,
                                  std::vector<double>* const x_refs);

  // // extract upper lower bound for constraint;
  // bool GetSConstraintByTime(const std::vector<const StBoundary*>& boundaries, const double time,
  //                           const double total_path_s, double* const s_upper_bound,
  //                           double* const s_lower_bound) const;

  // reference line kernel is a constant s line at s = 250m
  bool AddCruiseReferenceLineKernel(const double weight);

  bool AddSpeedRefKernel(const double weight);

  // std::vector<common::SpeedPoint> EstimateSpeedRefByInitSpeed(const CommonPoint& init_point,
  //                                                             const SpeedData& speed_ref) const;

  // void LinearFillingInsideSpeedRef(const std::vector<SpeedData>& speed_ref,
  //                                  std::vector<double>* time, std::vector<double>* v_ref) const;

  // const SpeedData GetHistorySpeed() const;
  // bool EstimateSpeedUpperBound(const CommonPoint& init_point, const Limits& speed_limit,
  //                              std::vector<double>* speed_upper_bound) const;

  // bool AddDpStReferenceKernel(const double weight) const;

 private:
  // solver
  Spline1dGenerator* spline_generator_ = nullptr;

  // t knots resolution
  double t_knots_resolution_ = 0.0;

  // knots
  std::vector<double> t_knots_;

  // evaluated t resolution
  double t_evaluated_resolution_ = 0.0;

  // evaluated points
  std::vector<double> t_evaluated_;

  // reference line kernel
  // std::vector<double> cruise_;
  // double speed_ref_;
  // std::vector<double> cruise_speed_;

  // condition

  // initial status
  CommonPoint init_point_;
  // limits
  Limits s_limits_;
  Limits v_limits_;
  std::pair<double, double> a_limits_;
  // references
  References s_refs_;
  References v_refs_;
};

}  // namespace planning
}  // namespace apollo

#endif  // PLANNING_QP_SPLINE_SPEED_QP_SPLINE_ST_GRAPH_H_
