#include "examples/sampling_c3/oim_t/xarm6_full_sampling_c3plus.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>
#include <limits>
#include <random>
#include <thread>
#include <stdexcept>

#include <drake/common/sorted_pair.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/solvers/moby_lcp_solver.h>
#include <drake/systems/framework/diagram_builder.h>

#include "common/find_resource.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "core/c3_plus.h"
#include "core/traj_eval.h"
#include "multibody/lcs_factory.h"

namespace dairlib::oim {

XarmSamplingC3PlannerMode ParseXarmSamplingC3PlannerMode(
    const std::string& mode) {
  if (mode == "reduced_exact_t") {
    return XarmSamplingC3PlannerMode::kReducedExactT;
  }
  if (mode == "full_sampling_c3plus") {
    return XarmSamplingC3PlannerMode::kFullSamplingC3Plus;
  }
  throw std::invalid_argument(
      "planner_mode must be reduced_exact_t or full_sampling_c3plus; got " +
      mode);
}

std::string XarmSamplingC3PlannerModeName(
    XarmSamplingC3PlannerMode mode) {
  switch (mode) {
    case XarmSamplingC3PlannerMode::kReducedExactT:
      return "reduced_exact_t";
    case XarmSamplingC3PlannerMode::kFullSamplingC3Plus:
      return "full_sampling_c3plus";
  }
  throw std::invalid_argument("unknown xArm Sampling-C3+ planner mode");
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
ConditionOpenTableObjectVelocity(
    const Eigen::Vector3d& angular_velocity_W,
    const Eigen::Vector3d& linear_velocity_W) {
  if (!angular_velocity_W.allFinite() || !linear_velocity_W.allFinite()) {
    throw std::invalid_argument(
        "measured open_table object velocity must be finite");
  }
  return std::make_pair(
      Eigen::Vector3d(0.0, 0.0, angular_velocity_W.z()),
      Eigen::Vector3d(linear_velocity_W.x(), linear_velocity_W.y(), 0.0));
}

XarmFullSamplingC3PlanarSettleReceipt
EvaluateXarmFullSamplingC3PlanarSettle(
    const Eigen::Vector3d& previous_position_W,
    const Eigen::Quaterniond& previous_orientation_WO,
    const Eigen::Vector3d& current_position_W,
    const Eigen::Quaterniond& current_orientation_WO,
    double resting_height, double planar_translation_tolerance,
    double yaw_tolerance, double vertical_position_tolerance,
    double tilt_tolerance) {
  if (!previous_position_W.allFinite() ||
      !previous_orientation_WO.coeffs().allFinite() ||
      !current_position_W.allFinite() ||
      !current_orientation_WO.coeffs().allFinite() ||
      !std::isfinite(resting_height) ||
      !std::isfinite(planar_translation_tolerance) ||
      !std::isfinite(yaw_tolerance) ||
      !std::isfinite(vertical_position_tolerance) ||
      !std::isfinite(tilt_tolerance) ||
      previous_orientation_WO.norm() == 0.0 ||
      current_orientation_WO.norm() == 0.0 ||
      planar_translation_tolerance < 0.0 || yaw_tolerance < 0.0 ||
      vertical_position_tolerance < 0.0 || tilt_tolerance < 0.0) {
    throw std::invalid_argument(
        "full Sampling-C3+ planar-settle inputs are invalid");
  }
  const Eigen::Quaterniond previous = previous_orientation_WO.normalized();
  const Eigen::Quaterniond current = current_orientation_WO.normalized();
  auto yaw = [](const Eigen::Quaterniond& q) {
    const double w = q.w();
    const double x = q.x();
    const double y = q.y();
    const double z = q.z();
    return std::atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z));
  };
  auto wrap = [](double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  };
  XarmFullSamplingC3PlanarSettleReceipt receipt;
  receipt.planar_translation_delta =
      (current_position_W.head<2>() -
       previous_position_W.head<2>()).norm();
  receipt.yaw_delta = std::abs(wrap(yaw(current) - yaw(previous)));
  receipt.vertical_position_error =
      std::abs(current_position_W.z() - resting_height);
  receipt.tilt_angle = std::acos(std::clamp(
      (current.toRotationMatrix() * Eigen::Vector3d::UnitZ()).z(),
      -1.0, 1.0));
  receipt.accepted =
      receipt.planar_translation_delta <= planar_translation_tolerance &&
      receipt.yaw_delta <= yaw_tolerance &&
      receipt.vertical_position_error <= vertical_position_tolerance &&
      receipt.tilt_angle <= tilt_tolerance;
  return receipt;
}

bool IsXarmFullSamplingC3ObjectUpright(
    const Eigen::Vector3d& position_W,
    const Eigen::Quaterniond& orientation_WO,
    double resting_height, double vertical_position_tolerance,
    double tilt_tolerance) {
  if (!position_W.allFinite() ||
      !orientation_WO.coeffs().allFinite() ||
      !std::isfinite(resting_height) ||
      !std::isfinite(vertical_position_tolerance) ||
      !std::isfinite(tilt_tolerance) || orientation_WO.norm() == 0.0 ||
      vertical_position_tolerance < 0.0 || tilt_tolerance < 0.0) {
    throw std::invalid_argument(
        "full Sampling-C3+ upright-pose inputs are invalid");
  }
  const Eigen::Quaterniond orientation = orientation_WO.normalized();
  const double vertical_error =
      std::abs(position_W.z() - resting_height);
  const double tilt = std::acos(std::clamp(
      (orientation.toRotationMatrix() * Eigen::Vector3d::UnitZ()).z(),
      -1.0, 1.0));
  return vertical_error <= vertical_position_tolerance &&
      tilt <= tilt_tolerance;
}

double FullSamplingC3LateralReserveLimit(
    double lateral_drift_tolerance, double contact_activation_tolerance) {
  if (!std::isfinite(lateral_drift_tolerance) ||
      !std::isfinite(contact_activation_tolerance) ||
      lateral_drift_tolerance < 0.0 || contact_activation_tolerance < 0.0) {
    throw std::invalid_argument(
        "full Sampling-C3+ lateral reserve inputs are invalid");
  }
  return std::max(0.0,
                  lateral_drift_tolerance - contact_activation_tolerance);
}

bool IsFullSamplingC3CentralSideContact(
    double sample_height_O, double object_half_height, double pusher_radius) {
  if (!std::isfinite(sample_height_O) ||
      !std::isfinite(object_half_height) ||
      !std::isfinite(pusher_radius) || object_half_height <= 0.0 ||
      pusher_radius <= 0.0 || pusher_radius > object_half_height) {
    throw std::invalid_argument(
        "full Sampling-C3+ side-contact inputs are invalid");
  }
  return std::abs(sample_height_O) <= pusher_radius;
}

bool HasFullSamplingC3ContactDwellBudget(
    int updates_used, int update_budget, int minimum_contact_steps) {
  if (updates_used < 0 || update_budget < 0 || minimum_contact_steps < 0 ||
      updates_used > update_budget) {
    throw std::invalid_argument(
        "full Sampling-C3+ execution budgets must be ordered and nonnegative");
  }
  return update_budget - updates_used > minimum_contact_steps;
}

XarmFullSamplingC3CycleBudgetReceipt
EvaluateFullSamplingC3CycleBudget(
    int updates_used, int update_budget, int acquisition_ik_steps,
    double acquisition_ik_step_seconds, int execution_update_period_ms,
    int minimum_contact_steps,
    const std::vector<int>& measured_release_recovery_updates) {
  if (updates_used < 0 || update_budget < 0 ||
      updates_used > update_budget || acquisition_ik_steps < 0 ||
      !std::isfinite(acquisition_ik_step_seconds) ||
      acquisition_ik_step_seconds <= 0.0 ||
      execution_update_period_ms <= 0 || minimum_contact_steps < 0 ||
      measured_release_recovery_updates.empty() ||
      std::any_of(measured_release_recovery_updates.begin(),
                  measured_release_recovery_updates.end(),
                  [](int updates) { return updates < 0; })) {
    throw std::invalid_argument(
        "full Sampling-C3+ cycle budget inputs are invalid");
  }
  const double acquisition_execution_updates =
      acquisition_ik_steps * acquisition_ik_step_seconds * 1000.0 /
      execution_update_period_ms;
  if (!std::isfinite(acquisition_execution_updates) ||
      acquisition_execution_updates >
          std::numeric_limits<int>::max()) {
    throw std::invalid_argument(
        "full Sampling-C3+ acquisition budget exceeds integer range");
  }

  XarmFullSamplingC3CycleBudgetReceipt receipt;
  receipt.remaining_updates = update_budget - updates_used;
  receipt.acquisition_updates =
      static_cast<int>(std::ceil(acquisition_execution_updates));
  receipt.contact_dwell_updates = minimum_contact_steps;
  receipt.release_recovery_updates = *std::max_element(
      measured_release_recovery_updates.begin(),
      measured_release_recovery_updates.end());
  receipt.measured_release_recovery_receipts =
      static_cast<int>(measured_release_recovery_updates.size());
  const int64_t required_updates =
      static_cast<int64_t>(receipt.acquisition_updates) +
      receipt.contact_dwell_updates + receipt.release_recovery_updates;
  if (required_updates > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(
        "full Sampling-C3+ required cycle budget exceeds integer range");
  }
  receipt.required_updates = static_cast<int>(required_updates);
  receipt.accepted = receipt.remaining_updates > receipt.required_updates;
  return receipt;
}

XarmFullSamplingC3AcquisitionConformanceReceipt
EvaluateFullSamplingC3AcquisitionConformance(
    bool preview_accepted, int expected_phases, int completed_phases,
    bool candidate_invalidated, bool neutral_anchor_reacquired,
    bool terminal_receipt_preserved) {
  if (expected_phases <= 0 || completed_phases < 0 ||
      completed_phases > expected_phases) {
    throw std::invalid_argument(
        "full Sampling-C3+ acquisition phase counts are invalid");
  }
  XarmFullSamplingC3AcquisitionConformanceReceipt receipt;
  receipt.preview_accepted = preview_accepted;
  receipt.expected_phases = expected_phases;
  receipt.completed_phases = completed_phases;
  receipt.physical_acquisition_completed =
      preview_accepted && completed_phases == expected_phases;
  receipt.recovery_required =
      preview_accepted && !receipt.physical_acquisition_completed;
  receipt.candidate_invalidated = candidate_invalidated;
  receipt.neutral_anchor_reacquired = neutral_anchor_reacquired;
  receipt.terminal_receipt_preserved = terminal_receipt_preserved;
  receipt.replanning_allowed = receipt.physical_acquisition_completed ||
      (receipt.recovery_required && candidate_invalidated &&
       neutral_anchor_reacquired && terminal_receipt_preserved);
  return receipt;
}

bool IsFullSamplingC3WaypointExecutionConformant(
    double phase_entry_waypoint_error, double measured_waypoint_error,
    double contact_activation_tolerance) {
  if (!std::isfinite(phase_entry_waypoint_error) ||
      phase_entry_waypoint_error < 0.0 ||
      !std::isfinite(measured_waypoint_error) ||
      measured_waypoint_error < 0.0 ||
      !std::isfinite(contact_activation_tolerance) ||
      contact_activation_tolerance < 0.0) {
    throw std::invalid_argument(
        "full Sampling-C3+ waypoint conformance inputs are invalid");
  }
  return measured_waypoint_error <=
      phase_entry_waypoint_error + contact_activation_tolerance;
}

XarmFullSamplingC3WaypointConformanceReceipt
EvaluateFullSamplingC3WaypointConformancePersistence(
    bool spatially_conformant, int prior_consecutive_violation_updates,
    int required_consecutive_violation_updates) {
  if (prior_consecutive_violation_updates < 0 ||
      required_consecutive_violation_updates <= 0) {
    throw std::invalid_argument(
        "full Sampling-C3+ waypoint conformance persistence is invalid");
  }
  XarmFullSamplingC3WaypointConformanceReceipt receipt;
  receipt.consecutive_violation_updates = spatially_conformant
      ? 0
      : prior_consecutive_violation_updates + 1;
  receipt.persistent_violation =
      receipt.consecutive_violation_updates >=
      required_consecutive_violation_updates;
  return receipt;
}

bool ShouldRetryXarmFullSamplingC3RecoveryResponse(
    bool release_cleared, bool lateral_recovered, bool crossed_goal,
    bool wrong_polarity, bool contact_lost, bool object_not_upright) {
  return release_cleared &&
      (object_not_upright ||
       (!lateral_recovered &&
        (crossed_goal || wrong_polarity || contact_lost)));
}

bool ShouldReplanXarmFullSamplingC3ReleasedExecutionFailure(
    bool execution_failed, bool release_cleared) {
  return execution_failed && release_cleared;
}

bool ShouldContinueXarmFullSamplingC3BoundedCorridorState(
    bool productive_cycle_credited, bool post_recovery_task_accepted,
    bool lateral_corridor_accepted, bool component_transaction_accepted,
    bool release_verified) {
  return !productive_cycle_credited && post_recovery_task_accepted &&
      lateral_corridor_accepted && component_transaction_accepted &&
      release_verified;
}

Eigen::VectorXd CanonicalizeXarmPeriodicIkSolutionNearestMeasured(
    const Eigen::VectorXd& solution, const Eigen::VectorXd& measured,
    const Eigen::VectorXd& lower_limits,
    const Eigen::VectorXd& upper_limits) {
  if (solution.size() == 0 || solution.size() != measured.size() ||
      solution.size() != lower_limits.size() ||
      solution.size() != upper_limits.size() || !solution.allFinite() ||
      !measured.allFinite() || !lower_limits.allFinite() ||
      !upper_limits.allFinite() ||
      (lower_limits.array() > upper_limits.array()).any()) {
    throw std::invalid_argument(
        "periodic IK canonicalization inputs are invalid");
  }
  const double two_pi = 2.0 * std::acos(-1.0);
  Eigen::VectorXd canonical = solution;
  for (int i = 0; i < canonical.size(); ++i) {
    if (upper_limits[i] - lower_limits[i] < two_pi) continue;
    const double minimum_turn = std::ceil(
        (lower_limits[i] - solution[i]) / two_pi);
    const double maximum_turn = std::floor(
        (upper_limits[i] - solution[i]) / two_pi);
    if (minimum_turn > maximum_turn) continue;
    const double nearest_turn = std::round(
        (measured[i] - solution[i]) / two_pi);
    canonical[i] = solution[i] + two_pi * std::clamp(
        nearest_turn, minimum_turn, maximum_turn);
  }
  return canonical;
}

Eigen::Vector3d BuildMeasuredVerticalTranslationSubtarget(
    const Eigen::Vector3d& measured_tip, double waypoint_z,
    double task_space_step_limit) {
  if (!measured_tip.allFinite() || !std::isfinite(waypoint_z) ||
      !std::isfinite(task_space_step_limit) || task_space_step_limit <= 0.0) {
    throw std::invalid_argument(
        "measured vertical translation inputs are invalid");
  }
  Eigen::Vector3d command = measured_tip;
  command.z() += std::clamp(
      waypoint_z - measured_tip.z(), -task_space_step_limit,
      task_space_step_limit);
  return command;
}

bool IsFullSamplingC3WaypointSettled(
    double waypoint_error, double measured_tip_speed,
    double planning_time_step, double contact_activation_tolerance) {
  if (!std::isfinite(waypoint_error) || waypoint_error < 0.0 ||
      !std::isfinite(measured_tip_speed) || measured_tip_speed < 0.0 ||
      !std::isfinite(planning_time_step) || planning_time_step <= 0.0 ||
      !std::isfinite(contact_activation_tolerance) ||
      contact_activation_tolerance < 0.0) {
    throw std::invalid_argument(
        "full Sampling-C3+ waypoint settle inputs are invalid");
  }
  return waypoint_error <= contact_activation_tolerance &&
      measured_tip_speed * planning_time_step <=
          contact_activation_tolerance;
}

bool IsFullSamplingC3WrongPolarityResponse(
    double start_signed_error, double measured_signed_error,
    int response_steps, int minimum_contact_steps) {
  if (!std::isfinite(start_signed_error) ||
      !std::isfinite(measured_signed_error) || response_steps < 0 ||
      minimum_contact_steps < 0) {
    throw std::invalid_argument(
        "full Sampling-C3+ response inputs must be finite and nonnegative");
  }
  return response_steps >= minimum_contact_steps &&
      start_signed_error *
          (measured_signed_error - start_signed_error) > 0.0;
}

XarmFullSamplingC3TerminalDescentReceipt
EvaluateFullSamplingC3TerminalDescent(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& end_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double minimum_translation_progress,
    double minimum_orientation_progress) {
  if (!start_object_pose.allFinite() || !end_object_pose.allFinite() ||
      !goal_object_pose.allFinite() ||
      !std::isfinite(minimum_translation_progress) ||
      !std::isfinite(minimum_orientation_progress) ||
      minimum_translation_progress < 0.0 ||
      minimum_orientation_progress < 0.0) {
    throw std::invalid_argument(
        "terminal descent inputs must be finite with nonnegative minima");
  }
  auto wrapped_error = [](double goal, double measured) {
    return std::atan2(std::sin(goal - measured),
                      std::cos(goal - measured));
  };
  XarmFullSamplingC3TerminalDescentReceipt receipt;
  receipt.translation_progress =
      (start_object_pose.head<2>() - goal_object_pose.head<2>()).norm() -
      (end_object_pose.head<2>() - goal_object_pose.head<2>()).norm();
  receipt.orientation_progress =
      std::abs(wrapped_error(goal_object_pose.z(), start_object_pose.z())) -
      std::abs(wrapped_error(goal_object_pose.z(), end_object_pose.z()));
  receipt.translation_nonregressive = receipt.translation_progress >= 0.0;
  receipt.orientation_nonregressive = receipt.orientation_progress >= 0.0;
  receipt.minimum_progress =
      receipt.translation_progress >= minimum_translation_progress ||
      receipt.orientation_progress >= minimum_orientation_progress;
  receipt.accepted = receipt.translation_nonregressive &&
      receipt.orientation_nonregressive && receipt.minimum_progress;
  return receipt;
}

XarmFullSamplingC3NormalizedParetoDescentReceipt
EvaluateFullSamplingC3NormalizedParetoDescent(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& end_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance,
    double orientation_tolerance) {
  if (!std::isfinite(translation_tolerance) ||
      !std::isfinite(orientation_tolerance) ||
      translation_tolerance <= 0.0 || orientation_tolerance <= 0.0) {
    throw std::invalid_argument(
        "normalized Pareto descent tolerances must be finite and positive");
  }
  XarmFullSamplingC3NormalizedParetoDescentReceipt receipt;
  receipt.terminal = EvaluateFullSamplingC3TerminalDescent(
      start_object_pose, end_object_pose, goal_object_pose, 0.0, 0.0);
  receipt.normalized_magnitude =
      receipt.terminal.translation_progress / translation_tolerance +
      receipt.terminal.orientation_progress / orientation_tolerance;
  return receipt;
}

XarmFullSamplingC3ComponentTransactionReceipt
EvaluateXarmFullSamplingC3ComponentTransaction(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& end_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance, double orientation_tolerance,
    double minimum_translation_progress,
    double minimum_orientation_progress) {
  if (!std::isfinite(minimum_translation_progress) ||
      !std::isfinite(minimum_orientation_progress) ||
      minimum_translation_progress < 0.0 ||
      minimum_orientation_progress < 0.0) {
    throw std::invalid_argument(
        "component transaction progress inputs are invalid");
  }
  const auto normalized = EvaluateFullSamplingC3NormalizedParetoDescent(
      start_object_pose, end_object_pose, goal_object_pose,
      translation_tolerance, orientation_tolerance);
  XarmFullSamplingC3ComponentTransactionReceipt receipt;
  receipt.terminal = normalized.terminal;
  receipt.normalized_magnitude = normalized.normalized_magnitude;
  receipt.translation_debt_bounded =
      -receipt.terminal.translation_progress <= translation_tolerance;
  receipt.orientation_debt_bounded =
      -receipt.terminal.orientation_progress <= orientation_tolerance;
  const bool minimum_active_progress =
      receipt.terminal.translation_progress >= minimum_translation_progress ||
      receipt.terminal.orientation_progress >= minimum_orientation_progress;
  receipt.accepted = receipt.translation_debt_bounded &&
      receipt.orientation_debt_bounded && minimum_active_progress &&
      receipt.normalized_magnitude > 0.0;
  return receipt;
}

XarmFullSamplingC3PostRecoveryReceipt
EvaluateFullSamplingC3PostRecoveryProgress(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& post_recovery_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double lateral_drift_tolerance,
    double minimum_translation_progress,
    double minimum_orientation_progress) {
  if (!std::isfinite(lateral_drift_tolerance) ||
      lateral_drift_tolerance < 0.0) {
    throw std::invalid_argument(
        "post-recovery lateral tolerance must be finite and nonnegative");
  }
  XarmFullSamplingC3PostRecoveryReceipt receipt;
  receipt.terminal = EvaluateFullSamplingC3TerminalDescent(
      start_object_pose, post_recovery_object_pose, goal_object_pose,
      minimum_translation_progress, minimum_orientation_progress);
  receipt.lateral_error = std::abs(
      post_recovery_object_pose.x() - goal_object_pose.x());
  receipt.lateral_accepted =
      receipt.lateral_error <= lateral_drift_tolerance;
  receipt.accepted = receipt.terminal.accepted && receipt.lateral_accepted;
  return receipt;
}

XarmFullSamplingC3ResponseConditioningReceipt
EvaluateFullSamplingC3MeasuredResponseConditioning(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& predicted_terminal_object_pose,
    const Eigen::Vector2d& sample_point_O,
    const Eigen::Vector2d& sample_normal_O,
    const Eigen::Vector3d& goal_object_pose,
    const std::vector<XarmFullSamplingC3MeasuredResponse>& observations,
    double translation_neighborhood,
    double orientation_neighborhood,
    double lateral_drift_tolerance,
    double minimum_translation_progress,
    double minimum_orientation_progress) {
  if (!start_object_pose.allFinite() ||
      !predicted_terminal_object_pose.allFinite() ||
      !sample_point_O.allFinite() || !sample_normal_O.allFinite() ||
      !goal_object_pose.allFinite() ||
      !std::isfinite(translation_neighborhood) ||
      !std::isfinite(orientation_neighborhood) ||
      !std::isfinite(lateral_drift_tolerance) ||
      !std::isfinite(minimum_translation_progress) ||
      !std::isfinite(minimum_orientation_progress) ||
      translation_neighborhood < 0.0 || orientation_neighborhood < 0.0 ||
      lateral_drift_tolerance < 0.0 ||
      minimum_translation_progress < 0.0 ||
      minimum_orientation_progress < 0.0 || sample_normal_O.norm() == 0.0) {
    throw std::invalid_argument(
        "measured-response conditioning inputs are invalid");
  }
  auto wrap = [](double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  };
  const Eigen::Vector2d candidate_normal = sample_normal_O.normalized();
  XarmFullSamplingC3ResponseConditioningReceipt receipt;
  double predicted_normalized_progress_sum = 0.0;
  double measured_normalized_progress_sum = 0.0;
  for (const auto& observation : observations) {
    if (!observation.start_object_pose.allFinite() ||
        !observation.predicted_terminal_object_pose.allFinite() ||
        !observation.measured_terminal_object_pose.allFinite() ||
        !observation.sample_point_O.allFinite() ||
        !observation.sample_normal_O.allFinite() ||
        observation.sample_normal_O.norm() == 0.0) {
      throw std::invalid_argument(
          "measured-response observation is invalid");
    }
    const double pose_translation_distance =
        (observation.start_object_pose.head<2>() -
         start_object_pose.head<2>()).norm();
    const double pose_orientation_distance = std::abs(wrap(
        observation.start_object_pose.z() - start_object_pose.z()));
    const double contact_translation_distance =
        (observation.sample_point_O - sample_point_O).norm();
    const Eigen::Vector2d observation_normal =
        observation.sample_normal_O.normalized();
    const double contact_orientation_distance = std::acos(std::clamp(
        candidate_normal.dot(observation_normal), -1.0, 1.0));
    if (pose_translation_distance > translation_neighborhood ||
        pose_orientation_distance > orientation_neighborhood ||
        contact_translation_distance > translation_neighborhood ||
        contact_orientation_distance > orientation_neighborhood) {
      continue;
    }

    ++receipt.matching_observations;
    const Eigen::Vector2d predicted_translation =
        observation.predicted_terminal_object_pose.head<2>() -
        observation.start_object_pose.head<2>();
    const Eigen::Vector2d measured_translation =
        observation.measured_terminal_object_pose.head<2>() -
        observation.start_object_pose.head<2>();
    receipt.mean_prediction_residual.head<2>() +=
        measured_translation - predicted_translation;
    const double predicted_yaw_delta = wrap(
        observation.predicted_terminal_object_pose.z() -
        observation.start_object_pose.z());
    const double measured_yaw_delta = wrap(
        observation.measured_terminal_object_pose.z() -
        observation.start_object_pose.z());
    receipt.mean_prediction_residual.z() +=
        wrap(measured_yaw_delta - predicted_yaw_delta);

    const auto measured_terminal = EvaluateFullSamplingC3TerminalDescent(
        observation.start_object_pose,
        observation.measured_terminal_object_pose, goal_object_pose,
        minimum_translation_progress, minimum_orientation_progress);
    if (translation_neighborhood > 0.0 &&
        orientation_neighborhood > 0.0) {
      predicted_normalized_progress_sum +=
          EvaluateFullSamplingC3NormalizedParetoDescent(
              observation.start_object_pose,
              observation.predicted_terminal_object_pose,
              goal_object_pose, translation_neighborhood,
              orientation_neighborhood).normalized_magnitude;
      measured_normalized_progress_sum +=
          EvaluateFullSamplingC3NormalizedParetoDescent(
              observation.start_object_pose,
              observation.measured_terminal_object_pose,
              goal_object_pose, translation_neighborhood,
              orientation_neighborhood).normalized_magnitude;
    }
    const bool measured_lateral_accepted =
        std::abs(observation.measured_terminal_object_pose.x() -
                 goal_object_pose.x()) <= lateral_drift_tolerance;
    if (observation.lateral_rejected) ++receipt.lateral_rejections;
    if (!measured_terminal.translation_nonregressive ||
        !measured_terminal.orientation_nonregressive) {
      ++receipt.terminal_regressions;
    }
    if (!observation.lateral_rejected && measured_lateral_accepted &&
        measured_terminal.accepted) {
      ++receipt.compatible_observations;
    }
  }

  receipt.corrected_terminal_object_pose =
      predicted_terminal_object_pose;
  if (receipt.matching_observations > 0) {
    receipt.mean_prediction_residual /= receipt.matching_observations;
    receipt.corrected_terminal_object_pose.head<2>() +=
        receipt.mean_prediction_residual.head<2>();
    receipt.corrected_terminal_object_pose.z() = wrap(
        receipt.corrected_terminal_object_pose.z() +
        receipt.mean_prediction_residual.z());
  }
  const auto corrected_terminal = EvaluateFullSamplingC3TerminalDescent(
      start_object_pose, receipt.corrected_terminal_object_pose,
      goal_object_pose, minimum_translation_progress,
      minimum_orientation_progress);
  receipt.corrected_terminal_accepted = corrected_terminal.accepted;
  receipt.corrected_lateral_error = std::abs(
      receipt.corrected_terminal_object_pose.x() - goal_object_pose.x());
  receipt.corrected_lateral_accepted =
      receipt.corrected_lateral_error <= lateral_drift_tolerance;
  if (receipt.matching_observations > 0 &&
      std::abs(predicted_normalized_progress_sum) >
          std::numeric_limits<double>::epsilon()) {
    receipt.observed_progress_gain =
        measured_normalized_progress_sum /
        predicted_normalized_progress_sum;
    receipt.calibrated_normalized_magnitude =
        EvaluateFullSamplingC3NormalizedParetoDescent(
            start_object_pose, predicted_terminal_object_pose,
            goal_object_pose, translation_neighborhood,
            orientation_neighborhood).normalized_magnitude *
        receipt.observed_progress_gain;
  }
  if (receipt.matching_observations < kMinimumResponseClassObservations) {
    receipt.ranking_class = 1;
  } else if (receipt.compatible_observations ==
                 receipt.matching_observations &&
             receipt.corrected_terminal_accepted &&
             receipt.corrected_lateral_accepted) {
    receipt.ranking_class = 0;
  } else {
    receipt.ranking_class = 2;
  }
  return receipt;
}

XarmFullSamplingC3ResponseConditioningReceipt
EvaluateFullSamplingC3EquivariantResponseConditioning(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& predicted_terminal_object_pose,
    const Eigen::Vector2d& sample_point_O,
    const Eigen::Vector2d& sample_normal_O,
    const Eigen::Vector3d& goal_object_pose,
    const std::vector<XarmFullSamplingC3MeasuredResponse>& observations,
    double contact_translation_neighborhood,
    double contact_orientation_neighborhood,
    double lateral_drift_tolerance,
    double minimum_translation_progress,
    double minimum_orientation_progress) {
  if (!start_object_pose.allFinite() ||
      !predicted_terminal_object_pose.allFinite() ||
      !sample_point_O.allFinite() || !sample_normal_O.allFinite() ||
      !goal_object_pose.allFinite() ||
      !std::isfinite(contact_translation_neighborhood) ||
      !std::isfinite(contact_orientation_neighborhood) ||
      !std::isfinite(lateral_drift_tolerance) ||
      !std::isfinite(minimum_translation_progress) ||
      !std::isfinite(minimum_orientation_progress) ||
      contact_translation_neighborhood < 0.0 ||
      contact_orientation_neighborhood < 0.0 ||
      lateral_drift_tolerance < 0.0 ||
      minimum_translation_progress < 0.0 ||
      minimum_orientation_progress < 0.0 ||
      sample_normal_O.norm() == 0.0) {
    throw std::invalid_argument(
        "equivariant response-conditioning inputs are invalid");
  }
  auto wrap = [](double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  };
  const Eigen::Vector2d candidate_normal = sample_normal_O.normalized();
  Eigen::Vector2d mean_translation_residual_O = Eigen::Vector2d::Zero();
  XarmFullSamplingC3ResponseConditioningReceipt receipt;
  for (const auto& observation : observations) {
    if (!observation.start_object_pose.allFinite() ||
        !observation.predicted_terminal_object_pose.allFinite() ||
        !observation.measured_terminal_object_pose.allFinite() ||
        !observation.sample_point_O.allFinite() ||
        !observation.sample_normal_O.allFinite() ||
        observation.sample_normal_O.norm() == 0.0) {
      throw std::invalid_argument(
          "equivariant response observation is invalid");
    }
    const double contact_translation_distance =
        (observation.sample_point_O - sample_point_O).norm();
    const double contact_orientation_distance = std::acos(std::clamp(
        candidate_normal.dot(observation.sample_normal_O.normalized()),
        -1.0, 1.0));
    if (contact_translation_distance > contact_translation_neighborhood ||
        contact_orientation_distance > contact_orientation_neighborhood) {
      continue;
    }
    ++receipt.matching_observations;
    const Eigen::Vector2d predicted_delta_W =
        observation.predicted_terminal_object_pose.head<2>() -
        observation.start_object_pose.head<2>();
    const Eigen::Vector2d measured_delta_W =
        observation.measured_terminal_object_pose.head<2>() -
        observation.start_object_pose.head<2>();
    mean_translation_residual_O +=
        Eigen::Rotation2Dd(-observation.start_object_pose.z()) *
        (measured_delta_W - predicted_delta_W);
    const double predicted_yaw_delta = wrap(
        observation.predicted_terminal_object_pose.z() -
        observation.start_object_pose.z());
    const double measured_yaw_delta = wrap(
        observation.measured_terminal_object_pose.z() -
        observation.start_object_pose.z());
    receipt.mean_prediction_residual.z() +=
        wrap(measured_yaw_delta - predicted_yaw_delta);
    const auto measured_terminal = EvaluateFullSamplingC3TerminalDescent(
        observation.start_object_pose,
        observation.measured_terminal_object_pose, goal_object_pose,
        minimum_translation_progress, minimum_orientation_progress);
    if (observation.lateral_rejected) ++receipt.lateral_rejections;
    if (!measured_terminal.translation_nonregressive ||
        !measured_terminal.orientation_nonregressive) {
      ++receipt.terminal_regressions;
    }
  }

  receipt.corrected_terminal_object_pose = predicted_terminal_object_pose;
  if (receipt.matching_observations > 0) {
    mean_translation_residual_O /= receipt.matching_observations;
    receipt.mean_prediction_residual.head<2>() =
        Eigen::Rotation2Dd(start_object_pose.z()) *
        mean_translation_residual_O;
    receipt.mean_prediction_residual.z() /= receipt.matching_observations;
    receipt.corrected_terminal_object_pose.head<2>() +=
        receipt.mean_prediction_residual.head<2>();
    receipt.corrected_terminal_object_pose.z() = wrap(
        receipt.corrected_terminal_object_pose.z() +
        receipt.mean_prediction_residual.z());
  }
  const auto corrected_terminal = EvaluateFullSamplingC3TerminalDescent(
      start_object_pose, receipt.corrected_terminal_object_pose,
      goal_object_pose, minimum_translation_progress,
      minimum_orientation_progress);
  receipt.corrected_terminal_accepted = corrected_terminal.accepted;
  receipt.corrected_lateral_error = std::abs(
      receipt.corrected_terminal_object_pose.x() - goal_object_pose.x());
  receipt.corrected_lateral_accepted =
      receipt.corrected_lateral_error <= lateral_drift_tolerance;
  if (receipt.matching_observations < kMinimumResponseClassObservations) {
    receipt.ranking_class = 1;
  } else if (receipt.corrected_terminal_accepted &&
             receipt.corrected_lateral_accepted) {
    receipt.compatible_observations = receipt.matching_observations;
    receipt.ranking_class = 0;
  } else {
    receipt.ranking_class = 2;
  }
  return receipt;
}

Eigen::VectorXd XarmFullSamplingC3State::Encode() const {
  const double quaternion_norm = object_quaternion_WO.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-12) {
    throw std::invalid_argument(
        "object quaternion must be finite and nonzero");
  }
  if (!pusher_position_W.allFinite() || !object_position_W.allFinite() ||
      !pusher_linear_velocity_W.allFinite() ||
      !object_angular_velocity_W.allFinite() ||
      !object_linear_velocity_W.allFinite()) {
    throw std::invalid_argument("full Sampling-C3+ state must be finite");
  }

  const Eigen::Quaterniond quaternion = object_quaternion_WO.normalized();
  Eigen::VectorXd state = Eigen::VectorXd::Zero(kSize);
  state.segment<3>(kPusherPosition) = pusher_position_W;
  state.segment<4>(kObjectQuaternion) << quaternion.w(), quaternion.x(),
      quaternion.y(), quaternion.z();
  state.segment<3>(kObjectPosition) = object_position_W;
  state.segment<3>(kPusherLinearVelocity) = pusher_linear_velocity_W;
  state.segment<3>(kObjectAngularVelocity) = object_angular_velocity_W;
  state.segment<3>(kObjectLinearVelocity) = object_linear_velocity_W;
  return state;
}

XarmFullSamplingC3State XarmFullSamplingC3State::Decode(
    const Eigen::VectorXd& state) {
  if (state.size() != kSize) {
    throw std::invalid_argument(
        "full Sampling-C3+ state must contain exactly 19 elements");
  }
  if (!state.allFinite()) {
    throw std::invalid_argument("full Sampling-C3+ state must be finite");
  }
  const Eigen::Vector4d quaternion_coefficients =
      state.segment<4>(kObjectQuaternion);
  if (quaternion_coefficients.norm() < 1.0e-12) {
    throw std::invalid_argument(
        "object quaternion must be finite and nonzero");
  }

  XarmFullSamplingC3State decoded;
  decoded.pusher_position_W = state.segment<3>(kPusherPosition);
  decoded.object_quaternion_WO =
      Eigen::Quaterniond(quaternion_coefficients[0],
                         quaternion_coefficients[1],
                         quaternion_coefficients[2],
                         quaternion_coefficients[3]).normalized();
  decoded.object_position_W = state.segment<3>(kObjectPosition);
  decoded.pusher_linear_velocity_W =
      state.segment<3>(kPusherLinearVelocity);
  decoded.object_angular_velocity_W =
      state.segment<3>(kObjectAngularVelocity);
  decoded.object_linear_velocity_W =
      state.segment<3>(kObjectLinearVelocity);
  return decoded;
}

namespace {

struct SpatialLcsProblem {
  c3::LCS lcs;
  Eigen::VectorXd x0;
  int num_contact_pairs{};
  bool finite{};
  double complementarity_offset_min{};
  double complementarity_offset_max{};
};

SpatialLcsProblem BuildSpatialLcsProblem(
    const OimTParams& params, const Eigen::Vector3d& pusher_position_W) {
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  drake::multibody::Parser parser(&plant);
  parser.SetAutoRenaming(true);
  parser.AddModels(dairlib::FindResourceOrThrow(
      "examples/sampling_c3/urdf/oim_xarm6_tabletop/"
      "xarm6_lcs_pusher.urdf"));
  parser.AddModels(dairlib::FindResourceOrThrow(dairlib::kGroundModel));
  parser.AddModels(dairlib::FindResourceOrThrow(params.object.model));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("xarm_lcs_pusher_base"));
  // OIM's physical table top is z=0. The generic Panda helper intentionally
  // uses a -0.029 m ground offset and therefore must not be used here.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"));
  plant.Finalize();
  auto plant_ad = drake::systems::System<double>::ToAutoDiffXd(plant);
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, diagram_context.get());
  auto plant_context_ad = plant_ad->CreateDefaultContext();

  if (plant.num_positions() != XarmFullSamplingC3State::kNumPositions ||
      plant.num_velocities() != XarmFullSamplingC3State::kNumVelocities ||
      plant.num_actuators() != XarmFullSamplingC3State::kInputSize) {
    throw std::runtime_error(
        "full Sampling-C3+ plant does not match the declared spatial layout");
  }

  XarmFullSamplingC3State state;
  state.pusher_position_W = pusher_position_W;
  state.object_quaternion_WO = Eigen::Quaterniond(
      Eigen::AngleAxisd(params.object.start_pose.z(),
                        Eigen::Vector3d::UnitZ()));
  state.object_position_W << params.object.start_pose.x(),
      params.object.start_pose.y(), params.object.resting_height;
  state.object_angular_velocity_W =
      params.object.start_angular_velocity_W;
  state.object_linear_velocity_W =
      params.object.start_linear_velocity_W;
  const Eigen::VectorXd x0 = state.Encode();
  const Eigen::VectorXd u0 =
      Eigen::VectorXd::Zero(XarmFullSamplingC3State::kInputSize);

  const auto& pusher = plant.GetBodyByName("xarm_lcs_pusher");
  const auto& object = plant.GetBodyByName(params.object.body);
  const auto& ground = plant.GetBodyByName("ground");
  const auto pusher_geometries = plant.GetCollisionGeometriesForBody(pusher);
  const auto object_geometries = plant.GetCollisionGeometriesForBody(object);
  const auto ground_geometries = plant.GetCollisionGeometriesForBody(ground);
  if (pusher_geometries.size() != 1 || object_geometries.size() != 2 ||
      ground_geometries.size() != 1) {
    throw std::runtime_error(
        "full Sampling-C3+ expected one pusher, two T, and one ground "
        "collision geometries");
  }

  using drake::SortedPair;
  using drake::geometry::GeometryId;
  std::vector<SortedPair<GeometryId>> contact_pairs;
  contact_pairs.emplace_back(pusher_geometries[0], ground_geometries[0]);
  for (const GeometryId object_geometry : object_geometries) {
    contact_pairs.emplace_back(pusher_geometries[0], object_geometry);
  }
  for (const GeometryId object_geometry : object_geometries) {
    contact_pairs.emplace_back(object_geometry, ground_geometries[0]);
  }

  c3::LCSFactoryOptions options;
  options.contact_model = params.full_sampling_c3plus.contact_model;
  options.N = params.full_sampling_c3plus.horizon;
  options.dt = params.task.planning_time_step;
  options.num_contacts = contact_pairs.size();
  options.num_friction_directions =
      params.full_sampling_c3plus.num_friction_directions;
  options.mu_per_contact = std::vector<double>{
      params.full_sampling_c3plus.pusher_ground_friction,
      params.full_sampling_c3plus.pusher_object_friction,
      params.full_sampling_c3plus.pusher_object_friction,
      params.full_sampling_c3plus.object_ground_friction,
      params.full_sampling_c3plus.object_ground_friction};

  c3::LCS lcs = c3::multibody::LCSFactory::LinearizePlantToLCS(
      plant, plant_context, *plant_ad, *plant_context_ad, contact_pairs,
      options, x0, u0);
  bool finite = true;
  for (int k = 0; k < lcs.N(); ++k) {
    finite = finite && lcs.A()[k].allFinite() && lcs.B()[k].allFinite() &&
        lcs.D()[k].allFinite() && lcs.d()[k].allFinite() &&
        lcs.E()[k].allFinite() && lcs.F()[k].allFinite() &&
        lcs.H()[k].allFinite() && lcs.c()[k].allFinite();
  }
  const Eigen::VectorXd complementarity_offset =
      lcs.E().front() * x0 + lcs.H().front() * u0 + lcs.c().front();
  return SpatialLcsProblem{
      std::move(lcs), x0, static_cast<int>(contact_pairs.size()), finite,
      complementarity_offset.minCoeff(),
      complementarity_offset.maxCoeff()};
}

double MaxAbs(const Eigen::VectorXd& value) {
  return value.size() == 0 ? 0.0 : value.cwiseAbs().maxCoeff();
}

double NonnegativeViolation(const Eigen::VectorXd& value) {
  return value.size() == 0 ? 0.0 : std::max(0.0, -value.minCoeff());
}

void AddDairlabSamplingConstraints(c3::C3* optimizer,
                                   const OimFullSamplingC3PlusParams& config) {
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::RowVectorXd pusher_position =
        Eigen::RowVectorXd::Zero(XarmFullSamplingC3State::kSize);
    pusher_position[XarmFullSamplingC3State::kPusherPosition + axis] = 1.0;
    optimizer->AddLinearConstraint(
        pusher_position, config.workspace_lower[axis] - config.workspace_margin,
        config.workspace_upper[axis] + config.workspace_margin,
        c3::ConstraintVariable::STATE);
    Eigen::RowVectorXd object_position =
        Eigen::RowVectorXd::Zero(XarmFullSamplingC3State::kSize);
    object_position[XarmFullSamplingC3State::kObjectPosition + axis] = 1.0;
    optimizer->AddLinearConstraint(
        object_position, config.workspace_lower[axis] - config.workspace_margin,
        config.workspace_upper[axis] + config.workspace_margin,
        c3::ConstraintVariable::STATE);
    Eigen::RowVectorXd pusher_velocity =
        Eigen::RowVectorXd::Zero(XarmFullSamplingC3State::kSize);
    pusher_velocity[XarmFullSamplingC3State::kPusherLinearVelocity + axis] =
        1.0;
    optimizer->AddLinearConstraint(
        pusher_velocity, config.ee_velocity_lower[axis],
        config.ee_velocity_upper[axis], c3::ConstraintVariable::STATE);
    Eigen::RowVectorXd input =
        Eigen::RowVectorXd::Zero(XarmFullSamplingC3State::kInputSize);
    input[axis] = 1.0;
    optimizer->AddLinearConstraint(
        input, config.input_lower[axis], config.input_upper[axis],
        c3::ConstraintVariable::INPUT);
  }
}

double BoxViolation(const Eigen::Vector3d& value,
                    const Eigen::VectorXd& lower,
                    const Eigen::VectorXd& upper) {
  return std::max((lower - value).maxCoeff(),
                  (value - upper).maxCoeff());
}

}  // namespace

XarmFullSamplingC3LcsReceipt BuildXarmFullSamplingC3SpatialLcsWitness(
    const OimTParams& params) {
  const SpatialLcsProblem problem = BuildSpatialLcsProblem(
      params, Eigen::Vector3d(
                  0.254967, -0.000911, params.object.resting_height));
  return XarmFullSamplingC3LcsReceipt{
      problem.finite,
      problem.lcs.num_states(),
      problem.lcs.num_inputs(),
      problem.num_contact_pairs,
      problem.lcs.num_lambdas(),
      problem.lcs.N(),
      problem.lcs.dt(),
      problem.complementarity_offset_min,
      problem.complementarity_offset_max};
}

namespace {

XarmFullSamplingC3SolveReceipt RunSolveAtSampledPusher(
    const OimTParams& params,
    const Eigen::Vector3d& sampled_pusher_W) {
  const SpatialLcsProblem problem =
      BuildSpatialLcsProblem(params, sampled_pusher_W);
  const int n = problem.lcs.num_states();
  const int m = problem.lcs.num_lambdas();
  const int k = problem.lcs.num_inputs();
  const int N = problem.lcs.N();
  const int nz = n + 2 * m + k;
  if (!problem.finite || n != 19 || k != 3 || m != 20 || N != 5) {
    throw std::runtime_error(
        "full Sampling-C3+ first solve received an invalid spatial LCS");
  }

  const auto& config = params.full_sampling_c3plus;
  Eigen::MatrixXd Q = config.state_cost_scale *
      config.state_cost_diagonal.asDiagonal();
  Eigen::MatrixXd R = config.input_cost_scale *
      config.input_cost_diagonal.asDiagonal();
  Eigen::VectorXd g = Eigen::VectorXd::Zero(nz);
  g.segment(n, m).setConstant(config.consensus_lambda_weight);
  g.segment(n + m + k, m).setConstant(config.consensus_eta_weight);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(nz);
  u.segment(n, m).setConstant(config.projection_lambda_weight);
  u.segment(n + m + k, m).setConstant(config.projection_eta_weight);
  Eigen::MatrixXd G = config.consensus_cost_scale * g.asDiagonal();
  Eigen::MatrixXd U = config.projection_cost_scale * u.asDiagonal();
  c3::C3::CostMatrices costs(
      std::vector<Eigen::MatrixXd>(N + 1, Q),
      std::vector<Eigen::MatrixXd>(N, R),
      std::vector<Eigen::MatrixXd>(N, G),
      std::vector<Eigen::MatrixXd>(N, U));

  XarmFullSamplingC3State desired_state =
      XarmFullSamplingC3State::Decode(problem.x0);
  desired_state.object_quaternion_WO = Eigen::Quaterniond(
      Eigen::AngleAxisd(params.object.goal_pose.z(),
                        Eigen::Vector3d::UnitZ()));
  desired_state.object_position_W << params.object.goal_pose.x(),
      params.object.goal_pose.y(), params.object.resting_height;
  const std::vector<Eigen::VectorXd> desired(
      N + 1, desired_state.Encode());

  c3::C3Options options;
  options.warm_start = config.warm_start;
  options.penalize_input_change = config.penalize_input_change;
  options.end_on_qp_step = config.end_on_qp_step;
  options.scale_lcs = config.scale_lcs;
  options.num_threads = config.num_threads;
  options.delta_option = config.delta_option;
  options.admm_iter = config.admm_iterations;
  options.gamma = config.gamma;
  options.rho_scale = config.rho_scale;
  options.qp_projection_alpha = config.qp_projection_alpha;
  options.qp_projection_scaling = config.qp_projection_scaling;

  c3::C3Plus optimizer(problem.lcs, costs, desired, options);
  AddDairlabSamplingConstraints(&optimizer, config);
  const auto start = std::chrono::steady_clock::now();
  optimizer.Solve(problem.x0);
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  const auto z_solution = optimizer.GetFullSolution();
  const c3::LCS& returned_lcs = optimizer.GetLCS();

  // With end_on_qp_step=false, C3 overwrites z_sol with projected delta and
  // then mixes in a rollout built from the final QP getters. Run the exact
  // same solve ending on its final QP to expose a coherent primal trajectory
  // for residual measurement. This diagnostic does not change or replace the
  // canonical returned plan above.
  c3::C3Options diagnostic_options = options;
  diagnostic_options.end_on_qp_step = true;
  c3::C3Plus diagnostic_optimizer(
      problem.lcs, costs, desired, diagnostic_options);
  AddDairlabSamplingConstraints(&diagnostic_optimizer, config);
  const auto diagnostic_start = std::chrono::steady_clock::now();
  diagnostic_optimizer.Solve(problem.x0);
  const double diagnostic_elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - diagnostic_start).count();
  const auto qp_solution = diagnostic_optimizer.GetFullSolution();
  const auto delta_solution = diagnostic_optimizer.GetDualDeltaSolution();
  const c3::LCS& solved_lcs = diagnostic_optimizer.GetLCS();
  if (z_solution.size() != static_cast<std::size_t>(N) ||
      qp_solution.size() != static_cast<std::size_t>(N) ||
      delta_solution.size() != static_cast<std::size_t>(N)) {
    throw std::runtime_error(
        "full Sampling-C3+ returned the wrong number of knots");
  }

  const double lambda_scale = config.scale_lcs
      ? solved_lcs.D().front().norm() / problem.lcs.D().front().norm()
      : 1.0;
  XarmFullSamplingC3SolveReceipt receipt;
  receipt.elapsed = elapsed;
  receipt.diagnostic_elapsed = diagnostic_elapsed;
  receipt.lambda_scale = lambda_scale;
  receipt.finite = std::isfinite(lambda_scale) && lambda_scale > 0.0;
  std::vector<Eigen::VectorXd> internal_z;
  std::vector<Eigen::VectorXd> returned_internal_z;
  internal_z.reserve(N);
  returned_internal_z.reserve(N);
  for (int knot = 0; knot < N; ++knot) {
    Eigen::VectorXd z = qp_solution[knot];
    Eigen::VectorXd returned_z = z_solution[knot];
    receipt.finite = receipt.finite && z.allFinite() && returned_z.allFinite() &&
        delta_solution[knot].allFinite();
    // C3 unscales only z's lambda segment before returning. Convert it back
    // to the optimizer's internal units before evaluating its scaled LCS and
    // comparing against the projected delta variables.
    z.segment(n, m) /= lambda_scale;
    returned_z.segment(n, m) /= lambda_scale;
    internal_z.push_back(z);
    returned_internal_z.push_back(returned_z);
    receipt.consensus_residual = std::max(
        receipt.consensus_residual,
        MaxAbs(z - delta_solution[knot]));
    const Eigen::VectorXd lambda = z.segment(n, m);
    const Eigen::VectorXd eta_model =
        solved_lcs.E()[knot] * z.head(n) +
        solved_lcs.F()[knot] * lambda +
        solved_lcs.H()[knot] * z.segment(n + m, k) +
        solved_lcs.c()[knot];
    const Eigen::VectorXd eta_stored = z.tail(m);
    receipt.equality_residual = std::max(
        receipt.equality_residual, MaxAbs(eta_stored - eta_model));
    receipt.nonnegative_residual = std::max(
        receipt.nonnegative_residual,
        std::max(NonnegativeViolation(lambda),
                 NonnegativeViolation(eta_model)));
    receipt.complementarity_residual = std::max(
        receipt.complementarity_residual,
        MaxAbs(lambda.array() * eta_model.array()));

    const Eigen::VectorXd projected_lambda =
        delta_solution[knot].segment(n, m);
    const Eigen::VectorXd projected_eta = delta_solution[knot].tail(m);
    receipt.projected_nonnegative_residual = std::max(
        receipt.projected_nonnegative_residual,
        std::max(NonnegativeViolation(projected_lambda),
                 NonnegativeViolation(projected_eta)));
    receipt.projected_complementarity_residual = std::max(
        receipt.projected_complementarity_residual,
        MaxAbs(projected_lambda.array() * projected_eta.array()));
  }
  receipt.initial_state_residual =
      MaxAbs(internal_z.front().head(n) - problem.x0);
  for (int knot = 0; knot + 1 < N; ++knot) {
    const Eigen::VectorXd& z = internal_z[knot];
    const Eigen::VectorXd predicted_next =
        solved_lcs.A()[knot] * z.head(n) +
        solved_lcs.B()[knot] * z.segment(n + m, k) +
        solved_lcs.D()[knot] * z.segment(n, m) +
        solved_lcs.d()[knot];
    receipt.dynamics_residual = std::max(
        receipt.dynamics_residual,
        MaxAbs(internal_z[knot + 1].head(n) - predicted_next));
    const Eigen::VectorXd& returned_z = returned_internal_z[knot];
    const Eigen::VectorXd returned_predicted_next =
        returned_lcs.A()[knot] * returned_z.head(n) +
        returned_lcs.B()[knot] * returned_z.segment(n + m, k) +
        returned_lcs.D()[knot] * returned_z.segment(n, m) +
        returned_lcs.d()[knot];
    receipt.returned_plan_dynamics_residual = std::max(
        receipt.returned_plan_dynamics_residual,
        MaxAbs(returned_internal_z[knot + 1].head(n) -
               returned_predicted_next));
  }

  // Match SamplingC3Controller::CalcCost(kSimLCS): forward-simulate the
  // unscaled LCS from the sampled state using the projected plan's inputs.
  // This trajectory is both dynamically feasible and backed by an LCP solve
  // at every knot, even when three-step ADMM consensus is still loose.
  std::vector<Eigen::VectorXd> projected_inputs;
  projected_inputs.reserve(N);
  for (int knot = 0; knot < N; ++knot) {
    projected_inputs.push_back(z_solution[knot].segment(n + m, k));
    receipt.planned_input_bound_violation = std::max(
        receipt.planned_input_bound_violation,
        std::max(0.0, BoxViolation(projected_inputs.back(),
                                   config.input_lower,
                                   config.input_upper)));
  }
  c3::LCSSimulateConfig simulate_config;
  simulate_config.regularized = true;
  simulate_config.min_exp = -8;
  const std::vector<Eigen::VectorXd> dynamic_rollout =
      c3::traj_eval::TrajectoryEvaluator::SimulateLCSOverTrajectory(
          problem.x0, projected_inputs, problem.lcs, simulate_config);
  receipt.dynamic_rollout_lcp_solved =
      dynamic_rollout.size() == static_cast<std::size_t>(N + 1);
  drake::solvers::MobyLcpSolver lcp_solver;
  for (int knot = 0;
       knot < N && receipt.dynamic_rollout_lcp_solved; ++knot) {
    const Eigen::VectorXd q =
        problem.lcs.E()[knot] * dynamic_rollout[knot] +
        problem.lcs.H()[knot] * projected_inputs[knot] +
        problem.lcs.c()[knot];
    Eigen::VectorXd lambda;
    const bool solved = lcp_solver.SolveLcpLemkeRegularized(
        problem.lcs.F()[knot], q, &lambda, simulate_config.min_exp,
        simulate_config.step_exp, simulate_config.max_exp,
        simulate_config.piv_tol, simulate_config.zero_tol);
    receipt.dynamic_rollout_lcp_solved =
        receipt.dynamic_rollout_lcp_solved && solved && lambda.allFinite();
    if (!receipt.dynamic_rollout_lcp_solved) break;
    const Eigen::VectorXd eta = problem.lcs.F()[knot] * lambda + q;
    const Eigen::VectorXd predicted_next =
        problem.lcs.A()[knot] * dynamic_rollout[knot] +
        problem.lcs.B()[knot] * projected_inputs[knot] +
        problem.lcs.D()[knot] * lambda + problem.lcs.d()[knot];
    receipt.dynamic_rollout_dynamics_residual = std::max(
        receipt.dynamic_rollout_dynamics_residual,
        MaxAbs(dynamic_rollout[knot + 1] - predicted_next));
    receipt.dynamic_rollout_nonnegative_residual = std::max(
        receipt.dynamic_rollout_nonnegative_residual,
        std::max(NonnegativeViolation(lambda),
                 NonnegativeViolation(eta)));
    receipt.dynamic_rollout_complementarity_residual = std::max(
        receipt.dynamic_rollout_complementarity_residual,
        MaxAbs(lambda.array() * eta.array()));
  }
  if (!dynamic_rollout.empty()) {
    receipt.dynamic_terminal_pusher_position =
        dynamic_rollout.back().segment<3>(
            XarmFullSamplingC3State::kPusherPosition);
    receipt.dynamic_terminal_object_position =
        dynamic_rollout.back().segment<3>(
            XarmFullSamplingC3State::kObjectPosition);
    for (const Eigen::VectorXd& state : dynamic_rollout) {
      receipt.finite = receipt.finite && state.allFinite();
      const Eigen::Vector3d pusher_position = state.segment<3>(
          XarmFullSamplingC3State::kPusherPosition);
      const Eigen::Vector3d object_position = state.segment<3>(
          XarmFullSamplingC3State::kObjectPosition);
      const Eigen::VectorXd lower = config.workspace_lower.array() -
          config.workspace_margin;
      const Eigen::VectorXd upper = config.workspace_upper.array() +
          config.workspace_margin;
      receipt.dynamic_rollout_workspace_violation = std::max(
          receipt.dynamic_rollout_workspace_violation,
          std::max(0.0, std::max(BoxViolation(pusher_position, lower, upper),
                                 BoxViolation(object_position, lower, upper))));
    }
  }
  receipt.dynamic_rollout_workspace_accepted =
      receipt.dynamic_rollout_workspace_violation <= 1.0e-12 &&
      receipt.planned_input_bound_violation <= 1.0e-12;
  if (dynamic_rollout.size() == static_cast<std::size_t>(N + 1)) {
    for (int knot = 0; knot < N; ++knot) {
      const Eigen::VectorXd state_error =
          dynamic_rollout[knot] - desired[knot];
      receipt.dynamic_rollout_cost +=
          state_error.dot(Q * state_error) +
          projected_inputs[knot].dot(R * projected_inputs[knot]);
    }
    const Eigen::VectorXd terminal_error =
        dynamic_rollout.back() - desired.back();
    receipt.dynamic_rollout_cost += terminal_error.dot(Q * terminal_error);
    receipt.finite = receipt.finite &&
        std::isfinite(receipt.dynamic_rollout_cost);
  } else {
    receipt.dynamic_rollout_cost =
        std::numeric_limits<double>::infinity();
  }
  receipt.dynamic_rollout_accepted = receipt.finite &&
      receipt.dynamic_rollout_lcp_solved &&
      receipt.dynamic_rollout_dynamics_residual <=
          config.dynamics_residual_tolerance &&
      receipt.dynamic_rollout_nonnegative_residual <=
          config.nonnegative_residual_tolerance &&
      receipt.dynamic_rollout_complementarity_residual <=
          config.complementarity_residual_tolerance;
  receipt.dynamic_state_trajectory = dynamic_rollout;
  receipt.planned_input_trajectory = projected_inputs;
  receipt.accepted = receipt.finite &&
      receipt.initial_state_residual <= config.dynamics_residual_tolerance &&
      receipt.dynamics_residual <= config.dynamics_residual_tolerance &&
      receipt.equality_residual <= config.equality_residual_tolerance &&
      receipt.nonnegative_residual <=
          config.nonnegative_residual_tolerance &&
      receipt.complementarity_residual <=
          config.complementarity_residual_tolerance &&
      receipt.consensus_residual <= config.consensus_residual_tolerance;
  return receipt;
}

}  // namespace

const std::vector<XarmFullSamplingC3ContactSample>&
GetXarmFullSamplingC3ExactTSamples() {
  static const std::vector<XarmFullSamplingC3ContactSample> samples = {
      {{-0.0445, 0.0198}, {0.0, 1.0}, "crossbar_top_left"},
      {{0.0, 0.0198}, {0.0, 1.0}, "crossbar_top"},
      {{0.0445, 0.0198}, {0.0, 1.0}, "crossbar_top_right"},
      {{-0.0445, 0.0099}, {-1.0, 0.0}, "crossbar_left"},
      {{0.0445, 0.0099}, {1.0, 0.0}, "crossbar_right"},
      {{-0.0099, -0.0397}, {-1.0, 0.0}, "stem_left"},
      {{0.0099, -0.0397}, {1.0, 0.0}, "stem_right"},
      {{0.0, -0.0794}, {0.0, -1.0}, "stem_bottom"}};
  return samples;
}

std::vector<XarmFullSamplingC3ContactSample>
GenerateXarmFullSamplingC3PerimeterSamples(int count, int seed) {
  if (count <= 0 || seed < 0) {
    throw std::invalid_argument("perimeter sample count must be positive and seed nonnegative");
  }
  struct Segment {
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    Eigen::Vector2d normal;
    const char* name;
  };
  // Exterior boundary of the union of the exact crossbar and stem boxes.
  static const std::vector<Segment> segments = {
      {{-0.0445, 0.0198}, {0.0445, 0.0198}, {0, 1}, "crossbar_top"},
      {{-0.0445, 0.0}, {-0.0445, 0.0198}, {-1, 0}, "crossbar_left"},
      {{0.0445, 0.0}, {0.0445, 0.0198}, {1, 0}, "crossbar_right"},
      {{-0.0445, 0.0}, {-0.0099, 0.0}, {0, -1}, "crossbar_bottom_left"},
      {{0.0099, 0.0}, {0.0445, 0.0}, {0, -1}, "crossbar_bottom_right"},
      {{-0.0099, -0.0794}, {-0.0099, 0.0}, {-1, 0}, "stem_left"},
      {{0.0099, -0.0794}, {0.0099, 0.0}, {1, 0}, "stem_right"},
      {{-0.0099, -0.0794}, {0.0099, -0.0794}, {0, -1}, "stem_bottom"}};
  std::vector<double> lengths;
  lengths.reserve(segments.size());
  for (const auto& segment : segments) {
    lengths.push_back((segment.end - segment.start).norm());
  }
  std::mt19937 generator(seed);
  std::discrete_distribution<int> segment_distribution(lengths.begin(),
                                                        lengths.end());
  std::uniform_real_distribution<double> alpha_distribution(0.0, 1.0);
  std::vector<XarmFullSamplingC3ContactSample> samples;
  samples.reserve(count);
  for (int index = 0; index < count; ++index) {
    const Segment& segment = segments[segment_distribution(generator)];
    const double alpha = alpha_distribution(generator);
    samples.push_back({segment.start + alpha * (segment.end - segment.start),
                       segment.normal,
                       std::string(segment.name) + "_seeded_" +
                           std::to_string(index)});
  }
  return samples;
}

std::vector<XarmFullSamplingC3ContactSample>
GenerateXarmFullSamplingC3StemLeftRefinementSamples(int count) {
  if (count <= 0) {
    throw std::invalid_argument(
        "stem-left refinement sample count must be positive");
  }
  std::vector<XarmFullSamplingC3ContactSample> samples;
  samples.reserve(count);
  for (int index = 0; index < count; ++index) {
    const double alpha = (index + 0.5) / static_cast<double>(count);
    samples.push_back({
        Eigen::Vector2d(-0.0099, -0.0794 + alpha * 0.0794),
        Eigen::Vector2d(-1.0, 0.0),
        "stem_left_refined_" + std::to_string(index)});
  }
  return samples;
}

std::vector<XarmFullSamplingC3ContactSample>
GenerateXarmFullSamplingC3StemRightRefinementSamples(int count) {
  if (count <= 0) {
    throw std::invalid_argument(
        "stem-right refinement sample count must be positive");
  }
  std::vector<XarmFullSamplingC3ContactSample> samples;
  samples.reserve(count);
  for (int index = 0; index < count; ++index) {
    const double alpha = (index + 0.5) / static_cast<double>(count);
    samples.push_back({
        Eigen::Vector2d(0.0099, -0.0794 + alpha * 0.0794),
        Eigen::Vector2d(1.0, 0.0),
        "stem_right_refined_" + std::to_string(index)});
  }
  return samples;
}

std::vector<XarmFullSamplingC3ContactSample>
GenerateXarmFullSamplingC3MeshNormalSamples(int count, int seed) {
  if (count <= 0 || seed < 0) {
    throw std::invalid_argument(
        "mesh sample count must be positive and seed nonnegative");
  }
  struct SideSegment {
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    Eigen::Vector2d normal;
    const char* name;
  };
  static const std::vector<SideSegment> segments = {
      {{-0.0445, 0.0198}, {0.0445, 0.0198}, {0, 1}, "crossbar_top"},
      {{-0.0445, 0.0}, {-0.0445, 0.0198}, {-1, 0}, "crossbar_left"},
      {{0.0445, 0.0}, {0.0445, 0.0198}, {1, 0}, "crossbar_right"},
      {{-0.0445, 0.0}, {-0.0099, 0.0}, {0, -1}, "crossbar_bottom_left"},
      {{0.0099, 0.0}, {0.0445, 0.0}, {0, -1}, "crossbar_bottom_right"},
      {{-0.0099, -0.0794}, {-0.0099, 0.0}, {-1, 0}, "stem_left"},
      {{0.0099, -0.0794}, {0.0099, 0.0}, {1, 0}, "stem_right"},
      {{-0.0099, -0.0794}, {0.0099, -0.0794}, {0, -1}, "stem_bottom"}};
  struct Triangle {
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    Eigen::Vector3d c;
    Eigen::Vector2d normal;
    std::string name;
    double area{};
  };
  constexpr double kHalfHeight = 0.0298;
  std::vector<Triangle> triangles;
  std::vector<double> areas;
  triangles.reserve(2 * segments.size());
  areas.reserve(2 * segments.size());
  for (const auto& segment : segments) {
    const Eigen::Vector3d a(segment.start.x(), segment.start.y(),
                            -kHalfHeight);
    const Eigen::Vector3d b(segment.end.x(), segment.end.y(),
                            -kHalfHeight);
    const Eigen::Vector3d c(segment.end.x(), segment.end.y(), kHalfHeight);
    const Eigen::Vector3d d(segment.start.x(), segment.start.y(), kHalfHeight);
    const double area = 0.5 * (b - a).cross(c - a).norm();
    triangles.push_back({a, b, c, segment.normal, segment.name, area});
    triangles.push_back({a, c, d, segment.normal, segment.name, area});
    areas.push_back(area);
    areas.push_back(area);
  }
  std::mt19937 generator(seed);
  std::discrete_distribution<int> triangle_distribution(areas.begin(),
                                                         areas.end());
  std::uniform_real_distribution<double> unit_distribution(0.0, 1.0);
  std::vector<XarmFullSamplingC3ContactSample> samples;
  samples.reserve(count);
  for (int index = 0; index < count; ++index) {
    const Triangle& triangle = triangles[triangle_distribution(generator)];
    const double root_r1 = std::sqrt(unit_distribution(generator));
    const double r2 = unit_distribution(generator);
    const Eigen::Vector3d point = (1.0 - root_r1) * triangle.a +
        root_r1 * (1.0 - r2) * triangle.b + root_r1 * r2 * triangle.c;
    samples.push_back({point.head<2>(), triangle.normal,
                       "mesh_" + triangle.name + "_seeded_" +
                           std::to_string(index),
                       point.z()});
  }
  return samples;
}

XarmFullSamplingC3SolveReceipt RunXarmFullSamplingC3FirstSolve(
    const OimTParams& params) {
  // Sampling-C3+ solves contact modes from sampled future pusher locations;
  // free-space travel to the sample belongs to repositioning. Keep the first
  // witness pinned to the retained reduced planner's stem-bottom sample.
  const auto& sample = GetXarmFullSamplingC3ExactTSamples().back();
  const Eigen::Rotation2Dd R_WO(params.object.start_pose.z());
  const Eigen::Vector2d point_W = R_WO * sample.point_O;
  const Eigen::Vector2d normal_W = R_WO * sample.outward_normal_O;
  Eigen::Vector3d sampled_pusher_W;
  sampled_pusher_W.head<2>() = params.object.start_pose.head<2>() + point_W +
      normal_W * (params.controller.pusher_radius -
                  0.5 * params.controller.contact_activation_tolerance);
  sampled_pusher_W.z() = params.object.resting_height;
  return RunSolveAtSampledPusher(params, sampled_pusher_W);
}

namespace {

XarmFullSamplingC3BatchReceipt RunSampleBatch(
    const OimTParams& params,
    const std::vector<XarmFullSamplingC3ContactSample>& samples,
    int num_outer_threads) {
  XarmFullSamplingC3BatchReceipt batch;
  batch.num_samples = static_cast<int>(samples.size());
  batch.selected_cost = std::numeric_limits<double>::infinity();
  batch.candidates.resize(samples.size());
  const Eigen::Rotation2Dd R_WO(params.object.start_pose.z());
  const auto evaluate = [&](int index) {
    const auto& sample = samples[index];
    const Eigen::Vector2d point_W = R_WO * sample.point_O;
    const Eigen::Vector2d normal_W = R_WO * sample.outward_normal_O;
    XarmFullSamplingC3CandidateReceipt candidate;
    candidate.sample_index = index;
    candidate.sample_name = sample.name;
    candidate.sample_point_O = sample.point_O;
    candidate.sample_normal_O = sample.outward_normal_O;
    candidate.sample_height_O = sample.height_O;
    candidate.initial_pusher_position_W.head<2>() =
        params.object.start_pose.head<2>() + point_W +
        normal_W * (params.controller.pusher_radius -
                    0.5 * params.controller.contact_activation_tolerance);
    candidate.initial_pusher_position_W.z() =
        params.object.resting_height + sample.height_O;
    candidate.solve =
        RunSolveAtSampledPusher(params, candidate.initial_pusher_position_W);
    return candidate;
  };
  if (num_outer_threads <= 1) {
    for (int index = 0; index < static_cast<int>(samples.size()); ++index) {
      batch.candidates[index] = evaluate(index);
    }
  } else {
    std::atomic<int> next_index{0};
    std::vector<std::thread> workers;
    const int worker_count =
        std::min(num_outer_threads, static_cast<int>(samples.size()));
    workers.reserve(worker_count);
    for (int worker = 0; worker < worker_count; ++worker) {
      workers.emplace_back([&]() {
        while (true) {
          const int index = next_index.fetch_add(1);
          if (index >= static_cast<int>(samples.size())) break;
          batch.candidates[index] = evaluate(index);
        }
      });
    }
    for (auto& worker : workers) worker.join();
  }
  // Reduction is intentionally serial and ordered by provider index.
  for (int index = 0; index < static_cast<int>(samples.size()); ++index) {
    const auto& candidate = batch.candidates[index];
    if (candidate.solve.dynamic_rollout_accepted &&
        candidate.solve.dynamic_rollout_workspace_accepted) {
      ++batch.num_feasible;
      if (candidate.solve.dynamic_rollout_cost < batch.selected_cost) {
        batch.selected_index = index;
        batch.selected_name = candidate.sample_name;
        batch.selected_cost = candidate.solve.dynamic_rollout_cost;
      }
    }
  }
  batch.accepted = batch.num_samples == static_cast<int>(samples.size()) &&
      batch.num_feasible > 0 && batch.selected_index >= 0 &&
      std::isfinite(batch.selected_cost);
  return batch;
}

}  // namespace

XarmFullSamplingC3BatchReceipt RunXarmFullSamplingC3ExactTBatch(
    const OimTParams& params) {
  return RunSampleBatch(params, GetXarmFullSamplingC3ExactTSamples(), 1);
}

XarmFullSamplingC3BatchReceipt RunXarmFullSamplingC3PerimeterBatch(
    const OimTParams& params) {
  return RunSampleBatch(
      params, GenerateXarmFullSamplingC3PerimeterSamples(
                  params.full_sampling_c3plus.perimeter_sample_count,
                  params.full_sampling_c3plus.random_seed), 1);
}

XarmFullSamplingC3BatchReceipt RunXarmFullSamplingC3PerimeterBatchParallel(
    const OimTParams& params) {
  return RunSampleBatch(
      params, GenerateXarmFullSamplingC3PerimeterSamples(
                  params.full_sampling_c3plus.perimeter_sample_count,
                  params.full_sampling_c3plus.random_seed),
      params.full_sampling_c3plus.num_outer_threads);
}

XarmFullSamplingC3BatchReceipt RunXarmFullSamplingC3MeshBatchParallel(
    const OimTParams& params) {
  return RunSampleBatch(
      params, GenerateXarmFullSamplingC3MeshNormalSamples(
                  params.full_sampling_c3plus.mesh_sample_count,
                  params.full_sampling_c3plus.mesh_random_seed),
      params.full_sampling_c3plus.num_outer_threads);
}

XarmFullSamplingC3BatchReceipt
RunXarmFullSamplingC3StemLeftRefinementBatchParallel(
    const OimTParams& params) {
  return RunSampleBatch(
      params, GenerateXarmFullSamplingC3StemLeftRefinementSamples(
                  params.full_sampling_c3plus.perimeter_sample_count),
      params.full_sampling_c3plus.num_outer_threads);
}

XarmFullSamplingC3BatchReceipt
RunXarmFullSamplingC3StemRightRefinementBatchParallel(
    const OimTParams& params) {
  return RunSampleBatch(
      params, GenerateXarmFullSamplingC3StemRightRefinementSamples(
                  params.full_sampling_c3plus.perimeter_sample_count),
      params.full_sampling_c3plus.num_outer_threads);
}

XarmFullSamplingC3CandidateBuffer BuildXarmFullSamplingC3CandidateBuffer(
    const std::vector<XarmFullSamplingC3BatchReceipt>& batches) {
  XarmFullSamplingC3CandidateBuffer buffer;
  for (const auto& batch : batches) {
    buffer.total_candidates += static_cast<int>(batch.candidates.size());
    for (const auto& candidate : batch.candidates) {
      if (candidate.solve.dynamic_rollout_accepted &&
          candidate.solve.dynamic_rollout_workspace_accepted &&
          std::isfinite(candidate.solve.dynamic_rollout_cost)) {
        buffer.successful.push_back(candidate);
      } else {
        buffer.unsuccessful.push_back(candidate);
      }
    }
  }
  std::stable_sort(
      buffer.successful.begin(), buffer.successful.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.solve.dynamic_rollout_cost < rhs.solve.dynamic_rollout_cost;
      });
  buffer.accepted = buffer.total_candidates > 0 &&
      !buffer.successful.empty() &&
      buffer.total_candidates == static_cast<int>(
          buffer.successful.size() + buffer.unsuccessful.size());
  return buffer;
}

XarmFullSamplingC3CandidateBuffer
BuildXarmFullSamplingC3ContactFeasibleCandidateBuffer(
    const XarmFullSamplingC3CandidateBuffer& workspace_filtered_buffer) {
  XarmFullSamplingC3CandidateBuffer buffer;
  buffer.total_candidates = workspace_filtered_buffer.total_candidates;
  auto classify = [&](const auto& candidates) {
    for (const auto& candidate : candidates) {
      if (candidate.solve.dynamic_rollout_accepted &&
          candidate.solve.planned_input_bound_violation <= 1.0e-12 &&
          std::isfinite(candidate.solve.dynamic_rollout_cost)) {
        buffer.successful.push_back(candidate);
      } else {
        buffer.unsuccessful.push_back(candidate);
      }
    }
  };
  classify(workspace_filtered_buffer.successful);
  classify(workspace_filtered_buffer.unsuccessful);
  std::stable_sort(
      buffer.successful.begin(), buffer.successful.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.solve.dynamic_rollout_cost < rhs.solve.dynamic_rollout_cost;
      });
  buffer.accepted = buffer.total_candidates > 0 &&
      !buffer.successful.empty() &&
      buffer.total_candidates == static_cast<int>(
          buffer.successful.size() + buffer.unsuccessful.size());
  return buffer;
}

XarmFullSamplingC3CorridorPrefixReceipt
BuildXarmFullSamplingC3CorridorSafePrefix(
    const XarmFullSamplingC3CandidateReceipt& candidate,
    double goal_object_x, double lateral_drift_tolerance) {
  XarmFullSamplingC3CorridorPrefixReceipt receipt;
  receipt.execution_candidate = candidate;
  const auto& states = candidate.solve.dynamic_state_trajectory;
  receipt.original_state_knots = states.size();
  if (!std::isfinite(goal_object_x) ||
      !std::isfinite(lateral_drift_tolerance) ||
      lateral_drift_tolerance < 0.0 || states.size() < 2 ||
      candidate.solve.planned_input_trajectory.size() + 1 != states.size()) {
    return receipt;
  }
  for (const Eigen::VectorXd& encoded_state : states) {
    if (encoded_state.size() != XarmFullSamplingC3State::kSize ||
        !encoded_state.allFinite()) {
      break;
    }
    const auto state = XarmFullSamplingC3State::Decode(encoded_state);
    const double lateral_error =
        std::abs(state.object_position_W.x() - goal_object_x);
    if (lateral_error > lateral_drift_tolerance) break;
    ++receipt.retained_state_knots;
    receipt.terminal_lateral_error = lateral_error;
  }
  if (receipt.retained_state_knots < 2) return receipt;

  auto& execution_solve = receipt.execution_candidate.solve;
  execution_solve.dynamic_state_trajectory.resize(
      receipt.retained_state_knots);
  execution_solve.planned_input_trajectory.resize(
      receipt.retained_state_knots - 1);
  const auto terminal = XarmFullSamplingC3State::Decode(
      execution_solve.dynamic_state_trajectory.back());
  execution_solve.dynamic_terminal_pusher_position =
      terminal.pusher_position_W;
  execution_solve.dynamic_terminal_object_position =
      terminal.object_position_W;
  receipt.accepted = true;
  return receipt;
}

XarmFullSamplingC3TaskSpacePlan BuildXarmFullSamplingC3TaskSpacePlan(
    const XarmFullSamplingC3CandidateBuffer& buffer, double dt) {
  XarmFullSamplingC3TaskSpacePlan plan;
  if (!buffer.accepted || buffer.successful.empty() || !std::isfinite(dt) ||
      dt <= 0.0) {
    return plan;
  }
  const auto& selected = buffer.successful.front();
  const auto& states = selected.solve.dynamic_state_trajectory;
  if (states.size() < 2 ||
      selected.solve.planned_input_trajectory.size() + 1 != states.size()) {
    return plan;
  }
  plan.sample_name = selected.sample_name;
  plan.sample_point_O = selected.sample_point_O;
  plan.sample_normal_O = selected.sample_normal_O;
  plan.sample_height_O = selected.sample_height_O;
  plan.cost = selected.solve.dynamic_rollout_cost;
  plan.time_vector.resize(states.size());
  plan.pusher_positions_W.resize(3, states.size());
  plan.pusher_velocities_W.resize(3, states.size());
  for (int knot = 0; knot < static_cast<int>(states.size()); ++knot) {
    if (states[knot].size() != XarmFullSamplingC3State::kSize ||
        !states[knot].allFinite()) {
      return XarmFullSamplingC3TaskSpacePlan{};
    }
    plan.time_vector[knot] = knot * dt;
    plan.pusher_positions_W.col(knot) = states[knot].segment<3>(
        XarmFullSamplingC3State::kPusherPosition);
    plan.pusher_velocities_W.col(knot) = states[knot].segment<3>(
        XarmFullSamplingC3State::kPusherLinearVelocity);
  }
  plan.accepted = plan.time_vector.allFinite() &&
      plan.pusher_positions_W.allFinite() &&
      plan.pusher_velocities_W.allFinite() &&
      plan.pusher_positions_W.col(0).isApprox(
          selected.initial_pusher_position_W, 1.0e-12);
  return plan;
}

XarmFullSamplingC3TaskSpacePlan
BuildXarmFullSamplingC3LateralGuardedTaskSpacePlan(
    const XarmFullSamplingC3CandidateBuffer& buffer, double dt,
    double goal_object_x, double lateral_drift_tolerance) {
  if (!std::isfinite(goal_object_x) ||
      !std::isfinite(lateral_drift_tolerance) ||
      lateral_drift_tolerance < 0.0) {
    return {};
  }
  XarmFullSamplingC3CandidateBuffer guarded = buffer;
  guarded.successful.clear();
  guarded.unsuccessful = buffer.unsuccessful;
  for (const auto& candidate : buffer.successful) {
    if (std::abs(candidate.solve.dynamic_terminal_object_position.x() -
                 goal_object_x) <= lateral_drift_tolerance) {
      guarded.successful.push_back(candidate);
    } else {
      guarded.unsuccessful.push_back(candidate);
    }
  }
  guarded.accepted = guarded.total_candidates > 0 &&
      !guarded.successful.empty() &&
      guarded.total_candidates == static_cast<int>(
          guarded.successful.size() + guarded.unsuccessful.size());
  return BuildXarmFullSamplingC3TaskSpacePlan(guarded, dt);
}

XarmFullSamplingC3OscExecutionPlan BuildXarmFullSamplingC3OscExecutionPlan(
    const Eigen::Vector3d& current_tip_W,
    const Eigen::Vector3d& planar_waypoint_W,
    const Eigen::Vector3d& elevated_waypoint_W,
    const Eigen::Vector3d& standoff_waypoint_W,
    const XarmFullSamplingC3TaskSpacePlan& c3_plan,
    double reposition_speed, double c3_dt) {
  XarmFullSamplingC3OscExecutionPlan execution;
  if (!c3_plan.accepted || !current_tip_W.allFinite() ||
      !planar_waypoint_W.allFinite() ||
      !elevated_waypoint_W.allFinite() ||
      !standoff_waypoint_W.allFinite() ||
      !std::isfinite(reposition_speed) || reposition_speed <= 0.0 ||
      !std::isfinite(c3_dt) || c3_dt <= 0.0 ||
      c3_plan.pusher_positions_W.cols() < 2) {
    return execution;
  }
  constexpr int kAcquisitionKnots = 5;
  const int c3_tail_knots = c3_plan.pusher_positions_W.cols() - 1;
  const int total_knots = kAcquisitionKnots + c3_tail_knots;
  execution.time_vector.resize(total_knots);
  execution.positions_W.resize(3, total_knots);
  execution.positions_W.col(0) = current_tip_W;
  execution.positions_W.col(1) = planar_waypoint_W;
  execution.positions_W.col(2) = elevated_waypoint_W;
  execution.positions_W.col(3) = standoff_waypoint_W;
  execution.positions_W.col(4) = c3_plan.pusher_positions_W.col(0);
  execution.time_vector[0] = 0.0;
  for (int knot = 1; knot < kAcquisitionKnots; ++knot) {
    const double distance =
        (execution.positions_W.col(knot) -
         execution.positions_W.col(knot - 1)).norm();
    execution.time_vector[knot] = execution.time_vector[knot - 1] +
        std::max(c3_dt, distance / reposition_speed);
  }
  for (int tail = 0; tail < c3_tail_knots; ++tail) {
    const int knot = kAcquisitionKnots + tail;
    execution.positions_W.col(knot) =
        c3_plan.pusher_positions_W.col(tail + 1);
    execution.time_vector[knot] =
        execution.time_vector[kAcquisitionKnots - 1] + (tail + 1) * c3_dt;
  }
  execution.acquisition_knots = kAcquisitionKnots;
  execution.c3_knots = c3_plan.pusher_positions_W.cols();
  execution.accepted = execution.time_vector.allFinite() &&
      execution.positions_W.allFinite();
  for (int knot = 1; knot < execution.time_vector.size(); ++knot) {
    execution.accepted = execution.accepted &&
        execution.time_vector[knot] > execution.time_vector[knot - 1];
  }
  return execution;
}

XarmFullSamplingC3OpenTableTerminalReceipt
EvaluateXarmFullSamplingC3OpenTableTerminal(
    const Eigen::Vector3d& object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance, double orientation_tolerance) {
  if (!object_pose.allFinite() || !goal_object_pose.allFinite() ||
      !std::isfinite(translation_tolerance) ||
      !std::isfinite(orientation_tolerance) ||
      translation_tolerance <= 0.0 || orientation_tolerance <= 0.0) {
    throw std::invalid_argument(
        "full Sampling-C3+ open_table terminal inputs are invalid");
  }
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  XarmFullSamplingC3OpenTableTerminalReceipt receipt;
  receipt.translation_error =
      (object_pose.head<2>() - goal_object_pose.head<2>()).norm();
  receipt.orientation_error = std::abs(std::remainder(
      goal_object_pose.z() - object_pose.z(), kTwoPi));
  receipt.translation_accepted =
      receipt.translation_error <= translation_tolerance;
  receipt.orientation_accepted =
      receipt.orientation_error <= orientation_tolerance;
  receipt.accepted =
      receipt.translation_accepted && receipt.orientation_accepted;
  return receipt;
}

XarmFullSamplingC3TerminalStatus EvaluateXarmFullSamplingC3TerminalStatus(
    int reached_initial_waypoints, int total_initial_waypoints,
    bool any_productive_cycle, bool cycle_budget_deferred,
    int execution_updates, int execution_budget,
    bool terminal_pose_accepted) {
  XarmFullSamplingC3TerminalStatus status;
  status.closed_loop_handoff =
      reached_initial_waypoints == total_initial_waypoints ||
      any_productive_cycle;
  status.accepted = status.closed_loop_handoff && terminal_pose_accepted;
  if (status.accepted) {
    status.reason = "accepted";
    return status;
  }
  if (!status.closed_loop_handoff) {
    if (execution_updates >= execution_budget) {
      status.reason = "execution_budget_exhausted";
      status.return_code = 2;
    } else if (cycle_budget_deferred) {
      status.reason = "measured_cycle_budget_deferred_without_handoff";
      status.return_code = 2;
    } else {
      status.reason = "closed_loop_handoff_failed";
      status.return_code = 3;
    }
    return status;
  }
  status.reason = cycle_budget_deferred
      ? "measured_cycle_budget_deferred_before_terminal"
      : "terminal_tolerance_failed";
  status.return_code = 4;
  return status;
}

XarmFullSamplingC3ParetoWrenchReceipt EvaluateXarmFullSamplingC3ParetoWrench(
    double desired_translation_direction, double force_translation_component,
    double desired_orientation_direction, double planar_moment) {
  if (!std::isfinite(desired_translation_direction) ||
      !std::isfinite(force_translation_component) ||
      !std::isfinite(desired_orientation_direction) ||
      !std::isfinite(planar_moment)) {
    throw std::invalid_argument("Pareto wrench inputs must be finite");
  }
  XarmFullSamplingC3ParetoWrenchReceipt receipt;
  receipt.translation_alignment =
      desired_translation_direction * force_translation_component;
  receipt.orientation_alignment =
      desired_orientation_direction * planar_moment;
  receipt.translation_nonregressive = receipt.translation_alignment >= 0.0;
  receipt.orientation_nonregressive = receipt.orientation_alignment >= 0.0;
  receipt.minimum_productivity = receipt.translation_alignment > 0.0 ||
      receipt.orientation_alignment > 0.0;
  receipt.accepted = receipt.translation_nonregressive &&
      receipt.orientation_nonregressive && receipt.minimum_productivity;
  return receipt;
}

XarmFullSamplingC3LateralRecoveryReceipt
EvaluateXarmFullSamplingC3LateralRecovery(
    const Eigen::Vector3d& cycle_start_object_pose,
    const Eigen::Vector3d& rejected_object_pose,
    const Eigen::Vector3d& recovery_terminal_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance, double orientation_tolerance,
    double lateral_drift_tolerance) {
  if (!cycle_start_object_pose.allFinite() ||
      !rejected_object_pose.allFinite() ||
      !recovery_terminal_object_pose.allFinite() ||
      !goal_object_pose.allFinite() ||
      !std::isfinite(translation_tolerance) ||
      !std::isfinite(orientation_tolerance) ||
      !std::isfinite(lateral_drift_tolerance) ||
      translation_tolerance <= 0.0 || orientation_tolerance <= 0.0 ||
      lateral_drift_tolerance <= 0.0) {
    throw std::invalid_argument("lateral recovery inputs are invalid");
  }
  auto wrap = [](double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  };
  XarmFullSamplingC3LateralRecoveryReceipt receipt;
  const double rejected_lateral_error = std::abs(
      rejected_object_pose.x() - goal_object_pose.x());
  receipt.lateral_error = std::abs(
      recovery_terminal_object_pose.x() - goal_object_pose.x());
  receipt.lateral_reduction =
      rejected_lateral_error - receipt.lateral_error;
  const double cycle_start_translation_error =
      (cycle_start_object_pose.head<2>() -
       goal_object_pose.head<2>()).norm();
  const double recovery_translation_error =
      (recovery_terminal_object_pose.head<2>() -
       goal_object_pose.head<2>()).norm();
  receipt.translation_progress =
      cycle_start_translation_error - recovery_translation_error;
  const double cycle_start_orientation_error = std::abs(wrap(
      goal_object_pose.z() - cycle_start_object_pose.z()));
  const double recovery_orientation_error = std::abs(wrap(
      goal_object_pose.z() - recovery_terminal_object_pose.z()));
  receipt.orientation_debt =
      recovery_orientation_error - cycle_start_orientation_error;
  receipt.lateral_restored =
      receipt.lateral_error <= lateral_drift_tolerance &&
      receipt.lateral_reduction > 0.0;
  receipt.translation_nonregressive = receipt.translation_progress >= 0.0;
  receipt.translation_debt_bounded =
      -receipt.translation_progress <= translation_tolerance;
  receipt.orientation_debt_bounded =
      receipt.orientation_debt <= orientation_tolerance;
  receipt.accepted = receipt.lateral_restored &&
      receipt.translation_debt_bounded && receipt.orientation_debt_bounded;
  receipt.normalized_magnitude =
      receipt.lateral_reduction / lateral_drift_tolerance +
      receipt.translation_progress / translation_tolerance -
      std::max(0.0, receipt.orientation_debt) / orientation_tolerance;
  return receipt;
}

XarmFullSamplingC3LateralEntryReceipt
FindXarmFullSamplingC3LateralCorridorEntry(
    const XarmFullSamplingC3CandidateReceipt& candidate,
    double goal_object_x, double lateral_drift_tolerance) {
  if (!std::isfinite(goal_object_x) ||
      !std::isfinite(lateral_drift_tolerance) ||
      lateral_drift_tolerance < 0.0) {
    throw std::invalid_argument("lateral entry inputs are invalid");
  }
  XarmFullSamplingC3LateralEntryReceipt receipt;
  for (int knot = 1;
       knot < static_cast<int>(
                  candidate.solve.dynamic_state_trajectory.size());
       ++knot) {
    const Eigen::VectorXd& encoded =
        candidate.solve.dynamic_state_trajectory[knot];
    if (encoded.size() != XarmFullSamplingC3State::kSize ||
        !encoded.allFinite()) {
      return {};
    }
    const auto state = XarmFullSamplingC3State::Decode(encoded);
    if (std::abs(state.object_position_W.x() - goal_object_x) >
        lateral_drift_tolerance) {
      continue;
    }
    const Eigen::Matrix3d R_WO =
        state.object_quaternion_WO.toRotationMatrix();
    receipt.object_pose.head<2>() = state.object_position_W.head<2>();
    receipt.object_pose.z() = std::atan2(R_WO(1, 0), R_WO(0, 0));
    receipt.state_knot = knot;
    receipt.accepted = receipt.object_pose.allFinite();
    return receipt;
  }
  return receipt;
}

XarmFullSamplingC3TerminalBudgetEstimate
EstimateXarmFullSamplingC3TerminalBudget(
    const Eigen::Vector3d& current_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance, double orientation_tolerance,
    const std::vector<XarmFullSamplingC3MeasuredCycleReceipt>& cycles) {
  if (!current_object_pose.allFinite() || !goal_object_pose.allFinite() ||
      !std::isfinite(translation_tolerance) ||
      !std::isfinite(orientation_tolerance) ||
      translation_tolerance <= 0.0 || orientation_tolerance <= 0.0) {
    throw std::invalid_argument("terminal budget inputs are invalid");
  }
  auto wrap = [](double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  };
  XarmFullSamplingC3TerminalBudgetEstimate estimate;
  estimate.required_translation_progress = std::max(
      0.0, (current_object_pose.head<2>() -
            goal_object_pose.head<2>()).norm() - translation_tolerance);
  estimate.required_orientation_progress = std::max(
      0.0, std::abs(wrap(goal_object_pose.z() - current_object_pose.z())) -
          orientation_tolerance);
  estimate.minimum_measured_cycle_updates =
      std::numeric_limits<int>::max();
  for (const auto& cycle : cycles) {
    if (!std::isfinite(cycle.translation_progress) ||
        !std::isfinite(cycle.orientation_progress) || cycle.updates <= 0) {
      throw std::invalid_argument("measured cycle receipt is invalid");
    }
    estimate.maximum_measured_translation_progress = std::max(
        estimate.maximum_measured_translation_progress,
        cycle.translation_progress);
    estimate.maximum_measured_orientation_progress = std::max(
        estimate.maximum_measured_orientation_progress,
        cycle.orientation_progress);
    estimate.minimum_measured_cycle_updates = std::min(
        estimate.minimum_measured_cycle_updates, cycle.updates);
  }
  const bool translation_observed =
      estimate.required_translation_progress == 0.0 ||
      estimate.maximum_measured_translation_progress > 0.0;
  const bool orientation_observed =
      estimate.required_orientation_progress == 0.0 ||
      estimate.maximum_measured_orientation_progress > 0.0;
  if (cycles.empty() || !translation_observed || !orientation_observed) {
    estimate.minimum_measured_cycle_updates = 0;
    return estimate;
  }
  const int translation_cycles = estimate.required_translation_progress == 0.0
      ? 0
      : static_cast<int>(std::ceil(
            estimate.required_translation_progress /
            estimate.maximum_measured_translation_progress));
  const int orientation_cycles = estimate.required_orientation_progress == 0.0
      ? 0
      : static_cast<int>(std::ceil(
            estimate.required_orientation_progress /
            estimate.maximum_measured_orientation_progress));
  estimate.optimistic_remaining_cycles =
      std::max(translation_cycles, orientation_cycles);
  estimate.optimistic_remaining_updates =
      static_cast<int64_t>(estimate.optimistic_remaining_cycles) *
      estimate.minimum_measured_cycle_updates;
  estimate.finite = true;
  return estimate;
}

XarmFullSamplingC3TerminalBudgetSufficiencyReceipt
EvaluateXarmFullSamplingC3TerminalBudgetSufficiency(
    int updates_used, int update_budget,
    const XarmFullSamplingC3TerminalBudgetEstimate& estimate) {
  if (updates_used < 0 || update_budget <= 0 ||
      updates_used > update_budget || estimate.optimistic_remaining_updates < 0) {
    throw std::invalid_argument(
        "terminal budget sufficiency inputs are invalid");
  }
  XarmFullSamplingC3TerminalBudgetSufficiencyReceipt receipt;
  receipt.remaining_updates = update_budget - updates_used;
  receipt.optimistic_required_updates = estimate.optimistic_remaining_updates;
  receipt.finite_estimate = estimate.finite;
  receipt.sufficient = estimate.finite &&
      receipt.remaining_updates >= receipt.optimistic_required_updates;
  return receipt;
}

}  // namespace dairlib::oim
