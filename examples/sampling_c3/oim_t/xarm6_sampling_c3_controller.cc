#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <dairlib/lcmt_robot_output.hpp>
#include <dairlib/lcmt_object_state.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/inverse_kinematics/differential_inverse_kinematics.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/solve.h>
#include <gflags/gflags.h>

#include "core/c3_plus.h"
#include "core/lcs.h"

#include "examples/sampling_c3/oim_t/xarm6_full_sampling_c3plus.h"
#include "examples/sampling_c3/oim_t/xarm6_process_common.h"
#include "lcm/lcm_trajectory.h"
#include "systems/framework/lcm_driven_loop.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
              "Canonical OIM-T configuration");
DEFINE_string(planner_mode, "reduced_exact_t",
              "Planner implementation: reduced_exact_t or "
              "full_sampling_c3plus");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0", "LCM URL");
DEFINE_double(smoke_offset_x, 0.0,
              "Publish a task-space smoke target this many meters in +x");
DEFINE_double(smoke_offset_y, 0.0,
              "Publish a task-space smoke target this many meters in +y");
DEFINE_double(smoke_offset_z, 0.0,
              "Publish a task-space smoke target this many meters in +z");
DEFINE_int32(publish_count, 100, "Number of trajectory messages to publish");
DEFINE_int32(full_execution_steps, 2000,
             "Maximum measured-state waypoint waits in full mode");
DEFINE_int32(full_execution_period_ms, 20,
             "Measured-state refresh period for full-mode waypoint gates");
DEFINE_bool(first_solve_only, false,
            "Run one exact-T linearized Sampling-C3+ solve and exit");
DEFINE_bool(live_sampled_plan, false,
            "Sample exact-T boundary contacts from live state and publish");
DEFINE_int32(live_control_steps, 1,
             "Number of live sampled-plan control steps");
DEFINE_int32(live_step_period_ms, 500,
             "State refresh and target hold period for each live step");
DEFINE_int32(live_log_every, 100, "Log every N live control steps");
DEFINE_int32(initial_contact_sample_index, -1,
             "Diagnostic exact-T sample override; -1 uses goal-facing acquisition");
DEFINE_int32(force_successor_after_contact_steps, 0,
             "Diagnostic forced face switch after N contact steps; zero disables");
DEFINE_int32(diagnostic_successor_sample_index, -1,
             "Diagnostic forced successor sample; -1 uses ranked selection");
DEFINE_double(contact_normal_step_override,
              std::numeric_limits<double>::quiet_NaN(),
              "Diagnostic contact normal-step override in meters");

namespace dairlib::oim {

struct ContactSolveResult {
  bool finite{};
  double elapsed{};
  double gap{};
  Eigen::Vector2d pusher_target;
  Eigen::Vector3d object_prediction;
};

enum class ContactFace {
  kCrossbarTop,
  kCrossbarLeft,
  kCrossbarRight,
  kStemLeft,
  kStemRight,
  kStemBottom,
};

struct ContactSample {
  Eigen::Vector2d point;
  Eigen::Vector2d normal;
  const char* name;
  ContactFace face;
};

// Geometry copied from the imported MJCF/SDF collision elements.  These are
// deliberately not tunable controller margins: the planner receipt is meant to
// describe the same pusher capsule and exact two-box T that Drake simulates.
constexpr double kCapsuleStartZ = 0.00555;
constexpr double kCapsuleEndZ = 0.17385;
constexpr double kCrossbarCenterY = 0.0099;
constexpr double kCrossbarHalfX = 0.0445;
constexpr double kCrossbarHalfY = 0.0099;
constexpr double kStemCenterY = -0.0397;
constexpr double kStemHalfX = 0.0099;
constexpr double kStemHalfY = 0.0397;
constexpr double kTHalfHeight = 0.0298;
constexpr int kSweptPostureSamples = 8;
constexpr double kMaximumVerticalAxisError = 0.05;

double CapsuleOrientationClearanceHeight(const OimTParams& params) {
  const double tip_local_z = params.robot.end_effector_point.z();
  const double maximum_tip_to_capsule_center_distance = std::max(
      std::abs(tip_local_z - kCapsuleStartZ),
      std::abs(tip_local_z - kCapsuleEndZ));
  return std::max(
      params.controller.reposition_waypoint_height,
      maximum_tip_to_capsule_center_distance +
          params.controller.pusher_radius);
}

double CapsuleObjectClearanceHeight(const OimTParams& params) {
  return CapsuleOrientationClearanceHeight(params) + 2.0 * kTHalfHeight +
      params.controller.contact_activation_tolerance;
}

struct RepositionCollisionReceipt {
  bool capsule_t_clear{true};
  bool capsule_table_clear{true};
  bool tip_t_clear{true};
  bool tip_table_clear{true};
  double capsule_table_margin{std::numeric_limits<double>::infinity()};
  double tip_table_margin{std::numeric_limits<double>::infinity()};

  bool collision_free() const {
    return capsule_t_clear && capsule_table_clear && tip_t_clear &&
        tip_table_clear;
  }
};

bool SegmentIntersectsAabb(const Eigen::Vector3d& p0,
                           const Eigen::Vector3d& p1,
                           const Eigen::Vector3d& lower,
                           const Eigen::Vector3d& upper) {
  double t_min = 0.0;
  double t_max = 1.0;
  const Eigen::Vector3d direction = p1 - p0;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < 1.0e-14) {
      if (p0[axis] < lower[axis] || p0[axis] > upper[axis]) return false;
      continue;
    }
    double entry = (lower[axis] - p0[axis]) / direction[axis];
    double exit = (upper[axis] - p0[axis]) / direction[axis];
    if (entry > exit) std::swap(entry, exit);
    t_min = std::max(t_min, entry);
    t_max = std::min(t_max, exit);
    if (t_min > t_max) return false;
  }
  return true;
}

bool PointInsideAabb(const Eigen::Vector3d& point,
                     const Eigen::Vector3d& lower,
                     const Eigen::Vector3d& upper) {
  return (point.array() >= lower.array()).all() &&
      (point.array() <= upper.array()).all();
}

RepositionCollisionReceipt EvaluateRepositionCollision(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, const OimTParams& params,
    const Eigen::VectorXd& q, const Eigen::Vector3d& object_xyz_yaw) {
  plant.SetPositions(context, q);
  const auto& end_effector =
      plant.GetBodyByName(params.robot.end_effector_body);
  const auto X_WEe = plant.EvalBodyPoseInWorld(*context, end_effector);
  const Eigen::Vector3d capsule_start_W =
      X_WEe * Eigen::Vector3d(0.0, 0.0, kCapsuleStartZ);
  const Eigen::Vector3d capsule_end_W =
      X_WEe * Eigen::Vector3d(0.0, 0.0, kCapsuleEndZ);
  const Eigen::Vector3d tip_W = X_WEe * params.robot.end_effector_point;
  const Eigen::Rotation2Dd R_OW(-object_xyz_yaw.z());
  auto to_object = [&](const Eigen::Vector3d& point_W) {
    Eigen::Vector3d point_O;
    point_O.head<2>() =
        R_OW * (point_W.head<2>() - object_xyz_yaw.head<2>());
    point_O.z() = point_W.z() - params.object.resting_height;
    return point_O;
  };
  const Eigen::Vector3d capsule_start_O = to_object(capsule_start_W);
  const Eigen::Vector3d capsule_end_O = to_object(capsule_end_W);
  const Eigen::Vector3d tip_O = to_object(tip_W);

  RepositionCollisionReceipt receipt;
  receipt.capsule_table_margin =
      std::min(capsule_start_W.z(), capsule_end_W.z()) -
      params.controller.pusher_radius;
  receipt.tip_table_margin = tip_W.z();
  receipt.capsule_table_clear = receipt.capsule_table_margin >= 0.0;
  receipt.tip_table_clear = receipt.tip_table_margin >= 0.0;
  const std::array<Eigen::Vector3d, 2> centers = {
      Eigen::Vector3d(0.0, kCrossbarCenterY, 0.0),
      Eigen::Vector3d(0.0, kStemCenterY, 0.0)};
  const std::array<Eigen::Vector3d, 2> half_extents = {
      Eigen::Vector3d(kCrossbarHalfX, kCrossbarHalfY, kTHalfHeight),
      Eigen::Vector3d(kStemHalfX, kStemHalfY, kTHalfHeight)};
  for (int box = 0; box < 2; ++box) {
    const Eigen::Vector3d expanded = half_extents[box] +
        Eigen::Vector3d::Constant(params.controller.pusher_radius);
    receipt.capsule_t_clear = receipt.capsule_t_clear &&
        !SegmentIntersectsAabb(capsule_start_O, capsule_end_O,
                               centers[box] - expanded,
                               centers[box] + expanded);
    receipt.tip_t_clear = receipt.tip_t_clear &&
        !PointInsideAabb(tip_O, centers[box] - half_extents[box],
                        centers[box] + half_extents[box]);
  }
  return receipt;
}

struct ContactCapsuleClearanceReceipt {
  bool shaft_t_clear{true};
  bool capsule_table_clear{true};
  bool tip_table_clear{true};
  double capsule_table_margin{std::numeric_limits<double>::infinity()};

  bool clear() const {
    return shaft_t_clear && capsule_table_clear && tip_table_clear;
  }
};

ContactCapsuleClearanceReceipt EvaluateContactCapsuleClearance(
    const OimTParams& params, const Eigen::Vector3d& tip_W,
    const Eigen::Vector3d& stick_axis_W,
    const Eigen::Vector3d& object_xyz_yaw,
    const ContactSample& allowed_contact) {
  ContactCapsuleClearanceReceipt receipt;
  if (!tip_W.allFinite() || !stick_axis_W.allFinite() ||
      stick_axis_W.norm() < 1.0e-12) {
    receipt.shaft_t_clear = false;
    return receipt;
  }
  const Eigen::Vector3d axis_W = stick_axis_W.normalized();
  const double tip_local_z = params.robot.end_effector_point.z();
  const Eigen::Vector3d capsule_start_W =
      tip_W - axis_W * (tip_local_z - kCapsuleStartZ);
  const Eigen::Vector3d capsule_end_W =
      tip_W - axis_W * (tip_local_z - kCapsuleEndZ);
  // The distal activation band is omitted from the conservative shaft
  // segment.  A selected-face exception below also admits OIM's intended
  // vertical cylindrical side contact; every other radius-expanded
  // centerline intersection remains a collision.
  const Eigen::Vector3d shaft_end_W = capsule_end_W -
      axis_W * params.controller.contact_activation_tolerance;
  receipt.capsule_table_margin =
      std::min(capsule_start_W.z(), shaft_end_W.z()) -
      params.controller.pusher_radius;
  receipt.capsule_table_clear = receipt.capsule_table_margin >= 0.0;
  receipt.tip_table_clear = tip_W.z() >= 0.0;

  const Eigen::Rotation2Dd R_OW(-object_xyz_yaw.z());
  auto to_object = [&](const Eigen::Vector3d& point_W) {
    Eigen::Vector3d point_O;
    point_O.head<2>() =
        R_OW * (point_W.head<2>() - object_xyz_yaw.head<2>());
    point_O.z() = point_W.z() - params.object.resting_height;
    return point_O;
  };
  const Eigen::Vector3d shaft_start_O = to_object(capsule_start_W);
  const Eigen::Vector3d shaft_end_O = to_object(shaft_end_W);
  const std::array<Eigen::Vector3d, 2> centers = {
      Eigen::Vector3d(0.0, kCrossbarCenterY, 0.0),
      Eigen::Vector3d(0.0, kStemCenterY, 0.0)};
  const std::array<Eigen::Vector3d, 2> half_extents = {
      Eigen::Vector3d(kCrossbarHalfX, kCrossbarHalfY, kTHalfHeight),
      Eigen::Vector3d(kStemHalfX, kStemHalfY, kTHalfHeight)};
  const int allowed_box =
      allowed_contact.face == ContactFace::kCrossbarTop ||
              allowed_contact.face == ContactFace::kCrossbarLeft ||
              allowed_contact.face == ContactFace::kCrossbarRight
          ? 0
          : 1;
  const double minimum_allowed_normal_distance =
      params.controller.pusher_radius -
      params.controller.contact_activation_tolerance;
  const double start_normal_distance = allowed_contact.normal.dot(
      shaft_start_O.head<2>() - allowed_contact.point);
  const double end_normal_distance = allowed_contact.normal.dot(
      shaft_end_O.head<2>() - allowed_contact.point);
  const bool selected_face_band_clear =
      std::min(start_normal_distance, end_normal_distance) >=
      minimum_allowed_normal_distance;
  for (int box = 0; box < 2; ++box) {
    const Eigen::Vector3d expanded = half_extents[box] +
        Eigen::Vector3d::Constant(params.controller.pusher_radius);
    const bool intersects = SegmentIntersectsAabb(
        shaft_start_O, shaft_end_O, centers[box] - expanded,
        centers[box] + expanded);
    // A vertical OIM stick intentionally contacts a side with its cylindrical
    // surface, not only its distal hemisphere. Permit that intersection only
    // for the selected finite T box and only while the entire checked shaft
    // remains outside the selected face minus the existing activation band.
    // An inclined shaft crossing the top, the opposite face, or the other T
    // box still fails this same conservative centerline test.
    receipt.shaft_t_clear = receipt.shaft_t_clear &&
        (!intersects || (box == allowed_box && selected_face_band_clear));
  }
  return receipt;
}

ContactCapsuleClearanceReceipt EvaluateSweptContactCapsuleClearance(
    const OimTParams& params, const Eigen::Vector3d& start_tip_W,
    const Eigen::Vector3d& target_tip_W,
    const Eigen::Vector3d& start_axis_W,
    const Eigen::Vector3d& target_axis_W,
    const Eigen::Vector3d& object_xyz_yaw,
    const ContactSample& allowed_contact) {
  ContactCapsuleClearanceReceipt swept;
  for (int sample = 0; sample <= kSweptPostureSamples; ++sample) {
    const double alpha = static_cast<double>(sample) /
        static_cast<double>(kSweptPostureSamples);
    const Eigen::Vector3d tip_W =
        start_tip_W + alpha * (target_tip_W - start_tip_W);
    Eigen::Vector3d axis_W =
        start_axis_W + alpha * (target_axis_W - start_axis_W);
    if (axis_W.norm() > 1.0e-12) axis_W.normalize();
    const auto receipt = EvaluateContactCapsuleClearance(
        params, tip_W, axis_W, object_xyz_yaw, allowed_contact);
    swept.shaft_t_clear = swept.shaft_t_clear && receipt.shaft_t_clear;
    swept.capsule_table_clear =
        swept.capsule_table_clear && receipt.capsule_table_clear;
    swept.tip_table_clear = swept.tip_table_clear && receipt.tip_table_clear;
    swept.capsule_table_margin = std::min(
        swept.capsule_table_margin, receipt.capsule_table_margin);
  }
  return swept;
}

struct ContactPostureResult {
  bool ik_solved{};
  bool finite{};
  bool home_seed_fallback{};
  Eigen::VectorXd target;
  ContactCapsuleClearanceReceipt receipt;
  double target_tip_error{std::numeric_limits<double>::infinity()};
  double target_axis_error{std::numeric_limits<double>::infinity()};
};

ContactPostureResult SolveVerticalContactPostureStep(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, const OimTParams& params,
    const Eigen::VectorXd& measured_q, const Eigen::Vector3d& desired_tip_W,
    const Eigen::Vector3d& object_xyz_yaw,
    const ContactSample& allowed_contact,
    bool allow_home_seed_fallback = false) {
  const auto& end_effector =
      plant.GetBodyByName(params.robot.end_effector_body);
  plant.SetPositions(context, measured_q);
  const auto X_WCurrent =
      plant.EvalBodyPoseInWorld(*context, end_effector);
  const double current_tip_error =
      (X_WCurrent * params.robot.end_effector_point - desired_tip_W).norm();
  const Eigen::Vector3d current_axis_W =
      X_WCurrent.rotation() * Eigen::Vector3d::UnitZ();
  const double current_axis_error = std::acos(std::clamp(
      current_axis_W.dot(-Eigen::Vector3d::UnitZ()), -1.0, 1.0));
  drake::multibody::InverseKinematics ik(plant, context);
  const Eigen::Vector3d position_tolerance =
      Eigen::Vector3d::Constant(1.0e-6);
  ik.AddPositionConstraint(
      end_effector.body_frame(), params.robot.end_effector_point,
      plant.world_frame(), desired_tip_W - position_tolerance,
      desired_tip_W + position_tolerance);
  // The source OIM controller's large tilt cost keeps the pushing stick
  // vertical.  Constrain only its axis; wrist roll remains unavailable and
  // irrelevant for the axisymmetric capsule.
  ik.AddAngleBetweenVectorsConstraint(
      end_effector.body_frame(), Eigen::Vector3d::UnitZ(),
      plant.world_frame(), -Eigen::Vector3d::UnitZ(), 0.0,
      kMaximumVerticalAxisError);
  ik.get_mutable_prog()->AddQuadraticErrorCost(
      Eigen::MatrixXd::Identity(measured_q.size(), measured_q.size()),
      measured_q, ik.q());
  ik.get_mutable_prog()->SetInitialGuess(ik.q(), measured_q);
  ContactPostureResult posture;
  posture.target = measured_q;
  drake::solvers::MathematicalProgramResult result =
      drake::solvers::Solve(ik.prog());
  if (!result.is_success() && allow_home_seed_fallback) {
    ik.get_mutable_prog()->SetInitialGuess(
        ik.q(), params.robot.home_positions);
    result = drake::solvers::Solve(ik.prog());
    posture.home_seed_fallback = result.is_success();
  }
  if (!result.is_success()) return posture;
  posture.ik_solved = true;
  const Eigen::VectorXd canonical_solution =
      CanonicalizeXarmPeriodicIkSolutionNearestMeasured(
          result.GetSolution(ik.q()), measured_q,
          plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits());
  if ((canonical_solution - result.GetSolution(ik.q())).norm() > 1.0e-12) {
    std::cout << "full_sampling_c3plus_periodic_ik_branch=PASS mode=vertical"
              << " raw_q=" << result.GetSolution(ik.q()).transpose()
              << " canonical_q=" << canonical_solution.transpose()
              << " measured_q=" << measured_q.transpose() << std::endl;
  }
  Eigen::VectorXd joint_step = canonical_solution - measured_q;
  for (int i = 0; i < joint_step.size(); ++i) {
    const double step_limit =
        params.robot.velocity_limits[i] * params.task.planning_time_step;
    joint_step[i] = std::clamp(joint_step[i], -step_limit, step_limit);
  }
  posture.target = measured_q + joint_step;
  posture.finite = posture.target.allFinite();
  for (int sample = 0; sample <= kSweptPostureSamples && posture.finite;
       ++sample) {
    const double alpha = static_cast<double>(sample) /
        static_cast<double>(kSweptPostureSamples);
    const Eigen::VectorXd q =
        measured_q + alpha * (posture.target - measured_q);
    plant.SetPositions(context, q);
    const auto X_WEe = plant.EvalBodyPoseInWorld(*context, end_effector);
    const Eigen::Vector3d tip_W =
        X_WEe * params.robot.end_effector_point;
    const Eigen::Vector3d axis_W =
        X_WEe.rotation() * Eigen::Vector3d::UnitZ();
    const auto receipt = EvaluateContactCapsuleClearance(
        params, tip_W, axis_W, object_xyz_yaw, allowed_contact);
    posture.receipt.shaft_t_clear =
        posture.receipt.shaft_t_clear && receipt.shaft_t_clear;
    posture.receipt.capsule_table_clear =
        posture.receipt.capsule_table_clear && receipt.capsule_table_clear;
    posture.receipt.tip_table_clear =
        posture.receipt.tip_table_clear && receipt.tip_table_clear;
    posture.receipt.capsule_table_margin = std::min(
        posture.receipt.capsule_table_margin,
        receipt.capsule_table_margin);
    posture.finite = posture.receipt.clear();
  }
  if (posture.finite) {
    plant.SetPositions(context, posture.target);
    const auto X_WEe = plant.EvalBodyPoseInWorld(*context, end_effector);
    posture.target_tip_error =
        (X_WEe * params.robot.end_effector_point - desired_tip_W).norm();
    const Eigen::Vector3d target_axis_W =
        X_WEe.rotation() * Eigen::Vector3d::UnitZ();
    posture.target_axis_error = std::acos(std::clamp(
        target_axis_W.dot(-Eigen::Vector3d::UnitZ()), -1.0, 1.0));
    if (posture.home_seed_fallback) {
      posture.finite =
          posture.target_axis_error < current_axis_error &&
          posture.target_tip_error <= current_tip_error +
              params.controller.contact_activation_tolerance;
    }
  }
  plant.SetPositions(context, measured_q);
  return posture;
}

struct FullCandidateAcquisitionReceipt {
  bool swept_capsule_clear{};
  bool used_controlled_escape{};
  bool ik_reached{};
  int ik_steps{};
  int failed_waypoint{-1};
  bool used_home_seed_fallback{};
  bool failure_ik_solved{};
  ContactCapsuleClearanceReceipt failure_clearance;
};

FullCandidateAcquisitionReceipt EvaluateFullCandidateAcquisition(
    const OimTParams& params,
    const XarmFullSamplingC3CandidateReceipt& candidate) {
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModels(params.robot.model);
  AddXarmActuators(params, &plant);
  plant.Finalize();
  ValidateXarmPlant(plant, params);
  auto context = plant.CreateDefaultContext();
  SetXarmHome(params, plant, context.get());
  const auto& ee = plant.GetBodyByName(params.robot.end_effector_body);
  Eigen::VectorXd q = plant.GetPositions(*context);
  const auto home_pose = plant.EvalBodyPoseInWorld(*context, ee);
  const Eigen::Vector3d home_tip =
      home_pose * params.robot.end_effector_point;
  const Eigen::Vector3d home_axis =
      home_pose.rotation() * Eigen::Vector3d::UnitZ();
  const Eigen::Rotation2Dd R_WO(params.object.start_pose.z());
  const Eigen::Vector2d normal_W = R_WO * candidate.sample_normal_O;
  const Eigen::Vector3d contact = candidate.initial_pusher_position_W;
  Eigen::Vector3d standoff = contact;
  standoff.head<2>() += normal_W *
      (params.controller.descent_clearance +
       0.5 * params.controller.contact_activation_tolerance);
  Eigen::Vector3d high = standoff;
  high.z() = params.controller.reposition_waypoint_height;
  Eigen::Vector3d planar = high;
  planar.z() = home_tip.z();
  ContactFace face = ContactFace::kCrossbarTop;
  if (candidate.sample_name.find("stem_left") != std::string::npos) {
    face = ContactFace::kStemLeft;
  } else if (candidate.sample_name.find("stem_right") != std::string::npos) {
    face = ContactFace::kStemRight;
  } else if (candidate.sample_name.find("stem_bottom") != std::string::npos) {
    face = ContactFace::kStemBottom;
  } else if (candidate.sample_name.find("crossbar_left") !=
             std::string::npos) {
    face = ContactFace::kCrossbarLeft;
  } else if (candidate.sample_name.find("crossbar_right") !=
             std::string::npos) {
    face = ContactFace::kCrossbarRight;
  }
  const ContactSample sample{candidate.sample_point_O,
                             candidate.sample_normal_O,
                             candidate.sample_name.c_str(), face};
  const Eigen::Vector3d vertical_axis = -Eigen::Vector3d::UnitZ();
  FullCandidateAcquisitionReceipt receipt;
  receipt.swept_capsule_clear =
      EvaluateSweptContactCapsuleClearance(
          params, home_tip, planar, home_axis, vertical_axis,
          params.object.start_pose, sample).clear() &&
      EvaluateSweptContactCapsuleClearance(
          params, planar, high, vertical_axis, vertical_axis,
          params.object.start_pose, sample).clear() &&
      EvaluateSweptContactCapsuleClearance(
          params, high, standoff, vertical_axis, vertical_axis,
          params.object.start_pose, sample).clear() &&
      EvaluateSweptContactCapsuleClearance(
          params, standoff, contact, vertical_axis, vertical_axis,
          params.object.start_pose, sample).clear();
  const std::array<Eigen::Vector3d, 4> waypoints = {
      planar, high, standoff, contact};
  receipt.ik_reached = true;
  for (int waypoint_index = 0; waypoint_index < 4; ++waypoint_index) {
    bool reached = false;
    for (int step = 0; step < 500; ++step) {
      plant.SetPositions(context.get(), q);
      const Eigen::Vector3d tip =
          plant.EvalBodyPoseInWorld(*context, ee) *
          params.robot.end_effector_point;
      if ((tip - waypoints[waypoint_index]).norm() <=
          params.controller.contact_activation_tolerance) {
        reached = true;
        break;
      }
      const auto posture = SolveVerticalContactPostureStep(
          plant, context.get(), params, q, waypoints[waypoint_index],
          params.object.start_pose, sample);
      if (!posture.finite || (posture.target - q).norm() <= 1.0e-12) break;
      q = posture.target;
      ++receipt.ik_steps;
    }
    if (!reached) {
      receipt.ik_reached = false;
      receipt.failed_waypoint = waypoint_index;
      break;
    }
  }
  return receipt;
}

struct CollisionAwarePostureResult {
  bool finite{};
  Eigen::VectorXd target;
  RepositionCollisionReceipt receipt;
  double target_shaft_clearance{
      -std::numeric_limits<double>::infinity()};
  double target_tip_error{
      std::numeric_limits<double>::infinity()};
};

std::optional<Eigen::VectorXd> SolveLiftPostureStep(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, const OimTParams& params,
    const Eigen::VectorXd& measured_q, const Eigen::Vector3d& desired_tip_W) {
  const auto& end_effector =
      plant.GetBodyByName(params.robot.end_effector_body);
  // Release begins at an intentional physical face contact, so the general
  // collision receipt cannot reject its first swept sample. Generate a local
  // velocity-bounded differential-IK step around the measured posture. A
  // distant nonlinear IK solution followed by independent joint clipping is
  // not a valid local Cartesian step and can reverse direction near a
  // singularity.
  plant.SetPositions(context, measured_q);
  const Eigen::Vector3d measured_tip_W =
      plant.EvalBodyPoseInWorld(*context, end_effector) *
      params.robot.end_effector_point;
  const double dt = params.task.planning_time_step;
  Eigen::Vector3d desired_velocity =
      (desired_tip_W - measured_tip_W) / dt;
  if (desired_velocity.norm() >
      params.controller.descent_diff_ik_max_velocity) {
    desired_velocity *=
        params.controller.descent_diff_ik_max_velocity /
        desired_velocity.norm();
  }
  Eigen::MatrixXd J(3, plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      *context, drake::multibody::JacobianWrtVariable::kV,
      end_effector.body_frame(), params.robot.end_effector_point,
      plant.world_frame(), plant.world_frame(), &J);
  drake::multibody::DifferentialInverseKinematicsParameters diff_ik(
      plant.num_positions(), plant.num_velocities());
  diff_ik.set_time_step(dt);
  diff_ik.set_nominal_joint_position(measured_q);
  diff_ik.set_joint_centering_gain(
      params.controller.descent_diff_ik_centering_gain *
      Eigen::MatrixXd::Identity(plant.num_positions(),
                                plant.num_positions()));
  diff_ik.set_joint_position_limits(
      {plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()});
  diff_ik.set_joint_velocity_limits(
      {-params.robot.velocity_limits, params.robot.velocity_limits});
  const auto result =
      drake::multibody::DoDifferentialInverseKinematics(
          measured_q, Eigen::VectorXd::Zero(plant.num_velocities()),
          desired_velocity, J, diff_ik);
  if (!result.joint_velocities.has_value()) return std::nullopt;
  Eigen::VectorXd target = measured_q + dt * *result.joint_velocities;
  if (!target.allFinite()) return std::nullopt;
  plant.SetPositions(context, target);
  const Eigen::Vector3d predicted_tip_W =
      plant.EvalBodyPoseInWorld(*context, end_effector) *
      params.robot.end_effector_point;
  plant.SetPositions(context, measured_q);
  if ((desired_tip_W - predicted_tip_W).norm() >=
      (desired_tip_W - measured_tip_W).norm()) {
    return std::nullopt;
  }
  return target;
}

FullCandidateAcquisitionReceipt EvaluateMeasuredCandidateAcquisition(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, const OimTParams& params,
    const Eigen::VectorXd& measured_q,
    const Eigen::Vector3d& object_xyz_yaw,
    const XarmFullSamplingC3CandidateReceipt& candidate,
    const ContactSample& release_contact,
    bool use_neutral_anchor = false) {
  ContactFace face = ContactFace::kCrossbarTop;
  if (candidate.sample_name.find("stem_right") != std::string::npos) {
    face = ContactFace::kStemRight;
  } else if (candidate.sample_name.find("stem_left") != std::string::npos) {
    face = ContactFace::kStemLeft;
  } else if (candidate.sample_name.find("stem_bottom") != std::string::npos) {
    face = ContactFace::kStemBottom;
  } else if (candidate.sample_name.find("crossbar_right") !=
             std::string::npos) {
    face = ContactFace::kCrossbarRight;
  } else if (candidate.sample_name.find("crossbar_left") !=
             std::string::npos) {
    face = ContactFace::kCrossbarLeft;
  }
  const ContactSample sample{candidate.sample_point_O,
                             candidate.sample_normal_O,
                             candidate.sample_name.c_str(), face};
  const Eigen::Rotation2Dd R_WO(object_xyz_yaw.z());
  const Eigen::Vector2d normal_W = R_WO * candidate.sample_normal_O;
  Eigen::Vector3d contact;
  contact.head<2>() = object_xyz_yaw.head<2>() +
      R_WO * candidate.sample_point_O +
      normal_W * (params.controller.pusher_radius -
                  0.5 * params.controller.contact_activation_tolerance);
  contact.z() = params.object.resting_height + candidate.sample_height_O;
  Eigen::Vector3d standoff = contact;
  standoff.head<2>() += normal_W *
      (params.controller.descent_clearance +
       0.5 * params.controller.contact_activation_tolerance);
  Eigen::Vector3d high = standoff;
  high.z() = params.controller.reposition_waypoint_height;
  const auto& end_effector =
      plant.GetBodyByName(params.robot.end_effector_body);
  plant.SetPositions(context, measured_q);
  Eigen::Vector3d lift =
      plant.EvalBodyPoseInWorld(*context, end_effector) *
      params.robot.end_effector_point;
  // A measured tip can already be above the nominal reposition plane after
  // a long dwell or release. A lift must never lower that arbitrarily
  // oriented capsule before the vertical-contact posture is established.
  lift.z() = std::max(lift.z(),
                      use_neutral_anchor
                          ? CapsuleObjectClearanceHeight(params)
                          : CapsuleOrientationClearanceHeight(params));
  // The repeated lift waypoint is intentional: waypoint 0 reaches the safe
  // height with position-only IK, then waypoint 1 holds that tip position
  // while establishing the vertical capsule axis before any low traverse.
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<bool> position_only;
  if (use_neutral_anchor) {
    plant.SetPositions(context, params.robot.home_positions);
    Eigen::Vector3d neutral_anchor =
        plant.EvalBodyPoseInWorld(*context, end_effector) *
        params.robot.end_effector_point;
    neutral_anchor.z() = lift.z();
    Eigen::Vector3d candidate_overhead = high;
    candidate_overhead.z() = lift.z();
    waypoints = {lift, neutral_anchor, neutral_anchor, candidate_overhead,
                 high, standoff, contact};
    position_only = {true, true, false, false, false, false, false};
  } else {
    waypoints = {lift, lift, high, standoff, contact};
    position_only = {true, false, false, false, false};
  }
  Eigen::VectorXd q = measured_q;
  FullCandidateAcquisitionReceipt receipt;
  receipt.swept_capsule_clear = true;
  receipt.ik_reached = true;
  for (int waypoint_index = 0;
       waypoint_index < static_cast<int>(waypoints.size());
       ++waypoint_index) {
    bool reached = false;
    for (int step = 0; step < 500; ++step) {
      plant.SetPositions(context, q);
      const auto X_WEe = plant.EvalBodyPoseInWorld(*context, end_effector);
      const Eigen::Vector3d tip =
          X_WEe * params.robot.end_effector_point;
      const Eigen::Vector3d axis_W =
          X_WEe.rotation() * Eigen::Vector3d::UnitZ();
      const double vertical_axis_error = std::acos(std::clamp(
          axis_W.dot(-Eigen::Vector3d::UnitZ()), -1.0, 1.0));
      const double waypoint_error = waypoint_index == 0
          ? std::abs(tip.z() - waypoints[waypoint_index].z())
          : (tip - waypoints[waypoint_index]).norm();
      if (waypoint_error <=
              params.controller.contact_activation_tolerance &&
          (position_only[waypoint_index] ||
           vertical_axis_error <= kMaximumVerticalAxisError)) {
        reached = true;
        break;
      }
      Eigen::Vector3d command_tip = waypoints[waypoint_index];
      if (waypoint_index == 0 && command_tip.z() > tip.z()) {
        command_tip.head<2>() = tip.head<2>();
      }
      Eigen::Vector3d task_step = command_tip - tip;
      if (task_step.norm() >
          params.controller.task_space_plan_step_limit) {
        task_step *= params.controller.task_space_plan_step_limit /
            task_step.norm();
        command_tip = tip + task_step;
      }
      Eigen::VectorXd next_q;
      if (position_only[waypoint_index]) {
        const auto lift_posture = SolveLiftPostureStep(
            plant, context, params, q, command_tip);
        if (!lift_posture.has_value()) break;
        next_q = *lift_posture;
        plant.SetPositions(context, next_q);
        const auto X_WNext =
            plant.EvalBodyPoseInWorld(*context, end_effector);
        const auto swept = EvaluateSweptContactCapsuleClearance(
            params, tip,
            X_WNext * params.robot.end_effector_point,
            X_WEe.rotation() * Eigen::Vector3d::UnitZ(),
            X_WNext.rotation() * Eigen::Vector3d::UnitZ(),
            object_xyz_yaw, release_contact);
        const Eigen::Vector3d next_tip =
            X_WNext * params.robot.end_effector_point;
        const auto endpoint = EvaluateContactCapsuleClearance(
            params, next_tip,
            X_WNext.rotation() * Eigen::Vector3d::UnitZ(),
            object_xyz_yaw, release_contact);
        const Eigen::Vector2d release_normal_W =
            Eigen::Rotation2Dd(object_xyz_yaw.z()) *
            release_contact.normal;
        const double outward_progress =
            (next_tip.head<2>() - tip.head<2>()).dot(release_normal_W);
        const bool controlled_vertical_escape =
            waypoint_index == 0 && endpoint.capsule_table_clear &&
            endpoint.tip_table_clear &&
            (command_tip.head<2>() - tip.head<2>()).squaredNorm() <=
                1.0e-24 &&
            next_tip.z() > tip.z() && outward_progress >= 0.0;
        receipt.used_controlled_escape =
            receipt.used_controlled_escape || controlled_vertical_escape;
        receipt.swept_capsule_clear =
            receipt.swept_capsule_clear &&
            (swept.clear() || controlled_vertical_escape);
        if (!swept.clear() && !controlled_vertical_escape) break;
      } else {
        const auto posture = SolveVerticalContactPostureStep(
            plant, context, params, q, command_tip,
            object_xyz_yaw, sample,
            use_neutral_anchor && waypoint_index == 2);
        receipt.used_home_seed_fallback =
            receipt.used_home_seed_fallback || posture.home_seed_fallback;
        if (!posture.finite) {
          receipt.failure_ik_solved = posture.ik_solved;
          receipt.failure_clearance = posture.receipt;
          break;
        }
        next_q = posture.target;
        receipt.swept_capsule_clear =
            receipt.swept_capsule_clear && posture.receipt.clear();
      }
      if ((next_q - q).norm() <= 1.0e-12) break;
      q = next_q;
      ++receipt.ik_steps;
    }
    if (!reached) {
      receipt.ik_reached = false;
      receipt.failed_waypoint = waypoint_index;
      break;
    }
  }
  plant.SetPositions(context, measured_q);
  return receipt;
}

CollisionAwarePostureResult SolveCollisionAwarePostureStep(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, const OimTParams& params,
    const Eigen::VectorXd& measured_q, const Eigen::Vector3d& desired_tip_W,
    const Eigen::Vector2d& outward_normal_W,
    const Eigen::Vector3d& object_xyz_yaw,
    double minimum_shaft_clearance) {
  const auto& end_effector =
      plant.GetBodyByName(params.robot.end_effector_body);
  drake::multibody::InverseKinematics ik(plant, context);
  const Eigen::Vector3d position_tolerance =
      Eigen::Vector3d::Constant(1.0e-6);
  ik.AddPositionConstraint(
      end_effector.body_frame(), params.robot.end_effector_point,
      plant.world_frame(), desired_tip_W - position_tolerance,
      desired_tip_W + position_tolerance);
  const Eigen::Vector3d desired_axis_W(-outward_normal_W.x(),
                                      -outward_normal_W.y(), 0.0);
  ik.AddAngleBetweenVectorsConstraint(
      end_effector.body_frame(), Eigen::Vector3d::UnitZ(),
      plant.world_frame(), desired_axis_W, 0.0,
      std::acos(std::clamp(minimum_shaft_clearance, 0.0, 1.0)));
  ik.get_mutable_prog()->AddQuadraticErrorCost(
      Eigen::MatrixXd::Identity(measured_q.size(), measured_q.size()),
      measured_q, ik.q());
  ik.get_mutable_prog()->SetInitialGuess(ik.q(), measured_q);
  const auto result = drake::solvers::Solve(ik.prog());
  CollisionAwarePostureResult posture;
  posture.target = measured_q;
  if (!result.is_success()) return posture;
  const Eigen::VectorXd q_solution =
      CanonicalizeXarmPeriodicIkSolutionNearestMeasured(
          result.GetSolution(ik.q()), measured_q,
          plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits());
  if ((q_solution - result.GetSolution(ik.q())).norm() > 1.0e-12) {
    std::cout << "full_sampling_c3plus_periodic_ik_branch=PASS mode=contact"
              << " raw_q=" << result.GetSolution(ik.q()).transpose()
              << " canonical_q=" << q_solution.transpose()
              << " measured_q=" << measured_q.transpose() << std::endl;
  }
  Eigen::VectorXd joint_step = q_solution - measured_q;
  for (int i = 0; i < joint_step.size(); ++i) {
    const double step_limit =
        params.robot.velocity_limits[i] * params.task.planning_time_step;
    joint_step[i] = std::clamp(joint_step[i], -step_limit, step_limit);
  }
  posture.target = measured_q + joint_step;
  posture.finite = posture.target.allFinite();
  if (posture.finite) {
    plant.SetPositions(context, posture.target);
    const auto X_WEe = plant.EvalBodyPoseInWorld(*context, end_effector);
    const Eigen::Vector2d target_stick_axis_W =
        (X_WEe.rotation() * Eigen::Vector3d::UnitZ()).head<2>();
    posture.target_shaft_clearance =
        -outward_normal_W.dot(target_stick_axis_W);
    posture.target_tip_error =
        (X_WEe * params.robot.end_effector_point - desired_tip_W).norm();
  }
  for (int sample = 0; sample <= kSweptPostureSamples && posture.finite;
       ++sample) {
    const double alpha = static_cast<double>(sample) /
        static_cast<double>(kSweptPostureSamples);
    const Eigen::VectorXd q =
        measured_q + alpha * (posture.target - measured_q);
    const auto sample_receipt = EvaluateRepositionCollision(
        plant, context, params, q, object_xyz_yaw);
    posture.receipt.capsule_t_clear =
        posture.receipt.capsule_t_clear && sample_receipt.capsule_t_clear;
    posture.receipt.capsule_table_clear =
        posture.receipt.capsule_table_clear &&
        sample_receipt.capsule_table_clear;
    posture.receipt.tip_t_clear =
        posture.receipt.tip_t_clear && sample_receipt.tip_t_clear;
    posture.receipt.tip_table_clear =
        posture.receipt.tip_table_clear && sample_receipt.tip_table_clear;
    posture.receipt.capsule_table_margin = std::min(
        posture.receipt.capsule_table_margin,
        sample_receipt.capsule_table_margin);
    posture.receipt.tip_table_margin = std::min(
        posture.receipt.tip_table_margin, sample_receipt.tip_table_margin);
    posture.finite = posture.receipt.collision_free();
  }
  plant.SetPositions(context, measured_q);
  return posture;
}

const std::vector<ContactSample>& ExactTSamples() {
  static const std::vector<ContactSample> samples = {
      {{-0.0445, 0.0198}, {0, 1}, "crossbar_top_left",
       ContactFace::kCrossbarTop},
      {{0.0, 0.0198}, {0, 1}, "crossbar_top", ContactFace::kCrossbarTop},
      {{0.0445, 0.0198}, {0, 1}, "crossbar_top_right",
       ContactFace::kCrossbarTop},
      {{-0.0445, 0.0099}, {-1, 0}, "crossbar_left",
       ContactFace::kCrossbarLeft},
      {{0.0445, 0.0099}, {1, 0}, "crossbar_right",
       ContactFace::kCrossbarRight},
      {{-0.0099, -0.0397}, {-1, 0}, "stem_left", ContactFace::kStemLeft},
      {{0.0099, -0.0397}, {1, 0}, "stem_right", ContactFace::kStemRight},
      {{0.0, -0.0794}, {0, -1}, "stem_bottom", ContactFace::kStemBottom}};
  return samples;
}

bool IsUnsafeVerticalSuccessor(const ContactSample& sample) {
  // The first crossbar-top acquisition is independently side-contact proven.
  // Reacquiring this thin crossbar side after rotation repeatedly reaches its
  // upper perimeter before useful force or is rejected by the capsule band.
  // Keep it out of successor arbitration; side/stem faces remain available.
  return sample.face == ContactFace::kCrossbarTop;
}

struct CandidateSelection {
  ContactSolveResult solve;
  int sample_index{-1};
  double cost{std::numeric_limits<double>::infinity()};
  int finite_samples{};
  int lateral_rejections{};
};

double WrappedAngleError(double target, double current) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  const double raw = target - current;
  double error = std::remainder(raw, kTwoPi);
  // The configured OIM goal is +pi. Preserve that intended rotation direction
  // at the otherwise ambiguous branch cut instead of returning -pi.
  if (raw > 0.0 && std::abs(error + kPi) < 1.0e-4) error = kPi;
  return error;
}

bool MeasuredResponseHasWrongPolarity(
    bool recovery_active, int contact_dwell, int required_dwell,
    double current_lateral_error, double recovery_start_error,
    double required_measured_reduction) {
  return recovery_active && contact_dwell >= required_dwell &&
      current_lateral_error - recovery_start_error >=
          required_measured_reduction;
}

double CandidateCost(const OimTParams& params,
                     const Eigen::Vector3d& prediction) {
  const double lateral_error =
      std::abs(prediction.x() - params.object.goal_pose.x());
  const double y_error = prediction.y() - params.object.goal_pose.y();
  const double yaw_error =
      WrappedAngleError(params.object.goal_pose.z(), prediction.z());
  return params.controller.lateral_drift_weight * lateral_error *
          lateral_error +
      y_error * y_error +
      params.controller.yaw_selection_weight * yaw_error * yaw_error;
}

ContactSolveResult SolveOneContact(const OimTParams& params,
                                   const Eigen::Vector2d& pusher,
                                   const Eigen::Vector3d& object_pose,
                                   const Eigen::Vector2d& normal_W,
                                   const Eigen::Vector2d& boundary_W) {
  // State: pusher xy, object xy/yaw, pusher xy velocity, object xy velocity,
  // object yaw rate.  The prior eight-state translational ordering is retained
  // around the inserted yaw/yaw-rate coordinates.
  constexpr int n = 10, k = 2, m = 1, N = 5;
  const double dt = params.task.planning_time_step;
  const double pusher_mass = 1.0;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n);
  A.block<2, 2>(0, 5) = dt * Eigen::MatrixXd::Identity(2, 2);
  A.block<2, 2>(2, 7) = dt * Eigen::MatrixXd::Identity(2, 2);
  A(4, 9) = dt;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, k);
  B(5, 0) = dt / pusher_mass;
  B(6, 1) = dt / pusher_mass;
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n, m);
  D.block<2, 1>(5, 0) = dt / pusher_mass * normal_W;
  D.block<2, 1>(7, 0) = -dt / params.object.mass * normal_W;
  const Eigen::Vector2d object_force_direction = -normal_W;
  const double contact_moment =
      boundary_W.x() * object_force_direction.y() -
      boundary_W.y() * object_force_direction.x();
  D(9, 0) = dt * contact_moment / params.object.planar_moment_inertia;
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(m, n);
  E.block<1, 2>(0, 0) = normal_W.transpose();
  E.block<1, 2>(0, 2) = -normal_W.transpose();
  const Eigen::Vector2d boundary_yaw_derivative(-boundary_W.y(),
                                                 boundary_W.x());
  E(0, 4) = -normal_W.dot(boundary_yaw_derivative);
  // Small compliance keeps the one-contact projection well-conditioned while
  // retaining a stiff unilateral contact for this first linearized solve.
  Eigen::MatrixXd F = 1.0e-3 * Eigen::MatrixXd::Identity(m, m);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, k);
  Eigen::VectorXd c(1);
  c[0] = -normal_W.dot(boundary_W) - params.controller.pusher_radius -
      E(0, 4) * object_pose.z();
  c3::LCS lcs(A, B, D, Eigen::VectorXd::Zero(n), E, F, H, c, N, dt);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
  x0.segment<2>(0) = pusher;
  x0.segment<3>(2) = object_pose;
  Eigen::VectorXd xd = x0;
  xd.segment<2>(2) = params.object.goal_pose.head<2>();
  xd[4] = object_pose.z() +
      WrappedAngleError(params.object.goal_pose.z(), object_pose.z());
  std::vector<Eigen::VectorXd> desired(N + 1, xd);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n);
  Q.diagonal() << 0.01, 0.01, 200.0, 200.0,
      params.controller.object_yaw_cost_weight,
      1.0, 1.0, 0.05, 0.05, 0.05;
  Eigen::MatrixXd R = 0.01 * Eigen::MatrixXd::Identity(k, k);
  const int nz = n + 2 * m + k;
  Eigen::MatrixXd G = Eigen::MatrixXd::Identity(nz, nz);
  Eigen::MatrixXd U = Eigen::MatrixXd::Identity(nz, nz);
  c3::C3::CostMatrices costs(std::vector<Eigen::MatrixXd>(N + 1, Q),
                             std::vector<Eigen::MatrixXd>(N, R),
                             std::vector<Eigen::MatrixXd>(N, G),
                             std::vector<Eigen::MatrixXd>(N, U));
  c3::C3Options options;
  options.admm_iter = 3;
  options.num_threads = 1;
  options.warm_start = false;
  options.end_on_qp_step = true;
  options.scale_lcs = false;
  options.rho_scale = 3.0;
  options.gamma = 1.0;
  options.qp_projection_alpha = 0.01;
  options.qp_projection_scaling = 1.0;
  c3::C3Plus optimizer(lcs, costs, desired, options);
  const auto start = std::chrono::steady_clock::now();
  optimizer.Solve(x0);
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  const auto states = optimizer.GetStateSolution();
  // This pinned C3+ revision stores N states (the terminal knot is omitted
  // from GetStateSolution even though the optimization contains it).
  const auto inputs = optimizer.GetInputSolution();
  bool finite = states.size() == N && inputs.size() == N;
  for (int i = 0; i < N && finite; ++i) finite = states[i].allFinite();
  for (const auto& input : inputs) finite = finite && input.allFinite();
  if (!finite) {
    std::cerr << "C3+ non-finite diagnostic:";
    for (std::size_t i = 0; i < states.size(); ++i) {
      std::cerr << " x" << i << "=" << states[i].transpose();
    }
    for (std::size_t i = 0; i < inputs.size(); ++i) {
      std::cerr << " u" << i << "=" << inputs[i].transpose();
    }
    std::cerr << std::endl;
    return ContactSolveResult{};
  }
  return ContactSolveResult{true, elapsed, (E * x0 + c)[0],
                            states[std::min(2, N - 1)].segment<2>(0),
                            states[N - 1].segment<3>(2)};
}

CandidateSelection SelectContactCandidate(
    const OimTParams& params, const Eigen::Vector2d& live_pusher,
    const Eigen::Vector3d& object_pose, std::optional<ContactFace> required_face,
    std::optional<ContactFace> excluded_face, bool place_pusher_at_contact) {
  const auto& samples = ExactTSamples();
  const Eigen::Rotation2Dd R_WO(object_pose.z());
  const double current_lateral_error =
      std::abs(object_pose.x() - params.object.goal_pose.x());
  const double allowed_lateral_error = std::max(
      current_lateral_error, params.controller.lateral_drift_tolerance);
  CandidateSelection best;
  for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
    const auto& sample = samples[i];
    if (IsUnsafeVerticalSuccessor(sample)) continue;
    if (required_face.has_value() && sample.face != *required_face) continue;
    if (excluded_face.has_value() && sample.face == *excluded_face) continue;
    const Eigen::Vector2d normal_W = R_WO * sample.normal;
    const Eigen::Vector2d boundary_W = R_WO * sample.point;
    Eigen::Vector2d pusher = live_pusher;
    if (place_pusher_at_contact) {
      // Evaluate a repositioning sample as DAIRLab does: score the C3 mode at
      // the proposed future pusher location, not at the currently separated
      // end-effector position.  A small penetration activates the unilateral
      // mode without changing the physical execution target.
      pusher = object_pose.head<2>() + boundary_W + normal_W *
          (params.controller.pusher_radius -
           0.5 * params.controller.contact_activation_tolerance);
    }
    const auto result = SolveOneContact(
        params, pusher, object_pose, normal_W, boundary_W);
    if (!result.finite) continue;
    ++best.finite_samples;
    const double lateral_error = std::abs(
        result.object_prediction.x() - params.object.goal_pose.x());
    if (lateral_error > allowed_lateral_error) {
      ++best.lateral_rejections;
      continue;
    }
    const double cost = CandidateCost(params, result.object_prediction);
    if (cost < best.cost) {
      best.solve = result;
      best.sample_index = i;
      best.cost = cost;
    }
  }
  return best;
}

CandidateSelection EvaluateExactContactSample(
    const OimTParams& params, const Eigen::Vector2d& live_pusher,
    const Eigen::Vector3d& object_pose, int sample_index,
    bool place_pusher_at_contact) {
  const auto& sample = ExactTSamples().at(sample_index);
  const Eigen::Rotation2Dd R_WO(object_pose.z());
  const Eigen::Vector2d normal_W = R_WO * sample.normal;
  const Eigen::Vector2d boundary_W = R_WO * sample.point;
  Eigen::Vector2d pusher = live_pusher;
  if (place_pusher_at_contact) {
    pusher = object_pose.head<2>() + boundary_W + normal_W *
        (params.controller.pusher_radius -
         0.5 * params.controller.contact_activation_tolerance);
  }
  CandidateSelection candidate;
  candidate.solve = SolveOneContact(
      params, pusher, object_pose, normal_W, boundary_W);
  if (!candidate.solve.finite) return candidate;
  candidate.sample_index = sample_index;
  candidate.finite_samples = 1;
  candidate.cost = CandidateCost(params, candidate.solve.object_prediction);
  return candidate;
}

CandidateSelection SelectMeasuredLateralCorrectiveCandidate(
    const OimTParams& params, const Eigen::Vector3d& object_pose,
    ContactFace excluded_face, const Eigen::Vector2d& stick_axis_W) {
  const auto& samples = ExactTSamples();
  const Eigen::Rotation2Dd R_WO(object_pose.z());
  const Eigen::Vector2d goal_delta =
      params.object.goal_pose.head<2>() - object_pose.head<2>();
  CandidateSelection best;
  double best_directional_score = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
    const auto& sample = samples[i];
    if (IsUnsafeVerticalSuccessor(sample)) continue;
    if (sample.face == excluded_face) continue;
    const Eigen::Vector2d normal_W = R_WO * sample.normal;
    // stick_axis_W points from wrist toward the pusher tip, so the wrist is
    // on the outward side of a face only when -normal dot stick is positive.
    if (normal_W.dot(stick_axis_W) > 1.0e-3) continue;
    const Eigen::Vector2d boundary_W = R_WO * sample.point;
    const Eigen::Vector2d object_force_direction = -normal_W;
    const double contact_moment =
        boundary_W.x() * object_force_direction.y() -
        boundary_W.y() * object_force_direction.x();
    // This selector is invoked only by the measured-x guard. Give that guard
    // an actual lateral recovery face instead of requiring the same contact
    // to improve x, y, and yaw simultaneously (which rejects the
    // high-authority side faces and repeatedly selects weak rotational
    // contacts). Task-space progress is resumed by SelectTaskProgressCandidate
    // after the measured recovery receipt.
    if (goal_delta.x() * object_force_direction.x() <= 0.0) {
      continue;
    }
    CandidateSelection candidate = EvaluateExactContactSample(
        params, Eigen::Vector2d::Zero(), object_pose, i, true);
    if (!candidate.solve.finite) continue;
    ++best.finite_samples;
    const double y_alignment =
        goal_delta.y() * object_force_direction.y();
    // Recovery needs force authority, not rotational leverage. Among samples
    // on the same face, prefer the smallest moment arm so the finite capsule
    // does not acquire contact at a crossbar corner/top edge. Rotational C3+
    // selection retains all off-centre samples outside this recovery helper.
    const double directional_score =
        std::abs(object_force_direction.x()) +
        1.0e-3 * y_alignment -
        1.0e-3 * std::abs(contact_moment);
    if (directional_score > best_directional_score) {
      const int finite_samples = best.finite_samples;
      best = candidate;
      best.finite_samples = finite_samples;
      best_directional_score = directional_score;
    }
  }
  return best;
}

CandidateSelection SelectTaskProgressCandidate(
    const OimTParams& params, const Eigen::Vector3d& object_pose,
    ContactFace excluded_face, const Eigen::Vector2d& stick_axis_W,
    double minimum_shaft_clearance = -1.0e-3) {
  const auto& samples = ExactTSamples();
  const Eigen::Rotation2Dd R_WO(object_pose.z());
  const Eigen::Vector2d goal_delta =
      params.object.goal_pose.head<2>() - object_pose.head<2>();
  const double yaw_error =
      WrappedAngleError(params.object.goal_pose.z(), object_pose.z());
  CandidateSelection best;
  double best_directional_score = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
    const auto& sample = samples[i];
    if (IsUnsafeVerticalSuccessor(sample)) continue;
    if (sample.face == excluded_face) continue;
    const Eigen::Vector2d normal_W = R_WO * sample.normal;
    if (-normal_W.dot(stick_axis_W) < minimum_shaft_clearance) continue;
    const Eigen::Vector2d boundary_W = R_WO * sample.point;
    const Eigen::Vector2d object_force_direction = -normal_W;
    const double contact_moment =
        boundary_W.x() * object_force_direction.y() -
        boundary_W.y() * object_force_direction.x();
    if (goal_delta.y() * object_force_direction.y() <= 0.0 ||
        yaw_error * contact_moment <= 0.0) {
      continue;
    }
    CandidateSelection candidate = EvaluateExactContactSample(
        params, Eigen::Vector2d::Zero(), object_pose, i, true);
    if (!candidate.solve.finite) continue;
    ++best.finite_samples;
    const double directional_score =
        std::abs(contact_moment) +
        1.0e-3 * goal_delta.normalized().dot(object_force_direction);
    if (directional_score > best_directional_score) {
      const int finite_samples = best.finite_samples;
      best = candidate;
      best.finite_samples = finite_samples;
      best_directional_score = directional_score;
    }
  }
  return best;
}

void RunFirstC3PlusSolve(const OimTParams& params) {
  const ContactSample& capsule_sample = ExactTSamples().front();
  const Eigen::Rotation2Dd capsule_R_WO(params.object.start_pose.z());
  const Eigen::Vector2d capsule_normal_W =
      capsule_R_WO * capsule_sample.normal;
  Eigen::Vector3d loaded_capsule_tip_W;
  loaded_capsule_tip_W.head<2>() =
      params.object.start_pose.head<2>() +
      capsule_R_WO * capsule_sample.point + capsule_normal_W *
          (params.controller.pusher_radius -
           0.5 * params.controller.contact_activation_tolerance);
  loaded_capsule_tip_W.z() = params.object.resting_height;
  const auto vertical_capsule_receipt = EvaluateContactCapsuleClearance(
      params, loaded_capsule_tip_W, -Eigen::Vector3d::UnitZ(),
      params.object.start_pose, capsule_sample);
  const auto inward_inclined_capsule_receipt =
      EvaluateContactCapsuleClearance(
          params, loaded_capsule_tip_W,
          Eigen::Vector3d(0.0, 0.6, -0.8), params.object.start_pose,
          capsule_sample);
  if (!vertical_capsule_receipt.clear() ||
      inward_inclined_capsule_receipt.shaft_t_clear) {
    throw std::runtime_error(
        "contact capsule posture witness failed selected-face acceptance or "
        "wrong-polarity shaft rejection");
  }
  std::cout << "contact_capsule_posture_witness=PASS contact="
            << capsule_sample.name
            << " vertical_shaft_t_clear="
            << vertical_capsule_receipt.shaft_t_clear
            << " inward_inclined_shaft_t_clear="
            << inward_inclined_capsule_receipt.shaft_t_clear
            << " loaded_tip_W=" << loaded_capsule_tip_W.transpose()
            << std::endl;
  const auto result = SolveOneContact(
      params, Eigen::Vector2d(0.254967, -0.000911),
      params.object.start_pose, Eigen::Vector2d(0.0, -1.0),
      Eigen::Vector2d(0.0, -0.0794));
  if (!result.finite) throw std::runtime_error("first C3+ solve failed");
  std::cout << "first_c3plus_solve=PASS elapsed_s=" << result.elapsed
            << " final_stored_object_xy_yaw="
            << result.object_prediction.transpose()
            << " contact_gap_m=" << result.gap << std::endl;
  const auto rotational = SolveOneContact(
      params, Eigen::Vector2d(0.3365, 0.4240), params.object.start_pose,
      Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(-0.0445, 0.0198));
  const double requested_yaw_step = WrappedAngleError(
      params.object.goal_pose.z(), params.object.start_pose.z());
  const double predicted_yaw_step = WrappedAngleError(
      rotational.object_prediction.z(), params.object.start_pose.z());
  if (!rotational.finite ||
      requested_yaw_step * predicted_yaw_step <= 0.0 ||
      std::abs(rotational.object_prediction.x() - params.object.goal_pose.x()) >
          params.controller.lateral_drift_tolerance) {
    throw std::runtime_error(
        "rotational C3+ witness failed yaw or lateral-drift gate");
  }
  std::cout << "rotational_c3plus_witness=PASS predicted_object_xy_yaw="
            << rotational.object_prediction.transpose()
            << " contact_gap_m=" << rotational.gap << std::endl;
  Eigen::Vector3d quarter_turn_pose = params.object.start_pose;
  quarter_turn_pose.z() = 0.5 * 3.14159265358979323846;
  const CandidateSelection successor = SelectContactCandidate(
      params, Eigen::Vector2d::Zero(), quarter_turn_pose, std::nullopt,
      ContactFace::kCrossbarTop, true);
  const double successor_yaw_step = WrappedAngleError(
      successor.solve.object_prediction.z(), quarter_turn_pose.z());
  const double remaining_yaw_step = WrappedAngleError(
      params.object.goal_pose.z(), quarter_turn_pose.z());
  const double minimum_collision_free_height =
      params.object.resting_height + params.controller.pusher_radius +
      params.controller.descent_clearance;
  if (!successor.solve.finite || successor.sample_index < 0 ||
      successor_yaw_step * remaining_yaw_step <= 0.0 ||
      params.controller.reposition_waypoint_height <=
          minimum_collision_free_height) {
    throw std::runtime_error(
        "successor-face C3+ or collision-free reposition witness failed");
  }
  std::cout << "successor_face_witness=PASS contact="
            << ExactTSamples()[successor.sample_index].name
            << " predicted_object_xy_yaw="
            << successor.solve.object_prediction.transpose()
            << " finite=" << successor.finite_samples
            << " lateral_rejections=" << successor.lateral_rejections
            << " reposition_height_m="
            << params.controller.reposition_waypoint_height << std::endl;

  // Map the deterministic successor decision over the first quarter turn.
  // This is a reduced-model readiness witness, not a physical shortcut: the
  // live controller still has to reach the corresponding yaw from measured
  // state with all diagnostic overrides disabled.
  constexpr int kReadinessIntervals = 16;
  int first_crossbar_right_interval = -1;
  for (int interval = 0; interval <= kReadinessIntervals; ++interval) {
    Eigen::Vector3d test_pose = params.object.start_pose;
    test_pose.z() = 0.5 * 3.14159265358979323846 * interval /
        kReadinessIntervals;
    const CandidateSelection test_successor = SelectContactCandidate(
        params, Eigen::Vector2d::Zero(), test_pose, std::nullopt,
        ContactFace::kCrossbarTop, true);
    if (test_successor.sample_index >= 0 &&
        ExactTSamples()[test_successor.sample_index].face ==
            ContactFace::kCrossbarRight) {
      first_crossbar_right_interval = interval;
      std::cout << "autonomous_successor_readiness=PASS yaw_rad="
                << test_pose.z() << " contact="
                << ExactTSamples()[test_successor.sample_index].name
                << " active_face=crossbar_top"
                << " finite=" << test_successor.finite_samples
                << " lateral_rejections="
                << test_successor.lateral_rejections << std::endl;
      break;
    }
  }
  if (first_crossbar_right_interval < 0) {
    throw std::runtime_error(
        "crossbar-right successor is unavailable through the quarter turn");
  }

  Eigen::Vector3d measured_guard_pose = params.object.start_pose;
  measured_guard_pose.x() +=
      params.controller.lateral_drift_tolerance -
      params.controller.contact_activation_tolerance;
  measured_guard_pose.z() = 0.1;
  const CandidateSelection corrective =
      SelectMeasuredLateralCorrectiveCandidate(
          params, measured_guard_pose, ContactFace::kCrossbarTop,
          Eigen::Vector2d(-1.0, -1.0).normalized());
  if (!corrective.solve.finite || corrective.sample_index < 0 ||
      ExactTSamples()[corrective.sample_index].face !=
          ContactFace::kCrossbarRight) {
    throw std::runtime_error(
        "measured lateral-guard corrective successor witness failed");
  }
  const Eigen::Vector2d corrective_force_direction =
      -(Eigen::Rotation2Dd(measured_guard_pose.z()) *
        ExactTSamples()[corrective.sample_index].normal);
  if (std::abs(corrective_force_direction.x()) < 0.9) {
    throw std::runtime_error(
        "measured lateral-guard successor lacks x-normal authority");
  }
  std::cout << "measured_lateral_guard_witness=PASS yaw_rad="
            << measured_guard_pose.z() << " lateral_error_m="
            << std::abs(measured_guard_pose.x() -
                        params.object.goal_pose.x())
            << " contact=" << ExactTSamples()[corrective.sample_index].name
            << " force_x=" << corrective_force_direction.x()
            << std::endl;
  Eigen::Vector3d centered_recovery_pose = params.object.start_pose;
  centered_recovery_pose.x() = params.object.goal_pose.x() - 0.004;
  centered_recovery_pose.z() = 0.6;
  const CandidateSelection centered_recovery =
      SelectMeasuredLateralCorrectiveCandidate(
          params, centered_recovery_pose, ContactFace::kCrossbarRight,
          Eigen::Vector2d::Zero());
  if (!centered_recovery.solve.finite ||
      centered_recovery.sample_index < 0 ||
      std::string(ExactTSamples()[centered_recovery.sample_index].name) ==
          "crossbar_top_left" ||
      std::string(ExactTSamples()[centered_recovery.sample_index].name) ==
          "crossbar_top_right") {
    throw std::runtime_error(
        "measured lateral recovery did not select the finite face center");
  }
  std::cout << "centered_corrective_face_witness=PASS contact="
            << ExactTSamples()[centered_recovery.sample_index].name
            << " yaw_rad=" << centered_recovery_pose.z() << std::endl;
  const CandidateSelection progress_successor = SelectTaskProgressCandidate(
      params, measured_guard_pose, ContactFace::kCrossbarTop,
      Eigen::Vector2d(-1.0, -1.0).normalized());
  if (!progress_successor.solve.finite ||
      progress_successor.sample_index < 0 ||
      ExactTSamples()[progress_successor.sample_index].face !=
          ContactFace::kCrossbarRight) {
    throw std::runtime_error(
        "unproductive-mode task-progress successor witness failed");
  }
  std::cout << "task_progress_successor_witness=PASS yaw_rad="
            << measured_guard_pose.z() << " contact="
            << ExactTSamples()[progress_successor.sample_index].name
            << std::endl;
  const double response_threshold = 0.5 * std::max(
      0.0, params.controller.lateral_drift_tolerance -
               params.controller.contact_activation_tolerance);
  const double response_start_error = 0.004;
  const bool rejects_wrong_response = MeasuredResponseHasWrongPolarity(
      true, params.controller.successor_minimum_contact_steps,
      params.controller.successor_minimum_contact_steps,
      response_start_error + response_threshold, response_start_error,
      response_threshold);
  const bool rejects_correct_response = MeasuredResponseHasWrongPolarity(
      true, params.controller.successor_minimum_contact_steps,
      params.controller.successor_minimum_contact_steps,
      response_start_error - response_threshold, response_start_error,
      response_threshold);
  if (!rejects_wrong_response || rejects_correct_response) {
    throw std::runtime_error(
        "measured wrong-polarity response predicate witness failed");
  }
  std::cout << "measured_wrong_polarity_witness=PASS start_error_m="
            << response_start_error
            << " rejection_growth_m=" << response_threshold
            << " required_dwell_steps="
            << params.controller.successor_minimum_contact_steps
            << std::endl;
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const OimTParams params = LoadAndValidateConfig(FLAGS_config);
  const XarmSamplingC3PlannerMode planner_mode =
      ParseXarmSamplingC3PlannerMode(FLAGS_planner_mode);
  if (planner_mode == XarmSamplingC3PlannerMode::kFullSamplingC3Plus) {
    XarmFullSamplingC3State initial_state;
    initial_state.pusher_position_W << 0.254967, -0.000911,
        params.object.resting_height;
    initial_state.object_quaternion_WO = Eigen::Quaterniond(
        Eigen::AngleAxisd(params.object.start_pose.z(),
                          Eigen::Vector3d::UnitZ()));
    initial_state.object_position_W << params.object.start_pose.x(),
        params.object.start_pose.y(), params.object.resting_height;
    const Eigen::VectorXd encoded = initial_state.Encode();
    const XarmFullSamplingC3State decoded =
        XarmFullSamplingC3State::Decode(encoded);
    if (!decoded.Encode().isApprox(encoded, 1.0e-14)) {
      throw std::runtime_error(
          "full Sampling-C3+ spatial state round-trip failed");
    }
    std::cout << "full_sampling_c3plus_spatial_state_gate=PASS state_size="
              << XarmFullSamplingC3State::kSize << " input_size="
              << XarmFullSamplingC3State::kInputSize
              << " planner_mode="
              << XarmSamplingC3PlannerModeName(planner_mode) << std::endl;
    const XarmFullSamplingC3LcsReceipt lcs_receipt =
        BuildXarmFullSamplingC3SpatialLcsWitness(params);
    if (!lcs_receipt.finite || lcs_receipt.num_states != 19 ||
        lcs_receipt.num_inputs != 3 ||
        lcs_receipt.num_contact_pairs != 5 ||
        lcs_receipt.num_contact_variables != 20) {
      throw std::runtime_error(
          "full Sampling-C3+ spatial multibody LCS gate failed");
    }
    std::cout << "full_sampling_c3plus_spatial_lcs_gate=PASS states="
              << lcs_receipt.num_states << " inputs="
              << lcs_receipt.num_inputs << " contact_pairs="
              << lcs_receipt.num_contact_pairs << " contact_variables="
              << lcs_receipt.num_contact_variables << " horizon="
              << lcs_receipt.horizon << " dt=" << lcs_receipt.dt
              << " complementarity_offset_range=["
              << lcs_receipt.complementarity_offset_min << ","
              << lcs_receipt.complementarity_offset_max << "]"
              << std::endl;
    const XarmFullSamplingC3SolveReceipt solve_receipt =
        RunXarmFullSamplingC3FirstSolve(params);
    std::cout << "full_sampling_c3plus_first_solve="
              << (solve_receipt.accepted ? "PASS" : "FAIL")
              << " finite=" << solve_receipt.finite
              << " elapsed_s=" << solve_receipt.elapsed
              << " diagnostic_elapsed_s="
              << solve_receipt.diagnostic_elapsed
              << " lambda_scale=" << solve_receipt.lambda_scale
              << " initial_residual="
              << solve_receipt.initial_state_residual
              << " dynamics_residual=" << solve_receipt.dynamics_residual
              << " returned_plan_dynamics_residual="
              << solve_receipt.returned_plan_dynamics_residual
              << " equality_residual=" << solve_receipt.equality_residual
              << " nonnegative_residual="
              << solve_receipt.nonnegative_residual
              << " complementarity_residual="
              << solve_receipt.complementarity_residual
              << " projected_nonnegative_residual="
              << solve_receipt.projected_nonnegative_residual
              << " projected_complementarity_residual="
              << solve_receipt.projected_complementarity_residual
              << " consensus_residual="
              << solve_receipt.consensus_residual
              << " dynamic_rollout="
              << (solve_receipt.dynamic_rollout_accepted ? "PASS" : "FAIL")
              << " dynamic_lcp_solved="
              << solve_receipt.dynamic_rollout_lcp_solved
              << " dynamic_workspace="
              << solve_receipt.dynamic_rollout_workspace_accepted
              << " dynamic_workspace_violation="
              << solve_receipt.dynamic_rollout_workspace_violation
              << " input_bound_violation="
              << solve_receipt.planned_input_bound_violation
              << " dynamic_dynamics_residual="
              << solve_receipt.dynamic_rollout_dynamics_residual
              << " dynamic_nonnegative_residual="
              << solve_receipt.dynamic_rollout_nonnegative_residual
              << " dynamic_complementarity_residual="
              << solve_receipt.dynamic_rollout_complementarity_residual
              << " dynamic_cost="
              << solve_receipt.dynamic_rollout_cost
              << " dynamic_terminal_pusher="
              << solve_receipt.dynamic_terminal_pusher_position.transpose()
              << " dynamic_terminal_object="
              << solve_receipt.dynamic_terminal_object_position.transpose()
              << std::endl;
    const XarmFullSamplingC3BatchReceipt batch_receipt =
        RunXarmFullSamplingC3ExactTBatch(params);
    std::cout << "full_sampling_c3plus_exact_t_batch="
              << (batch_receipt.accepted ? "PASS" : "FAIL")
              << " samples=" << batch_receipt.num_samples
              << " feasible=" << batch_receipt.num_feasible
              << " selected_index=" << batch_receipt.selected_index
              << " selected_name=" << batch_receipt.selected_name
              << " selected_cost=" << batch_receipt.selected_cost
              << std::endl;
    for (const auto& candidate : batch_receipt.candidates) {
      std::cout << "full_sampling_c3plus_candidate index="
                << candidate.sample_index << " name="
                << candidate.sample_name << " feasible="
                << candidate.solve.dynamic_rollout_accepted
                << " workspace="
                << candidate.solve.dynamic_rollout_workspace_accepted
                << " workspace_violation="
                << candidate.solve.dynamic_rollout_workspace_violation
                << " cost="
                << candidate.solve.dynamic_rollout_cost
                << " terminal_object="
                << candidate.solve.dynamic_terminal_object_position.transpose()
                << std::endl;
    }
    const XarmFullSamplingC3BatchReceipt perimeter_receipt =
        RunXarmFullSamplingC3PerimeterBatch(params);
    std::cout << "full_sampling_c3plus_seeded_perimeter_batch="
              << (perimeter_receipt.accepted ? "PASS" : "FAIL")
              << " seed=" << params.full_sampling_c3plus.random_seed
              << " samples=" << perimeter_receipt.num_samples
              << " feasible=" << perimeter_receipt.num_feasible
              << " selected_index=" << perimeter_receipt.selected_index
              << " selected_name=" << perimeter_receipt.selected_name
              << " selected_cost=" << perimeter_receipt.selected_cost
              << std::endl;
    const XarmFullSamplingC3CandidateBuffer candidate_buffer =
        BuildXarmFullSamplingC3CandidateBuffer(
            {batch_receipt, perimeter_receipt});
    std::cout << "full_sampling_c3plus_candidate_buffer="
              << (candidate_buffer.accepted ? "PASS" : "FAIL")
              << " total=" << candidate_buffer.total_candidates
              << " successful=" << candidate_buffer.successful.size()
              << " unsuccessful=" << candidate_buffer.unsuccessful.size()
              << " selected_name="
              << (candidate_buffer.successful.empty()
                      ? "none"
                      : candidate_buffer.successful.front().sample_name)
              << " selected_cost="
              << (candidate_buffer.successful.empty()
                      ? std::numeric_limits<double>::infinity()
                      : candidate_buffer.successful.front()
                            .solve.dynamic_rollout_cost)
              << std::endl;
    const XarmFullSamplingC3BatchReceipt mesh_receipt =
        RunXarmFullSamplingC3MeshBatchParallel(params);
    const XarmFullSamplingC3BatchReceipt refinement_receipt =
        RunXarmFullSamplingC3StemLeftRefinementBatchParallel(params);
    const XarmFullSamplingC3CandidateBuffer full_candidate_buffer =
        BuildXarmFullSamplingC3CandidateBuffer(
            {batch_receipt, perimeter_receipt, mesh_receipt,
             refinement_receipt});
    std::cout << "full_sampling_c3plus_mesh_normal_batch="
              << (mesh_receipt.accepted ? "PASS" : "FAIL")
              << " seed=" << params.full_sampling_c3plus.mesh_random_seed
              << " samples=" << mesh_receipt.num_samples
              << " feasible=" << mesh_receipt.num_feasible
              << " selected_name=" << mesh_receipt.selected_name
              << " selected_cost=" << mesh_receipt.selected_cost
              << std::endl;
    std::cout << "full_sampling_c3plus_stem_left_refinement_batch="
              << (refinement_receipt.accepted ? "PASS" : "FAIL")
              << " samples=" << refinement_receipt.num_samples
              << " feasible=" << refinement_receipt.num_feasible
              << " selected_name=" << refinement_receipt.selected_name
              << " selected_cost=" << refinement_receipt.selected_cost
              << std::endl;
    std::cout << "full_sampling_c3plus_full_candidate_buffer="
              << (full_candidate_buffer.accepted ? "PASS" : "FAIL")
              << " total=" << full_candidate_buffer.total_candidates
              << " successful=" << full_candidate_buffer.successful.size()
              << " unsuccessful=" << full_candidate_buffer.unsuccessful.size()
              << " selected_name="
              << (full_candidate_buffer.successful.empty()
                      ? "none"
                      : full_candidate_buffer.successful.front().sample_name)
              << " selected_cost="
              << (full_candidate_buffer.successful.empty()
                      ? std::numeric_limits<double>::infinity()
                      : full_candidate_buffer.successful.front()
                            .solve.dynamic_rollout_cost)
              << std::endl;
    XarmFullSamplingC3CandidateBuffer execution_candidate_buffer;
    execution_candidate_buffer.total_candidates =
        full_candidate_buffer.total_candidates;
    execution_candidate_buffer.unsuccessful =
        full_candidate_buffer.unsuccessful;
    for (const auto& candidate : full_candidate_buffer.successful) {
      const double predicted_lateral_drift = std::abs(
          candidate.solve.dynamic_terminal_object_position.x() -
          params.object.goal_pose.x());
      const bool lateral_gate = predicted_lateral_drift <=
          params.controller.lateral_drift_tolerance;
      std::cout << "full_sampling_c3plus_executable_candidate name="
                << candidate.sample_name
                << " point_O=" << candidate.sample_point_O.transpose()
                << " normal_O=" << candidate.sample_normal_O.transpose()
                << " cost=" << candidate.solve.dynamic_rollout_cost
                << " terminal_object_W="
                << candidate.solve.dynamic_terminal_object_position.transpose()
                << " predicted_lateral_drift=" << predicted_lateral_drift
                << " lateral_gate="
                << (lateral_gate ? "PASS" : "FAIL")
                << std::endl;
      bool acquisition_gate = false;
      {
        const auto acquisition =
            EvaluateFullCandidateAcquisition(params, candidate);
        acquisition_gate = acquisition.swept_capsule_clear &&
            acquisition.ik_reached;
        std::cout << "full_sampling_c3plus_candidate_acquisition name="
                  << candidate.sample_name << " swept_capsule_clear="
                  << acquisition.swept_capsule_clear << " ik_reached="
                  << acquisition.ik_reached << " ik_steps="
                  << acquisition.ik_steps << " failed_waypoint="
                  << acquisition.failed_waypoint << " gate="
                  << (acquisition_gate ? "PASS" : "FAIL") << std::endl;
      }
      if (acquisition_gate) {
        execution_candidate_buffer.successful.push_back(candidate);
      } else {
        execution_candidate_buffer.unsuccessful.push_back(candidate);
      }
    }
    execution_candidate_buffer.accepted =
        execution_candidate_buffer.total_candidates > 0 &&
        !execution_candidate_buffer.successful.empty() &&
        execution_candidate_buffer.total_candidates == static_cast<int>(
            execution_candidate_buffer.successful.size() +
            execution_candidate_buffer.unsuccessful.size());
    const XarmFullSamplingC3TaskSpacePlan selected_plan =
        BuildXarmFullSamplingC3TaskSpacePlan(
            execution_candidate_buffer, params.task.planning_time_step);
    const Eigen::Vector3d selected_plan_start =
        selected_plan.pusher_positions_W.cols() == 0
        ? Eigen::Vector3d::Zero()
        : Eigen::Vector3d(selected_plan.pusher_positions_W.col(0));
    const Eigen::Vector3d selected_plan_terminal =
        selected_plan.pusher_positions_W.cols() == 0
        ? Eigen::Vector3d::Zero()
        : Eigen::Vector3d(selected_plan.pusher_positions_W.col(
              selected_plan.pusher_positions_W.cols() - 1));
    std::cout << "full_sampling_c3plus_task_space_plan="
              << (selected_plan.accepted ? "PASS" : "FAIL")
              << " sample=" << selected_plan.sample_name
              << " knots=" << selected_plan.time_vector.size()
              << " duration_s="
              << (selected_plan.time_vector.size() == 0
                      ? 0.0
                      : selected_plan.time_vector[
                            selected_plan.time_vector.size() - 1])
              << " start_W="
              << selected_plan_start.transpose()
              << " terminal_W="
              << selected_plan_terminal.transpose()
              << std::endl;
    if (!selected_plan.accepted) {
      throw std::runtime_error(
          "full Sampling-C3+ selected task-space plan gate failed");
    }
    drake::multibody::MultibodyPlant<double> acquisition_plant(0.0);
    drake::multibody::Parser(&acquisition_plant).AddModels(params.robot.model);
    AddXarmActuators(params, &acquisition_plant);
    acquisition_plant.Finalize();
    ValidateXarmPlant(acquisition_plant, params);
    auto acquisition_context = acquisition_plant.CreateDefaultContext();
    SetXarmHome(params, acquisition_plant, acquisition_context.get());
    const auto& acquisition_ee = acquisition_plant.GetBodyByName(
        params.robot.end_effector_body);
    Eigen::VectorXd acquisition_q =
        acquisition_plant.GetPositions(*acquisition_context);
    const auto home_pose = acquisition_plant.EvalBodyPoseInWorld(
        *acquisition_context, acquisition_ee);
    const Eigen::Vector3d acquisition_home_tip =
        home_pose * params.robot.end_effector_point;
    const Eigen::Vector3d acquisition_home_axis =
        home_pose.rotation() * Eigen::Vector3d::UnitZ();
    const Eigen::Rotation2Dd acquisition_R_WO(params.object.start_pose.z());
    const Eigen::Vector2d acquisition_normal_W =
        acquisition_R_WO * selected_plan.sample_normal_O;
    const Eigen::Vector3d acquisition_contact =
        selected_plan.pusher_positions_W.col(0);
    Eigen::Vector3d acquisition_standoff = acquisition_contact;
    acquisition_standoff.head<2>() += acquisition_normal_W *
        (params.controller.descent_clearance +
         0.5 * params.controller.contact_activation_tolerance);
    Eigen::Vector3d acquisition_high = acquisition_standoff;
    acquisition_high.z() = params.controller.reposition_waypoint_height;
    Eigen::Vector3d acquisition_planar = acquisition_high;
    acquisition_planar.z() = acquisition_home_tip.z();
    ContactFace acquisition_face = ContactFace::kCrossbarTop;
    if (selected_plan.sample_name.find("stem_left") != std::string::npos) {
      acquisition_face = ContactFace::kStemLeft;
    } else if (selected_plan.sample_name.find("stem_right") !=
               std::string::npos) {
      acquisition_face = ContactFace::kStemRight;
    } else if (selected_plan.sample_name.find("stem_bottom") !=
               std::string::npos) {
      acquisition_face = ContactFace::kStemBottom;
    } else if (selected_plan.sample_name.find("crossbar_left") !=
               std::string::npos) {
      acquisition_face = ContactFace::kCrossbarLeft;
    } else if (selected_plan.sample_name.find("crossbar_right") !=
               std::string::npos) {
      acquisition_face = ContactFace::kCrossbarRight;
    }
    const ContactSample acquisition_sample{
        selected_plan.sample_point_O, selected_plan.sample_normal_O,
        selected_plan.sample_name.c_str(), acquisition_face};
    const Eigen::Vector3d object_xyz_yaw = params.object.start_pose;
    const Eigen::Vector3d vertical_axis = -Eigen::Vector3d::UnitZ();
    const auto swept_home_to_planar = EvaluateSweptContactCapsuleClearance(
        params, acquisition_home_tip, acquisition_planar,
        acquisition_home_axis, vertical_axis, object_xyz_yaw,
        acquisition_sample);
    const auto swept_planar_to_high = EvaluateSweptContactCapsuleClearance(
        params, acquisition_planar, acquisition_high,
        vertical_axis, vertical_axis, object_xyz_yaw, acquisition_sample);
    const auto swept_high_to_standoff = EvaluateSweptContactCapsuleClearance(
        params, acquisition_high, acquisition_standoff,
        vertical_axis, vertical_axis, object_xyz_yaw, acquisition_sample);
    const auto swept_standoff_to_contact =
        EvaluateSweptContactCapsuleClearance(
            params, acquisition_standoff, acquisition_contact,
            vertical_axis, vertical_axis, object_xyz_yaw, acquisition_sample);
    bool acquisition_capsule_clear = swept_home_to_planar.clear() &&
        swept_planar_to_high.clear() &&
        swept_high_to_standoff.clear() && swept_standoff_to_contact.clear();
    int acquisition_ik_steps = 0;
    bool acquisition_ik_reached = true;
    const std::array<Eigen::Vector3d, 4> acquisition_waypoints = {
        acquisition_planar, acquisition_high, acquisition_standoff,
        acquisition_contact};
    for (const Eigen::Vector3d& waypoint : acquisition_waypoints) {
      bool waypoint_reached = false;
      for (int step = 0; step < 500; ++step) {
        acquisition_plant.SetPositions(acquisition_context.get(),
                                       acquisition_q);
        const Eigen::Vector3d current_tip =
            acquisition_plant.EvalBodyPoseInWorld(
                *acquisition_context, acquisition_ee) *
            params.robot.end_effector_point;
        if ((current_tip - waypoint).norm() <=
            params.controller.contact_activation_tolerance) {
          waypoint_reached = true;
          break;
        }
        const auto posture = SolveVerticalContactPostureStep(
            acquisition_plant, acquisition_context.get(), params,
            acquisition_q, waypoint, object_xyz_yaw, acquisition_sample);
        if (!posture.finite ||
            (posture.target - acquisition_q).norm() <= 1.0e-12) {
          break;
        }
        acquisition_q = posture.target;
        ++acquisition_ik_steps;
      }
      acquisition_ik_reached = acquisition_ik_reached && waypoint_reached;
      if (!waypoint_reached) break;
    }
    const bool acquisition_gate = acquisition_capsule_clear &&
        acquisition_ik_reached;
    std::cout << "full_sampling_c3plus_collision_aware_acquisition="
              << (acquisition_gate ? "PASS" : "FAIL")
              << " sample=" << selected_plan.sample_name
              << " swept_capsule_clear=" << acquisition_capsule_clear
              << " ik_reached=" << acquisition_ik_reached
              << " ik_steps=" << acquisition_ik_steps
              << " home_W=" << acquisition_home_tip.transpose()
              << " planar_W=" << acquisition_planar.transpose()
              << " high_W=" << acquisition_high.transpose()
              << " standoff_W=" << acquisition_standoff.transpose()
              << " contact_W=" << acquisition_contact.transpose()
              << std::endl;
    if (!acquisition_gate) {
      throw std::runtime_error(
          "full Sampling-C3+ collision-aware acquisition gate failed");
    }
    const XarmFullSamplingC3OscExecutionPlan osc_execution_plan =
        BuildXarmFullSamplingC3OscExecutionPlan(
            acquisition_home_tip, acquisition_planar, acquisition_high,
            acquisition_standoff,
            selected_plan, params.controller.reposition_speed,
            params.task.planning_time_step);
    std::cout << "full_sampling_c3plus_osc_execution_plan="
              << (osc_execution_plan.accepted ? "PASS" : "FAIL")
              << " acquisition_knots="
              << osc_execution_plan.acquisition_knots
              << " c3_knots=" << osc_execution_plan.c3_knots
              << " total_knots=" << osc_execution_plan.time_vector.size()
              << " duration_s="
              << (osc_execution_plan.time_vector.size() == 0
                      ? 0.0
                      : osc_execution_plan.time_vector[
                            osc_execution_plan.time_vector.size() - 1])
              << std::endl;
    if (!osc_execution_plan.accepted) {
      throw std::runtime_error(
          "full Sampling-C3+ OSC execution-plan gate failed");
    }
    if (!FLAGS_first_solve_only) {
      drake::lcm::DrakeLcm full_lcm(FLAGS_lcm_url);
      systems::Subscriber<dairlib::lcmt_robot_output> full_state_subscriber(
          &full_lcm, params.lcm.robot_state_channel);
      systems::Subscriber<dairlib::lcmt_object_state> full_object_subscriber(
          &full_lcm, params.lcm.object_state_channel);
      while (full_state_subscriber.count() == 0 ||
             full_object_subscriber.count() == 0) {
        full_lcm.HandleSubscriptions(100);
      }
      const double start_time =
          full_state_subscriber.message().utime * 1.0e-6;
      LcmTrajectory::Trajectory target;
      target.traj_name = "end_effector_position_target";
      target.datatypes = {"x", "y", "z"};
      target.time_vector =
          osc_execution_plan.time_vector.array() + start_time;
      target.datapoints = osc_execution_plan.positions_W;
      LcmTrajectory::Trajectory axis;
      axis.traj_name = "end_effector_stick_axis_target";
      axis.datatypes = {"x", "y", "z"};
      axis.time_vector = target.time_vector;
      axis.datapoints.resize(3, target.time_vector.size());
      for (int knot = 0; knot < axis.datapoints.cols(); ++knot) {
        axis.datapoints.col(knot) = vertical_axis;
      }
      LcmTrajectory trajectory(
          {target, axis}, {target.traj_name, axis.traj_name}, target.traj_name,
          "Full Sampling-C3+ collision-aware xArm execution plan", false);
      dairlib::lcmt_timestamped_saved_traj message;
      message.utime = full_state_subscriber.message().utime;
      message.saved_traj = trajectory.GenerateLcmObject();
      for (int publish = 0; publish < FLAGS_publish_count; ++publish) {
        drake::lcm::Publish(&full_lcm,
                            params.lcm.tracking_trajectory_channel, message);
        full_lcm.HandleSubscriptions(10);
        while (full_lcm.HandleSubscriptions(0) > 0) {
        }
      }
      std::cout << "full_sampling_c3plus_osc_handoff=PASS channel="
                << params.lcm.tracking_trajectory_channel
                << " publishes=" << FLAGS_publish_count
                << " utime=" << message.utime
                << " knots=" << target.time_vector.size() << std::endl;
      auto read_full_tip = [&]() {
        const auto& robot_message = full_state_subscriber.message();
        for (int i = 0; i < robot_message.num_positions; ++i) {
          acquisition_plant.GetJointByName(robot_message.position_names[i])
              .SetPositions(
                  acquisition_context.get(),
                  Eigen::VectorXd::Constant(1, robot_message.position[i]));
        }
        for (int i = 0; i < robot_message.num_velocities; ++i) {
          std::string joint_name = robot_message.velocity_names[i];
          constexpr char kVelocitySuffix[] = "dot";
          if (joint_name.size() >= 3 &&
              joint_name.compare(joint_name.size() - 3, 3,
                                 kVelocitySuffix) == 0) {
            joint_name.erase(joint_name.size() - 3);
          }
          acquisition_plant.GetJointByName(joint_name)
              .SetVelocities(
                  acquisition_context.get(),
                  Eigen::VectorXd::Constant(1, robot_message.velocity[i]));
        }
        return acquisition_plant.EvalBodyPoseInWorld(
                   *acquisition_context, acquisition_ee) *
            params.robot.end_effector_point;
      };
      auto read_full_tip_speed = [&]() {
        Eigen::MatrixXd J(3, acquisition_plant.num_velocities());
        acquisition_plant.CalcJacobianTranslationalVelocity(
            *acquisition_context,
            drake::multibody::JacobianWrtVariable::kV,
            acquisition_ee.body_frame(), params.robot.end_effector_point,
            acquisition_plant.world_frame(),
            acquisition_plant.world_frame(), &J);
        return (J * acquisition_plant.GetVelocities(
                        *acquisition_context)).norm();
      };
      auto refresh_full_measurements = [&](const std::string& phase) {
        const int prior_robot_count = full_state_subscriber.count();
        const int64_t prior_utime =
            full_state_subscriber.message().utime;
        int polls = 0;
        while (full_state_subscriber.count() <= prior_robot_count &&
               polls < 10) {
          full_lcm.HandleSubscriptions(100);
          ++polls;
        }
        while (full_lcm.HandleSubscriptions(0) > 0) {
        }
        const int64_t latest_utime =
            full_state_subscriber.message().utime;
        std::cout << "full_sampling_c3plus_measurement_refresh="
                  << (latest_utime > prior_utime ? "PASS" : "STALE")
                  << " phase=" << phase
                  << " prior_utime=" << prior_utime
                  << " latest_utime=" << latest_utime
                  << " elapsed_updates="
                  << (latest_utime - prior_utime) /
                         (1000LL * FLAGS_full_execution_period_ms)
                  << " polls=" << polls << std::endl;
        return latest_utime > prior_utime;
      };
      auto read_full_object_x = [&]() {
        const auto& object_message = full_object_subscriber.message();
        for (int i = 0; i < object_message.num_positions; ++i) {
          if (object_message.position_names[i] == "block_x") {
            return object_message.position[i];
          }
        }
        throw std::runtime_error("full Sampling-C3+ state missing block_x");
      };
      auto read_full_object_pose = [&]() {
        const auto& object_message = full_object_subscriber.message();
        auto value = [&](const std::string& name) {
          for (int i = 0; i < object_message.num_positions; ++i) {
            if (object_message.position_names[i] == name) {
              return object_message.position[i];
            }
          }
          throw std::runtime_error(
              "full Sampling-C3+ state missing " + name);
        };
        const double qw = value("block_qw");
        const double qx = value("block_qx");
        const double qy = value("block_qy");
        const double qz = value("block_qz");
        return Eigen::Vector3d(
            value("block_x"), value("block_y"),
            std::atan2(2.0 * (qw * qz + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz * qz)));
      };
      auto read_full_object_velocity = [&]() {
        const auto& object_message = full_object_subscriber.message();
        auto value = [&](const std::string& name) {
          for (int i = 0; i < object_message.num_velocities; ++i) {
            if (object_message.velocity_names[i] == name) {
              return object_message.velocity[i];
            }
          }
          throw std::runtime_error(
              "full Sampling-C3+ state missing " + name);
        };
        return std::make_pair(
            Eigen::Vector3d(value("block_wx"), value("block_wy"),
                            value("block_wz")),
            Eigen::Vector3d(value("block_vx"), value("block_vy"),
                            value("block_vz")));
      };
      auto read_full_object_spatial_pose = [&]() {
        const auto& object_message = full_object_subscriber.message();
        auto value = [&](const std::string& name) {
          for (int i = 0; i < object_message.num_positions; ++i) {
            if (object_message.position_names[i] == name) {
              return object_message.position[i];
            }
          }
          throw std::runtime_error(
              "full Sampling-C3+ state missing " + name);
        };
        return std::make_pair(
            Eigen::Vector3d(value("block_x"), value("block_y"),
                            value("block_z")),
            Eigen::Quaterniond(value("block_qw"), value("block_qx"),
                               value("block_qy"), value("block_qz")));
      };
      const double progress_lateral_reserve_limit =
          FullSamplingC3LateralReserveLimit(
              params.controller.lateral_drift_tolerance,
              params.controller.contact_activation_tolerance);
      auto wait_for_planar_settle = [&](const std::string& phase) {
        auto previous = read_full_object_spatial_pose();
        int previous_count = full_object_subscriber.count();
        int consecutive = 0;
        int samples = 0;
        XarmFullSamplingC3PlanarSettleReceipt receipt;
        const int maximum_samples = std::max(
            2, static_cast<int>(std::ceil(
                   params.simulation.object_publish_rate)));
        const int maximum_polls_per_sample = std::max(
            10, static_cast<int>(std::ceil(
                    2.0 * params.simulation.robot_publish_rate /
                    params.simulation.object_publish_rate)));
        while (samples < maximum_samples && consecutive < 2) {
          int polls = 0;
          while (full_object_subscriber.count() <= previous_count &&
                 polls < maximum_polls_per_sample) {
            full_lcm.HandleSubscriptions(100);
            ++polls;
          }
          if (full_object_subscriber.count() <= previous_count) break;
          previous_count = full_object_subscriber.count();
          const auto current = read_full_object_spatial_pose();
          receipt = EvaluateXarmFullSamplingC3PlanarSettle(
              previous.first, previous.second, current.first, current.second,
              params.object.resting_height,
              params.controller.successor_minimum_translation_progress,
              params.controller.successor_minimum_yaw_progress,
              params.controller.contact_activation_tolerance,
              params.task.orientation_tolerance);
          consecutive = receipt.accepted ? consecutive + 1 : 0;
          ++samples;
          previous = current;
        }
        const bool accepted = consecutive >= 2;
        std::cout << "full_sampling_c3plus_planar_settle="
                  << (accepted ? "PASS" : "FAIL")
                  << " phase=" << phase
                  << " samples=" << samples
                  << " consecutive=" << consecutive
                  << " planar_delta_m="
                  << receipt.planar_translation_delta
                  << " yaw_delta_rad=" << receipt.yaw_delta
                  << " vertical_error_m="
                  << receipt.vertical_position_error
                  << " tilt_rad=" << receipt.tilt_angle << std::endl;
        return accepted;
      };
      int full_execution_updates = 0;
      int reached_waypoints = 1;
      int commanded_subtargets = 0;
      bool measured_lateral_rejected = false;
      double maximum_tracking_error = 0.0;
      for (int waypoint_index = 1;
           waypoint_index < osc_execution_plan.positions_W.cols() &&
           full_execution_updates < FLAGS_full_execution_steps &&
           !measured_lateral_rejected;
           ++waypoint_index) {
        const Eigen::Vector3d waypoint =
            osc_execution_plan.positions_W.col(waypoint_index);
        Eigen::Vector3d current_tip = read_full_tip();
        const auto waypoint_reached = [&](const Eigen::Vector3d& tip) {
          if (waypoint_index == 1) {
            return (waypoint.head<2>() - tip.head<2>()).norm() <=
                       params.controller.approach_planar_tolerance &&
                std::abs(waypoint.z() - tip.z()) <=
                       params.controller.contact_activation_tolerance;
          }
          return (waypoint - tip).norm() <=
              params.controller.contact_activation_tolerance;
        };
        const double initial_error = (waypoint - current_tip).norm();
        maximum_tracking_error = std::max(maximum_tracking_error,
                                          initial_error);
        while (!waypoint_reached(current_tip) &&
               full_execution_updates < FLAGS_full_execution_steps &&
               !measured_lateral_rejected) {
          const double command_step_limit = waypoint_index == 1
              ? params.controller.approach_command_step_limit
              : (waypoint_index <= 3
                     ? params.controller.descent_command_step_limit
                     : params.controller.task_space_plan_step_limit);
          Eigen::Vector3d command_delta = waypoint - current_tip;
          if (command_delta.norm() > command_step_limit) {
            command_delta *= command_step_limit / command_delta.norm();
          }
          const Eigen::Vector3d command_tip = current_tip + command_delta;
          LcmTrajectory::Trajectory gated_target;
          gated_target.traj_name = "end_effector_position_target";
          gated_target.datatypes = {"x", "y", "z"};
          gated_target.time_vector.resize(2);
          gated_target.time_vector[0] =
              full_state_subscriber.message().utime * 1.0e-6;
          gated_target.time_vector[1] = gated_target.time_vector[0] +
              params.task.planning_time_step;
          gated_target.datapoints.resize(3, 2);
          // Hold each bounded measured-state command at both knots. This lets
          // OscPhaseMux see the actual downward command when descent begins.
          gated_target.datapoints.col(0) = command_tip;
          gated_target.datapoints.col(1) = command_tip;
          LcmTrajectory::Trajectory gated_axis;
          gated_axis.traj_name = "end_effector_stick_axis_target";
          gated_axis.datatypes = {"x", "y", "z"};
          gated_axis.time_vector = gated_target.time_vector;
          gated_axis.datapoints.resize(3, 2);
          gated_axis.datapoints.col(0) = vertical_axis;
          gated_axis.datapoints.col(1) = vertical_axis;
          std::vector<LcmTrajectory::Trajectory> gated_targets{
              gated_target, gated_axis};
          std::vector<std::string> gated_target_names{
              gated_target.traj_name, gated_axis.traj_name};
          if (waypoint_index <= 4) {
            const Eigen::VectorXd current_q =
                acquisition_plant.GetPositions(*acquisition_context);
            const auto posture = SolveVerticalContactPostureStep(
                acquisition_plant, acquisition_context.get(), params,
                current_q, command_tip, object_xyz_yaw, acquisition_sample);
            if (!posture.finite) {
              throw std::runtime_error(
                  "full Sampling-C3+ measured acquisition posture gate "
                  "failed");
            }
            LcmTrajectory::Trajectory gated_posture;
            gated_posture.traj_name =
                "collision_aware_reposition_posture_target";
            gated_posture.datatypes = params.robot.controlled_joints;
            gated_posture.time_vector = gated_target.time_vector;
            gated_posture.datapoints.resize(current_q.size(), 2);
            gated_posture.datapoints.col(0) = current_q;
            gated_posture.datapoints.col(1) = posture.target;
            gated_targets.push_back(gated_posture);
            gated_target_names.push_back(gated_posture.traj_name);
          }
          LcmTrajectory gated_trajectory(
              gated_targets, gated_target_names,
              gated_target.traj_name,
              "Measured-state bounded Full Sampling-C3+ xArm subtarget",
              false);
          dairlib::lcmt_timestamped_saved_traj gated_message;
          gated_message.utime = full_state_subscriber.message().utime;
          gated_message.saved_traj = gated_trajectory.GenerateLcmObject();
          ++commanded_subtargets;
          // Match the reduced controller's receding measured-target contract:
          // advance the bounded target on each control-period observation,
          // rather than waiting for a temporary subtarget to settle before
          // issuing the next one.
          drake::lcm::Publish(&full_lcm,
                              params.lcm.tracking_trajectory_channel,
                              gated_message);
          const int64_t next_control_utime =
              full_state_subscriber.message().utime +
              1000LL * FLAGS_full_execution_period_ms;
          while (full_state_subscriber.message().utime <
                 next_control_utime) {
            full_lcm.HandleSubscriptions(100);
            while (full_lcm.HandleSubscriptions(0) > 0) {
            }
          }
          current_tip = read_full_tip();
          ++full_execution_updates;
          const double measured_lateral_drift = std::abs(
              read_full_object_x() - params.object.goal_pose.x());
          if (measured_lateral_drift >
              params.controller.lateral_drift_tolerance) {
            measured_lateral_rejected = true;
            std::cout << "full_sampling_c3plus_measured_lateral_response="
                      << "REJECT updates=" << full_execution_updates
                      << " drift_m=" << measured_lateral_drift
                      << " tolerance_m="
                      << params.controller.lateral_drift_tolerance
                      << std::endl;
          }
        }
        if (waypoint_reached(current_tip)) {
          ++reached_waypoints;
          std::cout << "full_sampling_c3plus_waypoint=PASS index="
                    << waypoint_index << " updates="
                    << full_execution_updates << " tip_W="
                    << current_tip.transpose() << std::endl;
        } else {
          std::cout << "full_sampling_c3plus_waypoint=FAIL index="
                    << waypoint_index << " error_W="
                    << (waypoint - current_tip).transpose()
                    << " updates=" << full_execution_updates << std::endl;
          break;
        }
      }
      bool corrective_contact_handoff = false;
      bool corrective_lateral_recovery = false;
      bool full_task_progress_cycle = false;
      bool any_full_task_progress_cycle = false;
      bool contact_cycle_budget_deferred = false;
      std::vector<XarmFullSamplingC3MeasuredCycleReceipt>
          measured_productive_cycles;
      if (measured_lateral_rejected &&
          full_execution_updates < FLAGS_full_execution_steps) {
        const int initial_recovery_start_updates = full_execution_updates;
        auto execute_posture_waypoint = [&](const Eigen::Vector3d& waypoint,
                                            const Eigen::Vector3d& object_pose,
                                            const ContactSample& sample,
                                            const std::string& phase,
                                            bool allow_release_fallback,
                                            bool stop_on_lateral_corridor,
                                            bool reject_on_lateral_exit =
                                                false,
                                            bool contact_engagement = false,
                                            bool vertical_release = false,
                                            bool enforce_preview_conformance =
                                                false,
                                            bool measured_vertical_translation =
                                                false) {
          refresh_full_measurements(phase);
          Eigen::Vector3d current_tip = read_full_tip();
          const double phase_entry_waypoint_error =
              measured_vertical_translation
                  ? std::abs(waypoint.z() - current_tip.z())
                  : (waypoint - current_tip).norm();
          auto posture_waypoint_reached = [&]() {
            const double waypoint_error =
                (vertical_release || measured_vertical_translation)
                ? std::abs(waypoint.z() - current_tip.z())
                : (waypoint - current_tip).norm();
            const bool settled = IsFullSamplingC3WaypointSettled(
                waypoint_error, read_full_tip_speed(),
                params.task.planning_time_step,
                params.controller.contact_activation_tolerance);
            if (!settled) {
              return false;
            }
            if (allow_release_fallback) return true;
            const Eigen::Vector3d measured_axis_W =
                acquisition_plant.EvalBodyPoseInWorld(
                    *acquisition_context, acquisition_ee).rotation() *
                Eigen::Vector3d::UnitZ();
            const double vertical_axis_error = std::acos(std::clamp(
                measured_axis_W.dot(-Eigen::Vector3d::UnitZ()), -1.0, 1.0));
            return vertical_axis_error <= kMaximumVerticalAxisError;
          };
          while (!posture_waypoint_reached() &&
                 full_execution_updates < FLAGS_full_execution_steps) {
            Eigen::Vector3d delta = waypoint - current_tip;
            const double task_step_limit = contact_engagement
                ? std::min(
                      params.controller.task_space_plan_step_limit,
                      params.controller.minimum_contact_normal_step)
                : params.controller.task_space_plan_step_limit;
            Eigen::Vector3d command_tip = current_tip + delta;
            if (measured_vertical_translation) {
              command_tip = BuildMeasuredVerticalTranslationSubtarget(
                  current_tip, waypoint.z(), task_step_limit);
            } else if (delta.norm() > task_step_limit) {
              delta *= task_step_limit /
                  delta.norm();
              command_tip = current_tip + delta;
            }
            if (vertical_release && command_tip.z() > current_tip.z()) {
              // Tracking drift after a low failed approach must not turn a
              // nominal lift into an inward scrape along the rejected face.
              command_tip.head<2>() = current_tip.head<2>();
            }
            if (allow_release_fallback &&
                command_tip.z() < current_tip.z() &&
                current_tip.z() <= waypoint.z() +
                    params.controller.contact_activation_tolerance) {
              // Re-evaluate the release height from measured state on every
              // substep. Tracking noise must not turn a nominally horizontal
              // contact escape into a descending capsule command.  Once an
              // upward waypoint has overshot by more than the existing
              // activation tolerance, allow the collision-checked IK loop to
              // correct it instead of deadlocking above the waypoint.
              command_tip.z() = std::max(command_tip.z(), current_tip.z());
            }
            const Eigen::VectorXd current_q =
                acquisition_plant.GetPositions(*acquisition_context);
            const auto& acquisition_ee = acquisition_plant.GetBodyByName(
                params.robot.end_effector_body);
            const auto current_ee_pose =
                acquisition_plant.EvalBodyPoseInWorld(
                    *acquisition_context, acquisition_ee);
            const Eigen::Vector3d current_axis_W =
                current_ee_pose.rotation() * Eigen::Vector3d::UnitZ();
            Eigen::Vector3d commanded_axis_W = vertical_axis;
            Eigen::VectorXd posture_q = current_q;
            if (allow_release_fallback) {
              const auto release_posture = SolveLiftPostureStep(
                  acquisition_plant, acquisition_context.get(), params,
                  current_q, command_tip);
              ContactCapsuleClearanceReceipt swept_release;
              Eigen::Vector3d release_target_tip = current_tip;
              Eigen::Vector3d release_target_axis_W = current_axis_W;
              if (release_posture.has_value()) {
                for (int sweep_sample = 0;
                     sweep_sample <= kSweptPostureSamples;
                     ++sweep_sample) {
                  const double alpha = static_cast<double>(sweep_sample) /
                      static_cast<double>(kSweptPostureSamples);
                  acquisition_plant.SetPositions(
                      acquisition_context.get(),
                      current_q + alpha * (*release_posture - current_q));
                  const auto release_ee_pose =
                      acquisition_plant.EvalBodyPoseInWorld(
                          *acquisition_context, acquisition_ee);
                  release_target_tip = release_ee_pose *
                      params.robot.end_effector_point;
                  release_target_axis_W = release_ee_pose.rotation() *
                      Eigen::Vector3d::UnitZ();
                  const auto release_clearance =
                      EvaluateContactCapsuleClearance(
                          params, release_target_tip,
                          release_target_axis_W, object_pose, sample);
                  swept_release.shaft_t_clear =
                      swept_release.shaft_t_clear &&
                      release_clearance.shaft_t_clear;
                  swept_release.capsule_table_clear =
                      swept_release.capsule_table_clear &&
                      release_clearance.capsule_table_clear;
                  swept_release.tip_table_clear =
                      swept_release.tip_table_clear &&
                      release_clearance.tip_table_clear;
                  swept_release.capsule_table_margin = std::min(
                      swept_release.capsule_table_margin,
                      release_clearance.capsule_table_margin);
                }
                acquisition_plant.SetPositions(
                    acquisition_context.get(), current_q);
              }
              const auto release_endpoint = EvaluateContactCapsuleClearance(
                  params, release_target_tip, release_target_axis_W,
                  object_pose, sample);
              const Eigen::Vector2d release_normal_W =
                  Eigen::Rotation2Dd(object_pose.z()) * sample.normal;
              const double outward_release_progress =
                  (release_target_tip.head<2>() - current_tip.head<2>())
                      .dot(release_normal_W);
              const bool controlled_contact_escape =
                  release_endpoint.capsule_table_clear &&
                  release_endpoint.tip_table_clear &&
                  outward_release_progress > 0.0 &&
                  release_target_tip.z() >= current_tip.z();
              const bool controlled_vertical_escape =
                  vertical_release &&
                  release_endpoint.capsule_table_clear &&
                  release_endpoint.tip_table_clear &&
                  (command_tip.head<2>() - current_tip.head<2>())
                          .squaredNorm() == 0.0 &&
                  release_target_tip.z() > current_tip.z() &&
                  outward_release_progress >= 0.0;
              if (release_posture.has_value() &&
                  (swept_release.clear() || controlled_contact_escape ||
                   controlled_vertical_escape)) {
                // A release/lift waypoint owns only the measured tip
                // translation.  Prefer its position-only IK solution even
                // when contact-posture IK is feasible: rotating the capsule
                // toward the next vertical contact posture during release
                // can make a joint-limited intermediate step move the tip
                // away from the commanded waypoint.  The subsequent
                // collision-checked traverse establishes that posture.
                posture_q = *release_posture;
                commanded_axis_W = current_axis_W;
              } else {
                std::cout <<
                    "full_sampling_c3plus_posture_step=REJECT phase="
                          << phase << " command_tip_W="
                          << command_tip.transpose()
                          << " position_only_ik="
                          << release_posture.has_value()
                          << " swept_clear=" << swept_release.clear()
                          << " escape_endpoint_clear="
                          << release_endpoint.clear()
                          << " controlled_escape="
                          << controlled_contact_escape
                          << " controlled_vertical_escape="
                          << controlled_vertical_escape
                          << " outward_progress_m="
                          << outward_release_progress
                          << std::endl;
                return false;
              }
            } else {
              const bool allow_home_seed_fallback =
                  phase.find("verticalize") != std::string::npos;
              const auto posture = SolveVerticalContactPostureStep(
                  acquisition_plant, acquisition_context.get(), params,
                  current_q, command_tip, object_pose, sample,
                  allow_home_seed_fallback);
              if (!posture.finite) {
                std::cout <<
                    "full_sampling_c3plus_posture_step=REJECT phase="
                          << phase << " command_tip_W="
                          << command_tip.transpose()
                          << " contact_ik=" << posture.finite << std::endl;
                return false;
              }
              if (posture.home_seed_fallback) {
                std::cout <<
                    "full_sampling_c3plus_verticalization_home_seed=PASS"
                          << " phase=" << phase
                          << " command_tip_W=" << command_tip.transpose()
                          << " target_tip_error_m="
                          << posture.target_tip_error
                          << " target_axis_error_rad="
                          << posture.target_axis_error << std::endl;
              }
              posture_q = posture.target;
            }
            const double time =
                full_state_subscriber.message().utime * 1.0e-6;
            LcmTrajectory::Trajectory task;
            task.traj_name = "end_effector_position_target";
            task.datatypes = {"x", "y", "z"};
            task.time_vector = Eigen::Vector2d(
                time, time + params.task.planning_time_step);
            task.datapoints.resize(3, 2);
            task.datapoints.col(0) = command_tip;
            task.datapoints.col(1) = command_tip;
            LcmTrajectory::Trajectory axis_target;
            axis_target.traj_name = "end_effector_stick_axis_target";
            axis_target.datatypes = {"x", "y", "z"};
            axis_target.time_vector = task.time_vector;
            axis_target.datapoints.resize(3, 2);
            axis_target.datapoints.col(0) = commanded_axis_W;
            axis_target.datapoints.col(1) = commanded_axis_W;
            LcmTrajectory::Trajectory posture_target;
            posture_target.traj_name =
                "collision_aware_reposition_posture_target";
            posture_target.datatypes = params.robot.controlled_joints;
            posture_target.time_vector = task.time_vector;
            posture_target.datapoints.resize(current_q.size(), 2);
            posture_target.datapoints.col(0) = current_q;
            posture_target.datapoints.col(1) = posture_q;
            LcmTrajectory command(
                {task, axis_target, posture_target},
                {task.traj_name, axis_target.traj_name,
                 posture_target.traj_name},
                task.traj_name,
                "Full Sampling-C3+ measured corrective reposition", false);
            dairlib::lcmt_timestamped_saved_traj command_message;
            command_message.utime = full_state_subscriber.message().utime;
            command_message.saved_traj = command.GenerateLcmObject();
            drake::lcm::Publish(&full_lcm,
                                params.lcm.tracking_trajectory_channel,
                                command_message);
            const int64_t next_utime =
                full_state_subscriber.message().utime +
                1000LL * FLAGS_full_execution_period_ms;
            while (full_state_subscriber.message().utime < next_utime) {
              full_lcm.HandleSubscriptions(100);
              while (full_lcm.HandleSubscriptions(0) > 0) {
              }
            }
            current_tip = read_full_tip();
            ++full_execution_updates;
            ++commanded_subtargets;
            const double measured_waypoint_error =
                measured_vertical_translation
                    ? std::abs(waypoint.z() - current_tip.z())
                    : (waypoint - current_tip).norm();
            if (enforce_preview_conformance &&
                !IsFullSamplingC3WaypointExecutionConformant(
                    phase_entry_waypoint_error, measured_waypoint_error,
                    params.controller.contact_activation_tolerance)) {
              std::cout <<
                  "full_sampling_c3plus_physical_preview_conformance=FAIL"
                        << " phase=" << phase
                        << " phase_entry_waypoint_error_m="
                        << phase_entry_waypoint_error
                        << " measured_waypoint_error_m="
                        << measured_waypoint_error
                        << " tolerance_m="
                        << params.controller.contact_activation_tolerance
                        << " updates=" << full_execution_updates
                        << std::endl;
              return false;
            }
            if (stop_on_lateral_corridor &&
                std::abs(read_full_object_x() -
                         params.object.goal_pose.x()) <=
                    params.controller.lateral_drift_tolerance) {
              break;
            }
            if (reject_on_lateral_exit &&
                std::abs(read_full_object_x() -
                         params.object.goal_pose.x()) >
                    params.controller.lateral_drift_tolerance) {
              break;
            }
          }
          const bool reached =
              posture_waypoint_reached() ||
              (stop_on_lateral_corridor &&
               std::abs(read_full_object_x() -
                        params.object.goal_pose.x()) <=
                   params.controller.lateral_drift_tolerance);
          std::cout << "full_sampling_c3plus_corrective_phase="
                    << (reached ? "PASS" : "FAIL") << " phase=" << phase
                    << " updates=" << full_execution_updates
                    << " tip_W=" << current_tip.transpose()
                    << " tip_speed_mps=" << read_full_tip_speed()
                    << " settle_drift_m="
                    << read_full_tip_speed() *
                           params.task.planning_time_step
                    << " measured_vertical_translation="
                    << measured_vertical_translation
                    << std::endl;
          return reached;
        };

        Eigen::Vector3d corrective_object_pose = read_full_object_pose();
        Eigen::Vector3d release = read_full_tip();
        const Eigen::Vector2d active_normal_W =
            Eigen::Rotation2Dd(corrective_object_pose.z()) *
            selected_plan.sample_normal_O;
        release.head<2>() += active_normal_W *
            (params.controller.descent_clearance +
             params.controller.contact_activation_tolerance);
        Eigen::Vector3d lift = release;
        lift.z() = std::max(lift.z(),
                            CapsuleOrientationClearanceHeight(params));
        const bool released = execute_posture_waypoint(
            release, corrective_object_pose, acquisition_sample, "release",
            true, false);
        const bool lifted = released && execute_posture_waypoint(
            lift, corrective_object_pose, acquisition_sample, "lift", false,
            false);
        if (lifted) {
          corrective_object_pose = read_full_object_pose();
          const auto corrective_object_velocity =
              read_full_object_velocity();
          const auto corrective_planar_velocity =
              ConditionOpenTableObjectVelocity(
                  corrective_object_velocity.first,
                  corrective_object_velocity.second);
          OimTParams corrective_params = params;
          corrective_params.object.start_pose = corrective_object_pose;
          corrective_params.object.start_angular_velocity_W =
              corrective_planar_velocity.first;
          corrective_params.object.start_linear_velocity_W =
              corrective_planar_velocity.second;
          // Condition the corrective solve on the violated coordinate. Hold
          // measured y/yaw while restoring canonical x; the unchanged full
          // open-table goal is resumed only after measured lateral recovery.
          corrective_params.object.goal_pose = corrective_object_pose;
          corrective_params.object.goal_pose.x() = params.object.goal_pose.x();
          const auto corrective_exact =
              RunXarmFullSamplingC3ExactTBatch(corrective_params);
          const auto corrective_perimeter =
              RunXarmFullSamplingC3PerimeterBatchParallel(corrective_params);
          const auto corrective_mesh =
              RunXarmFullSamplingC3MeshBatchParallel(corrective_params);
          const auto corrective_refinement =
              RunXarmFullSamplingC3StemLeftRefinementBatchParallel(
                  corrective_params);
          const auto corrective_right_refinement =
              RunXarmFullSamplingC3StemRightRefinementBatchParallel(
                  corrective_params);
          const auto corrective_buffer = BuildXarmFullSamplingC3CandidateBuffer(
              {corrective_exact, corrective_perimeter, corrective_mesh,
               corrective_refinement, corrective_right_refinement});
          std::cout << "full_sampling_c3plus_corrective_batches"
                    << " exact=" << corrective_exact.num_feasible
                    << " perimeter=" << corrective_perimeter.num_feasible
                    << " mesh=" << corrective_mesh.num_feasible
                    << " left_refined="
                    << corrective_refinement.num_feasible
                    << " right_refined="
                    << corrective_right_refinement.num_feasible
                    << " total=" << corrective_buffer.total_candidates
                    << std::endl;
          const double desired_x_direction =
              params.object.goal_pose.x() - corrective_object_pose.x();
          const XarmFullSamplingC3CandidateReceipt* corrective_candidate =
              nullptr;
          bool corrective_contact_only_fallback = false;
          auto has_corrective_polarity = [&](const auto& candidate) {
            const Eigen::Vector2d normal_W =
                Eigen::Rotation2Dd(corrective_object_pose.z()) *
                candidate.sample_normal_O;
            const double force_x = -normal_W.x();
            return desired_x_direction * force_x > 0.0 &&
                std::abs(force_x) > 0.5;
          };
          const std::array<const XarmFullSamplingC3BatchReceipt*, 5>
              corrective_batches = {
                  &corrective_exact, &corrective_perimeter, &corrective_mesh,
                  &corrective_refinement, &corrective_right_refinement};
          std::vector<const XarmFullSamplingC3CandidateReceipt*>
              ranked_corrective_candidates;
          for (const auto& candidate : corrective_buffer.successful) {
            ranked_corrective_candidates.push_back(&candidate);
          }
          std::vector<const XarmFullSamplingC3CandidateReceipt*>
              ranked_corrective_contact_candidates;
          for (const auto* batch : corrective_batches) {
            for (const auto& candidate : batch->candidates) {
              if (!candidate.solve.dynamic_rollout_accepted ||
                  candidate.solve.planned_input_bound_violation > 1.0e-12 ||
                  !std::isfinite(candidate.solve.dynamic_rollout_cost)) {
                continue;
              }
              ranked_corrective_contact_candidates.push_back(&candidate);
            }
          }
          std::stable_sort(
              ranked_corrective_contact_candidates.begin(),
              ranked_corrective_contact_candidates.end(),
              [](const auto* a, const auto* b) {
                return a->solve.dynamic_rollout_cost <
                    b->solve.dynamic_rollout_cost;
              });
          read_full_tip();
          const Eigen::VectorXd corrective_start_q =
              acquisition_plant.GetPositions(*acquisition_context);
          auto select_live_corrective_candidate =
              [&](const auto& ranked) {
                for (const auto* candidate : ranked) {
                  if (!IsFullSamplingC3CentralSideContact(
                          candidate->sample_height_O,
                          params.object.resting_height,
                          params.controller.pusher_radius)) {
                    continue;
                  }
                  if (!has_corrective_polarity(*candidate)) continue;
                  const auto live_receipt =
                      EvaluateMeasuredCandidateAcquisition(
                          acquisition_plant, acquisition_context.get(), params,
                          corrective_start_q, corrective_object_pose,
                          *candidate, acquisition_sample);
                  const bool live_accepted =
                      live_receipt.swept_capsule_clear &&
                      live_receipt.ik_reached;
                  std::cout <<
                      "full_sampling_c3plus_corrective_live_ik="
                            << (live_accepted ? "PASS" : "FAIL")
                            << " sample=" << candidate->sample_name
                            << " ik_steps=" << live_receipt.ik_steps
                            << " failed_waypoint="
                            << live_receipt.failed_waypoint
                            << " swept_capsule_clear="
                            << live_receipt.swept_capsule_clear
                            << " failure_ik_solved="
                            << live_receipt.failure_ik_solved
                            << " failure_shaft_clear="
                            << live_receipt.failure_clearance.shaft_t_clear
                            << " failure_capsule_table_clear="
                            << live_receipt.failure_clearance
                                   .capsule_table_clear
                            << " central_side_contact=1"
                            << " sample_height_O="
                            << candidate->sample_height_O
                            << std::endl;
                  if (live_accepted) return candidate;
                }
                return static_cast<const
                    XarmFullSamplingC3CandidateReceipt*>(nullptr);
              };
          corrective_candidate = select_live_corrective_candidate(
              ranked_corrective_candidates);
          if (corrective_candidate == nullptr) {
            corrective_candidate = select_live_corrective_candidate(
                ranked_corrective_contact_candidates);
            corrective_contact_only_fallback =
                corrective_candidate != nullptr;
          }
          if (corrective_candidate != nullptr) {
            ContactFace corrective_face = ContactFace::kCrossbarTop;
            if (corrective_candidate->sample_name.find("stem_right") !=
                std::string::npos) {
              corrective_face = ContactFace::kStemRight;
            } else if (corrective_candidate->sample_name.find("stem_left") !=
                       std::string::npos) {
              corrective_face = ContactFace::kStemLeft;
            } else if (corrective_candidate->sample_name.find(
                           "crossbar_right") != std::string::npos) {
              corrective_face = ContactFace::kCrossbarRight;
            } else if (corrective_candidate->sample_name.find(
                           "crossbar_left") != std::string::npos) {
              corrective_face = ContactFace::kCrossbarLeft;
            }
            const ContactSample corrective_sample{
                corrective_candidate->sample_point_O,
                corrective_candidate->sample_normal_O,
                corrective_candidate->sample_name.c_str(), corrective_face};
            const Eigen::Vector2d corrective_normal_W =
                Eigen::Rotation2Dd(corrective_object_pose.z()) *
                corrective_candidate->sample_normal_O;
            const Eigen::Vector3d corrective_contact =
                corrective_candidate->initial_pusher_position_W;
            Eigen::Vector3d corrective_standoff = corrective_contact;
            corrective_standoff.head<2>() += corrective_normal_W *
                (params.controller.descent_clearance +
                 0.5 * params.controller.contact_activation_tolerance);
            Eigen::Vector3d corrective_high = corrective_standoff;
            corrective_high.z() = params.controller.reposition_waypoint_height;
            Eigen::Vector3d corrective_traverse = corrective_high;
            corrective_traverse.z() = std::max(
                lift.z(), CapsuleOrientationClearanceHeight(params));
            std::cout << "full_sampling_c3plus_corrective_resample=PASS"
                      << " sample=" << corrective_candidate->sample_name
                      << " candidates=" << corrective_buffer.total_candidates
                      << " force_x=" << -corrective_normal_W.x()
                      << " workspace_horizon="
                      << corrective_candidate->solve
                             .dynamic_rollout_workspace_accepted
                      << " contact_only_fallback="
                      << corrective_contact_only_fallback
                      << " object_pose=" << corrective_object_pose.transpose()
                      << std::endl;
            // The lift preserves the departing face's stick-axis command.
            // Reorient at the geometry-derived clearance height before any
            // lateral traverse toward the newly selected corrective face.
            const bool verticalized = execute_posture_waypoint(
                lift, corrective_object_pose, corrective_sample,
                "verticalize", false, false);
            const bool traversed = verticalized && execute_posture_waypoint(
                corrective_traverse, corrective_object_pose,
                corrective_sample, "traverse_clear", false, false);
            const bool lowered = traversed && execute_posture_waypoint(
                corrective_high, corrective_object_pose, corrective_sample,
                "lower", false, false);
            const bool descended = lowered && execute_posture_waypoint(
                corrective_standoff, corrective_object_pose,
                corrective_sample, "descend", false, false);
            corrective_contact_handoff = descended &&
                execute_posture_waypoint(
                    corrective_contact, corrective_object_pose,
                    corrective_sample, "contact", false, true, false, true);
            std::cout << "full_sampling_c3plus_corrective_contact_handoff="
                      << (corrective_contact_handoff ? "PASS" : "FAIL")
                      << " updates=" << full_execution_updates << std::endl;
            if (corrective_contact_handoff) {
              const double recovery_start_signed_error =
                  read_full_object_x() - params.object.goal_pose.x();
              const double recovery_start_drift =
                  std::abs(recovery_start_signed_error);
              double best_recovery_drift = recovery_start_drift;
              int corrective_response_steps = 0;
              bool corrective_response_rejected = false;
              while (best_recovery_drift >
                         progress_lateral_reserve_limit &&
                     full_execution_updates < FLAGS_full_execution_steps) {
                const int64_t next_utime =
                    full_state_subscriber.message().utime +
                    1000LL * FLAGS_full_execution_period_ms;
                while (full_state_subscriber.message().utime < next_utime) {
                  full_lcm.HandleSubscriptions(100);
                  while (full_lcm.HandleSubscriptions(0) > 0) {
                  }
                }
                ++full_execution_updates;
                ++corrective_response_steps;
                const Eigen::Vector3d measured_object =
                    read_full_object_pose();
                const double measured_signed_error =
                    measured_object.x() - params.object.goal_pose.x();
                best_recovery_drift = std::min(
                    best_recovery_drift, std::abs(measured_signed_error));
                const Eigen::Rotation2Dd measured_R_WO(measured_object.z());
                const Eigen::Vector2d measured_normal_W =
                    measured_R_WO * corrective_candidate->sample_normal_O;
                const Eigen::Vector2d measured_boundary_W =
                    measured_object.head<2>() + measured_R_WO *
                        corrective_candidate->sample_point_O;
                const double measured_gap = measured_normal_W.dot(
                    read_full_tip().head<2>() - measured_boundary_W) -
                    params.controller.pusher_radius;
                const bool wrong_polarity_response =
                    IsFullSamplingC3WrongPolarityResponse(
                        recovery_start_signed_error, measured_signed_error,
                        corrective_response_steps,
                        params.controller.successor_minimum_contact_steps);
                const bool contact_lost =
                    corrective_response_steps >=
                        params.controller.successor_minimum_contact_steps &&
                    measured_gap >=
                        params.controller.contact_activation_tolerance;
                if (wrong_polarity_response || contact_lost) {
                  corrective_response_rejected = true;
                  std::cout <<
                      "full_sampling_c3plus_corrective_response=REJECT"
                            << " reason="
                            << (contact_lost ? "contact_lost" :
                                "wrong_polarity")
                            << " start_signed_error_m="
                            << recovery_start_signed_error
                            << " measured_signed_error_m="
                            << measured_signed_error
                            << " measured_gap_m=" << measured_gap
                            << " response_steps=" << corrective_response_steps
                            << " updates=" << full_execution_updates
                            << std::endl;
                  break;
                }
              }
              const bool corridor_reached = best_recovery_drift <=
                  progress_lateral_reserve_limit;
              bool corrective_released = false;
              while ((corridor_reached || corrective_response_rejected) &&
                     !corrective_released &&
                     full_execution_updates < FLAGS_full_execution_steps) {
                const Eigen::Vector3d release_object_pose =
                    read_full_object_pose();
                const Eigen::Rotation2Dd release_R_WO(
                    release_object_pose.z());
                const Eigen::Vector2d release_normal_W =
                    release_R_WO * corrective_candidate->sample_normal_O;
                const Eigen::Vector2d release_boundary_W =
                    release_object_pose.head<2>() +
                    release_R_WO * corrective_candidate->sample_point_O;
                const Eigen::Vector3d release_tip = read_full_tip();
                const double release_gap = release_normal_W.dot(
                    release_tip.head<2>() - release_boundary_W) -
                    params.controller.pusher_radius;
                if (release_gap >= params.controller.approach_clearance) {
                  corrective_released = true;
                  break;
                }
                Eigen::Vector3d release_step = release_tip;
                // The waypoint executor considers translations inside the
                // existing contact-activation tolerance complete.  Include
                // that tolerance in the requested outward motion so the last
                // sub-clearance release step cannot become a zero-update
                // loop.
                release_step.head<2>() += release_normal_W * std::min(
                    params.controller.task_space_plan_step_limit,
                    params.controller.approach_clearance - release_gap +
                        params.controller.contact_activation_tolerance);
                const int updates_before_release = full_execution_updates;
                if (!execute_posture_waypoint(
                        release_step, release_object_pose, corrective_sample,
                        "corrective_release_step", true, false)) {
                  break;
                }
                if (full_execution_updates == updates_before_release) {
                  std::cout <<
                      "full_sampling_c3plus_corrective_release_progress=FAIL"
                            << " gap_m=" << release_gap
                            << " updates=" << full_execution_updates
                            << std::endl;
                  break;
                }
              }
              std::cout << "full_sampling_c3plus_dynamic_release="
                        << (corrective_released ? "PASS" : "FAIL")
                        << " updates=" << full_execution_updates << std::endl;
              const double released_drift = std::abs(
                  read_full_object_x() - params.object.goal_pose.x());
              const bool measured_recovery_accepted = corrective_released &&
                  released_drift <= progress_lateral_reserve_limit;
              const bool corrective_release_rebound =
                  corrective_released && corridor_reached &&
                  !measured_recovery_accepted;
              // A rebound remains a failed recovery measurement, but the
              // pusher is physically clear and the live pose is valid input
              // to the next solve. Continue replanning without reclassifying
              // the unchanged lateral tolerance.
              corrective_lateral_recovery =
                  measured_recovery_accepted || corrective_release_rebound;
              std::cout <<
                  "full_sampling_c3plus_measured_lateral_recovery="
                        << (measured_recovery_accepted ? "PASS" : "FAIL")
                        << " start_drift_m=" << recovery_start_drift
                        << " best_drift_m=" << best_recovery_drift
                        << " released_drift_m=" << released_drift
                        << " reserve_limit_m="
                        << progress_lateral_reserve_limit
                        << " updates=" << full_execution_updates
                        << std::endl;
              if (corrective_release_rebound) {
                std::cout <<
                    "full_sampling_c3plus_release_rebound_continuation=PASS"
                          << " released_drift_m=" << released_drift
                          << " tolerance_m="
                          << params.controller.lateral_drift_tolerance
                          << " reserve_limit_m="
                          << progress_lateral_reserve_limit
                          << " updates=" << full_execution_updates
                          << std::endl;
              }
              std::vector<int> measured_release_recovery_updates{
                  full_execution_updates - initial_recovery_start_updates};
              std::cout <<
                  "full_sampling_c3plus_release_recovery_receipt=PASS"
                        << " phase=initial_corrective"
                        << " phase_updates="
                        << measured_release_recovery_updates.back()
                        << " receipts="
                        << measured_release_recovery_updates.size()
                        << std::endl;
              int progress_cycle_count = 0;
              std::string active_release_name =
                  corrective_candidate->sample_name;
              ContactSample active_release_sample = corrective_sample;
              active_release_sample.name = active_release_name.c_str();
              std::set<std::string> live_execution_rejections;
              std::set<std::string> progress_neutral_anchor_candidates;
              std::vector<XarmFullSamplingC3MeasuredResponse>
                  measured_response_history;
              std::vector<XarmFullSamplingC3MeasuredResponse>
                  recovery_response_history;
              while (corrective_lateral_recovery &&
                     full_execution_updates < FLAGS_full_execution_steps) {
                const int progress_cycle_entry_updates =
                    full_execution_updates;
                full_task_progress_cycle = false;
                if (!wait_for_planar_settle("progress_replan")) {
                  std::cout <<
                      "full_sampling_c3plus_progress_admission=FAIL"
                            << " reason=object_not_planar_settled"
                            << " updates=" << full_execution_updates
                            << std::endl;
                  corrective_lateral_recovery = false;
                  break;
                }
                const Eigen::Vector3d cycle_entry_pose =
                    read_full_object_pose();
                const double cycle_entry_lateral_drift = std::abs(
                    cycle_entry_pose.x() - params.object.goal_pose.x());
                const bool cycle_entry_has_reserve =
                    cycle_entry_lateral_drift <=
                    progress_lateral_reserve_limit;
                std::cout << "full_sampling_c3plus_lateral_reserve="
                          << (cycle_entry_has_reserve ? "PASS" : "FAIL")
                          << " drift_m=" << cycle_entry_lateral_drift
                          << " reserve_limit_m="
                          << progress_lateral_reserve_limit
                          << " outer_tolerance_m="
                          << params.controller.lateral_drift_tolerance
                          << " updates=" << full_execution_updates
                          << std::endl;
                if (!cycle_entry_has_reserve) {
                  corrective_lateral_recovery = false;
                  break;
                }
                const double cycle_entry_translation_error =
                    (cycle_entry_pose.head<2>() -
                     params.object.goal_pose.head<2>()).norm();
                const double cycle_entry_orientation_error = std::abs(
                    WrappedAngleError(params.object.goal_pose.z(),
                                      cycle_entry_pose.z()));
                if (cycle_entry_translation_error <=
                        params.task.translation_tolerance &&
                    cycle_entry_orientation_error <=
                        params.task.orientation_tolerance) {
                  full_task_progress_cycle = true;
                  std::cout <<
                      "full_sampling_c3plus_receding_terminal=PASS cycles="
                            << progress_cycle_count
                            << " object_pose="
                            << cycle_entry_pose.transpose()
                            << " translation_error_m="
                            << cycle_entry_translation_error
                            << " orientation_error_rad="
                            << cycle_entry_orientation_error << std::endl;
                  break;
                }
                if (!HasFullSamplingC3ContactDwellBudget(
                        full_execution_updates, FLAGS_full_execution_steps,
                        params.controller.successor_minimum_contact_steps)) {
                  contact_cycle_budget_deferred = true;
                  std::cout <<
                      "full_sampling_c3plus_contact_cycle_budget="
                      "DEFER"
                            << " updates=" << full_execution_updates
                            << " budget=" << FLAGS_full_execution_steps
                            << " remaining="
                            << FLAGS_full_execution_steps -
                                   full_execution_updates
                            << " required_contact_steps="
                            << params.controller
                                   .successor_minimum_contact_steps
                            << std::endl;
                  break;
                }
                if (progress_cycle_count > 0) {
                  bool cycle_entry_released = false;
                  while (!cycle_entry_released &&
                         full_execution_updates <
                             FLAGS_full_execution_steps) {
                    const Eigen::Vector3d release_object_pose =
                        read_full_object_pose();
                    const Eigen::Rotation2Dd release_R_WO(
                        release_object_pose.z());
                    const Eigen::Vector2d release_normal_W =
                        release_R_WO * active_release_sample.normal;
                    const Eigen::Vector2d release_boundary_W =
                        release_object_pose.head<2>() +
                        release_R_WO * active_release_sample.point;
                    const Eigen::Vector3d release_tip = read_full_tip();
                    const double release_gap = release_normal_W.dot(
                        release_tip.head<2>() - release_boundary_W) -
                        params.controller.pusher_radius;
                    if (release_gap >=
                        params.controller.approach_clearance) {
                      cycle_entry_released = true;
                      break;
                    }
                    Eigen::Vector3d release_step = release_tip;
                    release_step.head<2>() += release_normal_W * std::min(
                        params.controller.task_space_plan_step_limit,
                        params.controller.approach_clearance - release_gap +
                            params.controller.contact_activation_tolerance);
                    const int updates_before_release =
                        full_execution_updates;
                    if (!execute_posture_waypoint(
                            release_step, release_object_pose,
                            active_release_sample,
                            "progress_cycle_release", true, false) ||
                        full_execution_updates == updates_before_release) {
                      break;
                    }
                  }
                  std::cout <<
                      "full_sampling_c3plus_receding_release="
                            << (cycle_entry_released ? "PASS" : "FAIL")
                            << " cycle=" << progress_cycle_count
                            << " updates=" << full_execution_updates
                            << std::endl;
                  if (!cycle_entry_released) {
                    corrective_lateral_recovery = false;
                    break;
                  }
                }
                const Eigen::Vector3d progress_start_pose =
                    read_full_object_pose();
                const auto progress_start_velocity =
                    read_full_object_velocity();
                const auto progress_planar_velocity =
                    ConditionOpenTableObjectVelocity(
                        progress_start_velocity.first,
                        progress_start_velocity.second);
                OimTParams progress_params = params;
                progress_params.object.start_pose = progress_start_pose;
                progress_params.object.start_angular_velocity_W =
                    progress_planar_velocity.first;
                progress_params.object.start_linear_velocity_W =
                    progress_planar_velocity.second;
                // The five-knot model covers 0.25 s and cannot represent a
                // single transition to the terminal open-table pose 0.8 m
                // and approximately pi radians away.  Plan a measured-state
                // receding subgoal bounded by the unchanged terminal
                // tolerances, while all acceptance checks below continue to
                // use the original task goal.
                progress_params.object.goal_pose = progress_start_pose;
                progress_params.object.goal_pose.x() =
                    params.object.goal_pose.x();
                progress_params.object.goal_pose.y() += std::clamp(
                    params.object.goal_pose.y() - progress_start_pose.y(),
                    -params.task.translation_tolerance,
                    params.task.translation_tolerance);
                progress_params.object.goal_pose.z() += std::clamp(
                    WrappedAngleError(params.object.goal_pose.z(),
                                      progress_start_pose.z()),
                    -params.task.orientation_tolerance,
                    params.task.orientation_tolerance);
                auto build_progress_buffer = [](const OimTParams& solve_params) {
                  const auto exact =
                      RunXarmFullSamplingC3ExactTBatch(solve_params);
                  const auto perimeter =
                      RunXarmFullSamplingC3PerimeterBatchParallel(
                          solve_params);
                  const auto mesh =
                      RunXarmFullSamplingC3MeshBatchParallel(solve_params);
                  const auto left =
                      RunXarmFullSamplingC3StemLeftRefinementBatchParallel(
                          solve_params);
                  const auto right =
                      RunXarmFullSamplingC3StemRightRefinementBatchParallel(
                          solve_params);
                  OimTParams retry_one = solve_params;
                  ++retry_one.full_sampling_c3plus.random_seed;
                  ++retry_one.full_sampling_c3plus.mesh_random_seed;
                  const auto perimeter_retry_one =
                      RunXarmFullSamplingC3PerimeterBatchParallel(retry_one);
                  const auto mesh_retry_one =
                      RunXarmFullSamplingC3MeshBatchParallel(retry_one);
                  OimTParams retry_two = retry_one;
                  ++retry_two.full_sampling_c3plus.random_seed;
                  ++retry_two.full_sampling_c3plus.mesh_random_seed;
                  const auto perimeter_retry_two =
                      RunXarmFullSamplingC3PerimeterBatchParallel(retry_two);
                  const auto mesh_retry_two =
                      RunXarmFullSamplingC3MeshBatchParallel(retry_two);
                  OimTParams retry_three = retry_two;
                  ++retry_three.full_sampling_c3plus.random_seed;
                  ++retry_three.full_sampling_c3plus.mesh_random_seed;
                  const auto perimeter_retry_three =
                      RunXarmFullSamplingC3PerimeterBatchParallel(retry_three);
                  const auto mesh_retry_three =
                      RunXarmFullSamplingC3MeshBatchParallel(retry_three);
                  OimTParams retry_four = retry_three;
                  ++retry_four.full_sampling_c3plus.random_seed;
                  ++retry_four.full_sampling_c3plus.mesh_random_seed;
                  const auto perimeter_retry_four =
                      RunXarmFullSamplingC3PerimeterBatchParallel(retry_four);
                  const auto mesh_retry_four =
                      RunXarmFullSamplingC3MeshBatchParallel(retry_four);
                  return BuildXarmFullSamplingC3CandidateBuffer(
                      {exact, perimeter, mesh, left, right,
                       perimeter_retry_one, mesh_retry_one,
                       perimeter_retry_two, mesh_retry_two,
                       perimeter_retry_three, mesh_retry_three,
                       perimeter_retry_four, mesh_retry_four});
                };
                auto progress_buffer = build_progress_buffer(progress_params);
                if (progress_buffer.successful.empty() &&
                    (progress_params.object.start_angular_velocity_W.norm() >
                         0.0 ||
                     progress_params.object.start_linear_velocity_W.norm() >
                         0.0)) {
                  OimTParams quasistatic_params = progress_params;
                  quasistatic_params.object.start_angular_velocity_W.setZero();
                  quasistatic_params.object.start_linear_velocity_W.setZero();
                  progress_buffer = build_progress_buffer(quasistatic_params);
                  std::cout <<
                      "full_sampling_c3plus_progress_velocity_fallback="
                            << (progress_buffer.successful.empty() ? "FAIL" :
                                "PASS")
                            << " reason=measured_batch_empty"
                            << " measured_angular_velocity_W="
                            << progress_params.object
                                   .start_angular_velocity_W.transpose()
                            << " measured_linear_velocity_W="
                            << progress_params.object
                                   .start_linear_velocity_W.transpose()
                            << " executable="
                            << progress_buffer.successful.size()
                            << std::endl;
                }
                bool contact_feasible_replenishment = false;
                if (progress_buffer.successful.empty()) {
                  auto replenished =
                      BuildXarmFullSamplingC3ContactFeasibleCandidateBuffer(
                          progress_buffer);
                  contact_feasible_replenishment =
                      !replenished.successful.empty();
                  std::cout <<
                      "full_sampling_c3plus_post_yaw_replenishment="
                            << (contact_feasible_replenishment ? "PASS" :
                                "FAIL")
                            << " reason=workspace_filtered_batch_empty"
                            << " dynamic_contact_candidates="
                            << replenished.successful.size()
                            << " mandatory_live_ik=1"
                            << " mandatory_capsule_execution=1"
                            << std::endl;
                  if (contact_feasible_replenishment) {
                    progress_buffer = std::move(replenished);
                  }
                }
                auto candidate_terminal_pose = [](
                    const XarmFullSamplingC3CandidateReceipt& candidate)
                    -> std::optional<Eigen::Vector3d> {
                  if (candidate.solve.dynamic_state_trajectory.empty()) {
                    return std::nullopt;
                  }
                  const auto terminal_state = XarmFullSamplingC3State::Decode(
                      candidate.solve.dynamic_state_trajectory.back());
                  const Eigen::Matrix3d R_WO =
                      terminal_state.object_quaternion_WO.toRotationMatrix();
                  Eigen::Vector3d terminal_pose;
                  terminal_pose.head<2>() =
                      terminal_state.object_position_W.head<2>();
                  terminal_pose.z() = std::atan2(R_WO(1, 0), R_WO(0, 0));
                  if (!terminal_pose.allFinite()) return std::nullopt;
                  return terminal_pose;
                };
                auto condition_candidate = [&](const auto& candidate) {
                  XarmFullSamplingC3ResponseConditioningReceipt receipt;
                  const auto terminal_pose =
                      candidate_terminal_pose(candidate);
                  if (!terminal_pose.has_value()) {
                    receipt.ranking_class = 2;
                    return receipt;
                  }
                  return EvaluateFullSamplingC3MeasuredResponseConditioning(
                      progress_start_pose, *terminal_pose,
                      candidate.sample_point_O, candidate.sample_normal_O,
                      params.object.goal_pose, measured_response_history,
                      params.task.translation_tolerance,
                      params.task.orientation_tolerance,
                      params.controller.lateral_drift_tolerance,
                      params.controller.successor_minimum_translation_progress,
                      params.controller.successor_minimum_yaw_progress);
                };
                // Preserve dynamic-cost order within each evidence class.
                // Compatible measured neighborhoods lead, unseen contacts
                // retain an exploration path, and incompatible neighborhoods
                // remain available only as deterministic fall-through.
                std::stable_sort(
                    progress_buffer.successful.begin(),
                    progress_buffer.successful.end(),
                    [&](const auto& a, const auto& b) {
                      const auto a_response = condition_candidate(a);
                      const auto b_response = condition_candidate(b);
                      if (a_response.ranking_class !=
                          b_response.ranking_class) {
                        return a_response.ranking_class <
                            b_response.ranking_class;
                      }
                      if (a_response.ranking_class == 0 &&
                          a_response.calibrated_normalized_magnitude !=
                              b_response.calibrated_normalized_magnitude) {
                        return a_response.calibrated_normalized_magnitude >
                            b_response.calibrated_normalized_magnitude;
                      }
                      return false;
                    });
                std::array<int, 3> response_rank_counts{0, 0, 0};
                for (const auto& candidate : progress_buffer.successful) {
                  const int rank = condition_candidate(candidate).ranking_class;
                  if (rank >= 0 && rank < 3) ++response_rank_counts[rank];
                }
                std::cout <<
                    "full_sampling_c3plus_measured_response_ranking=PASS"
                          << " observations="
                          << measured_response_history.size()
                          << " compatible=" << response_rank_counts[0]
                          << " unseen=" << response_rank_counts[1]
                          << " incompatible=" << response_rank_counts[2]
                          << std::endl;
                XarmFullSamplingC3CandidateBuffer progress_execution_buffer;
                progress_execution_buffer.total_candidates =
                    progress_buffer.total_candidates;
                progress_execution_buffer.unsuccessful =
                    progress_buffer.unsuccessful;
                // Snapshot the actual six-joint posture once. Every ranked
                // candidate below is checked from this same measured state,
                // so arbitration cannot depend on mutations made by a prior
                // feasibility probe.
                read_full_tip();
                const Eigen::VectorXd progress_start_q =
                    acquisition_plant.GetPositions(*acquisition_context);
                std::vector<XarmFullSamplingC3CandidateReceipt>
                    monitored_progress_candidates;
                std::optional<FullCandidateAcquisitionReceipt>
                    selected_progress_acquisition;
                double monitored_progress_lateral =
                    std::numeric_limits<double>::infinity();
                auto evaluate_progress_candidates = [&](const auto& candidates) {
                for (const auto& full_candidate : candidates) {
                  if (live_execution_rejections.contains(
                          full_candidate.sample_name)) {
                    progress_execution_buffer.unsuccessful.push_back(
                        full_candidate);
                    std::cout <<
                        "full_sampling_c3plus_progress_candidate=SKIP"
                              << " sample=" << full_candidate.sample_name
                              << " reason=prior_live_execution_rejection"
                              << std::endl;
                    continue;
                  }
                  const auto corridor_prefix =
                      BuildXarmFullSamplingC3CorridorSafePrefix(
                          full_candidate, params.object.goal_pose.x(),
                          params.controller.lateral_drift_tolerance);
                  if (!corridor_prefix.accepted) {
                    progress_execution_buffer.unsuccessful.push_back(
                        full_candidate);
                    std::cout <<
                        "full_sampling_c3plus_progress_corridor_prefix=FAIL"
                              << " sample=" << full_candidate.sample_name
                              << " original_knots="
                              << corridor_prefix.original_state_knots
                              << " retained_knots="
                              << corridor_prefix.retained_state_knots
                              << std::endl;
                    continue;
                  }
                  const auto& candidate =
                      corridor_prefix.execution_candidate;
                  const double predicted_lateral = std::abs(
                      candidate.solve.dynamic_terminal_object_position.x() -
                      params.object.goal_pose.x());
                  const double predicted_y_progress =
                      std::abs(progress_start_pose.y() -
                               params.object.goal_pose.y()) -
                      std::abs(
                          candidate.solve.dynamic_terminal_object_position.y() -
                          params.object.goal_pose.y());
                  double predicted_yaw_progress =
                      -std::numeric_limits<double>::infinity();
                  XarmFullSamplingC3TerminalDescentReceipt
                      predicted_terminal_descent;
                  const auto predicted_terminal_pose =
                      candidate_terminal_pose(candidate);
                  if (predicted_terminal_pose.has_value()) {
                    const double terminal_yaw =
                        predicted_terminal_pose->z();
                    predicted_yaw_progress =
                        std::abs(WrappedAngleError(
                            params.object.goal_pose.z(),
                            progress_start_pose.z())) -
                        std::abs(WrappedAngleError(
                            params.object.goal_pose.z(), terminal_yaw));
                    predicted_terminal_descent =
                        EvaluateFullSamplingC3TerminalDescent(
                            progress_start_pose, *predicted_terminal_pose,
                            params.object.goal_pose,
                            params.controller
                                .successor_minimum_translation_progress,
                            params.controller
                                .successor_minimum_yaw_progress);
                  }
                  const auto response_conditioning =
                      condition_candidate(candidate);
                  const bool predicted_productive =
                      predicted_terminal_descent.accepted;
                  const bool measured_response_compatible =
                      response_conditioning.ranking_class == 0;
                  const bool measured_response_incompatible =
                      response_conditioning.ranking_class == 2;
                  const bool component_decomposed_candidate =
                      candidate.sample_name.rfind("translation_only_", 0) ==
                          0 ||
                      candidate.sample_name.rfind("rotation_only_", 0) == 0;
                  XarmFullSamplingC3ComponentTransactionReceipt
                      component_transaction;
                  if (component_decomposed_candidate &&
                      predicted_terminal_pose.has_value()) {
                    const Eigen::Vector3d& transaction_terminal =
                        response_conditioning.matching_observations > 0
                            ? response_conditioning
                                  .corrected_terminal_object_pose
                            : *predicted_terminal_pose;
                    component_transaction =
                        EvaluateXarmFullSamplingC3ComponentTransaction(
                            progress_start_pose, transaction_terminal,
                            params.object.goal_pose,
                            params.task.translation_tolerance,
                            params.task.orientation_tolerance,
                            params.controller
                                .successor_minimum_translation_progress,
                            params.controller
                                .successor_minimum_yaw_progress);
                  }
                  const bool response_conditioned_productive =
                      component_decomposed_candidate
                          ? component_transaction.accepted
                      : measured_response_compatible
                          ? response_conditioning
                                .corrected_terminal_accepted
                          : predicted_productive;
                  const double response_conditioned_lateral =
                      measured_response_compatible
                          ? response_conditioning.corrected_lateral_error
                          : predicted_lateral;
                  const Eigen::Rotation2Dd progress_R_WO(
                      progress_start_pose.z());
                  const Eigen::Vector2d initial_object_force_W =
                      -(progress_R_WO * candidate.sample_normal_O);
                  const Eigen::Vector2d initial_lever_W =
                      progress_R_WO * candidate.sample_point_O;
                  const double initial_object_moment =
                      initial_lever_W.x() * initial_object_force_W.y() -
                      initial_lever_W.y() * initial_object_force_W.x();
                  const double desired_y_direction =
                      params.object.goal_pose.y() - progress_start_pose.y();
                  const double desired_yaw_direction = WrappedAngleError(
                      params.object.goal_pose.z(), progress_start_pose.z());
                  const auto initial_wrench =
                      EvaluateXarmFullSamplingC3ParetoWrench(
                          desired_y_direction, initial_object_force_W.y(),
                          desired_yaw_direction, initial_object_moment);
                  bool predicted_capsule_table_clear =
                      candidate.initial_pusher_position_W.z() >=
                      params.controller.pusher_radius;
                  for (const Eigen::VectorXd& state :
                       candidate.solve.dynamic_state_trajectory) {
                    predicted_capsule_table_clear =
                        predicted_capsule_table_clear &&
                        state[XarmFullSamplingC3State::kPusherPosition + 2] >=
                            params.controller.pusher_radius;
                  }
                  const bool central_side_contact =
                      IsFullSamplingC3CentralSideContact(
                          candidate.sample_height_O,
                          params.object.resting_height,
                          params.controller.pusher_radius);
                  const bool predicted_lateral_safe =
                      predicted_lateral <=
                      params.controller.lateral_drift_tolerance;
                  const bool corrected_lateral_safe =
                      response_conditioning.matching_observations == 0 ||
                      response_conditioning.corrected_lateral_error <=
                          params.controller.lateral_drift_tolerance;
                  std::cout <<
                      "full_sampling_c3plus_progress_candidate name="
                            << candidate.sample_name
                            << " predicted_lateral_m=" << predicted_lateral
                            << " predicted_y_progress_m="
                            << predicted_y_progress
                            << " predicted_yaw_progress_rad="
                            << predicted_yaw_progress
                            << " predicted_translation_progress_m="
                            << predicted_terminal_descent
                                   .translation_progress
                            << " terminal_translation_nonregressive="
                            << predicted_terminal_descent
                                   .translation_nonregressive
                            << " terminal_orientation_nonregressive="
                            << predicted_terminal_descent
                                   .orientation_nonregressive
                            << " response_observations="
                            << response_conditioning.matching_observations
                            << " response_rank_class="
                            << response_conditioning.ranking_class
                            << " corrected_lateral_m="
                            << response_conditioning.corrected_lateral_error
                            << " corrected_terminal_accepted="
                            << response_conditioning
                                   .corrected_terminal_accepted
                            << " component_transaction="
                            << component_decomposed_candidate
                            << " component_transaction_accepted="
                            << component_transaction.accepted
                            << " component_normalized_magnitude="
                            << component_transaction.normalized_magnitude
                            << " component_translation_debt_bounded="
                            << component_transaction.translation_debt_bounded
                            << " component_orientation_debt_bounded="
                            << component_transaction.orientation_debt_bounded
                            << " observed_progress_gain="
                            << response_conditioning.observed_progress_gain
                            << " calibrated_normalized_magnitude="
                            << response_conditioning
                                   .calibrated_normalized_magnitude
                            << " ranking_lateral_m="
                            << response_conditioned_lateral
                            << " wrench_productive="
                            << initial_wrench.accepted
                            << " wrench_translation_alignment="
                            << initial_wrench.translation_alignment
                            << " wrench_orientation_alignment="
                            << initial_wrench.orientation_alignment
                            << " capsule_table_clear="
                            << predicted_capsule_table_clear
                            << " central_side_contact="
                            << central_side_contact
                            << " sample_height_O="
                            << candidate.sample_height_O
                            << " original_knots="
                            << corridor_prefix.original_state_knots
                            << " retained_knots="
                            << corridor_prefix.retained_state_knots
                            << " full_terminal_lateral_m="
                            << std::abs(
                                full_candidate.solve
                                        .dynamic_terminal_object_position.x() -
                                params.object.goal_pose.x())
                            << " predicted_lateral_safe="
                            << predicted_lateral_safe
                            << " corrected_lateral_safe="
                            << corrected_lateral_safe << std::endl;
                  const bool monitored_candidate =
                      response_conditioned_productive &&
                      initial_wrench.accepted &&
                      predicted_capsule_table_clear &&
                      central_side_contact && predicted_lateral_safe &&
                      corrected_lateral_safe;
                  const bool strict_candidate = monitored_candidate &&
                      !measured_response_incompatible &&
                      response_conditioned_lateral <=
                          progress_lateral_reserve_limit;
                  if (strict_candidate) {
                    // Never rotate the long pusher in place at a far-reach
                    // contact posture. Lift above the entire T, retreat to
                    // the neutral workspace, and verticalize there.
                    bool used_neutral_anchor = true;
                    auto live_receipt =
                        EvaluateMeasuredCandidateAcquisition(
                            acquisition_plant, acquisition_context.get(),
                            params, progress_start_q, progress_start_pose,
                            candidate, active_release_sample,
                            used_neutral_anchor);
                    bool live_accepted =
                        live_receipt.swept_capsule_clear &&
                        live_receipt.ik_reached;
                    if (!live_accepted && !used_neutral_anchor) {
                      live_receipt = EvaluateMeasuredCandidateAcquisition(
                          acquisition_plant, acquisition_context.get(),
                          params, progress_start_q, progress_start_pose,
                          candidate, active_release_sample, true);
                      used_neutral_anchor = true;
                      live_accepted = live_receipt.swept_capsule_clear &&
                          live_receipt.ik_reached;
                    }
                    std::cout <<
                        "full_sampling_c3plus_progress_live_ik="
                              << (live_accepted ? "PASS" : "FAIL")
                              << " sample=" << candidate.sample_name
                              << " ik_steps=" << live_receipt.ik_steps
                              << " failed_waypoint="
                              << live_receipt.failed_waypoint
                              << " swept_capsule_clear="
                              << live_receipt.swept_capsule_clear
                              << " failure_ik_solved="
                              << live_receipt.failure_ik_solved
                              << " failure_shaft_clear="
                              << live_receipt.failure_clearance.shaft_t_clear
                              << " failure_capsule_table_clear="
                              << live_receipt.failure_clearance
                                     .capsule_table_clear
                              << " neutral_anchor="
                              << used_neutral_anchor
                              << " home_seed_fallback="
                              << live_receipt.used_home_seed_fallback
                              << std::endl;
                    if (live_accepted) {
                      if (used_neutral_anchor) {
                        progress_neutral_anchor_candidates.insert(
                            candidate.sample_name);
                      }
                      if (!selected_progress_acquisition.has_value()) {
                        selected_progress_acquisition = live_receipt;
                      }
                      progress_execution_buffer.successful.push_back(
                          candidate);
                    } else {
                      progress_execution_buffer.unsuccessful.push_back(
                          candidate);
                    }
                  } else {
                    progress_execution_buffer.unsuccessful.push_back(candidate);
                  }
                  if (monitored_candidate && !strict_candidate) {
                    monitored_progress_candidates.push_back(candidate);
                  }
                }
                };
                evaluate_progress_candidates(progress_buffer.successful);
                bool monitored_lateral_fallback = false;
                auto try_monitored_lateral_fallback = [&]() {
                if (progress_execution_buffer.successful.empty() &&
                    !monitored_progress_candidates.empty()) {
                  std::stable_sort(
                      monitored_progress_candidates.begin(),
                      monitored_progress_candidates.end(),
                      [&](const auto& a, const auto& b) {
                        const int a_response_rank =
                            condition_candidate(a).ranking_class;
                        const int b_response_rank =
                            condition_candidate(b).ranking_class;
                        if (a_response_rank != b_response_rank) {
                          return a_response_rank < b_response_rank;
                        }
                        return std::abs(
                                   a.solve
                                           .dynamic_terminal_object_position
                                           .x() -
                                   params.object.goal_pose.x()) <
                            std::abs(
                                   b.solve
                                           .dynamic_terminal_object_position
                                           .x() -
                                   params.object.goal_pose.x());
                      });
                  std::optional<XarmFullSamplingC3CandidateReceipt>
                      monitored_progress_candidate;
                  for (const auto& candidate :
                       monitored_progress_candidates) {
                    bool used_neutral_anchor = true;
                    auto live_receipt =
                        EvaluateMeasuredCandidateAcquisition(
                            acquisition_plant, acquisition_context.get(),
                            params, progress_start_q, progress_start_pose,
                            candidate, active_release_sample,
                            used_neutral_anchor);
                    bool live_accepted =
                        live_receipt.swept_capsule_clear &&
                        live_receipt.ik_reached;
                    if (!live_accepted && !used_neutral_anchor) {
                      live_receipt = EvaluateMeasuredCandidateAcquisition(
                          acquisition_plant, acquisition_context.get(),
                          params, progress_start_q, progress_start_pose,
                          candidate, active_release_sample, true);
                      used_neutral_anchor = true;
                      live_accepted = live_receipt.swept_capsule_clear &&
                          live_receipt.ik_reached;
                    }
                    std::cout <<
                        "full_sampling_c3plus_progress_live_ik="
                              << (live_accepted ? "PASS" : "FAIL")
                              << " sample=" << candidate.sample_name
                              << " monitored=1 ik_steps="
                              << live_receipt.ik_steps
                              << " failed_waypoint="
                              << live_receipt.failed_waypoint
                              << " swept_capsule_clear="
                              << live_receipt.swept_capsule_clear
                              << " failure_ik_solved="
                              << live_receipt.failure_ik_solved
                              << " failure_shaft_clear="
                              << live_receipt.failure_clearance.shaft_t_clear
                              << " failure_capsule_table_clear="
                              << live_receipt.failure_clearance
                                     .capsule_table_clear
                              << " neutral_anchor="
                              << used_neutral_anchor
                              << " home_seed_fallback="
                              << live_receipt.used_home_seed_fallback
                              << std::endl;
                    if (live_accepted) {
                      if (used_neutral_anchor) {
                        progress_neutral_anchor_candidates.insert(
                            candidate.sample_name);
                      }
                      selected_progress_acquisition = live_receipt;
                      monitored_progress_candidate = candidate;
                      monitored_progress_lateral = std::abs(
                          candidate.solve
                                  .dynamic_terminal_object_position.x() -
                          params.object.goal_pose.x());
                      break;
                    }
                  }
                  if (monitored_progress_candidate.has_value()) {
                    progress_execution_buffer.successful.push_back(
                        *monitored_progress_candidate);
                  const auto rejected = std::find_if(
                      progress_execution_buffer.unsuccessful.begin(),
                      progress_execution_buffer.unsuccessful.end(),
                      [&](const auto& candidate) {
                        return candidate.sample_name ==
                                   monitored_progress_candidate->sample_name &&
                            (candidate.initial_pusher_position_W -
                             monitored_progress_candidate
                                 ->initial_pusher_position_W)
                                    .norm() <= 1e-12;
                      });
                  if (rejected !=
                      progress_execution_buffer.unsuccessful.end()) {
                    progress_execution_buffer.unsuccessful.erase(rejected);
                  }
                  monitored_lateral_fallback = true;
                  }
                }
                };
                try_monitored_lateral_fallback();
                if (progress_execution_buffer.successful.empty() &&
                    !contact_feasible_replenishment) {
                  auto replenished =
                      BuildXarmFullSamplingC3ContactFeasibleCandidateBuffer(
                          progress_buffer);
                  contact_feasible_replenishment =
                      !replenished.successful.empty();
                  std::stable_sort(
                      replenished.successful.begin(),
                      replenished.successful.end(),
                      [&](const auto& a, const auto& b) {
                        return condition_candidate(a).ranking_class <
                            condition_candidate(b).ranking_class;
                      });
                  std::cout <<
                      "full_sampling_c3plus_post_filter_replenishment="
                            << (contact_feasible_replenishment ? "PASS" :
                                "FAIL")
                            << " reason=no_corridor_safe_executable_candidate"
                            << " workspace_candidates="
                            << progress_buffer.successful.size()
                            << " dynamic_contact_candidates="
                            << replenished.successful.size()
                            << " mandatory_live_ik=1"
                            << " mandatory_capsule_execution=1"
                            << " mandatory_lateral_corridor=1"
                            << std::endl;
                  if (contact_feasible_replenishment) {
                    progress_execution_buffer = {};
                    progress_execution_buffer.total_candidates =
                        replenished.total_candidates;
                    progress_execution_buffer.unsuccessful =
                        replenished.unsuccessful;
                    monitored_progress_candidates.clear();
                    selected_progress_acquisition.reset();
                    monitored_progress_lateral =
                        std::numeric_limits<double>::infinity();
                    evaluate_progress_candidates(replenished.successful);
                    try_monitored_lateral_fallback();
                    progress_buffer = std::move(replenished);
                  }
                }
                bool component_decomposed_fallback = false;
                if (progress_execution_buffer.successful.empty()) {
                  OimTParams translation_only_params = progress_params;
                  translation_only_params.object.goal_pose.z() =
                      progress_start_pose.z();
                  OimTParams rotation_only_params = progress_params;
                  rotation_only_params.object.goal_pose.y() =
                      progress_start_pose.y();
                  auto translation_only =
                      build_progress_buffer(translation_only_params);
                  auto rotation_only =
                      build_progress_buffer(rotation_only_params);
                  auto qualify_component = [](const std::string& component,
                                              auto* buffer) {
                    for (auto& candidate : buffer->successful) {
                      candidate.sample_name = component + "_" +
                          candidate.sample_name;
                    }
                    for (auto& candidate : buffer->unsuccessful) {
                      candidate.sample_name = component + "_" +
                          candidate.sample_name;
                    }
                  };
                  qualify_component("translation_only", &translation_only);
                  qualify_component("rotation_only", &rotation_only);
                  XarmFullSamplingC3CandidateBuffer component_buffer;
                  component_buffer.total_candidates =
                      translation_only.total_candidates +
                      rotation_only.total_candidates;
                  component_buffer.successful =
                      std::move(translation_only.successful);
                  component_buffer.successful.insert(
                      component_buffer.successful.end(),
                      std::make_move_iterator(
                          rotation_only.successful.begin()),
                      std::make_move_iterator(
                          rotation_only.successful.end()));
                  component_buffer.unsuccessful =
                      std::move(translation_only.unsuccessful);
                  component_buffer.unsuccessful.insert(
                      component_buffer.unsuccessful.end(),
                      std::make_move_iterator(
                          rotation_only.unsuccessful.begin()),
                      std::make_move_iterator(
                          rotation_only.unsuccessful.end()));
                  std::stable_sort(
                      component_buffer.successful.begin(),
                      component_buffer.successful.end(),
                      [&](const auto& a, const auto& b) {
                        return condition_candidate(a).ranking_class <
                            condition_candidate(b).ranking_class;
                      });
                  progress_execution_buffer = {};
                  progress_execution_buffer.total_candidates =
                      component_buffer.total_candidates;
                  progress_execution_buffer.unsuccessful =
                      component_buffer.unsuccessful;
                  monitored_progress_candidates.clear();
                  selected_progress_acquisition.reset();
                  monitored_progress_lateral =
                      std::numeric_limits<double>::infinity();
                  evaluate_progress_candidates(
                      component_buffer.successful);
                  try_monitored_lateral_fallback();
                  component_decomposed_fallback =
                      !progress_execution_buffer.successful.empty();
                  std::cout <<
                      "full_sampling_c3plus_component_decomposed_search="
                            << (component_decomposed_fallback ? "PASS" :
                                "FAIL")
                            << " candidates="
                            << component_buffer.total_candidates
                            << " executable="
                            << component_buffer.successful.size()
                            << " progress_accepted="
                            << progress_execution_buffer.successful.size()
                            << " global_pareto_gate=1"
                            << " measured_response_gate=1"
                            << " live_ik_gate=1 capsule_gate=1"
                            << std::endl;
                  progress_buffer = std::move(component_buffer);
                }
                progress_execution_buffer.accepted =
                    progress_execution_buffer.total_candidates > 0 &&
                    !progress_execution_buffer.successful.empty() &&
                    progress_execution_buffer.total_candidates ==
                        static_cast<int>(
                            progress_execution_buffer.successful.size() +
                            progress_execution_buffer.unsuccessful.size());
                const auto progress_plan =
                    BuildXarmFullSamplingC3TaskSpacePlan(
                        progress_execution_buffer,
                        params.task.planning_time_step);
                XarmFullSamplingC3ResponseConditioningReceipt
                    selected_response_conditioning;
                if (!progress_execution_buffer.successful.empty()) {
                  selected_response_conditioning = condition_candidate(
                      progress_execution_buffer.successful.front());
                }
                std::cout << "full_sampling_c3plus_progress_resample="
                          << (progress_plan.accepted ? "PASS" : "FAIL")
                          << " candidates="
                          << progress_buffer.total_candidates
                          << " executable="
                          << progress_buffer.successful.size()
                          << " progress_accepted="
                          << progress_execution_buffer.successful.size()
                          << " monitored_lateral_fallback="
                          << monitored_lateral_fallback
                          << " component_decomposed_fallback="
                          << component_decomposed_fallback
                          << " selected_predicted_lateral_m="
                          << (monitored_lateral_fallback
                                  ? monitored_progress_lateral
                                  : 0.0)
                          << " selected_response_rank_class="
                          << selected_response_conditioning.ranking_class
                          << " selected_response_observations="
                          << selected_response_conditioning
                                 .matching_observations
                          << " selected_corrected_lateral_m="
                          << selected_response_conditioning
                                 .corrected_lateral_error
                          << " contact_feasible_replenishment="
                          << contact_feasible_replenishment
                          << " sample=" << progress_plan.sample_name
                          << " sample_height_O="
                          << progress_plan.sample_height_O
                          << " central_side_contact="
                          << (progress_plan.accepted &&
                              IsFullSamplingC3CentralSideContact(
                                  progress_plan.sample_height_O,
                                  params.object.resting_height,
                                  params.controller.pusher_radius))
                          << " object_pose="
                          << progress_start_pose.transpose()
                          << " object_angular_velocity_W="
                          << progress_start_velocity.first.transpose()
                          << " object_linear_velocity_W="
                          << progress_start_velocity.second.transpose()
                          << " conditioned_angular_velocity_W="
                          << progress_params.object.start_angular_velocity_W
                                 .transpose()
                          << " conditioned_linear_velocity_W="
                          << progress_params.object.start_linear_velocity_W
                                 .transpose()
                          << " subgoal="
                          << progress_params.object.goal_pose.transpose()
                          << " pusher_knots_W="
                          << progress_plan.pusher_positions_W << std::endl;
                if (progress_plan.accepted) {
                  if (!selected_progress_acquisition.has_value()) {
                    std::cout <<
                        "full_sampling_c3plus_measured_cycle_budget=FAIL"
                              << " reason=selected_live_ik_receipt_missing"
                              << " sample=" << progress_plan.sample_name
                              << " updates=" << full_execution_updates
                              << std::endl;
                    corrective_lateral_recovery = false;
                    break;
                  }
                  const auto measured_cycle_budget =
                      EvaluateFullSamplingC3CycleBudget(
                          full_execution_updates,
                          FLAGS_full_execution_steps,
                          selected_progress_acquisition->ik_steps,
                          params.task.planning_time_step,
                          FLAGS_full_execution_period_ms,
                          params.controller.successor_minimum_contact_steps,
                          measured_release_recovery_updates);
                  std::cout <<
                      "full_sampling_c3plus_measured_cycle_budget="
                            << (measured_cycle_budget.accepted ? "PASS" :
                                "DEFER")
                            << " sample=" << progress_plan.sample_name
                            << " updates=" << full_execution_updates
                            << " budget=" << FLAGS_full_execution_steps
                            << " remaining_updates="
                            << measured_cycle_budget.remaining_updates
                            << " acquisition_ik_steps="
                            << selected_progress_acquisition->ik_steps
                            << " acquisition_updates="
                            << measured_cycle_budget.acquisition_updates
                            << " contact_dwell_updates="
                            << measured_cycle_budget.contact_dwell_updates
                            << " release_recovery_updates="
                            << measured_cycle_budget.release_recovery_updates
                            << " measured_receipts="
                            << measured_cycle_budget
                                   .measured_release_recovery_receipts
                            << " required_updates="
                            << measured_cycle_budget.required_updates
                            << std::endl;
                  if (!measured_cycle_budget.accepted) {
                    contact_cycle_budget_deferred = true;
                    break;
                  }
                  ContactFace progress_face = ContactFace::kCrossbarTop;
                  if (progress_plan.sample_name.find("stem_right") !=
                      std::string::npos) {
                    progress_face = ContactFace::kStemRight;
                  } else if (progress_plan.sample_name.find("stem_left") !=
                             std::string::npos) {
                    progress_face = ContactFace::kStemLeft;
                  } else if (progress_plan.sample_name.find(
                                 "stem_bottom") != std::string::npos) {
                    progress_face = ContactFace::kStemBottom;
                  } else if (progress_plan.sample_name.find(
                                 "crossbar_right") != std::string::npos) {
                    progress_face = ContactFace::kCrossbarRight;
                  } else if (progress_plan.sample_name.find(
                                 "crossbar_left") != std::string::npos) {
                    progress_face = ContactFace::kCrossbarLeft;
                  }
                  const ContactSample progress_sample{
                      progress_plan.sample_point_O,
                      progress_plan.sample_normal_O,
                      progress_plan.sample_name.c_str(), progress_face};
                  const Eigen::Vector2d progress_normal_W =
                      Eigen::Rotation2Dd(progress_start_pose.z()) *
                      progress_plan.sample_normal_O;
                  const Eigen::Vector3d progress_contact =
                      progress_plan.pusher_positions_W.col(0);
                  Eigen::Vector3d progress_standoff = progress_contact;
                  progress_standoff.head<2>() += progress_normal_W *
                      (params.controller.descent_clearance +
                       0.5 * params.controller.contact_activation_tolerance);
                  Eigen::Vector3d progress_high = progress_standoff;
                  progress_high.z() =
                      params.controller.reposition_waypoint_height;
                  const bool use_progress_neutral_anchor =
                      true;
                  Eigen::Vector3d progress_lift = read_full_tip();
                  progress_lift.z() = std::max(
                      progress_lift.z(),
                      use_progress_neutral_anchor
                          ? CapsuleObjectClearanceHeight(params)
                          : CapsuleOrientationClearanceHeight(params));
                  Eigen::Vector3d progress_anchor = acquisition_home_tip;
                  progress_anchor.z() = progress_lift.z();
                  Eigen::Vector3d progress_overhead = progress_high;
                  progress_overhead.z() = progress_lift.z();
                  const bool progress_lifted = execute_posture_waypoint(
                      progress_lift, progress_start_pose,
                      active_release_sample,
                      "progress_lift", true, false, false, false, true,
                      true);
                  const bool progress_anchored = progress_lifted &&
                      (!use_progress_neutral_anchor ||
                       execute_posture_waypoint(
                           progress_anchor, progress_start_pose,
                           active_release_sample,
                           "progress_neutral_anchor", true, false, false,
                           false, false, true));
                  const Eigen::Vector3d progress_verticalization_point =
                      use_progress_neutral_anchor ? progress_anchor :
                          progress_lift;
                  const bool progress_verticalized = progress_anchored &&
                      execute_posture_waypoint(
                          progress_verticalization_point,
                          progress_start_pose, progress_sample,
                          "progress_verticalize", false, false, false,
                          false, false, false);
                  const bool progress_traversed = progress_verticalized &&
                      execute_posture_waypoint(
                          use_progress_neutral_anchor ? progress_overhead :
                              progress_high,
                          progress_start_pose, progress_sample,
                          "progress_traverse", false, false, false, false,
                          false, true);
                  const bool progress_lowered = progress_traversed &&
                      (!use_progress_neutral_anchor ||
                       execute_posture_waypoint(
                           progress_high, progress_start_pose,
                           progress_sample, "progress_lower", false,
                           false, false, false, false, true));
                  const bool progress_descended = progress_lowered &&
                      execute_posture_waypoint(
                          progress_standoff, progress_start_pose,
                          progress_sample, "progress_descend", false, false,
                          false, false, false, true);
                  const bool progress_contact_reached = progress_descended &&
                      execute_posture_waypoint(
                          progress_contact, progress_start_pose,
                          progress_sample, "progress_contact", false, false,
                          false, true, false, true);
                  const int completed_acquisition_phases =
                      static_cast<int>(progress_lifted) +
                      static_cast<int>(progress_anchored) +
                      static_cast<int>(progress_verticalized) +
                      static_cast<int>(progress_traversed) +
                      static_cast<int>(progress_lowered) +
                      static_cast<int>(progress_descended) +
                      static_cast<int>(progress_contact_reached);
                  if (progress_contact_reached) {
                    const auto conformance =
                        EvaluateFullSamplingC3AcquisitionConformance(
                            true, 7, completed_acquisition_phases, false,
                            false, true);
                    std::cout <<
                        "full_sampling_c3plus_acquisition_conformance=PASS"
                              << " sample=" << progress_plan.sample_name
                              << " completed_phases="
                              << conformance.completed_phases
                              << " expected_phases="
                              << conformance.expected_phases
                              << " physical_acquisition_completed="
                              << conformance.physical_acquisition_completed
                              << " recovery_required="
                              << conformance.recovery_required
                              << " replanning_allowed="
                              << conformance.replanning_allowed
                              << " updates=" << full_execution_updates
                              << std::endl;
                  }
                  if (!progress_contact_reached &&
                      full_execution_updates < FLAGS_full_execution_steps) {
                    const std::size_t terminal_receipts_before_recovery =
                        measured_response_history.size();
                    // Before descent, the pusher is already above and clear
                    // of the rejected candidate. Applying that candidate's
                    // face normal as a release command would create a large
                    // unrelated traverse. Only a failed descent/contact
                    // approach owns a physical face-release transaction.
                    bool progress_fallback_released = !progress_descended;
                    while (!progress_fallback_released &&
                           full_execution_updates <
                               FLAGS_full_execution_steps) {
                      const Eigen::Vector3d release_pose =
                          read_full_object_pose();
                      const Eigen::Rotation2Dd release_R_WO(
                          release_pose.z());
                      const Eigen::Vector2d release_normal_W =
                          release_R_WO * progress_sample.normal;
                      const Eigen::Vector2d release_boundary_W =
                          release_pose.head<2>() +
                          release_R_WO * progress_sample.point;
                      const Eigen::Vector3d release_tip = read_full_tip();
                      const double release_gap = release_normal_W.dot(
                          release_tip.head<2>() - release_boundary_W) -
                          params.controller.pusher_radius;
                      if (release_gap >=
                          params.controller.approach_clearance) {
                        progress_fallback_released = true;
                        break;
                      }
                      Eigen::Vector3d release_step = release_tip;
                      release_step.head<2>() += release_normal_W * std::min(
                          params.controller.task_space_plan_step_limit,
                          params.controller.approach_clearance - release_gap +
                              params.controller
                                  .contact_activation_tolerance);
                      const int updates_before_release =
                          full_execution_updates;
                      if (!execute_posture_waypoint(
                              release_step, release_pose, progress_sample,
                              "progress_fallback_release", true, false) ||
                          full_execution_updates == updates_before_release) {
                        break;
                      }
                    }
                    live_execution_rejections.insert(
                        progress_plan.sample_name);
                    bool neutral_anchor_reacquired = false;
                    bool reachable_anchor_fallback = false;
                    bool overhead_anchor_fallback = false;
                    bool clearance_height_fallback = false;
                    if (progress_fallback_released &&
                        full_execution_updates <
                            FLAGS_full_execution_steps) {
                      Eigen::Vector3d recovery_lift = read_full_tip();
                      recovery_lift.z() = std::max(
                          recovery_lift.z(),
                          CapsuleObjectClearanceHeight(params));
                      const bool recovery_lifted =
                          execute_posture_waypoint(
                              recovery_lift, read_full_object_pose(),
                              progress_sample,
                              "progress_preview_recovery_lift", true,
                              false, false, false, true, true);
                      Eigen::Vector3d recovery_anchor =
                          acquisition_home_tip;
                      recovery_anchor.z() = recovery_lift.z();
                      neutral_anchor_reacquired = recovery_lifted &&
                          execute_posture_waypoint(
                              recovery_anchor, read_full_object_pose(),
                              progress_sample,
                              "progress_preview_recovery_verticalize_anchor",
                              false, false, false, false, false, false);
                      if (recovery_lifted && !neutral_anchor_reacquired) {
                        // A large object-yaw transition can leave the fixed
                        // home-tip anchor outside the current joint-limit
                        // component. The measured lift endpoint is already
                        // capsule-clear; verticalize there before allowing a
                        // retry, using the identical posture/capsule gate.
                        const Eigen::Vector3d reachable_anchor =
                            read_full_tip();
                        reachable_anchor_fallback =
                            execute_posture_waypoint(
                                reachable_anchor,
                                read_full_object_pose(), progress_sample,
                                "progress_preview_recovery_reachable_"
                                "verticalize_anchor",
                                false, false, false, false, false, false);
                        neutral_anchor_reacquired =
                            reachable_anchor_fallback;
                      }
                      if (recovery_lifted &&
                          !neutral_anchor_reacquired) {
                        // At an outer joint limit, in-place
                        // verticalization can be infeasible even though the
                        // selected contact's forward overhead posture was
                        // live-IK feasible. Reuse that measured-height
                        // overhead point; the home-seeded posture solver
                        // validates the complete joint/capsule interpolation.
                        Eigen::Vector3d overhead_anchor = progress_high;
                        overhead_anchor.z() = recovery_lift.z();
                        overhead_anchor_fallback =
                            execute_posture_waypoint(
                                overhead_anchor,
                                read_full_object_pose(), progress_sample,
                                "progress_preview_recovery_overhead_"
                                "verticalize_anchor",
                                false, false, false, false, false, false);
                        neutral_anchor_reacquired =
                            overhead_anchor_fallback;
                      }
                      if (recovery_lifted &&
                          !neutral_anchor_reacquired) {
                        // The measured lift may be much higher than required
                        // after a failed long traverse, leaving a joint at its
                        // limit and making every vertical-axis IK seed
                        // infeasible. First descend position-only, outside the
                        // object footprint, to the exact capsule/object
                        // clearance height. Both the descent and subsequent
                        // in-place verticalization retain swept capsule checks.
                        Eigen::Vector3d clearance_anchor = read_full_tip();
                        clearance_anchor.z() =
                            CapsuleObjectClearanceHeight(params);
                        const bool clearance_height_reached =
                            execute_posture_waypoint(
                                clearance_anchor,
                                read_full_object_pose(), progress_sample,
                                "progress_preview_recovery_clearance_descent",
                                true, false, false, false, false, false, true);
                        clearance_height_fallback =
                            clearance_height_reached &&
                            execute_posture_waypoint(
                                read_full_tip(), read_full_object_pose(),
                                progress_sample,
                                "progress_preview_recovery_clearance_"
                                "verticalize_anchor",
                                false, false, false, false, false, false);
                        neutral_anchor_reacquired =
                            clearance_height_fallback;
                      }
                    }
                    const bool terminal_receipt_preserved =
                        measured_response_history.size() ==
                        terminal_receipts_before_recovery;
                    const auto conformance =
                        EvaluateFullSamplingC3AcquisitionConformance(
                            true, 7, completed_acquisition_phases, true,
                            neutral_anchor_reacquired,
                            terminal_receipt_preserved);
                    std::cout <<
                        "full_sampling_c3plus_progress_execution_fallback="
                              << (conformance.replanning_allowed ? "RETRY" :
                                  "EXHAUSTED")
                              << " sample="
                              << progress_plan.sample_name
                              << " rejected_candidates="
                              << live_execution_rejections.size()
                              << " updates=" << full_execution_updates
                              << std::endl;
                    std::cout <<
                        "full_sampling_c3plus_acquisition_conformance="
                              << (conformance.replanning_allowed ? "PASS" :
                                  "FAIL")
                              << " sample=" << progress_plan.sample_name
                              << " completed_phases="
                              << conformance.completed_phases
                              << " expected_phases="
                              << conformance.expected_phases
                              << " candidate_invalidated="
                              << conformance.candidate_invalidated
                              << " neutral_anchor_reacquired="
                              << conformance.neutral_anchor_reacquired
                              << " reachable_anchor_fallback="
                              << reachable_anchor_fallback
                              << " overhead_anchor_fallback="
                              << overhead_anchor_fallback
                              << " clearance_height_fallback="
                              << clearance_height_fallback
                              << " terminal_receipt_preserved="
                              << conformance.terminal_receipt_preserved
                              << " replanning_allowed="
                              << conformance.replanning_allowed
                              << " updates=" << full_execution_updates
                              << std::endl;
                    if (conformance.replanning_allowed) {
                      corrective_lateral_recovery = true;
                      continue;
                    }
                    corrective_lateral_recovery = false;
                    break;
                  }
                  bool progress_lateral_rejected = false;
                  bool progress_contact_productive = false;
                  bool progress_terminal_regression = false;
                  int progress_contact_dwell = 0;
                  while (progress_contact_reached &&
                         progress_contact_dwell <
                             params.controller.successor_progress_window_steps &&
                         full_execution_updates <
                             FLAGS_full_execution_steps) {
                    const int64_t next_utime =
                        full_state_subscriber.message().utime +
                        1000LL * FLAGS_full_execution_period_ms;
                    while (full_state_subscriber.message().utime <
                           next_utime) {
                      full_lcm.HandleSubscriptions(100);
                      while (full_lcm.HandleSubscriptions(0) > 0) {
                      }
                    }
                    ++full_execution_updates;
                    ++progress_contact_dwell;
                    const Eigen::Vector3d dwell_pose =
                        read_full_object_pose();
                    progress_lateral_rejected =
                        std::abs(dwell_pose.x() -
                                 params.object.goal_pose.x()) >
                        params.controller.lateral_drift_tolerance;
                    const auto dwell_terminal_descent =
                        EvaluateFullSamplingC3TerminalDescent(
                            progress_start_pose, dwell_pose,
                            params.object.goal_pose,
                            params.controller
                                .successor_minimum_translation_progress,
                            params.controller
                                .successor_minimum_yaw_progress);
                    progress_contact_productive =
                        progress_contact_dwell >=
                            params.controller.successor_minimum_contact_steps &&
                        dwell_terminal_descent.accepted;
                    progress_terminal_regression =
                        progress_contact_dwell >=
                            params.controller.successor_minimum_contact_steps &&
                        (!dwell_terminal_descent.translation_nonregressive ||
                         !dwell_terminal_descent.orientation_nonregressive);
                    if (progress_lateral_rejected ||
                        progress_contact_productive ||
                        progress_terminal_regression) {
                      break;
                    }
                  }
                  std::cout <<
                      "full_sampling_c3plus_progress_contact_dwell="
                            << (progress_contact_productive ? "PASS" :
                                "FAIL")
                            << " steps=" << progress_contact_dwell
                            << " lateral_rejected="
                            << progress_lateral_rejected
                            << " terminal_regression="
                            << progress_terminal_regression
                            << " updates=" << full_execution_updates
                            << std::endl;
                  auto execute_task_waypoint = [&](const Eigen::Vector3d& goal,
                                                   int knot) {
                    const bool reached = execute_posture_waypoint(
                        goal, read_full_object_pose(), progress_sample,
                        "progress_knot_" + std::to_string(knot), false,
                        false, true);
                    progress_lateral_rejected =
                        std::abs(read_full_object_x() -
                                 params.object.goal_pose.x()) >
                        params.controller.lateral_drift_tolerance;
                    if (progress_lateral_rejected) {
                      std::cout <<
                          "full_sampling_c3plus_progress_lateral="
                          "REJECT knot=" << knot
                                << " updates=" << full_execution_updates
                                << std::endl;
                    }
                    std::cout << "full_sampling_c3plus_progress_knot="
                              << (reached ? "PASS" : "FAIL")
                              << " knot=" << knot
                              << " updates=" << full_execution_updates
                              << std::endl;
                    return reached;
                  };
                  bool progress_knots_reached =
                      progress_contact_productive;
                  bool execute_progress_tail = progress_contact_reached &&
                      !progress_contact_productive &&
                      !progress_terminal_regression &&
                      !progress_lateral_rejected &&
                      full_execution_updates < FLAGS_full_execution_steps;
                  for (int knot = 1;
                       knot < progress_plan.pusher_positions_W.cols() &&
                       execute_progress_tail;
                       ++knot) {
                    execute_progress_tail = execute_task_waypoint(
                        progress_plan.pusher_positions_W.col(knot), knot);
                    progress_knots_reached = execute_progress_tail &&
                        knot + 1 ==
                            progress_plan.pusher_positions_W.cols();
                  }
                  const int release_recovery_start_updates =
                      full_execution_updates;
                  std::optional<
                      XarmFullSamplingC3NormalizedParetoDescentReceipt>
                          selected_cycle_recovery_prediction;
                  std::optional<XarmFullSamplingC3MeasuredResponse>
                      selected_cycle_recovery_observation;
                  if (progress_lateral_rejected) {
                    live_execution_rejections.insert(
                        progress_plan.sample_name);
                    std::cout <<
                        "full_sampling_c3plus_progress_response_"
                        "quarantine=PASS sample="
                              << progress_plan.sample_name
                              << " reason=measured_lateral_rejection"
                              << " rejected_candidates="
                              << live_execution_rejections.size()
                              << " updates=" << full_execution_updates
                              << std::endl;
                  }
                  Eigen::Vector3d progress_end_pose =
                      read_full_object_pose();
                  const auto measured_terminal_descent =
                      EvaluateFullSamplingC3TerminalDescent(
                          progress_start_pose, progress_end_pose,
                          params.object.goal_pose,
                          params.controller
                              .successor_minimum_translation_progress,
                          params.controller.successor_minimum_yaw_progress);
                  const bool productive_progress =
                      measured_terminal_descent.accepted;
                  if (progress_contact_reached &&
                      progress_contact_dwell > 0 &&
                      !progress_execution_buffer.successful.empty()) {
                    const auto selected_predicted_terminal =
                        candidate_terminal_pose(
                            progress_execution_buffer.successful.front());
                    if (selected_predicted_terminal.has_value()) {
                      XarmFullSamplingC3MeasuredResponse observation;
                      observation.start_object_pose = progress_start_pose;
                      observation.predicted_terminal_object_pose =
                          *selected_predicted_terminal;
                      observation.measured_terminal_object_pose =
                          progress_end_pose;
                      observation.sample_point_O =
                          progress_plan.sample_point_O;
                      observation.sample_normal_O =
                          progress_plan.sample_normal_O;
                      observation.lateral_rejected =
                          progress_lateral_rejected;
                      measured_response_history.push_back(observation);
                      std::cout <<
                          "full_sampling_c3plus_measured_response_observation="
                          "PASS sample=" << progress_plan.sample_name
                                << " history_size="
                                << measured_response_history.size()
                                << " predicted_terminal_pose="
                                << selected_predicted_terminal->transpose()
                                << " measured_terminal_pose="
                                << progress_end_pose.transpose()
                                << " prediction_residual="
                                << (progress_end_pose -
                                    *selected_predicted_terminal).transpose()
                                << " lateral_rejected="
                                << progress_lateral_rejected
                                << " terminal_accepted="
                                << measured_terminal_descent.accepted
                                << std::endl;
                    }
                  }
                  bool progress_recovered =
                      std::abs(progress_end_pose.x() -
                               params.object.goal_pose.x()) <=
                      progress_lateral_reserve_limit;
                  const bool terminal_response_observed =
                      progress_contact_reached &&
                      progress_contact_dwell >=
                          params.controller.successor_minimum_contact_steps;
                  const bool terminal_descent_rejected =
                      terminal_response_observed && !productive_progress &&
                      !progress_lateral_rejected;
                  bool terminal_descent_released = false;
                  if (terminal_descent_rejected) {
                    live_execution_rejections.insert(
                        progress_plan.sample_name);
                    const char* terminal_rejection_reason =
                        (!measured_terminal_descent
                              .translation_nonregressive ||
                         !measured_terminal_descent
                              .orientation_nonregressive)
                            ? "measured_terminal_regression"
                            : "insufficient_terminal_progress";
                    std::cout <<
                        "full_sampling_c3plus_progress_response_"
                        "quarantine=PASS sample="
                              << progress_plan.sample_name
                              << " reason=" << terminal_rejection_reason
                              << " translation_progress_m="
                              << measured_terminal_descent
                                     .translation_progress
                              << " orientation_progress_rad="
                              << measured_terminal_descent
                                     .orientation_progress
                              << " rejected_candidates="
                              << live_execution_rejections.size()
                              << " updates=" << full_execution_updates
                              << std::endl;
                  }
                  while (terminal_descent_rejected &&
                         progress_contact_reached && progress_recovered &&
                         !terminal_descent_released &&
                         full_execution_updates <
                             FLAGS_full_execution_steps) {
                    const Eigen::Vector3d release_pose =
                        read_full_object_pose();
                    const Eigen::Rotation2Dd release_R_WO(release_pose.z());
                    const Eigen::Vector2d release_normal_W =
                        release_R_WO * progress_sample.normal;
                    const Eigen::Vector2d release_boundary_W =
                        release_pose.head<2>() +
                        release_R_WO * progress_sample.point;
                    const Eigen::Vector3d release_tip = read_full_tip();
                    const double release_gap = release_normal_W.dot(
                        release_tip.head<2>() - release_boundary_W) -
                        params.controller.pusher_radius;
                    if (release_gap >=
                        params.controller.approach_clearance) {
                      terminal_descent_released = true;
                      break;
                    }
                    Eigen::Vector3d release_step = release_tip;
                    release_step.head<2>() += release_normal_W * std::min(
                        params.controller.task_space_plan_step_limit,
                        params.controller.approach_clearance - release_gap +
                            params.controller
                                .contact_activation_tolerance);
                    const int updates_before_release =
                        full_execution_updates;
                    if (!execute_posture_waypoint(
                            release_step, release_pose, progress_sample,
                            "progress_terminal_rejection_release", true,
                            false) ||
                        full_execution_updates == updates_before_release) {
                      break;
                    }
                  }
                  if (terminal_descent_rejected) {
                    progress_end_pose = read_full_object_pose();
                    progress_recovered = terminal_descent_released &&
                        std::abs(progress_end_pose.x() -
                                 params.object.goal_pose.x()) <=
                            progress_lateral_reserve_limit;
                    std::cout <<
                        "full_sampling_c3plus_terminal_descent_release="
                              << (terminal_descent_released ? "PASS" :
                                  "FAIL")
                              << " recovered=" << progress_recovered
                              << " object_pose="
                              << progress_end_pose.transpose()
                              << " updates=" << full_execution_updates
                              << std::endl;
                  }
                  if ((productive_progress || progress_lateral_rejected ||
                       terminal_descent_rejected) &&
                      !progress_recovered &&
                      full_execution_updates < FLAGS_full_execution_steps) {
                    // Recovery feasibility must be evaluated from a released
                    // posture.  The measured lateral rejection occurs while
                    // the pusher is still on the progress face; replaying a
                    // corrective candidate from that interpenetrating start
                    // would reject every otherwise valid lift at waypoint 0.
                    bool rejected_face_released = false;
                    while (!rejected_face_released &&
                           full_execution_updates <
                               FLAGS_full_execution_steps) {
                      const Eigen::Vector3d release_object_pose =
                          read_full_object_pose();
                      const Eigen::Rotation2Dd release_R_WO(
                          release_object_pose.z());
                      const Eigen::Vector2d release_normal_W =
                          release_R_WO * progress_sample.normal;
                      const Eigen::Vector2d release_boundary_W =
                          release_object_pose.head<2>() +
                          release_R_WO * progress_sample.point;
                      const Eigen::Vector3d release_tip = read_full_tip();
                      const double release_gap = release_normal_W.dot(
                          release_tip.head<2>() - release_boundary_W) -
                          params.controller.pusher_radius;
                      if (release_gap >=
                          params.controller.approach_clearance) {
                        rejected_face_released = true;
                        break;
                      }
                      Eigen::Vector3d release_step = release_tip;
                      release_step.head<2>() += release_normal_W * std::min(
                          params.controller.task_space_plan_step_limit,
                          params.controller.approach_clearance - release_gap +
                              params.controller
                                  .contact_activation_tolerance);
                      const int updates_before_release =
                          full_execution_updates;
                      if (!execute_posture_waypoint(
                              release_step, release_object_pose,
                              progress_sample,
                              "progress_rejection_release", true, false) ||
                          full_execution_updates ==
                              updates_before_release) {
                        break;
                      }
                    }
                    progress_end_pose = read_full_object_pose();
                    progress_recovered = rejected_face_released &&
                        std::abs(progress_end_pose.x() -
                                 params.object.goal_pose.x()) <=
                            progress_lateral_reserve_limit;
                    std::cout <<
                        "full_sampling_c3plus_progress_rejection_release="
                              << (rejected_face_released ? "PASS" : "FAIL")
                              << " recovered=" << progress_recovered
                              << " object_pose="
                              << progress_end_pose.transpose()
                              << " updates=" << full_execution_updates
                              << std::endl;
                    // A productive tail may be stopped by the unchanged
                    // lateral corridor.  Condition a new 72-sample solve on
                    // that measured rejection before accepting the cycle.
                    OimTParams cycle_recovery_params = params;
                    cycle_recovery_params.object.start_pose =
                        progress_end_pose;
                    const auto cycle_recovery_velocity =
                        read_full_object_velocity();
                    const auto cycle_recovery_planar_velocity =
                        ConditionOpenTableObjectVelocity(
                            cycle_recovery_velocity.first,
                            cycle_recovery_velocity.second);
                    cycle_recovery_params.object.start_angular_velocity_W =
                        cycle_recovery_planar_velocity.first;
                    cycle_recovery_params.object.start_linear_velocity_W =
                        cycle_recovery_planar_velocity.second;
                    // The five-knot recovery horizon cannot represent the
                    // distant terminal task directly. Use the same unchanged-
                    // tolerance receding subgoal as primary planning while
                    // preserving the global x correction. Prediction and
                    // physical acceptance below still use the global goal.
                    cycle_recovery_params.object.goal_pose =
                        progress_end_pose;
                    cycle_recovery_params.object.goal_pose.x() =
                        params.object.goal_pose.x();
                    cycle_recovery_params.object.goal_pose.y() += std::clamp(
                        params.object.goal_pose.y() - progress_end_pose.y(),
                        -params.task.translation_tolerance,
                        params.task.translation_tolerance);
                    cycle_recovery_params.object.goal_pose.z() += std::clamp(
                        WrappedAngleError(params.object.goal_pose.z(),
                                          progress_end_pose.z()),
                        -params.task.orientation_tolerance,
                        params.task.orientation_tolerance);
                    const auto cycle_exact =
                        RunXarmFullSamplingC3ExactTBatch(
                            cycle_recovery_params);
                    const auto cycle_perimeter =
                        RunXarmFullSamplingC3PerimeterBatchParallel(
                            cycle_recovery_params);
                    const auto cycle_mesh =
                        RunXarmFullSamplingC3MeshBatchParallel(
                            cycle_recovery_params);
                    const auto cycle_left =
                        RunXarmFullSamplingC3StemLeftRefinementBatchParallel(
                            cycle_recovery_params);
                    const auto cycle_right =
                        RunXarmFullSamplingC3StemRightRefinementBatchParallel(
                            cycle_recovery_params);
                    const auto cycle_buffer =
                        BuildXarmFullSamplingC3CandidateBuffer(
                            {cycle_exact, cycle_perimeter, cycle_mesh,
                             cycle_left, cycle_right});
                    const double cycle_x_direction =
                        params.object.goal_pose.x() - progress_end_pose.x();
                    const XarmFullSamplingC3CandidateReceipt*
                        cycle_candidate = nullptr;
                    Eigen::Vector3d cycle_live_pose = progress_end_pose;
                    std::set<std::string> cycle_execution_rejections;
                    auto has_cycle_polarity = [&](const auto& candidate) {
                      const Eigen::Vector2d normal_W =
                          Eigen::Rotation2Dd(progress_end_pose.z()) *
                          candidate.sample_normal_O;
                      return cycle_x_direction * (-normal_W.x()) > 0.0 &&
                          std::abs(normal_W.x()) > 0.5;
                    };
                    auto condition_cycle_candidate =
                        [&](const auto& candidate) {
                          XarmFullSamplingC3ResponseConditioningReceipt
                              receipt;
                          const auto entry =
                              FindXarmFullSamplingC3LateralCorridorEntry(
                                  candidate, params.object.goal_pose.x(),
                                  params.controller
                                      .lateral_drift_tolerance);
                          if (!entry.accepted) {
                            receipt.ranking_class = 2;
                            return receipt;
                          }
                          return
                              EvaluateFullSamplingC3EquivariantResponseConditioning(
                                  progress_end_pose, entry.object_pose,
                                  candidate.sample_point_O,
                                  candidate.sample_normal_O,
                                  params.object.goal_pose,
                                  recovery_response_history,
                                  2.0 * params.controller.pusher_radius,
                                  params.task.orientation_tolerance,
                                  params.controller.lateral_drift_tolerance,
                                  params.controller
                                      .successor_minimum_translation_progress,
                                  params.controller
                                      .successor_minimum_yaw_progress);
                        };
                    auto has_cycle_task_nonregression =
                        [&](const auto& candidate) {
                          const auto entry =
                              FindXarmFullSamplingC3LateralCorridorEntry(
                                  candidate, params.object.goal_pose.x(),
                                  params.controller
                                      .lateral_drift_tolerance);
                          if (!entry.accepted) return false;
                          const auto conditioning =
                              condition_cycle_candidate(candidate);
                          const Eigen::Vector3d& ranked_terminal =
                              conditioning.ranking_class == 0
                                  ? conditioning
                                        .corrected_terminal_object_pose
                                  : entry.object_pose;
                          return EvaluateXarmFullSamplingC3LateralRecovery(
                              progress_start_pose, progress_end_pose,
                              ranked_terminal, params.object.goal_pose,
                              params.task.translation_tolerance,
                              params.task.orientation_tolerance,
                              params.controller.lateral_drift_tolerance)
                              .accepted;
                        };
                    auto cycle_task_descent_magnitude =
                        [&](const auto& candidate) {
                          const auto entry =
                              FindXarmFullSamplingC3LateralCorridorEntry(
                                  candidate, params.object.goal_pose.x(),
                                  params.controller
                                      .lateral_drift_tolerance);
                          if (!entry.accepted) {
                            return -std::numeric_limits<double>::infinity();
                          }
                          const auto descent =
                              EvaluateXarmFullSamplingC3LateralRecovery(
                                  progress_start_pose, progress_end_pose,
                                  condition_cycle_candidate(candidate)
                                              .ranking_class == 0
                                      ? condition_cycle_candidate(candidate)
                                            .corrected_terminal_object_pose
                                      : entry.object_pose,
                                  params.object.goal_pose,
                                  params.task.translation_tolerance,
                                  params.task.orientation_tolerance,
                                  params.controller.lateral_drift_tolerance);
                          return descent.accepted
                              ? descent.normalized_magnitude
                              : -std::numeric_limits<double>::infinity();
                        };
                    auto select_live_cycle_candidate =
                        [&](const std::vector<
                                const XarmFullSamplingC3CandidateReceipt*>&
                                ranked,
                            const Eigen::VectorXd& candidate_start_q,
                            const ContactSample& release_sample) {
                          for (const auto* candidate : ranked) {
                            if (cycle_execution_rejections.contains(
                                    candidate->sample_name)) {
                              continue;
                            }
                            if (!IsFullSamplingC3CentralSideContact(
                                    candidate->sample_height_O,
                                    params.object.resting_height,
                                    params.controller.pusher_radius)) {
                              continue;
                            }
                            if (!has_cycle_polarity(*candidate)) continue;
                            if (!has_cycle_task_nonregression(*candidate)) {
                              const auto entry =
                                  FindXarmFullSamplingC3LateralCorridorEntry(
                                      *candidate,
                                      params.object.goal_pose.x(),
                                      params.controller
                                          .lateral_drift_tolerance);
                              XarmFullSamplingC3LateralRecoveryReceipt
                                  recovery_receipt;
                              if (entry.accepted) {
                                recovery_receipt =
                                    EvaluateXarmFullSamplingC3LateralRecovery(
                                        progress_start_pose,
                                        progress_end_pose,
                                        entry.object_pose,
                                        params.object.goal_pose,
                                        params.task.translation_tolerance,
                                        params.task.orientation_tolerance,
                                        params.controller
                                            .lateral_drift_tolerance);
                              }
                              std::cout <<
                                  "full_sampling_c3plus_cycle_recovery_"
                                  "task_nonregression=REJECT sample="
                                        << candidate->sample_name
                                        << " corridor_entry_knot="
                                        << entry.state_knot
                                        << " lateral_error_m="
                                        << recovery_receipt.lateral_error
                                        << " lateral_reduction_m="
                                        << recovery_receipt.lateral_reduction
                                        << " translation_progress_m="
                                        << recovery_receipt
                                               .translation_progress
                                        << " translation_debt_bounded="
                                        << recovery_receipt
                                               .translation_debt_bounded
                                        << " orientation_debt_rad="
                                        << recovery_receipt.orientation_debt
                                        << std::endl;
                              continue;
                            }
                            const auto response_conditioning =
                                condition_cycle_candidate(*candidate);
                            if (response_conditioning.ranking_class == 2) {
                              std::cout <<
                                  "full_sampling_c3plus_cycle_recovery_"
                                  "response_quarantine=REJECT sample="
                                        << candidate->sample_name
                                        << " response_observations="
                                        << response_conditioning
                                               .matching_observations
                                        << " corrected_terminal_accepted="
                                        << response_conditioning
                                               .corrected_terminal_accepted
                                        << " corrected_lateral_accepted="
                                        << response_conditioning
                                               .corrected_lateral_accepted
                                        << " before_live_ik=1"
                                        << std::endl;
                              continue;
                            }
                            const auto live_receipt =
                                EvaluateMeasuredCandidateAcquisition(
                                    acquisition_plant,
                                    acquisition_context.get(), params,
                                    candidate_start_q, cycle_live_pose,
                                    *candidate, release_sample, true);
                            const bool live_accepted =
                                live_receipt.swept_capsule_clear &&
                                live_receipt.ik_reached;
                            std::cout <<
                                "full_sampling_c3plus_cycle_recovery_live_ik="
                                      << (live_accepted ? "PASS" : "FAIL")
                                      << " sample="
                                      << candidate->sample_name
                                      << " ik_steps="
                                      << live_receipt.ik_steps
                                      << " failed_waypoint="
                                      << live_receipt.failed_waypoint
                                      << " swept_capsule_clear="
                                      << live_receipt.swept_capsule_clear
                                      << " controlled_escape="
                                      << live_receipt
                                             .used_controlled_escape
                                      << " failure_ik_solved="
                                      << live_receipt.failure_ik_solved
                                      << " failure_shaft_clear="
                                      << live_receipt.failure_clearance
                                             .shaft_t_clear
                                      << " failure_capsule_table_clear="
                                      << live_receipt.failure_clearance
                                             .capsule_table_clear
                                      << " neutral_anchor="
                                      << true
                                      << " home_seed_fallback="
                                      << live_receipt
                                             .used_home_seed_fallback
                                      << " central_side_contact=1"
                                      << " sample_height_O="
                                      << candidate->sample_height_O
                                      << " response_rank_class="
                                      << response_conditioning.ranking_class
                                      << " response_observations="
                                      << response_conditioning
                                             .matching_observations
                                      << " corrected_terminal_accepted="
                                      << response_conditioning
                                             .corrected_terminal_accepted
                                      << std::endl;
                            if (live_accepted) return candidate;
                          }
                          return static_cast<const
                              XarmFullSamplingC3CandidateReceipt*>(nullptr);
                        };
                    std::vector<const
                        XarmFullSamplingC3CandidateReceipt*>
                            ranked_cycle_candidates;
                    for (const auto& candidate : cycle_buffer.successful) {
                      ranked_cycle_candidates.push_back(&candidate);
                    }
                    std::stable_sort(
                        ranked_cycle_candidates.begin(),
                        ranked_cycle_candidates.end(),
                        [&](const auto* a, const auto* b) {
                          const int a_rank =
                              condition_cycle_candidate(*a).ranking_class;
                          const int b_rank =
                              condition_cycle_candidate(*b).ranking_class;
                          if (a_rank != b_rank) return a_rank < b_rank;
                          return cycle_task_descent_magnitude(*a) >
                              cycle_task_descent_magnitude(*b);
                        });
                    std::vector<const
                        XarmFullSamplingC3CandidateReceipt*>
                            ranked_cycle_contact_candidates;
                    const std::array<
                        const XarmFullSamplingC3BatchReceipt*, 5>
                        cycle_batches = {&cycle_exact, &cycle_perimeter,
                                         &cycle_mesh, &cycle_left,
                                         &cycle_right};
                    for (const auto* batch : cycle_batches) {
                      for (const auto& candidate : batch->candidates) {
                        if (!candidate.solve.dynamic_rollout_accepted ||
                            candidate.solve.planned_input_bound_violation >
                                1.0e-12 ||
                            !std::isfinite(
                                candidate.solve.dynamic_rollout_cost)) {
                          continue;
                        }
                        ranked_cycle_contact_candidates.push_back(&candidate);
                      }
                    }
                    std::stable_sort(
                        ranked_cycle_contact_candidates.begin(),
                        ranked_cycle_contact_candidates.end(),
                        [&](const auto* a, const auto* b) {
                          const int a_rank =
                              condition_cycle_candidate(*a).ranking_class;
                          const int b_rank =
                              condition_cycle_candidate(*b).ranking_class;
                          if (a_rank != b_rank) return a_rank < b_rank;
                          const double a_descent =
                              cycle_task_descent_magnitude(*a);
                          const double b_descent =
                              cycle_task_descent_magnitude(*b);
                          if (a_descent != b_descent) {
                            return a_descent > b_descent;
                          }
                          return a->solve.dynamic_rollout_cost <
                              b->solve.dynamic_rollout_cost;
                        });
                    bool cycle_contact_only_fallback = false;
                    ContactSample cycle_release_sample = progress_sample;
                    std::string cycle_release_name = progress_sample.name;
                    auto select_next_cycle_candidate = [&]() {
                      read_full_tip();
                      const Eigen::VectorXd candidate_start_q =
                          acquisition_plant.GetPositions(
                              *acquisition_context);
                      cycle_contact_only_fallback = false;
                      const auto* candidate = select_live_cycle_candidate(
                          ranked_cycle_candidates, candidate_start_q,
                          cycle_release_sample);
                      if (candidate != nullptr) return candidate;
                      candidate = select_live_cycle_candidate(
                          ranked_cycle_contact_candidates,
                          candidate_start_q, cycle_release_sample);
                      cycle_contact_only_fallback = candidate != nullptr;
                      return candidate;
                    };
                    if (!progress_recovered) {
                      cycle_candidate = select_next_cycle_candidate();
                    }
                    bool cycle_seed_replenishment = false;
                    std::optional<XarmFullSamplingC3CandidateBuffer>
                        cycle_replenished_buffer;
                    if (cycle_candidate == nullptr &&
                        !progress_recovered) {
                      std::vector<XarmFullSamplingC3BatchReceipt>
                          retry_batches;
                      retry_batches.reserve(8);
                      for (int retry = 1; retry <= 4; ++retry) {
                        OimTParams retry_params = cycle_recovery_params;
                        retry_params.full_sampling_c3plus.random_seed +=
                            retry;
                        retry_params.full_sampling_c3plus.mesh_random_seed +=
                            retry;
                        auto perimeter_retry =
                            RunXarmFullSamplingC3PerimeterBatchParallel(
                                retry_params);
                        auto mesh_retry =
                            RunXarmFullSamplingC3MeshBatchParallel(
                                retry_params);
                        const std::string prefix =
                            "recovery_retry_" + std::to_string(retry) + "_";
                        for (auto& candidate :
                             perimeter_retry.candidates) {
                          candidate.sample_name =
                              prefix + candidate.sample_name;
                        }
                        for (auto& candidate : mesh_retry.candidates) {
                          candidate.sample_name =
                              prefix + candidate.sample_name;
                        }
                        retry_batches.push_back(
                            std::move(perimeter_retry));
                        retry_batches.push_back(std::move(mesh_retry));
                      }
                      std::vector<XarmFullSamplingC3BatchReceipt>
                          retry_batch_copies = retry_batches;
                      cycle_replenished_buffer =
                          BuildXarmFullSamplingC3CandidateBuffer(
                              retry_batch_copies);
                      if (cycle_replenished_buffer->successful.empty()) {
                        *cycle_replenished_buffer =
                            BuildXarmFullSamplingC3ContactFeasibleCandidateBuffer(
                                *cycle_replenished_buffer);
                      }
                      std::vector<const
                          XarmFullSamplingC3CandidateReceipt*>
                              ranked_replenished;
                      for (const auto& candidate :
                           cycle_replenished_buffer->successful) {
                        ranked_replenished.push_back(&candidate);
                      }
                      std::stable_sort(
                          ranked_replenished.begin(),
                          ranked_replenished.end(),
                          [&](const auto* a, const auto* b) {
                            const int a_rank =
                                condition_cycle_candidate(*a).ranking_class;
                            const int b_rank =
                                condition_cycle_candidate(*b).ranking_class;
                            if (a_rank != b_rank) return a_rank < b_rank;
                            return cycle_task_descent_magnitude(*a) >
                                cycle_task_descent_magnitude(*b);
                          });
                      read_full_tip();
                      const Eigen::VectorXd replenished_start_q =
                          acquisition_plant.GetPositions(
                              *acquisition_context);
                      cycle_candidate = select_live_cycle_candidate(
                          ranked_replenished, replenished_start_q,
                          cycle_release_sample);
                      cycle_seed_replenishment =
                          cycle_candidate != nullptr;
                      std::cout <<
                          "full_sampling_c3plus_cycle_recovery_seed_"
                          "replenishment="
                                << (cycle_seed_replenishment ? "PASS" :
                                    "FAIL")
                                << " retries=4 candidates="
                                << cycle_replenished_buffer->total_candidates
                                << " executable="
                                << cycle_replenished_buffer
                                       ->successful.size()
                                << " response_quarantine=1"
                                << " live_ik_gate=1 capsule_gate=1"
                                << std::endl;
                    }
                    std::cout <<
                        "full_sampling_c3plus_cycle_recovery_resample="
                              << (cycle_candidate != nullptr ? "PASS" :
                                  "FAIL")
                              << " candidates="
                              << cycle_buffer.total_candidates
                              << " executable="
                              << cycle_buffer.successful.size()
                              << " contact_only_fallback="
                              << cycle_contact_only_fallback
                              << " seed_replenishment="
                              << cycle_seed_replenishment
                              << " response_contact_neighborhood_m="
                              << 2.0 * params.controller.pusher_radius
                              << " rejected_pose="
                              << progress_end_pose.transpose()
                              << " object_angular_velocity_W="
                              << cycle_recovery_velocity.first.transpose()
                              << " object_linear_velocity_W="
                              << cycle_recovery_velocity.second.transpose()
                              << " conditioned_angular_velocity_W="
                              << cycle_recovery_params.object
                                     .start_angular_velocity_W.transpose()
                              << " conditioned_linear_velocity_W="
                              << cycle_recovery_params.object
                                     .start_linear_velocity_W.transpose()
                              << std::endl;
                    while (cycle_candidate != nullptr &&
                           !progress_recovered &&
                           full_execution_updates <
                               FLAGS_full_execution_steps) {
                      ContactFace cycle_face = ContactFace::kCrossbarTop;
                      if (cycle_candidate->sample_name.find("stem_right") !=
                          std::string::npos) {
                        cycle_face = ContactFace::kStemRight;
                      } else if (cycle_candidate->sample_name.find(
                                     "stem_left") != std::string::npos) {
                        cycle_face = ContactFace::kStemLeft;
                      } else if (cycle_candidate->sample_name.find(
                                     "crossbar_right") !=
                                 std::string::npos) {
                        cycle_face = ContactFace::kCrossbarRight;
                      } else if (cycle_candidate->sample_name.find(
                                     "crossbar_left") !=
                                 std::string::npos) {
                        cycle_face = ContactFace::kCrossbarLeft;
                      }
                      const ContactSample cycle_sample{
                          cycle_candidate->sample_point_O,
                          cycle_candidate->sample_normal_O,
                          cycle_candidate->sample_name.c_str(), cycle_face};
                      const Eigen::Vector3d cycle_attempt_pose =
                          read_full_object_pose();
                      cycle_live_pose = cycle_attempt_pose;
                      const Eigen::Rotation2Dd cycle_R_WO(
                          cycle_attempt_pose.z());
                      const Eigen::Vector2d cycle_normal_W =
                          cycle_R_WO * cycle_candidate->sample_normal_O;
                      // The rejected progress face was released before this
                      // recovery solve. Start the recovery lift from that
                      // measured clear posture instead of applying a second
                      // stale-face release displacement.
                      const bool use_cycle_neutral_anchor =
                          true;
                      Eigen::Vector3d cycle_lift = read_full_tip();
                      cycle_lift.z() = std::max(
                          cycle_lift.z(),
                          use_cycle_neutral_anchor
                              ? CapsuleObjectClearanceHeight(params)
                              : CapsuleOrientationClearanceHeight(params));
                      Eigen::Vector3d cycle_contact;
                      cycle_contact.head<2>() =
                          cycle_attempt_pose.head<2>() +
                          cycle_R_WO * cycle_candidate->sample_point_O +
                          cycle_normal_W *
                              (params.controller.pusher_radius -
                               0.5 * params.controller
                                         .contact_activation_tolerance);
                      cycle_contact.z() = params.object.resting_height +
                          cycle_candidate->sample_height_O;
                      Eigen::Vector3d cycle_standoff = cycle_contact;
                      cycle_standoff.head<2>() += cycle_normal_W *
                          (params.controller.descent_clearance +
                           0.5 * params.controller
                                     .contact_activation_tolerance);
                      Eigen::Vector3d cycle_high = cycle_standoff;
                      cycle_high.z() =
                          params.controller.reposition_waypoint_height;
                      Eigen::Vector3d cycle_anchor = acquisition_home_tip;
                      cycle_anchor.z() = cycle_lift.z();
                      Eigen::Vector3d cycle_overhead = cycle_high;
                      cycle_overhead.z() = cycle_lift.z();
                      const bool cycle_released = true;
                      const bool cycle_lifted = cycle_released &&
                          execute_posture_waypoint(
                              cycle_lift, cycle_attempt_pose,
                              cycle_release_sample,
                              "cycle_recovery_lift", true, false, false,
                              false, true);
                      bool cycle_anchored = cycle_lifted &&
                          (!use_cycle_neutral_anchor ||
                           execute_posture_waypoint(
                               cycle_anchor, cycle_attempt_pose,
                               cycle_release_sample,
                               "cycle_recovery_neutral_anchor", true,
                               false));
                      bool cycle_reachable_anchor_fallback = false;
                      bool cycle_overhead_anchor_fallback = false;
                      bool cycle_clearance_height_fallback = false;
                      bool cycle_verticalized = false;
                      if (cycle_anchored) {
                        const Eigen::Vector3d cycle_verticalization_point =
                            use_cycle_neutral_anchor ? cycle_anchor :
                                                       cycle_lift;
                        cycle_verticalized = execute_posture_waypoint(
                            cycle_verticalization_point, cycle_attempt_pose,
                            cycle_sample,
                            "cycle_recovery_verticalize", false, false);
                      }
                      if (cycle_lifted && !cycle_verticalized) {
                        const Eigen::Vector3d reachable_anchor =
                            read_full_tip();
                        cycle_reachable_anchor_fallback =
                            execute_posture_waypoint(
                                reachable_anchor, read_full_object_pose(),
                                cycle_sample,
                                "cycle_recovery_reachable_verticalize", false,
                                false);
                        cycle_verticalized =
                            cycle_reachable_anchor_fallback;
                      }
                      if (cycle_lifted && !cycle_verticalized) {
                        Eigen::Vector3d overhead_anchor = cycle_overhead;
                        overhead_anchor.z() = read_full_tip().z();
                        cycle_overhead_anchor_fallback =
                            execute_posture_waypoint(
                                overhead_anchor, read_full_object_pose(),
                                cycle_sample,
                                "cycle_recovery_overhead_verticalize", false,
                                false);
                        cycle_verticalized =
                            cycle_overhead_anchor_fallback;
                      }
                      if (cycle_lifted && !cycle_verticalized) {
                        Eigen::Vector3d clearance_anchor = read_full_tip();
                        clearance_anchor.z() =
                            CapsuleObjectClearanceHeight(params);
                        const bool clearance_height_reached =
                            execute_posture_waypoint(
                                clearance_anchor, read_full_object_pose(),
                                cycle_sample,
                                "cycle_recovery_clearance_descent", true,
                                false, false, false, false, false, true);
                        cycle_clearance_height_fallback =
                            clearance_height_reached &&
                            execute_posture_waypoint(
                                read_full_tip(), read_full_object_pose(),
                                cycle_sample,
                                "cycle_recovery_clearance_verticalize", false,
                                false);
                        cycle_verticalized =
                            cycle_clearance_height_fallback;
                      }
                      cycle_anchored = cycle_anchored ||
                          cycle_reachable_anchor_fallback ||
                          cycle_overhead_anchor_fallback ||
                          cycle_clearance_height_fallback;
                      std::cout <<
                          "full_sampling_c3plus_cycle_recovery_anchor="
                                << (cycle_verticalized ? "PASS" : "FAIL")
                                << " fixed="
                                << (cycle_anchored &&
                                    !cycle_reachable_anchor_fallback &&
                                    !cycle_overhead_anchor_fallback &&
                                    !cycle_clearance_height_fallback)
                                << " reachable="
                                << cycle_reachable_anchor_fallback
                                << " overhead="
                                << cycle_overhead_anchor_fallback
                                << " clearance_height="
                                << cycle_clearance_height_fallback
                                << " updates=" << full_execution_updates
                                << std::endl;
                      const bool cycle_traversed = cycle_verticalized &&
                          execute_posture_waypoint(
                              use_cycle_neutral_anchor ? cycle_overhead :
                                  cycle_high,
                              cycle_attempt_pose, cycle_sample,
                              "cycle_recovery_traverse", false, false);
                      const bool cycle_lowered = cycle_traversed &&
                          (!use_cycle_neutral_anchor ||
                           execute_posture_waypoint(
                               cycle_high, cycle_attempt_pose, cycle_sample,
                               "cycle_recovery_lower", false, false));
                      const bool cycle_descended = cycle_lowered &&
                          execute_posture_waypoint(
                              cycle_standoff, cycle_attempt_pose, cycle_sample,
                              "cycle_recovery_descend", false, false);
                      const bool cycle_contacted = cycle_descended &&
                          execute_posture_waypoint(
                              cycle_contact, cycle_attempt_pose, cycle_sample,
                              "cycle_recovery_contact", false, true, false,
                              true);
                      if (cycle_contacted) {
                        const auto predicted_entry =
                            FindXarmFullSamplingC3LateralCorridorEntry(
                                *cycle_candidate,
                                params.object.goal_pose.x(),
                                params.controller.lateral_drift_tolerance);
                        if (predicted_entry.accepted) {
                          selected_cycle_recovery_prediction =
                              EvaluateFullSamplingC3NormalizedParetoDescent(
                                  progress_start_pose,
                                  predicted_entry.object_pose,
                                  params.object.goal_pose,
                                  params.task.translation_tolerance,
                                  params.task.orientation_tolerance);
                          XarmFullSamplingC3MeasuredResponse observation;
                          observation.start_object_pose = cycle_attempt_pose;
                          observation.predicted_terminal_object_pose =
                              predicted_entry.object_pose;
                          observation.sample_point_O =
                              cycle_candidate->sample_point_O;
                          observation.sample_normal_O =
                              cycle_candidate->sample_normal_O;
                          selected_cycle_recovery_observation = observation;
                          const auto response_conditioning =
                              condition_cycle_candidate(*cycle_candidate);
                          std::cout <<
                              "full_sampling_c3plus_cycle_recovery_"
                              "predicted_descent=PASS sample="
                                    << cycle_candidate->sample_name
                                    << " corridor_entry_knot="
                                    << predicted_entry.state_knot
                                    << " normalized_magnitude="
                                    << selected_cycle_recovery_prediction
                                           ->normalized_magnitude
                                    << " translation_progress_m="
                                    << selected_cycle_recovery_prediction
                                           ->terminal.translation_progress
                                    << " orientation_progress_rad="
                                    << selected_cycle_recovery_prediction
                                           ->terminal.orientation_progress
                                    << " response_rank_class="
                                    << response_conditioning.ranking_class
                                    << " response_observations="
                                    << response_conditioning
                                           .matching_observations
                                    << std::endl;
                        }
                      }
                      bool cycle_contact_loss_retry = false;
                      if (cycle_contacted) {
                        bool cycle_cleared = false;
                        bool cycle_contact_lost_rejected = false;
                        bool cycle_wrong_polarity_rejected = false;
                        int cycle_recovery_response_steps = 0;
                        const double recovery_start_signed_error =
                            cycle_attempt_pose.x() -
                            params.object.goal_pose.x();
                        while (full_execution_updates <
                               FLAGS_full_execution_steps) {
                          const Eigen::Vector3d measured_object =
                              read_full_object_pose();
                          const double measured_signed_error =
                              measured_object.x() -
                              params.object.goal_pose.x();
                          const bool crossed_goal =
                              recovery_start_signed_error *
                                  measured_signed_error <= 0.0;
                          const bool wrong_polarity_response =
                              IsFullSamplingC3WrongPolarityResponse(
                                  recovery_start_signed_error,
                                  measured_signed_error,
                                  cycle_recovery_response_steps,
                                  params.controller
                                      .successor_minimum_contact_steps);
                          const Eigen::Rotation2Dd measured_R_WO(
                              measured_object.z());
                          const Eigen::Vector2d measured_normal_W =
                              measured_R_WO *
                              cycle_candidate->sample_normal_O;
                          const Eigen::Vector2d measured_boundary_W =
                              measured_object.head<2>() +
                              measured_R_WO *
                              cycle_candidate->sample_point_O;
                          const Eigen::Vector3d measured_tip =
                              read_full_tip();
                          const double measured_gap =
                              measured_normal_W.dot(
                                  measured_tip.head<2>() -
                                  measured_boundary_W) -
                              params.controller.pusher_radius;
                          const bool contact_lost =
                              cycle_recovery_response_steps >=
                                  params.controller
                                      .successor_minimum_contact_steps &&
                              measured_gap >=
                                  params.controller
                                      .contact_activation_tolerance;
                          if (std::abs(measured_signed_error) <=
                                  progress_lateral_reserve_limit ||
                              crossed_goal || wrong_polarity_response ||
                              contact_lost) {
                            if (crossed_goal) {
                              std::cout <<
                                  "full_sampling_c3plus_cycle_recovery_"
                                  "crossing=REJECT"
                                        << " start_signed_error_m="
                                        << recovery_start_signed_error
                                        << " measured_signed_error_m="
                                        << measured_signed_error
                                        << " updates="
                                        << full_execution_updates
                                        << std::endl;
                            }
                            if (wrong_polarity_response) {
                              cycle_wrong_polarity_rejected = true;
                              std::cout <<
                                  "full_sampling_c3plus_cycle_recovery_"
                                  "wrong_polarity_response=REJECT"
                                        << " start_signed_error_m="
                                        << recovery_start_signed_error
                                        << " measured_signed_error_m="
                                        << measured_signed_error
                                        << " response_steps="
                                        << cycle_recovery_response_steps
                                        << " updates="
                                        << full_execution_updates
                                        << std::endl;
                            }
                            if (contact_lost) {
                              cycle_contact_lost_rejected = true;
                              std::cout <<
                                  "full_sampling_c3plus_cycle_recovery_"
                                  "contact_loss=REJECT"
                                        << " measured_gap_m="
                                        << measured_gap
                                        << " response_steps="
                                        << cycle_recovery_response_steps
                                        << " updates="
                                        << full_execution_updates
                                        << std::endl;
                            }
                            if (measured_gap >=
                                params.controller.approach_clearance) {
                              cycle_cleared = true;
                              break;
                            }
                            Eigen::Vector3d clear_step = measured_tip;
                            clear_step.head<2>() += measured_normal_W *
                                std::min(
                                    params.controller
                                        .task_space_plan_step_limit,
                                    params.controller.approach_clearance -
                                        measured_gap +
                                        params.controller
                                            .contact_activation_tolerance);
                            const int updates_before_clear =
                                full_execution_updates;
                            if (!execute_posture_waypoint(
                                    clear_step, measured_object, cycle_sample,
                                    "cycle_recovery_clear", true, false) ||
                                full_execution_updates ==
                                    updates_before_clear) {
                              break;
                            }
                            continue;
                          }
                          const int64_t next_utime =
                              full_state_subscriber.message().utime +
                              1000LL * FLAGS_full_execution_period_ms;
                          while (full_state_subscriber.message().utime <
                                 next_utime) {
                            full_lcm.HandleSubscriptions(100);
                            while (full_lcm.HandleSubscriptions(0) > 0) {
                            }
                          }
                          ++full_execution_updates;
                          ++cycle_recovery_response_steps;
                        }
                        progress_recovered = cycle_cleared &&
                            std::abs(read_full_object_x() -
                                     params.object.goal_pose.x()) <=
                                progress_lateral_reserve_limit;
                        if (progress_recovered) {
                          // The corrective face is the last physically
                          // engaged face in measured state. Preserve it as
                          // the release reference for the next receding
                          // iteration even when the rejected progress push
                          // made no y/yaw progress.
                          active_release_name = cycle_candidate->sample_name;
                          active_release_sample = ContactSample{
                              cycle_candidate->sample_point_O,
                              cycle_candidate->sample_normal_O,
                              active_release_name.c_str(), cycle_face};
                        }
                        if (cycle_wrong_polarity_rejected) {
                          std::cout <<
                              "full_sampling_c3plus_cycle_recovery_"
                              "wrong_polarity_release="
                                    << (cycle_cleared ? "PASS" : "FAIL")
                                    << " updates=" << full_execution_updates
                                    << std::endl;
                        }
                        if (cycle_contact_lost_rejected) {
                          cycle_contact_loss_retry =
                              cycle_cleared && !progress_recovered;
                          std::cout <<
                              "full_sampling_c3plus_cycle_recovery_"
                              "contact_loss_release="
                                    << (cycle_cleared ? "PASS" : "FAIL")
                                    << " updates=" << full_execution_updates
                                    << std::endl;
                        }
                      }
                      if (cycle_contacted && !cycle_contact_loss_retry) break;

                      const std::string failed_cycle_sample =
                          cycle_candidate->sample_name;
                      const char* failed_phase =
                          cycle_contact_loss_retry ? "contact_response" :
                          !cycle_lifted ? "lift" :
                          !cycle_anchored ? "neutral_anchor" :
                          !cycle_verticalized ? "verticalize" :
                          !cycle_traversed ? "traverse" :
                          !cycle_lowered ? "lower" :
                          !cycle_descended ? "descend" : "contact";
                      bool failed_approach_released =
                          cycle_contact_loss_retry;
                      while (!cycle_contacted &&
                             !failed_approach_released &&
                             full_execution_updates <
                                 FLAGS_full_execution_steps) {
                        const Eigen::Vector3d release_pose =
                            read_full_object_pose();
                        const Eigen::Rotation2Dd release_R_WO(
                            release_pose.z());
                        const Eigen::Vector2d release_normal_W =
                            release_R_WO * cycle_sample.normal;
                        const Eigen::Vector2d release_boundary_W =
                            release_pose.head<2>() +
                            release_R_WO * cycle_sample.point;
                        const Eigen::Vector3d release_tip = read_full_tip();
                        const double release_gap = release_normal_W.dot(
                            release_tip.head<2>() - release_boundary_W) -
                            params.controller.pusher_radius;
                        if (release_gap >=
                            params.controller.approach_clearance) {
                          failed_approach_released = true;
                          break;
                        }
                        Eigen::Vector3d release_step = release_tip;
                        release_step.head<2>() += release_normal_W * std::min(
                            params.controller.task_space_plan_step_limit,
                            params.controller.approach_clearance -
                                release_gap + params.controller
                                    .contact_activation_tolerance);
                        const int updates_before_release =
                            full_execution_updates;
                        if (!execute_posture_waypoint(
                                release_step, release_pose, cycle_sample,
                                "cycle_recovery_fallback_release", true,
                                false) ||
                            full_execution_updates ==
                                updates_before_release) {
                          break;
                        }
                      }
                      std::cout <<
                          "full_sampling_c3plus_cycle_recovery_fallback_"
                          "release="
                                << (failed_approach_released ? "PASS" :
                                    "FAIL")
                                << " failed_phase=" << failed_phase
                                << " updates=" << full_execution_updates
                                << std::endl;
                      cycle_execution_rejections.insert(
                          failed_cycle_sample);
                      cycle_release_name = failed_cycle_sample;
                      cycle_release_sample = ContactSample{
                          cycle_candidate->sample_point_O,
                          cycle_candidate->sample_normal_O,
                          cycle_release_name.c_str(), cycle_face};
                      cycle_live_pose = read_full_object_pose();
                      cycle_candidate = failed_approach_released
                          ? select_next_cycle_candidate() : nullptr;
                      std::cout <<
                          "full_sampling_c3plus_cycle_recovery_execution_"
                          "fallback="
                                << (cycle_candidate != nullptr ? "RETRY" :
                                    "EXHAUSTED")
                                << " failed_sample=" << failed_cycle_sample
                                << " failed_phase=" << failed_phase
                                << " rejected_candidates="
                                << cycle_execution_rejections.size()
                                << " next_sample="
                                << (cycle_candidate != nullptr ?
                                    cycle_candidate->sample_name : "")
                                << " contact_only_fallback="
                                << cycle_contact_only_fallback
                                << " updates=" << full_execution_updates
                                << std::endl;
                    }
                    std::cout <<
                        "full_sampling_c3plus_cycle_lateral_recovery="
                              << (progress_recovered ? "PASS" : "FAIL")
                              << " released_drift_m="
                              << std::abs(read_full_object_x() -
                                          params.object.goal_pose.x())
                              << " reserve_limit_m="
                              << progress_lateral_reserve_limit
                              << " updates=" << full_execution_updates
                              << std::endl;
                  }
                  const Eigen::Vector3d post_recovery_pose =
                      read_full_object_pose();
                  const auto post_recovery_progress =
                      EvaluateFullSamplingC3PostRecoveryProgress(
                          progress_start_pose, post_recovery_pose,
                          params.object.goal_pose,
                          params.controller.lateral_drift_tolerance,
                          params.controller
                              .successor_minimum_translation_progress,
                          params.controller.successor_minimum_yaw_progress);
                  const double post_recovery_y_progress =
                      std::abs(progress_start_pose.y() -
                               params.object.goal_pose.y()) -
                      std::abs(post_recovery_pose.y() -
                               params.object.goal_pose.y());
                  const double post_recovery_yaw_progress =
                      std::abs(WrappedAngleError(
                          params.object.goal_pose.z(),
                          progress_start_pose.z())) -
                      std::abs(WrappedAngleError(
                          params.object.goal_pose.z(),
                          post_recovery_pose.z()));
                  if (selected_cycle_recovery_prediction.has_value()) {
                    const auto measured_recovery_descent =
                        EvaluateFullSamplingC3NormalizedParetoDescent(
                            progress_start_pose, post_recovery_pose,
                            params.object.goal_pose,
                            params.task.translation_tolerance,
                            params.task.orientation_tolerance);
                    const bool prediction_sign_conformant =
                        selected_cycle_recovery_prediction->terminal.accepted &&
                        measured_recovery_descent.terminal.accepted;
                    std::cout <<
                        "full_sampling_c3plus_cycle_recovery_descent_"
                        "conformance="
                              << (prediction_sign_conformant ? "PASS" :
                                  "FAIL")
                              << " predicted_normalized_magnitude="
                              << selected_cycle_recovery_prediction
                                     ->normalized_magnitude
                              << " measured_normalized_magnitude="
                              << measured_recovery_descent
                                     .normalized_magnitude
                              << " normalized_residual="
                              << measured_recovery_descent
                                     .normalized_magnitude -
                                  selected_cycle_recovery_prediction
                                     ->normalized_magnitude
                              << " predicted_translation_progress_m="
                              << selected_cycle_recovery_prediction
                                     ->terminal.translation_progress
                              << " measured_translation_progress_m="
                              << measured_recovery_descent.terminal
                                     .translation_progress
                              << " predicted_orientation_progress_rad="
                              << selected_cycle_recovery_prediction
                                     ->terminal.orientation_progress
                              << " measured_orientation_progress_rad="
                              << measured_recovery_descent.terminal
                                     .orientation_progress
                              << std::endl;
                  }
                  if (selected_cycle_recovery_observation.has_value()) {
                    selected_cycle_recovery_observation
                        ->measured_terminal_object_pose = post_recovery_pose;
                    selected_cycle_recovery_observation->lateral_rejected =
                        !post_recovery_progress.lateral_accepted;
                    measured_response_history.push_back(
                        *selected_cycle_recovery_observation);
                    recovery_response_history.push_back(
                        *selected_cycle_recovery_observation);
                    std::cout <<
                        "full_sampling_c3plus_cycle_recovery_response_"
                        "observation=PASS history_size="
                              << measured_response_history.size()
                              << " recovery_history_size="
                              << recovery_response_history.size()
                              << " sample_point_O="
                              << selected_cycle_recovery_observation
                                     ->sample_point_O.transpose()
                              << " sample_normal_O="
                              << selected_cycle_recovery_observation
                                     ->sample_normal_O.transpose()
                              << " lateral_rejected="
                              << selected_cycle_recovery_observation
                                     ->lateral_rejected
                              << " terminal_accepted="
                              << post_recovery_progress.terminal.accepted
                              << std::endl;
                  }
                  std::cout <<
                      "full_sampling_c3plus_post_recovery_progress="
                            << (post_recovery_progress.accepted ? "PASS" :
                                "FAIL")
                            << " pre_recovery_pose="
                            << progress_end_pose.transpose()
                            << " post_recovery_pose="
                            << post_recovery_pose.transpose()
                            << " translation_progress_m="
                            << post_recovery_progress.terminal
                                   .translation_progress
                            << " orientation_progress_rad="
                            << post_recovery_progress.terminal
                                   .orientation_progress
                            << " lateral_error_m="
                            << post_recovery_progress.lateral_error
                            << " lateral_accepted="
                            << post_recovery_progress.lateral_accepted
                            << " updates=" << full_execution_updates
                            << std::endl;
                  const int measured_release_recovery =
                      full_execution_updates -
                      release_recovery_start_updates;
                  measured_release_recovery_updates.push_back(
                      measured_release_recovery);
                  std::cout <<
                      "full_sampling_c3plus_release_recovery_receipt=PASS"
                            << " phase=progress_cycle"
                            << " phase_updates="
                            << measured_release_recovery
                            << " receipts="
                            << measured_release_recovery_updates.size()
                            << " maximum_updates="
                            << *std::max_element(
                                   measured_release_recovery_updates.begin(),
                                   measured_release_recovery_updates.end())
                            << std::endl;
                  full_task_progress_cycle =
                      post_recovery_progress.accepted &&
                      progress_recovered &&
                      (progress_knots_reached ||
                       progress_lateral_rejected);
                  const auto bounded_replanning_transaction =
                      EvaluateXarmFullSamplingC3ComponentTransaction(
                          progress_start_pose, post_recovery_pose,
                          params.object.goal_pose,
                          params.task.translation_tolerance,
                          params.task.orientation_tolerance,
                          params.controller
                              .successor_minimum_translation_progress,
                          params.controller.successor_minimum_yaw_progress);
                  const bool recovery_only_continuation =
                      !post_recovery_progress.accepted &&
                      progress_recovered &&
                      (progress_lateral_rejected ||
                       terminal_descent_released ||
                       bounded_replanning_transaction.accepted);
                  std::cout << "full_sampling_c3plus_task_progress_cycle="
                            << (full_task_progress_cycle ? "PASS" : "FAIL")
                            << " y_progress_m="
                            << post_recovery_y_progress
                            << " yaw_progress_rad="
                            << post_recovery_yaw_progress
                            << " translation_progress_m="
                            << post_recovery_progress.terminal
                                   .translation_progress
                            << " orientation_progress_rad="
                            << post_recovery_progress.terminal
                                   .orientation_progress
                            << " translation_nonregressive="
                            << post_recovery_progress.terminal
                                   .translation_nonregressive
                            << " orientation_nonregressive="
                            << post_recovery_progress.terminal
                                   .orientation_nonregressive
                            << " end_pose="
                            << post_recovery_pose.transpose()
                            << " updates=" << full_execution_updates
                            << std::endl;
                  if (full_task_progress_cycle) {
                    any_full_task_progress_cycle = true;
                    measured_productive_cycles.push_back({
                        post_recovery_progress.terminal.translation_progress,
                        post_recovery_progress.terminal.orientation_progress,
                        full_execution_updates -
                            progress_cycle_entry_updates});
                    const auto live_budget_estimate =
                        EstimateXarmFullSamplingC3TerminalBudget(
                            post_recovery_pose, params.object.goal_pose,
                            params.task.translation_tolerance,
                            params.task.orientation_tolerance,
                            measured_productive_cycles);
                    const auto budget_sufficiency =
                        EvaluateXarmFullSamplingC3TerminalBudgetSufficiency(
                            full_execution_updates,
                            FLAGS_full_execution_steps,
                            live_budget_estimate);
                    std::cout <<
                        "full_sampling_c3plus_terminal_budget_sufficiency="
                              << (budget_sufficiency.sufficient ? "PASS" :
                                  "FAIL")
                              << " remaining_updates="
                              << budget_sufficiency.remaining_updates
                              << " optimistic_required_updates="
                              << budget_sufficiency
                                     .optimistic_required_updates
                              << " finite_estimate="
                              << budget_sufficiency.finite_estimate
                              << " necessary_not_sufficient=1"
                              << std::endl;
                    ++progress_cycle_count;
                    live_execution_rejections.clear();
                    if (!progress_lateral_rejected) {
                      active_release_name = progress_plan.sample_name;
                      active_release_sample = ContactSample{
                          progress_plan.sample_point_O,
                          progress_plan.sample_normal_O,
                          active_release_name.c_str(), progress_face};
                    }
                    corrective_lateral_recovery = progress_recovered;
                    std::cout <<
                        "full_sampling_c3plus_receding_cycle=PASS cycle="
                              << progress_cycle_count
                              << " object_pose="
                              << progress_end_pose.transpose()
                              << " updates=" << full_execution_updates
                              << std::endl;
                  } else if (recovery_only_continuation) {
                    // A wrong-polarity response is not scientific progress,
                    // but once its corrective face has restored the lateral
                    // corridor and cleared contact it is a valid state from
                    // which to re-solve. Do not terminate the receding loop.
                    corrective_lateral_recovery = true;
                    if (terminal_descent_released) {
                      active_release_name = progress_plan.sample_name;
                      active_release_sample = ContactSample{
                          progress_plan.sample_point_O,
                          progress_plan.sample_normal_O,
                          active_release_name.c_str(), progress_face};
                    }
                    std::cout <<
                        "full_sampling_c3plus_receding_recovery_"
                        "continuation=PASS cycle="
                              << progress_cycle_count
                              << " bounded_component_transaction="
                              << bounded_replanning_transaction.accepted
                              << " normalized_magnitude="
                              << bounded_replanning_transaction
                                     .normalized_magnitude
                              << " translation_debt_bounded="
                              << bounded_replanning_transaction
                                     .translation_debt_bounded
                              << " orientation_debt_bounded="
                              << bounded_replanning_transaction
                                     .orientation_debt_bounded
                              << " object_pose="
                              << read_full_object_pose().transpose()
                              << " updates=" << full_execution_updates
                              << std::endl;
                  } else {
                    corrective_lateral_recovery = false;
                  }
                } else {
                  corrective_lateral_recovery = false;
                }
              }
            }
          } else {
            std::cout << "full_sampling_c3plus_corrective_resample=FAIL"
                      << " reason=no_force_polarity_candidate" << std::endl;
          }
        }
      }
      const bool initial_plan_handoff =
          reached_waypoints == osc_execution_plan.positions_W.cols();
      std::cout << "full_sampling_c3plus_measured_closed_loop="
                << ((initial_plan_handoff || any_full_task_progress_cycle)
                        ? "PASS" : "FAIL")
                << " reached=" << reached_waypoints
                << " total=" << osc_execution_plan.positions_W.cols()
                << " updates=" << full_execution_updates
                << " subtargets=" << commanded_subtargets
                << " measured_lateral_rejected="
                << measured_lateral_rejected
                << " productive_handoff_preserved="
                << any_full_task_progress_cycle
                << " max_initial_error=" << maximum_tracking_error
                << std::endl;
      const Eigen::Vector3d hold_tip = read_full_tip();
      const Eigen::VectorXd hold_q =
          acquisition_plant.GetPositions(*acquisition_context);
      const double hold_time =
          full_state_subscriber.message().utime * 1.0e-6;
      LcmTrajectory::Trajectory hold_task;
      hold_task.traj_name = "end_effector_position_target";
      hold_task.datatypes = {"x", "y", "z"};
      hold_task.time_vector = Eigen::Vector2d(
          hold_time, hold_time + params.task.planning_time_step);
      hold_task.datapoints.resize(3, 2);
      hold_task.datapoints.col(0) = hold_tip;
      hold_task.datapoints.col(1) = hold_tip;
      LcmTrajectory::Trajectory hold_axis;
      hold_axis.traj_name = "end_effector_stick_axis_target";
      hold_axis.datatypes = {"x", "y", "z"};
      hold_axis.time_vector = hold_task.time_vector;
      hold_axis.datapoints.resize(3, 2);
      hold_axis.datapoints.col(0) = vertical_axis;
      hold_axis.datapoints.col(1) = vertical_axis;
      LcmTrajectory::Trajectory hold_posture;
      hold_posture.traj_name =
          "collision_aware_reposition_posture_target";
      hold_posture.datatypes = params.robot.controlled_joints;
      hold_posture.time_vector = hold_task.time_vector;
      hold_posture.datapoints.resize(hold_q.size(), 2);
      hold_posture.datapoints.col(0) = hold_q;
      hold_posture.datapoints.col(1) = hold_q;
      LcmTrajectory hold_trajectory(
          {hold_task, hold_axis, hold_posture},
          {hold_task.traj_name, hold_axis.traj_name, hold_posture.traj_name},
          hold_task.traj_name,
          "Full Sampling-C3+ terminal measured-state hold", false);
      dairlib::lcmt_timestamped_saved_traj hold_message;
      hold_message.utime = full_state_subscriber.message().utime;
      hold_message.saved_traj = hold_trajectory.GenerateLcmObject();
      for (int publish = 0; publish < FLAGS_publish_count; ++publish) {
        drake::lcm::Publish(&full_lcm,
                            params.lcm.tracking_trajectory_channel,
                            hold_message);
        full_lcm.HandleSubscriptions(10);
        while (full_lcm.HandleSubscriptions(0) > 0) {
        }
      }
      std::cout << "full_sampling_c3plus_terminal_hold=PASS tip_W="
                << hold_tip.transpose() << " q=" << hold_q.transpose()
                << std::endl;
      const Eigen::Vector3d terminal_pose = read_full_object_pose();
      const double terminal_translation_error =
          (terminal_pose.head<2>() - params.object.goal_pose.head<2>()).norm();
      const double terminal_orientation_error = std::abs(
          WrappedAngleError(params.object.goal_pose.z(), terminal_pose.z()));
      const bool open_table_terminal =
          terminal_translation_error <= params.task.translation_tolerance &&
          terminal_orientation_error <= params.task.orientation_tolerance;
      const auto terminal_budget_estimate =
          EstimateXarmFullSamplingC3TerminalBudget(
              terminal_pose, params.object.goal_pose,
              params.task.translation_tolerance,
              params.task.orientation_tolerance,
              measured_productive_cycles);
      std::cout << "full_sampling_c3plus_terminal_budget_estimate="
                << (terminal_budget_estimate.finite ? "PASS" : "UNAVAILABLE")
                << " measured_cycles=" << measured_productive_cycles.size()
                << " required_translation_progress_m="
                << terminal_budget_estimate.required_translation_progress
                << " required_orientation_progress_rad="
                << terminal_budget_estimate.required_orientation_progress
                << " maximum_translation_progress_m="
                << terminal_budget_estimate
                       .maximum_measured_translation_progress
                << " maximum_orientation_progress_rad="
                << terminal_budget_estimate
                       .maximum_measured_orientation_progress
                << " minimum_cycle_updates="
                << terminal_budget_estimate.minimum_measured_cycle_updates
                << " optimistic_remaining_cycles="
                << terminal_budget_estimate.optimistic_remaining_cycles
                << " optimistic_remaining_updates="
                << terminal_budget_estimate.optimistic_remaining_updates
                << std::endl;
      std::cout << "full_sampling_c3plus_open_table_terminal="
                << (open_table_terminal ? "PASS" : "FAIL")
                << " object_pose=" << terminal_pose.transpose()
                << " translation_error_m=" << terminal_translation_error
                << " orientation_error_rad=" << terminal_orientation_error
                << " translation_tolerance_m="
                << params.task.translation_tolerance
                << " orientation_tolerance_rad="
                << params.task.orientation_tolerance << std::endl;
      const auto terminal_status =
          EvaluateXarmFullSamplingC3TerminalStatus(
              reached_waypoints, osc_execution_plan.positions_W.cols(),
              any_full_task_progress_cycle, contact_cycle_budget_deferred,
              full_execution_updates, FLAGS_full_execution_steps,
              open_table_terminal);
      std::cout << "full_sampling_c3plus_acceptance_gate="
                << (terminal_status.accepted ? "PASS" : "FAIL")
                << " reason=" << terminal_status.reason
                << " cycle_budget_deferred="
                << contact_cycle_budget_deferred
                << " productive_handoff_preserved="
                << any_full_task_progress_cycle
                << " updates=" << full_execution_updates
                << " budget=" << FLAGS_full_execution_steps << std::endl;
      if (!terminal_status.accepted) return terminal_status.return_code;
    }
    return 0;
  }
  const double contact_normal_step =
      std::isfinite(FLAGS_contact_normal_step_override)
          ? FLAGS_contact_normal_step_override
          : params.controller.minimum_contact_normal_step;
  if (contact_normal_step <= 0.0 ||
      contact_normal_step > params.controller.task_space_plan_step_limit) {
    throw std::runtime_error(
        "contact normal-step override must be positive and no larger than "
        "task_space_plan_step_limit");
  }
  if (FLAGS_first_solve_only) {
    RunFirstC3PlusSolve(params);
    return 0;
  }
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModels(params.robot.model);
  AddXarmActuators(params, &plant);
  plant.Finalize();
  ValidateXarmPlant(plant, params);
  auto context = plant.CreateDefaultContext();
  SetXarmHome(params, plant, context.get());
  const Eigen::Vector3d home_tip =
      plant.EvalBodyPoseInWorld(
          *context, plant.GetBodyByName(params.robot.end_effector_body)) *
      params.robot.end_effector_point;
  Eigen::Vector3d live_tip_for_execution_trajectory = home_tip;
  bool publish_moving_execution_trajectory = false;
  Eigen::VectorXd live_controlled_positions = plant.GetPositions(*context);
  std::optional<Eigen::VectorXd> reposition_posture_target;
  std::optional<Eigen::Vector3d> contact_stick_axis_target;

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  systems::Subscriber<dairlib::lcmt_robot_output> state_subscriber(
      &lcm, params.lcm.robot_state_channel);
  systems::Subscriber<dairlib::lcmt_object_state> object_subscriber(
      &lcm, params.lcm.object_state_channel);
  while (state_subscriber.count() == 0 || object_subscriber.count() == 0) {
    lcm.HandleSubscriptions(100);
  }

  auto publish_target = [&](const Eigen::Vector3d& task_target,
                            int publish_count) {
    LcmTrajectory::Trajectory target;
    target.traj_name = "end_effector_position_target";
    target.datatypes = {"x", "y", "z"};
    // DAIRLab's execution trajectories carry the measured current knot and a
    // future C3/reposition knot, allowing OSC to use the planned velocity. The
    // staged free-space acquisition predates that contract and intentionally
    // retains its constant setpoint until descent/contact is established.
    target.time_vector.resize(2);
    target.time_vector[0] = state_subscriber.message().utime * 1e-6;
    target.time_vector[1] =
        target.time_vector[0] + params.task.planning_time_step;
    target.datapoints.resize(3, 2);
    target.datapoints.col(0) = publish_moving_execution_trajectory
        ? live_tip_for_execution_trajectory
        : task_target;
    target.datapoints.col(1) = task_target;
    std::vector<LcmTrajectory::Trajectory> targets{target};
    std::vector<std::string> target_names{target.traj_name};
    if (reposition_posture_target.has_value()) {
      LcmTrajectory::Trajectory posture;
      posture.traj_name = "collision_aware_reposition_posture_target";
      posture.datatypes = params.robot.controlled_joints;
      posture.time_vector = target.time_vector;
      posture.datapoints.resize(live_controlled_positions.size(), 2);
      posture.datapoints.col(0) = live_controlled_positions;
      posture.datapoints.col(1) = *reposition_posture_target;
      targets.push_back(posture);
      target_names.push_back(posture.traj_name);
    }
    if (contact_stick_axis_target.has_value()) {
      LcmTrajectory::Trajectory axis;
      axis.traj_name = "end_effector_stick_axis_target";
      axis.datatypes = {"x", "y", "z"};
      axis.time_vector = target.time_vector;
      axis.datapoints.resize(3, 2);
      axis.datapoints.col(0) = *contact_stick_axis_target;
      axis.datapoints.col(1) = *contact_stick_axis_target;
      targets.push_back(axis);
      target_names.push_back(axis.traj_name);
    }
    LcmTrajectory trajectory(targets, target_names, target.traj_name,
                             "OIM xArm task-space and posture targets", false);
    dairlib::lcmt_timestamped_saved_traj message;
    message.utime = state_subscriber.message().utime;
    message.saved_traj = trajectory.GenerateLcmObject();
    for (int i = 0; i < publish_count; ++i) {
      drake::lcm::Publish(&lcm, params.lcm.tracking_trajectory_channel, message);
      lcm.HandleSubscriptions(
          FLAGS_live_sampled_plan ? FLAGS_live_step_period_ms : 10);
      // Match DAIRLab's LcmDrivenLoop contract: one blocking receive can leave
      // hundreds of high-rate robot states queued behind the message it
      // returned.  Drain to the newest state before the next solve so control
      // does not accumulate memory or operate on an ever-growing delay.
      while (lcm.HandleSubscriptions(0) > 0) {
      }
    }
  };

  std::optional<Eigen::Vector3d> approach_hold_pose;
  std::optional<Eigen::Vector3d> descent_waypoint;
  std::string approach_contact_name;
  std::optional<int> approach_sample_index;
  enum class ApproachPhase {
    kAlignY,
    kAlignX,
    kDescend,
    kEngage,
    kContact,
    kRepositionRetreat,
    kRepositionLift,
    kRepositionOrient,
    kRepositionTraverse,
    kRepositionDescend,
  };
  ApproachPhase approach_phase_state = ApproachPhase::kAlignY;
  std::optional<Eigen::Vector2d> reposition_lift_xy;
  std::optional<Eigen::Vector3d> reposition_object_pose_start;
  std::optional<int> reposition_orient_start_step;
  int contact_steps_on_face = 0;
  int progress_window_start_step = 0;
  double progress_window_start_yaw_error = 0.0;
  double progress_window_start_y_error = 0.0;
  int completed_repositions = 0;
  bool successor_geometric_contact_reported = false;
  std::optional<Eigen::Vector3d> successor_contact_pose_start;
  std::optional<Eigen::Vector3d> multiface_cycle_pose_start;
  int multiface_cycle_start_repositions = 0;
  int accepted_multiface_cycles = 0;
  std::optional<double> lateral_recovery_start_error;
  std::optional<double> lateral_recovery_start_signed_error;
  std::optional<Eigen::Vector2d> lateral_recovery_tip_start;
  double lateral_recovery_best_error =
      std::numeric_limits<double>::infinity();
  bool active_face_requires_lateral_recovery = false;
  bool lateral_recovery_uses_inline_fallback = false;
  bool lateral_recovery_entry_force_correct = false;
  bool lateral_recovery_corridor_reached = false;
  bool lateral_recovery_receipt_reported = false;
  bool contact_capsule_command_rejected = false;
  int completed_lateral_recoveries = 0;
  int successor_contact_start_step_on_face = 0;
  double reposition_min_traverse_z = std::numeric_limits<double>::infinity();
  RepositionCollisionReceipt reposition_collision_receipt;
  const bool diagnostic_overrides_enabled =
      FLAGS_initial_contact_sample_index >= 0 ||
      FLAGS_force_successor_after_contact_steps > 0 ||
      FLAGS_diagnostic_successor_sample_index >= 0 ||
      std::isfinite(FLAGS_contact_normal_step_override);
  auto solve_live_target = [&](int step_index) {
    const auto& robot_message = state_subscriber.message();
    for (int i = 0; i < robot_message.num_positions; ++i) {
      plant.GetJointByName(robot_message.position_names[i])
          .SetPositions(context.get(), Eigen::VectorXd::Constant(
                                          1, robot_message.position[i]));
    }
    const auto X_WEe = plant.EvalBodyPoseInWorld(
        *context, plant.GetBodyByName(params.robot.end_effector_body));
    const Eigen::Vector3d live_tip =
        X_WEe * params.robot.end_effector_point;
    const Eigen::Vector3d stick_axis_3d_W =
        X_WEe.rotation() * Eigen::Vector3d::UnitZ();
    live_tip_for_execution_trajectory = live_tip;
    publish_moving_execution_trajectory = false;
    live_controlled_positions = plant.GetPositions(*context);
    reposition_posture_target.reset();
    contact_stick_axis_target.reset();
    const Eigen::Vector2d stick_axis_W = stick_axis_3d_W.head<2>();
    const auto& object_message = object_subscriber.message();
    auto position_value = [&](const std::string& name) {
      for (int i = 0; i < object_message.num_positions; ++i) {
        if (object_message.position_names[i] == name)
          return object_message.position[i];
      }
      throw std::runtime_error("live T state missing " + name);
    };
    const Eigen::Vector2d object_xy(position_value("block_x"),
                                    position_value("block_y"));
    const double object_z = position_value("block_z");
    const double qw = position_value("block_qw");
    const double qx = position_value("block_qx");
    const double qy = position_value("block_qy");
    const double qz = position_value("block_qz");
    const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                  1.0 - 2.0 * (qy * qy + qz * qz));
    const Eigen::Vector3d object_pose(object_xy.x(), object_xy.y(), yaw);
    const Eigen::Rotation2Dd R_WO(yaw);
    const auto& samples = ExactTSamples();
    if (!approach_sample_index.has_value() &&
        FLAGS_initial_contact_sample_index >= 0) {
      if (FLAGS_initial_contact_sample_index >=
          static_cast<int>(samples.size())) {
        throw std::runtime_error("initial_contact_sample_index is out of range");
      }
      approach_sample_index = FLAGS_initial_contact_sample_index;
    }
    const bool log_step = FLAGS_live_log_every > 0 &&
        (step_index % FLAGS_live_log_every == 0 ||
         step_index + 1 == FLAGS_live_control_steps);

    // A separated unilateral LCS cannot create contact by itself. Select the
    // face whose reaction on the object (-normal) best matches the goal
    // direction, breaking equal-face ties by pusher proximity, and explicitly
    // align the physical tip with that world-frame contact before solving C3+.
    Eigen::Vector2d goal_direction =
        params.object.goal_pose.head<2>() - object_xy;
    if (goal_direction.norm() > 0.0) goal_direction.normalize();
    const ContactSample* approach_sample = nullptr;
    Eigen::Vector2d approach_point_W;
    Eigen::Vector2d descent_point_W;
    double approach_score = -std::numeric_limits<double>::infinity();
    for (int sample_index = 0; sample_index < static_cast<int>(samples.size());
         ++sample_index) {
      const auto& sample = samples[sample_index];
      const Eigen::Vector2d normal_W = R_WO * sample.normal;
      const Eigen::Vector2d point_W =
          object_xy + R_WO * sample.point +
          normal_W * (params.controller.pusher_radius +
                      params.controller.approach_clearance);
      const double score =
          (-normal_W).dot(goal_direction) - 1.0e-3 *
          (point_W - live_tip.head<2>()).squaredNorm();
      if ((!approach_sample_index.has_value() && score > approach_score) ||
          approach_sample_index == sample_index) {
        approach_score = score;
        approach_sample = &sample;
        approach_point_W = point_W;
        descent_point_W = object_xy + R_WO * sample.point +
            normal_W * (params.controller.pusher_radius +
                        params.controller.descent_clearance);
        if (approach_sample_index == sample_index) break;
      }
    }
    if (approach_sample == nullptr) {
      throw std::runtime_error("no exact-T approach sample available");
    }
    if (!approach_hold_pose.has_value() ||
        approach_contact_name != approach_sample->name) {
      approach_sample_index = static_cast<int>(
          std::distance(samples.data(), approach_sample));
      approach_hold_pose = live_tip;
      approach_contact_name = approach_sample->name;
      approach_phase_state = ApproachPhase::kAlignY;
      descent_waypoint.reset();
    }
    Eigen::Vector3d approach_target(descent_point_W.x(), descent_point_W.y(),
                                    object_z);
    const bool repositioning =
        approach_phase_state == ApproachPhase::kRepositionRetreat ||
        approach_phase_state == ApproachPhase::kRepositionLift ||
        approach_phase_state == ApproachPhase::kRepositionOrient ||
        approach_phase_state == ApproachPhase::kRepositionTraverse ||
        approach_phase_state == ApproachPhase::kRepositionDescend;
    if (repositioning) {
      publish_moving_execution_trajectory = true;
      if (!reposition_lift_xy.has_value() ||
          !reposition_object_pose_start.has_value()) {
        throw std::runtime_error("reposition phase is missing latched state");
      }
      double shaft_clearance =
          -(R_WO * approach_sample->normal).dot(stick_axis_W);
      // The successor selected at the contact boundary is the transaction's
      // task-space intent.  Keep that face latched through retreat, lift and
      // traverse.  Re-running task-progress arbitration during traverse used
      // to replace (for example) a selected stem-bottom contact with a
      // crossbar-right contact, so the collision receipt described a
      // different transaction from successor_face_switch.  The elevated
      // orientation fallback below remains the only allowed replacement: it
      // runs after a bounded tracking window and requires explicit shaft
      // clearance for the replacement face.
      const Eigen::Vector3d lift_target(
          reposition_lift_xy->x(), reposition_lift_xy->y(),
          params.controller.reposition_waypoint_height);
      const Eigen::Vector3d retreat_target(
          reposition_lift_xy->x(), reposition_lift_xy->y(), object_z);
      // The task-progress face is chosen while the pusher is still in contact,
      // before the outward retreat changes the six-DOF shaft posture.  Once
      // the tip is elevated, revalidate that face against the same clearance
      // required by the orient-to-traverse gate.  Replanning here prevents an
      // unreachable face from deadlocking kRepositionOrient; the pusher is
      // already above the full swept-capsule clearance height, so changing the
      // intended face does not create a new contact transition.
      if (approach_phase_state == ApproachPhase::kRepositionOrient &&
          (live_tip - lift_target).norm() <=
              params.controller.contact_activation_tolerance &&
          shaft_clearance < 1.0e-3 &&
          reposition_orient_start_step.has_value() &&
          step_index - *reposition_orient_start_step >=
              params.controller.successor_progress_window_steps) {
        const int previous_sample_index = *approach_sample_index;
        CandidateSelection safe_successor = SelectTaskProgressCandidate(
            params, object_pose, approach_sample->face, stick_axis_W,
            1.0e-3);
        if (!safe_successor.solve.finite) {
          throw std::runtime_error(
              "elevated lift pose has no shaft-safe task-progress face");
        }
        approach_sample_index = safe_successor.sample_index;
        approach_sample = &samples[*approach_sample_index];
        approach_contact_name = approach_sample->name;
        const Eigen::Vector2d replanned_normal_W =
            R_WO * approach_sample->normal;
        approach_point_W = object_xy + R_WO * approach_sample->point +
            replanned_normal_W * (params.controller.pusher_radius +
                                  params.controller.approach_clearance);
        descent_point_W = object_xy + R_WO * approach_sample->point +
            replanned_normal_W * (params.controller.pusher_radius +
                                  params.controller.descent_clearance);
        shaft_clearance = -replanned_normal_W.dot(stick_axis_W);
        std::cout << "reposition_orient_face_replan=PASS step=" << step_index
                  << " from=" << samples[previous_sample_index].name
                  << " to=" << approach_sample->name
                  << " shaft_clearance=" << shaft_clearance << std::endl;
      }
      // The configured descent clearance locates the tip center outside the
      // face.  During a six-DOF vertical descent the rest of the inclined
      // finite-radius capsule can still graze the T by a few micrometers.
      // Keep one known capsule radius of execution standoff until descent is
      // complete; kEngage then moves laterally to the unchanged approach and
      // contact clearances.
      const Eigen::Vector2d reposition_descent_point_W =
          descent_point_W +
          (R_WO * approach_sample->normal) *
              params.controller.pusher_radius;
      const Eigen::Vector3d traverse_target(
          reposition_descent_point_W.x(), reposition_descent_point_W.y(),
          params.controller.reposition_waypoint_height);
      const Eigen::Vector3d descend_target(
          reposition_descent_point_W.x(), reposition_descent_point_W.y(),
          object_z);
      const char* reposition_phase = "reposition_retreat";
      Eigen::Vector3d waypoint = retreat_target;
      if (approach_phase_state == ApproachPhase::kRepositionRetreat &&
          (live_tip - retreat_target).norm() <=
              params.controller.contact_activation_tolerance) {
        approach_phase_state = ApproachPhase::kRepositionLift;
      }
      if (approach_phase_state == ApproachPhase::kRepositionLift) {
        reposition_phase = "reposition_lift";
        waypoint = lift_target;
      }
      if (approach_phase_state == ApproachPhase::kRepositionLift &&
          (live_tip - lift_target).norm() <=
              params.controller.contact_activation_tolerance) {
        approach_phase_state = ApproachPhase::kRepositionOrient;
        reposition_orient_start_step = step_index;
      }
      if (approach_phase_state == ApproachPhase::kRepositionOrient) {
        reposition_phase = "reposition_orient";
        waypoint = lift_target;
        if (shaft_clearance >= 1.0e-3 &&
            (live_tip - lift_target).norm() <=
                params.controller.contact_activation_tolerance) {
          approach_phase_state = ApproachPhase::kRepositionTraverse;
          reposition_orient_start_step.reset();
        }
      }
      if (approach_phase_state == ApproachPhase::kRepositionTraverse) {
        reposition_phase = "reposition_traverse";
        waypoint = traverse_target;
        reposition_min_traverse_z =
            std::min(reposition_min_traverse_z, live_tip.z());
        if ((live_tip - traverse_target).norm() <=
            params.controller.contact_activation_tolerance) {
          approach_phase_state = ApproachPhase::kRepositionDescend;
        }
      }
      if (approach_phase_state == ApproachPhase::kRepositionDescend) {
        reposition_phase = "reposition_descend";
        waypoint = descend_target;
        if ((live_tip - descend_target).norm() <=
            params.controller.contact_activation_tolerance) {
          approach_phase_state = ApproachPhase::kEngage;
          waypoint = Eigen::Vector3d(
              approach_point_W.x(), approach_point_W.y(), object_z);
          const double object_motion =
              (object_xy - reposition_object_pose_start->head<2>()).norm();
          const double object_yaw_motion = std::abs(WrappedAngleError(
              yaw, reposition_object_pose_start->z()));
          const double minimum_collision_free_height =
              params.object.resting_height + params.controller.pusher_radius +
              params.controller.descent_clearance;
          const bool collision_free =
              object_motion <= params.controller.contact_activation_tolerance &&
              object_yaw_motion <= params.task.orientation_tolerance &&
              reposition_min_traverse_z >= minimum_collision_free_height &&
              reposition_collision_receipt.collision_free();
          std::cout << "collision_free_reposition="
                    << (collision_free ? "PASS" : "FAIL")
                    << " completed=" << completed_repositions
                    << " acceptance_cycle="
                    << accepted_multiface_cycles + 1
                    << " successor=" << approach_sample->name
                    << " object_motion_m=" << object_motion
                    << " object_yaw_motion_rad=" << object_yaw_motion
                    << " min_traverse_z_m=" << reposition_min_traverse_z
                    << " capsule_t_clear="
                    << reposition_collision_receipt.capsule_t_clear
                    << " capsule_table_clear="
                    << reposition_collision_receipt.capsule_table_clear
                    << " tip_t_clear="
                    << reposition_collision_receipt.tip_t_clear
                    << " tip_table_clear="
                    << reposition_collision_receipt.tip_table_clear
                    << " capsule_table_margin_m="
                    << reposition_collision_receipt.capsule_table_margin
                    << " tip_table_margin_m="
                    << reposition_collision_receipt.tip_table_margin
                    << std::endl;
        }
      }
      Eigen::Vector3d task_target = live_tip;
      Eigen::Vector3d step = waypoint - live_tip;
      double posture_target_shaft_clearance = shaft_clearance;
      double posture_target_tip_error = 0.0;
      const double reposition_step_limit =
          params.controller.reposition_speed * params.task.planning_time_step;
      if (step.norm() > reposition_step_limit) {
        step *= reposition_step_limit / step.norm();
      }
      task_target += step;
      if (approach_phase_state == ApproachPhase::kRepositionRetreat ||
          approach_phase_state == ApproachPhase::kRepositionLift ||
          approach_phase_state == ApproachPhase::kRepositionDescend) {
        const auto posture = SolveLiftPostureStep(
            plant, context.get(), params, live_controlled_positions,
            task_target);
        if (!posture.has_value()) {
          throw std::runtime_error(
              "bounded six-DOF release-lift IK failed");
        }
        reposition_posture_target = *posture;
        if (approach_phase_state == ApproachPhase::kRepositionDescend) {
          // Descend from the collision-cleared traverse pose without allowing
          // the face-axis objective to keep rotating the shaft as its capsule
          // enters the T's vertical span.  Check the same entire capsule and
          // tip receipt across this posture-preserving commanded joint step.
          for (int sample = 0; sample <= kSweptPostureSamples; ++sample) {
            const double alpha = static_cast<double>(sample) /
                static_cast<double>(kSweptPostureSamples);
            const Eigen::VectorXd q = live_controlled_positions + alpha *
                (*posture - live_controlled_positions);
            const auto receipt = EvaluateRepositionCollision(
                plant, context.get(), params, q, object_pose);
            if (!receipt.collision_free()) {
              throw std::runtime_error(
                  "posture-preserving descend swept capsule receipt failed");
            }
            reposition_collision_receipt.capsule_t_clear =
                reposition_collision_receipt.capsule_t_clear &&
                receipt.capsule_t_clear;
            reposition_collision_receipt.capsule_table_clear =
                reposition_collision_receipt.capsule_table_clear &&
                receipt.capsule_table_clear;
            reposition_collision_receipt.tip_t_clear =
                reposition_collision_receipt.tip_t_clear &&
                receipt.tip_t_clear;
            reposition_collision_receipt.tip_table_clear =
                reposition_collision_receipt.tip_table_clear &&
                receipt.tip_table_clear;
            reposition_collision_receipt.capsule_table_margin = std::min(
                reposition_collision_receipt.capsule_table_margin,
                receipt.capsule_table_margin);
            reposition_collision_receipt.tip_table_margin = std::min(
                reposition_collision_receipt.tip_table_margin,
                receipt.tip_table_margin);
          }
          plant.SetPositions(context.get(), live_controlled_positions);
        }
      } else {
        const double commanded_shaft_clearance =
            approach_phase_state == ApproachPhase::kRepositionOrient
                ? (params.controller.pusher_radius +
                   params.controller.contact_activation_tolerance) /
                      (kCapsuleEndZ - kCapsuleStartZ)
                : 1.0e-3;
        const auto posture = SolveCollisionAwarePostureStep(
            plant, context.get(), params, live_controlled_positions,
            task_target, R_WO * approach_sample->normal, object_pose,
            commanded_shaft_clearance);
        if (!posture.finite || !posture.receipt.collision_free()) {
          if (approach_phase_state != ApproachPhase::kRepositionTraverse) {
            throw std::runtime_error(
                "collision-aware six-DOF reposition IK or swept capsule "
                "receipt failed");
          }
          // If the next diagonal posture step fails the swept receipt, first
          // recover shaft clearance at the measured tip. A simultaneous lift
          // and rotation can itself sweep through the T top; the fixed-tip
          // collision-aware solve changes posture without advancing the
          // traverse, then retries the unchanged waypoint next update.
          task_target = live_tip;
          const auto posture_recovery = SolveCollisionAwarePostureStep(
              plant, context.get(), params, live_controlled_positions,
              task_target, R_WO * approach_sample->normal, object_pose,
              (params.controller.pusher_radius +
               params.controller.contact_activation_tolerance) /
                  (kCapsuleEndZ - kCapsuleStartZ));
          if (!posture_recovery.finite ||
              !posture_recovery.receipt.collision_free()) {
            const int blocked_sample_index = *approach_sample_index;
            CandidateSelection alternative = SelectTaskProgressCandidate(
                params, object_pose, approach_sample->face,
                Eigen::Vector2d::Zero());
            task_target = live_tip;
            reposition_posture_target = live_controlled_positions;
            if (alternative.solve.finite && alternative.sample_index >= 0) {
              approach_sample_index = alternative.sample_index;
              approach_sample = &samples[*approach_sample_index];
              approach_contact_name = approach_sample->name;
              approach_phase_state = ApproachPhase::kRepositionOrient;
              reposition_orient_start_step = step_index;
              std::cout << "reposition_traverse_reachability_replan=PASS step="
                        << step_index << " from="
                        << samples[blocked_sample_index].name << " to="
                        << approach_sample->name << std::endl;
            } else {
              std::cout << "reposition_traverse_reachability=BLOCKED step="
                        << step_index << " successor="
                        << samples[blocked_sample_index].name
                        << " measured_state_held=1" << std::endl;
            }
            return task_target;
          }
          reposition_posture_target = posture_recovery.target;
          std::cout << "reposition_traverse_posture_recovery=PASS step="
                    << step_index << " successor=" << approach_sample->name
                    << " live_z_m=" << live_tip.z()
                    << " target_shaft_clearance="
                    << posture_recovery.target_shaft_clearance << std::endl;
          return task_target;
        }
        reposition_posture_target = posture.target;
        posture_target_shaft_clearance = posture.target_shaft_clearance;
        posture_target_tip_error = posture.target_tip_error;
        reposition_collision_receipt.capsule_t_clear =
            reposition_collision_receipt.capsule_t_clear &&
            posture.receipt.capsule_t_clear;
        reposition_collision_receipt.capsule_table_clear =
            reposition_collision_receipt.capsule_table_clear &&
            posture.receipt.capsule_table_clear;
        reposition_collision_receipt.tip_t_clear =
            reposition_collision_receipt.tip_t_clear &&
            posture.receipt.tip_t_clear;
        reposition_collision_receipt.tip_table_clear =
            reposition_collision_receipt.tip_table_clear &&
            posture.receipt.tip_table_clear;
        reposition_collision_receipt.capsule_table_margin = std::min(
            reposition_collision_receipt.capsule_table_margin,
            posture.receipt.capsule_table_margin);
        reposition_collision_receipt.tip_table_margin = std::min(
            reposition_collision_receipt.tip_table_margin,
            posture.receipt.tip_table_margin);
      }
      if (log_step) {
        std::cout << "live_sampled_c3plus=PASS step=" << step_index
                  << " phase=" << reposition_phase
                  << " successor=" << approach_sample->name
                  << " shaft_clearance=" << shaft_clearance
                  << " posture_target_shaft_clearance="
                  << posture_target_shaft_clearance
                  << " posture_target_tip_error_m="
                  << posture_target_tip_error
                  << " waypoint_W=" << waypoint.transpose()
                  << " target_W=" << task_target.transpose() << std::endl;
      }
      return task_target;
    }
    const double x_error = approach_target.x() - live_tip.x();
    const double y_error = approach_target.y() - live_tip.y();
    const bool y_ready =
        std::abs(y_error) <= params.controller.approach_planar_tolerance;
    const bool x_ready =
        std::abs(x_error) <= params.controller.approach_planar_tolerance;
    if (approach_phase_state == ApproachPhase::kAlignY && y_ready) {
      approach_phase_state = ApproachPhase::kAlignX;
    }
    if (approach_phase_state == ApproachPhase::kAlignX && x_ready) {
      approach_phase_state = ApproachPhase::kDescend;
      descent_waypoint = Eigen::Vector3d(
          descent_point_W.x(), descent_point_W.y(),
          std::max(object_z, live_tip.z() -
              params.controller.descent_command_step_limit));
    }
    const char* approach_phase = "approach_z";
    if (approach_phase_state == ApproachPhase::kAlignY) {
      approach_phase = "approach_y";
      approach_target.x() = approach_hold_pose->x();
      approach_target.z() = approach_hold_pose->z();
    } else if (approach_phase_state == ApproachPhase::kAlignX) {
      approach_phase = "approach_x";
      approach_target.z() = approach_hold_pose->z();
    } else if (approach_phase_state == ApproachPhase::kDescend) {
      if (!descent_waypoint.has_value()) {
        throw std::runtime_error("descent phase is missing its latched waypoint");
      }
      approach_target = *descent_waypoint;
      if ((approach_target - live_tip).norm() <=
          params.controller.contact_activation_tolerance) {
        if (approach_target.z() <= object_z +
            params.controller.contact_activation_tolerance) {
          approach_phase_state = ApproachPhase::kEngage;
          approach_phase = "approach_engage";
          approach_target = Eigen::Vector3d(
              approach_point_W.x(), approach_point_W.y(), object_z);
        } else {
          descent_waypoint = Eigen::Vector3d(
              descent_point_W.x(), descent_point_W.y(),
              std::max(object_z, approach_target.z() -
                  params.controller.descent_command_step_limit));
          approach_target = *descent_waypoint;
        }
      }
    } else if (approach_phase_state == ApproachPhase::kEngage) {
      approach_phase = "approach_engage";
      approach_target.head<2>() = approach_point_W;
    }
    if (approach_phase_state == ApproachPhase::kDescend ||
        approach_phase_state == ApproachPhase::kEngage ||
        approach_phase_state == ApproachPhase::kContact) {
      // OIM's physical xArm policy keeps the capsule's local +Z axis
      // vertical-down while acquiring and sustaining side contact.  The OSC
      // source consumes this as a differential-IK condition alongside the
      // unchanged task-space translation target.
      contact_stick_axis_target = -Eigen::Vector3d::UnitZ();
    }
    const Eigen::Vector3d approach_error = approach_target - live_tip;
    const Eigen::Vector2d approach_normal_W = R_WO * approach_sample->normal;
    const Eigen::Vector2d approach_tangent_W(
        -approach_normal_W.y(), approach_normal_W.x());
    const Eigen::Vector2d approach_boundary_center_W =
        object_xy + R_WO * approach_sample->point;
    const double approach_normal_gap = approach_normal_W.dot(
        live_tip.head<2>() - approach_boundary_center_W) -
        params.controller.pusher_radius;
    const double approach_tangential_error = std::abs(
        approach_tangent_W.dot(
            live_tip.head<2>() - approach_boundary_center_W));
    const bool face_contact_aligned =
        approach_normal_gap <=
            params.controller.approach_clearance +
                params.controller.contact_activation_tolerance &&
        approach_tangential_error <=
            params.controller.approach_planar_tolerance;
    Eigen::Vector3d loaded_contact_tip_W;
    loaded_contact_tip_W.head<2>() = approach_boundary_center_W +
        approach_normal_W *
            (params.controller.pusher_radius -
             0.5 * params.controller.contact_activation_tolerance);
    loaded_contact_tip_W.z() = object_z;
    const auto loaded_contact_capsule =
        EvaluateSweptContactCapsuleClearance(
            params, live_tip, loaded_contact_tip_W, stick_axis_3d_W,
            -Eigen::Vector3d::UnitZ(), object_pose, *approach_sample);
    const bool contact_capsule_ready = loaded_contact_capsule.clear();
    if (approach_phase_state == ApproachPhase::kEngage &&
        face_contact_aligned && contact_capsule_ready) {
      approach_phase_state = ApproachPhase::kContact;
      contact_steps_on_face = 0;
      progress_window_start_step = 0;
      progress_window_start_yaw_error = std::abs(WrappedAngleError(
          params.object.goal_pose.z(), yaw));
      progress_window_start_y_error =
          std::abs(object_xy.y() - params.object.goal_pose.y());
      if (!multiface_cycle_pose_start.has_value()) {
        multiface_cycle_pose_start =
            Eigen::Vector3d(object_xy.x(), object_xy.y(), yaw);
        multiface_cycle_start_repositions = completed_repositions;
      }
      if (completed_repositions > 0) {
        std::cout << "successor_contact_handoff=PASS step=" << step_index
                  << " contact=" << approach_sample->name
                  << " gap_m=" << approach_normal_gap
                  << " tangential_error_m=" << approach_tangential_error
                  << " acceptance_cycle="
                  << accepted_multiface_cycles + 1
                  << " completed=" << completed_repositions << std::endl;
      }
    }
    const bool contact_ready = approach_phase_state == ApproachPhase::kContact;
    if (!contact_ready) {
      Eigen::Vector3d task_target = live_tip;
      Eigen::Vector3d step = approach_error;
      const double command_limit =
          approach_phase_state == ApproachPhase::kDescend
              ? params.controller.descent_command_step_limit
              : (approach_phase_state == ApproachPhase::kEngage &&
                         completed_repositions > 0
                     ? contact_normal_step
                     : params.controller.approach_command_step_limit);
      if (step.norm() > command_limit) {
        step *= command_limit / step.norm();
      }
      if (approach_phase_state == ApproachPhase::kEngage &&
          completed_repositions > 0) {
        publish_moving_execution_trajectory = true;
      }
      task_target += step;
      if (approach_phase_state == ApproachPhase::kEngage &&
          completed_repositions > 0) {
        const auto posture = SolveVerticalContactPostureStep(
            plant, context.get(), params, live_controlled_positions,
            task_target, object_pose, *approach_sample);
        if (posture.finite) {
          // Successor acquisition is not delegated to the weak descent
          // null-space objective.  Execute a velocity-bounded, measured-state
          // seeded IK step whose entire joint interpolation has already
          // passed the selected-face whole-capsule receipt.
          reposition_posture_target = posture.target;
        } else {
          task_target = live_tip;
          if (log_step) {
            std::cout << "successor_contact_reachability=REJECT step="
                      << step_index << " contact=" << approach_sample->name
                      << " shaft_t_clear=" << posture.receipt.shaft_t_clear
                      << " capsule_table_clear="
                      << posture.receipt.capsule_table_clear
                      << " tip_table_clear="
                      << posture.receipt.tip_table_clear << std::endl;
          }
        }
        if (log_step && posture.finite) {
          std::cout << "successor_contact_reachability=PASS step="
                    << step_index << " contact=" << approach_sample->name
                    << " target_tip_error_m=" << posture.target_tip_error
                    << " target_axis_error_rad="
                    << posture.target_axis_error << std::endl;
        }
      }
      if (log_step) {
        const Eigen::Vector2d normal_W = R_WO * approach_sample->normal;
        const double gap = normal_W.dot(live_tip.head<2>() - object_xy) -
            normal_W.dot(R_WO * approach_sample->point) -
            params.controller.pusher_radius;
        std::cout << "live_sampled_c3plus=PASS step=" << step_index
                  << " phase=" << approach_phase
                  << " contact=" << approach_sample->name
                  << " gap_m=" << gap
                  << " approach_error_m=" << approach_error.norm()
                  << " target_W=" << task_target.transpose() << std::endl;
      }
      return task_target;
    }
    publish_moving_execution_trajectory = true;

    if (!approach_sample_index.has_value()) {
      throw std::runtime_error("contact mode is missing its active T face");
    }
    const ContactFace active_face = samples[*approach_sample_index].face;
    // Keep the exact physical contact sample latched during C3.  Reduced-model
    // lateral predictions choose future faces, while the measured guard below
    // protects execution from model mismatch without hopping among points on
    // the current face.
    CandidateSelection active = EvaluateExactContactSample(
        params, live_tip.head<2>(), object_pose, *approach_sample_index, false);
    CandidateSelection successor = SelectContactCandidate(
        params, live_tip.head<2>(), object_pose, std::nullopt, active_face,
        true);
    if (successor.sample_index >= 0) {
      const Eigen::Vector2d successor_normal_W =
          R_WO * samples[successor.sample_index].normal;
      if (successor_normal_W.dot(stick_axis_W) > 1.0e-3) {
        successor = CandidateSelection{};
      }
    }
    const auto measured_contact_capsule = EvaluateContactCapsuleClearance(
        params, live_tip, stick_axis_3d_W, object_pose,
        samples[*approach_sample_index]);
    const bool contact_capsule_clearance_rejection =
        contact_capsule_command_rejected ||
        !measured_contact_capsule.clear();
    bool contact_capsule_staging_lift = false;
    if (contact_capsule_clearance_rejection) {
      CandidateSelection capsule_safe_successor =
          SelectTaskProgressCandidate(
              params, object_pose, active_face, stick_axis_W);
      if (!capsule_safe_successor.solve.finite) {
        capsule_safe_successor = SelectTaskProgressCandidate(
            params, object_pose, active_face, Eigen::Vector2d::Zero());
        contact_capsule_staging_lift =
            capsule_safe_successor.solve.finite;
      }
      if (capsule_safe_successor.solve.finite) {
        successor = capsule_safe_successor;
      }
    }
    const double measured_lateral_error =
        std::abs(object_pose.x() - params.object.goal_pose.x());
    const double measured_lateral_guard_threshold = std::max(
        0.0, params.controller.lateral_drift_tolerance -
                 params.controller.contact_activation_tolerance);
    const double minimum_measured_lateral_recovery =
        0.5 * measured_lateral_guard_threshold;
    const int required_productive_recovery_dwell_steps =
        completed_lateral_recoveries == 0
            ? params.controller.successor_minimum_contact_steps
            : 0;
    const bool measured_lateral_at_guard =
        measured_lateral_error >= measured_lateral_guard_threshold;
    const Eigen::Vector2d active_force_direction =
        -(R_WO * samples[active.sample_index].normal);
    const bool awaiting_multiface_cycle_acceptance =
        !diagnostic_overrides_enabled &&
        multiface_cycle_pose_start.has_value() &&
        completed_repositions > multiface_cycle_start_repositions &&
        successor_geometric_contact_reported;
    const bool translation_dominant_acceptance_grace =
        awaiting_multiface_cycle_acceptance &&
        std::abs(active_force_direction.y()) >
            std::abs(active_force_direction.x()) &&
        measured_lateral_error < params.controller.lateral_drift_tolerance;
    const double measured_lateral_signed_error =
        params.object.goal_pose.x() - object_pose.x();
    const bool active_contact_corrects_measured_lateral_error =
        measured_lateral_signed_error * active_force_direction.x() > 0.0;
    const bool measured_lateral_recovery_active =
        active_face_requires_lateral_recovery &&
        successor_geometric_contact_reported &&
        lateral_recovery_start_error.has_value();
    if (measured_lateral_recovery_active) {
      lateral_recovery_best_error =
          std::min(lateral_recovery_best_error, measured_lateral_error);
      if (!lateral_recovery_corridor_reached &&
          lateral_recovery_entry_force_correct &&
          measured_lateral_error < measured_lateral_guard_threshold &&
          *lateral_recovery_start_error - lateral_recovery_best_error >=
              minimum_measured_lateral_recovery) {
        lateral_recovery_corridor_reached = true;
        std::cout << "measured_lateral_recovery=CORRIDOR step="
                  << step_index
                  << " contact=" << samples[active.sample_index].name
                  << " start_error_m=" << *lateral_recovery_start_error
                  << " best_error_m=" << lateral_recovery_best_error
                  << " required_reduction_m="
                  << minimum_measured_lateral_recovery
                  << " signed_error_m=" << measured_lateral_signed_error
                  << std::endl;
      }
    }
    const bool measured_lateral_recovery_grace =
        measured_lateral_recovery_active &&
        (active_contact_corrects_measured_lateral_error ||
         lateral_recovery_corridor_reached);
    bool measured_lateral_guard_switch = false;
    bool measured_lateral_guard_staging_lift = false;
    bool measured_lateral_inline_recovery_start = false;
    if (measured_lateral_at_guard &&
        !translation_dominant_acceptance_grace &&
        !measured_lateral_recovery_grace) {
      CandidateSelection corrective =
          SelectMeasuredLateralCorrectiveCandidate(
              params, object_pose, active_face, stick_axis_W);
      if (corrective.solve.finite) {
        successor = corrective;
        measured_lateral_guard_switch = true;
      } else if (!active_face_requires_lateral_recovery &&
                 active_contact_corrects_measured_lateral_error &&
                 active.solve.gap <=
                     params.controller.contact_activation_tolerance) {
        // The six-DOF arm cannot always turn the pusher shaft around to the
        // high-authority side face while retaining the tip waypoint. Keep the
        // already physical, correctly oriented face and condition its
        // tangential command from measured x instead. This is a real contact
        // recovery mode: it receives the same measured-reduction and dwell
        // gates below and is not a geometric success shortcut.
        active_face_requires_lateral_recovery = true;
        lateral_recovery_uses_inline_fallback = true;
        successor_geometric_contact_reported = true;
        successor_contact_pose_start = object_pose;
        successor_contact_start_step_on_face = contact_steps_on_face;
        lateral_recovery_start_error = measured_lateral_error;
        lateral_recovery_start_signed_error =
            measured_lateral_signed_error;
        lateral_recovery_tip_start = live_tip.head<2>();
        lateral_recovery_best_error = measured_lateral_error;
        lateral_recovery_corridor_reached = false;
        lateral_recovery_receipt_reported = false;
        lateral_recovery_entry_force_correct = true;
        measured_lateral_inline_recovery_start = true;
        std::cout << "productive_corrective_face_dwell=START step="
                  << step_index
                  << " contact=" << samples[active.sample_index].name
                  << " mode=measured_tangential_fallback"
                  << " error_m=" << measured_lateral_error
                  << " signed_error_m=" << measured_lateral_signed_error
                  << " active_force_x=" << active_force_direction.x()
                  << " force_corrects=1"
                  << " required_reduction_m="
                  << minimum_measured_lateral_recovery
                  << " required_dwell_steps="
                  << required_productive_recovery_dwell_steps
                  << std::endl;
      } else {
        // At the low contact posture the high-authority lateral face can be
        // blocked by the inclined shaft even though a vertical release is
        // safe. Select that same measured-x face without the low-posture shaft
        // filter to initiate the lift; kRepositionTraverse must revalidate the
        // capsule and tip before any lateral motion or descent.
        CandidateSelection staging =
            SelectMeasuredLateralCorrectiveCandidate(
                params, object_pose, active_face, Eigen::Vector2d::Zero());
        if (staging.solve.finite) {
          successor = staging;
          measured_lateral_guard_switch = true;
          measured_lateral_guard_staging_lift = true;
        }
      }
    }
    if (FLAGS_force_successor_after_contact_steps > 0 &&
        completed_repositions == 0 &&
        FLAGS_diagnostic_successor_sample_index >= 0) {
      if (FLAGS_diagnostic_successor_sample_index >=
          static_cast<int>(samples.size())) {
        throw std::runtime_error(
            "diagnostic_successor_sample_index is out of range");
      }
      const int i = FLAGS_diagnostic_successor_sample_index;
      if (samples[i].face == active_face) {
        throw std::runtime_error(
            "diagnostic successor must be on a different T face");
      }
      successor = EvaluateExactContactSample(
          params, Eigen::Vector2d::Zero(), object_pose, i, true);
      if (!successor.solve.finite) {
        throw std::runtime_error("diagnostic successor C3+ solve failed");
      }
      measured_lateral_guard_switch = false;
    }
    ++contact_steps_on_face;
    const double current_yaw_error = std::abs(WrappedAngleError(
        params.object.goal_pose.z(), yaw));
    const bool progress_window_complete =
        contact_steps_on_face - progress_window_start_step >=
        params.controller.successor_progress_window_steps;
    const double window_yaw_progress =
        progress_window_start_yaw_error - current_yaw_error;
    const double current_y_error =
        std::abs(object_pose.y() - params.object.goal_pose.y());
    const double window_y_progress =
        progress_window_start_y_error - current_y_error;
    const bool yaw_progress_deficient =
        window_yaw_progress <
            params.controller.successor_minimum_yaw_progress;
    const bool translation_progress_deficient =
        window_y_progress <
            params.controller.successor_minimum_translation_progress;
    const bool insufficient_progress =
        progress_window_complete &&
        (yaw_progress_deficient || translation_progress_deficient);
    bool task_progress_reselection = false;
    bool progress_watchdog_staging_lift = false;
    if (insufficient_progress) {
      CandidateSelection progress_candidate = SelectTaskProgressCandidate(
          params, object_pose, active_face, stick_axis_W);
      if (!progress_candidate.solve.finite) {
        // A low inclined shaft can make every lateral successor unsafe before
        // release.  A vertical lift is still safe, so choose the task-progress
        // face without the low-posture shaft filter and revalidate it at the
        // elevated traverse pose before any lateral motion.
        progress_candidate = SelectTaskProgressCandidate(
            params, object_pose, active_face, Eigen::Vector2d::Zero());
        progress_watchdog_staging_lift = progress_candidate.solve.finite;
      }
      if (progress_candidate.solve.finite) {
        successor = progress_candidate;
        task_progress_reselection = true;
      }
    }
    // Recovery grace must not override the configured progress watchdog
    // indefinitely, whether the lateral error is inside or outside the 5 mm
    // acceptance bound.  Once a full watchdog window is deficient and a
    // task-progress face is available, preserve the failed recovery receipt
    // and release to that face.
    const bool stalled_lateral_recovery =
        measured_lateral_recovery_active && insufficient_progress &&
        !lateral_recovery_corridor_reached &&
        task_progress_reselection && successor.solve.finite;
    const int successor_contact_dwell = successor_geometric_contact_reported
        ? contact_steps_on_face - successor_contact_start_step_on_face
        : 0;
    const bool measured_wrong_polarity_response =
        measured_lateral_recovery_active &&
        MeasuredResponseHasWrongPolarity(
            true, successor_contact_dwell,
            params.controller.successor_minimum_contact_steps,
            measured_lateral_error, *lateral_recovery_start_error,
            minimum_measured_lateral_recovery);
    bool wrong_polarity_staging_lift = false;
    if (measured_wrong_polarity_response) {
      CandidateSelection measured_response_successor =
          SelectMeasuredLateralCorrectiveCandidate(
              params, object_pose, active_face, stick_axis_W);
      if (!measured_response_successor.solve.finite) {
        measured_response_successor =
            SelectMeasuredLateralCorrectiveCandidate(
                params, object_pose, active_face, Eigen::Vector2d::Zero());
        wrong_polarity_staging_lift =
            measured_response_successor.solve.finite;
      }
      if (measured_response_successor.solve.finite) {
        successor = measured_response_successor;
      }
    }
    const bool productive_lateral_recovery_complete =
        measured_lateral_recovery_active &&
        lateral_recovery_corridor_reached &&
        successor_contact_dwell >=
            required_productive_recovery_dwell_steps &&
        *lateral_recovery_start_error - lateral_recovery_best_error >=
            minimum_measured_lateral_recovery &&
        measured_lateral_error < params.controller.lateral_drift_tolerance;
    const bool resume_productive_inline_face =
        productive_lateral_recovery_complete &&
        lateral_recovery_uses_inline_fallback && !insufficient_progress;
    bool productive_lateral_recovery_switch = false;
    if (productive_lateral_recovery_complete) {
      if (!lateral_recovery_receipt_reported) {
        lateral_recovery_receipt_reported = true;
        ++completed_lateral_recoveries;
        std::cout << "productive_corrective_face_dwell=PASS step="
                  << step_index
                  << " contact=" << samples[active.sample_index].name
                  << " recovery=" << completed_lateral_recoveries
                  << " dwell_steps=" << successor_contact_dwell
                  << " start_error_m=" << *lateral_recovery_start_error
                  << " start_signed_error_m="
                  << *lateral_recovery_start_signed_error
                  << " best_error_m=" << lateral_recovery_best_error
                  << " final_error_m=" << measured_lateral_error
                  << " measured_reduction_m="
                  << *lateral_recovery_start_error -
                         lateral_recovery_best_error
                  << " required_reduction_m="
                  << minimum_measured_lateral_recovery
                  << " final_within_lateral_tolerance=1"
                  << std::endl;
      }
      if (resume_productive_inline_face) {
        std::cout << "productive_lateral_recovery_resume=PASS step="
                  << step_index
                  << " contact=" << samples[active.sample_index].name
                  << " recovery=" << completed_lateral_recoveries
                  << " measured_lateral_error_m="
                  << measured_lateral_error
                  << " progress_watchdog_deficient=0" << std::endl;
        active_face_requires_lateral_recovery = false;
        lateral_recovery_uses_inline_fallback = false;
        lateral_recovery_start_error.reset();
        lateral_recovery_start_signed_error.reset();
        lateral_recovery_tip_start.reset();
        lateral_recovery_best_error =
            std::numeric_limits<double>::infinity();
        lateral_recovery_corridor_reached = false;
        lateral_recovery_receipt_reported = false;
        lateral_recovery_entry_force_correct = false;
      } else {
        CandidateSelection progress_candidate = SelectTaskProgressCandidate(
            params, object_pose, active_face, stick_axis_W);
        if (!progress_candidate.solve.finite) {
          progress_candidate = SelectTaskProgressCandidate(
              params, object_pose, active_face, Eigen::Vector2d::Zero());
        }
        if (progress_candidate.solve.finite) {
          successor = progress_candidate;
          productive_lateral_recovery_switch = true;
        }
      }
    }
    if (progress_window_complete) {
      std::cout << "task_progress_watchdog="
                << (insufficient_progress ?
                        (task_progress_reselection ? "RESELECT" : "BLOCKED")
                        : "PASS")
                << " step=" << step_index
                << " contact=" << samples[active.sample_index].name
                << " window_steps="
                << contact_steps_on_face - progress_window_start_step
                << " negative_y_progress_m=" << window_y_progress
                << " yaw_progress_rad=" << window_yaw_progress
                << " translation_deficient="
                << translation_progress_deficient
                << " yaw_deficient=" << yaw_progress_deficient
                << " staging_lift=" << progress_watchdog_staging_lift
                << std::endl;
    }
    const bool successor_wins_hysteresis =
        successor.solve.finite && active.solve.finite &&
        successor.cost < active.cost *
            (1.0 - params.controller.successor_cost_hysteresis_fraction);
    const bool minimum_contact_complete = completed_repositions == 0
        ? contact_steps_on_face >=
              params.controller.successor_minimum_contact_steps
        : successor_geometric_contact_reported &&
              contact_steps_on_face - successor_contact_start_step_on_face >=
                  params.controller.successor_minimum_contact_steps;

    // Every autonomous multi-face cycle requires more than reaching the
    // clearance pose or briefly reaching successor planar geometry.  Retain
    // the face for a full interval and require measured progress from the
    // previous accepted cycle boundary.  xarm6_sim independently reports the
    // corresponding Drake capsule--T force episodes.  Diagnostic overrides
    // never emit this receipt.
    if (awaiting_multiface_cycle_acceptance &&
        successor_contact_pose_start.has_value() &&
        multiface_cycle_pose_start.has_value()) {
      const int continuation_steps =
          contact_steps_on_face - successor_contact_start_step_on_face;
      const double y_error_start = std::abs(
          multiface_cycle_pose_start->y() - params.object.goal_pose.y());
      const double y_progress = y_error_start - current_y_error;
      const double yaw_error_start = std::abs(WrappedAngleError(
          params.object.goal_pose.z(), multiface_cycle_pose_start->z()));
      const double yaw_progress = yaw_error_start - current_yaw_error;
      if (log_step) {
        std::cout << "autonomous_multiface_progress step=" << step_index
                  << " cycle=" << accepted_multiface_cycles + 1
                  << " contact=" << samples[active.sample_index].name
                  << " continuation_steps=" << continuation_steps
                  << " negative_y_progress_m=" << y_progress
                  << " yaw_progress_rad=" << yaw_progress
                  << " measured_lateral_error_m=" << measured_lateral_error
                  << " translation_guard_grace="
                  << translation_dominant_acceptance_grace
                  << " lateral_recovery_grace="
                  << measured_lateral_recovery_grace
                  << std::endl;
      }
      if (continuation_steps >=
              params.controller.successor_minimum_contact_steps &&
          y_progress >=
              params.controller.successor_minimum_translation_progress &&
          yaw_progress >=
              params.controller.successor_minimum_yaw_progress &&
          measured_lateral_error <=
              params.controller.lateral_drift_tolerance) {
        ++accepted_multiface_cycles;
        std::cout << "autonomous_multiface_continuation=PASS step="
                  << step_index << " cycle=" << accepted_multiface_cycles
                  << " contact="
                  << samples[active.sample_index].name
                  << " completed_repositions=" << completed_repositions
                  << " continuation_steps=" << continuation_steps
                  << " negative_y_progress_m=" << y_progress
                  << " yaw_progress_rad=" << yaw_progress
                  << " measured_lateral_error_m=" << measured_lateral_error
                  << std::endl;
        std::cout << "autonomous_multiface_cycle=PASS cycle="
                  << accepted_multiface_cycles
                  << " step=" << step_index
                  << " completed_repositions=" << completed_repositions
                  << " negative_y_progress_m=" << y_progress
                  << " yaw_progress_rad=" << yaw_progress
                  << " measured_lateral_error_m=" << measured_lateral_error
                  << std::endl;
        multiface_cycle_pose_start = object_pose;
        multiface_cycle_start_repositions = completed_repositions;
      }
    }
    const bool diagnostic_forced_switch =
        FLAGS_force_successor_after_contact_steps > 0 &&
        completed_repositions == 0 &&
        contact_steps_on_face >= FLAGS_force_successor_after_contact_steps;
    // A completed geometric reposition is not a physical contact.  Keep the
    // selected successor active until its measured reduced-model gap reaches
    // the existing activation tolerance and completes the configured physical
    // dwell; otherwise the lateral guard can immediately abandon a
    // task-progress face after Drake reports force but before that face has a
    // meaningful opportunity to move the object.  Productive or stalled
    // recovery decisions below remain explicit higher-priority exits.
    const bool hold_successor_until_contact = completed_repositions > 0 &&
        (!successor_geometric_contact_reported ||
         successor_contact_dwell <
             params.controller.successor_minimum_contact_steps);
    const bool hold_successor_until_cycle_acceptance =
        awaiting_multiface_cycle_acceptance;
    const bool should_reposition =
        successor.solve.finite &&
        (productive_lateral_recovery_switch ||
         stalled_lateral_recovery ||
         measured_wrong_polarity_response ||
         contact_capsule_clearance_rejection ||
         (!hold_successor_until_contact &&
        ((!active.solve.finite) ||
         diagnostic_forced_switch ||
         measured_lateral_guard_switch ||
          ((!measured_lateral_at_guard ||
            translation_dominant_acceptance_grace) &&
          minimum_contact_complete &&
          (insufficient_progress ||
           (!hold_successor_until_cycle_acceptance &&
            successor_wins_hysteresis))))));
    if (should_reposition) {
      const auto& successor_sample = samples[successor.sample_index];
      const double successor_shaft_clearance =
          -(R_WO * successor_sample.normal).dot(stick_axis_W);
      const char* switch_reason = "cost_hysteresis";
      if (contact_capsule_clearance_rejection) {
        switch_reason = "contact_capsule_clearance";
      } else if (measured_wrong_polarity_response) {
        switch_reason = "measured_wrong_polarity";
      } else if (!active.solve.finite) {
        switch_reason = "active_invalid";
      } else if (diagnostic_forced_switch) {
        switch_reason = "diagnostic_forced";
      } else if (stalled_lateral_recovery) {
        switch_reason = "stalled_lateral_recovery";
      } else if (productive_lateral_recovery_switch) {
        switch_reason = "productive_lateral_recovery";
      } else if (measured_lateral_guard_switch) {
        switch_reason = "measured_lateral_guard";
      } else if (insufficient_progress) {
        switch_reason = "insufficient_progress";
      }
      std::cout << "successor_face_switch=PASS step=" << step_index
                << " from=" << samples[*approach_sample_index].name
                << " to=" << successor_sample.name
                << " reason=" << switch_reason
                << " contact_steps=" << contact_steps_on_face
                << " acceptance_cycle=" << accepted_multiface_cycles + 1
                << " negative_y_progress_m=" << window_y_progress
                << " yaw_progress_rad=" << window_yaw_progress
                << " measured_lateral_error_m=" << measured_lateral_error
                << " measured_lateral_guard_m="
                << measured_lateral_guard_threshold
                << " measured_lateral_at_guard="
                << measured_lateral_at_guard
                << " measured_lateral_guard_staging_lift="
                << measured_lateral_guard_staging_lift
                << " measured_lateral_inline_recovery_start="
                << measured_lateral_inline_recovery_start
                << " lateral_recovery_grace="
                << measured_lateral_recovery_grace
                << " completed_lateral_recoveries="
                << completed_lateral_recoveries
                << " task_progress_reselection="
                << task_progress_reselection
                << " contact_capsule_command_rejected="
                << contact_capsule_command_rejected
                << " contact_capsule_shaft_t_clear="
                << measured_contact_capsule.shaft_t_clear
                << " contact_capsule_table_clear="
                << measured_contact_capsule.capsule_table_clear
                << " contact_capsule_table_margin_m="
                << measured_contact_capsule.capsule_table_margin
                << " contact_capsule_staging_lift="
                << contact_capsule_staging_lift
                << " wrong_polarity_staging_lift="
                << wrong_polarity_staging_lift
                << " stick_axis_xy=" << stick_axis_W.transpose()
                << " successor_shaft_clearance="
                << successor_shaft_clearance
                << " active_cost=" << active.cost
                << " successor_cost=" << successor.cost << std::endl;
      if (measured_wrong_polarity_response) {
        std::cout << "measured_wrong_polarity_response=REJECT step="
                  << step_index
                  << " contact=" << samples[*approach_sample_index].name
                  << " dwell_steps=" << successor_contact_dwell
                  << " start_error_m=" << *lateral_recovery_start_error
                  << " final_error_m=" << measured_lateral_error
                  << " wrong_direction_growth_m="
                  << measured_lateral_error - *lateral_recovery_start_error
                  << " rejection_threshold_m="
                  << minimum_measured_lateral_recovery << std::endl;
      }
      if (contact_capsule_clearance_rejection) {
        std::cout << "contact_phase_capsule_clearance=REJECT step="
                  << step_index
                  << " contact=" << samples[*approach_sample_index].name
                  << " measured_shaft_t_clear="
                  << measured_contact_capsule.shaft_t_clear
                  << " measured_capsule_table_clear="
                  << measured_contact_capsule.capsule_table_clear
                  << " measured_tip_table_clear="
                  << measured_contact_capsule.tip_table_clear
                  << " command_rejected="
                  << contact_capsule_command_rejected << std::endl;
      }
      if (lateral_recovery_start_error.has_value()) {
        if (!lateral_recovery_receipt_reported) {
          std::cout << "productive_corrective_face_dwell=ABORT step="
                    << step_index
                    << " contact=" << samples[*approach_sample_index].name
                    << " dwell_steps=" << successor_contact_dwell
                    << " start_error_m=" << *lateral_recovery_start_error
                    << " best_error_m=" << lateral_recovery_best_error
                    << " final_error_m=" << measured_lateral_error
                    << " active_force_x=" << active_force_direction.x()
                    << " force_corrects="
                    << active_contact_corrects_measured_lateral_error
                    << std::endl;
        }
      }
      // Break unilateral contact along the current face normal before any
      // vertical motion.  A direct lift can sweep the inclined capsule across
      // the T top even when the tip itself moves upward.  Reuse the configured
      // free-space descent clearance as a deterministic outward release.
      const Eigen::Vector2d release_normal_W =
          R_WO * samples[*approach_sample_index].normal;
      approach_sample_index = successor.sample_index;
      approach_contact_name = successor_sample.name;
      reposition_lift_xy = live_tip.head<2>() +
          release_normal_W * params.controller.descent_clearance;
      reposition_object_pose_start = object_pose;
      approach_phase_state = ApproachPhase::kRepositionRetreat;
      contact_capsule_command_rejected = false;
      reposition_orient_start_step.reset();
      ++completed_repositions;
      successor_geometric_contact_reported = false;
      successor_contact_pose_start.reset();
      successor_contact_start_step_on_face = 0;
      active_face_requires_lateral_recovery =
          measured_lateral_guard_switch ||
          measured_lateral_at_guard;
      lateral_recovery_uses_inline_fallback = false;
      lateral_recovery_start_error.reset();
      lateral_recovery_start_signed_error.reset();
      lateral_recovery_tip_start.reset();
      lateral_recovery_best_error =
          std::numeric_limits<double>::infinity();
      lateral_recovery_corridor_reached = false;
      lateral_recovery_receipt_reported = false;
      lateral_recovery_entry_force_correct = false;
      reposition_min_traverse_z = std::numeric_limits<double>::infinity();
      reposition_collision_receipt = RepositionCollisionReceipt{};
      Eigen::Vector3d task_target = live_tip;
      const double reposition_step_limit =
          params.controller.reposition_speed * params.task.planning_time_step;
      Eigen::Vector2d release_step = *reposition_lift_xy -
          live_tip.head<2>();
      if (release_step.norm() > reposition_step_limit) {
        release_step *= reposition_step_limit / release_step.norm();
      }
      task_target.head<2>() += release_step;
      publish_moving_execution_trajectory = true;
      const auto release_posture = SolveLiftPostureStep(
          plant, context.get(), params, live_controlled_positions,
          task_target);
      if (!release_posture.has_value()) {
        throw std::runtime_error(
            "bounded six-DOF outward-release IK failed");
      }
      reposition_posture_target = *release_posture;
      return task_target;
    }
    if (progress_window_complete) {
      progress_window_start_step = contact_steps_on_face;
      progress_window_start_yaw_error = current_yaw_error;
      progress_window_start_y_error = current_y_error;
    }
    if (!active.solve.finite) {
      throw std::runtime_error(
          "active T face failed and no valid successor face is available");
    }
    if (completed_repositions > 0 &&
        !successor_geometric_contact_reported &&
        active.solve.gap <= params.controller.contact_activation_tolerance) {
      successor_geometric_contact_reported = true;
      successor_contact_pose_start = object_pose;
      successor_contact_start_step_on_face = contact_steps_on_face;
      std::cout << "successor_geometric_contact=PASS step=" << step_index
                << " contact=" << samples[active.sample_index].name
                << " gap_m=" << active.solve.gap
                << " acceptance_cycle=" << accepted_multiface_cycles + 1
                << " completed=" << completed_repositions << std::endl;
      if (active_face_requires_lateral_recovery) {
        lateral_recovery_uses_inline_fallback = false;
        lateral_recovery_start_error = measured_lateral_error;
        lateral_recovery_start_signed_error =
            measured_lateral_signed_error;
        lateral_recovery_tip_start = live_tip.head<2>();
        lateral_recovery_best_error = measured_lateral_error;
        lateral_recovery_corridor_reached = false;
        lateral_recovery_receipt_reported = false;
        lateral_recovery_entry_force_correct =
            active_contact_corrects_measured_lateral_error;
        std::cout << "productive_corrective_face_dwell=START step="
                  << step_index
                  << " contact=" << samples[active.sample_index].name
                  << " error_m=" << measured_lateral_error
                  << " signed_error_m=" << measured_lateral_signed_error
                  << " active_force_x=" << active_force_direction.x()
                  << " force_corrects="
                  << active_contact_corrects_measured_lateral_error
                  << " required_reduction_m="
                  << minimum_measured_lateral_recovery
                  << " required_dwell_steps="
                  << required_productive_recovery_dwell_steps
                  << std::endl;
      }
    }
    const auto& best_sample = samples[active.sample_index];
    const Eigen::Vector2d best_normal_W = R_WO * best_sample.normal;
    const double current_lateral_error =
        std::abs(object_xy.x() - params.object.goal_pose.x());
    const double allowed_lateral_error = std::max(
        current_lateral_error, params.controller.lateral_drift_tolerance);
    Eigen::Vector2d step = active.solve.pusher_target - live_tip.head<2>();
    const Eigen::Vector2d inward_direction = -best_normal_W;
    const Eigen::Vector2d tangent_direction(
        -best_normal_W.y(), best_normal_W.x());
    const Eigen::Vector2d boundary_center_W =
        object_xy + R_WO * best_sample.point;
    const double tangential_error = tangent_direction.dot(
        boundary_center_W - live_tip.head<2>());
    // SolveOneContact linearizes a half-space and can otherwise command a
    // large tangential move beyond the finite T face. Keep the exact sample
    // latched by replacing only that tangential component; the planned normal
    // component and configured inward-loading floor remain unchanged.
    step += (tangential_error - step.dot(tangent_direction)) *
        tangent_direction;
    // Recovery state can complete and resume the same face earlier in this
    // control update. Re-evaluate it here instead of using the pre-transition
    // snapshot that drives selection and receipt logging above.
    const bool execute_measured_lateral_recovery =
        active_face_requires_lateral_recovery &&
        successor_geometric_contact_reported &&
        lateral_recovery_start_error.has_value();
    double commanded_contact_normal_step = contact_normal_step;
    if (execute_measured_lateral_recovery &&
        lateral_recovery_corridor_reached) {
      // The measured guard corridor is the corrective face's destination.
      // Freeze Cartesian advance through the one full proof dwell so normal
      // rotation cannot erase recovery. Later independently measured inline
      // recoveries resume immediately and do not spend another proof dwell.
      step.setZero();
      commanded_contact_normal_step = 0.0;
    } else {
      if (execute_measured_lateral_recovery) {
        if (!lateral_recovery_tip_start.has_value() ||
            !lateral_recovery_start_signed_error.has_value()) {
          throw std::runtime_error(
              "measured lateral recovery is missing its latched tip target");
        }
        // Bound recovery to one existing task-space step from the measured
        // entry tip. Repeated live-relative offsets would integrate across
        // updates while the 20 Hz object measurement lags and can leave the
        // finite T face before recovery is observed.
        const double recovery_target_x = lateral_recovery_tip_start->x() +
            std::copysign(params.controller.task_space_plan_step_limit,
                          *lateral_recovery_start_signed_error);
        step.x() = std::clamp(
            recovery_target_x - live_tip.x(),
            -params.controller.task_space_plan_step_limit,
            params.controller.task_space_plan_step_limit);
      }
      const double inward_step = step.dot(inward_direction);
      if (execute_measured_lateral_recovery && active.solve.gap <= 0.0) {
        // Once the corrective face is physically loaded, preserve its current
        // normal compression. Reapplying the loading floor every update drove
        // the centerline through the selected-face activation band before the
        // measured tangential recovery could act. Project the command onto
        // the face tangent instead; the existing compliance maintains force
        // and the whole-capsule predicate remains authoritative.
        step -= inward_step * inward_direction;
        commanded_contact_normal_step = 0.0;
      } else if (inward_step < contact_normal_step) {
        step += (contact_normal_step - inward_step) * inward_direction;
      }
    }
    if (step.norm() > params.controller.task_space_plan_step_limit) {
      step *= params.controller.task_space_plan_step_limit / step.norm();
    }
    Eigen::Vector3d task_target = live_tip;
    task_target.head<2>() = live_tip.head<2>() + step;
    // Keep the contact command on the object's planar pushing height.  Feeding
    // the measured tip height back into the next command lets vertical contact
    // reactions ratchet the target upward; the pusher can then pass over the T
    // while the reduced planar model still reports penetration and rotation.
    task_target.z() = object_z;
    const auto commanded_contact_capsule =
        EvaluateSweptContactCapsuleClearance(
            params, live_tip, task_target, stick_axis_3d_W,
            stick_axis_3d_W, object_pose, best_sample);
    if (commanded_contact_capsule.clear()) {
      contact_capsule_command_rejected = false;
    } else {
      // Hold one update at the last measured safe state. The latched rejection
      // is consumed by the next solve and takes the normal reposition path,
      // so an unsafe target is never published to OSC.
      task_target = live_tip;
      contact_capsule_command_rejected = true;
      std::cout << "contact_phase_capsule_command=REJECT step="
                << step_index << " contact=" << best_sample.name
                << " shaft_t_clear="
                << commanded_contact_capsule.shaft_t_clear
                << " capsule_table_clear="
                << commanded_contact_capsule.capsule_table_clear
                << " tip_table_clear="
                << commanded_contact_capsule.tip_table_clear
                << " capsule_table_margin_m="
                << commanded_contact_capsule.capsule_table_margin
                << std::endl;
    }
    if (log_step) {
      std::cout << "live_sampled_c3plus=PASS step=" << step_index
                << " phase=contact contact=" << best_sample.name
                << " gap_m=" << active.solve.gap
                << " predicted_object_xy_yaw="
                << active.solve.object_prediction.transpose()
                << " active_lateral_rejections="
                << active.lateral_rejections << "/" << active.finite_samples
                << " successor="
                << (successor.sample_index >= 0
                        ? samples[successor.sample_index].name
                        : "none")
                << " successor_lateral_rejections="
                << successor.lateral_rejections << "/"
                << successor.finite_samples
                << " active_cost=" << active.cost
                << " successor_cost=" << successor.cost
                << " window_yaw_progress_rad=" << window_yaw_progress
                << " lateral_corridor_m=" << allowed_lateral_error
                << " measured_lateral_guard_m="
                << measured_lateral_guard_threshold
                << " contact_steps=" << contact_steps_on_face
                << " contact_normal_step_m="
                << commanded_contact_normal_step
                << " tangential_error_m=" << tangential_error
                << " contact_capsule_shaft_t_clear="
                << commanded_contact_capsule.shaft_t_clear
                << " target_W=" << task_target.transpose() << std::endl;
    }
    return task_target;
  };

  if (FLAGS_live_sampled_plan) {
    if (FLAGS_live_control_steps <= 0 || FLAGS_live_step_period_ms <= 0 ||
        FLAGS_live_log_every < 0) {
      throw std::runtime_error("invalid live step count, period, or log rate");
    }
    const int publishes_per_step =
        std::max(1, FLAGS_live_step_period_ms / 10);
    for (int i = 0; i < FLAGS_live_control_steps; ++i) {
      publish_target(solve_live_target(i), publishes_per_step);
    }
  } else {
    publish_target(home_tip + Eigen::Vector3d(
                       FLAGS_smoke_offset_x, FLAGS_smoke_offset_y,
                       FLAGS_smoke_offset_z),
                   FLAGS_publish_count);
  }
  return 0;
}

}  // namespace dairlib::oim

int main(int argc, char* argv[]) { return dairlib::oim::DoMain(argc, argv); }
