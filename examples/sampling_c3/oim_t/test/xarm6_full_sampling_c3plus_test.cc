#include "examples/sampling_c3/oim_t/xarm6_full_sampling_c3plus.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include <gtest/gtest.h>

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/solve.h>

namespace dairlib::oim {
namespace {

TEST(XarmFullSamplingC3PlusTest, ConditionsMeasuredVelocityForOpenTableModel) {
  const Eigen::Vector3d angular_velocity_W(-0.4, 0.5, -0.6);
  const Eigen::Vector3d linear_velocity_W(0.7, -0.8, 0.9);

  const auto [conditioned_angular_W, conditioned_linear_W] =
      ConditionOpenTableObjectVelocity(angular_velocity_W,
                                       linear_velocity_W);

  EXPECT_TRUE(conditioned_angular_W.isApprox(
      Eigen::Vector3d(0.0, 0.0, -0.6)));
  EXPECT_TRUE(conditioned_linear_W.isApprox(
      Eigen::Vector3d(0.7, -0.8, 0.0)));
}

TEST(XarmFullSamplingC3PlusTest, PlanarSettleUsesMeasuredPoseStability) {
  const Eigen::Vector3d previous_position(0.381, 0.4, 0.0298);
  const Eigen::Quaterniond previous_orientation(
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d current_position(0.3815, 0.3995, 0.0300);
  const Eigen::Quaterniond current_orientation(
      Eigen::AngleAxisd(0.201, Eigen::Vector3d::UnitZ()));
  const auto settled = EvaluateXarmFullSamplingC3PlanarSettle(
      previous_position, previous_orientation, current_position,
      current_orientation, 0.0298, 0.002, 0.002, 0.003, 0.10);
  EXPECT_TRUE(settled.accepted);

  const Eigen::Quaterniond tilted(
      Eigen::AngleAxisd(0.11, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0.201, Eigen::Vector3d::UnitZ()));
  EXPECT_FALSE(EvaluateXarmFullSamplingC3PlanarSettle(
      previous_position, previous_orientation, current_position, tilted,
      0.0298, 0.002, 0.002, 0.003, 0.10).accepted);
  EXPECT_FALSE(EvaluateXarmFullSamplingC3PlanarSettle(
      previous_position, previous_orientation,
      Eigen::Vector3d(0.3815, 0.3995, 0.034), current_orientation,
      0.0298, 0.002, 0.002, 0.003, 0.10).accepted);
}

TEST(XarmFullSamplingC3PlusTest,
     ContactResponseRejectsPhysicalFallThroughImmediately) {
  EXPECT_TRUE(IsXarmFullSamplingC3ObjectUpright(
      Eigen::Vector3d(0.381, 0.2, 0.03), Eigen::Quaterniond::Identity(),
      0.03, 0.003, 0.10));
  EXPECT_FALSE(IsXarmFullSamplingC3ObjectUpright(
      Eigen::Vector3d(0.381, 0.2, 0.03),
      Eigen::Quaterniond(Eigen::AngleAxisd(
          0.1001, Eigen::Vector3d::UnitX())),
      0.03, 0.003, 0.10));
  EXPECT_FALSE(IsXarmFullSamplingC3ObjectUpright(
      Eigen::Vector3d(0.381, 0.2, 0.0331), Eigen::Quaterniond::Identity(),
      0.03, 0.003, 0.10));
}

TEST(XarmFullSamplingC3PlusTest, LateralReserveKeepsActivationMargin) {
  EXPECT_DOUBLE_EQ(FullSamplingC3LateralReserveLimit(0.005, 0.003),
                   0.002);
  EXPECT_DOUBLE_EQ(FullSamplingC3LateralReserveLimit(0.002, 0.003), 0.0);
  EXPECT_THROW(FullSamplingC3LateralReserveLimit(-0.005, 0.003),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     RecoveryFaceOwnsReleaseAfterPhysicalClear) {
  EXPECT_TRUE(ShouldReplaceXarmFullSamplingC3ReleaseReferenceWithPrimary(
      false, false));
  EXPECT_FALSE(ShouldReplaceXarmFullSamplingC3ReleaseReferenceWithPrimary(
      false, true));
  EXPECT_FALSE(ShouldReplaceXarmFullSamplingC3ReleaseReferenceWithPrimary(
      true, false));
  EXPECT_FALSE(ShouldReplaceXarmFullSamplingC3ReleaseReferenceWithPrimary(
      true, true));
}

TEST(XarmFullSamplingC3PlusTest, CentralSideContactRejectsVerticalEdges) {
  EXPECT_TRUE(IsFullSamplingC3CentralSideContact(0.0, 0.0298, 0.00555));
  EXPECT_TRUE(IsFullSamplingC3CentralSideContact(0.00555, 0.0298, 0.00555));
  EXPECT_FALSE(IsFullSamplingC3CentralSideContact(0.00556, 0.0298, 0.00555));
  EXPECT_FALSE(IsFullSamplingC3CentralSideContact(0.0297, 0.0298, 0.00555));
  EXPECT_THROW(IsFullSamplingC3CentralSideContact(0.0, 0.004, 0.00555),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     PhysicalContactClassificationSeparatesTipSideAndGrazes) {
  const Eigen::Vector3d center(0.4, 0.2, 0.0298);
  const Eigen::Vector3d tip(0.4, 0.16, 0.0298);
  const auto tip_side = ClassifyXarmPhysicalContact(
      Eigen::Vector3d(0.4, 0.165, 0.032), tip, center, 0.00555, 0.003);
  EXPECT_EQ(tip_side.contact_class,
            XarmPhysicalContactClass::kTipSideCandidate);
  EXPECT_DOUBLE_EQ(tip_side.relative_height_m, 0.032 - 0.0298);
  EXPECT_NEAR(tip_side.point_to_tip_m,
              (Eigen::Vector3d(0.4, 0.165, 0.032) - tip).norm(), 1e-15);
  EXPECT_STREQ(XarmPhysicalContactClassName(tip_side.contact_class),
               "tip_side_candidate");

  const auto shaft = ClassifyXarmPhysicalContact(
      Eigen::Vector3d(0.4, 0.2, 0.0298), tip, center, 0.00555, 0.003);
  EXPECT_EQ(shaft.contact_class, XarmPhysicalContactClass::kSideShaft);
  const auto top = ClassifyXarmPhysicalContact(
      Eigen::Vector3d(0.4, 0.2, 0.0595), tip, center, 0.00555, 0.003);
  EXPECT_EQ(top.contact_class, XarmPhysicalContactClass::kTopGraze);
  const auto bottom = ClassifyXarmPhysicalContact(
      Eigen::Vector3d(0.4, 0.2, 0.020), tip, center, 0.00555, 0.003);
  EXPECT_EQ(bottom.contact_class, XarmPhysicalContactClass::kBottomGraze);
  EXPECT_THROW(ClassifyXarmPhysicalContact(
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero(), 0.0, 0.003),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     CorridorSafePrefixStopsBeforePredictedLateralViolation) {
  XarmFullSamplingC3CandidateReceipt candidate;
  candidate.sample_name = "synthetic_stem_right";
  for (int knot = 0; knot < 5; ++knot) {
    XarmFullSamplingC3State state;
    state.pusher_position_W = Eigen::Vector3d(0.4 + 0.001 * knot,
                                               0.3, 0.03);
    state.object_position_W = Eigen::Vector3d(
        0.381 + 0.002 * knot, 0.4 - 0.001 * knot, 0.0298);
    candidate.solve.dynamic_state_trajectory.push_back(state.Encode());
    if (knot > 0) {
      candidate.solve.planned_input_trajectory.push_back(
          Eigen::Vector3d::Zero());
    }
  }
  candidate.solve.dynamic_terminal_object_position =
      Eigen::Vector3d(0.389, 0.396, 0.0298);

  const auto prefix = BuildXarmFullSamplingC3CorridorSafePrefix(
      candidate, 0.381, 0.005);
  ASSERT_TRUE(prefix.accepted);
  EXPECT_EQ(prefix.original_state_knots, 5);
  EXPECT_EQ(prefix.retained_state_knots, 3);
  EXPECT_DOUBLE_EQ(prefix.terminal_lateral_error, 0.004);
  EXPECT_EQ(prefix.execution_candidate.solve.dynamic_state_trajectory.size(),
            3);
  EXPECT_EQ(prefix.execution_candidate.solve.planned_input_trajectory.size(),
            2);
  EXPECT_DOUBLE_EQ(
      prefix.execution_candidate.solve.dynamic_terminal_object_position.x(),
      0.385);
  EXPECT_EQ(candidate.solve.dynamic_state_trajectory.size(), 5);
  EXPECT_DOUBLE_EQ(candidate.solve.dynamic_terminal_object_position.x(),
                   0.389);
}

TEST(XarmFullSamplingC3PlusTest,
     CorridorSafePrefixRejectsUnsafeInitialStateAndOneKnotPlan) {
  XarmFullSamplingC3CandidateReceipt candidate;
  XarmFullSamplingC3State state;
  state.object_position_W = Eigen::Vector3d(0.387, 0.4, 0.0298);
  candidate.solve.dynamic_state_trajectory = {state.Encode(), state.Encode()};
  candidate.solve.planned_input_trajectory = {Eigen::Vector3d::Zero()};
  EXPECT_FALSE(BuildXarmFullSamplingC3CorridorSafePrefix(
                   candidate, 0.381, 0.005).accepted);

  state.object_position_W.x() = 0.381;
  candidate.solve.dynamic_state_trajectory.front() = state.Encode();
  EXPECT_FALSE(BuildXarmFullSamplingC3CorridorSafePrefix(
                   candidate, 0.381, 0.005).accepted);
  EXPECT_FALSE(BuildXarmFullSamplingC3CorridorSafePrefix(
                   candidate, 0.381, -0.005).accepted);
}

TEST(XarmFullSamplingC3PlusTest,
     PredictedContactPrefixRequiresFaceGapAndHeightConformance) {
  XarmFullSamplingC3CandidateReceipt candidate;
  candidate.sample_point_O = Eigen::Vector2d::Zero();
  candidate.sample_normal_O = Eigen::Vector2d::UnitX();
  candidate.sample_height_O = 0.005;
  for (int knot = 0; knot < 2; ++knot) {
    XarmFullSamplingC3State state;
    state.object_position_W = Eigen::Vector3d(0.381, 0.4, 0.0298);
    state.pusher_position_W = Eigen::Vector3d(
        0.381 + 0.02 - 0.0015, 0.4, 0.0298 + 0.005);
    candidate.solve.dynamic_state_trajectory.push_back(state.Encode());
  }
  const auto accepted = EvaluateXarmFullSamplingC3PredictedContactPrefix(
      candidate, 0.02, 0.003);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_EQ(accepted.checked_state_knots, 2);
  EXPECT_NEAR(accepted.maximum_absolute_signed_gap, 0.0015, 1.0e-15);
  EXPECT_DOUBLE_EQ(accepted.maximum_separating_gap, 0.0);
  EXPECT_DOUBLE_EQ(accepted.maximum_contact_height_error, 0.0);

  auto lifted = candidate;
  auto lifted_state = XarmFullSamplingC3State::Decode(
      lifted.solve.dynamic_state_trajectory.back());
  lifted_state.pusher_position_W.z() += 0.01;
  lifted.solve.dynamic_state_trajectory.back() = lifted_state.Encode();
  const auto lifted_receipt =
      EvaluateXarmFullSamplingC3PredictedContactPrefix(
          lifted, 0.02, 0.003);
  EXPECT_FALSE(lifted_receipt.accepted);
  EXPECT_DOUBLE_EQ(lifted_receipt.maximum_contact_height_error, 0.01);

  auto separated = candidate;
  auto separated_state = XarmFullSamplingC3State::Decode(
      separated.solve.dynamic_state_trajectory.back());
  separated_state.pusher_position_W.x() += 0.01;
  separated.solve.dynamic_state_trajectory.back() = separated_state.Encode();
  const auto separated_receipt =
      EvaluateXarmFullSamplingC3PredictedContactPrefix(
          separated, 0.02, 0.003);
  EXPECT_FALSE(separated_receipt.accepted);
  EXPECT_GT(separated_receipt.maximum_absolute_signed_gap, 0.003);
  EXPECT_GT(separated_receipt.maximum_separating_gap, 0.003);

  auto inward = candidate;
  auto inward_state = XarmFullSamplingC3State::Decode(
      inward.solve.dynamic_state_trajectory.back());
  inward_state.pusher_position_W.x() -= 0.01;
  inward.solve.dynamic_state_trajectory.back() = inward_state.Encode();
  const auto inward_receipt =
      EvaluateXarmFullSamplingC3PredictedContactPrefix(
          inward, 0.02, 0.003);
  EXPECT_TRUE(inward_receipt.accepted);
  EXPECT_GT(inward_receipt.maximum_absolute_signed_gap, 0.003);
  EXPECT_DOUBLE_EQ(inward_receipt.maximum_separating_gap, 0.0);
}

TEST(XarmFullSamplingC3PlusTest, ContactCycleRequiresCompleteDwellBudget) {
  EXPECT_TRUE(HasFullSamplingC3ContactDwellBudget(623, 2000, 1000));
  EXPECT_FALSE(HasFullSamplingC3ContactDwellBudget(1000, 2000, 1000));
  EXPECT_FALSE(HasFullSamplingC3ContactDwellBudget(1937, 2000, 1000));
  EXPECT_THROW(HasFullSamplingC3ContactDwellBudget(2001, 2000, 1000),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     LatchedContactUsesActivationBandAsGeometricHysteresis) {
  XarmFullSamplingC3ContactDwellReceipt receipt;
  receipt.contact_gap_accepted = true;
  receipt.contact_continuation = false;
  EXPECT_FALSE(IsFullSamplingC3LatchedContactContinuation(false, receipt));
  EXPECT_TRUE(IsFullSamplingC3LatchedContactContinuation(true, receipt));
  receipt.contact_gap_accepted = false;
  EXPECT_FALSE(IsFullSamplingC3LatchedContactContinuation(true, receipt));
}

TEST(XarmFullSamplingC3PlusTest,
     PrimaryContactStopsBeforeConsumingRecoveryReserve) {
  EXPECT_FALSE(IsFullSamplingC3LateralReserveExhausted(
      0.3829, 0.381, 0.002));
  EXPECT_TRUE(IsFullSamplingC3LateralReserveExhausted(
      0.3831, 0.381, 0.002));
  EXPECT_THROW(IsFullSamplingC3LateralReserveExhausted(
                   0.381, 0.381, -0.002),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     InitialLateralGateIsAuthoritativeDuringCandidateAdmission) {
  EXPECT_TRUE(IsFullSamplingC3InitialCandidateAdmitted(true, true, true));
  EXPECT_FALSE(IsFullSamplingC3InitialCandidateAdmitted(false, true, true));
  EXPECT_FALSE(IsFullSamplingC3InitialCandidateAdmitted(true, false, true));
  EXPECT_FALSE(IsFullSamplingC3InitialCandidateAdmitted(true, true, false));
}

TEST(XarmFullSamplingC3PlusTest,
     InitialAdmissionMayUseOnlyTheCorridorSafeExecutionCopy) {
  XarmFullSamplingC3CandidateReceipt candidate;
  for (int knot = 0; knot < 5; ++knot) {
    XarmFullSamplingC3State state;
    state.pusher_position_W = Eigen::Vector3d(0.4 + 0.001 * knot,
                                               0.3, 0.03);
    state.object_position_W = Eigen::Vector3d(
        0.381 + 0.002 * knot, 0.4 - 0.001 * knot, 0.0298);
    candidate.solve.dynamic_state_trajectory.push_back(state.Encode());
    if (knot > 0) {
      candidate.solve.planned_input_trajectory.push_back(
          Eigen::Vector3d::Zero());
    }
  }
  candidate.solve.dynamic_terminal_object_position =
      Eigen::Vector3d(0.389, 0.396, 0.0298);
  EXPECT_FALSE(IsFullSamplingC3InitialCandidateAdmitted(
      std::abs(candidate.solve.dynamic_terminal_object_position.x() - 0.381)
          <= 0.005,
      true, true));

  const auto prefix = BuildXarmFullSamplingC3CorridorSafePrefix(
      candidate, 0.381, 0.005);
  ASSERT_TRUE(prefix.accepted);
  EXPECT_TRUE(IsFullSamplingC3InitialCandidateAdmitted(
      std::abs(prefix.execution_candidate.solve
                       .dynamic_terminal_object_position.x() -
               0.381) <= 0.005,
      true, true));
  EXPECT_EQ(prefix.retained_state_knots, 3);
  EXPECT_EQ(candidate.solve.dynamic_state_trajectory.size(), 5);
}

TEST(XarmFullSamplingC3PlusTest,
     ContactAcquisitionRequiresMeasuredPhysicalEngagement) {
  EXPECT_FALSE(IsFullSamplingC3ContactEngagementComplete(0.0001));
  EXPECT_TRUE(IsFullSamplingC3ContactEngagementComplete(0.0));
  EXPECT_TRUE(IsFullSamplingC3ContactEngagementComplete(-0.0001));
  EXPECT_THROW(IsFullSamplingC3ContactEngagementComplete(
                   std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     BoundedCorrectiveContactCanRestoreThePrimaryReserve) {
  EXPECT_TRUE(IsFullSamplingC3ContactLateralRejected(
      0.3831, 0.381, 0.0021, 0.002, 0.005, 0.001, false));
  EXPECT_FALSE(IsFullSamplingC3ContactLateralRejected(
      0.3831, 0.381, 0.0021, 0.002, 0.005, 0.001, true));
  EXPECT_TRUE(IsFullSamplingC3ContactLateralRejected(
      0.3842, 0.381, 0.0021, 0.002, 0.005, 0.001, true));
  EXPECT_TRUE(IsFullSamplingC3ContactLateralRejected(
      0.3861, 0.381, 0.0021, 0.002, 0.005, 0.001, true));
  EXPECT_THROW(IsFullSamplingC3ContactLateralRejected(
                   0.381, 0.381, -0.001, 0.002, 0.005, 0.001, true),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     VelocityLimitedPostureStepMustReduceLiveFkError) {
  EXPECT_TRUE(IsFullSamplingC3PostureStepProgressive(0.1, 0.09, 0.003));
  EXPECT_TRUE(IsFullSamplingC3PostureStepProgressive(0.1, 0.100001, 0.003));
  EXPECT_FALSE(IsFullSamplingC3PostureStepProgressive(0.1, 0.104, 0.003));
  EXPECT_THROW(IsFullSamplingC3PostureStepProgressive(-0.1, 0.09, 0.003),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     RepositionTargetUsesReferenceRelativeCostHysteresis) {
  EXPECT_TRUE(ShouldPreserveXarmFullSamplingC3RepositionTarget(
      true, false, 10.0, 9.6, 0.05));
  EXPECT_FALSE(ShouldPreserveXarmFullSamplingC3RepositionTarget(
      true, false, 10.0, 9.4, 0.05));
  EXPECT_FALSE(ShouldPreserveXarmFullSamplingC3RepositionTarget(
      true, true, 10.0, 9.6, 0.05));
  EXPECT_FALSE(ShouldPreserveXarmFullSamplingC3RepositionTarget(
      false, false, 10.0, 9.6, 0.05));
}

TEST(XarmFullSamplingC3PlusTest,
     CartesianCommandDurationUsesConfiguredSpeedAndMinimumPeriod) {
  EXPECT_DOUBLE_EQ(XarmFullSamplingC3CommandDuration(0.018, 0.18, 0.05),
                   0.1);
  EXPECT_DOUBLE_EQ(XarmFullSamplingC3CommandDuration(0.001, 0.18, 0.05),
                   0.05);
  EXPECT_THROW(XarmFullSamplingC3CommandDuration(0.1, 0.0, 0.05),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredCycleBudgetReservesAcquisitionDwellAndRecovery) {
  const std::vector<int> measured_release_recovery{692, 788, 770};
  const auto late = EvaluateFullSamplingC3CycleBudget(
      6709, 8000, 200, 0.05, 20, 1000,
      measured_release_recovery);
  EXPECT_FALSE(late.accepted);
  EXPECT_EQ(late.remaining_updates, 1291);
  EXPECT_EQ(late.acquisition_updates, 500);
  EXPECT_EQ(late.contact_dwell_updates, 1000);
  EXPECT_EQ(late.release_recovery_updates, 788);
  EXPECT_EQ(late.required_updates, 2288);
  EXPECT_EQ(late.measured_release_recovery_receipts, 3);

  const auto sufficient = EvaluateFullSamplingC3CycleBudget(
      5000, 8000, 200, 0.05, 20, 1000,
      measured_release_recovery);
  EXPECT_TRUE(sufficient.accepted);
  EXPECT_EQ(sufficient.remaining_updates, 3000);

  const auto equality = EvaluateFullSamplingC3CycleBudget(
      5712, 8000, 200, 0.05, 20, 1000,
      measured_release_recovery);
  EXPECT_FALSE(equality.accepted);
  EXPECT_EQ(equality.remaining_updates, equality.required_updates);
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredCycleBudgetRejectsUnmeasuredOrInvalidInputs) {
  EXPECT_THROW(EvaluateFullSamplingC3CycleBudget(
                   100, 2000, 200, 0.05, 20, 1000, {}),
               std::invalid_argument);
  EXPECT_THROW(EvaluateFullSamplingC3CycleBudget(
                   100, 2000, 200, 0.05, 20, 1000, {-1}),
               std::invalid_argument);
  EXPECT_THROW(EvaluateFullSamplingC3CycleBudget(
                   100, 2000, 200, 0.0, 20, 1000, {100}),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredCompletionBudgetUsesWorstPhysicalDwellReceipt) {
  const std::vector<int> release_recovery{692, 788, 770};
  const auto cold_start =
      EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
          5000, 8000, 200, 0.05, 20, 1000, {}, release_recovery);
  EXPECT_EQ(cold_start.contact_dwell_updates, 1000);
  EXPECT_EQ(cold_start.required_updates, 2288);

  const auto measured =
      EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
          6600, 8000, 200, 0.05, 20, 1000, {40, 55, 47},
          release_recovery);
  EXPECT_EQ(measured.contact_dwell_updates, 55);
  EXPECT_EQ(measured.required_updates, 1343);
  EXPECT_TRUE(measured.accepted);

  const auto insufficient =
      EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
          6709, 8000, 200, 0.05, 20, 1000, {40, 55, 47},
          release_recovery);
  EXPECT_FALSE(insufficient.accepted);
  EXPECT_THROW(EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
                   5000, 8000, 200, 0.05, 20, 1000, {0},
                   release_recovery),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     PhysicalAcquisitionConformanceRequiresVerifiedRecovery) {
  const auto completed = EvaluateFullSamplingC3AcquisitionConformance(
      true, 7, 7, false, false, true);
  EXPECT_TRUE(completed.physical_acquisition_completed);
  EXPECT_FALSE(completed.recovery_required);
  EXPECT_TRUE(completed.replanning_allowed);

  const auto unsafe_retry = EvaluateFullSamplingC3AcquisitionConformance(
      true, 7, 1, true, false, true);
  EXPECT_TRUE(unsafe_retry.recovery_required);
  EXPECT_FALSE(unsafe_retry.replanning_allowed);

  const auto recovered = EvaluateFullSamplingC3AcquisitionConformance(
      true, 7, 1, true, true, true);
  EXPECT_TRUE(recovered.recovery_required);
  EXPECT_TRUE(recovered.replanning_allowed);

  const auto lost_receipt = EvaluateFullSamplingC3AcquisitionConformance(
      true, 7, 1, true, true, false);
  EXPECT_FALSE(lost_receipt.replanning_allowed);
}

TEST(XarmFullSamplingC3PlusTest,
     WaypointConformanceUsesExistingActivationTolerance) {
  EXPECT_TRUE(IsFullSamplingC3WaypointExecutionConformant(
      0.05, 0.053, 0.003));
  EXPECT_FALSE(IsFullSamplingC3WaypointExecutionConformant(
      0.05, 0.053001, 0.003));
  EXPECT_TRUE(IsFullSamplingC3WaypointExecutionConformant(
      0.10, 0.03, 0.003));
  EXPECT_THROW(IsFullSamplingC3WaypointExecutionConformant(
                   -1.0, 0.0, 0.003),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     WaypointConformanceRequiresOnePlanningIntervalOfViolation) {
  auto receipt = EvaluateFullSamplingC3WaypointConformancePersistence(
      false, 0, 3);
  EXPECT_EQ(receipt.consecutive_violation_updates, 1);
  EXPECT_FALSE(receipt.persistent_violation);
  receipt = EvaluateFullSamplingC3WaypointConformancePersistence(
      false, receipt.consecutive_violation_updates, 3);
  EXPECT_FALSE(receipt.persistent_violation);
  receipt = EvaluateFullSamplingC3WaypointConformancePersistence(
      true, receipt.consecutive_violation_updates, 3);
  EXPECT_EQ(receipt.consecutive_violation_updates, 0);
  receipt = EvaluateFullSamplingC3WaypointConformancePersistence(
      false, 2, 3);
  EXPECT_TRUE(receipt.persistent_violation);
  EXPECT_THROW(EvaluateFullSamplingC3WaypointConformancePersistence(
                   false, -1, 3),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     ReleasedRecoveryFailuresAllInvalidateAndRetry) {
  EXPECT_TRUE(ShouldRetryXarmFullSamplingC3RecoveryResponse(
      true, false, true, false, false));
  EXPECT_TRUE(ShouldRetryXarmFullSamplingC3RecoveryResponse(
      true, false, false, true, false));
  EXPECT_TRUE(ShouldRetryXarmFullSamplingC3RecoveryResponse(
      true, false, false, false, true));
  EXPECT_TRUE(ShouldRetryXarmFullSamplingC3RecoveryResponse(
      true, false, false, false, false, true));
  EXPECT_TRUE(ShouldRetryXarmFullSamplingC3RecoveryResponse(
      true, true, false, false, false, true));
  EXPECT_FALSE(ShouldRetryXarmFullSamplingC3RecoveryResponse(
      false, false, true, false, false));
  EXPECT_FALSE(ShouldRetryXarmFullSamplingC3RecoveryResponse(
      true, true, true, false, false));
}

TEST(XarmFullSamplingC3PlusTest,
     EveryReleasedExecutionFailureRequiresMeasuredReplan) {
  EXPECT_TRUE(ShouldReplanXarmFullSamplingC3ReleasedExecutionFailure(
      true, true));
  EXPECT_FALSE(ShouldReplanXarmFullSamplingC3ReleasedExecutionFailure(
      true, false));
  EXPECT_FALSE(ShouldReplanXarmFullSamplingC3ReleasedExecutionFailure(
      false, true));
}

TEST(XarmFullSamplingC3PlusTest,
     BoundedCorridorStateContinuesWithoutProductiveCredit) {
  EXPECT_TRUE(ShouldContinueXarmFullSamplingC3BoundedCorridorState(
      false, true, true, true, true));
  EXPECT_FALSE(ShouldContinueXarmFullSamplingC3BoundedCorridorState(
      true, true, true, true, true));
  EXPECT_FALSE(ShouldContinueXarmFullSamplingC3BoundedCorridorState(
      false, true, true, true, false));
  EXPECT_FALSE(ShouldContinueXarmFullSamplingC3BoundedCorridorState(
      false, true, false, true, true));
  EXPECT_TRUE(ShouldContinueXarmFullSamplingC3BoundedCorridorState(
      false, false, true, true, true));
}

TEST(XarmFullSamplingC3PlusTest,
     ReleasedNoResponsePreservesPriorBoundedHandoffWithoutCredit) {
  EXPECT_TRUE(ShouldContinueXarmFullSamplingC3BoundedNoResponse(
      true, true, true, true, true, 6.65161e-5, 4.68074e-5,
      1.0e-3, 1.0e-3, 0.00211062, 0.005));
  EXPECT_FALSE(ShouldContinueXarmFullSamplingC3BoundedNoResponse(
      false, true, true, true, true, 0.0, 0.0,
      1.0e-3, 1.0e-3, 0.002, 0.005));
  EXPECT_FALSE(ShouldContinueXarmFullSamplingC3BoundedNoResponse(
      true, false, true, true, true, 0.0, 0.0,
      1.0e-3, 1.0e-3, 0.002, 0.005));
  EXPECT_FALSE(ShouldContinueXarmFullSamplingC3BoundedNoResponse(
      true, true, true, true, true, 1.0e-3, 0.0,
      1.0e-3, 1.0e-3, 0.002, 0.005));
  EXPECT_FALSE(ShouldContinueXarmFullSamplingC3BoundedNoResponse(
      true, true, true, true, false, 0.0, 0.0,
      1.0e-3, 1.0e-3, 0.002, 0.005));
  EXPECT_FALSE(ShouldContinueXarmFullSamplingC3BoundedNoResponse(
      true, true, true, true, true, 0.0, 0.0,
      1.0e-3, 1.0e-3, 0.0051, 0.005));
  EXPECT_THROW(ShouldContinueXarmFullSamplingC3BoundedNoResponse(
                   true, true, true, true, true, 0.0, 0.0,
                   0.0, 1.0e-3, 0.002, 0.005),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     PeriodicIkSolutionUsesEquivalentBranchNearestMeasuredState) {
  const double pi = std::acos(-1.0);
  Eigen::VectorXd solution(3), measured(3), lower(3), upper(3);
  solution << -2.0 * pi, -2.0, 0.5;
  measured << 0.01, -1.9, 0.4;
  lower << -2.0 * pi, -2.0, -1.0;
  upper << 2.0 * pi, 2.0, 1.0;
  const Eigen::VectorXd canonical =
      CanonicalizeXarmPeriodicIkSolutionNearestMeasured(
          solution, measured, lower, upper);
  EXPECT_NEAR(canonical[0], 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(canonical[1], solution[1]);
  EXPECT_DOUBLE_EQ(canonical[2], solution[2]);
  EXPECT_THROW(CanonicalizeXarmPeriodicIkSolutionNearestMeasured(
                   solution, measured.head(2), lower, upper),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredVerticalSubtargetRejectsStaleLateralCoordinates) {
  const Eigen::Vector3d measured_tip(-0.13, 0.12, 0.61);
  const Eigen::Vector3d down = BuildMeasuredVerticalTranslationSubtarget(
      measured_tip, 0.55, 0.01);
  EXPECT_EQ(down.x(), measured_tip.x());
  EXPECT_EQ(down.y(), measured_tip.y());
  EXPECT_DOUBLE_EQ(down.z(), 0.60);

  const Eigen::Vector3d up = BuildMeasuredVerticalTranslationSubtarget(
      measured_tip, 0.63, 0.01);
  EXPECT_EQ(up.x(), measured_tip.x());
  EXPECT_EQ(up.y(), measured_tip.y());
  EXPECT_DOUBLE_EQ(up.z(), 0.62);
  EXPECT_THROW(BuildMeasuredVerticalTranslationSubtarget(
                   measured_tip, 0.55, 0.0),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     WaypointSettlePredictsOnePlanningStepOfMeasuredDrift) {
  EXPECT_TRUE(IsFullSamplingC3WaypointSettled(
      0.003, 0.06, 0.05, 0.003));
  EXPECT_FALSE(IsFullSamplingC3WaypointSettled(
      0.003001, 0.0, 0.05, 0.003));
  EXPECT_FALSE(IsFullSamplingC3WaypointSettled(
      0.0, 0.060001, 0.05, 0.003));
  EXPECT_THROW(IsFullSamplingC3WaypointSettled(
                   0.0, 0.0, 0.0, 0.003),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     PlanningHandoffRequiresFreshObjectAndAcquisitionConformance) {
  const Eigen::Vector3d planned(0.381, 0.35, 3.13);
  const auto accepted = EvaluateFullSamplingC3PlanningHandoff(
      planned, Eigen::Vector3d(0.3815, 0.351, -3.141), true, true,
      0.002, 0.02);
  EXPECT_TRUE(accepted.object_translation_conformant);
  EXPECT_TRUE(accepted.object_orientation_conformant);
  EXPECT_TRUE(accepted.acquisition_conformant);
  EXPECT_TRUE(accepted.accepted);

  const auto stale_object = EvaluateFullSamplingC3PlanningHandoff(
      planned, Eigen::Vector3d(0.384, 0.35, 3.13), true, true,
      0.002, 0.02);
  EXPECT_FALSE(stale_object.object_translation_conformant);
  EXPECT_FALSE(stale_object.accepted);

  const auto stale_acquisition = EvaluateFullSamplingC3PlanningHandoff(
      planned, planned, true, false, 0.002, 0.02);
  EXPECT_FALSE(stale_acquisition.acquisition_conformant);
  EXPECT_FALSE(stale_acquisition.accepted);
  EXPECT_THROW(EvaluateFullSamplingC3PlanningHandoff(
                   planned, planned, true, true, -0.002, 0.02),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest, RejectsOnlyMatureWrongPolarityResponse) {
  EXPECT_FALSE(IsFullSamplingC3WrongPolarityResponse(
      0.006, 0.007, 999, 1000));
  EXPECT_TRUE(IsFullSamplingC3WrongPolarityResponse(
      0.006, 0.007, 1000, 1000));
  EXPECT_TRUE(IsFullSamplingC3WrongPolarityResponse(
      -0.006, -0.007, 1000, 1000));
  EXPECT_FALSE(IsFullSamplingC3WrongPolarityResponse(
      0.006, 0.005, 1000, 1000));
}

TEST(XarmFullSamplingC3PlusTest,
     ContactDwellRequiresMeasuredTipHoldAndFaceContinuation) {
  const Eigen::Vector3d target_W(0.12, 0.0, 0.03);
  const Eigen::Vector3d object_pose(0.0, 0.0, 0.0);
  const Eigen::Vector2d point_O(0.10, 0.0);
  const Eigen::Vector2d normal_O(1.0, 0.0);
  const auto held = EvaluateFullSamplingC3ContactDwellContinuation(
      target_W, Eigen::Vector3d(0.121, 0.0, 0.03), object_pose,
      point_O, normal_O, 0.02, 0.003);
  EXPECT_TRUE(held.tip_hold_conformant);
  EXPECT_TRUE(held.contact_gap_accepted);
  EXPECT_TRUE(held.contact_continuation);

  const auto separated = EvaluateFullSamplingC3ContactDwellContinuation(
      target_W, target_W, Eigen::Vector3d(-0.01, 0.0, 0.0),
      point_O, normal_O, 0.02, 0.003);
  EXPECT_TRUE(separated.tip_hold_conformant);
  EXPECT_FALSE(separated.contact_gap_accepted);
  EXPECT_FALSE(separated.contact_continuation);

  const auto left_hold = EvaluateFullSamplingC3ContactDwellContinuation(
      target_W, Eigen::Vector3d(0.122, 0.004, 0.03), object_pose,
      point_O, normal_O, 0.02, 0.003);
  EXPECT_FALSE(left_hold.tip_hold_conformant);
  EXPECT_TRUE(left_hold.contact_gap_accepted);
  EXPECT_FALSE(left_hold.contact_continuation);

  const auto engaged = EvaluateFullSamplingC3ContactDwellContinuation(
      target_W, Eigen::Vector3d(0.119, 0.004, 0.03), object_pose,
      point_O, normal_O, 0.02, 0.003);
  EXPECT_FALSE(engaged.tip_hold_conformant);
  EXPECT_LT(engaged.signed_contact_gap, 0.0);
  EXPECT_TRUE(engaged.contact_continuation);
  EXPECT_THROW(EvaluateFullSamplingC3ContactDwellContinuation(
                   target_W, target_W, object_pose, point_O,
                   Eigen::Vector2d::Zero(), 0.02, 0.003),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredResponseCompletionRequiresPersistentStrictDescent) {
  const Eigen::Vector3d start(0.381, 0.400, 0.0);
  const Eigen::Vector3d previous(0.381, 0.390, 0.010);
  const Eigen::Vector3d measured(0.381, 0.389, 0.011);
  const Eigen::Vector3d goal(0.381, -0.400, M_PI);
  const auto pending = EvaluateFullSamplingC3MeasuredResponseCompletion(
      start, previous, measured, goal, true, true, 1, 3,
      0.002, 0.002, 0.002);
  EXPECT_TRUE(pending.contact_continuation);
  EXPECT_TRUE(pending.terminal_descent_accepted);
  EXPECT_TRUE(pending.incremental_motion_bounded);
  EXPECT_EQ(pending.consecutive_bounded_updates, 2);
  EXPECT_FALSE(pending.completed);

  const auto complete = EvaluateFullSamplingC3MeasuredResponseCompletion(
      start, previous, measured, goal, true, true, 2, 3,
      0.002, 0.002, 0.002);
  EXPECT_TRUE(complete.persistence_accepted);
  EXPECT_TRUE(complete.completed);

  const auto separated = EvaluateFullSamplingC3MeasuredResponseCompletion(
      start, previous, measured, goal, false, true, 2, 3,
      0.002, 0.002, 0.002);
  EXPECT_EQ(separated.consecutive_bounded_updates, 0);
  EXPECT_FALSE(separated.completed);

  const auto lateral = EvaluateFullSamplingC3MeasuredResponseCompletion(
      start, previous, Eigen::Vector3d(0.386, 0.389, 0.011), goal,
      true, true, 2, 3, 0.002, 0.002, 0.002);
  EXPECT_FALSE(lateral.lateral_accepted);
  EXPECT_FALSE(lateral.completed);
  EXPECT_THROW(EvaluateFullSamplingC3MeasuredResponseCompletion(
                   start, previous, measured, goal, true, true, 0, 0,
                   0.002, 0.002, 0.002),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredContactBudgetEvidenceRequiresAuthoritativeExit) {
  EXPECT_TRUE(ShouldRecordFullSamplingC3MeasuredContactPhase(
      51, true, false, true, false, false, false));
  EXPECT_TRUE(ShouldRecordFullSamplingC3MeasuredContactPhase(
      40, true, true, false, false, false, false));
  EXPECT_TRUE(ShouldRecordFullSamplingC3MeasuredContactPhase(
      12, true, false, false, false, false, true));
  EXPECT_FALSE(ShouldRecordFullSamplingC3MeasuredContactPhase(
      51, true, false, false, false, false, false));
  EXPECT_FALSE(ShouldRecordFullSamplingC3MeasuredContactPhase(
      0, true, true, false, false, false, false));
  EXPECT_FALSE(ShouldRecordFullSamplingC3MeasuredContactPhase(
      51, false, false, true, false, false, false));
  EXPECT_THROW(ShouldRecordFullSamplingC3MeasuredContactPhase(
                   -1, true, true, false, false, false, false),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     LocalRepositionIsPreferredOnlyWithCapsuleAndIkReceipts) {
  const auto local = EvaluateFullSamplingC3LocalReposition(
      true, true, true, true);
  EXPECT_TRUE(local.accepted);
  EXPECT_TRUE(local.local_accepted);
  EXPECT_FALSE(local.use_neutral_anchor);

  const auto fallback = EvaluateFullSamplingC3LocalReposition(
      false, true, true, true);
  EXPECT_TRUE(fallback.accepted);
  EXPECT_FALSE(fallback.local_accepted);
  EXPECT_TRUE(fallback.use_neutral_anchor);

  const auto rejected = EvaluateFullSamplingC3LocalReposition(
      true, false, true, false);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_FALSE(rejected.use_neutral_anchor);
}

TEST(XarmFullSamplingC3PlusTest,
     RecoveryPolarityUsesPostPrimaryPoseAndBothTerminalComponents) {
  const Eigen::Vector3d start(0.387, -0.35, 2.90);
  const Eigen::Vector3d goal(0.381, -0.40, M_PI);
  const auto accepted = EvaluateFullSamplingC3MeasuredRecoveryPolarity(
      start, Eigen::Vector3d(0.382, -0.357, 2.91), goal, 0.002, 0.002);
  EXPECT_TRUE(accepted.response_observed);
  EXPECT_TRUE(accepted.terminal.translation_nonregressive);
  EXPECT_TRUE(accepted.terminal.orientation_nonregressive);
  EXPECT_FALSE(accepted.wrong_polarity);

  // Restoring x does not authorize borrowing y/yaw progress from the primary
  // push that ended at `start`.
  const auto wrong = EvaluateFullSamplingC3MeasuredRecoveryPolarity(
      start, Eigen::Vector3d(0.382, -0.343, 2.89), goal, 0.002, 0.002);
  EXPECT_TRUE(wrong.response_observed);
  EXPECT_FALSE(wrong.terminal.translation_nonregressive);
  EXPECT_FALSE(wrong.terminal.orientation_nonregressive);
  EXPECT_TRUE(wrong.wrong_polarity);

  const auto unobserved = EvaluateFullSamplingC3MeasuredRecoveryPolarity(
      start, start, goal, 0.002, 0.002);
  EXPECT_FALSE(unobserved.response_observed);
  EXPECT_FALSE(unobserved.wrong_polarity);
}

TEST(XarmFullSamplingC3PlusTest,
     RecoveryPredictionUsesSuccessorMinimaAsDebtBounds) {
  const Eigen::Vector3d start(0.376, 0.37, 0.10);
  const Eigen::Vector3d goal(0.381, -0.40, M_PI);
  const auto bounded = EvaluateFullSamplingC3RecoveryCandidateAdmission(
      start, Eigen::Vector3d(0.380, 0.3708, 0.103), goal,
      0.002, 0.002);
  EXPECT_LT(bounded.terminal.translation_progress, 0.0);
  EXPECT_TRUE(bounded.translation_debt_bounded);
  EXPECT_TRUE(bounded.orientation_debt_bounded);
  EXPECT_TRUE(bounded.accepted);

  const auto excessive = EvaluateFullSamplingC3RecoveryCandidateAdmission(
      start, Eigen::Vector3d(0.380, 0.373, 0.097), goal,
      0.002, 0.002);
  EXPECT_FALSE(excessive.accepted);
  EXPECT_THROW(EvaluateFullSamplingC3RecoveryCandidateAdmission(
                   start, start, goal, -0.002, 0.002),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest, TerminalDescentRequiresParetoProgress) {
  const Eigen::Vector3d goal(0.381, -0.4, 3.141592653589793);
  const Eigen::Vector3d start(0.381, 0.4, 0.5);

  const auto translation_only = EvaluateFullSamplingC3TerminalDescent(
      start, Eigen::Vector3d(0.381, 0.39, 0.501), goal, 0.001, 0.01);
  EXPECT_TRUE(translation_only.accepted);
  EXPECT_TRUE(translation_only.translation_nonregressive);
  EXPECT_TRUE(translation_only.orientation_nonregressive);

  const auto yaw_only = EvaluateFullSamplingC3TerminalDescent(
      start, Eigen::Vector3d(0.381, 0.4, 0.52), goal, 0.001, 0.01);
  EXPECT_TRUE(yaw_only.accepted);

  const auto purchased_with_yaw_regression =
      EvaluateFullSamplingC3TerminalDescent(
          start, Eigen::Vector3d(0.381, 0.39, 0.49), goal, 0.001, 0.01);
  EXPECT_FALSE(purchased_with_yaw_regression.accepted);
  EXPECT_FALSE(purchased_with_yaw_regression.orientation_nonregressive);

  const auto purchased_with_translation_regression =
      EvaluateFullSamplingC3TerminalDescent(
          start, Eigen::Vector3d(0.381, 0.401, 0.52), goal, 0.001, 0.01);
  EXPECT_FALSE(purchased_with_translation_regression.accepted);
  EXPECT_FALSE(purchased_with_translation_regression.translation_nonregressive);

  const auto too_small = EvaluateFullSamplingC3TerminalDescent(
      start, Eigen::Vector3d(0.381, 0.3999, 0.501), goal, 0.001, 0.01);
  EXPECT_FALSE(too_small.accepted);
  EXPECT_FALSE(too_small.minimum_progress);
}

TEST(XarmFullSamplingC3PlusTest,
     NormalizedParetoDescentUsesUnchangedTerminalTolerances) {
  const Eigen::Vector3d start(0.0, 1.0, 1.0);
  const Eigen::Vector3d goal(0.0, 0.0, 0.0);
  const auto balanced = EvaluateFullSamplingC3NormalizedParetoDescent(
      start, Eigen::Vector3d(0.0, 0.95, 0.9), goal, 0.05, 0.1);
  EXPECT_TRUE(balanced.terminal.accepted);
  EXPECT_NEAR(balanced.normalized_magnitude, 2.0, 1.0e-12);

  const auto regressive = EvaluateFullSamplingC3NormalizedParetoDescent(
      start, Eigen::Vector3d(0.0, 0.9, 1.1), goal, 0.05, 0.1);
  EXPECT_FALSE(regressive.terminal.accepted);
  EXPECT_NEAR(regressive.normalized_magnitude, 1.0, 1.0e-12);
  EXPECT_THROW(EvaluateFullSamplingC3NormalizedParetoDescent(
                   start, goal, goal, 0.0, 0.1),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     ComponentTransactionBoundsInactiveTaskDebt) {
  const Eigen::Vector3d start(0.0, 1.0, 1.0);
  const Eigen::Vector3d goal(0.0, 0.0, 0.0);
  const auto yaw_with_small_translation_debt =
      EvaluateXarmFullSamplingC3ComponentTransaction(
          start, Eigen::Vector3d(0.0, 1.001, 0.995), goal,
          0.05, 0.10, 0.002, 0.002);
  EXPECT_FALSE(
      yaw_with_small_translation_debt.terminal.translation_nonregressive);
  EXPECT_TRUE(yaw_with_small_translation_debt.translation_debt_bounded);
  EXPECT_GT(yaw_with_small_translation_debt.normalized_magnitude, 0.0);
  EXPECT_TRUE(yaw_with_small_translation_debt.accepted);

  const auto net_regressive = EvaluateXarmFullSamplingC3ComponentTransaction(
      start, Eigen::Vector3d(0.0, 1.01, 0.995), goal,
      0.05, 0.10, 0.002, 0.002);
  EXPECT_LT(net_regressive.normalized_magnitude, 0.0);
  EXPECT_FALSE(net_regressive.accepted);

  const auto excessive_debt = EvaluateXarmFullSamplingC3ComponentTransaction(
      start, Eigen::Vector3d(0.0, 1.051, 0.8), goal,
      0.05, 0.10, 0.002, 0.002);
  EXPECT_FALSE(excessive_debt.translation_debt_bounded);
  EXPECT_FALSE(excessive_debt.accepted);
}

TEST(XarmFullSamplingC3PlusTest,
     PostRecoveryProgressOwnsTheFinalMeasuredPose) {
  const Eigen::Vector3d start(0.381, 0.40, 0.0);
  const Eigen::Vector3d goal(0.381, -0.40, 3.141592653589793);
  const auto preserved = EvaluateFullSamplingC3PostRecoveryProgress(
      start, Eigen::Vector3d(0.382, 0.38, 0.10), goal,
      0.005, 0.001, 0.01);
  EXPECT_TRUE(preserved.accepted);
  EXPECT_TRUE(preserved.lateral_accepted);

  const auto undone = EvaluateFullSamplingC3PostRecoveryProgress(
      start, Eigen::Vector3d(0.382, 0.41, 0.10), goal,
      0.005, 0.001, 0.01);
  EXPECT_FALSE(undone.accepted);
  EXPECT_FALSE(undone.terminal.translation_nonregressive);

  const auto lateral_exit = EvaluateFullSamplingC3PostRecoveryProgress(
      start, Eigen::Vector3d(0.39, 0.38, 0.10), goal,
      0.005, 0.001, 0.01);
  EXPECT_FALSE(lateral_exit.accepted);
  EXPECT_FALSE(lateral_exit.lateral_accepted);
}

TEST(XarmFullSamplingC3PlusTest,
     IncrementalRecoveryCannotBorrowPrimaryPushProgress) {
  const Eigen::Vector3d cycle_start(0.381, 0.40, 0.0);
  const Eigen::Vector3d primary_end(0.3787, 0.3734, -0.0601);
  const Eigen::Vector3d recovery_end(0.3797, 0.3736, -0.0704);
  const Eigen::Vector3d goal(0.381, -0.40, 3.141592653589793);

  const auto complete_transaction =
      EvaluateFullSamplingC3PostRecoveryProgress(
          cycle_start, recovery_end, goal, 0.005, 0.002, 0.002);
  EXPECT_TRUE(complete_transaction.accepted);

  const auto recovery_increment =
      EvaluateFullSamplingC3PostRecoveryProgress(
          primary_end, recovery_end, goal, 0.005, 0.002, 0.002);
  EXPECT_FALSE(recovery_increment.accepted);
  EXPECT_FALSE(recovery_increment.terminal.translation_nonregressive);
  EXPECT_TRUE(recovery_increment.terminal.orientation_nonregressive);
}

TEST(XarmFullSamplingC3PlusTest,
     IncrementalRecoveryCannotBorrowPrimaryYawProgress) {
  const Eigen::Vector3d cycle_start(0.381, 0.40, 0.0);
  const Eigen::Vector3d primary_end(0.381, 0.3734, -0.10);
  const Eigen::Vector3d recovery_end(0.381, 0.3720, -0.09);
  const Eigen::Vector3d goal(0.381, -0.40, 3.141592653589793);

  const auto complete_transaction =
      EvaluateFullSamplingC3PostRecoveryProgress(
          cycle_start, recovery_end, goal, 0.005, 0.002, 0.002);
  EXPECT_TRUE(complete_transaction.accepted);

  const auto recovery_increment =
      EvaluateFullSamplingC3PostRecoveryProgress(
          primary_end, recovery_end, goal, 0.005, 0.002, 0.002);
  EXPECT_FALSE(recovery_increment.accepted);
  EXPECT_TRUE(recovery_increment.terminal.translation_nonregressive);
  EXPECT_FALSE(recovery_increment.terminal.orientation_nonregressive);
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredResponseConditioningPreservesCostClassSemantics) {
  const Eigen::Vector3d goal(0.0, -1.0, 1.0);
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d predicted(0.0, -0.02, 0.02);
  const Eigen::Vector2d point(0.0, 0.02);
  const Eigen::Vector2d normal(0.0, 1.0);

  const auto unseen = EvaluateFullSamplingC3MeasuredResponseConditioning(
      start, predicted, point, normal, goal, {}, 0.05, 0.10, 0.01,
      0.002, 0.002);
  EXPECT_EQ(unseen.ranking_class, 1);
  EXPECT_EQ(unseen.matching_observations, 0);
  EXPECT_TRUE(unseen.corrected_terminal_object_pose.isApprox(predicted));

  XarmFullSamplingC3MeasuredResponse compatible;
  compatible.start_object_pose = start;
  compatible.predicted_terminal_object_pose = predicted;
  compatible.measured_terminal_object_pose =
      Eigen::Vector3d(0.0, -0.03, 0.03);
  compatible.sample_point_O = point;
  compatible.sample_normal_O = normal;
  const auto preferred =
      EvaluateFullSamplingC3MeasuredResponseConditioning(
          start, predicted, point, normal, goal, {compatible}, 0.05, 0.10,
          0.01, 0.002, 0.002);
  EXPECT_EQ(preferred.ranking_class, 1);
  EXPECT_EQ(preferred.compatible_observations, 1);
  EXPECT_TRUE(preferred.corrected_terminal_accepted);
  EXPECT_TRUE(preferred.corrected_lateral_accepted);
  EXPECT_TRUE(preferred.corrected_terminal_object_pose.isApprox(
      compatible.measured_terminal_object_pose));
  EXPECT_NEAR(preferred.observed_progress_gain, 1.5, 1.0e-12);
  EXPECT_NEAR(preferred.calibrated_normalized_magnitude, 0.9, 1.0e-12);
  const auto replicated_preferred =
      EvaluateFullSamplingC3MeasuredResponseConditioning(
          start, predicted, point, normal, goal,
          {compatible, compatible}, 0.05, 0.10, 0.01, 0.002, 0.002);
  EXPECT_EQ(replicated_preferred.ranking_class, 0);
  EXPECT_EQ(replicated_preferred.compatible_observations, 2);

  XarmFullSamplingC3MeasuredResponse incompatible = compatible;
  incompatible.measured_terminal_object_pose =
      Eigen::Vector3d(0.02, -0.01, -0.01);
  incompatible.lateral_rejected = true;
  const auto demoted =
      EvaluateFullSamplingC3MeasuredResponseConditioning(
          start, predicted, point, normal, goal, {incompatible}, 0.05, 0.10,
          0.01, 0.002, 0.002);
  EXPECT_EQ(demoted.ranking_class, 1);
  EXPECT_EQ(demoted.lateral_rejections, 1);
  EXPECT_EQ(demoted.terminal_regressions, 1);
  EXPECT_FALSE(demoted.corrected_terminal_accepted);
  EXPECT_FALSE(demoted.corrected_lateral_accepted);
  const auto replicated_demoted =
      EvaluateFullSamplingC3MeasuredResponseConditioning(
          start, predicted, point, normal, goal,
          {incompatible, incompatible}, 0.05, 0.10, 0.01, 0.002, 0.002);
  EXPECT_EQ(replicated_demoted.ranking_class, 2);

  incompatible.start_object_pose.y() = 0.2;
  const auto outside_neighborhood =
      EvaluateFullSamplingC3MeasuredResponseConditioning(
          start, predicted, point, normal, goal, {incompatible}, 0.05, 0.10,
          0.01, 0.002, 0.002);
  EXPECT_EQ(outside_neighborhood.ranking_class, 1);
  EXPECT_EQ(outside_neighborhood.matching_observations, 0);
}

TEST(XarmFullSamplingC3PlusTest,
     EquivariantResponseConditioningTransfersAcrossLargeYaw) {
  XarmFullSamplingC3MeasuredResponse observation;
  observation.start_object_pose = Eigen::Vector3d(0.0, 0.0, 0.0);
  observation.predicted_terminal_object_pose =
      Eigen::Vector3d(0.0, -0.02, 0.0);
  observation.measured_terminal_object_pose =
      Eigen::Vector3d(0.0, -0.03, 0.0);
  observation.sample_point_O = Eigen::Vector2d(0.0, 0.02);
  observation.sample_normal_O = Eigen::Vector2d(0.0, 1.0);

  const Eigen::Vector3d start(0.0, 0.0, M_PI_2);
  const Eigen::Vector3d predicted(0.02, 0.0, M_PI_2);
  const Eigen::Vector3d goal(0.03, 0.0, M_PI_2);
  const auto receipt =
      EvaluateFullSamplingC3EquivariantResponseConditioning(
          start, predicted, observation.sample_point_O,
          observation.sample_normal_O, goal, {observation}, 0.05, 0.1,
          0.05, 0.001, 0.01);
  EXPECT_EQ(receipt.matching_observations, 1);
  EXPECT_EQ(receipt.ranking_class, 1);
  EXPECT_TRUE(receipt.corrected_terminal_accepted);
  EXPECT_TRUE(receipt.corrected_terminal_object_pose.isApprox(
      Eigen::Vector3d(0.03, 0.0, M_PI_2), 1.0e-12));
  const auto replicated_receipt =
      EvaluateFullSamplingC3EquivariantResponseConditioning(
          start, predicted, observation.sample_point_O,
          observation.sample_normal_O, goal, {observation, observation},
          0.05, 0.1, 0.05, 0.001, 0.01);
  EXPECT_EQ(replicated_receipt.ranking_class, 0);
}

TEST(XarmFullSamplingC3PlusTest, PlannerModesAreExplicit) {
  EXPECT_EQ(ParseXarmSamplingC3PlannerMode("reduced_exact_t"),
            XarmSamplingC3PlannerMode::kReducedExactT);
  EXPECT_EQ(ParseXarmSamplingC3PlannerMode("full_sampling_c3plus"),
            XarmSamplingC3PlannerMode::kFullSamplingC3Plus);
  EXPECT_THROW(ParseXarmSamplingC3PlannerMode("full"), std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest, SpatialStateRoundTrips) {
  XarmFullSamplingC3State source;
  source.pusher_position_W << 0.25, -0.1, 0.03;
  source.object_quaternion_WO =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  source.object_position_W << 0.381, 0.4, 0.0298;
  source.pusher_linear_velocity_W << 0.1, 0.2, 0.3;
  source.object_angular_velocity_W << -0.4, 0.5, -0.6;
  source.object_linear_velocity_W << 0.7, -0.8, 0.9;

  const Eigen::VectorXd encoded = source.Encode();
  ASSERT_EQ(encoded.size(), XarmFullSamplingC3State::kSize);
  EXPECT_EQ(XarmFullSamplingC3State::kSize, 19);
  EXPECT_EQ(XarmFullSamplingC3State::kInputSize, 3);
  const XarmFullSamplingC3State decoded =
      XarmFullSamplingC3State::Decode(encoded);
  EXPECT_TRUE(decoded.pusher_position_W.isApprox(source.pusher_position_W));
  EXPECT_TRUE(decoded.object_quaternion_WO.coeffs().isApprox(
      source.object_quaternion_WO.coeffs()));
  EXPECT_TRUE(decoded.object_position_W.isApprox(source.object_position_W));
  EXPECT_TRUE(decoded.pusher_linear_velocity_W.isApprox(
      source.pusher_linear_velocity_W));
  EXPECT_TRUE(decoded.object_angular_velocity_W.isApprox(
      source.object_angular_velocity_W));
  EXPECT_TRUE(decoded.object_linear_velocity_W.isApprox(
      source.object_linear_velocity_W));
}

TEST(XarmFullSamplingC3PlusTest, InvalidStatesAreRejected) {
  EXPECT_THROW(
      XarmFullSamplingC3State::Decode(Eigen::VectorXd::Zero(18)),
      std::invalid_argument);
  EXPECT_THROW(
      XarmFullSamplingC3State::Decode(
          Eigen::VectorXd::Zero(XarmFullSamplingC3State::kSize)),
      std::invalid_argument);
  Eigen::VectorXd nonfinite =
      Eigen::VectorXd::Zero(XarmFullSamplingC3State::kSize);
  nonfinite[XarmFullSamplingC3State::kObjectQuaternion] = 1.0;
  nonfinite[XarmFullSamplingC3State::kObjectPosition] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(XarmFullSamplingC3State::Decode(nonfinite),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest, SpatialMultibodyLcsHasCanonicalDimensions) {
  const OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(
      "examples/sampling_c3/oim_t/parameters/oim_t.yaml");
  const XarmFullSamplingC3LcsReceipt receipt =
      BuildXarmFullSamplingC3SpatialLcsWitness(params);
  EXPECT_TRUE(receipt.finite);
  EXPECT_EQ(receipt.num_states, 19);
  EXPECT_EQ(receipt.num_inputs, 3);
  EXPECT_EQ(receipt.num_contact_pairs, 5);
  EXPECT_EQ(receipt.num_contact_variables, 20);
  EXPECT_EQ(receipt.horizon, 5);
  EXPECT_DOUBLE_EQ(receipt.dt, 0.05);
  EXPECT_TRUE(std::isfinite(receipt.complementarity_offset_min));
  EXPECT_TRUE(std::isfinite(receipt.complementarity_offset_max));
}

TEST(XarmFullSamplingC3PlusTest, FirstSolveReportsBothQpAndProjectionResiduals) {
  OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(
      "examples/sampling_c3/oim_t/parameters/oim_t.yaml");
  params.object.start_angular_velocity_W << 0.0, 0.0, 1.0e-3;
  params.object.start_linear_velocity_W << 1.0e-3, -2.0e-3, 0.0;
  const XarmFullSamplingC3SolveReceipt receipt =
      RunXarmFullSamplingC3FirstSolve(params);
  EXPECT_TRUE(receipt.finite);
  EXPECT_LE(receipt.initial_state_residual,
            params.full_sampling_c3plus.dynamics_residual_tolerance);
  EXPECT_LE(receipt.dynamics_residual,
            params.full_sampling_c3plus.dynamics_residual_tolerance);
  EXPECT_LE(receipt.equality_residual,
            params.full_sampling_c3plus.equality_residual_tolerance);
  EXPECT_LE(receipt.projected_nonnegative_residual,
            params.full_sampling_c3plus.nonnegative_residual_tolerance);
  EXPECT_LE(receipt.projected_complementarity_residual,
            params.full_sampling_c3plus.complementarity_residual_tolerance);
  EXPECT_TRUE(receipt.dynamic_rollout_accepted);
  EXPECT_TRUE(receipt.dynamic_rollout_lcp_solved);
  EXPECT_LE(receipt.dynamic_rollout_dynamics_residual,
            params.full_sampling_c3plus.dynamics_residual_tolerance);
  EXPECT_LE(receipt.dynamic_rollout_nonnegative_residual,
            params.full_sampling_c3plus.nonnegative_residual_tolerance);
  EXPECT_LE(receipt.dynamic_rollout_complementarity_residual,
            params.full_sampling_c3plus.complementarity_residual_tolerance);
  ASSERT_FALSE(receipt.dynamic_state_trajectory.empty());
  const XarmFullSamplingC3State measured_initial_state =
      XarmFullSamplingC3State::Decode(
          receipt.dynamic_state_trajectory.front());
  EXPECT_TRUE(measured_initial_state.object_angular_velocity_W.isApprox(
      params.object.start_angular_velocity_W, 1.0e-12));
  EXPECT_TRUE(measured_initial_state.object_linear_velocity_W.isApprox(
      params.object.start_linear_velocity_W, 1.0e-12));
}

TEST(XarmFullSamplingC3PlusTest, ExactTSamplesMatchReducedPlannerGeometry) {
  const auto& samples = GetXarmFullSamplingC3ExactTSamples();
  ASSERT_EQ(samples.size(), 8u);
  EXPECT_EQ(samples.front().name, "crossbar_top_left");
  EXPECT_TRUE(samples.front().point_O.isApprox(
      Eigen::Vector2d(-0.0445, 0.0198)));
  EXPECT_TRUE(samples.front().outward_normal_O.isApprox(
      Eigen::Vector2d(0.0, 1.0)));
  EXPECT_EQ(samples.back().name, "stem_bottom");
  EXPECT_TRUE(samples.back().point_O.isApprox(
      Eigen::Vector2d(0.0, -0.0794)));
  EXPECT_TRUE(samples.back().outward_normal_O.isApprox(
      Eigen::Vector2d(0.0, -1.0)));
  for (const auto& sample : samples) {
    EXPECT_NEAR(sample.outward_normal_O.norm(), 1.0, 1.0e-15);
  }
}

TEST(XarmFullSamplingC3PlusTest, ExactTBatchRanksOnlyFeasibleRollouts) {
  const OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(
      "examples/sampling_c3/oim_t/parameters/oim_t.yaml");
  const XarmFullSamplingC3BatchReceipt batch =
      RunXarmFullSamplingC3ExactTBatch(params);
  EXPECT_TRUE(batch.accepted);
  EXPECT_EQ(batch.num_samples, 8);
  EXPECT_GT(batch.num_feasible, 0);
  ASSERT_GE(batch.selected_index, 0);
  ASSERT_LT(batch.selected_index, batch.num_samples);
  EXPECT_EQ(batch.selected_name,
            batch.candidates[batch.selected_index].sample_name);
  EXPECT_DOUBLE_EQ(batch.selected_cost,
                   batch.candidates[batch.selected_index]
                       .solve.dynamic_rollout_cost);
  for (const auto& candidate : batch.candidates) {
    EXPECT_TRUE(candidate.solve.dynamic_rollout_accepted)
        << candidate.sample_name;
    EXPECT_TRUE(std::isfinite(candidate.solve.dynamic_rollout_cost))
        << candidate.sample_name;
    if (candidate.solve.dynamic_rollout_workspace_accepted) {
      EXPECT_LE(batch.selected_cost, candidate.solve.dynamic_rollout_cost);
    }
  }
  EXPECT_LT(batch.num_feasible, batch.num_samples);
}

TEST(XarmFullSamplingC3PlusTest, SeededPerimeterSamplesAreReproducible) {
  const auto first = GenerateXarmFullSamplingC3PerimeterSamples(16, 0);
  const auto second = GenerateXarmFullSamplingC3PerimeterSamples(16, 0);
  ASSERT_EQ(first.size(), 16u);
  ASSERT_EQ(second.size(), first.size());
  for (int i = 0; i < static_cast<int>(first.size()); ++i) {
    EXPECT_EQ(first[i].name, second[i].name);
    EXPECT_TRUE(first[i].point_O.isApprox(second[i].point_O));
    EXPECT_TRUE(first[i].outward_normal_O.isApprox(
        second[i].outward_normal_O));
    EXPECT_NEAR(first[i].outward_normal_O.norm(), 1.0, 1.0e-15);
  }
  EXPECT_THROW(GenerateXarmFullSamplingC3PerimeterSamples(0, 0),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest, SeededPerimeterBatchHasExecutableCandidate) {
  const OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(
      "examples/sampling_c3/oim_t/parameters/oim_t.yaml");
  const XarmFullSamplingC3BatchReceipt batch =
      RunXarmFullSamplingC3PerimeterBatch(params);
  EXPECT_TRUE(batch.accepted);
  EXPECT_EQ(batch.num_samples,
            params.full_sampling_c3plus.perimeter_sample_count);
  EXPECT_GT(batch.num_feasible, 0);
  EXPECT_GE(batch.selected_index, 0);
  EXPECT_TRUE(std::isfinite(batch.selected_cost));
}

TEST(XarmFullSamplingC3PlusTest, CandidateBufferPreservesRejectedSamples) {
  const OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(
      "examples/sampling_c3/oim_t/parameters/oim_t.yaml");
  const XarmFullSamplingC3CandidateBuffer buffer =
      BuildXarmFullSamplingC3CandidateBuffer({
          RunXarmFullSamplingC3ExactTBatch(params),
          RunXarmFullSamplingC3PerimeterBatch(params)});
  EXPECT_TRUE(buffer.accepted);
  EXPECT_EQ(buffer.total_candidates,
            8 + params.full_sampling_c3plus.perimeter_sample_count);
  EXPECT_FALSE(buffer.successful.empty());
  EXPECT_FALSE(buffer.unsuccessful.empty());
  for (int i = 1; i < static_cast<int>(buffer.successful.size()); ++i) {
    EXPECT_LE(buffer.successful[i - 1].solve.dynamic_rollout_cost,
              buffer.successful[i].solve.dynamic_rollout_cost);
  }
  const auto contact_feasible =
      BuildXarmFullSamplingC3ContactFeasibleCandidateBuffer(buffer);
  EXPECT_TRUE(contact_feasible.accepted);
  EXPECT_EQ(contact_feasible.total_candidates, buffer.total_candidates);
  EXPECT_GE(contact_feasible.successful.size(), buffer.successful.size());
  EXPECT_EQ(contact_feasible.total_candidates,
            static_cast<int>(contact_feasible.successful.size() +
                             contact_feasible.unsuccessful.size()));
  for (const auto& candidate : contact_feasible.successful) {
    EXPECT_TRUE(candidate.solve.dynamic_rollout_accepted);
    EXPECT_LE(candidate.solve.planned_input_bound_violation, 1.0e-12);
    EXPECT_TRUE(std::isfinite(candidate.solve.dynamic_rollout_cost));
  }
}

TEST(XarmFullSamplingC3PlusTest, ParallelReductionMatchesSerialSelection) {
  const OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(
      "examples/sampling_c3/oim_t/parameters/oim_t.yaml");
  const auto serial = RunXarmFullSamplingC3PerimeterBatch(params);
  const auto parallel = RunXarmFullSamplingC3PerimeterBatchParallel(params);
  EXPECT_TRUE(serial.accepted);
  EXPECT_TRUE(parallel.accepted);
  EXPECT_EQ(parallel.num_samples, serial.num_samples);
  EXPECT_EQ(parallel.num_feasible, serial.num_feasible);
  EXPECT_EQ(parallel.selected_index, serial.selected_index);
  EXPECT_EQ(parallel.selected_name, serial.selected_name);
  EXPECT_NEAR(parallel.selected_cost, serial.selected_cost, 1.0e-8);
  ASSERT_EQ(parallel.candidates.size(), serial.candidates.size());
  for (int i = 0; i < static_cast<int>(serial.candidates.size()); ++i) {
    EXPECT_EQ(parallel.candidates[i].sample_name,
              serial.candidates[i].sample_name);
    EXPECT_NEAR(parallel.candidates[i].solve.dynamic_rollout_cost,
                serial.candidates[i].solve.dynamic_rollout_cost, 1.0e-8);
  }
}

TEST(XarmFullSamplingC3PlusTest, ExactCollisionMeshSamplesAreReproducible) {
  const auto first = GenerateXarmFullSamplingC3MeshNormalSamples(16, 1);
  const auto second = GenerateXarmFullSamplingC3MeshNormalSamples(16, 1);
  ASSERT_EQ(first.size(), 16u);
  ASSERT_EQ(second.size(), first.size());
  for (int i = 0; i < static_cast<int>(first.size()); ++i) {
    EXPECT_EQ(first[i].name, second[i].name);
    EXPECT_TRUE(first[i].point_O.isApprox(second[i].point_O));
    EXPECT_DOUBLE_EQ(first[i].height_O, second[i].height_O);
    EXPECT_GE(first[i].height_O, -0.0298);
    EXPECT_LE(first[i].height_O, 0.0298);
    EXPECT_NEAR(first[i].outward_normal_O.norm(), 1.0, 1.0e-15);
    const Eigen::Vector2d n = first[i].outward_normal_O;
    const Eigen::Vector2d p = first[i].point_O;
    if (n.y() > 0.5) {
      EXPECT_NEAR(p.y(), 0.0198, 1.0e-15);
    }
    if (n.y() < -0.5) {
      EXPECT_TRUE(std::abs(p.y()) < 1.0e-15 ||
                  std::abs(p.y() + 0.0794) < 1.0e-15);
    }
    if (n.x() < -0.5) {
      EXPECT_TRUE(std::abs(p.x() + 0.0445) < 1.0e-15 ||
                  std::abs(p.x() + 0.0099) < 1.0e-15);
    }
    if (n.x() > 0.5) {
      EXPECT_TRUE(std::abs(p.x() - 0.0445) < 1.0e-15 ||
                  std::abs(p.x() - 0.0099) < 1.0e-15);
    }
  }
}

TEST(XarmFullSamplingC3PlusTest, ExactCollisionMeshBatchIsExecutable) {
  const OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(
      "examples/sampling_c3/oim_t/parameters/oim_t.yaml");
  const auto batch = RunXarmFullSamplingC3MeshBatchParallel(params);
  EXPECT_TRUE(batch.accepted);
  EXPECT_EQ(batch.num_samples, params.full_sampling_c3plus.mesh_sample_count);
  EXPECT_GT(batch.num_feasible, 0);
  EXPECT_GE(batch.selected_index, 0);
}

TEST(XarmFullSamplingC3PlusTest, SelectedPlanPreservesEverySpatialKnot) {
  const OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(
      "examples/sampling_c3/oim_t/parameters/oim_t.yaml");
  const auto buffer = BuildXarmFullSamplingC3CandidateBuffer({
      RunXarmFullSamplingC3ExactTBatch(params),
      RunXarmFullSamplingC3PerimeterBatchParallel(params),
      RunXarmFullSamplingC3MeshBatchParallel(params)});
  const auto plan = BuildXarmFullSamplingC3TaskSpacePlan(
      buffer, params.task.planning_time_step);
  EXPECT_TRUE(plan.accepted);
  EXPECT_EQ(plan.sample_name, buffer.successful.front().sample_name);
  EXPECT_DOUBLE_EQ(plan.cost,
                   buffer.successful.front().solve.dynamic_rollout_cost);
  EXPECT_EQ(plan.time_vector.size(),
            params.full_sampling_c3plus.horizon + 1);
  EXPECT_EQ(plan.pusher_positions_W.cols(), plan.time_vector.size());
  EXPECT_EQ(plan.pusher_velocities_W.cols(), plan.time_vector.size());
  EXPECT_EQ(plan.pusher_forces_W.cols(), plan.time_vector.size());
  EXPECT_DOUBLE_EQ(plan.time_vector[0], 0.0);
  EXPECT_DOUBLE_EQ(plan.time_vector[plan.time_vector.size() - 1],
                   params.full_sampling_c3plus.horizon *
                       params.task.planning_time_step);
  const Eigen::Vector3d current_tip(0.254967, -0.000911, 0.334228);
  Eigen::Vector3d elevated = plan.pusher_positions_W.col(0);
  elevated.z() = params.controller.reposition_waypoint_height;
  Eigen::Vector3d planar = elevated;
  planar.z() = current_tip.z();
  Eigen::Vector3d standoff = plan.pusher_positions_W.col(0);
  standoff.x() -= params.controller.descent_clearance;
  const auto execution = BuildXarmFullSamplingC3OscExecutionPlan(
      current_tip, planar, elevated, standoff, plan,
      params.controller.reposition_speed, params.task.planning_time_step);
  EXPECT_TRUE(execution.accepted);
  EXPECT_EQ(execution.acquisition_knots, 5);
  EXPECT_EQ(execution.c3_knots, plan.time_vector.size());
  EXPECT_EQ(execution.time_vector.size(), plan.time_vector.size() + 4);
  EXPECT_TRUE(execution.positions_W.col(0).isApprox(current_tip));
  EXPECT_TRUE(execution.positions_W.col(4).isApprox(
      plan.pusher_positions_W.col(0)));
  EXPECT_TRUE(execution.positions_W.rightCols(1).isApprox(
      plan.pusher_positions_W.rightCols(1)));
  EXPECT_TRUE(execution.forces_W.leftCols(4).isZero(0.0));
  EXPECT_TRUE(execution.forces_W.col(4).isApprox(
      plan.pusher_forces_W.col(0)));
  EXPECT_TRUE(execution.forces_W.rightCols(1).isApprox(
      plan.pusher_forces_W.rightCols(1)));
  for (int i = 1; i < execution.time_vector.size(); ++i) {
    EXPECT_GT(execution.time_vector[i], execution.time_vector[i - 1]);
  }
  const auto guarded_plan =
      BuildXarmFullSamplingC3LateralGuardedTaskSpacePlan(
          buffer, params.task.planning_time_step, params.object.goal_pose.x(),
          params.controller.lateral_drift_tolerance);
  ASSERT_TRUE(guarded_plan.accepted);
  const auto guarded_candidate = std::find_if(
      buffer.successful.begin(), buffer.successful.end(),
      [&](const auto& candidate) {
        return candidate.sample_name == guarded_plan.sample_name;
      });
  ASSERT_NE(guarded_candidate, buffer.successful.end());
  EXPECT_LE(std::abs(
                guarded_candidate->solve.dynamic_terminal_object_position.x() -
                params.object.goal_pose.x()),
            params.controller.lateral_drift_tolerance);
  for (const auto& candidate : buffer.successful) {
    if (candidate.solve.dynamic_rollout_cost < guarded_plan.cost) {
      EXPECT_GT(std::abs(
                    candidate.solve.dynamic_terminal_object_position.x() -
                    params.object.goal_pose.x()),
                params.controller.lateral_drift_tolerance);
    }
  }
}

TEST(XarmFullSamplingC3PlusTest,
     SelectedPlanUsesTheExplicitC3ExecutionPusherTrajectory) {
  XarmFullSamplingC3CandidateReceipt candidate;
  candidate.sample_name = "hybrid_execution";
  candidate.solve.dynamic_rollout_cost = 1.0;
  candidate.solve.dynamic_rollout_accepted = true;
  candidate.solve.dynamic_rollout_workspace_accepted = true;
  for (int knot = 0; knot < 2; ++knot) {
    XarmFullSamplingC3State dynamic_state;
    dynamic_state.pusher_position_W =
        Eigen::Vector3d(0.4 + knot, 0.3, 0.2);
    candidate.solve.dynamic_state_trajectory.push_back(
        dynamic_state.Encode());
    XarmFullSamplingC3State execution_state = dynamic_state;
    execution_state.pusher_position_W =
        Eigen::Vector3d(0.4 + 0.01 * knot, 0.3, 0.03);
    candidate.solve.execution_state_trajectory.push_back(
        execution_state.Encode());
    if (knot > 0) {
      candidate.solve.planned_input_trajectory.push_back(
          Eigen::Vector3d::Zero());
    }
  }
  candidate.initial_pusher_position_W =
      XarmFullSamplingC3State::Decode(
          candidate.solve.execution_state_trajectory.front())
          .pusher_position_W;
  XarmFullSamplingC3BatchReceipt batch;
  batch.candidates = {candidate};
  const auto buffer = BuildXarmFullSamplingC3CandidateBuffer({batch});
  ASSERT_TRUE(buffer.accepted);
  const auto plan = BuildXarmFullSamplingC3TaskSpacePlan(buffer, 0.05);
  ASSERT_TRUE(plan.accepted);
  EXPECT_DOUBLE_EQ(plan.pusher_positions_W(0, 1), 0.41);
  EXPECT_DOUBLE_EQ(plan.pusher_positions_W(2, 1), 0.03);
  EXPECT_NE(plan.pusher_positions_W(0, 1),
            candidate.solve.dynamic_state_trajectory.back()[
                XarmFullSamplingC3State::kPusherPosition]);
}

TEST(XarmFullSamplingC3PlusTest,
     ProductiveHandoffSurvivesLaterMeasuredBudgetDefer) {
  const auto status = EvaluateXarmFullSamplingC3TerminalStatus(
      5, 10, true, true, 5862, 8000, false);
  EXPECT_TRUE(status.closed_loop_handoff);
  EXPECT_FALSE(status.accepted);
  EXPECT_EQ(status.return_code, 4);
  EXPECT_EQ(status.reason,
            "measured_cycle_budget_deferred_before_terminal");
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredBudgetDeferWithoutProductiveHandoffFailsClosed) {
  const auto status = EvaluateXarmFullSamplingC3TerminalStatus(
      5, 10, false, true, 5862, 8000, false);
  EXPECT_FALSE(status.closed_loop_handoff);
  EXPECT_FALSE(status.accepted);
  EXPECT_EQ(status.return_code, 2);
  EXPECT_EQ(status.reason,
            "measured_cycle_budget_deferred_without_handoff");
}

TEST(XarmFullSamplingC3PlusTest, ParetoWrenchRejectsOpposingTaskComponent) {
  const auto yaw_only_with_wrong_translation =
      EvaluateXarmFullSamplingC3ParetoWrench(-1.0, 1.0, 1.0, 0.5);
  EXPECT_FALSE(yaw_only_with_wrong_translation.translation_nonregressive);
  EXPECT_TRUE(yaw_only_with_wrong_translation.orientation_nonregressive);
  EXPECT_TRUE(yaw_only_with_wrong_translation.minimum_productivity);
  EXPECT_FALSE(yaw_only_with_wrong_translation.accepted);

  const auto pareto_productive =
      EvaluateXarmFullSamplingC3ParetoWrench(-1.0, -0.5, 1.0, 0.25);
  EXPECT_TRUE(pareto_productive.translation_nonregressive);
  EXPECT_TRUE(pareto_productive.orientation_nonregressive);
  EXPECT_TRUE(pareto_productive.accepted);
}

TEST(XarmFullSamplingC3PlusTest,
     LateralRecoveryUsesUnchangedOrientationDebtBound) {
  const Eigen::Vector3d goal(0.381, -0.4, M_PI);
  const Eigen::Vector3d cycle_start(0.381, 0.35, -0.20);
  const Eigen::Vector3d rejected(0.3873, 0.344, -0.25);
  const auto bounded = EvaluateXarmFullSamplingC3LateralRecovery(
      cycle_start, rejected, Eigen::Vector3d(0.384, 0.34, -0.15), goal,
      0.05, 0.10, 0.005);
  EXPECT_TRUE(bounded.lateral_restored);
  EXPECT_TRUE(bounded.translation_nonregressive);
  EXPECT_TRUE(bounded.translation_debt_bounded);
  EXPECT_TRUE(bounded.orientation_debt_bounded);
  EXPECT_TRUE(bounded.accepted);

  const auto bounded_translation_debt =
      EvaluateXarmFullSamplingC3LateralRecovery(
          cycle_start, rejected, Eigen::Vector3d(0.384, 0.36, -0.15), goal,
          0.05, 0.10, 0.005);
  EXPECT_FALSE(bounded_translation_debt.translation_nonregressive);
  EXPECT_TRUE(bounded_translation_debt.translation_debt_bounded);
  EXPECT_TRUE(bounded_translation_debt.accepted);

  const auto excessive_translation_debt =
      EvaluateXarmFullSamplingC3LateralRecovery(
          cycle_start, rejected, Eigen::Vector3d(0.384, 0.41, -0.15), goal,
          0.05, 0.10, 0.005);
  EXPECT_FALSE(excessive_translation_debt.translation_debt_bounded);
  EXPECT_FALSE(excessive_translation_debt.accepted);

  const auto excessive_debt = EvaluateXarmFullSamplingC3LateralRecovery(
      cycle_start, rejected, Eigen::Vector3d(0.384, 0.34, -0.05), goal,
      0.05, 0.10, 0.005);
  EXPECT_FALSE(excessive_debt.orientation_debt_bounded);
  EXPECT_FALSE(excessive_debt.accepted);
}

TEST(XarmFullSamplingC3PlusTest,
     LateralRecoveryUsesFirstCorridorEntryBeforeOvershoot) {
  XarmFullSamplingC3CandidateReceipt candidate;
  auto make_state = [](double object_x) {
    XarmFullSamplingC3State state;
    state.object_quaternion_WO = Eigen::Quaterniond::Identity();
    state.object_position_W = Eigen::Vector3d(object_x, 0.35, 0.0298);
    return state.Encode();
  };
  candidate.solve.dynamic_state_trajectory = {
      make_state(0.375), make_state(0.379), make_state(0.410)};
  const auto entry = FindXarmFullSamplingC3LateralCorridorEntry(
      candidate, 0.381, 0.005);
  EXPECT_TRUE(entry.accepted);
  EXPECT_EQ(entry.state_knot, 1);
  EXPECT_DOUBLE_EQ(entry.object_pose.x(), 0.379);
}

TEST(XarmFullSamplingC3PlusTest,
     TerminalBudgetEstimateIsAnOptimisticMeasuredLowerBound) {
  const auto estimate = EstimateXarmFullSamplingC3TerminalBudget(
      Eigen::Vector3d(0.382, 0.334567, -0.146344),
      Eigen::Vector3d(0.381, -0.4, M_PI), 0.05, 0.10,
      {{0.0227562, 0.0160722, 1515},
       {0.0204729, 0.0680510, 1067},
       {0.00126094, 0.0276520, 937}});
  EXPECT_TRUE(estimate.finite);
  EXPECT_EQ(estimate.minimum_measured_cycle_updates, 937);
  EXPECT_EQ(estimate.optimistic_remaining_cycles, 43);
  EXPECT_EQ(estimate.optimistic_remaining_updates, 40291);
}

TEST(XarmFullSamplingC3PlusTest,
     TerminalBudgetSufficiencyIsNecessaryButNotTerminalSuccess) {
  XarmFullSamplingC3TerminalBudgetEstimate estimate;
  estimate.finite = true;
  estimate.optimistic_remaining_updates = 32798;
  const auto insufficient =
      EvaluateXarmFullSamplingC3TerminalBudgetSufficiency(
          6369, 8000, estimate);
  EXPECT_EQ(insufficient.remaining_updates, 1631);
  EXPECT_FALSE(insufficient.sufficient);

  const auto sufficient =
      EvaluateXarmFullSamplingC3TerminalBudgetSufficiency(
          6369, 39167, estimate);
  EXPECT_TRUE(sufficient.sufficient);
  EXPECT_THROW(EvaluateXarmFullSamplingC3TerminalBudgetSufficiency(
                   8001, 8000, estimate),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     Gate100RequiresBothUnchangedOpenTableTolerances) {
  const Eigen::Vector3d goal(0.381, -0.4, 3.1416);
  const auto pass = EvaluateXarmFullSamplingC3OpenTableTerminal(
      Eigen::Vector3d(0.401, -0.37, -3.091585307179586), goal,
      0.05, 0.10);
  EXPECT_TRUE(pass.translation_accepted);
  EXPECT_TRUE(pass.orientation_accepted);
  EXPECT_TRUE(pass.accepted);

  const auto translation_fail = EvaluateXarmFullSamplingC3OpenTableTerminal(
      Eigen::Vector3d(0.4311, -0.4, 3.1416), goal, 0.05, 0.10);
  EXPECT_FALSE(translation_fail.translation_accepted);
  EXPECT_TRUE(translation_fail.orientation_accepted);
  EXPECT_FALSE(translation_fail.accepted);

  const auto orientation_fail = EvaluateXarmFullSamplingC3OpenTableTerminal(
      Eigen::Vector3d(0.381, -0.4, 3.0415), goal, 0.05, 0.10);
  EXPECT_TRUE(orientation_fail.translation_accepted);
  EXPECT_FALSE(orientation_fail.orientation_accepted);
  EXPECT_FALSE(orientation_fail.accepted);
  EXPECT_THROW(EvaluateXarmFullSamplingC3OpenTableTerminal(
                   Eigen::Vector3d::Zero(), goal, 0.0, 0.10),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     JointStepLimiterPreservesDirectionUnderSaturation) {
  const Eigen::VectorXd velocity_limits =
      Eigen::VectorXd::Constant(6, 0.5);
  const double planning_time_step = 0.05;

  Eigen::VectorXd within(6);
  within << 0.0, 0.01, -0.02, 0.0, 0.015, 0.0;
  EXPECT_TRUE(LimitXarmFullSamplingC3JointStepPreservingDirection(
      within, velocity_limits, planning_time_step).isApprox(within));

  // The exact-8000 progress_local_lower failure shape: a wrist-dominated
  // vertical-axis correction (joint 5) alongside an elbow advance (joint 3).
  // Independent clamping forces their commanded ratio to 1:1; the limiter
  // must keep the solved ratio and run the most-saturated joint at its
  // velocity window.
  Eigen::VectorXd saturated(6);
  saturated << 0.0, -0.0021, 0.0273, 0.0, -0.0808, 0.0;
  const Eigen::VectorXd limited =
      LimitXarmFullSamplingC3JointStepPreservingDirection(
          saturated, velocity_limits, planning_time_step);
  EXPECT_TRUE(limited.isApprox(saturated * (0.025 / 0.0808), 1.0e-12));
  EXPECT_NEAR(limited.cwiseAbs().maxCoeff(), 0.025, 1.0e-12);
  EXPECT_NEAR(limited[4] / limited[2], saturated[4] / saturated[2],
              1.0e-12);

  EXPECT_THROW(LimitXarmFullSamplingC3JointStepPreservingDirection(
                   Eigen::VectorXd(), velocity_limits, planning_time_step),
               std::invalid_argument);
  EXPECT_THROW(LimitXarmFullSamplingC3JointStepPreservingDirection(
                   within, Eigen::VectorXd::Constant(5, 0.5),
                   planning_time_step),
               std::invalid_argument);
  EXPECT_THROW(LimitXarmFullSamplingC3JointStepPreservingDirection(
                   within, Eigen::VectorXd::Zero(6), planning_time_step),
               std::invalid_argument);
  EXPECT_THROW(LimitXarmFullSamplingC3JointStepPreservingDirection(
                   within, velocity_limits, 0.0),
               std::invalid_argument);
  Eigen::VectorXd non_finite = within;
  non_finite[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(LimitXarmFullSamplingC3JointStepPreservingDirection(
                   non_finite, velocity_limits, planning_time_step),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     SaturatedVerticalPostureStepStaysCartesianProgressive) {
  // Pinned live-failure state from
  // results/xarm6_gates375_379_exact8000_final2_v8rOvB: measured joints at
  // t=131.110 s (xarm_control.csv), where progress_local_lower raised
  // posture_progress=REJECT at update 4,107 with current_tip_error_m=0.01
  // and target_tip_error_m=0.0132073. Canonical oim_t.yaml literals below:
  // end_effector_point, velocity_limits, planning_time_step,
  // task_space_plan_step_limit, contact_activation_tolerance,
  // reposition_waypoint_height, and the 0.05 rad vertical-axis band.
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModels(
      "examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6.xml");
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  const auto& end_effector = plant.GetBodyByName("xarm6_stick");
  const Eigen::Vector3d end_effector_point(0.0, 0.0, 0.1794);
  const Eigen::VectorXd velocity_limits =
      Eigen::VectorXd::Constant(6, 0.5);
  const double planning_time_step = 0.05;
  const double contact_activation_tolerance = 0.003;
  const double maximum_vertical_axis_error = 0.05;
  const double vertical_axis_tilt_cost_weight = 80.0;

  Eigen::VectorXd measured_q(6);
  measured_q << 0.80396, 0.29174, -1.12233, 0.00006, 0.93621, 0.00020;
  plant.SetPositions(context.get(), measured_q);
  const auto X_WCurrent =
      plant.EvalBodyPoseInWorld(*context, end_effector);
  const Eigen::Vector3d current_tip = X_WCurrent * end_effector_point;
  const double current_axis_error = std::acos(std::clamp(
      (X_WCurrent.rotation() * Eigen::Vector3d::UnitZ())
          .dot(-Eigen::Vector3d::UnitZ()),
      -1.0, 1.0));

  const Eigen::Vector3d waypoint(0.36509, 0.37769, 0.07738005);
  const Eigen::Vector3d command_tip = current_tip +
      0.01 * (waypoint - current_tip).normalized();
  const double current_tip_error = (command_tip - current_tip).norm();

  // The same IK program SolveVerticalContactPostureStep builds. The
  // band-only probe reproduces the historical formulation the 2026-09-01
  // deadlock ran under; the tilt-cost probe is the production formulation
  // with the restored source w_tilt objective.
  auto solve_ik_step = [&](bool with_tilt_cost)
      -> std::optional<Eigen::VectorXd> {
    plant.SetPositions(context.get(), measured_q);
    drake::multibody::InverseKinematics ik(plant, context.get());
    const Eigen::Vector3d position_tolerance =
        Eigen::Vector3d::Constant(1.0e-6);
    ik.AddPositionConstraint(
        end_effector.body_frame(), end_effector_point, plant.world_frame(),
        command_tip - position_tolerance, command_tip + position_tolerance);
    ik.AddAngleBetweenVectorsConstraint(
        end_effector.body_frame(), Eigen::Vector3d::UnitZ(),
        plant.world_frame(), -Eigen::Vector3d::UnitZ(), 0.0,
        maximum_vertical_axis_error);
    if (with_tilt_cost) {
      ik.AddAngleBetweenVectorsCost(
          end_effector.body_frame(), Eigen::Vector3d::UnitZ(),
          plant.world_frame(), -Eigen::Vector3d::UnitZ(),
          vertical_axis_tilt_cost_weight);
    }
    ik.get_mutable_prog()->AddQuadraticErrorCost(
        Eigen::MatrixXd::Identity(6, 6), measured_q, ik.q());
    ik.get_mutable_prog()->SetInitialGuess(ik.q(), measured_q);
    const auto result = drake::solvers::Solve(ik.prog());
    if (!result.is_success()) return std::nullopt;
    return CanonicalizeXarmPeriodicIkSolutionNearestMeasured(
        result.GetSolution(ik.q()), measured_q,
        plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits());
  };

  auto tip_error_and_axis_at = [&](const Eigen::VectorXd& q) {
    plant.SetPositions(context.get(), q);
    const auto X_WEe = plant.EvalBodyPoseInWorld(*context, end_effector);
    const double tip_error =
        (X_WEe * end_effector_point - command_tip).norm();
    const double axis_error = std::acos(std::clamp(
        (X_WEe.rotation() * Eigen::Vector3d::UnitZ())
            .dot(-Eigen::Vector3d::UnitZ()),
        -1.0, 1.0));
    return std::make_pair(tip_error, axis_error);
  };

  // Historical band-only formulation: clamping each joint independently guts
  // the wrist correction while keeping the elbow step, and the executed tip
  // moves away from its command. This is the defect the state was pinned
  // for; the direction-preserving limiter makes the same solution
  // progressive.
  const auto band_only_solution = solve_ik_step(false);
  ASSERT_TRUE(band_only_solution.has_value());
  const Eigen::VectorXd band_only_step = *band_only_solution - measured_q;
  ASSERT_GT(band_only_step.cwiseAbs().maxCoeff(),
            1.5 * velocity_limits[0] * planning_time_step);
  Eigen::VectorXd componentwise_step = band_only_step;
  for (int i = 0; i < componentwise_step.size(); ++i) {
    const double step_limit =
        velocity_limits[i] * planning_time_step;
    componentwise_step[i] =
        std::clamp(componentwise_step[i], -step_limit, step_limit);
  }
  const auto [componentwise_error, componentwise_axis] =
      tip_error_and_axis_at(measured_q + componentwise_step);
  EXPECT_FALSE(IsFullSamplingC3PostureStepProgressive(
      current_tip_error, componentwise_error,
      contact_activation_tolerance));
  const auto [band_limited_error, band_limited_axis] =
      tip_error_and_axis_at(measured_q +
          LimitXarmFullSamplingC3JointStepPreservingDirection(
              band_only_step, velocity_limits, planning_time_step));
  EXPECT_TRUE(IsFullSamplingC3PostureStepProgressive(
      current_tip_error, band_limited_error,
      contact_activation_tolerance));
  EXPECT_LT(band_limited_error, current_tip_error);
  EXPECT_LT(band_limited_axis, current_axis_error);

  // Production formulation: with the restored source tilt objective the
  // limited step stays progressive and re-verticalizes the drifted capsule.
  const auto production_solution = solve_ik_step(true);
  ASSERT_TRUE(production_solution.has_value());
  const Eigen::VectorXd production_step =
      *production_solution - measured_q;
  const auto [production_error, production_axis] =
      tip_error_and_axis_at(measured_q +
          LimitXarmFullSamplingC3JointStepPreservingDirection(
              production_step, velocity_limits, planning_time_step));
  EXPECT_TRUE(IsFullSamplingC3PostureStepProgressive(
      current_tip_error, production_error, contact_activation_tolerance));
  EXPECT_LT(production_error, current_tip_error);
  EXPECT_LT(production_axis, current_axis_error);
}

TEST(XarmFullSamplingC3PlusTest,
     MeasuredAcquisitionReceiptsFloorTheBudgetEstimate) {
  // 74 IK steps at 0.05 s over 20 ms updates -> a 185-update estimate; with
  // 2,000 updates remaining the bare estimate admits the cycle.
  const auto estimate_only =
      EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
          6000, 8000, 74, 0.05, 20, 1000, {}, {475});
  EXPECT_EQ(estimate_only.acquisition_updates, 185);
  EXPECT_EQ(estimate_only.required_updates, 185 + 1000 + 475);
  EXPECT_TRUE(estimate_only.accepted);

  // The worst measured acquisition receipt floors the estimate and defers
  // the cycle the bare estimate would have admitted.
  const auto measured =
      EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
          6000, 8000, 74, 0.05, 20, 1000, {}, {475}, {700, 900});
  EXPECT_EQ(measured.acquisition_updates, 900);
  EXPECT_EQ(measured.required_updates, 900 + 1000 + 475);
  EXPECT_FALSE(measured.accepted);

  // A measured receipt smaller than the estimate never lowers it.
  EXPECT_EQ(EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
                6000, 8000, 74, 0.05, 20, 1000, {}, {475}, {100})
                .acquisition_updates,
            185);

  EXPECT_THROW(EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
                   6000, 8000, 74, 0.05, 20, 1000, {}, {475}, {0}),
               std::invalid_argument);
}

TEST(XarmFullSamplingC3PlusTest,
     PhysicalDwellJoinRequiresFreshOnFaceInwardContact) {
  // Canonical literals: pusher_radius 0.00555, contact_activation_tolerance
  // 0.003 (face bound 0.01410), one 20 ms execution period of staleness.
  const double radius = 0.00555;
  const double tolerance = 0.003;
  const int64_t staleness = 20000;
  const Eigen::Vector2d boundary(0.3786, 0.4196);
  const Eigen::Vector2d outward_normal(0.0, 1.0);
  const Eigen::Vector2d on_face = boundary + Eigen::Vector2d(0.004, 0.0);
  const Eigen::Vector2d inward_force(0.0, -2.0);

  const auto accepted = EvaluateXarmFullSamplingC3PhysicalDwellJoin(
      true, 1000000, 1004000, staleness, true, on_face, inward_force,
      boundary, outward_normal, radius, tolerance);
  EXPECT_TRUE(accepted.receipt_present);
  EXPECT_TRUE(accepted.receipt_fresh);
  EXPECT_TRUE(accepted.contact_active);
  EXPECT_TRUE(accepted.face_identity);
  EXPECT_TRUE(accepted.normal_polarity);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_NEAR(accepted.face_distance, 0.004, 1.0e-12);
  EXPECT_NEAR(accepted.inward_normal_force, 2.0, 1.0e-12);

  // A receipt newer than the state message is fresh, not stale.
  EXPECT_TRUE(EvaluateXarmFullSamplingC3PhysicalDwellJoin(
      true, 1006000, 1004000, staleness, true, on_face, inward_force,
      boundary, outward_normal, radius, tolerance).accepted);

  const auto stale = EvaluateXarmFullSamplingC3PhysicalDwellJoin(
      true, 1000000, 1021000, staleness, true, on_face, inward_force,
      boundary, outward_normal, radius, tolerance);
  EXPECT_FALSE(stale.receipt_fresh);
  EXPECT_FALSE(stale.accepted);

  const auto missing = EvaluateXarmFullSamplingC3PhysicalDwellJoin(
      false, 0, 1004000, staleness, true, on_face, inward_force,
      boundary, outward_normal, radius, tolerance);
  EXPECT_FALSE(missing.receipt_present);
  EXPECT_FALSE(missing.accepted);

  EXPECT_FALSE(EvaluateXarmFullSamplingC3PhysicalDwellJoin(
      true, 1000000, 1004000, staleness, false, on_face, inward_force,
      boundary, outward_normal, radius, tolerance).accepted);

  const Eigen::Vector2d off_face = boundary + Eigen::Vector2d(0.0142, 0.0);
  EXPECT_FALSE(EvaluateXarmFullSamplingC3PhysicalDwellJoin(
      true, 1000000, 1004000, staleness, true, off_face, inward_force,
      boundary, outward_normal, radius, tolerance).accepted);

  // The measured wrong-polarity class: a tiny outward normal component.
  const Eigen::Vector2d outward_force(0.0, 1.35e-4);
  const auto wrong_polarity = EvaluateXarmFullSamplingC3PhysicalDwellJoin(
      true, 1000000, 1004000, staleness, true, on_face, outward_force,
      boundary, outward_normal, radius, tolerance);
  EXPECT_FALSE(wrong_polarity.normal_polarity);
  EXPECT_FALSE(wrong_polarity.accepted);

  EXPECT_THROW(EvaluateXarmFullSamplingC3PhysicalDwellJoin(
                   true, 1000000, 1004000, 0, true, on_face, inward_force,
                   boundary, outward_normal, radius, tolerance),
               std::invalid_argument);
  EXPECT_THROW(EvaluateXarmFullSamplingC3PhysicalDwellJoin(
                   true, 1000000, 1004000, staleness, true, on_face,
                   inward_force, boundary, Eigen::Vector2d::Zero(), radius,
                   tolerance),
               std::invalid_argument);
  EXPECT_THROW(EvaluateXarmFullSamplingC3PhysicalDwellJoin(
                   true, 1000000, 1004000, staleness, true, on_face,
                   inward_force, boundary, outward_normal, radius, -0.001),
               std::invalid_argument);
}

}  // namespace
}  // namespace dairlib::oim
