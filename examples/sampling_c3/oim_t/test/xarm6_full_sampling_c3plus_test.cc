#include "examples/sampling_c3/oim_t/xarm6_full_sampling_c3plus.h"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include <drake/common/yaml/yaml_io.h>

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

TEST(XarmFullSamplingC3PlusTest, LateralReserveKeepsActivationMargin) {
  EXPECT_DOUBLE_EQ(FullSamplingC3LateralReserveLimit(0.005, 0.003),
                   0.002);
  EXPECT_DOUBLE_EQ(FullSamplingC3LateralReserveLimit(0.002, 0.003), 0.0);
  EXPECT_THROW(FullSamplingC3LateralReserveLimit(-0.005, 0.003),
               std::invalid_argument);
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

TEST(XarmFullSamplingC3PlusTest, ContactCycleRequiresCompleteDwellBudget) {
  EXPECT_TRUE(HasFullSamplingC3ContactDwellBudget(623, 2000, 1000));
  EXPECT_FALSE(HasFullSamplingC3ContactDwellBudget(1000, 2000, 1000));
  EXPECT_FALSE(HasFullSamplingC3ContactDwellBudget(1937, 2000, 1000));
  EXPECT_THROW(HasFullSamplingC3ContactDwellBudget(2001, 2000, 1000),
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
  EXPECT_THROW(IsFullSamplingC3WaypointExecutionConformant(
                   -1.0, 0.0, 0.003),
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
  EXPECT_EQ(preferred.ranking_class, 0);
  EXPECT_EQ(preferred.compatible_observations, 1);
  EXPECT_TRUE(preferred.corrected_terminal_accepted);
  EXPECT_TRUE(preferred.corrected_lateral_accepted);
  EXPECT_TRUE(preferred.corrected_terminal_object_pose.isApprox(
      compatible.measured_terminal_object_pose));

  XarmFullSamplingC3MeasuredResponse incompatible = compatible;
  incompatible.measured_terminal_object_pose =
      Eigen::Vector3d(0.02, -0.01, -0.01);
  incompatible.lateral_rejected = true;
  const auto demoted =
      EvaluateFullSamplingC3MeasuredResponseConditioning(
          start, predicted, point, normal, goal, {incompatible}, 0.05, 0.10,
          0.01, 0.002, 0.002);
  EXPECT_EQ(demoted.ranking_class, 2);
  EXPECT_EQ(demoted.lateral_rejections, 1);
  EXPECT_EQ(demoted.terminal_regressions, 1);
  EXPECT_FALSE(demoted.corrected_terminal_accepted);
  EXPECT_FALSE(demoted.corrected_lateral_accepted);

  incompatible.start_object_pose.y() = 0.2;
  const auto outside_neighborhood =
      EvaluateFullSamplingC3MeasuredResponseConditioning(
          start, predicted, point, normal, goal, {incompatible}, 0.05, 0.10,
          0.01, 0.002, 0.002);
  EXPECT_EQ(outside_neighborhood.ranking_class, 1);
  EXPECT_EQ(outside_neighborhood.matching_observations, 0);
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

}  // namespace
}  // namespace dairlib::oim
