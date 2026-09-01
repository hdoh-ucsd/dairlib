#pragma once

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "examples/sampling_c3/parameter_headers/oim_t_params.h"

namespace dairlib::oim {

// The reduced planner remains the default until every full Sampling-C3+ gate
// has passed. This enum is the executable's explicit compatibility boundary.
enum class XarmSamplingC3PlannerMode {
  kReducedExactT,
  kFullSamplingC3Plus,
};

XarmSamplingC3PlannerMode ParseXarmSamplingC3PlannerMode(
    const std::string& mode);
std::string XarmSamplingC3PlannerModeName(
    XarmSamplingC3PlannerMode mode);

// The open_table LCS constrains object roll, pitch, and vertical translation
// through table contact. Preserve measured planar motion while removing Drake
// contact-chatter in those three constrained coordinates at the model boundary.
std::pair<Eigen::Vector3d, Eigen::Vector3d>
ConditionOpenTableObjectVelocity(
    const Eigen::Vector3d& angular_velocity_W,
    const Eigen::Vector3d& linear_velocity_W);

struct XarmFullSamplingC3PlanarSettleReceipt {
  bool accepted{};
  double planar_translation_delta{};
  double yaw_delta{};
  double vertical_position_error{};
  double tilt_angle{};
};

// Instantaneous Drake contact velocities contain table-compliance chatter.
// Admit a new planar solve only after consecutive measured poses are stable
// and the T remains upright at its configured resting height.
XarmFullSamplingC3PlanarSettleReceipt
EvaluateXarmFullSamplingC3PlanarSettle(
    const Eigen::Vector3d& previous_position_W,
    const Eigen::Quaterniond& previous_orientation_WO,
    const Eigen::Vector3d& current_position_W,
    const Eigen::Quaterniond& current_orientation_WO,
    double resting_height, double planar_translation_tolerance,
    double yaw_tolerance, double vertical_position_tolerance,
    double tilt_tolerance);

// Preserve the unchanged outer lateral tolerance while reserving one existing
// contact-activation band for the next physical response.
double FullSamplingC3LateralReserveLimit(
    double lateral_drift_tolerance, double contact_activation_tolerance);

// The spatial sampler covers the complete vertical side mesh.  Progress and
// recovery execution use the central band so the pusher cannot realize a
// nominal planar sample as an upper/lower-edge contact.
bool IsFullSamplingC3CentralSideContact(
    double sample_height_O, double object_half_height, double pusher_radius);

// A new contact cycle is not admitted unless the remaining measured-update
// budget can still contain its unchanged minimum contact dwell. Repositioning
// consumes additional updates, so equality is intentionally insufficient.
bool HasFullSamplingC3ContactDwellBudget(
    int updates_used, int update_budget, int minimum_contact_steps);

struct XarmFullSamplingC3CycleBudgetReceipt {
  bool accepted{};
  int remaining_updates{};
  int acquisition_updates{};
  int contact_dwell_updates{};
  int release_recovery_updates{};
  int required_updates{};
  int measured_release_recovery_receipts{};
};

// Admit a selected physical cycle only when its live-IK acquisition estimate,
// unchanged contact dwell, and worst observed release/recovery receipt all fit
// in the remaining measured-update budget. The IK estimate is converted from
// planning-time steps to execution-update units; the release/recovery term is
// measured rather than supplied by a new tuned margin. Equality is
// intentionally insufficient so the terminal hold retains one update.
XarmFullSamplingC3CycleBudgetReceipt
EvaluateFullSamplingC3CycleBudget(
    int updates_used, int update_budget, int acquisition_ik_steps,
    double acquisition_ik_step_seconds, int execution_update_period_ms,
    int minimum_contact_steps,
    const std::vector<int>& measured_release_recovery_updates);

struct XarmFullSamplingC3AcquisitionConformanceReceipt {
  bool preview_accepted{};
  int expected_phases{};
  int completed_phases{};
  bool physical_acquisition_completed{};
  bool recovery_required{};
  bool candidate_invalidated{};
  bool neutral_anchor_reacquired{};
  bool terminal_receipt_preserved{};
  bool replanning_allowed{};
};

// A candidate admitted by live IK may be executed directly only when every
// physical acquisition phase completes. If measured execution diverges, the
// candidate must be invalidated and replanning remains fail-closed until the
// verified neutral anchor is physically reacquired without discarding the
// completed terminal-descent evidence from the preceding cycle.
XarmFullSamplingC3AcquisitionConformanceReceipt
EvaluateFullSamplingC3AcquisitionConformance(
    bool preview_accepted, int expected_phases, int completed_phases,
    bool candidate_invalidated, bool neutral_anchor_reacquired,
    bool terminal_receipt_preserved);

// Reject task-space tracking that moves farther from the active waypoint by
// more than the existing contact-activation tolerance. This detects physical
// execution leaving a live-IK preview without introducing another threshold.
bool IsFullSamplingC3WaypointExecutionConformant(
    double phase_entry_waypoint_error, double measured_waypoint_error,
    double contact_activation_tolerance);

struct XarmFullSamplingC3WaypointConformanceReceipt {
  int consecutive_violation_updates{};
  bool persistent_violation{};
};

// A single control observation can cross the spatial conformance boundary
// during normal settling. Preserve that boundary, but invalidate execution
// only when it remains crossed for one planning interval.
XarmFullSamplingC3WaypointConformanceReceipt
EvaluateFullSamplingC3WaypointConformancePersistence(
    bool spatially_conformant, int prior_consecutive_violation_updates,
    int required_consecutive_violation_updates);

// Revolute IK may return q +/- 2*pi at identical Cartesian posture. For joints
// whose declared range contains a complete revolution, select the equivalent
// in-limit representation nearest measured q before forming a bounded step.
Eigen::VectorXd CanonicalizeXarmPeriodicIkSolutionNearestMeasured(
    const Eigen::VectorXd& solution, const Eigen::VectorXd& measured,
    const Eigen::VectorXd& lower_limits,
    const Eigen::VectorXd& upper_limits);

// Form a measurement-conditioned vertical translation command. The caller's
// waypoint contributes only its z coordinate; x-y is latched from the latest
// measured tip so a measurement refresh cannot create an unintended diagonal
// recovery command.
Eigen::Vector3d BuildMeasuredVerticalTranslationSubtarget(
    const Eigen::Vector3d& measured_tip, double waypoint_z,
    double task_space_step_limit);

// A physical waypoint is a safe handoff only when it is geometrically reached
// and its measured tip motion cannot leave the existing activation ball over
// one unchanged planning interval.
bool IsFullSamplingC3WaypointSettled(
    double waypoint_error, double measured_tip_speed,
    double planning_time_step, double contact_activation_tolerance);

bool IsFullSamplingC3WrongPolarityResponse(
    double start_signed_error, double measured_signed_error,
    int response_steps, int minimum_contact_steps);

struct XarmFullSamplingC3TerminalDescentReceipt {
  bool translation_nonregressive{};
  bool orientation_nonregressive{};
  bool minimum_progress{};
  bool accepted{};
  double translation_progress{};
  double orientation_progress{};
};

// A cycle may improve either terminal component by its existing minimum, but
// it may not purchase that progress by worsening the other component. This is
// a parameter-free Pareto gate over the unchanged global open_table errors.
XarmFullSamplingC3TerminalDescentReceipt
EvaluateFullSamplingC3TerminalDescent(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& end_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double minimum_translation_progress,
    double minimum_orientation_progress);

struct XarmFullSamplingC3NormalizedParetoDescentReceipt {
  XarmFullSamplingC3TerminalDescentReceipt terminal;
  double normalized_magnitude{};
};

// Compare task descent across meters and radians without adding a new weight:
// each component is normalized by its unchanged terminal tolerance.
XarmFullSamplingC3NormalizedParetoDescentReceipt
EvaluateFullSamplingC3NormalizedParetoDescent(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& end_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance,
    double orientation_tolerance);

struct XarmFullSamplingC3ComponentTransactionReceipt {
  XarmFullSamplingC3TerminalDescentReceipt terminal;
  double normalized_magnitude{};
  bool translation_debt_bounded{};
  bool orientation_debt_bounded{};
  bool accepted{};
};

// Component-decomposed planning may spend at most one unchanged terminal
// tolerance in the inactive component when total normalized descent is
// positive and the active component reaches an existing progress minimum.
// Physical productive-cycle credit remains governed by strict Pareto descent.
XarmFullSamplingC3ComponentTransactionReceipt
EvaluateXarmFullSamplingC3ComponentTransaction(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& end_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance, double orientation_tolerance,
    double minimum_translation_progress, double minimum_orientation_progress);

struct XarmFullSamplingC3PostRecoveryReceipt {
  XarmFullSamplingC3TerminalDescentReceipt terminal;
  bool lateral_accepted{};
  bool accepted{};
  double lateral_error{};
};

// Productivity belongs to the measured state after the complete corrective
// transaction, not to the transient pose that triggered recovery.
XarmFullSamplingC3PostRecoveryReceipt
EvaluateFullSamplingC3PostRecoveryProgress(
    const Eigen::Vector3d& start_object_pose,
    const Eigen::Vector3d& post_recovery_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double lateral_drift_tolerance,
    double minimum_translation_progress,
    double minimum_orientation_progress);

// A physical contact response is retained in measured coordinates together
// with the model prediction that selected it.  The contact point and normal
// identify a local face neighborhood without depending on provider-specific
// sample names.
struct XarmFullSamplingC3MeasuredResponse {
  Eigen::Vector3d start_object_pose{Eigen::Vector3d::Zero()};
  Eigen::Vector3d predicted_terminal_object_pose{Eigen::Vector3d::Zero()};
  Eigen::Vector3d measured_terminal_object_pose{Eigen::Vector3d::Zero()};
  Eigen::Vector2d sample_point_O{Eigen::Vector2d::Zero()};
  Eigen::Vector2d sample_normal_O{Eigen::Vector2d::Zero()};
  bool lateral_rejected{};
};

struct XarmFullSamplingC3ResponseConditioningReceipt {
  int matching_observations{};
  int compatible_observations{};
  int lateral_rejections{};
  int terminal_regressions{};
  // 0: all local observations compatible, 1: unseen, 2: incompatible.
  // This discrete class intentionally avoids inventing a scalar tuning weight.
  int ranking_class{1};
  bool corrected_terminal_accepted{};
  bool corrected_lateral_accepted{};
  Eigen::Vector3d mean_prediction_residual{Eigen::Vector3d::Zero()};
  Eigen::Vector3d corrected_terminal_object_pose{Eigen::Vector3d::Zero()};
  double corrected_lateral_error{};
  // Dimensionless task descent uses the unchanged translation and
  // orientation tolerances. The gain is measured/predicted local descent;
  // calibrated magnitude applies that gain to the current candidate.
  double observed_progress_gain{};
  double calibrated_normalized_magnitude{};
};

// Corrects one candidate's predicted terminal displacement with accumulated
// physical model residuals from the same pose/contact neighborhood. Existing
// task tolerances define both neighborhoods; no learned threshold or weight is
// introduced. Invalid inputs fail loudly rather than silently changing order.
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
    double minimum_orientation_progress);

// Recovery contacts are attached to the object. Expressing their model
// residual in O and rotating it into the current W permits evidence reuse
// after large yaw changes. Contact neighborhoods and task acceptance still
// use the unchanged terminal tolerances.
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
    double minimum_orientation_progress);

// One-object DAIRLab Sampling-C3 state layout:
//   q = [pusher xyz, object quaternion wxyz, object xyz]
//   v = [pusher linear xyz, object angular xyz, object linear xyz]
// Explicit indices prevent silent drift as spatial contacts are introduced.
struct XarmFullSamplingC3State {
  static constexpr int kPusherPosition = 0;
  static constexpr int kObjectQuaternion = 3;
  static constexpr int kObjectPosition = 7;
  static constexpr int kPusherLinearVelocity = 10;
  static constexpr int kObjectAngularVelocity = 13;
  static constexpr int kObjectLinearVelocity = 16;
  static constexpr int kNumPositions = 10;
  static constexpr int kNumVelocities = 9;
  static constexpr int kSize = kNumPositions + kNumVelocities;
  static constexpr int kInputSize = 3;

  Eigen::Vector3d pusher_position_W{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond object_quaternion_WO{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d object_position_W{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pusher_linear_velocity_W{Eigen::Vector3d::Zero()};
  Eigen::Vector3d object_angular_velocity_W{Eigen::Vector3d::Zero()};
  Eigen::Vector3d object_linear_velocity_W{Eigen::Vector3d::Zero()};

  Eigen::VectorXd Encode() const;
  static XarmFullSamplingC3State Decode(const Eigen::VectorXd& state);
};

struct XarmFullSamplingC3LcsReceipt {
  bool finite{};
  int num_states{};
  int num_inputs{};
  int num_contact_pairs{};
  int num_contact_variables{};
  int horizon{};
  double dt{};
  double complementarity_offset_min{};
  double complementarity_offset_max{};
};

// Builds the full spatial pusher/T/ground LCS using DAIRLab's multibody
// factory. This is an inspection gate only; C3+ execution is enabled later,
// after numerical residual thresholds are fixed and verified.
XarmFullSamplingC3LcsReceipt BuildXarmFullSamplingC3SpatialLcsWitness(
    const OimTParams& params);

struct XarmFullSamplingC3SolveReceipt {
  bool finite{};
  bool accepted{};
  bool dynamic_rollout_accepted{};
  bool dynamic_rollout_lcp_solved{};
  bool dynamic_rollout_workspace_accepted{};
  double elapsed{};
  double diagnostic_elapsed{};
  double lambda_scale{};
  double initial_state_residual{};
  double dynamics_residual{};
  double returned_plan_dynamics_residual{};
  double equality_residual{};
  double nonnegative_residual{};
  double complementarity_residual{};
  double projected_nonnegative_residual{};
  double projected_complementarity_residual{};
  double consensus_residual{};
  double dynamic_rollout_dynamics_residual{};
  double dynamic_rollout_nonnegative_residual{};
  double dynamic_rollout_complementarity_residual{};
  double dynamic_rollout_workspace_violation{};
  double planned_input_bound_violation{};
  double dynamic_rollout_cost{};
  Eigen::Vector3d dynamic_terminal_pusher_position{
      Eigen::Vector3d::Zero()};
  Eigen::Vector3d dynamic_terminal_object_position{
      Eigen::Vector3d::Zero()};
  std::vector<Eigen::VectorXd> dynamic_state_trajectory;
  std::vector<Eigen::VectorXd> planned_input_trajectory;
};

XarmFullSamplingC3SolveReceipt RunXarmFullSamplingC3FirstSolve(
    const OimTParams& params);

// Deterministic exact-T samples are expressed in the object's planar frame.
// Their ordering and geometry are identical to the retained reduced planner,
// providing a traceable first sampling provider before stochastic/mesh
// providers are enabled.
struct XarmFullSamplingC3ContactSample {
  Eigen::Vector2d point_O{Eigen::Vector2d::Zero()};
  Eigen::Vector2d outward_normal_O{Eigen::Vector2d::Zero()};
  std::string name;
  double height_O{};
};

const std::vector<XarmFullSamplingC3ContactSample>&
GetXarmFullSamplingC3ExactTSamples();

std::vector<XarmFullSamplingC3ContactSample>
GenerateXarmFullSamplingC3PerimeterSamples(int count, int seed);

std::vector<XarmFullSamplingC3ContactSample>
GenerateXarmFullSamplingC3MeshNormalSamples(int count, int seed);

// Deterministic local refinement of the long stem-left face. This provider
// resolves the narrow intersection between the xArm reachability boundary and
// the unchanged object-x drift gate without changing a random seed.
std::vector<XarmFullSamplingC3ContactSample>
GenerateXarmFullSamplingC3StemLeftRefinementSamples(int count);

std::vector<XarmFullSamplingC3ContactSample>
GenerateXarmFullSamplingC3StemRightRefinementSamples(int count);

struct XarmFullSamplingC3CandidateReceipt {
  int sample_index{-1};
  std::string sample_name;
  Eigen::Vector3d initial_pusher_position_W{Eigen::Vector3d::Zero()};
  Eigen::Vector2d sample_point_O{Eigen::Vector2d::Zero()};
  Eigen::Vector2d sample_normal_O{Eigen::Vector2d::Zero()};
  double sample_height_O{};
  XarmFullSamplingC3SolveReceipt solve;
};

struct XarmFullSamplingC3CorridorPrefixReceipt {
  bool accepted{};
  int original_state_knots{};
  int retained_state_knots{};
  double terminal_lateral_error{};
  XarmFullSamplingC3CandidateReceipt execution_candidate;
};

// Converts a full-horizon solver candidate into its longest contiguous
// receding-horizon execution prefix whose predicted object x remains inside
// the unchanged lateral corridor. The solver receipt and full candidate are
// inputs by value and remain available to the caller for scientific tracing;
// only the explicitly named execution copy is shortened.
XarmFullSamplingC3CorridorPrefixReceipt
BuildXarmFullSamplingC3CorridorSafePrefix(
    const XarmFullSamplingC3CandidateReceipt& candidate,
    double goal_object_x, double lateral_drift_tolerance);

struct XarmFullSamplingC3BatchReceipt {
  bool accepted{};
  int num_samples{};
  int num_feasible{};
  int selected_index{-1};
  std::string selected_name;
  double selected_cost{};
  std::vector<XarmFullSamplingC3CandidateReceipt> candidates;
};

// Solves and dynamically re-simulates every deterministic contact sample.
// Only residual-accepted dynamic rollouts participate in cost ranking.
XarmFullSamplingC3BatchReceipt RunXarmFullSamplingC3ExactTBatch(
    const OimTParams& params);

XarmFullSamplingC3BatchReceipt RunXarmFullSamplingC3PerimeterBatch(
    const OimTParams& params);

XarmFullSamplingC3BatchReceipt RunXarmFullSamplingC3PerimeterBatchParallel(
    const OimTParams& params);

XarmFullSamplingC3BatchReceipt RunXarmFullSamplingC3MeshBatchParallel(
    const OimTParams& params);

XarmFullSamplingC3BatchReceipt
RunXarmFullSamplingC3StemLeftRefinementBatchParallel(
    const OimTParams& params);

XarmFullSamplingC3BatchReceipt
RunXarmFullSamplingC3StemRightRefinementBatchParallel(
    const OimTParams& params);

struct XarmFullSamplingC3CandidateBuffer {
  bool accepted{};
  int total_candidates{};
  std::vector<XarmFullSamplingC3CandidateReceipt> successful;
  std::vector<XarmFullSamplingC3CandidateReceipt> unsuccessful;
};

// Merges provider batches without re-solving. Successful candidates are
// stably ordered by dynamic rollout cost; rejected candidates remain visible.
XarmFullSamplingC3CandidateBuffer BuildXarmFullSamplingC3CandidateBuffer(
    const std::vector<XarmFullSamplingC3BatchReceipt>& batches);

// Replenishment view used only after the workspace-filtered buffer is empty.
// It retains dynamically accepted, input-bounded contacts for mandatory live
// xArm IK and whole-capsule validation; it does not reclassify solver failures.
XarmFullSamplingC3CandidateBuffer
BuildXarmFullSamplingC3ContactFeasibleCandidateBuffer(
    const XarmFullSamplingC3CandidateBuffer& workspace_filtered_buffer);

struct XarmFullSamplingC3TaskSpacePlan {
  bool accepted{};
  std::string sample_name;
  double cost{};
  Eigen::VectorXd time_vector;
  Eigen::MatrixXd pusher_positions_W;
  Eigen::MatrixXd pusher_velocities_W;
  Eigen::Vector2d sample_point_O{Eigen::Vector2d::Zero()};
  Eigen::Vector2d sample_normal_O{Eigen::Vector2d::Zero()};
  double sample_height_O{};
};

XarmFullSamplingC3TaskSpacePlan BuildXarmFullSamplingC3TaskSpacePlan(
    const XarmFullSamplingC3CandidateBuffer& buffer, double dt);

// Applies the unchanged object-x tolerance before cost ranking. Numerically
// valid candidates that fail this task-level gate remain preserved in the
// input buffer and are not reclassified as solver failures.
XarmFullSamplingC3TaskSpacePlan
BuildXarmFullSamplingC3LateralGuardedTaskSpacePlan(
    const XarmFullSamplingC3CandidateBuffer& buffer, double dt,
    double goal_object_x, double lateral_drift_tolerance);

struct XarmFullSamplingC3OscExecutionPlan {
  bool accepted{};
  Eigen::VectorXd time_vector;
  Eigen::MatrixXd positions_W;
  int acquisition_knots{};
  int c3_knots{};
};

XarmFullSamplingC3OscExecutionPlan BuildXarmFullSamplingC3OscExecutionPlan(
    const Eigen::Vector3d& current_tip_W,
    const Eigen::Vector3d& planar_waypoint_W,
    const Eigen::Vector3d& elevated_waypoint_W,
    const Eigen::Vector3d& standoff_waypoint_W,
    const XarmFullSamplingC3TaskSpacePlan& c3_plan,
    double reposition_speed, double c3_dt);

struct XarmFullSamplingC3TerminalStatus {
  bool closed_loop_handoff{};
  bool accepted{};
  int return_code{};
  std::string reason;
};

// Separates cumulative physical handoff provenance from terminal task
// acceptance. A later measured-budget defer must not erase an earlier
// productive cycle, while the unchanged pose tolerances remain authoritative.
XarmFullSamplingC3TerminalStatus EvaluateXarmFullSamplingC3TerminalStatus(
    int reached_initial_waypoints, int total_initial_waypoints,
    bool any_productive_cycle, bool cycle_budget_deferred,
    int execution_updates, int execution_budget, bool terminal_pose_accepted);

struct XarmFullSamplingC3ParetoWrenchReceipt {
  double translation_alignment{};
  double orientation_alignment{};
  bool translation_nonregressive{};
  bool orientation_nonregressive{};
  bool minimum_productivity{};
  bool accepted{};
};

// Applies the terminal Pareto contract to an instantaneous planar wrench:
// neither task component may oppose its desired direction, and at least one
// component must be strictly productive. No magnitude threshold is introduced.
XarmFullSamplingC3ParetoWrenchReceipt EvaluateXarmFullSamplingC3ParetoWrench(
    double desired_translation_direction, double force_translation_component,
    double desired_orientation_direction, double planar_moment);

struct XarmFullSamplingC3LateralRecoveryReceipt {
  double lateral_error{};
  double lateral_reduction{};
  double translation_progress{};
  double orientation_debt{};
  double normalized_magnitude{};
  bool lateral_restored{};
  bool translation_nonregressive{};
  bool translation_debt_bounded{};
  bool orientation_debt_bounded{};
  bool accepted{};
};

// A recovery transaction may temporarily spend at most the corresponding
// unchanged terminal translation/orientation tolerance only when it predicts
// restoration of the existing lateral corridor. Measured post-recovery
// productive-cycle acceptance remains a separate, strictly nonregressive
// authority.
XarmFullSamplingC3LateralRecoveryReceipt
EvaluateXarmFullSamplingC3LateralRecovery(
    const Eigen::Vector3d& cycle_start_object_pose,
    const Eigen::Vector3d& rejected_object_pose,
    const Eigen::Vector3d& recovery_terminal_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance, double orientation_tolerance,
    double lateral_drift_tolerance);

struct XarmFullSamplingC3LateralEntryReceipt {
  bool accepted{};
  int state_knot{-1};
  Eigen::Vector3d object_pose{Eigen::Vector3d::Zero()};
};

// Returns the first predicted state inside the unchanged object-x corridor.
// Recovery execution already stops on this measured event; selection must not
// judge a later five-knot overshoot as though it were the commanded endpoint.
XarmFullSamplingC3LateralEntryReceipt
FindXarmFullSamplingC3LateralCorridorEntry(
    const XarmFullSamplingC3CandidateReceipt& candidate,
    double goal_object_x, double lateral_drift_tolerance);

struct XarmFullSamplingC3MeasuredCycleReceipt {
  double translation_progress{};
  double orientation_progress{};
  int updates{};
};

struct XarmFullSamplingC3TerminalBudgetEstimate {
  bool finite{};
  double required_translation_progress{};
  double required_orientation_progress{};
  double maximum_measured_translation_progress{};
  double maximum_measured_orientation_progress{};
  int minimum_measured_cycle_updates{};
  int optimistic_remaining_cycles{};
  int64_t optimistic_remaining_updates{};
};

struct XarmFullSamplingC3TerminalBudgetSufficiencyReceipt {
  int remaining_updates{};
  int64_t optimistic_required_updates{};
  bool finite_estimate{};
  bool sufficient{};
};

// Produces an explicit optimistic lower bound, not a success prediction: it
// combines the best observed progress in each task component with the cheapest
// complete productive cycle and the unchanged terminal tolerances.
XarmFullSamplingC3TerminalBudgetEstimate
EstimateXarmFullSamplingC3TerminalBudget(
    const Eigen::Vector3d& current_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance, double orientation_tolerance,
    const std::vector<XarmFullSamplingC3MeasuredCycleReceipt>& cycles);

// A PASS is necessary, never sufficient, for terminal success. This compares
// a requested run budget with the measured optimistic lower bound and does not
// alter per-cycle admission or terminal acceptance.
XarmFullSamplingC3TerminalBudgetSufficiencyReceipt
EvaluateXarmFullSamplingC3TerminalBudgetSufficiency(
    int updates_used, int update_budget,
    const XarmFullSamplingC3TerminalBudgetEstimate& estimate);

}  // namespace dairlib::oim
