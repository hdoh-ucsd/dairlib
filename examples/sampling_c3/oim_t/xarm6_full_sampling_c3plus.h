#pragma once

#include <cstdint>
#include <limits>
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

// Contact response must remain in the planar model's physical manifold. This
// uses the same resting-height and tilt bounds as planar-settle admission, but
// can be evaluated from one live spatial pose during dwell.
bool IsXarmFullSamplingC3ObjectUpright(
    const Eigen::Vector3d& position_W,
    const Eigen::Quaterniond& orientation_WO,
    double resting_height, double vertical_position_tolerance,
    double tilt_tolerance);

// Preserve the unchanged outer lateral tolerance while reserving one existing
// contact-activation band for the next physical response.
double FullSamplingC3LateralReserveLimit(
    double lateral_drift_tolerance, double contact_activation_tolerance);

// Stop a primary contact before it consumes the lateral band reserved for a
// physical correction. Equality remains admissible; nonfinite and negative
// inputs fail loudly.
bool IsFullSamplingC3LateralReserveExhausted(
    double measured_object_x, double goal_x, double lateral_reserve_limit);

// Initial execution admission is the conjunction of predicted lateral
// safety and the existing whole-capsule/live-IK acquisition receipt.  A
// reachable candidate outside the unchanged lateral corridor is diagnostic
// evidence, not an executable plan.
bool IsFullSamplingC3InitialCandidateAdmitted(
    bool lateral_gate, bool swept_capsule_clear, bool ik_reached);

// A contact acquisition completes only after the measured capsule reaches the
// sampled face.  The activation tolerance controls tracking/continuation; it
// is not a substitute for physical engagement.
bool IsFullSamplingC3ContactEngagementComplete(double signed_contact_gap);

struct XarmFullSamplingC3ContactDwellReceipt;

// After a zero-gap engagement receipt, reconstructed geometry may oscillate
// by micrometers around the unilateral boundary while Drake contact remains
// active. The existing activation band supplies hysteresis; it is not allowed
// to create engagement from a previously separated state.
bool IsFullSamplingC3LatchedContactContinuation(
    bool engagement_latched,
    const XarmFullSamplingC3ContactDwellReceipt& receipt);

// A bounded-corridor handoff may begin outside the primary reserve so that a
// corrective face can restore it.  During that contact, reject only an outer
// corridor exit or measured worsening beyond the unchanged activation band.
bool IsFullSamplingC3ContactLateralRejected(
    double measured_object_x, double goal_x,
    double cycle_entry_lateral_error, double lateral_reserve_limit,
    double lateral_drift_tolerance, double contact_activation_tolerance,
    bool bounded_recovery_mode);

// A velocity-limited posture step must make the live FK error to its command
// nonincreasing before it is sent to the controller.
bool IsFullSamplingC3PostureStepProgressive(
    double current_tip_error, double target_tip_error,
    double allowed_regression);

// Bounds an IK joint step by the per-joint velocity window while preserving
// its direction: the whole step is scaled so the most-saturated joint runs at
// its limit. Clamping each joint independently distorts the solved joint
// ratio (a truncated wrist correction under a nearly whole elbow step), which
// tilts the capsule off the vertical band and can move the executed tip away
// from its command.
Eigen::VectorXd LimitXarmFullSamplingC3JointStepPreservingDirection(
    const Eigen::VectorXd& joint_step,
    const Eigen::VectorXd& velocity_limits, double planning_time_step);

struct XarmFullSamplingC3PhysicalDwellJoinReceipt {
  bool receipt_present{};
  bool receipt_fresh{};
  bool contact_active{};
  bool face_identity{};
  bool normal_polarity{};
  bool accepted{};
  double face_distance{std::numeric_limits<double>::infinity()};
  double inward_normal_force{};
};

// Joins the simulator's live physical-contact receipt to the controller's
// selected-face transaction before any dwell credit. Face identity and
// polarity use the gates-375-379 join: the measured planar contact point must
// lie within two pusher radii plus the activation tolerance of the selected
// boundary point, and the measured force on the object must push inward
// against the selected outward normal. Missing, stale, wrong-face,
// wrong-polarity, or physically absent contact is not accepted; the
// geometric latch remains a separate continuity diagnostic.
XarmFullSamplingC3PhysicalDwellJoinReceipt
EvaluateXarmFullSamplingC3PhysicalDwellJoin(
    bool receipt_present, int64_t receipt_utime, int64_t now_utime,
    int64_t staleness_limit_us, bool raw_contact_active,
    const Eigen::Vector2d& contact_point_xy_W,
    const Eigen::Vector2d& force_on_object_xy_W,
    const Eigen::Vector2d& selected_boundary_W,
    const Eigen::Vector2d& selected_outward_normal_W,
    double pusher_radius, double contact_activation_tolerance);

// Retains the previously executable sample unless a newly ranked sample
// improves its dynamic cost by more than the same relative hysteresis used by
// the reference Sampling-C3 controller. An explicitly invalidated target is
// never retained.
bool ShouldPreserveXarmFullSamplingC3RepositionTarget(
    bool previous_available, bool previous_invalidated,
    double previous_cost, double best_cost, double hysteresis_fraction);

// Converts a Cartesian segment into a time-parameterized command while
// retaining the configured planning period as the minimum segment duration.
double XarmFullSamplingC3CommandDuration(
    double distance, double speed, double planning_dt);

// Once a corrective contact has physically engaged and cleared, that face
// owns the next release.  A productive primary plan must not overwrite the
// measured recovery reference with its older sampled face.
bool ShouldReplaceXarmFullSamplingC3ReleaseReferenceWithPrimary(
    bool progress_lateral_rejected,
    bool recovery_release_reference_active);

// The spatial sampler covers the complete vertical side mesh.  Progress and
// recovery execution use the central band so the pusher cannot realize a
// nominal planar sample as an upper/lower-edge contact.
bool IsFullSamplingC3CentralSideContact(
    double sample_height_O, double object_half_height, double pusher_radius);

enum class XarmPhysicalContactClass {
  kTipSideCandidate,
  kSideShaft,
  kTopGraze,
  kBottomGraze,
};

struct XarmPhysicalContactClassification {
  XarmPhysicalContactClass contact_class{};
  double relative_height_m{};
  double point_to_tip_m{};
};

// Classify a measured Drake point-pair contact geometrically.  This receipt is
// diagnostic: kTipSideCandidate means that the point lies in the existing
// central-side band and near the configured tip.  It does not replace the
// controller's selected-face admission or contact-continuation gates.
XarmPhysicalContactClassification ClassifyXarmPhysicalContact(
    const Eigen::Vector3d& contact_point_W,
    const Eigen::Vector3d& pusher_tip_W,
    const Eigen::Vector3d& object_center_W,
    double pusher_radius, double contact_activation_tolerance);

const char* XarmPhysicalContactClassName(XarmPhysicalContactClass value);

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
// planning-time steps to execution-update units and floored by the worst
// measured acquisition receipt when one exists (the estimate omits settle
// waits and corrective sub-steps, and a cycle admitted on the bare estimate
// has exhausted the budget mid-recovery); the release/recovery term is
// measured rather than supplied by a new tuned margin. Equality is
// intentionally insufficient so the terminal hold retains one update.
XarmFullSamplingC3CycleBudgetReceipt
EvaluateFullSamplingC3CycleBudget(
    int updates_used, int update_budget, int acquisition_ik_steps,
    double acquisition_ik_step_seconds, int execution_update_period_ms,
    int minimum_contact_steps,
    const std::vector<int>& measured_release_recovery_updates,
    int minimum_acquisition_updates = 0);

// Once authoritative physical contact-phase receipts exist, reserve their
// worst measured duration rather than silently assuming every future response
// consumes the configured dwell cap. Before the first receipt, the configured
// value remains authoritative. Release/recovery retains its independent
// worst-case measured reservation, and measured acquisition receipts floor
// the live-IK acquisition estimate the same way.
XarmFullSamplingC3CycleBudgetReceipt
EvaluateFullSamplingC3MeasuredContactPhaseCycleBudget(
    int updates_used, int update_budget, int acquisition_ik_steps,
    double acquisition_ik_step_seconds, int execution_update_period_ms,
    int configured_contact_dwell_steps,
    const std::vector<int>& measured_contact_phase_updates,
    const std::vector<int>& measured_release_recovery_updates,
    const std::vector<int>& measured_acquisition_updates = {});

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

struct XarmFullSamplingC3PlanningHandoffReceipt {
  double object_translation_drift{};
  double object_orientation_drift{};
  bool object_translation_conformant{};
  bool object_orientation_conformant{};
  bool acquisition_conformant{};
  bool accepted{};
};

// Sampling and live IK are computationally expensive while Drake continues
// publishing measured state. Revalidate both the object linearization point
// and the selected acquisition from one fresh execution-boundary snapshot.
XarmFullSamplingC3PlanningHandoffReceipt
EvaluateFullSamplingC3PlanningHandoff(
    const Eigen::Vector3d& planning_object_pose,
    const Eigen::Vector3d& measured_object_pose,
    bool swept_capsule_clear, bool ik_reached,
    double maximum_translation_drift,
    double maximum_orientation_drift);

// A released physical response that did not restore the lateral reserve must
// invalidate the candidate and retry, regardless of whether the observed
// failure was crossing, wrong polarity, or contact loss.
bool ShouldRetryXarmFullSamplingC3RecoveryResponse(
    bool release_cleared, bool lateral_recovered, bool crossed_goal,
    bool wrong_polarity, bool contact_lost, bool object_not_upright = false);

// Any failed physical acquisition or response invalidates trajectories
// linearized at the pre-execution object state. Replan only after the pusher
// is verified released so the new batch cannot inherit an active contact.
bool ShouldReplanXarmFullSamplingC3ReleasedExecutionFailure(
    bool execution_failed, bool release_cleared);

// A released pose that passes the global lateral corridor and bounded task
// transaction may seed another solve even when it misses strict Pareto credit
// or the tighter reserve. The strict receipt remains reported but is not the
// authority for a zero-credit continuation.
bool ShouldContinueXarmFullSamplingC3BoundedCorridorState(
    bool productive_cycle_credited, bool post_recovery_task_accepted,
    bool lateral_corridor_accepted, bool component_transaction_accepted,
    bool release_verified);

// A released contact with no threshold-scale response must not consume an
// already certified bounded-corridor handoff. This is a zero-credit retry:
// both task components must remain nonregressive, both measured increments
// must remain below the unchanged successor thresholds, and the object must
// remain upright and inside the unchanged outer lateral corridor.
bool ShouldContinueXarmFullSamplingC3BoundedNoResponse(
    bool prior_bounded_handoff, bool release_verified, bool object_upright,
    bool translation_nonregressive, bool orientation_nonregressive,
    double translation_progress, double orientation_progress,
    double minimum_translation_progress, double minimum_orientation_progress,
    double lateral_error, double lateral_drift_tolerance);

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

struct XarmFullSamplingC3ContactDwellReceipt {
  double tip_hold_error{};
  double signed_contact_gap{};
  bool tip_hold_conformant{};
  bool contact_gap_accepted{};
  bool contact_continuation{};
};

// A fixed contact dwell remains authoritative only while the measured tip is
// still at its commanded contact and the object-attached face has not
// separated. Both comparisons reuse the existing contact-activation
// tolerance; this receipt introduces no numerical parameter.
XarmFullSamplingC3ContactDwellReceipt
EvaluateFullSamplingC3ContactDwellContinuation(
    const Eigen::Vector3d& active_contact_target_W,
    const Eigen::Vector3d& measured_tip_W,
    const Eigen::Vector3d& measured_object_pose,
    const Eigen::Vector2d& contact_point_O,
    const Eigen::Vector2d& contact_normal_O,
    double pusher_radius, double contact_activation_tolerance);

struct XarmFullSamplingC3MeasuredResponseCompletionReceipt {
  bool contact_continuation{};
  bool object_upright{};
  bool lateral_accepted{};
  bool terminal_descent_accepted{};
  bool incremental_motion_bounded{};
  bool persistence_accepted{};
  bool completed{};
  int consecutive_bounded_updates{};
  double incremental_translation{};
  double incremental_orientation{};
  double lateral_error{};
};

// A measured contact response may complete before the configured dwell cap
// only after it has produced strict terminal descent, retained the lateral
// reserve and upright/contact guards, and remained incrementally bounded for
// one caller-supplied persistence interval. The incremental bounds reuse the
// unchanged successor progress minima; no response-speed tolerance is added.
XarmFullSamplingC3MeasuredResponseCompletionReceipt
EvaluateFullSamplingC3MeasuredResponseCompletion(
    const Eigen::Vector3d& response_start_object_pose,
    const Eigen::Vector3d& previous_object_pose,
    const Eigen::Vector3d& measured_object_pose,
    const Eigen::Vector3d& goal_pose,
    bool contact_continuation, bool object_upright,
    int prior_consecutive_bounded_updates,
    int required_persistence_updates,
    double minimum_translation_progress,
    double minimum_yaw_progress,
    double lateral_reserve_limit);

// Budget evidence is recorded only when a nonempty physical contact phase
// reaches an authoritative measured exit. Exhausting process time or merely
// falling out of the loop is not a completion receipt.
bool ShouldRecordFullSamplingC3MeasuredContactPhase(
    int contact_updates, bool contact_engaged, bool response_completed,
    bool lateral_rejected, bool terminal_regression, bool upright_rejected,
    bool contact_lost);

struct XarmFullSamplingC3LocalRepositionReceipt {
  bool local_capsule_clear{};
  bool local_ik_reached{};
  bool anchored_capsule_clear{};
  bool anchored_ik_reached{};
  bool local_accepted{};
  bool anchored_accepted{};
  bool accepted{};
  bool use_neutral_anchor{};
};

// Prefer a collision- and IK-certified local acquisition. The global
// home-derived neutral anchor remains a fail-closed fallback and is never
// selected merely because it was evaluated first.
XarmFullSamplingC3LocalRepositionReceipt
EvaluateFullSamplingC3LocalReposition(
    bool local_capsule_clear, bool local_ik_reached,
    bool anchored_capsule_clear, bool anchored_ik_reached);

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

struct XarmFullSamplingC3MeasuredPolarityReceipt {
  XarmFullSamplingC3TerminalDescentReceipt terminal;
  double measured_translation{};
  double measured_orientation{};
  bool response_observed{};
  bool wrong_polarity{};
};

// Recovery may stop at the lateral reserve only after a measured response has
// been observed and neither global terminal component has moved away from its
// goal. Existing successor minima define response observability.
XarmFullSamplingC3MeasuredPolarityReceipt
EvaluateFullSamplingC3MeasuredRecoveryPolarity(
    const Eigen::Vector3d& recovery_start_object_pose,
    const Eigen::Vector3d& measured_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double minimum_translation_motion,
    double minimum_orientation_motion);

struct XarmFullSamplingC3RecoveryAdmissionReceipt {
  XarmFullSamplingC3TerminalDescentReceipt terminal;
  bool translation_debt_bounded{};
  bool orientation_debt_bounded{};
  bool accepted{};
};

// The contact model may predict a terminal-task debt while restoring x.
// User-authorized reconciliation (option a, 2026-09-02): recovery candidates
// receive the same trade allowance component-decomposed progress
// transactions already receive — at most one unchanged terminal tolerance of
// predicted debt per component — with the lateral-recovery receipt gating
// the active purpose. The measured response remains subject to strict
// zero-debt polarity rejection, so an admitted prediction that regresses in
// reality is still released and replanned.
XarmFullSamplingC3RecoveryAdmissionReceipt
EvaluateFullSamplingC3RecoveryCandidateAdmission(
    const Eigen::Vector3d& recovery_start_object_pose,
    const Eigen::Vector3d& predicted_object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double maximum_translation_debt,
    double maximum_orientation_debt);

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
  // 0: replicated local observations compatible, 1: insufficient evidence,
  // 2: replicated observations include incompatible physical response.
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

// One contact is an observation, not a repeatable response class. Require one
// replication before physical evidence changes candidate precedence.
inline constexpr int kMinimumResponseClassObservations = 2;

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
  // DAIRLab kSimLCSReplaceC3EEPlan equivalent: dynamically re-simulated
  // object states with the C3 plan's pusher position/velocity restored for
  // physical execution. Keep this distinct from the residual-certified
  // dynamic rollout above.
  std::vector<Eigen::VectorXd> execution_state_trajectory;
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

struct XarmFullSamplingC3PredictedContactPrefixReceipt {
  bool accepted{};
  int checked_state_knots{};
  double maximum_absolute_signed_gap{};
  double maximum_separating_gap{};
  double maximum_contact_height_error{};
};

// Verifies that a retained execution prefix remains on its sampled
// object-attached face. This catches spatial rollouts whose object motion is
// corridor-safe but whose pusher lifts away from, or crosses through, the
// intended contact. An inward target is allowed because the physical object
// supplies the unilateral constraint; only positive separation is rejected.
// The existing contact-activation tolerance is used for the separating gap
// and contact-height error.
XarmFullSamplingC3PredictedContactPrefixReceipt
EvaluateXarmFullSamplingC3PredictedContactPrefix(
    const XarmFullSamplingC3CandidateReceipt& candidate,
    double pusher_radius, double contact_activation_tolerance);

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
  Eigen::MatrixXd pusher_forces_W;
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
  Eigen::MatrixXd forces_W;
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

struct XarmFullSamplingC3OpenTableTerminalReceipt {
  double translation_error{};
  double orientation_error{};
  bool translation_accepted{};
  bool orientation_accepted{};
  bool accepted{};
};

// Gate 100 is the unchanged open_table definition: planar Euclidean position
// error and wrapped yaw error must independently satisfy their configured
// tolerances. Productive-cycle provenance cannot override either component.
XarmFullSamplingC3OpenTableTerminalReceipt
EvaluateXarmFullSamplingC3OpenTableTerminal(
    const Eigen::Vector3d& object_pose,
    const Eigen::Vector3d& goal_object_pose,
    double translation_tolerance, double orientation_tolerance);

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
