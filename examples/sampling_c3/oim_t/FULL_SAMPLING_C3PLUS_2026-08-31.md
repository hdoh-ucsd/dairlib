# Full Sampling-C3+ implementation record — 2026-08-31

This record starts the spatial, multi-contact Sampling-C3+ implementation for
the OIM T-on-table xArm scenario. The existing reduced exact-T controller is
retained as the default compatibility path. Previously reported run records
and numerical results are not modified by this work.

Canonical files, Bazel targets, and produced binaries now use the explicit
`xarm6_*` prefix. Earlier command blocks below are immutable historical run
records; their legacy Bazel labels remain aliases to the corresponding
`xarm6_*` targets, while new direct `bazel-bin` commands must use `xarm6_*`.

## Gate 0 — baseline provenance

```text
worktree:       /root/push_anything_ADMM/reference_repos/oim_c++_anything
branch:         oim_c++_anything
source commit:  f5c53479c6b7bb9e0a5d58d29af2c6c2b10f8bce
dirty state:    pre-existing OIM/xArm and FastOSQP modifications preserved
config:         examples/sampling_c3/oim_t/parameters/oim_t.yaml
config SHA-256: dd7c8e2e9e7ffa8d75d5b2f7109d57e6a96ddafb432bcf14d2b7214da6cc55cd
seed:           not applicable; deterministic exact-T enumeration
platform:       Linux 6.18.33.2-microsoft-standard-WSL2 x86_64
compiler:       g++ 13.3.0
Bazel:          8.4.0
Drake:          v1.51.1
C3:             5c08cb2e14b1ab10e024cb46e8504970cffcd5ea
```

Baseline command:

```sh
bazel build --jobs=8 \
  //examples/sampling_c3/oim_t:oim_t_config_check \
  //examples/sampling_c3/oim_t:xarm_sim \
  //examples/sampling_c3/oim_t:xarm_osc_controller \
  //examples/sampling_c3/oim_t:xarm_sampling_c3_controller
bazel-bin/examples/sampling_c3/oim_t/oim_t_config_check
/usr/bin/time -v \
  bazel-bin/examples/sampling_c3/oim_t/xarm_sampling_c3_controller \
  --first_solve_only=true
bazel test //solvers:fast_osqp_solver_test \
  //solvers:cost_constraint_approximation_test
```

Result before the Full Sampling-C3+ additions:

```text
four OIM build targets:       PASS
configuration validation:    PASS
deterministic witnesses:      PASS
first stored object xy/yaw:   0.381, 0.349173, -3.77079e-22
first contact gap:            0.315961 m
first solve elapsed:          1.1988 ms
first-solve peak RSS:         73832 KiB
solver regressions:           2 / 2 PASS
terminal open_table baseline: FAIL (preserved prior result)
```

## Gate 1/2 implementation boundary

`--planner_mode=reduced_exact_t` remains the default. The new
`--planner_mode=full_sampling_c3plus` path initially exposes and validates the
one-object DAIRLab-compatible 19-state spatial layout and three-dimensional
input layout. Live execution intentionally stops until the spatial LCS and
residual-acceptance gates pass.

Validation command and result:

```sh
bazel test \
  //examples/sampling_c3/oim_t:xarm_full_sampling_c3plus_test
bazel build --jobs=8 \
  //examples/sampling_c3/oim_t:oim_t_config_check \
  //examples/sampling_c3/oim_t:xarm_sim \
  //examples/sampling_c3/oim_t:xarm_osc_controller \
  //examples/sampling_c3/oim_t:xarm_sampling_c3_controller
bazel-bin/examples/sampling_c3/oim_t/xarm_sampling_c3_controller \
  --first_solve_only=true --planner_mode=reduced_exact_t
bazel-bin/examples/sampling_c3/oim_t/xarm_sampling_c3_controller \
  --first_solve_only=true --planner_mode=full_sampling_c3plus
```

```text
spatial codec tests:          3 / 3 PASS
all four OIM build targets:   PASS
reduced first prediction:     0.381, 0.349173, -3.77079e-22 (unchanged)
reduced contact gap:          0.315961 m (unchanged)
full spatial state gate:      PASS
full state/input dimensions:  19 / 3
full live execution:          gated off pending spatial LCS acceptance
```

## Gates 3–6 — spatial input, multi-contact LCS, and residual audit

The full path now builds an xArm-specific task-space plant with three
translational pusher actuators, a 5.55 mm pusher collision sphere, the exact
two-box OIM T, and the table top at world z=0. It does not use the Panda
helper's 19.5 mm sphere or its -29 mm ground offset. The five contact pairs are
pusher-ground, two pusher-T pairs, and two T-ground pairs. With two Anitescu
friction directions per pair, the LCS contains 20 contact variables.

All full C3+ parameters and the unchanged Push-T pose cost are now explicit in
`oim_t.yaml`. Residual thresholds were fixed before the first solve. The first
contact-mode state uses the exact stem-bottom sample rather than pretending the
distant home pusher is already in a contact mode.

```text
current config SHA-256:       532075ac88874c3c6e6562271db3b08f4d1a570e4bab8553e936273d51aa60c5
spatial LCS dimensions:       19 states / 3 inputs / 5 pairs / 20 variables
horizon / planning dt:        5 / 0.05 s
QP initial-state residual:    1.11022e-16
QP dynamics residual:         1.77636e-15
QP eta equality residual:     4.61853e-14
projected nonnegative error:  0
projected complementarity:    0
QP nonnegative error:         28.5034
QP complementarity residual: 602.537
QP/projection consensus:      34.336
returned-plan dynamics error: 16.5293
full first-solve acceptance:  FAIL
```

This is an accepted negative diagnostic, not permission to change the three
ADMM iterations or loosen residual tolerances. The QP trajectory is dynamically
and algebraically consistent, and the projection is exactly complementary,
but they have not reached consensus. In addition, the pinned C3 implementation
with `end_on_qp_step=false` mixes projected `z` variables with QP-derived
rollout pieces; its returned plan is not dynamically consistent. The next
gate is the same dynamically feasible LCS rollout used by the DAIRLab Sampling
C3 cost path, followed by residual reporting on that executable trajectory.

## Gates 7–8 — dynamically feasible rollout and deterministic sample batch

The projected C3+ inputs are now passed through the same regularized LCS
simulation path used by `SamplingC3Controller::CalcCost(kSimLCS)`. Every knot
is independently checked with Drake's regularized Lemke LCP solver. The raw
three-iteration ADMM return remains rejected; it was not substituted for or
misreported as this feasible rollout.

The deterministic provider exposes the same eight exact-T boundary samples as
the retained reduced controller. Each sample receives an independent spatial
LCS, C3+ solve, LCS re-simulation, residual audit, and full state/input
trajectory cost. DAIRLab's Push-T workspace and ±50 input constraints and
Anything's ±0.14 m/s EE velocity constraints are explicit in `oim_t.yaml`.
Only residual-valid rollouts inside those physical bounds may be ranked.

```text
current config SHA-256:       4e7dfde3ebfccd6cc3dbfa77b6630db6f28f86a00dd26d1a181d9738d721feb7
dynamic LCP solved:           PASS
dynamic dynamics residual:   0
dynamic nonnegative error:    4.44089e-16
dynamic complementarity:     3.3048e-16
planned input-bound error:    0
exact-T samples solved:       8 / 8
workspace-executable:         1 / 8
selected sample:              stem_left (index 5)
selected trajectory cost:     40563.5
reduced first prediction:     0.381, 0.349173, -3.77079e-22 (unchanged)
reduced contact gap:          0.315961 m (unchanged)
focused full-path tests:      7 / 7 PASS
```

Seven residual-valid candidates were correctly rejected because their
dynamically simulated pusher or object state left the configured workspace;
the largest observed violation was 0.318772 m. This is an acceptance gate, not
a solver retune. The next gate adds seeded perimeter/mesh sample providers and
candidate buffers while retaining this exact-T batch as a deterministic
regression fixture.

## Gate 9 — seeded perimeter sampling and candidate buffers

The first stochastic provider samples the complete exterior perimeter of the
two-box T union, including the exposed crossbar-bottom segments omitted from
the eight-point fixture. Segment selection is length-weighted and the random
seed is explicit. Provider outputs are merged into successful and unsuccessful
buffers without discarding negative results; the successful buffer uses stable
ascending dynamic-rollout cost order.

```text
current config SHA-256:       af2d19a40314592d4c104370d634a5c801f3eae93c7fab8828414d2eb78680a5
random seed / sample count:   0 / 16
perimeter residual-valid:     16 / 16
perimeter workspace-valid:    6 / 16
combined candidates:          24
combined successful/rejected: 7 / 17
selected sample:              stem_left_seeded_13
selected trajectory cost:     38101.9
focused full-path tests:      10 / 10 PASS
```

The next gate is parallel candidate evaluation with deterministic reduction,
followed by exporting the selected full task-space plan to the xArm OSC path.

## Gate 10 — deterministic parallel evaluation

The seeded provider can now use four outer workers, matching DAIRLab's
`num_outer_threads`. Each worker writes to its preassigned sample index; the
cost reduction remains serial and stable. A serial-versus-parallel regression
checks every candidate name and cost as well as the selected index and cost.

```text
current config SHA-256:       de986b1891d3402703d521ab2f8a665fc091d47bf88f50f5f1a111c7de34e774
outer worker count:           4
serial/parallel selection:    identical
serial/parallel costs:        identical within 1e-8
focused full-path tests:      11 / 11 PASS
```

No existing T mesh is substituted for the OIM object: the repository's
`T_small_video.obj` is a different asset. The next provider gate must derive a
mesh from the exact two-box OIM collision geometry or add an explicitly
proven-equivalent asset before mesh-normal sampling is enabled.

## Gates 11–12 — exact collision mesh and xArm execution handoff

The mesh-normal provider is derived from the side triangles of the exact two-
box collision union. It does not use `T_small_video.obj`. Sixteen samples use
seed 1 and deterministic four-worker reduction. The combined exact, perimeter,
and mesh buffer contains 40 candidates (9 executable, 31 rejected). The
selected six-knot spatial plan is wrapped by home-height planar travel,
elevated traverse, whole-capsule descent, standoff, and contact waypoints.

The offline five-joint IK/capsule witness passes in 64 bounded steps. Online
execution now publishes the same measured-state IK steps to the existing
collision-aware posture OSC branch; Cartesian C3 knots remain task-space
controlled. This corrected the earlier Cartesian-only acquisition failure.

## Gate 13 — first physical Full Sampling-C3+ contact

```text
config SHA-256:              19a9bd0a245b71194cef9702a6342093823d2aeb5e102554a0fbb6d876d849e7
raw result:                  /root/push_anything_ADMM/results/full_sampling_posture_acquisition_qd5QTx
acquisition waypoints:       5 / 5 PASS
C3/execution waypoints:      9 / 10 within 2,000 updates
physical contact:            9.184..13.602 s
contact samples/max force:   1,539 / 8.11563 N
measured yaw progress:       0.102477 rad
measured lateral drift:      0.017956 m
unchanged 0.005 m gate:      FAIL
open_table_success:          FAIL
```

The failure is retained as negative evidence. No gain, tolerance, horizon,
ADMM setting, or random seed was changed.

## Gate 14 — deterministic refinement and measured lateral rejection

A deterministic 16-point refinement of the long stem-left face uses the
existing configured perimeter count and no additional random seed. Candidate
receipts now expose predicted terminal x drift and a complete physical
acquisition predicate. The model's predicted 5 mm band and the five-DOF
vertical-contact reachability band do not intersect, so predicted drift remains
a planning receipt rather than an unsafe deadlock condition.

The unchanged 5 mm condition is enforced on measured object state during
execution. The controller stops the sampled rollout and emits a rejection
before continuing to another waypoint.

```text
raw result:                  /root/push_anything_ADMM/results/full_sampling_measured_lateral_rejection_EDEKCB
selected reachable sample:   stem_left_seeded_13
acquisition waypoints:       5 / 5 PASS
measured rejection update:   350
measured drift/tolerance:    0.0053535 / 0.005 m
physical contact episode:    1
contact samples/max force:   617 / 8.28724 N
open_table_success:          FAIL (corrective resampling not yet executed)
focused full-path test:      PASS
all xArm targets build:      PASS
git diff --check:            PASS
```

The next gate is corrective full-batch resampling from the measured rejected
object pose, followed by whole-capsule collision-free repositioning and proof
of a second physical contact. Terminal translation/orientation tolerances stay
unchanged.

## Gates 15–16 — corrective resampling, second contact, and recovery

The measured rejection now starts a complete corrective transaction instead
of aborting: outward release, collision-clear lift, resampling at the measured
object pose, force-polarity selection, elevated traverse, descent, contact,
measured recovery, and outward release. Corrective C3+ holds measured y/yaw
while restoring canonical x; the original open-table goal is retained for
task-progress cycles. A mirrored deterministic stem-right refinement uses the
same configured count and no new random seed.

If no corrective candidate's unused predicted horizon remains inside the
workspace, a contact-only fallback may use a residual-valid, input-bounded
sample. Its failed horizon-workspace receipt remains visible; only the sampled
contact is executed, behind the full capsule/IK gate, and recovery is accepted
from measured object state. This does not execute or relabel the rejected
predicted tail.

```text
config SHA-256:                 19a9bd0a245b71194cef9702a6342093823d2aeb5e102554a0fbb6d876d849e7
accepted raw result:            /root/push_anything_ADMM/results/full_sampling_release_hold_B7v4Yw
initial measured rejection:     0.00514317 m
corrective candidates:          72
selected corrective sample:     mesh_stem_right_seeded_8
corrective force x:             -0.999608
contact-only fallback:          yes; predicted workspace tail not executed
second physical contact:        18.768..18.882 s
physical contact episodes:      2
contact samples/max force:      189 / 18.6969 N
recovery start/best/released:   0.00549346 / 0.00493874 / 0.00492714 m
measured lateral recovery:      PASS
terminal measured-state hold:   PASS
30 s terminal lateral drift:    0.00492988 m, PASS
open_table_success:             FAIL
focused full-path test:         PASS
git diff --check:               PASS
```

A constant measured posture/Cartesian trajectory is published before planner
exit. This prevents Drake's first-order-hold trajectory from extrapolating its
last moving segment indefinitely; the accepted run holds the tip within about
0.4 mm and keeps object x inside the unchanged corridor.

The next gate resumes full-goal Sampling-C3+ from the recovered measured pose,
then repeats collision-aware contact cycles until the unchanged terminal
translation and orientation tolerances pass or a measured gate rejects.

## Gates 17–25 — measured receding execution and fail-closed reposition

The controller now repeats full 200-sample C3+ batches from measured object and
six-joint state. Raw Drake spatial velocity is logged and reduced to the
planar state only at the LCS boundary. Each physical cycle has explicit budget,
contact-loss, wrong-polarity, lateral-response, and final global acceptance
receipts. Failed candidate geometry is reconstructed from the live object pose;
the pusher releases that face before ranked fall-through. Contact engagement is
bounded by the existing 2 mm normal-step value.

Release, lift, and neutral-anchor motion no longer uses a distant nonlinear IK
solution followed by componentwise joint clipping. It uses Drake differential
IK with the already configured planning time step, Cartesian velocity limit,
joint velocity/position limits, and centering gain. A step is publishable only
when its predicted tip reduces the waypoint error and its interpolated entire
capsule remains admissible. Position-only phases preserve the measured shaft
axis. Every post-contact route now lifts above the complete T, retreats to the
home neutral x/y workspace, verticalizes there, traverses above the object,
then lowers and engages.

```text
best connected rollout:       xarm6_velocity_fallback_physical_20000_CeISun
productive cycles:            14
best reported pose:           x=0.375712, y=0.320223, yaw=-0.0230317
vertical completion rollout:  xarm6_vertical_completion_physical_20000_CroKyA
productive cycles:            11
mandatory-neutral proof:      xarm6_mandatory_neutral_anchor_physical_5000_tGkCM7
progress neutral anchors:      3 PASS
recovery neutral anchors:      3 PASS
budget-end tip z:              0.241177 m (above-table traverse)
focused native tests:          4 / 4 PASS
open_table terminal:           FAIL
terminal tolerances:           0.05 m / 0.10 rad, unchanged
```

The retained negative run
`/root/push_anything_ADMM/results/xarm6_local_diffik_terminal_physical_20000_Z2R2Ua`
shows why mandatory neutral routing is required: direct far-reach
verticalization passed static replay but the physical posture controller lost
clearance before fallback. The successor physical run
`/root/push_anything_ADMM/results/xarm6_mandatory_neutral_anchor_physical_5000_tGkCM7`
contains no fall-through.

The next gate is not connectivity. It is cycle-level terminal-error descent:
accepted contacts currently may improve y while regressing yaw. Candidate
arbitration must compare predicted and then measured change in the unchanged
global translation/orientation objective, quarantine regressive responses, and
continue until the existing terminal tolerances pass. No numerical controller,
solver, task, contact, seed, or acceptance parameter was changed in gates
17–25.

## Gate 26 — cycle-level global terminal descent

Candidate arbitration and physical acceptance now share
`EvaluateFullSamplingC3TerminalDescent`. It computes planar translation-error
reduction and wrapped-yaw-error reduction against the unchanged `open_table`
goal. A receipt passes only when both components are nonregressive and at least
one meets its existing successor minimum-progress threshold. Predicted
regression is filtered before execution; measured regression stops the dwell,
quarantines the face after a real minimum dwell, releases it, and replans. A
pre-contact acquisition or budget failure cannot be mislabeled as a measured
response rejection.

Collision-aware reposition OSC now tracks both the differential-IK posture and
the Cartesian tip waypoint from which it was constructed. This closes the
neutral-anchor steady-state tip error while retaining the local differential-IK
and nine-sample whole-capsule clearance receipts. Existing task-space and
posture gains are reused unchanged.

```text
source commit:                f5c53479c6b7bb9e0a5d58d29af2c6c2b10f8bce
worktree:                     dirty; cumulative OIM/xArm work preserved
config SHA-256:               d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed:                         configured deterministic sampling path, unchanged
dependencies:                 Drake 1.51.1, Bazel 8.4.0, g++ 13.3.0
physical output:              /root/push_anything_ADMM/results/xarm6_terminal_descent_physical_20000_0Ov79a
physical contact episodes:    24
accepted descent cycles:      7
rejected regressive cycles:   4
full sustained dwell:         1000 steps, PASS
sustained-dwell progress:      +0.00142678 m / +0.0332332 rad
focused native tests:         4 / 4 PASS
simulator terminal:           FAIL (0.746148 m / 2.93704 rad)
terminal tolerances:           0.05 m / 0.10 rad, unchanged
```

The 20,000-update controller was externally time-limited after update 18,076;
the simulator completed normally and remains the authoritative unchanged
terminal failure. Every logged `task_progress_cycle=PASS` has both
`translation_nonregressive=1` and `orientation_nonregressive=1`. Retained
negative evidence includes translation-improving cycles with orientation
progress `-0.00330205`, `-0.041851`, `-0.0303707`, and `-0.0442978` rad; all
were marked `FAIL`.

The next gate is measured-response-conditioned candidate ranking. The reduced
model still overpredicts some yaw and underpredicts lateral response, causing
many safe releases and recovery transits. Ranking must use accumulated measured
face/pose-neighborhood response without changing the underlying C3+ solver or
the terminal tolerances.

## Gate 27 — measured-response-conditioned candidate ranking

The receding controller now retains every physically observed contact response
as its measured start pose, predicted terminal pose, measured terminal pose,
object-frame contact point/normal, and lateral-rejection receipt. For a new
candidate, observations are matched using the unchanged task translation and
orientation tolerances for both pose and contact neighborhoods. The mean
measured-minus-predicted translation/yaw displacement corrects the candidate's
terminal estimate.

Conditioning is discrete and deterministic rather than a new weighted cost:

1. all locally observed responses compatible with the unchanged lateral and
   Pareto terminal gates;
2. unseen contact neighborhoods;
3. locally incompatible responses retained only for ranked fall-through.

Original C3+ dynamic-cost order is stable within each class. The corrected
terminal/lateral receipt owns strict admission for compatible candidates;
unseen candidates use the original model receipt. Measured-incompatible
candidates cannot jump ahead through a favorable raw prediction.

```text
source commit:                  f5c53479c6b7bb9e0a5d58d29af2c6c2b10f8bce
worktree:                       dirty; cumulative OIM/xArm work preserved
config SHA-256:                 d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings:                  unchanged
primary physical output:        /root/push_anything_ADMM/results/xarm6_measured_response_ranking_physical_8000_ZLD4wx
aligned-arbitration output:      /root/push_anything_ADMM/results/xarm6_measured_response_ranking_aligned_6500_sR3r2Y
stored observations:             3
post-two-response partition:     0 compatible / 158 unseen / 42 incompatible
selected exploration contact:    mesh_stem_left_seeded_3 (unseen)
exploration sustained dwell:      1000 steps, PASS
post-dwell partition:             1 compatible / 18 unseen / 39 incompatible
simulator terminal:               FAIL
terminal tolerances:              0.05 m / 0.10 rad, unchanged
```

The first contact demonstrated the motivating discrepancy directly: predicted
yaw was `0.891824` rad while measured yaw was `0.170312` rad, a `-0.721512`
rad residual, and the lateral gate rejected it. After the second incompatible
observation, 42 matching candidates were demoted and the controller selected
an unseen stem contact. That contact completed a 1,000-step dwell and produced
a compatible observation. The next solve put one compatible neighborhood ahead
of 18 unseen and 39 incompatible candidates.

The gate therefore passes, but `open_table` does not. The newly exposed gate is
post-response candidate availability: after the large physical yaw transition,
one run had 58 dynamic candidates but every admitted route failed live IK at
verticalization waypoint 2; the aligned successor run produced zero dynamic
candidates after its first contact. This must be fixed through reachability-
conditioned replenishment, not by changing response neighborhoods or terminal
tolerances.

## Gate 28 — post-yaw executable and live-IK availability

Two independent failure paths are now covered:

- When measured-velocity and quasistatic workspace-filtered batches are both
  empty, the controller reclassifies the preserved candidates only for a
  contact-feasible fallback. A candidate must still have an accepted dynamic
  rollout, unchanged input-bound compliance, finite cost, measured six-joint
  live IK, and whole-capsule execution receipts.
- When every candidate shares a failed high neutral-anchor verticalization,
  the same nonlinear IK problem is retried from the configured xArm home seed.
  The resulting step retains existing velocity clipping and swept collision
  checks, and additionally must decrease vertical-axis error without moving the
  tip beyond the existing contact-activation tolerance.

```text
source commit:                f5c53479c6b7bb9e0a5d58d29af2c6c2b10f8bce
worktree:                     dirty; cumulative OIM/xArm work preserved
config SHA-256:               d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings:                unchanged
physical output:              /root/push_anything_ADMM/results/xarm6_post_yaw_candidate_availability_physical_8000_bfs6i3
post-yaw replans:             5 PASS
progress live-IK passes:      20
productive cycles:           3
nonproductive cycles:         2, safely recovered
physical contact episodes:   12
final lateral corridor:       0.00359921 m, PASS
simulator terminal:           FAIL (0.731024 m / 2.93876 rad)
terminal tolerances:          0.05 m / 0.10 rad, unchanged
```

The physical run crossed a large first response (`-1.11431 rad` model residual)
and continued through live candidate sets of 56, 191, 194, and 200. It ended
through the unchanged dwell-budget admission gate at update 7,728, rather than
candidate exhaustion or live-IK failure. The contact-feasible and home-seed
fallback branches were not activated in this trajectory; their structural
guards remain mandatory. The contact-buffer classification is covered by the
focused native regression; the home-seed path is build-verified but is not
claimed as a physical receipt in this run.

`open_table` remains the only unchecked task-level acceptance item. The next
gate is a full terminal rollout using the connected response-conditioned loop,
with the unchanged translation and orientation tolerances.

## Gate 29 — settled, corridor-safe prefix execution and post-filter replenishment

Progress replanning now requires two consecutive stable planar object poses,
an upright T at its configured resting height, and measured x error inside a
2 mm inner reserve derived from the unchanged 5 mm lateral tolerance and 3 mm
contact-activation tolerance. Progress and recovery contacts are restricted to
the pusher-radius central side band so a planar sample cannot silently become a
top or bottom edge contact.

When a productive full C3+ rollout leaves the 5 mm corridor, execution uses a
separate copy containing the longest contiguous predicted-safe prefix and then
re-solves from measured state. The original solver receipt is not modified.
If workspace-filtered candidates survive but all fail the downstream prefix,
contact, or response gates, contact-feasible replenishment is retried after
filtering. Replenished candidates still require the same central-side,
predicted/corrected lateral, live six-joint IK, and swept-capsule receipts.

```text
source commit:                    f5c53479c6b7bb9e0a5d58d29af2c6c2b10f8bce
worktree:                         dirty; cumulative OIM/xArm work preserved
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
prefix physical output:           /root/push_anything_ADMM/results/xarm6_corridor_prefix_gate_8000_9BC4hk
replenishment physical output:    /root/push_anything_ADMM/results/xarm6_postfilter_replenishment_8000_kj2Oc9
settle / reserve receipts:         5 PASS / 5 PASS in each rollout
prefix progress replans:           4 PASS
prefix productive cycles:          3 PASS
post-filter replenishments:        2 PASS
replenished progress replans:      5 total PASS
final pose:                         0.380192, 0.331412, -0.525025
simulator terminal:                 FAIL (0.731412 m / 2.61656 rad)
terminal tolerances:                0.05 m / 0.10 rad, unchanged
```

The first receipt crossed the previous zero-intersection failure with 19 live
prefix candidates and three productive cycles. It then exposed a state with
nine workspace candidates but no usable downstream contact. The second receipt
exercised the new post-filter branch twice, recovering 200 dynamic candidates
and respectively seven and four fully admitted plans.

`open_table` remains unchecked. The next gate is measured-overhead-aware
contact-cycle admission: at update 6,709 the existing dwell-only budget test
admitted another cycle with 1,291 updates left. Acquisition consumed most of
that reserve and corrective recovery reached the 8,000-update limit. Admission
must reserve observed acquisition and recovery work in addition to the
unchanged 1,000-step physical dwell, without changing the solver, horizon,
tolerances, or task definition.

## Gate 30 — measured-overhead-aware cycle admission

The selected candidate's live-IK receipt is now converted into the same update
units used by physical execution:

```text
acquisition_updates = ceil(live_ik_steps * planning_dt / execution_dt)
required_updates = acquisition_updates
                 + successor_minimum_contact_steps
                 + max(measured_release_recovery_updates)
```

The release/recovery term is the maximum completed phase receipt observed in
the current physical rollout. An unmeasured fallback and an exactly exhausted
budget are both rejected; no YAML margin was introduced. Admission occurs
after candidate selection and live six-joint IK, but before `progress_lift`, so
a deferred cycle remains in its measured safe hold.

```text
source commit:                    68f1cda36d0fe0d9ef1ba15b2b65c288a6980a14
worktree:                         dirty; Gate-30 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
insufficient-budget output:       /root/push_anything_ADMM/results/xarm6_measured_cycle_budget_2000_uKAhCs
sufficient-budget output:         /root/push_anything_ADMM/results/xarm6_measured_cycle_budget_8000_q8FGmN
insufficient receipt:             1,418 remaining / 1,815 required, DEFER
progress-lift commands after defer: 0
terminal hold after defer:        PASS
sufficient receipt:               7,413 remaining / 1,837 required, PASS
physical acquisition:             PASS
unchanged contact dwell:          1,000 steps, PASS
terminal translation progress:    0.0232898 m, nonregressive
terminal orientation progress:    0.0663094 rad, nonregressive
productive receding cycle:        PASS
simulator terminal:               FAIL (0.775167 m / 3.05327 rad)
```

The gate passes both required branches. The longer run then exposed a separate
execution-consistency failure: a second candidate passed its measured-state
live-IK preview with 228 steps, but physical OSC execution passed only the
initial lift, entered fallback, and left the arm at a posture from which every
candidate failed neutral-anchor waypoint 1. The next gate is therefore
selected-receipt versus physical-phase conformance: an acquisition that
diverges from its preview must return to the physically verified neutral anchor
before replanning, and the candidate must be invalidated without losing the
completed terminal-descent receipt. The full terminal rollout follows that
gate under the unchanged 0.05 m / 0.10 rad tolerances.

## Gate 31 — selected-preview versus physical-acquisition conformance

Every selected live-IK acquisition now has a seven-phase physical receipt:
lift, neutral anchor, verticalization, overhead traverse, lower, descent, and
contact. Measured task-space regression beyond the existing 3 mm activation
tolerance invalidates the candidate. A pre-descent failure does not execute an
unowned face-release command. Replanning is allowed only after collision-aware
return to the elevated home-tip anchor, verification of the vertical capsule,
and preservation of the preceding terminal-descent evidence.

```text
source commit:                    4a7ba15119f55f3bacc223d9f182e98624f80f9e
worktree:                         dirty; Gate-31 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_acquisition_conformance_retry_8000_DgKtuO
preview divergences:              25
candidate invalidations:          25
verified neutral reacquisitions:  25
terminal receipts preserved:      25
unsafe replans:                   0
prior unguarded failure:          update 3,132
first guarded failure:            update 1,066
planner terminal:                 wall-clock timeout after simulator duration
open_table terminal:              not reached
```

Gate 31 passes: every observed divergence was rejected and every retry was
conditioned on a measured neutral-anchor recovery. The receipt also isolates
Gate 32. The controller declares a waypoint complete while the arm still has
substantial measured motion; Sampling-C3+ then blocks for seconds while OSC
continues the last endpoint command. The next subtarget can therefore observe
a large overshoot despite its 5 mm command bound. Gate 32 must condition phase
completion on measured end-effector settling derived from the unchanged 3 mm
activation tolerance and 50 ms planning interval, then demonstrate a physical
acquisition beyond `progress_lower` without changing OSC gains or task limits.

## Gate 32 — fresh measured-state settled phase handoff

Robot position and velocity channels are now imported together into the
acquisition plant (`xarm6_jointNdot` maps explicitly to `xarm6_jointN`). A
phase is complete only when the tip is inside the unchanged 3 mm activation
ball and its measured velocity predicts no more than 3 mm motion over the
unchanged 50 ms planning interval. Every physical phase boundary drains a new
robot measurement before constructing its first command. Conformance uses the
admitted phase-entry error plus the existing activation tolerance; this catches
runaway motion while allowing a converging lower phase to have bounded local
transients.

```text
source commit:                    5fb19db5
worktree:                         dirty; Gate-32 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_entry_envelope_handoff_8000_A05xBP
measurement refresh receipts:     86 PASS
seven-phase acquisitions:         4 PASS
physical contact responses:       4
reported productive cycles:       3
terminal translation error:       0.765420 m
terminal orientation error:       2.16789 rad
simulator terminal:               FAIL
```

Gate 32 passes by crossing the previous four-phase ceiling and proving repeated
physical contact. The next receipt boundary is incorrect: productivity is
computed from the object pose immediately after the rejected contact, before
the corrective-face recovery completes. Gate 33 must re-evaluate translation,
orientation, and lateral acceptance from the measured post-recovery pose and
must not credit a cycle whose corrective transaction undoes the initial task
progress.

## Gate 33 — post-recovery net-progress ownership

The productive-cycle receipt is now evaluated from the measured object pose
after all release and lateral-correction phases complete. The same unchanged
Pareto rule is applied to that pose, and lateral error must also satisfy the
existing 5 mm corridor. The immediate contact pose remains in the response
history because it is the physical response to the selected C3+ candidate, but
it can no longer clear the candidate quarantine or increment the receding-cycle
counter by itself.

```text
source commit:                    19e1a8a1
worktree:                         dirty; Gate-33 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_post_recovery_receipt_8000_AU6gGW
seven-phase acquisitions:         4 PASS
post-recovery progress:           1 PASS / 3 FAIL
failure classifications:         2 yaw, 1 translation, 1 lateral (overlap)
honest productive cycles:         1
simulator terminal:               FAIL (0.779160 m / 2.46560 rad)
```

Gate 33 passes by rejecting three previously miscredited transactions. The
new failure is upstream: corrective candidate generation replaces the global
goal with an x-only lateral target, so its C3+ ranking is free to undo y or yaw
progress. Gate 34 must retain the global task goal during lateral recovery and
require predicted translation/orientation nonregression in addition to the
existing corrective x-force polarity and physical clearance gates.

## Gate 34 — task-preserving corrective recovery

Corrective recovery now retains the unchanged global object goal in its C3+
problem instead of replacing it with an x-only target. Candidate admission
also requires predicted translation and orientation nonregression from the
cycle baseline, in addition to the existing corrective x-force polarity,
central-side contact, live-IK, and whole-capsule gates. The check uses zero
progress thresholds: it prevents regression without changing the configured
task tolerances or claiming predicted progress as physical success.

```text
source commit:                    f422fc7d
worktree:                         dirty; Gate-34 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_task_preserving_recovery_8000_Zm4a3u
seven-phase acquisitions:         4 PASS
predicted-regressive rejections:  21
post-recovery progress:           4 PASS / 0 FAIL
honest productive cycles:         4
terminal translation error:       0.744534 m
terminal orientation error:       3.02579 rad
simulator terminal:               FAIL
```

Gate 34 passes its ownership requirement and improves the Gate-33 physical
record from `1 PASS / 3 FAIL` to `4 PASS / 0 FAIL`. The unchanged 8,000-update
budget then defers a fifth transaction because its measured worst-case
acquisition/dwell/recovery receipt requires 2,242 updates while 1,730 remain.
The larger blocker is progress quality: nonregression admits very small or
poorly balanced gains, so four cycles still leave the object far from both
terminal tolerances. Gate 35 must rank task-preserving recovery candidates by
predicted Pareto descent magnitude and compare that ordering with measured
post-recovery descent, without weakening the physical acceptance gate.

## Gate 35 — predicted Pareto-ranked recovery conformance

Task-preserving recovery candidates are now stable-ranked by their predicted
Pareto descent before live IK. Translation and orientation gains are divided
by the unchanged 0.05 m and 0.10 rad terminal tolerances and summed; this gives
a dimensionless ordering without introducing a new weight. Dynamic rollout
cost remains the deterministic tie-breaker. Every physically engaged recovery
candidate stores its selected prediction and compares it with the measured
post-recovery pose.

```text
source commit:                    4de568ce
worktree:                         dirty; Gate-35 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_pareto_ranked_recovery_8000_RZ453L
prediction receipts:              3
prediction-sign conformance:      2 PASS / 1 FAIL
post-recovery progress:           2 PASS / 1 FAIL
largest normalized overprediction: 24.2769
terminal translation error:       0.747945 m
terminal orientation error:       2.36896 rad
simulator terminal:               FAIL
```

Gate 35 passes because ranking and physical comparison are both executable and
fail closed; it does not establish that raw predicted magnitude is a useful
physical ranking. All three predictions were strongly overconfident. The third
predicted `+0.385644 m / +2.17689 rad`, but measured `-0.0198602 m /
+0.560209 rad`, so physical translation regressed. Gate 36 must apply the
existing measured-response conditioning to recovery candidates and rank the
corrected terminal descent ahead of unseen raw predictions while retaining an
exploration fall-through.

## Gate 36 — measured-response-conditioned recovery history

Corrective recovery now queries the same local pose/contact response model as
primary candidate arbitration. Evidence class is ordered before normalized
descent: compatible corrected terminals lead, unseen contacts preserve the
exploration path, and incompatible neighborhoods remain last. Each physically
engaged recovery stores its start, raw prediction, measured post-recovery pose,
contact point, contact normal, and lateral outcome in the shared history.

```text
source commit:                    364c035d
worktree:                         dirty; Gate-36 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_response_conditioned_recovery_8000_2sybHu
recovery observations stored:     1 PASS
later recovery reuse:             not reached
conditioned successor demotions:  39
post-recovery progress:           1 PASS / 1 FAIL
terminal translation error:       0.793330 m
terminal orientation error:       2.43800 rad
simulator terminal:               FAIL
```

Gate 36 is implemented and its history-write branch is physically proven, but
the read/ranking branch remains pending because the next primary resample
failed first. The new recovery observation plus prior top-face evidence
correctly demoted 39 candidates; all 200 replenished candidates then failed
the unchanged Pareto acceptance rule, so execution held at update 3,892.
Gate 37 must restore live-IK candidate availability after evidence-based face
demotion by searching the remaining contact faces, while leaving incompatible
top-face candidates quarantined and all tolerances unchanged.

## Gate 37 — component-decomposed candidate search

When both the canonical combined translation/yaw subgoal and contact-feasible
replenishment yield no globally acceptable candidate, the controller now
solves translation-only and rotation-only subgoals from the identical measured
state. Their candidates remain subject to the original global Pareto test,
measured-response class, lateral corridor, central-contact, live-IK, and full
capsule gates. Candidate names record the component source; no cost, horizon,
seed, tolerance, or physical acceptance rule changes.

```text
source commit:                    503a3028
worktree:                         dirty; Gate-37 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_component_search_8000_BzLBfL
decomposed-search activations:     0 (trigger state not reached)
recovery observations stored:     4
post-recovery progress:           3 PASS / 1 FAIL
honest productive cycles:         3
terminal translation error:       0.788189 m
terminal orientation error:       2.36720 rad
simulator terminal:               FAIL
```

Gate 37 is implemented and regression-tested, but its physical activation
receipt remains pending because this rollout retained canonical candidates at
every arbitration. The denser history did expose the next repeatable boundary:
a late recovery remained unseen at selection and physically regressed
translation. Gate 38 must demonstrate that a later local recovery candidate
actually consumes a prior recovery observation, with incompatible corrected
descent rejected before live IK.

## Gate 38 — yaw-equivariant recovery-response conditioning

Recovery residuals now live in the T object frame. Translation residuals are
rotated from the observation world frame into O, averaged within the unchanged
contact point/normal neighborhoods, and rotated into the current world frame.
Yaw residuals are wrapped and transferred directly. The existing local
measured-response estimator is unchanged; recovery uses a separate history so
primary and corrective execution semantics cannot contaminate each other.

```text
source commit:                    4e667ef8
worktree:                         dirty; Gate-38 implementation under test
config SHA-256:                   d11cd65efbcf6ac7c814a0b690cc76f9135d27a9f607f6f10dc7d9b26051b990
seed/settings/tolerances:         unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_equivariant_recovery_8000_fb1lbo
equivariant unit receipt:         PASS across pi/2 yaw
recovery observations stored:     1
later physical recovery reuse:    not reached
honest productive cycles:         1
simulator terminal:               FAIL (0.775391 m / 2.97934 rad)
```

Gate 38's transformation and isolated history are implemented and tested, but
physical reuse remains pending. The next acquisition failed earlier: after the
large yaw transition, the fixed neutral anchor had no position-only IK and its
recovery anchor also failed contact IK. Gate 39 must construct a measured,
reachable elevated anchor while preserving the same whole-capsule clearance
and neutral-reacquisition ownership rules.

## Gate 39 — measured reachable elevated anchor

Failed preview recovery still tries the fixed elevated home-tip anchor first.
If that anchor is outside the current joint-limit component, the measured lift
endpoint may become the neutral anchor only after the unchanged vertical
posture solver and whole-capsule clearance gate pass there.

```text
source commit:                    dce1e3c5
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_reachable_anchor_8000_c0sgo0
fixed-anchor reacquisitions:       2 PASS
reachable-anchor activations:      0 (fallback not triggered)
equivariant recovery reuse:        1 physical receipt
honest productive cycles:          3
simulator terminal:                FAIL (0.731981 m / 2.94436 rad)
```

Gate 39 is implemented and tested but remains physically untriggered. This run
does provide Gate 38's missing read receipt: a later `crossbar_left` recovery
matched one earlier recovery observation across yaw and was classified rank 2.
Because rank-2 recovery was still retained as deterministic fall-through, it
reached live IK. Gate 40 must quarantine incompatible corrected recovery before
live IK while leaving unseen contacts available.

## Gate 40 — incompatible recovery quarantine

Rank-2 equivariantly corrected recovery candidates are now rejected before
live IK. Rank-1 unseen contacts remain explorable and rank-0 corrected contacts
retain priority; no observed-incompatible recovery can remain as physical
fall-through.

```text
source commit:                    b0e272dc
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_recovery_quarantine_8000_xcLDEr
quarantine activations:            0 (history not yet populated)
unseen recovery live-IK failures:  8 at waypoint 0
simulator terminal:                FAIL (0.820813 m / 2.54645 rad)
```

Gate 40 is implemented and regression-tested but its physical rejection branch
remains pending. The first recovery started from a rejected loaded posture and
all eight unseen candidates failed swept-capsule admission at waypoint 0.
Gate 41 must own a measured outward controlled release before constructing any
recovery preview, then re-evaluate all candidates from the released posture.

Gate 40's pending physical branch subsequently passed in the Gate-41 rollout:
two observed `crossbar_left` candidates were rejected with
`before_live_ik=1`, and neither produced a live-IK or motion receipt.

## Gate 41 — controlled-escape acquisition preview

Measured recovery preview now applies the same controlled vertical escape rule
as physical execution at waypoint 0. A presently intersecting shaft may escape
only while the command is vertical, height strictly increases, outward motion
is nonnegative, and endpoint capsule/table and tip/table clearance pass. All
later preview waypoints retain ordinary swept-capsule clearance.

```text
source commit:                    df5e1eb6
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_preview_controlled_escape_8000_XEtH1U
controlled-escape preview:        1 PASS
Gate-40 quarantines before IK:     2 PASS
honest productive cycles:         1
simulator terminal:               FAIL (0.767900 m / 3.04036 rad)
```

Gate 41 passes. The previously rejected waypoint-0 recovery reached live IK
with `controlled_escape=1`, then completed its physical transaction. After the
history became available, Gate 40 also rejected both incompatible corrective
faces before IK. No remaining recovery candidate could restore the final
5.20 mm lateral error. Gate 42 must replenish compatible or unseen corrective
faces after quarantine without re-admitting the observed-incompatible face.

## Gate 42 — deterministic recovery seed replenishment

Only after the canonical 72 recovery proposals are exhausted, four additional
perimeter/mesh seed pairs are solved. Their names record retry provenance, and
all existing response, Pareto, polarity, central-contact, controlled-escape,
live-IK, and capsule gates remain mandatory.

```text
source commit:                    1c499d1a
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_recovery_seed_replenishment_8000_iiRHtt
retry candidates:                 128
dynamically executable:           94
globally accepted:                0
observed-face quarantines:         2
simulator terminal:               FAIL (0.778069 m / 3.13836 rad)
```

Gate 42 passes its activation and provenance requirements but does not restore
physical availability. Every corrective-face retry predicted task regression.
The five-knot recovery solve still targets the distant terminal goal directly,
unlike primary receding planning. Gate 43 must use the same tolerance-bounded
translation/yaw recovery subgoal while continuing to judge every prediction
and measurement against the unchanged global goal.

## Gate 43 — bounded receding recovery subgoal

Recovery C3+ now uses the same horizon-reachable construction as primary
planning: x retains the global lateral correction, while y and yaw advance by
no more than the unchanged 0.05 m and 0.10 rad terminal tolerances. Prediction
and physical receipts continue to evaluate the unchanged global goal.

```text
source commit:                    65de258f
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_bounded_recovery_subgoal_8000_aN7r8X
bounded recovery live candidates: 3
controlled-escape recoveries:     1
honest productive cycles:         1
late response quarantines:        34
simulator terminal:               FAIL (0.756664 m / 3.06797 rad)
```

Gate 43 passes. The bounded objective restored repeated recovery availability
without retry seeds. Later, two observations covered nearly every stem-left
proposal because recovery contact matching reused the 50 mm task tolerance,
which is comparable to the whole face. Gate 44 must localize recovery evidence
to the existing pusher contact footprint rather than the terminal task scale.

## Gate 44 — contact-footprint-local recovery evidence

Yaw-equivariant recovery matching now uses the configured pusher diameter as
its object-frame contact-position neighborhood. This replaces the unrelated
50 mm terminal translation scale without introducing a parameter; normal
matching and all task/collision acceptance remain unchanged.

```text
source commit:                    941a3170
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_contact_footprint_response_8000_HGFEw7
contact neighborhood source:      existing pusher diameter
later recovery arbitration:       not reached
fixed-anchor recovery:            FAIL
reachable in-place anchor:        FAIL
simulator terminal:               FAIL (0.775161 m / 2.90208 rad)
```

Gate 44 is implemented and tested but remains physically unexercised. The run
triggered the Gate-39 fallback instead: after lifting at a joint limit, neither
the fixed elevated home point nor in-place verticalization had IK. Gate 45 must
retreat through a whole-capsule-checked elevated joint-space path toward the
home posture before requesting verticalization.

## Gate 45 — candidate-overhead recovery anchor

After fixed-home and measured-lift anchors fail, preview recovery now attempts
the rejected candidate's already validated overhead x-y point. It retains the
measured lift height, uses home-seeded vertical-posture IK, and checks every
interpolated capsule sample before a retry can be authorized.

```text
source commit:                    364ecfc5
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_overhead_anchor_recovery_8000_jFp0Rr
fixed anchor contact IK:          FAIL
measured-lift anchor contact IK:  FAIL
candidate-overhead contact IK:    FAIL
common attempt height:            0.57081 m
simulator terminal:               FAIL (0.774098 m / 2.94377 rad)
```

Gate 45 is implemented and physically exercised but not accepted. The three
geometrically distinct anchors all fail at the same elevated height while the
second arm joint is at its limit. Gate 46 must perform a collision-checked
position-only descent to the existing geometric capsule-clearance height, then
retry verticalization. This changes recovery sequencing, not configuration.

## Gate 46 — clearance-height de-elevation

If all elevated verticalization anchors fail, preview recovery now descends
position-only at its already-clear x-y point to the exact geometric
capsule/object clearance height. Only after that swept-capsule-checked descent
does it request in-place vertical posture IK; failure still invalidates the
candidate and preserves terminal evidence.

```text
source commit:                    b00c0453
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_clearance_height_recovery_8000_Q6LHxV
clearance-height branch:          not reached
ordinary neutral recoveries:      3 PASS
honest productive cycles:         2
measured budget defer:            2138 remaining / 2241 required
simulator terminal:               FAIL (0.749225 m / 2.92554 rad)
```

Gate 46 is implemented and tested but remains activation-pending. The rollout
did expose a bookkeeping defect: entering the next loop iteration clears the
last productive-cycle boolean before a safe measured-budget defer. Gate 47
must retain cumulative productive-handoff provenance while keeping the global
terminal tolerance as the only task-success authority.

## Gate 47 — cumulative handoff and budget-defer provenance

Terminal classification now consumes a cumulative productive-cycle latch,
not the current iteration's transient boolean. A pure evaluator separates
closed-loop handoff, unchanged terminal acceptance, return code, and reason.
Regression tests cover both productive-then-deferred and
deferred-without-progress states.

```text
source commit:                    dce8b368
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_budget_defer_provenance_8000_EM8K9f
physical productive/defer branch: not reached
terminal-status regressions:      2 PASS
focused Bazel tests:              2 targets PASS
simulator terminal:               FAIL (0.797272 m / 3.13386 rad)
```

Gate 47 passes deterministic provenance acceptance; its physical replicate
correctly failed closed after a wrong-polarity first cycle. Across recent
rollouts, the remaining limiting behavior is now measured candidate response:
C3+ frequently predicts order-one yaw descent while the physical T produces
only hundredths of a radian or the opposite sign. Gate 48 must quantify model
gain/sign calibration per local contact before ranking another dwell.

## Gate 48 — measured-productivity gain ranking

For each matching primary contact neighborhood, the controller now computes
measured normalized Pareto descent divided by predicted normalized descent.
The same unchanged translation/orientation tolerances provide units. Evidence
class remains the first sort key; only compatible contacts are then ordered by
their gain-calibrated predicted magnitude. No acceptance or collision gate is
relaxed.

```text
source commit:                    378f804d
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_measured_productivity_ranking_8000_UxtbEQ
nonzero gain candidate receipts:  77
honest productive cycles:         2
productive handoff preserved:     PASS
simulator terminal:               FAIL (0.807974 m / 2.33882 rad)
```

Gate 48 passes activation. The terminal regression was caused by an unseen
crossbar-bottom-left contact: its moment had desired yaw polarity, but its
normal force opposed desired y translation. The current disjunctive wrench
test admits either component. Gate 49 must apply the same Pareto contract used
for terminal descent: both wrench components nonregressive and at least one
strictly productive.

## Gate 49 — Pareto-consistent wrench admission

Initial contact wrench admission now mirrors terminal Pareto descent. Desired
y times contact-force y and desired yaw times planar moment must each be
nonnegative, and at least one must be strictly positive. This adds no magnitude
threshold and runs before live IK or physical motion.

```text
source commit:                    2dbd25e8
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_pareto_wrench_8000_wytIfk
translation-polarity rejections:  326
orientation-polarity rejections:  391
honest productive cycles:         3
simulator terminal:               FAIL (0.744168 m / 2.90777 rad)
```

Gate 49 passes physical activation. The prior large y regression did not
recur, and productive cycling increased from two to three. A fourth contact
exited the lateral corridor by 6.285 mm. Every corrective proposal then failed
strict global two-component Pareto prediction, leaving no recovery. Gate 50
must admit a bounded lateral-restoration transaction while keeping final
global tolerance and measured post-recovery acceptance unchanged.

## Gate 50 — first-entry bounded lateral recovery

Recovery may spend at most the unchanged 0.10 rad orientation tolerance only
when its prediction restores the unchanged 5 mm x corridor and preserves
global translation over the full cycle. Selection evaluates the first
predicted corridor-entry knot, matching the event-driven stop already used by
physical execution, rather than rejecting a later five-knot overshoot.

```text
source commit:                    6a2d7411
config/settings/tolerances:       unchanged
negative output:                 /root/push_anything_ADMM/results/xarm6_bounded_lateral_recovery_8000_QonInM
passing output:                  /root/push_anything_ADMM/results/xarm6_lateral_entry_recovery_8000_vRIO5g
first-entry recovery live IK:     4 PASS
measured 2 mm reserve recovery:   4 PASS
honest productive cycles:         3
measured budget defer:            2103 remaining / 2277 required
simulator terminal:               FAIL (0.734567 m / 2.99524 rad)
```

Gate 50 passes. It also physically proves Gate 47: cumulative productive
handoff survives a later safe defer and is reported separately from global
task failure. The remaining 0.735 m distance cannot be closed at the measured
cycle throughput within 8,000 updates. Gate 51 must quantify a lower-bound
terminal update budget from observed task progress and complete-cycle cost;
it must not relabel the 8,000-update benchmark as success.

## Gate 51 — measured terminal-budget lower bound

Each productive cycle now records complete update cost plus measured global
translation and orientation progress. At terminal hold, the controller emits
an explicitly optimistic lower bound using the best observed progress in each
component, the cheapest complete cycle, and the unchanged terminal tolerances.
It does not alter budget admission or success.

```text
source commit:                    a8a58527
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_terminal_budget_estimate_8000_hCGL2b
measured productive cycles:       1
minimum complete cycle cost:      1674 updates
optimistic remaining cycles:      32
optimistic remaining updates:     53568
simulator terminal:               FAIL (0.773723 m / 2.93634 rad)
```

Gate 51 passes activation and confirms that 8,000 updates cannot be treated as
a plausible terminal budget at current throughput. The rollout also finally
activated Gate 46's de-elevation branch. Measurement refresh moved the tip x-y
after its waypoint was formed, turning a vertical descent into a diagonal
command that failed position-only IK. Gate 52 must latch x-y from the refreshed
measurement within the phase while retaining collision checks.

## Gate 52 — measurement-conditioned vertical de-elevation

Recovery de-elevation now treats the stored clearance waypoint as a height,
not a stale Cartesian point. Each substep latches x-y from the newest measured
tip and clamps only the z displacement by the unchanged task-space step limit.
Completion and preview-conformance errors are likewise one-dimensional for
this explicitly vertical phase. Position-only IK and every interpolated
whole-capsule/table clearance check remain mandatory.

```text
source commit:                    bb0e0c5f
config/settings/tolerances:       unchanged
focused tests:                    PASS
live controller build:            PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_measured_vertical_deelevation_8000_slsgwJ
physical branch activations:      0 (activation pending)
productive cycles:                1
simulator terminal:               FAIL (0.788526 m / 2.95698 rad)
```

Gate 52 is implemented but its physical activation proof remains pending; the
rollout reacquired the fixed neutral anchor instead. The next observed blocker
is deterministic: all available lateral-recovery candidates restored the
unchanged x corridor but predicted 0.00288716 m of complete-cycle translation
debt. Gate 53 must represent bounded recovery debt explicitly while retaining
separate measured productive-cycle and global terminal acceptance.

## Gate 53 — bounded translation-debt recovery transaction

Lateral recovery may now spend at most the unchanged 0.05 m terminal
translation tolerance, symmetric with its existing 0.10 rad orientation-debt
bound, while restoring the unchanged x corridor. This is candidate admission,
not progress credit: the independent measured post-recovery gate still
requires globally nonregressive translation and orientation before a cycle is
called productive, and terminal success still uses the original tolerances.

```text
source commit:                    66f060ae
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_bounded_translation_recovery_8000_GFFTqD
corrective recovery passes:       4
measured productive cycles:       4
minimum complete cycle cost:      1058 updates
terminal budget decision:         DEFER (1631 remain / 2331 required)
simulator terminal:               FAIL (0.745583 m / 2.78197 rad)
```

Gate 53 passes physical activation. The formerly empty recovery path now
produces repeated live-IK, whole-capsule-clear corrective contacts, and every
credited cycle passed the strict measured nonregression check. The next gate
must expose whether a requested terminal-run budget is sufficient for the
measurement-derived optimistic lower bound; the 8,000-step benchmark must
remain an honest expected failure.

## Gate 54 — live terminal-budget sufficiency receipt

After every measured productive cycle, the controller now compares remaining
requested updates with Gate 51's optimistic lower bound. A PASS is documented
as necessary but not sufficient for success; the receipt does not change cycle
admission or terminal acceptance.

```text
source commit:                    e4e21ebb
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_budget_sufficiency_8000_WYpHGc
remaining requested updates:      5839
optimistic required updates:      96624
budget sufficiency:               FAIL (expected)
simulator terminal:               FAIL (0.773285 m / 3.07183 rad)
```

Gate 54 passes activation and prevents an 8,000-step run from being mistaken
for a credible terminal attempt. The physical trace also exposes Gate 55:
after the first productive cycle, all component-decomposed yaw candidates
predicted only 0.7--1.2 mm of temporary translation debt and were rejected by
the strict intermediate two-component Pareto gate. Candidate admission needs
a bounded task-component transaction while measured progress credit remains
strictly Pareto.

## Gate 55 — bounded component-decomposed transaction

Translation-only and rotation-only fallback candidates may now spend at most
one unchanged terminal tolerance in the inactive task component, only when
their total tolerance-normalized descent is positive and the active component
meets an existing successor progress minimum. Wrench polarity, measured
response conditioning, live IK, capsule clearance, measured productive-cycle
acceptance, and the global terminal gate remain unchanged.

```text
source commit:                    e9b04462
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_component_transaction_8000_Vze1cL
component fallback activations:   0 (activation pending)
measured productive cycles:       1
later uncredited task progress:   0.024396 m / 0.159697 rad
simulator terminal:               FAIL (0.765001 m / 2.85981 rad)
```

Gate 55 is implemented but activation remains pending. The run progressed to a
new corrective-recovery failure: its fixed high neutral anchor was
position-only-IK infeasible after a large measured transition. Gate 56 must
give corrective recovery the same reachable, candidate-overhead, and
measurement-conditioned de-elevation fallback ladder as primary recovery,
without bypassing swept whole-capsule clearance.

## Gate 56 — corrective-recovery anchor ladder

Corrective recovery now mirrors primary recovery after a fixed neutral-anchor
failure: measured in-place verticalization, candidate-overhead verticalization,
then measurement-conditioned clearance-height de-elevation and verticalization.
Every option retains posture IK, per-interpolation whole-capsule clearance, and
explicit provenance in the live log.

```text
source commit:                    253ea3a4
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_corrective_anchor_ladder_8000_H5JNqw
fixed corrective anchor passes:   4
alternate anchor activations:     0 (activation pending)
measured productive cycles:       4
optimistic total update need:     34468 (6631 used + 27837 remaining)
simulator terminal:               FAIL (0.714302 m / 2.75140 rad)
```

Gate 56 passes its fixed-path physical receipt; alternate fallbacks remain
activation-pending. The controller again reaches only the honest budget defer.
Gate 57 is therefore a separately labeled 40,000-update terminal attempt,
chosen above the live optimistic lower bound. The canonical 8,000-step
benchmark remains unchanged and failed.

## Gate 57 — extended-budget physical fall-through

The 40,000-update terminal attempt was explicitly labeled as an extended run;
it did not change YAML or defaults. It stopped after only 3,901 updates, proving
that budget was not yet the active limit. A nonlinear posture solve selected a
kinematically equivalent joint-1 winding near `-2*pi`; joint 2 simultaneously
reached its lower limit, leaving every recovery anchor without position IK.

```text
source commit:                    4d2deefd
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_terminal_attempt_40000_GsOIbG
requested updates / used:         40000 / 3901
terminal measured q1 / q2:        -6.28318 / -2.0 rad
measured productive cycles:       1
simulator terminal:               FAIL (0.775869 m / 2.98699 rad)
```

Gate 57 is a failed terminal attempt and a passed diagnostic gate. Gate 58
must canonicalize periodic revolute IK solutions to the equivalent angle
nearest the measured configuration before velocity-bounded stepping, avoiding
an unnecessary winding to the joint-limit boundary without changing the
requested Cartesian posture or joint limits.

## Gate 58 — nearest-measured periodic IK representation

For a revolute joint whose declared interval contains a full revolution, a
nonlinear posture solution is now shifted by an in-limit integer multiple of
`2*pi` to the representation nearest measured q before the unchanged velocity
step clamp. This preserves exact forward kinematics and all constraints while
preventing an unnecessary full-turn trajectory to a limit boundary.

```text
source commit:                    fd700c7d
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_periodic_ik_branch_8000_SkAQpJ
explicit substitutions:          0 (activation pending)
measured q1 range:                [-0.179786, 0.905267] rad
measured q2 range:                [-0.928897, 0.655898] rad
measured productive cycles:       4
simulator terminal:               FAIL (0.729523 m / 2.91010 rad)
```

Gate 58 passes regression and the Gate-57 saturation did not recur, though a
raw equivalent-turn substitution was not needed in this trace. Gate 59 repeats
the separately labeled 40,000-update attempt to identify the next blocker.

## Gate 59 — sustained extended rollout

The second extended run completed twelve strictly Pareto productive cycles and
continued through repeated contact loss, wrong-polarity response, candidate
rejection, and corrective retry. The periodic joint winding did not recur. It
reduced translation error by 0.245 m and yaw error by 1.287 rad before stopping
at a nonproductive but safely recovered transaction.

```text
source commit:                    43da2ab9
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_terminal_attempt2_40000_OAXQZo
requested updates / used:         40000 / 26985
measured productive cycles:       12
measured q1 range:                [-0.248928, 0.892447] rad
measured q2 range:                [-1.760778, 0.752782] rad
simulator terminal:               FAIL (0.554551 m / 1.85494 rad)
```

The final recovery restored x and improved translation by 0.029856 m while
regressing orientation by 0.013573 rad. Strict measured productivity correctly
failed, but its positive tolerance-normalized net descent and bounded debt made
the released state safe for another solve. Gate 60 must allow this state to
continue replanning without granting productive-cycle credit.

## Gate 60 — bounded recovered-state replanning

A laterally recovered transaction that fails strict measured Pareto credit may
now continue the receding loop only when Gate 55's bounded component
transaction passes. It receives no productive-cycle count and contributes no
terminal-budget progress receipt.

```text
source commit:                    e155f204
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_bounded_replan_continuation_40000_PacwQD
bounded continuation activations: 0 (activation pending)
measured productive cycles:       1
simulator terminal:               FAIL (0.775847 m / 2.96896 rad)
```

Gate 60 is implemented but its late branch remained dormant because a prior
neutral-anchor traversal repeated Gate 57's joint winding. The existing
physical conformance check compares current error only with phase-entry error;
after initially improving a large error, it can reverse by a large amount and
still pass. Gate 61 must use best-so-far error with the same unchanged
activation tolerance.

## Gate 61 — best-so-far physical waypoint conformance

Physical waypoint conformance now compares each measured error with the best
error observed in that phase, using the unchanged 3 mm activation tolerance.
The recovery neutral anchor is covered by the same check. This catches
cumulative Cartesian reversal even when the error remains below its much
larger phase-entry value.

```text
source commit:                    b613b38a
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_best_so_far_conformance_8000_JazfSl
conformance rejections:           12
measured q1 range:                [-0.278375, 0.860012] rad
measured q2 range:                [-0.864791, 0.647816] rad
measured productive cycles:       0
simulator terminal:               FAIL (0.797035 m / 3.12327 rad)
```

Gate 61 passes its diagnostic objective: the periodic winding does not recur
and every reversal is explicitly rejected. It also shows that a one-sample
best-error comparison is too sensitive to the normal 20 ms settling ripple:
all twelve otherwise executable acquisitions were rejected after only
3.1--4.0 mm of transient backslide. Gate 62 must require a persistent
best-so-far violation while preserving the same spatial tolerance.

## Gate 62 — planning-interval conformance persistence

The same 3 mm best-so-far boundary now has temporal persistence derived from
existing timing: `ceil(planning_time_step / execution_period) = 3` measured
updates. A conformant sample resets the consecutive count; no YAML field or
spatial tolerance was added.

```text
source commit:                    10c94f3c
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_persistent_conformance_8000_lLMN8g
persistent conformance rejects:   12 (3 consecutive updates each)
measured q1 range:                [-0.273459, 0.859734] rad
measured q2 range:                [-0.862359, 0.646928] rad
measured productive cycles:       0
simulator terminal:               FAIL (0.796994 m / 3.12119 rad)
```

Gate 62 proves the lower-phase reversal persists for a full planning interval,
so it is not a one-sample measurement spike. The best-so-far invariant is
still too broad: normal lower/descend convergence can be nonmonotone, whereas
the observed periodic winding belongs specifically to long neutral-anchor
traversal. Gate 63 must preserve phase-entry conformance for ordinary phases
and apply persistent best-so-far conformance only to neutral anchors.

## Gate 63 — neutral-anchor-scoped best-so-far conformance

Persistent conformance now chooses its reference by phase: neutral-anchor
traversals use best-so-far error, while other already-monitored reposition
phases retain the pre-Gate-61 phase-entry reference. The spatial and temporal
criteria are otherwise identical.

```text
source commit:                    03e751df
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_scoped_conformance_8000_K5UI1q
measured productive cycles:       4
neutral-anchor passes:            4
measured q1 range:                [-0.242683, 0.845011] rad
measured q2 range:                [-0.990659, 0.762936] rad
simulator terminal:               FAIL (0.732075 m / 2.62883 rad)
```

Gate 63 restores physical contact progress and keeps both periodic joints away
from their winding limits. The canonical run ends only at its honest budget
defer, with an optimistic 34,472 additional updates required. Gate 64 is a
separately labeled 40,000-update fall-through to exercise the scoped guard and
Gate 60 bounded continuation after larger measured yaw transitions.

## Gate 64 — scoped-conformance extended fall-through

The separately labeled 40,000-update run reached twelve strict productive
cycles and 21,814 measured updates without periodic winding. It stopped before
the budget on a recovery candidate whose corridor transaction passed but whose
predicted terminal tolerance-normalized task descent was negative.

```text
source commit:                    676e69b0
config/settings/tolerances:       unchanged
physical output:                  /root/push_anything_ADMM/results/xarm6_scoped_terminal_40000_7XX5n7
requested / used updates:         40000 / 21814
measured productive cycles:       12
measured q1 range:                [-0.168219, 0.898037] rad
measured q2 range:                [-0.998504, 0.758852] rad
last predicted normalized descent:-0.535660
last measured normalized descent: -0.421531
simulator terminal:               FAIL (0.620622 m / 2.05269 rad)
```

Gate 64 passes the scoped-winding objective and exposes a candidate-admission
gap. Bounded corridor restoration may spend component debt, but Gate 55
already requires its total terminal normalized transaction to remain positive.
Gate 65 must apply that receipt before live IK and physical contact, not only
after recovery has already spent thousands of updates.

## Gate 65 — positive recovery component transaction

Lateral-recovery candidate admission and ranking now require both corridor
restoration and Gate 55's positive bounded component transaction. Ranking uses
terminal normalized magnitude, so lateral correction cannot hide a net
translation/yaw regression. Physical credit and terminal acceptance are
unchanged.

```text
source commit:                    98e1b284
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_positive_recovery_transaction_40000_owPwA1
recovery-filter activation:       pending (primary search stopped first)
measured productive cycles:       1
primary candidates / executable:  400 / 115
simulator terminal:               FAIL (0.781443 m / 3.03857 rad)
```

Gate 65 is implemented and test-covered, but this run did not reach its late
recovery branch. One failed measured response classified every geometrically
equivalent candidate as incompatible; the primary resample exhausted despite
115 live-IK-executable candidates. Gate 66 must require replicated response
evidence before either preference or quarantine changes an evidence class.

## Gate 66 — replicated response evidence classes

A single measured contact remains provisional evidence class 1. Two matching
observations are now required before a class becomes preferred (0) or
quarantined (2). Residual and gain diagnostics are still computed immediately;
only candidate precedence waits for replication.

```text
source commit:                    68659f8c
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_replicated_response_evidence_40000_723bPe
measured productive cycles:       3
replicated quarantine observed:   yes (2 matching observations)
updates used:                     8348
simulator terminal:               FAIL (0.761556 m / 2.59308 rad)
```

Gate 66 removes the one-observation exhaustion and activates replicated
quarantine. The next failure crossed the x corridor during contact, was safely
released, but did not retry because only contact-loss release currently enters
the fallback loop. Gate 67 must treat released crossing and wrong-polarity
responses as retryable candidate invalidations too.

## Gate 67 — unified released-response retry

Corridor crossing, measured wrong polarity, and contact loss now share a pure
retry receipt. A response can invalidate its candidate only after the pusher
is physically clear and the lateral reserve is still not restored; otherwise
execution remains fail-closed.

```text
source commit:                    364ce593
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_recovery_response_retry_40000_jOvR5s
response-retry activation:        pending (no crossing in this trace)
acquisition retry activations:    2
measured productive cycles:       5
updates used:                     9986
simulator terminal:               FAIL (0.719903 m / 2.31254 rad)
```

Gate 67 is implemented and unit-tested. The trace progressed through five
cycles, then a live-IK-approved recovery repeatedly failed its physical
descent solve; later candidates were either nonpositive transactions or not
live-executable. Gate 68 must allow the existing home-seeded nonlinear IK
fallback during descend/contact, retaining measured-error improvement and
whole-capsule clearance checks.

## Gate 68 — home-seeded descend/contact IK fallback

The existing home-seeded nonlinear posture fallback now applies to descend and
contact phases as well as verticalization. Any fallback result is still moved
to the measured periodic branch, velocity-step bounded, swept for whole-capsule
clearance, and required to improve measured tip/axis error. Rejection logs now
separate solver failure from each clearance component.

```text
source commit:                    52a0ce50
config/settings/tolerances:       unchanged
focused tests / live build:       PASS / PASS
physical output:                  /root/push_anything_ADMM/results/xarm6_descent_home_seed_40000_T2IS4w
home-seed activation:             pending
strict terminal transaction:      PASS (0.006735 m / 0.093843 rad)
lateral error / reserve:          0.003047 m / 0.002000 m
simulator terminal:               FAIL (0.791126 m / 3.03629 rad)
```

Gate 68 is implemented but activation-pending. This run instead exposed a
continuation gap: a released pose inside the unchanged 5 mm lateral corridor
and with strict positive task progress missed only the 2 mm productive-credit
reserve. Gate 69 must allow bounded replanning from that pose without granting
productive credit or weakening the reserve.
