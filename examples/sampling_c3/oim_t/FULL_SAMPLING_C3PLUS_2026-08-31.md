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
